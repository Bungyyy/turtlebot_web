/**
 * Map Viewer – map-centric rendering.
 * The occupancy grid auto-fits to fill the canvas.
 * Robot, laser, path overlays are positioned using the map's coordinate system.
 */

/* global RosBridge, ROSLIB */
/* exported MapViewer */

const MapViewer = (() => {
  let canvas, ctx;
  let mapData = null;
  let mapImage = null;
  let robotPose = null;       // { x, y, theta } in MAP frame
  let odomPose = null;        // { x, y, theta } in ODOM frame
  let hasAmcl = false;
  let tfMapToOdom = null;     // { x, y, theta }
  let goalPose = null;
  let homePose = { x: 0, y: 0 };

  const TRAIL_MAX = 600;
  let odomTrail = [];
  let lastTrailTime = 0;
  let laserPoints = [];
  let navPath = [];
  let costmapImage = null;
  let costmapData = null;
  let globalCostmapImage = null;
  let globalCostmapData = null;
  let nav2Status = { navActive: false, locActive: false, feedbackActive: false,
                     eta: 0, distance: 0, timeTaken: 0, recoveries: 0, goalStartTime: 0 };

  let layers = {
    map: true, costmap: true, laser: true, path: true,
    odom: true, tf: false, grid: true, robot: true,
  };

  let scale = 1.0;
  let panX = 0, panY = 0;
  let isDragging = false, dragStart = { x: 0, y: 0 }, dragMoved = false;
  let autoFitted = false;
  let interactionMode = null;
  let activeSubs = [];

  let mapOriginYaw = 0; // cached yaw of map origin orientation

  // ---- Coordinate conversion --------------------------------------------
  // World (meters) -> grid cell -> canvas pixels
  // Handles rotated map origins (common with Cartographer SLAM)

  function _worldToCanvas(wx, wy) {
    if (!mapData) return { cx: 0, cy: 0 };
    const info = mapData.info;
    const dx = wx - info.origin.position.x;
    const dy = wy - info.origin.position.y;
    // Rotate into map-local frame if origin has orientation
    const cosO = Math.cos(-mapOriginYaw);
    const sinO = Math.sin(-mapOriginYaw);
    const lx = cosO * dx - sinO * dy;
    const ly = sinO * dx + cosO * dy;
    const col = lx / info.resolution;
    const row = info.height - ly / info.resolution;
    return { cx: col * scale + panX, cy: row * scale + panY };
  }

  function _canvasToWorld(cx, cy) {
    if (!mapData) return { x: 0, y: 0 };
    const info = mapData.info;
    const col = (cx - panX) / scale;
    const row = (cy - panY) / scale;
    const lx = col * info.resolution;
    const ly = (info.height - row) * info.resolution;
    // Rotate back from map-local frame to world frame
    const cosO = Math.cos(mapOriginYaw);
    const sinO = Math.sin(mapOriginYaw);
    return {
      x: info.origin.position.x + cosO * lx - sinO * ly,
      y: info.origin.position.y + sinO * lx + cosO * ly,
    };
  }

  // ---- Init -------------------------------------------------------------

  function init() {
    canvas = document.getElementById("map-canvas");
    ctx = canvas.getContext("2d");
    _resizeCanvas();
    window.addEventListener("resize", () => { _resizeCanvas(); if (autoFitted) _autoFit(); });

    canvas.addEventListener("mousedown", _onMouseDown);
    canvas.addEventListener("mousemove", _onMouseMove);
    canvas.addEventListener("mouseup", _onMouseUp);
    canvas.addEventListener("wheel", _onWheel, { passive: false });
    canvas.addEventListener("click", _onClick);

    document.getElementById("btn-zoom-in").addEventListener("click", () => _zoomCenter(1.25));
    document.getElementById("btn-zoom-out").addEventListener("click", () => _zoomCenter(0.8));
    document.getElementById("btn-center-robot").addEventListener("click", _centerOnRobot);
    document.getElementById("btn-fit-map").addEventListener("click", _autoFit);

    canvas.addEventListener("mousemove", _onMouseMoveCoords);
    canvas.addEventListener("mouseleave", () => {
      const el = document.getElementById("map-cursor-pos");
      if (el) el.textContent = "X: -- Y: --";
    });

    document.querySelectorAll(".layer-toggle").forEach((cb) => {
      cb.addEventListener("change", (e) => {
        layers[e.target.dataset.layer] = e.target.checked;
        _render();
      });
    });
  }

  // ---- Subscriptions ----------------------------------------------------

  function subscribeTopics() {
    activeSubs.forEach((t) => { try { t.unsubscribe(); } catch (_) {} });
    activeSubs = [];
    console.log("[MapViewer] Subscribing to topics...");

    // OccupancyGrid
    activeSubs.push(RosBridge.subscribe("/map", "nav_msgs/msg/OccupancyGrid", (msg) => {
      console.log("[MapViewer] /map received:", msg.info.width + "x" + msg.info.height,
        "origin:", msg.info.origin.position.x.toFixed(3), msg.info.origin.position.y.toFixed(3),
        "orientation:", JSON.stringify(msg.info.origin.orientation));
      mapData = msg;
      mapOriginYaw = _qToYaw(msg.info.origin.orientation);
      console.log("[MapViewer] Map origin yaw:", (mapOriginYaw * 180 / Math.PI).toFixed(2) + "°");
      _buildMapImage(msg);
      if (!autoFitted) { autoFitted = true; _autoFit(); }
      _render();
      document.getElementById("map-resolution").textContent = "Res: " + msg.info.resolution.toFixed(3) + " m/px";
      document.getElementById("map-size").textContent = "Size: " + msg.info.width + "x" + msg.info.height;
    }, { throttle: 2000 }));

    // AMCL pose (MAP frame)
    let amclCount = 0;
    activeSubs.push(RosBridge.subscribe("/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", (msg) => {
      const p = msg.pose.pose;
      robotPose = { x: p.position.x, y: p.position.y, theta: _qToYaw(p.orientation) };
      hasAmcl = true;
      if (amclCount++ < 3) {
        console.log("[MapViewer] AMCL pose:", robotPose.x.toFixed(3), robotPose.y.toFixed(3),
          "theta:", (robotPose.theta * 180 / Math.PI).toFixed(1) + "°");
        if (mapData) {
          const { cx, cy } = _worldToCanvas(robotPose.x, robotPose.y);
          console.log("[MapViewer] Robot canvas pos:", cx.toFixed(1), cy.toFixed(1),
            "canvas:", canvas.width + "x" + canvas.height);
        }
      }
      nav2Status.locActive = true;
      _updateNav2UI();
      _pushTrail(robotPose);
      _updatePoseUI(p);
      _render();
    }, { throttle: 100 }));

    // Odom (ODOM frame)
    let odomCount = 0;
    activeSubs.push(RosBridge.subscribe("/odom", "nav_msgs/msg/Odometry", (msg) => {
      if (odomCount++ === 0) console.log("[MapViewer] First /odom received");
      const p = msg.pose.pose;
      odomPose = { x: p.position.x, y: p.position.y, theta: _qToYaw(p.orientation) };
      if (!hasAmcl) {
        robotPose = _odomToMap(odomPose);
        _updatePoseUI(p);
      }
      _pushTrail(robotPose);
      _render();
    }, { throttle: 100 }));

    // TF map->odom
    let tfCount = 0;
    const _processTF = (msg) => {
      for (const t of (msg.transforms || [])) {
        const parent = (t.header.frame_id || "").replace(/^\//, "");
        const child = (t.child_frame_id || "").replace(/^\//, "");
        if (parent === "map" && child === "odom") {
          const tr = t.transform;
          tfMapToOdom = {
            x: tr.translation.x, y: tr.translation.y,
            theta: _qToYaw(tr.rotation),
          };
          if (tfCount++ === 0) console.log("[MapViewer] First map->odom TF:", tfMapToOdom);
          if (!hasAmcl && odomPose) robotPose = _odomToMap(odomPose);
        }
      }
    };
    activeSubs.push(RosBridge.subscribe("/tf", "tf2_msgs/msg/TFMessage", _processTF, { throttle: 50 }));
    activeSubs.push(RosBridge.subscribe("/tf_static", "tf2_msgs/msg/TFMessage", _processTF, { throttle: 0 }));

    // LaserScan
    activeSubs.push(RosBridge.subscribe("/scan", "sensor_msgs/msg/LaserScan", (msg) => {
      _processLaserScan(msg);
      _render();
    }, { throttle: 150 }));

    // Nav path
    const _pathCb = (msg) => {
      navPath = (msg.poses || []).map((ps) => ({ x: ps.pose.position.x, y: ps.pose.position.y }));
      _render();
    };
    activeSubs.push(RosBridge.subscribe("/plan", "nav_msgs/msg/Path", _pathCb, { throttle: 500 }));
    activeSubs.push(RosBridge.subscribe("/received_global_plan", "nav_msgs/msg/Path", _pathCb, { throttle: 500 }));

    // Global costmap
    activeSubs.push(RosBridge.subscribe("/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid", (msg) => {
      globalCostmapData = msg;
      globalCostmapImage = _buildCostmapImage(msg);
      _render();
    }, { throttle: 3000 }));

    // Local costmap (also fallback for robot position)
    activeSubs.push(RosBridge.subscribe("/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid", (msg) => {
      costmapData = msg;
      costmapImage = _buildCostmapImage(msg);
      if (!tfMapToOdom && !hasAmcl && msg.info) {
        const cx = msg.info.origin.position.x + (msg.info.width * msg.info.resolution) / 2;
        const cy = msg.info.origin.position.y + (msg.info.height * msg.info.resolution) / 2;
        robotPose = { x: cx, y: cy, theta: robotPose ? robotPose.theta : 0 };
      }
      _render();
    }, { throttle: 1000 }));

    // Nav2 action status
    activeSubs.push(RosBridge.subscribe("/navigate_to_pose/_action/status", "action_msgs/msg/GoalStatusArray", (msg) => {
      const list = msg.status_list || [];
      if (list.length > 0) {
        const last = list[list.length - 1];
        // Status: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        const active = last.status === 1 || last.status === 2;
        nav2Status.navActive = active;
        _updateNav2UI();
        if (last.status === 4) {
          goalPose = null; navPath = []; _render();
          nav2Status.navActive = false;
          _updateNav2UI();
          _setStatus("Navigation goal reached");
        } else if (last.status === 6) {
          nav2Status.navActive = false;
          _updateNav2UI();
          _setStatus("Navigation aborted");
        }
      }
    }, { throttle: 500 }));

    // Nav2 feedback (ETA, distance, recoveries)
    activeSubs.push(RosBridge.subscribe("/navigate_to_pose/_action/feedback", "nav2_msgs/action/NavigateToPose_FeedbackMessage", (msg) => {
      const fb = msg.feedback || msg;
      nav2Status.feedbackActive = true;
      if (fb.estimated_time_remaining) {
        nav2Status.eta = (fb.estimated_time_remaining.sec || 0) + (fb.estimated_time_remaining.nanosec || 0) / 1e9;
      }
      if (fb.distance_remaining !== undefined) nav2Status.distance = fb.distance_remaining;
      if (fb.number_of_recoveries !== undefined) nav2Status.recoveries = fb.number_of_recoveries;
      if (nav2Status.goalStartTime > 0) {
        nav2Status.timeTaken = (Date.now() - nav2Status.goalStartTime) / 1000;
      }
      _updateNav2UI();
    }, { throttle: 500 }));

    // AMCL active = localization active
    activeSubs.push(RosBridge.subscribe("/particle_cloud", "nav2_msgs/msg/ParticleCloud", () => {
      nav2Status.locActive = true;
      _updateNav2UI();
    }, { throttle: 2000 }));
  }

  // ---- Odom trail -------------------------------------------------------

  function _pushTrail(pose) {
    if (!pose) return;
    const now = Date.now();
    if (now - lastTrailTime < 80) return;
    lastTrailTime = now;
    odomTrail.push({ x: pose.x, y: pose.y, t: now });
    if (odomTrail.length > TRAIL_MAX) odomTrail.shift();
  }

  // ---- Laser scan -------------------------------------------------------

  function _processLaserScan(msg) {
    const pose = robotPose || (odomPose && _odomToMap(odomPose));
    if (!pose) return;
    const pts = [];
    const { angle_min, angle_increment, ranges, range_min, range_max } = msg;
    const cosR = Math.cos(pose.theta);
    const sinR = Math.sin(pose.theta);
    for (let i = 0; i < ranges.length; i += 3) {
      const r = ranges[i];
      if (r < range_min || r > range_max || !isFinite(r)) continue;
      const a = angle_min + i * angle_increment;
      const lx = r * Math.cos(a), ly = r * Math.sin(a);
      pts.push({ x: pose.x + cosR*lx - sinR*ly, y: pose.y + sinR*lx + cosR*ly });
    }
    laserPoints = pts;
  }

  // ---- Map image build --------------------------------------------------

  function _buildMapImage(msg) {
    const w = msg.info.width, h = msg.info.height;
    const data = msg.data;
    const imgData = new ImageData(w, h);

    // Log data encoding once for debugging
    if (data.length > 0) {
      const sample = [];
      for (let j = 0; j < Math.min(20, data.length); j++) sample.push(data[j]);
      console.log("[MapViewer] Map data sample (first 20):", sample,
        "type:", typeof data[0], "isUint8Array:", data instanceof Uint8Array);
    }

    for (let i = 0; i < data.length; i++) {
      const v = data[i], idx = i * 4;
      // Handle both signed int8 (-1=unknown) and unsigned uint8 (255=unknown)
      // Valid occupancy range: 0-100. Everything else = unknown.
      if (v >= 0 && v <= 50) {
        // Free space: white
        imgData.data[idx] = 255; imgData.data[idx+1] = 255; imgData.data[idx+2] = 255;
      } else if (v > 50 && v <= 100) {
        // Occupied: black
        imgData.data[idx] = 0; imgData.data[idx+1] = 0; imgData.data[idx+2] = 0;
      } else {
        // Unknown (-1 as int8, 255 as uint8, or any out-of-range): gray
        imgData.data[idx] = 184; imgData.data[idx+1] = 184; imgData.data[idx+2] = 184;
      }
      imgData.data[idx+3] = 255;
    }

    // OccupancyGrid is row-major bottom-to-top; canvas is top-to-bottom -> flip
    const tmp = document.createElement("canvas");
    tmp.width = w; tmp.height = h;
    tmp.getContext("2d").putImageData(imgData, 0, 0);

    const off = document.createElement("canvas");
    off.width = w; off.height = h;
    const offCtx = off.getContext("2d");
    offCtx.translate(0, h);
    offCtx.scale(1, -1);
    offCtx.drawImage(tmp, 0, 0);
    mapImage = off;
  }

  // ---- Costmap image build ----------------------------------------------

  // RViz-style costmap: lethal=magenta, inscribed=purple, gradient=blue->cyan
  function _buildCostmapImage(msg) {
    const w = msg.info.width, h = msg.info.height;
    const data = msg.data;
    const imgData = new ImageData(w, h);

    for (let i = 0; i < data.length; i++) {
      const v = data[i], idx = i * 4;
      // Skip unknown/free: -1 (signed), 255 (unsigned), 0
      if (v <= 0 || v === -1 || v === 255 || v > 100) { imgData.data[idx+3] = 0; continue; }
      // Lethal (100) - bright magenta/pink like RViz
      if (v === 100) { imgData.data[idx]=255; imgData.data[idx+1]=0; imgData.data[idx+2]=200; imgData.data[idx+3]=200; continue; }
      // Inscribed (99) - purple
      if (v >= 99) { imgData.data[idx]=180; imgData.data[idx+1]=0; imgData.data[idx+2]=255; imgData.data[idx+3]=180; continue; }
      // Gradient: low=deep blue, mid=cyan, high=purple (matching RViz costmap2D)
      const t = v / 98;
      if (t < 0.33) {
        const s = t / 0.33;
        imgData.data[idx] = Math.round(s * 50);
        imgData.data[idx+1] = Math.round(s * 150);
        imgData.data[idx+2] = Math.round(150 + s * 105);
      } else if (t < 0.66) {
        const s = (t - 0.33) / 0.33;
        imgData.data[idx] = Math.round(50 + s * 100);
        imgData.data[idx+1] = Math.round(150 + s * 50);
        imgData.data[idx+2] = 255;
      } else {
        const s = (t - 0.66) / 0.34;
        imgData.data[idx] = Math.round(150 + s * 105);
        imgData.data[idx+1] = Math.round(200 * (1 - s * 0.6));
        imgData.data[idx+2] = Math.round(255 * (1 - s * 0.2));
      }
      imgData.data[idx+3] = Math.round(60 + t * 140);
    }

    const tmp = document.createElement("canvas");
    tmp.width = w; tmp.height = h;
    tmp.getContext("2d").putImageData(imgData, 0, 0);
    const off = document.createElement("canvas");
    off.width = w; off.height = h;
    const offCtx = off.getContext("2d");
    offCtx.translate(0, h);
    offCtx.scale(1, -1);
    offCtx.drawImage(tmp, 0, 0);
    return off;
  }

  // ---- Costmap overlay helper -------------------------------------------

  function _drawCostmapOverlay(img, cData, alpha) {
    const mi = mapData.info, ci = cData.info;
    const col = (ci.origin.position.x - mi.origin.position.x) / mi.resolution;
    const row = mi.height - (ci.origin.position.y - mi.origin.position.y) / mi.resolution
                - ci.height * (ci.resolution / mi.resolution);
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = alpha;
    ctx.drawImage(img,
      panX + col * scale, panY + row * scale,
      ci.width * (ci.resolution / mi.resolution) * scale,
      ci.height * (ci.resolution / mi.resolution) * scale);
    ctx.globalAlpha = 1;
    ctx.restore();
  }

  // ---- Nav2 UI update ---------------------------------------------------

  function _updateNav2UI() {
    const s = nav2Status;
    const setEl = (id, text, cls) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.textContent = text;
      if (cls !== undefined) { el.className = "nav2-value" + (cls ? " " + cls : ""); }
    };
    setEl("nav2-nav-status", s.navActive ? "active" : "inactive", s.navActive ? "active" : "");
    setEl("nav2-loc-status", s.locActive ? "active" : "inactive", s.locActive ? "active" : "");
    setEl("nav2-feedback-status", s.feedbackActive ? "active" : "inactive", s.feedbackActive ? "active" : "");
    setEl("nav2-eta", s.eta > 0 ? Math.round(s.eta) + " s" : "--", "mono");
    setEl("nav2-distance", s.distance > 0 ? s.distance.toFixed(2) + " m" : "--", "mono");
    setEl("nav2-time", s.timeTaken > 0 ? Math.round(s.timeTaken) + " s" : "--", "mono");
    setEl("nav2-recoveries", String(s.recoveries), "mono");
  }

  // ---- Rendering --------------------------------------------------------

  function _render() {
    if (!canvas) return;
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    // Light gray background like KraiPlatform reference
    ctx.fillStyle = "#b8b8b8";
    ctx.fillRect(0, 0, W, H);

    // 1. Occupancy grid
    if (mapImage && mapData && layers.map) {
      ctx.save();
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapImage, panX, panY, mapData.info.width * scale, mapData.info.height * scale);
      ctx.imageSmoothingEnabled = true;
      ctx.restore();
    }

    // 2. Global costmap overlay
    if (globalCostmapImage && globalCostmapData && mapData && layers.costmap) {
      _drawCostmapOverlay(globalCostmapImage, globalCostmapData, 0.5);
    }

    // 2b. Local costmap overlay (on top of global)
    if (costmapImage && costmapData && mapData && layers.costmap) {
      _drawCostmapOverlay(costmapImage, costmapData, 0.7);
    }

    // 3. Grid
    if (layers.grid && mapData) _drawGrid();

    // 4. Odom trail
    if (layers.odom && odomTrail.length > 1) _drawOdomTrail();

    // 5. Nav path
    if (layers.path && navPath.length > 1) _drawNavPath();

    // 6. Laser scan
    if (layers.laser && laserPoints.length > 0) _drawLaserScan();

    // 7. Home marker
    if (mapData) _drawMarker(homePose.x, homePose.y, "#e53e3e", "H");

    // 8. Goal marker
    if (goalPose && mapData) _drawMarker(goalPose.x, goalPose.y, "#3182ce", "G");

    // 9. Robot (green triangle)
    if (robotPose && mapData && layers.robot) _drawRobot();

    // 10. TF frames
    if (robotPose && mapData && layers.tf) _drawTFAxes(robotPose, "base_link");
    if (mapData && layers.tf) _drawTFAxes({ x:0, y:0, theta:0 }, "map");

    // 11. Debug: show robot world coords on canvas
    if (robotPose && mapData) {
      const src = hasAmcl ? "AMCL" : (tfMapToOdom ? "odom+TF" : "odom");
      const dbg = src + " (" + robotPose.x.toFixed(2) + ", " + robotPose.y.toFixed(2) + ") " +
                  (robotPose.theta * 180 / Math.PI).toFixed(0) + "\u00b0";
      ctx.save();
      ctx.font = "bold 11px monospace";
      ctx.fillStyle = "rgba(0,0,0,0.6)";
      ctx.fillText(dbg, 10, 20);
      ctx.restore();
    }

    // 12. Warning
    if (!tfMapToOdom && !hasAmcl && odomPose && mapData) {
      ctx.fillStyle = "rgba(180,80,0,0.9)";
      ctx.font = "bold 11px monospace";
      ctx.fillText("No map\u2192odom TF \u2014 robot position approximate", 10, H - 30);
    }
  }

  // ---- Grid (subtle meter lines) ----------------------------------------

  function _drawGrid() {
    const res = mapData.info.resolution;
    const pxPerMeter = scale / res;
    if (pxPerMeter < 15) return; // too zoomed out

    ctx.strokeStyle = "rgba(0,0,0,0.12)";
    ctx.lineWidth = 0.5;
    const info = mapData.info;
    const mapWpx = info.width * scale, mapHpx = info.height * scale;

    const startX = info.origin.position.x;
    for (let wx = Math.ceil(startX); wx < startX + info.width * res; wx++) {
      const { cx } = _worldToCanvas(wx, 0);
      ctx.beginPath(); ctx.moveTo(cx, panY); ctx.lineTo(cx, panY + mapHpx); ctx.stroke();
    }
    const startY = info.origin.position.y;
    for (let wy = Math.ceil(startY); wy < startY + info.height * res; wy++) {
      const { cy } = _worldToCanvas(0, wy);
      ctx.beginPath(); ctx.moveTo(panX, cy); ctx.lineTo(panX + mapWpx, cy); ctx.stroke();
    }
  }

  // ---- Robot (green triangle like reference) ----------------------------

  function _drawRobot() {
    const { cx, cy } = _worldToCanvas(robotPose.x, robotPose.y);
    const sz = 14;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-robotPose.theta);

    ctx.beginPath();
    ctx.moveTo(sz, 0);
    ctx.lineTo(-sz * 0.6, -sz * 0.55);
    ctx.lineTo(-sz * 0.6,  sz * 0.55);
    ctx.closePath();
    ctx.fillStyle = "#22c55e";
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fillStyle = "#fff";
    ctx.fill();

    ctx.restore();
  }

  // ---- TF axes ----------------------------------------------------------

  function _drawTFAxes(pose, label) {
    const { cx, cy } = _worldToCanvas(pose.x, pose.y);
    const len = 25;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-pose.theta);

    ctx.strokeStyle = "#ff3333"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(len, 0); ctx.stroke();
    ctx.strokeStyle = "#33ff33";
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0, -len); ctx.stroke();
    ctx.restore();

    if (label) {
      ctx.font = "10px monospace";
      ctx.fillStyle = "rgba(0,0,0,0.7)";
      ctx.fillText(label, cx + 8, cy - 8);
    }
  }

  // ---- Odom trail -------------------------------------------------------

  function _drawOdomTrail() {
    const now = Date.now();
    for (let i = 1; i < odomTrail.length; i++) {
      const p0 = _worldToCanvas(odomTrail[i-1].x, odomTrail[i-1].y);
      const p1 = _worldToCanvas(odomTrail[i].x, odomTrail[i].y);
      const alpha = Math.max(0.05, 1 - (now - odomTrail[i].t) / 30000);
      ctx.strokeStyle = "rgba(255,170,0," + (alpha * 0.7).toFixed(3) + ")";
      ctx.lineWidth = 2 * alpha;
      ctx.beginPath(); ctx.moveTo(p0.cx, p0.cy); ctx.lineTo(p1.cx, p1.cy); ctx.stroke();
    }
  }

  // ---- Nav path ---------------------------------------------------------

  function _drawNavPath() {
    // Solid green line like RViz Global Planner path
    ctx.strokeStyle = "#00e060";
    ctx.lineWidth = 3;
    const f = _worldToCanvas(navPath[0].x, navPath[0].y);
    ctx.beginPath(); ctx.moveTo(f.cx, f.cy);
    for (let i = 1; i < navPath.length; i++) {
      const p = _worldToCanvas(navPath[i].x, navPath[i].y);
      ctx.lineTo(p.cx, p.cy);
    }
    ctx.stroke();
  }

  // ---- Laser scan -------------------------------------------------------

  function _drawLaserScan() {
    // Bright cyan like RViz LaserScan display
    ctx.fillStyle = "#00ffff";
    for (const lp of laserPoints) {
      const p = _worldToCanvas(lp.x, lp.y);
      ctx.beginPath(); ctx.arc(p.cx, p.cy, 2.5, 0, Math.PI * 2); ctx.fill();
    }
  }

  // ---- Markers ----------------------------------------------------------

  function _drawMarker(wx, wy, color, label) {
    const { cx, cy } = _worldToCanvas(wx, wy);
    ctx.beginPath(); ctx.arc(cx, cy, 8, 0, Math.PI*2);
    ctx.fillStyle = color; ctx.fill();
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
    if (label) {
      ctx.fillStyle = "#fff"; ctx.font = "bold 10px sans-serif";
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(label, cx, cy);
    }
  }

  // ---- Interaction ------------------------------------------------------

  function setMode(mode) {
    interactionMode = mode;
    dragMoved = false;
    canvas.style.cursor = mode ? "crosshair" : "grab";
  }

  function _onClick(e) {
    if (dragMoved) { dragMoved = false; return; }
    if (!interactionMode || !mapData) return;
    const rect = canvas.getBoundingClientRect();
    const world = _canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
    if (interactionMode === "navigate") _sendNavGoal(world.x, world.y);
    else if (interactionMode === "initial_pose") _sendInitialPose(world.x, world.y);
    interactionMode = null;
    canvas.style.cursor = "grab";
  }

  function _sendNavGoal(x, y, oz, ow) {
    goalPose = { x, y };
    nav2Status.goalStartTime = Date.now();
    nav2Status.navActive = true;
    nav2Status.feedbackActive = false;
    nav2Status.eta = 0;
    nav2Status.distance = 0;
    nav2Status.timeTaken = 0;
    nav2Status.recoveries = 0;
    _updateNav2UI();
    _render();
    const poseMsg = {
      header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
      pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: oz||0, w: ow||1 } },
    };
    RosBridge.publish("/goal_pose", "geometry_msgs/msg/PoseStamped", poseMsg);
    try {
      const ac = RosBridge.actionClient("/navigate_to_pose", "nav2_msgs/action/NavigateToPose");
      new ROSLIB.Goal({ actionClient: ac, goalMessage: { pose: poseMsg } }).send();
    } catch (_) {}
    _setStatus("Nav goal: (" + x.toFixed(2) + ", " + y.toFixed(2) + ")");
  }

  function _sendInitialPose(x, y) {
    RosBridge.publish("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", {
      header: { frame_id: "map" },
      pose: { pose: { position: { x, y, z: 0 }, orientation: { x:0, y:0, z:0, w:1 } }, covariance: new Array(36).fill(0) },
    });
    _setStatus("Initial pose set: (" + x.toFixed(2) + ", " + y.toFixed(2) + ")");
  }

  function navigateTo(x, y, oz, ow) { _sendNavGoal(x, y, oz, ow); }

  // ---- Pan & Zoom -------------------------------------------------------

  function _onMouseDown(e) {
    if (interactionMode) return;
    isDragging = true; dragMoved = false;
    dragStart = { x: e.clientX - panX, y: e.clientY - panY };
    canvas.style.cursor = "grabbing";
  }

  function _onMouseMove(e) {
    if (!isDragging) return;
    dragMoved = true;
    panX = e.clientX - dragStart.x;
    panY = e.clientY - dragStart.y;
    _render();
  }

  function _onMouseUp() {
    isDragging = false;
    if (!interactionMode) canvas.style.cursor = "grab";
  }

  function _onWheel(e) {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.1 : 0.9;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left, my = e.clientY - rect.top;
    const ns = Math.max(0.1, Math.min(scale * factor, 30));
    panX = mx - (mx - panX) * (ns / scale);
    panY = my - (my - panY) * (ns / scale);
    scale = ns;
    _render();
  }

  function _zoomCenter(factor) {
    const mx = canvas.width / 2, my = canvas.height / 2;
    const ns = Math.max(0.1, Math.min(scale * factor, 30));
    panX = mx - (mx - panX) * (ns / scale);
    panY = my - (my - panY) * (ns / scale);
    scale = ns;
    _render();
  }

  // ---- Helpers ----------------------------------------------------------

  function _resizeCanvas() {
    if (!canvas) return;
    const c = canvas.parentElement;
    canvas.width = c.clientWidth;
    canvas.height = c.clientHeight;
    _render();
  }

  function _autoFit() {
    if (!mapData || !canvas) return;
    const W = canvas.width, H = canvas.height;
    const mw = mapData.info.width, mh = mapData.info.height;
    const pad = 20;
    scale = Math.min((W - pad*2) / mw, (H - pad*2) / mh);
    scale = Math.max(0.1, Math.min(scale, 30));
    panX = (W - mw * scale) / 2;
    panY = (H - mh * scale) / 2;
    _render();
  }

  function _centerOnRobot() {
    if (!robotPose || !mapData || !canvas) return;
    // Use the same coordinate conversion as _worldToCanvas
    const { cx, cy } = _worldToCanvas(robotPose.x, robotPose.y);
    panX += canvas.width / 2 - cx;
    panY += canvas.height / 2 - cy;
    _render();
  }

  function _onMouseMoveCoords(e) {
    if (!mapData) return;
    const rect = canvas.getBoundingClientRect();
    const w = _canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
    const el = document.getElementById("map-cursor-pos");
    if (el) el.textContent = "X: " + w.x.toFixed(2) + " Y: " + w.y.toFixed(2);
  }

  function _qToYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  function _odomToMap(pose) {
    if (!tfMapToOdom) return pose;
    const c = Math.cos(tfMapToOdom.theta), s = Math.sin(tfMapToOdom.theta);
    return {
      x: tfMapToOdom.x + c * pose.x - s * pose.y,
      y: tfMapToOdom.y + s * pose.x + c * pose.y,
      theta: pose.theta + tfMapToOdom.theta,
    };
  }

  function _updatePoseUI(pose) {
    const ids = ["pos-x","pos-y","pos-z","ori-x","ori-y","ori-z","ori-w"];
    const vals = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w];
    ids.forEach((id, i) => { const el = document.getElementById(id); if (el) el.textContent = vals[i].toFixed(3); });
  }

  function _setStatus(msg) { document.getElementById("status-message").textContent = msg; }
  function getRobotPose() { return robotPose; }

  return { init, subscribeTopics, setMode, navigateTo, getRobotPose, centerOnRobot: _centerOnRobot, fitMap: _autoFit };
})();
