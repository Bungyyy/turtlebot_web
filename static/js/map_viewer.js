/**
 * Map Viewer – RViz-style visualisation
 * Renders OccupancyGrid, robot model, TF frames, odom trail,
 * laser scan, navigation path, costmap overlay, and goal markers.
 */

/* global RosBridge, ROSLIB */
/* exported MapViewer */

const MapViewer = (() => {
  let canvas, ctx;
  let mapData = null;
  let mapImage = null;        // offscreen canvas for OccupancyGrid
  let robotPose = null;       // { x, y, theta } — best known pose in MAP frame
  let odomPose = null;        // { x, y, theta } — pose in ODOM frame (fallback)
  let hasAmcl = false;        // true once AMCL has published
  let goalPose = null;        // { x, y }
  let homePose = { x: 0, y: 0 };

  // Odom trail (ring buffer)
  const TRAIL_MAX = 600;
  let odomTrail = [];
  let lastTrailTime = 0;

  // Laser scan points
  let laserPoints = [];       // [{ x, y }, ...]

  // Navigation path
  let navPath = [];           // [{ x, y }, ...]

  // Local costmap
  let costmapImage = null;
  let costmapData = null;

  // Layer visibility (toggled from UI)
  let layers = {
    map: true,
    costmap: true,
    laser: true,
    path: true,
    odom: true,
    tf: true,
    grid: true,
    robot: true,
  };

  let scale = 1.0;
  let panX = 0, panY = 0;
  let isDragging = false, dragStart = { x: 0, y: 0 }, dragMoved = false;

  let interactionMode = null;

  // ---- Initialisation ---------------------------------------------------

  function init() {
    canvas = document.getElementById("map-canvas");
    ctx = canvas.getContext("2d");
    _resizeCanvas();
    window.addEventListener("resize", _resizeCanvas);

    canvas.addEventListener("mousedown", _onMouseDown);
    canvas.addEventListener("mousemove", _onMouseMove);
    canvas.addEventListener("mouseup", _onMouseUp);
    canvas.addEventListener("wheel", _onWheel, { passive: false });
    canvas.addEventListener("click", _onClick);

    document.getElementById("btn-zoom-in").addEventListener("click", () => { scale *= 1.25; _render(); });
    document.getElementById("btn-zoom-out").addEventListener("click", () => { scale *= 0.8; _render(); });

    // Layer toggle checkboxes
    document.querySelectorAll(".layer-toggle").forEach((cb) => {
      cb.addEventListener("change", (e) => {
        layers[e.target.dataset.layer] = e.target.checked;
        _render();
      });
    });
  }

  function subscribeTopics() {
    // OccupancyGrid
    RosBridge.subscribe("/map", "nav_msgs/msg/OccupancyGrid", (msg) => {
      mapData = msg;
      _buildMapImage(msg);
      _render();
      document.getElementById("map-resolution").textContent = `Res: ${msg.info.resolution.toFixed(3)} m/px`;
      document.getElementById("map-size").textContent = `Size: ${msg.info.width}x${msg.info.height}`;
    }, { throttle: 2000 });

    // Robot pose (AMCL — in MAP frame, authoritative for position on map)
    RosBridge.subscribe("/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", (msg) => {
      const p = msg.pose.pose;
      const yaw = _quaternionToYaw(p.orientation);
      robotPose = { x: p.position.x, y: p.position.y, theta: yaw };
      hasAmcl = true;
      _pushTrail(robotPose);
      _updatePoseUI(p);
      _render();
    }, { throttle: 100 });

    // Odom — in ODOM frame; use as fallback when AMCL is not available (e.g. SLAM mode)
    RosBridge.subscribe("/odom", "nav_msgs/msg/Odometry", (msg) => {
      const p = msg.pose.pose;
      const yaw = _quaternionToYaw(p.orientation);
      odomPose = { x: p.position.x, y: p.position.y, theta: yaw };
      if (!hasAmcl) {
        robotPose = odomPose;
        _updatePoseUI(p);
      }
      _pushTrail(robotPose || odomPose);
      _render();
    }, { throttle: 100 });

    // LaserScan
    RosBridge.subscribe("/scan", "sensor_msgs/msg/LaserScan", (msg) => {
      _processLaserScan(msg);
      _render();
    }, { throttle: 150 });

    // Navigation plan path
    RosBridge.subscribe("/plan", "nav_msgs/msg/Path", (msg) => {
      navPath = (msg.poses || []).map((ps) => ({
        x: ps.pose.position.x,
        y: ps.pose.position.y,
      }));
      _render();
    }, { throttle: 500 });

    // Also try /received_global_plan (Nav2 controller)
    RosBridge.subscribe("/received_global_plan", "nav_msgs/msg/Path", (msg) => {
      navPath = (msg.poses || []).map((ps) => ({
        x: ps.pose.position.x,
        y: ps.pose.position.y,
      }));
      _render();
    }, { throttle: 500 });

    // Local costmap
    RosBridge.subscribe(
      "/local_costmap/costmap",
      "nav_msgs/msg/OccupancyGrid",
      (msg) => {
        costmapData = msg;
        _buildCostmapImage(msg);
        _render();
      },
      { throttle: 1000 }
    );

    // Nav2 status – clear goal on success
    RosBridge.subscribe(
      "/navigate_to_pose/_action/status",
      "action_msgs/msg/GoalStatusArray",
      (msg) => {
        const statuses = msg.status_list || [];
        if (statuses.length > 0 && statuses[statuses.length - 1].status === 4) {
          goalPose = null;
          navPath = [];
          _render();
          _setStatus("Navigation goal reached");
        }
      },
      { throttle: 1000 }
    );
  }

  // ---- Odom trail -------------------------------------------------------

  function _pushTrail(pose) {
    const now = Date.now();
    if (now - lastTrailTime < 80) return; // ~12 Hz sampling
    lastTrailTime = now;
    odomTrail.push({ x: pose.x, y: pose.y, t: now });
    if (odomTrail.length > TRAIL_MAX) odomTrail.shift();
  }

  // ---- Laser scan processing --------------------------------------------

  function _processLaserScan(msg) {
    const pose = robotPose || odomPose;
    if (!pose) return;
    const pts = [];
    const { angle_min, angle_increment, ranges, range_min, range_max } = msg;
    const cosR = Math.cos(pose.theta);
    const sinR = Math.sin(pose.theta);

    // Downsample for performance (every 3rd ray)
    for (let i = 0; i < ranges.length; i += 3) {
      const r = ranges[i];
      if (r < range_min || r > range_max || !isFinite(r)) continue;
      const angle = angle_min + i * angle_increment;
      const lx = r * Math.cos(angle);
      const ly = r * Math.sin(angle);
      // Transform to map frame
      pts.push({
        x: pose.x + cosR * lx - sinR * ly,
        y: pose.y + sinR * lx + cosR * ly,
      });
    }
    laserPoints = pts;
  }

  // ---- Map building -----------------------------------------------------

  function _buildMapImage(msg) {
    const w = msg.info.width;
    const h = msg.info.height;
    const data = msg.data;
    const imgData = new ImageData(w, h);

    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      const idx = i * 4;
      if (val === -1) {
        // Unknown – RViz default gray
        imgData.data[idx]     = 205;
        imgData.data[idx + 1] = 205;
        imgData.data[idx + 2] = 205;
      } else if (val <= 50) {
        // Free / low probability – white (RViz uses threshold ~65)
        imgData.data[idx]     = 254;
        imgData.data[idx + 1] = 254;
        imgData.data[idx + 2] = 254;
      } else {
        // Occupied – solid black
        imgData.data[idx]     = 0;
        imgData.data[idx + 1] = 0;
        imgData.data[idx + 2] = 0;
      }
      imgData.data[idx + 3] = 255;
    }

    const offscreen = document.createElement("canvas");
    offscreen.width = w;
    offscreen.height = h;
    offscreen.getContext("2d").putImageData(imgData, 0, 0);
    mapImage = offscreen;
  }

  // ---- Costmap building -------------------------------------------------

  function _buildCostmapImage(msg) {
    const w = msg.info.width;
    const h = msg.info.height;
    const data = msg.data;
    const imgData = new ImageData(w, h);

    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      const idx = i * 4;
      if (val <= 0 || val === -1) {
        // Free / unknown – transparent
        imgData.data[idx + 3] = 0;
      } else if (val >= 100) {
        // Lethal – solid magenta/red
        imgData.data[idx]     = 200;
        imgData.data[idx + 1] = 0;
        imgData.data[idx + 2] = 120;
        imgData.data[idx + 3] = 180;
      } else if (val >= 99) {
        // Inscribed
        imgData.data[idx]     = 180;
        imgData.data[idx + 1] = 0;
        imgData.data[idx + 2] = 255;
        imgData.data[idx + 3] = 160;
      } else {
        // Cost gradient: blue (low) -> yellow (mid) -> red (high)
        const t = val / 98;
        if (t < 0.5) {
          const s = t * 2;
          imgData.data[idx]     = Math.round(s * 255);
          imgData.data[idx + 1] = Math.round(s * 200);
          imgData.data[idx + 2] = Math.round((1 - s) * 200);
        } else {
          const s = (t - 0.5) * 2;
          imgData.data[idx]     = 255;
          imgData.data[idx + 1] = Math.round((1 - s) * 200);
          imgData.data[idx + 2] = 0;
        }
        imgData.data[idx + 3] = Math.round(40 + t * 100);
      }
    }

    const offscreen = document.createElement("canvas");
    offscreen.width = w;
    offscreen.height = h;
    offscreen.getContext("2d").putImageData(imgData, 0, 0);
    costmapImage = offscreen;
  }

  // ---- Rendering --------------------------------------------------------

  function _render() {
    if (!canvas) return;
    const W = canvas.width;
    const H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    // RViz-style dark background
    ctx.fillStyle = "#303030";
    ctx.fillRect(0, 0, W, H);

    ctx.save();
    ctx.translate(W / 2 + panX, H / 2 + panY);
    ctx.scale(scale, scale);

    const res = mapData ? mapData.info.resolution : 0.05;

    // 1. Grid + axes
    if (layers.grid) _drawWorldGrid(res);

    // 2. Map
    if (mapImage && mapData && layers.map) {
      const ox = mapData.info.origin.position.x;
      const oy = mapData.info.origin.position.y;
      const w = mapData.info.width;
      const h = mapData.info.height;
      ctx.save();
      ctx.imageSmoothingEnabled = false;
      ctx.scale(1, -1);
      ctx.drawImage(mapImage, ox / res, -(oy / res) - h, w, h);
      ctx.imageSmoothingEnabled = true;
      ctx.restore();
    }

    // 3. Costmap overlay
    if (costmapImage && costmapData && layers.costmap) {
      const ox = costmapData.info.origin.position.x;
      const oy = costmapData.info.origin.position.y;
      const cres = costmapData.info.resolution;
      const w = costmapData.info.width;
      const h = costmapData.info.height;
      ctx.save();
      ctx.imageSmoothingEnabled = false;
      ctx.scale(1, -1);
      ctx.drawImage(costmapImage, ox / cres, -(oy / cres) - h, w, h);
      ctx.imageSmoothingEnabled = true;
      ctx.restore();
    }

    // 4. Odom trail
    if (layers.odom && odomTrail.length > 1) _drawOdomTrail(res);

    // 5. Navigation path
    if (layers.path && navPath.length > 1) _drawNavPath(res);

    // 6. Laser scan
    if (layers.laser && laserPoints.length > 0) _drawLaserScan(res);

    // 7. Home marker
    _drawMarker(homePose.x, homePose.y, "#3b82f6", 6, "H", res);

    // 8. Goal marker
    if (goalPose) _drawMarker(goalPose.x, goalPose.y, "#ef4444", 7, "G", res);

    // 9. Robot model + TF
    if (robotPose && mapData) {
      if (layers.tf) _drawTFAxes(robotPose, res, "base_link");
      if (layers.robot) _drawTurtleBot(robotPose, res);
    }

    // 10. Map origin TF
    if (layers.tf && mapData) _drawTFAxes({ x: 0, y: 0, theta: 0 }, res, "map");

    ctx.restore();
  }

  // ---- World grid with axes ---------------------------------------------

  function _drawWorldGrid(res) {
    const meterPx = 1.0 / res;
    const viewR = (Math.max(canvas.width, canvas.height) / scale) * 0.6;
    const gridStep = meterPx; // 1 meter

    // Sub-grid (0.5m) when zoomed in
    if (meterPx * scale > 30) {
      ctx.strokeStyle = "rgba(255,255,255,0.03)";
      ctx.lineWidth = 0.3 / scale;
      const half = meterPx * 0.5;
      const start = -Math.ceil(viewR / half) * half;
      const end = Math.ceil(viewR / half) * half;
      ctx.beginPath();
      for (let x = start; x <= end; x += half) {
        ctx.moveTo(x, -viewR);
        ctx.lineTo(x, viewR);
      }
      for (let y = start; y <= end; y += half) {
        ctx.moveTo(-viewR, y);
        ctx.lineTo(viewR, y);
      }
      ctx.stroke();
    }

    // Main grid (1m)
    ctx.strokeStyle = "rgba(255,255,255,0.07)";
    ctx.lineWidth = 0.5 / scale;
    const start = -Math.ceil(viewR / gridStep) * gridStep;
    const end = Math.ceil(viewR / gridStep) * gridStep;
    ctx.beginPath();
    for (let x = start; x <= end; x += gridStep) {
      ctx.moveTo(x, -viewR);
      ctx.lineTo(x, viewR);
    }
    for (let y = start; y <= end; y += gridStep) {
      ctx.moveTo(-viewR, y);
      ctx.lineTo(viewR, y);
    }
    ctx.stroke();

    // Origin axes (X=red right, Y=green up in ROS but we flip Y on screen)
    const axLen = meterPx * 1.5;
    const axW = 2.5 / scale;

    // X axis (red) – points right in map pixels
    ctx.strokeStyle = "#e04040";
    ctx.lineWidth = axW;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(axLen, 0);
    ctx.stroke();

    // Y axis (green) – points up in ROS = negative Y in canvas (we're in scaled space)
    ctx.strokeStyle = "#40c040";
    ctx.lineWidth = axW;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -axLen);
    ctx.stroke();

    // Origin dot
    ctx.beginPath();
    ctx.arc(0, 0, 3 / scale, 0, Math.PI * 2);
    ctx.fillStyle = "#fff";
    ctx.fill();

    // Axis labels — draw in screen coords for consistent size
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const ox = canvas.width / 2 + panX;
    const oy = canvas.height / 2 + panY;
    ctx.font = "bold 11px monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillStyle = "#e04040";
    ctx.fillText("X", ox + axLen * scale + 10, oy);
    ctx.fillStyle = "#40c040";
    ctx.fillText("Y", ox, oy - axLen * scale - 10);
    ctx.restore();
  }

  // ---- TF coordinate frame axes -----------------------------------------

  function _drawTFAxes(pose, res, label) {
    const px = pose.x / res;
    const py = -pose.y / res;
    const axLen = 20 / scale;
    const axW = 2 / scale;

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-pose.theta);

    // X axis (red)
    ctx.strokeStyle = "#ff3333";
    ctx.lineWidth = axW;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(axLen, 0);
    ctx.stroke();

    // Arrow head
    ctx.fillStyle = "#ff3333";
    ctx.beginPath();
    ctx.moveTo(axLen + 3 / scale, 0);
    ctx.lineTo(axLen - 2 / scale, -2.5 / scale);
    ctx.lineTo(axLen - 2 / scale, 2.5 / scale);
    ctx.closePath();
    ctx.fill();

    // Y axis (green)
    ctx.strokeStyle = "#33ff33";
    ctx.lineWidth = axW;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -axLen);
    ctx.stroke();

    ctx.fillStyle = "#33ff33";
    ctx.beginPath();
    ctx.moveTo(0, -axLen - 3 / scale);
    ctx.lineTo(-2.5 / scale, -axLen + 2 / scale);
    ctx.lineTo(2.5 / scale, -axLen + 2 / scale);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // Label — draw outside scale transform so font is always 10px on screen
    if (label) {
      const screenX = (canvas.width / 2 + panX) + px * scale;
      const screenY = (canvas.height / 2 + panY) + py * scale;
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0); // reset to screen coords
      ctx.font = "10px monospace";
      ctx.fillStyle = "rgba(255,255,255,0.6)";
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(label, screenX + 6, screenY + 6);
      ctx.restore();
    }
  }

  // ---- Odom trail -------------------------------------------------------

  function _drawOdomTrail(res) {
    const now = Date.now();
    const maxAge = 30000; // 30 seconds fade

    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    for (let i = 1; i < odomTrail.length; i++) {
      const p0 = odomTrail[i - 1];
      const p1 = odomTrail[i];
      const age = now - p1.t;
      const alpha = Math.max(0.05, 1 - age / maxAge);

      ctx.strokeStyle = `rgba(255,170,0,${(alpha * 0.7).toFixed(3)})`;
      ctx.lineWidth = Math.max(1, 2.5 / scale) * alpha;

      ctx.beginPath();
      ctx.moveTo(p0.x / res, -p0.y / res);
      ctx.lineTo(p1.x / res, -p1.y / res);
      ctx.stroke();
    }

    // Small dots at trail points (every 10th)
    ctx.fillStyle = "rgba(255,170,0,0.4)";
    for (let i = 0; i < odomTrail.length; i += 10) {
      const p = odomTrail[i];
      const age = now - p.t;
      const alpha = Math.max(0.05, 1 - age / maxAge);
      ctx.globalAlpha = alpha;
      ctx.beginPath();
      ctx.arc(p.x / res, -p.y / res, 1.2 / scale, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
  }

  // ---- Navigation path --------------------------------------------------

  function _drawNavPath(res) {
    ctx.strokeStyle = "rgba(0,200,80,0.8)";
    ctx.lineWidth = 2 / scale;
    ctx.setLineDash([6 / scale, 3 / scale]);
    ctx.lineCap = "round";

    ctx.beginPath();
    ctx.moveTo(navPath[0].x / res, -navPath[0].y / res);
    for (let i = 1; i < navPath.length; i++) {
      ctx.lineTo(navPath[i].x / res, -navPath[i].y / res);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // ---- Laser scan -------------------------------------------------------

  function _drawLaserScan(res) {
    const dotR = 1.2 / scale;
    ctx.fillStyle = "#ff2222";

    for (let i = 0; i < laserPoints.length; i++) {
      const p = laserPoints[i];
      ctx.beginPath();
      ctx.arc(p.x / res, -p.y / res, dotR, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // ---- TurtleBot3 model (top-down footprint) ----------------------------

  function _drawTurtleBot(pose, res) {
    const px = pose.x / res;
    const py = -pose.y / res;
    // TurtleBot3 Burger: ~138mm radius body
    const bodyR = 8 / scale;

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-pose.theta);

    // Shadow
    ctx.beginPath();
    ctx.arc(1 / scale, 1 / scale, bodyR * 1.1, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(0,0,0,0.3)";
    ctx.fill();

    // Body plate (dark circle)
    ctx.beginPath();
    ctx.arc(0, 0, bodyR, 0, Math.PI * 2);
    const bodyGrad = ctx.createRadialGradient(
      -bodyR * 0.2, -bodyR * 0.2, 0,
      0, 0, bodyR
    );
    bodyGrad.addColorStop(0, "#555");
    bodyGrad.addColorStop(1, "#333");
    ctx.fillStyle = bodyGrad;
    ctx.fill();
    ctx.strokeStyle = "#666";
    ctx.lineWidth = 1 / scale;
    ctx.stroke();

    // Wheel wells (two rectangles on left/right)
    const wheelW = bodyR * 0.25;
    const wheelH = bodyR * 0.6;
    ctx.fillStyle = "#222";
    // Left wheel
    ctx.fillRect(-wheelW / 2, -bodyR * 0.95, wheelW, wheelH);
    // Right wheel
    ctx.fillRect(-wheelW / 2, bodyR * 0.95 - wheelH, wheelW, wheelH);

    // Caster (small circle at back)
    ctx.beginPath();
    ctx.arc(-bodyR * 0.65, 0, bodyR * 0.12, 0, Math.PI * 2);
    ctx.fillStyle = "#444";
    ctx.fill();

    // LiDAR unit on top (smaller circle, offset forward)
    ctx.beginPath();
    ctx.arc(bodyR * 0.15, 0, bodyR * 0.35, 0, Math.PI * 2);
    const lidarGrad = ctx.createRadialGradient(
      bodyR * 0.1, -bodyR * 0.05, 0,
      bodyR * 0.15, 0, bodyR * 0.35
    );
    lidarGrad.addColorStop(0, "#4a4a4a");
    lidarGrad.addColorStop(1, "#2a2a2a");
    ctx.fillStyle = lidarGrad;
    ctx.fill();
    ctx.strokeStyle = "#555";
    ctx.lineWidth = 0.5 / scale;
    ctx.stroke();

    // LiDAR spinning indicator
    ctx.beginPath();
    ctx.arc(bodyR * 0.15, 0, bodyR * 0.12, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(0,200,255,0.5)";
    ctx.lineWidth = 0.8 / scale;
    ctx.stroke();

    // Forward direction indicator (green LED-like dot)
    ctx.beginPath();
    ctx.arc(bodyR * 0.8, 0, 2 / scale, 0, Math.PI * 2);
    const ledGlow = ctx.createRadialGradient(
      bodyR * 0.8, 0, 0,
      bodyR * 0.8, 0, 4 / scale
    );
    ledGlow.addColorStop(0, "rgba(34,197,94,0.9)");
    ledGlow.addColorStop(1, "rgba(34,197,94,0)");
    ctx.fillStyle = ledGlow;
    ctx.fill();
    ctx.beginPath();
    ctx.arc(bodyR * 0.8, 0, 1.5 / scale, 0, Math.PI * 2);
    ctx.fillStyle = "#22c55e";
    ctx.fill();

    // Direction arrow (translucent)
    ctx.beginPath();
    ctx.moveTo(bodyR * 1.4, 0);
    ctx.lineTo(bodyR * 0.95, -bodyR * 0.35);
    ctx.lineTo(bodyR * 0.95, bodyR * 0.35);
    ctx.closePath();
    ctx.fillStyle = "rgba(34,197,94,0.6)";
    ctx.fill();

    ctx.restore();
  }

  // ---- Markers (goal / home) --------------------------------------------

  function _drawMarker(worldX, worldY, color, size, label, res) {
    if (!mapData) return;
    const px = worldX / res;
    const py = -worldY / res;
    const r = size / scale;

    // Soft glow
    const glow = ctx.createRadialGradient(px, py, r * 0.3, px, py, r * 2.5);
    glow.addColorStop(0, color + "55");
    glow.addColorStop(1, color + "00");
    ctx.beginPath();
    ctx.arc(px, py, r * 2.5, 0, Math.PI * 2);
    ctx.fillStyle = glow;
    ctx.fill();

    // Outer ring
    ctx.beginPath();
    ctx.arc(px, py, r * 1.3, 0, Math.PI * 2);
    ctx.strokeStyle = color + "88";
    ctx.lineWidth = 1 / scale;
    ctx.stroke();

    // Filled circle
    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.2 / scale;
    ctx.stroke();

    // Label
    if (label) {
      ctx.fillStyle = "#fff";
      ctx.font = `bold ${Math.max(8, 11 / scale)}px sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(label, px, py);
    }
  }

  // ---- Interaction ------------------------------------------------------

  function setMode(mode) {
    interactionMode = mode;
    dragMoved = false;  // reset so stale drag state doesn't swallow clicks
    canvas.style.cursor = mode ? "crosshair" : "grab";
  }

  function _onClick(e) {
    // If we were dragging (pan), ignore the click
    if (dragMoved) { dragMoved = false; return; }
    if (!interactionMode || !mapData) return;

    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left - canvas.width / 2 - panX;
    const cy = e.clientY - rect.top - canvas.height / 2 - panY;
    const res = mapData.info.resolution;
    const worldX = (cx / scale) * res;
    const worldY = -(cy / scale) * res;

    if (interactionMode === "navigate") {
      _sendNavGoal(worldX, worldY);
    } else if (interactionMode === "initial_pose") {
      _sendInitialPose(worldX, worldY);
    }

    interactionMode = null;
    canvas.style.cursor = "grab";
  }

  function _sendNavGoal(x, y) {
    goalPose = { x, y };
    _render();

    RosBridge.publish("/goal_pose", "geometry_msgs/msg/PoseStamped", {
      header: { frame_id: "map" },
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
    _setStatus(`Navigation goal sent: (${x.toFixed(2)}, ${y.toFixed(2)})`);
  }

  function _sendInitialPose(x, y) {
    RosBridge.publish("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", {
      header: { frame_id: "map" },
      pose: {
        pose: {
          position: { x, y, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        covariance: new Array(36).fill(0),
      },
    });
    _setStatus(`Initial pose set: (${x.toFixed(2)}, ${y.toFixed(2)})`);
  }

  function navigateTo(x, y, oz, ow) {
    goalPose = { x, y };
    _render();
    RosBridge.publish("/goal_pose", "geometry_msgs/msg/PoseStamped", {
      header: { frame_id: "map" },
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: oz || 0, w: ow || 1 },
      },
    });
  }

  // ---- Pan & Zoom -------------------------------------------------------

  function _onMouseDown(e) {
    if (interactionMode) return;
    isDragging = true;
    dragMoved = false;
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
    scale *= factor;
    scale = Math.max(0.2, Math.min(scale, 30));
    _render();
  }

  // ---- Helpers ----------------------------------------------------------

  function _resizeCanvas() {
    if (!canvas) return;
    const container = canvas.parentElement;
    canvas.width = container.clientWidth;
    canvas.height = container.clientHeight;
    _render();
  }

  function _quaternionToYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  function _updatePoseUI(pose) {
    document.getElementById("pos-x").textContent = pose.position.x.toFixed(3);
    document.getElementById("pos-y").textContent = pose.position.y.toFixed(3);
    document.getElementById("pos-z").textContent = pose.position.z.toFixed(3);
    document.getElementById("ori-x").textContent = pose.orientation.x.toFixed(3);
    document.getElementById("ori-y").textContent = pose.orientation.y.toFixed(3);
    document.getElementById("ori-z").textContent = pose.orientation.z.toFixed(3);
    document.getElementById("ori-w").textContent = pose.orientation.w.toFixed(3);
  }

  function _setStatus(msg) {
    document.getElementById("status-message").textContent = msg;
  }

  function getRobotPose() {
    return robotPose;
  }

  // ---- Public -----------------------------------------------------------

  return { init, subscribeTopics, setMode, navigateTo, getRobotPose };
})();
