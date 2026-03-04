/**
 * Map Viewer
 * Renders the OccupancyGrid (/map topic), robot pose, goal markers,
 * and handles click-to-navigate.
 */

/* global RosBridge, ROSLIB */
/* exported MapViewer */

const MapViewer = (() => {
  let canvas, ctx;
  let mapData = null;         // last OccupancyGrid message
  let mapImage = null;        // pre-rendered ImageData
  let robotPose = null;       // { x, y, theta }
  let goalPose = null;        // { x, y }
  let homePose = { x: 0, y: 0 };

  let scale = 1.0;
  let panX = 0, panY = 0;
  let isDragging = false, dragStart = { x: 0, y: 0 };

  // Interaction mode: "navigate" | "initial_pose" | null
  let interactionMode = null;

  // ---- Initialisation ---------------------------------------------------

  function init() {
    canvas = document.getElementById("map-canvas");
    ctx = canvas.getContext("2d");
    _resizeCanvas();
    window.addEventListener("resize", _resizeCanvas);

    // Mouse / touch interactions
    canvas.addEventListener("mousedown", _onMouseDown);
    canvas.addEventListener("mousemove", _onMouseMove);
    canvas.addEventListener("mouseup", _onMouseUp);
    canvas.addEventListener("wheel", _onWheel, { passive: false });
    canvas.addEventListener("click", _onClick);

    // Zoom buttons
    document.getElementById("btn-zoom-in").addEventListener("click", () => { scale *= 1.25; _render(); });
    document.getElementById("btn-zoom-out").addEventListener("click", () => { scale *= 0.8; _render(); });
  }

  function subscribeTopics() {
    // OccupancyGrid
    RosBridge.subscribe("/map", "nav_msgs/OccupancyGrid", (msg) => {
      mapData = msg;
      _buildMapImage(msg);
      _render();
      document.getElementById("map-resolution").textContent = `Res: ${msg.info.resolution.toFixed(3)} m/px`;
      document.getElementById("map-size").textContent = `Size: ${msg.info.width}x${msg.info.height}`;
    }, { throttle: 2000 });

    // Robot pose (AMCL or odom)
    RosBridge.subscribe("/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped", (msg) => {
      const p = msg.pose.pose;
      const yaw = _quaternionToYaw(p.orientation);
      robotPose = { x: p.position.x, y: p.position.y, theta: yaw };
      _updatePoseUI(p);
      _render();
    }, { throttle: 200 });

    // Fallback: odom
    RosBridge.subscribe("/odom", "nav_msgs/Odometry", (msg) => {
      if (!robotPose) {
        const p = msg.pose.pose;
        const yaw = _quaternionToYaw(p.orientation);
        robotPose = { x: p.position.x, y: p.position.y, theta: yaw };
        _updatePoseUI(p);
        _render();
      }
    }, { throttle: 200 });

    // Navigation result
    RosBridge.subscribe("/move_base/result", "move_base_msgs/MoveBaseActionResult", () => {
      goalPose = null;
      _render();
    });
  }

  // ---- Map Rendering ----------------------------------------------------

  function _buildMapImage(msg) {
    const { width, height, data } = { width: msg.info.width, height: msg.info.height, data: msg.data };
    const imgData = ctx.createImageData(width, height);

    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      const idx = i * 4;
      if (val === -1) {
        // Unknown
        imgData.data[idx]     = 40;
        imgData.data[idx + 1] = 44;
        imgData.data[idx + 2] = 52;
      } else if (val === 0) {
        // Free
        imgData.data[idx]     = 220;
        imgData.data[idx + 1] = 220;
        imgData.data[idx + 2] = 220;
      } else {
        // Occupied
        imgData.data[idx]     = 20;
        imgData.data[idx + 1] = 20;
        imgData.data[idx + 2] = 30;
      }
      imgData.data[idx + 3] = 255;
    }

    // Store as offscreen canvas for quick drawing
    const offscreen = document.createElement("canvas");
    offscreen.width = width;
    offscreen.height = height;
    offscreen.getContext("2d").putImageData(imgData, 0, 0);
    mapImage = offscreen;
  }

  function _render() {
    if (!canvas) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#111318";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    ctx.translate(canvas.width / 2 + panX, canvas.height / 2 + panY);
    ctx.scale(scale, scale);

    // Draw map
    if (mapImage && mapData) {
      const res = mapData.info.resolution;
      const ox = mapData.info.origin.position.x;
      const oy = mapData.info.origin.position.y;
      const w = mapData.info.width;
      const h = mapData.info.height;

      ctx.save();
      ctx.scale(1, -1); // flip Y for ROS convention
      ctx.drawImage(mapImage, ox / res, -(oy / res) - h, w, h);
      ctx.restore();
    }

    // Draw home marker
    _drawMarker(homePose.x, homePose.y, "#3b82f6", 6, "H");

    // Draw goal marker
    if (goalPose) {
      _drawMarker(goalPose.x, goalPose.y, "#ef4444", 7, "G");
    }

    // Draw robot
    if (robotPose && mapData) {
      _drawRobot(robotPose);
    }

    ctx.restore();
  }

  function _drawRobot(pose) {
    const res = mapData ? mapData.info.resolution : 0.05;
    const px = pose.x / res;
    const py = -pose.y / res;
    const r = 8 / scale;

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-pose.theta);

    // Body
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(34,197,94,0.8)";
    ctx.fill();
    ctx.strokeStyle = "#16a34a";
    ctx.lineWidth = 1.5 / scale;
    ctx.stroke();

    // Direction arrow
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.lineTo(r * 0.4, -r * 0.5);
    ctx.lineTo(r * 0.4, r * 0.5);
    ctx.closePath();
    ctx.fillStyle = "#fff";
    ctx.fill();

    ctx.restore();
  }

  function _drawMarker(worldX, worldY, color, size, label) {
    if (!mapData) return;
    const res = mapData.info.resolution;
    const px = worldX / res;
    const py = -worldY / res;
    const r = size / scale;

    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    if (label) {
      ctx.fillStyle = "#fff";
      ctx.font = `bold ${Math.max(10, 12 / scale)}px sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(label, px, py);
    }
  }

  // ---- Interaction ------------------------------------------------------

  function setMode(mode) {
    interactionMode = mode;
    canvas.style.cursor = mode ? "crosshair" : "grab";
  }

  function _onClick(e) {
    if (!interactionMode || !mapData) return;

    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left - canvas.width / 2 - panX;
    const cy = e.clientY - rect.top - canvas.height / 2 - panY;
    const worldX = (cx / scale) * mapData.info.resolution;
    const worldY = -(cy / scale) * mapData.info.resolution;

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

    RosBridge.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", {
      header: { frame_id: "map" },
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
    _setStatus(`Navigation goal sent: (${x.toFixed(2)}, ${y.toFixed(2)})`);
  }

  function _sendInitialPose(x, y) {
    RosBridge.publish("/initialpose", "geometry_msgs/PoseWithCovarianceStamped", {
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
    RosBridge.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", {
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
    dragStart = { x: e.clientX - panX, y: e.clientY - panY };
    canvas.style.cursor = "grabbing";
  }

  function _onMouseMove(e) {
    if (!isDragging) return;
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
    scale = Math.max(0.2, Math.min(scale, 20));
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
