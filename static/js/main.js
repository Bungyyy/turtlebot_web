/**
 * Main Application Entry Point
 * Wires together all modules and handles tab switching.
 */

/* global RosBridge, MapViewer, Controls, Waypoints, Camera */

(function () {
  "use strict";

  // ---- Tab Navigation ---------------------------------------------------

  const tabs = document.querySelectorAll(".nav-tabs li");
  const panels = document.querySelectorAll(".tab-panel");

  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.remove("active"));
      panels.forEach((p) => p.classList.remove("active"));
      tab.classList.add("active");
      const target = document.getElementById("tab-" + tab.dataset.tab);
      if (target) target.classList.add("active");
    });
  });

  // ---- Initialise modules -----------------------------------------------

  MapViewer.init();
  Controls.init();
  Waypoints.init();
  Camera.init();

  // ---- ROS Connection ---------------------------------------------------

  const badge = document.getElementById("ros-connection-badge");
  let reconnectTimer = null;

  function setNodeStatus(nodeId, online) {
    const el = document.getElementById(nodeId);
    if (online) el.classList.add("online");
    else el.classList.remove("online");
  }

  RosBridge.on("connect", () => {
    // Clear any pending reconnect timer
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }

    badge.textContent = "ROS Connected";
    badge.className = "badge badge-connected";
    setNodeStatus("node-rosbridge", true);
    document.getElementById("status-message").textContent = "Connected to rosbridge";

    // Subscribe to map, pose, etc.
    MapViewer.subscribeTopics();
    Controls.start();

    // Probe node availability
    _checkNodes();
  });

  RosBridge.on("close", () => {
    badge.textContent = "ROS Disconnected";
    badge.className = "badge badge-disconnected";
    ["node-rosbridge", "node-turtlebot", "node-slam", "node-navigation", "node-camera"]
      .forEach((id) => setNodeStatus(id, false));
    document.getElementById("status-message").textContent = "Disconnected – reconnecting...";

    // Auto-reconnect after 3 s (clear any existing timer first)
    if (reconnectTimer) clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(() => { reconnectTimer = null; _connect(); }, 3000);
  });

  RosBridge.on("error", () => {
    badge.textContent = "ROS Error";
    badge.className = "badge badge-disconnected";
  });

  function _connect() {
    const host = document.getElementById("setting-rosbridge-host").value || window.APP_CONFIG.rosbridgeHost;
    const port = parseInt(document.getElementById("setting-rosbridge-port").value) || window.APP_CONFIG.rosbridgePort;
    RosBridge.connect(host, port);
  }

  // Initial connection
  _connect();

  // Reconnect button (settings tab)
  document.getElementById("btn-reconnect").addEventListener("click", _connect);

  // ---- Mode buttons -----------------------------------------------------

  document.getElementById("btn-nav-mode").addEventListener("click", () => {
    document.getElementById("btn-nav-mode").classList.add("active");
    document.getElementById("btn-slam-mode").classList.remove("active");
    document.getElementById("status-message").textContent = "Mode: Navigation";
  });

  document.getElementById("btn-slam-mode").addEventListener("click", () => {
    document.getElementById("btn-slam-mode").classList.add("active");
    document.getElementById("btn-nav-mode").classList.remove("active");
    document.getElementById("status-message").textContent = "Mode: SLAM";
  });

  // ---- Action buttons ---------------------------------------------------

  document.getElementById("btn-initial-pose").addEventListener("click", () => {
    MapViewer.setMode("initial_pose");
    document.getElementById("status-message").textContent = "Click on the map to set initial pose";
  });

  document.getElementById("btn-navigate").addEventListener("click", () => {
    MapViewer.setMode("navigate");
    document.getElementById("status-message").textContent = "Click on the map to set navigation goal";
  });

  document.getElementById("btn-manual").addEventListener("click", (e) => {
    e.target.classList.toggle("active");
    MapViewer.setMode(null);
    document.getElementById("status-message").textContent = "Manual control mode";
  });

  // ---- Save map ---------------------------------------------------------

  document.getElementById("btn-save-map").addEventListener("click", async () => {
    const statusEl = document.getElementById("status-message");
    statusEl.textContent = "Saving map...";

    // Try ROS services first (rosbridge direct)
    const services = [
      { name: "/slam_toolbox/serialize_state", type: "slam_toolbox/srv/SerializePoseGraph", req: { filename: "map" }, label: "slam_toolbox serialize" },
      { name: "/slam_toolbox/save_map", type: "slam_toolbox/srv/SaveMap", req: { name: { data: "map" } }, label: "slam_toolbox save_map" },
      { name: "/map_saver/save_map", type: "nav2_msgs/srv/SaveMap", req: { map_url: "map", image_format: "pgm" }, label: "nav2 map_saver" },
    ];

    for (const svc of services) {
      try {
        await RosBridge.callService(svc.name, svc.type, svc.req);
        statusEl.textContent = `Map saved via ${svc.label}`;
        return;
      } catch (_) { /* try next */ }
    }

    // Fallback: call our backend which runs the CLI command
    try {
      const resp = await fetch("/api/save_map", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: "map" }),
      });
      const data = await resp.json();
      if (data.ok) {
        statusEl.textContent = `Map saved to ${data.path} (${data.method})`;
        return;
      }
      statusEl.textContent = `Map save failed: ${data.error}`;
    } catch (err) {
      statusEl.textContent = `Map save failed: ${err.message}`;
    }
  });

  // ---- Node health check ------------------------------------------------

  function _checkNodes() {
    RosBridge.callService("/rosapi/nodes", "rosapi/srv/Nodes", {})
      .then((result) => {
        const nodes = result.nodes || [];
        setNodeStatus("node-turtlebot", nodes.some((n) => n.includes("turtlebot")));
        setNodeStatus("node-slam", nodes.some((n) => n.includes("slam") || n.includes("cartographer") || n.includes("gmapping")));
        setNodeStatus("node-navigation", nodes.some((n) => n.includes("move_base") || n.includes("nav2") || n.includes("navigation")));
        setNodeStatus("node-camera", nodes.some((n) => n.includes("camera") || n.includes("image") || n.includes("astra")));
      })
      .catch(() => {
        console.warn("[Main] Could not query /rosapi/nodes");
      });
  }
})();
