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

  function setNodeStatus(nodeId, online) {
    const el = document.getElementById(nodeId);
    if (online) el.classList.add("online");
    else el.classList.remove("online");
  }

  RosBridge.on("connect", () => {
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

    // Auto-reconnect after 3 s
    setTimeout(() => _connect(), 3000);
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

    // Try slam_toolbox serialize first (most common for SLAM mode)
    try {
      await RosBridge.callService(
        "/slam_toolbox/serialize_state",
        "slam_toolbox/srv/SerializePoseGraph",
        { filename: "map" }
      );
      statusEl.textContent = "Map serialized via slam_toolbox";
      return;
    } catch (_) { /* try next */ }

    // Fallback: slam_toolbox save_map
    try {
      await RosBridge.callService(
        "/slam_toolbox/save_map",
        "slam_toolbox/srv/SaveMap",
        { name: { data: "map" } }
      );
      statusEl.textContent = "Map saved via slam_toolbox";
      return;
    } catch (_) { /* try next */ }

    // Fallback: nav2 map_saver
    try {
      await RosBridge.callService(
        "/map_saver/save_map",
        "nav2_msgs/srv/SaveMap",
        { map_url: "map", image_format: "pgm" }
      );
      statusEl.textContent = "Map saved via nav2 map_saver";
      return;
    } catch (_) { /* all failed */ }

    statusEl.textContent = "Map save failed — no save service found. Run: ros2 run nav2_map_server map_saver_cli -f ~/map";
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
