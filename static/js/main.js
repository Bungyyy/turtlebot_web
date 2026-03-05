/**
 * Main Application Entry Point
 * Wires together all modules and handles tab switching.
 */

/* global RosBridge, MapViewer, Controls, Waypoints, Camera, LaunchManager */

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
  LaunchManager.init();

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

  // ---- Mode buttons (SLAM / Navigation) ---------------------------------

  const slamControls = document.getElementById("slam-controls");
  const navControls = document.getElementById("nav-controls");
  const btnSlam = document.getElementById("btn-slam-mode");
  const btnNav = document.getElementById("btn-nav-mode");

  function _showModeUI(mode) {
    btnSlam.classList.toggle("active", mode === "slam");
    btnNav.classList.toggle("active", mode === "navigation");
    slamControls.style.display = mode === "slam" ? "block" : "none";
    navControls.style.display = mode === "navigation" ? "block" : "none";
  }

  btnSlam.addEventListener("click", async () => {
    _showModeUI("slam");
    document.getElementById("status-message").textContent = "Starting SLAM...";

    // Stop Navigation if running
    if (LaunchManager.isRunning("navigation")) {
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "navigation" }) });
    }

    // Start SLAM
    if (!LaunchManager.isRunning("slam")) {
      const res = await (await fetch("/api/launch", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "slam" }) })).json();
      document.getElementById("status-message").textContent = res.ok ? "SLAM started — drive around to map" : "SLAM: " + (res.message || res.error);
    } else {
      document.getElementById("status-message").textContent = "SLAM is running — drive around to map";
    }
  });

  btnNav.addEventListener("click", async () => {
    const mapSelect = document.getElementById("lm-map-select");

    // Check if a map is selected
    if (!mapSelect || !mapSelect.value) {
      _showModeUI("navigation");
      document.getElementById("status-message").textContent = "Select a saved map first, then click Navigation again";
      return;
    }

    _showModeUI("navigation");
    document.getElementById("status-message").textContent = "Starting Navigation...";

    // Stop SLAM if running
    if (LaunchManager.isRunning("slam")) {
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "slam" }) });
    }

    // Start Navigation with map
    if (!LaunchManager.isRunning("navigation")) {
      const args = [`map:=${mapSelect.value}`];
      const res = await (await fetch("/api/launch", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "navigation", args }) })).json();
      document.getElementById("status-message").textContent = res.ok ? "Navigation started — click Navigate to send goals" : "Nav: " + (res.message || res.error);
    } else {
      document.getElementById("status-message").textContent = "Navigation is running";
    }
  });

  // ---- Action buttons ---------------------------------------------------

  document.getElementById("btn-initial-pose").addEventListener("click", () => {
    MapViewer.setMode("initial_pose");
    document.getElementById("status-message").textContent = "Click on the map to set initial pose";
  });

  document.getElementById("btn-navigate").addEventListener("click", () => {
    MapViewer.setMode("navigate");
    const navNode = document.getElementById("node-navigation");
    const navOk = navNode && navNode.classList.contains("online");
    const hint = navOk ? "" : " (Nav2 not detected — start Navigation mode first)";
    document.getElementById("status-message").textContent = "Click on the map to set navigation goal" + hint;
  });

  document.getElementById("btn-manual").addEventListener("click", (e) => {
    e.target.classList.toggle("active");
    MapViewer.setMode(null);
    document.getElementById("status-message").textContent = "Manual control mode — use D-pad or WASD keys";
  });

  // ---- Save map (toolbar button) ----------------------------------------

  document.getElementById("btn-save-map").addEventListener("click", () => {
    // Delegate to the Launch Manager save (uses the name from the SLAM controls input)
    LaunchManager.saveMap();
  });

  // ---- Node health check ------------------------------------------------

  function _checkNodes() {
    RosBridge.callService("/rosapi/nodes", "rosapi/srv/Nodes", {})
      .then((result) => {
        const nodes = result.nodes || [];
        setNodeStatus("node-turtlebot", nodes.some((n) => n.includes("turtlebot")));
        setNodeStatus("node-slam", nodes.some((n) =>
          n.includes("slam") || n.includes("cartographer") ||
          n.includes("gmapping") || n.includes("map_server") || n.includes("amcl")
        ));
        setNodeStatus("node-navigation", nodes.some((n) =>
          n.includes("bt_navigator") || n.includes("controller_server") ||
          n.includes("planner_server") || n.includes("nav2") ||
          n.includes("navigation") || n.includes("move_base") ||
          n.includes("lifecycle_manager_navigation") || n.includes("behavior_server")
        ));
        setNodeStatus("node-camera", nodes.some((n) => n.includes("camera") || n.includes("image") || n.includes("astra")));
      })
      .catch(() => {
        console.warn("[Main] Could not query /rosapi/nodes");
      });
  }
})();
