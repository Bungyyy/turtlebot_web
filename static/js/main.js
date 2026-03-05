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
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }

    badge.textContent = "ROS Connected";
    badge.className = "badge badge-connected";
    setNodeStatus("node-rosbridge", true);
    _setStatus("Connected to rosbridge");

    MapViewer.subscribeTopics();
    Controls.start();
    _checkNodes();
    _checkTopics();
  });

  RosBridge.on("close", () => {
    badge.textContent = "ROS Disconnected";
    badge.className = "badge badge-disconnected";
    ["node-rosbridge", "node-turtlebot", "node-slam", "node-navigation", "node-camera"]
      .forEach((id) => setNodeStatus(id, false));
    Controls.stop();
    _setStatus("Disconnected – reconnecting...");

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

  _connect();
  document.getElementById("btn-reconnect").addEventListener("click", _connect);

  // ---- Helper ------------------------------------------------------------

  function _setStatus(msg) {
    document.getElementById("status-message").textContent = msg;
  }

  function _updateStepBadge(id, text, cls) {
    const el = document.getElementById(id);
    if (el) { el.textContent = text; el.className = "step-badge " + (cls || ""); }
  }

  // ---- STEP 1: Robot Connection (Bringup + ROS Bridge) -------------------

  document.getElementById("btn-bringup").addEventListener("click", async () => {
    const btn = document.getElementById("btn-bringup");
    const sshHost = document.getElementById("ssh-host").value.trim();

    if (LaunchManager.isRunning("bringup")) {
      btn.textContent = "Stopping...";
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "bringup" }) });
      btn.textContent = "Start Bringup";
      _updateStepBadge("step1-badge", "Offline", "");
      _setStatus("Bringup stopped");
      return;
    }

    btn.textContent = "Starting...";
    _setStatus("Starting robot bringup...");

    const body = { name: "bringup" };
    if (sshHost) body.ssh_host = sshHost;

    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })).json();

    if (res.ok) {
      btn.textContent = "Stop Bringup";
      _updateStepBadge("step1-badge", "Running", "badge-ok");
      _setStatus("Robot bringup started" + (sshHost ? ` on ${sshHost}` : ""));
    } else {
      btn.textContent = "Start Bringup";
      _setStatus("Bringup failed: " + (res.message || res.error));
    }
  });

  document.getElementById("btn-rosbridge").addEventListener("click", async () => {
    const btn = document.getElementById("btn-rosbridge");

    if (LaunchManager.isRunning("rosbridge")) {
      btn.textContent = "Stopping...";
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "rosbridge" }) });
      btn.textContent = "Start ROS Bridge";
      _setStatus("ROS Bridge stopped");
      return;
    }

    btn.textContent = "Starting...";
    _setStatus("Starting ROS Bridge...");
    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: "rosbridge" }),
    })).json();

    if (res.ok) {
      btn.textContent = "Stop ROS Bridge";
      _setStatus("ROS Bridge started — connecting...");
      setTimeout(_connect, 2000);
    } else {
      btn.textContent = "Start ROS Bridge";
      _setStatus("ROS Bridge failed: " + (res.message || res.error));
    }
  });

  // ---- Process Log Viewer ------------------------------------------------

  const logEl = document.getElementById("process-log");
  const logBtn = document.getElementById("btn-show-log");
  let logVisible = false;
  let logPollTimer = null;

  logBtn.addEventListener("click", () => {
    logVisible = !logVisible;
    logEl.style.display = logVisible ? "block" : "none";
    logBtn.textContent = logVisible ? "Hide Log" : "Show Log";

    if (logVisible) {
      _pollLog();
      logPollTimer = setInterval(_pollLog, 2000);
    } else if (logPollTimer) {
      clearInterval(logPollTimer);
      logPollTimer = null;
    }
  });

  let _lastLaunchedProcess = "bringup";

  async function _pollLog() {
    // Show log for the last launched process first (even if stopped/failed)
    const order = [_lastLaunchedProcess, "slam", "navigation", "bringup", "rosbridge"];
    const seen = new Set();
    for (const name of order) {
      if (seen.has(name)) continue;
      seen.add(name);
      try {
        const data = await (await fetch(`/api/process_log/${name}`)).json();
        if (data.log && data.log.length > 0) {
          const status = data.running ? "running" : "STOPPED";
          logEl.textContent = `[${name}] (${status})\n` + data.log.join("\n");
          logEl.scrollTop = logEl.scrollHeight;
          return;
        }
      } catch (_) {}
    }
    logEl.textContent = "(no process log available)";
  }

  // ---- STEP 2: Mode (SLAM / Navigation) ---------------------------------

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
    // If SLAM is already running, stop it
    if (LaunchManager.isRunning("slam")) {
      _setStatus("Stopping SLAM...");
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "slam" }) });
      btnSlam.classList.remove("active");
      slamControls.style.display = "none";
      _updateStepBadge("step2-badge", "None", "");
      _setStatus("SLAM stopped");
      return;
    }

    _showModeUI("slam");
    _setStatus("Starting SLAM...");
    _lastLaunchedProcess = "slam";

    // Stop Navigation if running
    if (LaunchManager.isRunning("navigation")) {
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "navigation" }) });
    }

    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: "slam" }),
    })).json();

    if (res.ok) {
      _updateStepBadge("step2-badge", "SLAM", "badge-slam");
      _setStatus("SLAM started — drive the robot to map the area");
    } else {
      _setStatus("SLAM: " + (res.message || res.error));
    }
  });

  btnNav.addEventListener("click", async () => {
    // If Navigation is already running, stop it
    if (LaunchManager.isRunning("navigation")) {
      _setStatus("Stopping Navigation...");
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "navigation" }) });
      btnNav.classList.remove("active");
      navControls.style.display = "none";
      _updateStepBadge("step2-badge", "None", "");
      _setStatus("Navigation stopped");
      return;
    }

    const mapSelect = document.getElementById("lm-map-select");
    if (!mapSelect || !mapSelect.value) {
      _showModeUI("navigation");
      _setStatus("Select a saved map first, then click Navigation again");
      return;
    }

    _showModeUI("navigation");
    _setStatus("Starting Navigation...");
    _lastLaunchedProcess = "navigation";

    // Stop SLAM if running
    if (LaunchManager.isRunning("slam")) {
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "slam" }) });
    }

    const args = [`map:=${mapSelect.value}`];
    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: "navigation", args }),
    })).json();

    if (res.ok) {
      _updateStepBadge("step2-badge", "Nav", "badge-nav");
      _setStatus("Navigation started — use buttons below to control");
    } else {
      _setStatus("Nav: " + (res.message || res.error));
    }
  });

  // ---- STEP 3: Action buttons -------------------------------------------

  document.getElementById("btn-initial-pose").addEventListener("click", () => {
    MapViewer.setMode("initial_pose");
    _setStatus("Click on the map to set initial pose");
  });

  document.getElementById("btn-navigate").addEventListener("click", () => {
    MapViewer.setMode("navigate");
    const navNode = document.getElementById("node-navigation");
    const navOk = navNode && navNode.classList.contains("online");
    const hint = navOk ? "" : " (start Navigation mode first)";
    _setStatus("Click on the map to set goal" + hint);
  });

  // ---- Save map (toolbar + SLAM controls) --------------------------------

  document.getElementById("btn-save-map").addEventListener("click", () => {
    LaunchManager.saveMap();
  });

  // ---- Update button text based on process status -------------------------

  setInterval(() => {
    // Bringup button
    const bringBtn = document.getElementById("btn-bringup");
    if (LaunchManager.isRunning("bringup")) {
      bringBtn.textContent = "Stop Bringup";
      _updateStepBadge("step1-badge", "Running", "badge-ok");
    } else if (bringBtn.textContent === "Stop Bringup") {
      bringBtn.textContent = "Start Bringup";
      _updateStepBadge("step1-badge", "Offline", "");
    }

    // ROS Bridge button
    const rbBtn = document.getElementById("btn-rosbridge");
    if (LaunchManager.isRunning("rosbridge")) {
      rbBtn.textContent = "Stop ROS Bridge";
    } else if (rbBtn.textContent === "Stop ROS Bridge") {
      rbBtn.textContent = "Start ROS Bridge";
    }

    // SLAM / Nav badges
    if (LaunchManager.isRunning("slam")) {
      _updateStepBadge("step2-badge", "SLAM", "badge-slam");
    } else if (LaunchManager.isRunning("navigation")) {
      _updateStepBadge("step2-badge", "Nav", "badge-nav");
    }

    // Refresh node status indicators
    if (RosBridge.isConnected()) _checkNodes();
  }, 3000);

  // ---- Node health check ------------------------------------------------

  function _checkNodes() {
    RosBridge.callService("/rosapi/nodes", "rosapi/srv/Nodes", {})
      .then((result) => {
        const nodes = result.nodes || [];
        console.log("[Main] ROS nodes:", nodes);
        setNodeStatus("node-turtlebot", nodes.some((n) =>
          n.includes("turtlebot") || n.includes("diff_drive") ||
          n.includes("robot_state_publisher")
        ));
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
      .catch((err) => {
        console.warn("[Main] Could not query /rosapi/nodes:", err);
      });
  }

  // ---- Topic availability check -----------------------------------------

  function _checkTopics() {
    // Check via rosapi (through rosbridge WebSocket)
    RosBridge.callService("/rosapi/topics", "rosapi/srv/Topics", {})
      .then((result) => {
        const topics = result.topics || [];
        console.log("[Main] ROS topics (via rosbridge):", topics);
        const hasMap = topics.some((t) => t === "/map");
        const hasScan = topics.some((t) => t === "/scan");
        const hasOdom = topics.some((t) => t === "/odom");
        const hasCmdVel = topics.some((t) => t === "/cmd_vel");
        console.log("[Main] Key topics: /map=" + hasMap + " /scan=" + hasScan + " /odom=" + hasOdom + " /cmd_vel=" + hasCmdVel);
        if (!hasMap && !hasScan && !hasOdom) {
          _setStatus("Warning: No robot topics found — is bringup running?");
        }
      })
      .catch((err) => {
        console.warn("[Main] Could not query /rosapi/topics:", err);
        // Fallback: try server-side topic check
        _checkTopicsServerSide();
      });
  }

  function _checkTopicsServerSide() {
    fetch("/api/ros2/topics")
      .then((r) => r.json())
      .then((data) => {
        if (data.ok && data.topics) {
          console.log("[Main] ROS topics (server-side):", data.topics);
          const hasMap = data.topics.some((t) => t === "/map");
          if (!hasMap && data.topics.length < 5) {
            _setStatus("Warning: Very few ROS topics — check ROS2 network");
          }
        }
      })
      .catch(() => {});
  }
})();
