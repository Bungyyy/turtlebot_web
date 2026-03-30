/**
 * Main Application Entry Point — Unitree Go2 Web Teleop
 * Wires together all modules, handles tab switching, Go2 state subscription.
 */

/* global RosBridge, MapViewer, Controls, Waypoints, Camera, LaunchManager, ROSLIB */

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

  // Teleop enable/disable toggle
  const teleopToggle = document.getElementById("teleop-enabled");
  const teleopLabel  = document.getElementById("teleop-toggle-label");
  if (teleopToggle) {
    const teleopCard = teleopToggle.closest(".card-controls");
    teleopToggle.addEventListener("change", () => {
      const on = teleopToggle.checked;
      Controls.setEnabled(on);
      if (teleopLabel) teleopLabel.textContent = on ? "ON" : "OFF";
      if (teleopCard) teleopCard.classList.toggle("teleop-disabled", !on);
    });
  }

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
    _setStatus("Connected to rosbridge — teleop via ROS Bridge active");

    MapViewer.subscribeTopics();
    Controls.start();
    _subscribeGo2State();
    _checkNodes();
    _checkTopics();
  });

  RosBridge.on("close", () => {
    badge.textContent = "ROS Disconnected";
    badge.className = "badge badge-disconnected";
    ["node-rosbridge", "node-turtlebot", "node-slam", "node-navigation", "node-camera"]
      .forEach((id) => setNodeStatus(id, false));
    Controls.stop();
    _setStatus("Disconnected — reconnecting...");

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

  function _launchBody(name, extra) {
    const body = { name, ...extra };
    const sshHost = document.getElementById("ssh-host").value.trim();
    const sshPassword = document.getElementById("ssh-password").value;
    if (sshHost) body.ssh_host = sshHost;
    if (sshPassword) body.ssh_password = sshPassword;
    return body;
  }

  // ---- Go2 State Subscription (via rosbridge) ----------------------------

  let go2StateSub = null;

  function _subscribeGo2State() {
    // Try subscribing to Go2 sport mode state for battery, mode, gait info
    // Topic name varies: /sportmodestate, /go2/sportmodestate, /lowstate, etc.
    const topicCandidates = [
      { topic: "/sportmodestate", type: "unitree_go2_msgs/msg/SportModeState" },
      { topic: "/go2/sportmodestate", type: "unitree_go2_msgs/msg/SportModeState" },
      { topic: "/lowstate", type: "unitree_go2_msgs/msg/LowState" },
    ];

    // Check available topics then subscribe
    RosBridge.callService("/rosapi/topics", "rosapi/srv/Topics", {})
      .then((result) => {
        const topics = result.topics || [];

        for (const candidate of topicCandidates) {
          if (topics.includes(candidate.topic)) {
            console.log("[Main] Subscribing to Go2 state:", candidate.topic);
            go2StateSub = RosBridge.subscribe(candidate.topic, candidate.type, _onGo2State, { throttle: 500 });
            return;
          }
        }

        // Also try to get battery from /bms_state if available
        if (topics.includes("/bms_state")) {
          RosBridge.subscribe("/bms_state", "unitree_go2_msgs/msg/BmsState", (msg) => {
            _updateBattery(msg.soc || msg.battery_percentage || 0);
          }, { throttle: 2000 });
        }

        console.log("[Main] No Go2 state topic found — status panel will show limited info");
      })
      .catch(() => {
        console.log("[Main] Could not query topics for Go2 state");
      });
  }

  function _onGo2State(msg) {
    // SportModeState: mode, gait_type, progress, foot_raise_height, body_height, velocity, imu, etc.
    const modeEl = document.getElementById("go2-mode");
    const gaitEl = document.getElementById("go2-gait");

    if (modeEl && msg.mode !== undefined) {
      const modeNames = { 0: "Idle", 1: "Balancing", 2: "Locomotion", 3: "Lidar Stance", 5: "AI Sport", 7: "Jump", 8: "Go Stairs" };
      modeEl.textContent = modeNames[msg.mode] || `Mode ${msg.mode}`;
    }

    if (gaitEl && msg.gait_type !== undefined) {
      const gaitNames = { 0: "Idle", 1: "Trot", 2: "Trot Running", 3: "Climb Stairs", 4: "Trot Obstacle" };
      gaitEl.textContent = gaitNames[msg.gait_type] || `Gait ${msg.gait_type}`;
    }

    // Battery (if available in sport mode state)
    if (msg.bms_state && msg.bms_state.soc !== undefined) {
      _updateBattery(msg.bms_state.soc);
    }
  }

  function _updateBattery(pct) {
    const fill = document.getElementById("go2-battery-fill");
    const label = document.getElementById("go2-battery-pct");
    if (!fill || !label) return;

    const p = Math.max(0, Math.min(100, pct));
    fill.style.width = p + "%";
    label.textContent = p + "%";

    fill.classList.remove("low", "medium");
    if (p < 20) fill.classList.add("low");
    else if (p < 50) fill.classList.add("medium");
  }

  // ---- STEP 1: Robot Connection (Bringup + ROS Bridge) -------------------

  document.getElementById("btn-bringup").addEventListener("click", async () => {
    const btn = document.getElementById("btn-bringup");
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

    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("bringup")),
    })).json();

    if (res.ok) {
      btn.textContent = "Stop Bringup";
      _updateStepBadge("step1-badge", "Running", "badge-ok");
      _setStatus("Robot bringup started");
    } else {
      btn.textContent = "Start Bringup";
      _setStatus("Bringup failed: " + (res.message || res.error));
    }
  });

  document.getElementById("btn-teleop-node").addEventListener("click", async () => {
    const btn = document.getElementById("btn-teleop-node");
    if (LaunchManager.isRunning("teleop")) {
      btn.textContent = "Stopping...";
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "teleop" }) });
      btn.textContent = "Start Teleop Node";
      _setStatus("Teleop node stopped");
      return;
    }

    btn.textContent = "Starting...";
    _setStatus("Starting teleop node (go2_teleop)...");

    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("teleop")),
    })).json();

    if (res.ok) {
      btn.textContent = "Stop Teleop Node";
      _setStatus("Teleop node started — /cmd_vel bridge active");
    } else {
      btn.textContent = "Start Teleop Node";
      _setStatus("Teleop failed: " + (res.message || res.error));
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
      body: JSON.stringify(_launchBody("rosbridge")),
    })).json();

    if (res.ok) {
      btn.textContent = "Stop ROS Bridge";
      _setStatus("ROS Bridge started — connecting...");
      const sshHost = document.getElementById("ssh-host").value.trim();
      if (sshHost) {
        const ip = sshHost.includes("@") ? sshHost.split("@")[1] : sshHost;
        const rbHostInput = document.getElementById("setting-rosbridge-host");
        if (rbHostInput && ip) rbHostInput.value = ip;
      }
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
    const order = [_lastLaunchedProcess, "slam", "livox", "pcl_to_scan", "navigation", "bringup", "rosbridge"];
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
  const locControls = document.getElementById("loc-controls");
  const navControls = document.getElementById("nav-controls");
  const btnSlam = document.getElementById("btn-slam-mode");
  const btnLoc = document.getElementById("btn-loc-mode");
  const btnNav = document.getElementById("btn-nav-mode");

  function _showModeUI(mode) {
    btnSlam.classList.toggle("active", mode === "slam");
    btnLoc.classList.toggle("active", mode === "localization");
    btnNav.classList.toggle("active", mode === "navigation");
    slamControls.style.display = mode === "slam" ? "block" : "none";
    locControls.style.display = mode === "localization" ? "block" : "none";
    navControls.style.display = mode === "navigation" ? "block" : "none";
  }

  /** Check if a process is still running after launch. Returns { running, log } */
  async function _checkProcessHealth(name, delayMs) {
    await new Promise((r) => setTimeout(r, delayMs));
    try {
      const data = await (await fetch(`/api/process_log/${name}`)).json();
      return { running: data.running, log: data.log || [] };
    } catch (_) {
      return { running: false, log: [] };
    }
  }

  btnSlam.addEventListener("click", async () => {
    if (LaunchManager.isRunning("slam") || LaunchManager.isRunning("livox") || LaunchManager.isRunning("pcl_to_scan")) {
      _setStatus("Stopping SLAM...");
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "pcl_to_scan" }) });
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "slam" }) });
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "livox" }) });
      btnSlam.classList.remove("active");
      slamControls.style.display = "none";
      _updateStepBadge("step2-badge", "None", "");
      _setStatus("SLAM stopped");
      return;
    }

    _showModeUI("slam");
    _setStatus("Starting SLAM...");
    _lastLaunchedProcess = "slam";

    // Stop conflicting modes
    for (const proc of ["navigation", "nav_stack", "transform", "localization"]) {
      if (LaunchManager.isRunning(proc)) {
        await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: proc }) });
      }
    }

    // Ensure bringup is running (needed for odometry)
    if (!LaunchManager.isRunning("bringup")) {
      _setStatus("Starting bringup for SLAM...");
      await (await fetch("/api/launch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(_launchBody("bringup")),
      })).json();
      await new Promise((r) => setTimeout(r, 2000));
    }

    // Launch Livox LiDAR driver first
    _setStatus("Starting LiDAR driver (Livox MID-360)...");
    const livoxRes = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("livox")),
    })).json();

    if (!livoxRes.ok) {
      _setStatus("LiDAR driver failed: " + (livoxRes.message || livoxRes.error));
      return;
    }

    // Check if livox is actually still running after 3s
    _setStatus("Waiting for LiDAR driver to initialize...");
    const livoxHealth = await _checkProcessHealth("livox", 3000);
    if (!livoxHealth.running) {
      const lastLines = livoxHealth.log.slice(-5).join("\n");
      _setStatus("LiDAR driver crashed: " + (lastLines || "check logs"));
      _lastLaunchedProcess = "livox";
      return;
    }

    // Launch FAST-LIO2 mapping
    _setStatus("Starting FAST-LIO2 mapping...");
    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("slam")),
    })).json();

    if (!res.ok) {
      _setStatus("SLAM: " + (res.message || res.error));
      return;
    }

    // Check if fast_lio is actually still running after 3s
    const slamHealth = await _checkProcessHealth("slam", 3000);
    if (!slamHealth.running) {
      const lastLines = slamHealth.log.slice(-5).join("\n");
      _setStatus("FAST-LIO2 crashed: " + (lastLines || "check logs"));
      return;
    }

    // Launch pointcloud_to_laserscan to convert /cloud_registered -> /scan
    _setStatus("Starting pointcloud to laserscan converter...");
    await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("pcl_to_scan")),
    })).json();

    _updateStepBadge("step2-badge", "FAST-LIO2", "badge-slam");
    _setStatus("FAST-LIO2 3D mapping started — drive the robot to map the area");
  });

  // ---- Localization mode: bringup + localization.launch --------------------

  btnLoc.addEventListener("click", async () => {
    if (LaunchManager.isRunning("localization")) {
      _setStatus("Stopping Localization...");
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "goal_handler" }) });
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "localization" }) });
      btnLoc.classList.remove("active");
      locControls.style.display = "none";
      _updateStepBadge("step2-badge", "None", "");
      _setStatus("Localization stopped");
      return;
    }

    const locMapSelect = document.getElementById("lm-loc-map-select");
    const hasLocTopicMap = MapViewer.hasMapTopic();
    const hasLocSelectedMap = locMapSelect && locMapSelect.value;

    if (!hasLocSelectedMap && !hasLocTopicMap) {
      _showModeUI("localization");
      _setStatus("Select a saved map first or wait for a map on /map topic");
      return;
    }

    _showModeUI("localization");
    _setStatus("Starting Localization + Navigation (AMCL)...");
    _lastLaunchedProcess = "localization";

    // Stop conflicting modes
    for (const proc of ["slam", "livox", "pcl_to_scan", "navigation", "nav_stack", "transform", "goal_handler"]) {
      if (LaunchManager.isRunning(proc)) {
        await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: proc }) });
      }
    }

    // Ensure bringup is running
    if (!LaunchManager.isRunning("bringup")) {
      _setStatus("Starting bringup for localization...");
      await (await fetch("/api/launch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(_launchBody("bringup")),
      })).json();
    }

    // Launch full nav stack with AMCL + map
    // (localization command now uses navigation.launch with use_amcl:=True)
    const locArgs = hasLocSelectedMap ? [`map:=${locMapSelect.value}`] : [];
    _setStatus("Starting AMCL + Nav2 stack...");
    const res = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("localization", { args: locArgs })),
    })).json();

    if (res.ok) {
      _updateStepBadge("step2-badge", "Localization", "badge-nav");
      _setStatus(hasLocSelectedMap
        ? "AMCL localization started with map — set initial pose, then navigate"
        : "AMCL localization started (using live map topic) — set initial pose, then navigate");

      // Auto-launch goal handler after nav stack is ready (bt_navigator crashes on Jetson)
      setTimeout(async () => {
        _setStatus("Starting goal handler...");
        await (await fetch("/api/launch", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(_launchBody("goal_handler")),
        })).json();
        _setStatus("Localization ready — set initial pose on the map, then navigate");
      }, 5000);
    } else {
      _setStatus("Localization: " + (res.message || res.error));
    }
  });

  // ---- Navigation mode: transform.launch + navigation.launch --------------

  btnNav.addEventListener("click", async () => {
    if (LaunchManager.isRunning("nav_stack") || LaunchManager.isRunning("transform")) {
      _setStatus("Stopping Navigation...");
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "goal_handler" }) });
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "nav_stack" }) });
      await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: "transform" }) });
      btnNav.classList.remove("active");
      navControls.style.display = "none";
      _updateStepBadge("step2-badge", "None", "");
      _setStatus("Navigation stopped");
      return;
    }

    const mapSelect = document.getElementById("lm-map-select");
    const hasTopicMap = MapViewer.hasMapTopic();
    const hasSelectedMap = mapSelect && mapSelect.value;

    if (!hasSelectedMap && !hasTopicMap) {
      _showModeUI("navigation");
      _setStatus("Select a saved map first or wait for a map on /map topic");
      return;
    }

    _showModeUI("navigation");
    _setStatus("Starting Navigation...");
    _lastLaunchedProcess = "nav_stack";

    // Stop conflicting modes
    for (const proc of ["slam", "livox", "pcl_to_scan", "navigation", "localization", "goal_handler"]) {
      if (LaunchManager.isRunning(proc)) {
        await fetch("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: proc }) });
      }
    }

    // Launch transform.launch first
    _setStatus("Starting transform...");
    const trRes = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("transform")),
    })).json();

    if (!trRes.ok) {
      _setStatus("Transform failed: " + (trRes.message || trRes.error));
      return;
    }

    // Launch navigation.launch with map (if selected) or without (using topic map)
    _setStatus("Starting navigation stack...");
    const navArgs = hasSelectedMap ? [`map:=${mapSelect.value}`] : [];
    const navRes = await (await fetch("/api/launch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(_launchBody("nav_stack", { args: navArgs })),
    })).json();

    if (navRes.ok) {
      _updateStepBadge("step2-badge", "Nav", "badge-nav");
      _setStatus(hasSelectedMap ? "Navigation started with map" : "Navigation started (using live map topic)");

      // Auto-launch lightweight goal handler (replaces bt_navigator which crashes on Jetson)
      setTimeout(async () => {
        _setStatus("Starting goal handler...");
        await (await fetch("/api/launch", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(_launchBody("goal_handler")),
        })).json();
        _setStatus("Navigation ready — set a goal on the map");
      }, 3000);
    } else {
      _setStatus("Navigation: " + (navRes.message || navRes.error));
    }
  });

  // ---- STEP 3: Navigation controls ----------------------------------------

  const navBanner = document.getElementById("nav-mode-banner");
  const navBannerText = document.getElementById("nav-mode-banner-text");
  const navModeBadge = document.getElementById("nav-mode-badge");
  const btnCancelNav = document.getElementById("btn-cancel-nav");

  function _showNavBanner(mode, text) {
    navBanner.style.display = "flex";
    const modeClass = mode === "initial_pose" ? "mode-initial" : "mode-navigate";
    navBanner.className = "nav-mode-banner " + modeClass;
    navBannerText.textContent = text;
    const badgeText = mode === "initial_pose" ? "Setting Pose"
                    : mode === "add_waypoint" ? "Adding WPs"
                    : "Setting Goal";
    const badgeClass = mode === "initial_pose" ? "badge-slam"
                     : mode === "add_waypoint" ? "badge-nav"
                     : "badge-nav";
    navModeBadge.textContent = badgeText;
    navModeBadge.className = "step-badge " + badgeClass;
  }

  function _hideNavBanner() {
    navBanner.style.display = "none";
    if (!MapViewer.getNav2Status().navActive) {
      navModeBadge.textContent = "Idle";
      navModeBadge.className = "step-badge";
    }
  }

  // Cancel mode button in banner
  document.getElementById("nav-mode-cancel").addEventListener("click", () => {
    MapViewer.setMode(null);
    _hideNavBanner();
    _setStatus("Mode cancelled");
  });

  // "Click Map" button for initial pose
  document.getElementById("btn-initial-pose-map").addEventListener("click", () => {
    MapViewer.setMode("initial_pose");
    _showNavBanner("initial_pose", "Click on map to set initial pose (drag to set heading)");
    _setStatus("Click on the map to set initial pose");
  });

  // "Click Map" button for navigation goal
  document.getElementById("btn-navigate-map").addEventListener("click", () => {
    MapViewer.setMode("navigate");
    const navNode = document.getElementById("node-navigation");
    const navOk = navNode && navNode.classList.contains("online");
    _showNavBanner("navigate", "Click on map to set goal (drag to set heading)");
    _setStatus("Click on the map to set goal" + (navOk ? "" : " (start Navigation first)"));
  });

  // Set Initial Pose from manual inputs
  document.getElementById("btn-set-initial-pose").addEventListener("click", () => {
    const x = parseFloat(document.getElementById("init-pose-x").value) || 0;
    const y = parseFloat(document.getElementById("init-pose-y").value) || 0;
    const yawDeg = parseFloat(document.getElementById("init-pose-yaw").value) || 0;
    const yaw = yawDeg * Math.PI / 180;
    MapViewer.sendInitialPose(x, y, yaw);
  });

  // Send Navigation Goal from manual inputs
  document.getElementById("btn-send-nav-goal").addEventListener("click", () => {
    const x = parseFloat(document.getElementById("nav-goal-x").value) || 0;
    const y = parseFloat(document.getElementById("nav-goal-y").value) || 0;
    const yawDeg = parseFloat(document.getElementById("nav-goal-yaw").value) || 0;
    const yaw = yawDeg * Math.PI / 180;
    const oz = Math.sin(yaw / 2);
    const ow = Math.cos(yaw / 2);
    MapViewer.sendNavGoal(x, y, oz, ow);
    btnCancelNav.style.display = "block";
    navModeBadge.textContent = "Navigating";
    navModeBadge.className = "step-badge badge-nav";
  });

  // Cancel Navigation
  btnCancelNav.addEventListener("click", () => {
    MapViewer.cancelNavigation();
    btnCancelNav.style.display = "none";
    navModeBadge.textContent = "Idle";
    navModeBadge.className = "step-badge";
  });

  // Use current robot pose as initial pose
  document.getElementById("btn-use-current-pose").addEventListener("click", () => {
    const pose = MapViewer.getRobotPose();
    if (!pose) {
      _setStatus("No robot pose available");
      return;
    }
    document.getElementById("init-pose-x").value = pose.x.toFixed(2);
    document.getElementById("init-pose-y").value = pose.y.toFixed(2);
    document.getElementById("init-pose-yaw").value = (pose.theta * 180 / Math.PI).toFixed(0);
    _setStatus("Filled initial pose from current robot position");
  });

  // When a map interaction completes, update the input fields
  MapViewer.onModeComplete((mode, x, y, yaw) => {
    const deg = (yaw * 180 / Math.PI).toFixed(0);
    if (mode === "initial_pose") {
      document.getElementById("init-pose-x").value = x.toFixed(2);
      document.getElementById("init-pose-y").value = y.toFixed(2);
      document.getElementById("init-pose-yaw").value = deg;
      _hideNavBanner();
    } else if (mode === "navigate") {
      document.getElementById("nav-goal-x").value = x.toFixed(2);
      document.getElementById("nav-goal-y").value = y.toFixed(2);
      document.getElementById("nav-goal-yaw").value = deg;
      btnCancelNav.style.display = "block";
      navModeBadge.textContent = "Navigating";
      navModeBadge.className = "step-badge badge-nav";
      _hideNavBanner();
    } else if (mode === "add_waypoint") {
      // Add clicked position as waypoint — stay in mode
      const oz = Math.sin(yaw / 2), ow = Math.cos(yaw / 2);
      Waypoints.addWaypoint(x, y, oz, ow);
      _setStatus(`Waypoint added at (${x.toFixed(2)}, ${y.toFixed(2)}) — click to add more, ESC to finish`);
    }
  });

  // "Click Map to Add Waypoint" button
  document.getElementById("btn-add-wp-click").addEventListener("click", () => {
    MapViewer.setMode("add_waypoint");
    _showNavBanner("add_waypoint", "Click map to add waypoints (drag to set heading, ESC to finish)");
    _setStatus("Click on the map to add waypoints");
  });

  // ESC key exits add_waypoint mode
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      MapViewer.setMode(null);
      _hideNavBanner();
    }
  });

  // ---- Waypoint Presets ---------------------------------------------------

  const presetSelector = document.getElementById("preset-selector");

  async function _loadPresetList() {
    try {
      const res = await fetch("/api/waypoint_presets");
      const data = await res.json();
      if (!data.ok) return;
      // Keep first option (placeholder)
      presetSelector.innerHTML = '<option value="">-- Presets --</option>';
      for (const [name, count] of Object.entries(data.presets || {})) {
        const opt = document.createElement("option");
        opt.value = name;
        opt.textContent = `${name} (${count} pts)`;
        presetSelector.appendChild(opt);
      }
    } catch (e) {
      console.warn("[Presets] Failed to load:", e);
    }
  }

  // Load preset list on startup
  _loadPresetList();

  // Load preset
  document.getElementById("btn-load-preset").addEventListener("click", async () => {
    const name = presetSelector.value;
    if (!name) { _setStatus("Select a preset first"); return; }
    try {
      const res = await fetch(`/api/waypoint_presets/${encodeURIComponent(name)}`);
      const data = await res.json();
      if (data.ok && data.waypoints) {
        Waypoints.setWaypoints(data.waypoints);
        _setStatus(`Loaded preset "${name}" (${data.waypoints.length} waypoints)`);
      } else {
        _setStatus("Failed to load preset: " + (data.error || "unknown"));
      }
    } catch (e) {
      _setStatus("Load preset error: " + e.message);
    }
  });

  // Save preset
  document.getElementById("btn-save-preset").addEventListener("click", async () => {
    const wps = Waypoints.getWaypoints();
    if (wps.length === 0) { _setStatus("No waypoints to save"); return; }
    const name = prompt("Preset name:");
    if (!name) return;
    try {
      const res = await fetch(`/api/waypoint_presets/${encodeURIComponent(name)}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ waypoints: wps }),
      });
      const data = await res.json();
      if (data.ok) {
        _setStatus(`Saved preset "${name}" (${wps.length} waypoints)`);
        await _loadPresetList();
        presetSelector.value = name;
      } else {
        _setStatus("Save preset failed: " + (data.error || "unknown"));
      }
    } catch (e) {
      _setStatus("Save preset error: " + e.message);
    }
  });

  // Delete preset
  document.getElementById("btn-del-preset").addEventListener("click", async () => {
    const name = presetSelector.value;
    if (!name) { _setStatus("Select a preset to delete"); return; }
    if (!confirm(`Delete preset "${name}"?`)) return;
    try {
      const res = await fetch(`/api/waypoint_presets/${encodeURIComponent(name)}`, { method: "DELETE" });
      const data = await res.json();
      if (data.ok) {
        _setStatus(`Deleted preset "${name}"`);
        await _loadPresetList();
      } else {
        _setStatus("Delete failed: " + (data.error || "unknown"));
      }
    } catch (e) {
      _setStatus("Delete preset error: " + e.message);
    }
  });

  // Update cancel button visibility based on nav2 status
  setInterval(() => {
    const status = MapViewer.getNav2Status();
    if (status.navActive) {
      btnCancelNav.style.display = "block";
      navModeBadge.textContent = "Navigating";
      navModeBadge.className = "step-badge badge-nav";
    } else if (btnCancelNav.style.display === "block") {
      btnCancelNav.style.display = "none";
      navModeBadge.textContent = "Idle";
      navModeBadge.className = "step-badge";
    }
  }, 1000);

  // Save map
  document.getElementById("btn-save-map").addEventListener("click", () => {
    LaunchManager.saveMap();
  });

  // ---- Update button text based on process status -------------------------

  setInterval(() => {
    const bringBtn = document.getElementById("btn-bringup");
    if (LaunchManager.isRunning("bringup")) {
      bringBtn.textContent = "Stop Bringup";
      _updateStepBadge("step1-badge", "Running", "badge-ok");
    } else if (bringBtn.textContent === "Stop Bringup") {
      bringBtn.textContent = "Start Bringup";
      _updateStepBadge("step1-badge", "Offline", "");
    }

    const teleopBtn = document.getElementById("btn-teleop-node");
    if (LaunchManager.isRunning("teleop")) {
      teleopBtn.textContent = "Stop Teleop Node";
    } else if (teleopBtn.textContent === "Stop Teleop Node") {
      teleopBtn.textContent = "Start Teleop Node";
    }

    const rbBtn = document.getElementById("btn-rosbridge");
    if (LaunchManager.isRunning("rosbridge")) {
      rbBtn.textContent = "Stop ROS Bridge";
    } else if (rbBtn.textContent === "Stop ROS Bridge") {
      rbBtn.textContent = "Start ROS Bridge";
    }

    if (LaunchManager.isRunning("slam")) {
      _updateStepBadge("step2-badge", "FAST-LIO2", "badge-slam");
    } else if (LaunchManager.isRunning("localization")) {
      _updateStepBadge("step2-badge", "Localization", "badge-nav");
    } else if (LaunchManager.isRunning("nav_stack") || LaunchManager.isRunning("transform")) {
      _updateStepBadge("step2-badge", "Nav", "badge-nav");
    } else if (LaunchManager.isRunning("navigation")) {
      _updateStepBadge("step2-badge", "Nav", "badge-nav");
    }

    if (RosBridge.isConnected()) _checkNodes();
  }, 3000);

  // ---- Node health check ------------------------------------------------

  function _setRosNode(id, active) {
    const el = document.getElementById(id);
    if (el) el.classList.toggle("active", active);
  }

  let _lastNodeKey = "";
  function _checkNodes() {
    RosBridge.callService("/rosapi/nodes", "rosapi/srv/Nodes", {})
      .then((result) => {
        const nodes = result.nodes || [];
        const key = nodes.slice().sort().join(",");
        if (key !== _lastNodeKey) {
          _lastNodeKey = key;
          console.log("[Main] ROS nodes:", nodes);
        }

        const hasGo2 = nodes.some((n) =>
          n.includes("go2") || n.includes("unitree") ||
          n.includes("robot_state_publisher")
        );
        const hasLivox = nodes.some((n) =>
          n.includes("livox") || n.includes("MID360")
        );
        const hasSlam = nodes.some((n) =>
          n.includes("slam") || n.includes("fast_lio") ||
          n.includes("fastlio") || n.includes("lio_sam")
        );
        const hasAmcl = nodes.some((n) => n.includes("amcl"));
        const hasNav = nodes.some((n) =>
          n.includes("bt_navigator") || n.includes("controller_server") ||
          n.includes("planner_server") || n.includes("nav2") ||
          n.includes("navigation") || n.includes("move_base") ||
          n.includes("lifecycle_manager_navigation") || n.includes("behavior_server")
        );
        const hasMapServer = nodes.some((n) => n.includes("map_server"));
        const hasCamera = nodes.some((n) => n.includes("camera") || n.includes("image") || n.includes("astra"));

        setNodeStatus("node-turtlebot", hasGo2);
        setNodeStatus("node-livox", hasLivox);
        setNodeStatus("node-slam", hasSlam || hasAmcl);
        setNodeStatus("node-navigation", hasNav);
        setNodeStatus("node-camera", hasCamera);

        _setRosNode("rn-rosbridge", true);
        _setRosNode("rn-turtlebot", hasGo2);
        _setRosNode("rn-livox", hasLivox);
        _setRosNode("rn-slam", hasSlam);
        _setRosNode("rn-amcl", hasAmcl);
        _setRosNode("rn-navigation", hasNav);
        _setRosNode("rn-map-server", hasMapServer);
      })
      .catch(() => {});
  }

  // ---- Topic availability check -----------------------------------------

  function _checkTopics() {
    RosBridge.callService("/rosapi/topics", "rosapi/srv/Topics", {})
      .then((result) => {
        const topics = result.topics || [];
        console.log("[Main] ROS topics:", topics.length, "total");
        const hasScan = topics.includes("/scan");
        const hasOdom = topics.includes("/odom");
        const hasCmdVel = topics.includes("/cmd_vel");
        console.log("[Main] Key: /scan=" + hasScan + " /odom=" + hasOdom + " /cmd_vel=" + hasCmdVel);
        if (!hasScan && !hasOdom) {
          _setStatus("Warning: No robot topics found — is bringup running?");
        }
      })
      .catch(() => {
        _checkTopicsServerSide();
      });
  }

  function _checkTopicsServerSide() {
    return fetch("/api/ros2/topics")
      .then((r) => r.json())
      .then((data) => {
        if (data.ok && data.topics) {
          const hasScan = data.topics.includes("/scan");
          const hasOdom = data.topics.includes("/odom");
          if (!hasScan && !hasOdom) {
            _setStatus("No robot topics visible. Check ROS_DOMAIN_ID matches robot.");
          }
        }
      })
      .catch(() => {});
  }

  // ---- Check Topics button (Settings) ------------------------------------

  const checkTopicsBtn = document.getElementById("btn-check-topics");
  if (checkTopicsBtn) {
    checkTopicsBtn.addEventListener("click", async () => {
      const output = document.getElementById("ros2-topics-output");
      output.style.display = "block";
      output.textContent = "Checking via rosbridge...";

      if (!RosBridge.isConnected()) {
        output.textContent = "Not connected to rosbridge.";
        return;
      }

      try {
        const result = await RosBridge.callService("/rosapi/topics", "rosapi/srv/Topics", {});
        const topics = (result.topics || []).sort();
        const check = (t) => topics.includes(t) ? "YES" : "NO";
        output.textContent =
          `Key topics:\n  /scan: ${check("/scan")}\n  /odom: ${check("/odom")}\n  /map: ${check("/map")}\n  /cmd_vel: ${check("/cmd_vel")}\n  /api/sport/request: ${check("/api/sport/request")}\n\n` +
          `All topics (${topics.length}):\n` + topics.join("\n");
      } catch (e) {
        output.textContent = "Query failed: " + e.message;
        try {
          const data = await (await fetch("/api/ros2/topics")).json();
          if (data.ok) output.textContent = `Topics (${data.topics.length}):\n` + data.topics.join("\n");
        } catch (_) {}
      }
    });
  }

  // ---- Map Switcher -------------------------------------------------------

  const mapSwitcher = document.getElementById("map-switcher");
  const btnSwitchMap = document.getElementById("btn-switch-map");
  const btnSaveMapPose = document.getElementById("btn-save-map-pose");
  const btnAddMap = document.getElementById("btn-add-map");

  async function _loadMapsConfig() {
    try {
      const res = await fetch("/api/maps/config");
      const config = await res.json();
      if (!mapSwitcher) return;
      mapSwitcher.innerHTML = "";
      for (const [id, m] of Object.entries(config.maps || {})) {
        const opt = document.createElement("option");
        opt.value = id;
        opt.textContent = m.name || id;
        if (id === config.active_map) opt.selected = true;
        mapSwitcher.appendChild(opt);
      }
      return config;
    } catch (e) {
      console.warn("[MapSwitcher] Failed to load config:", e);
    }
  }

  // Load on startup
  _loadMapsConfig();

  if (btnSwitchMap) {
    btnSwitchMap.addEventListener("click", async () => {
      const mapId = mapSwitcher.value;
      if (!mapId) return;
      btnSwitchMap.textContent = "Loading...";
      btnSwitchMap.disabled = true;
      try {
        // Save current waypoints first
        const currentWps = typeof Waypoints !== "undefined" && Waypoints.getWaypoints
          ? Waypoints.getWaypoints() : [];
        const res = await fetch("/api/maps/switch", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ map_id: mapId, current_waypoints: currentWps }),
        });
        const data = await res.json();
        if (data.ok) {
          _setStatus("Switched to map: " + (data.name || mapId));
          // Load new waypoints
          if (typeof Waypoints !== "undefined" && Waypoints.setWaypoints && data.waypoints) {
            Waypoints.setWaypoints(data.waypoints);
          }
          // Pre-fill initial pose
          if (data.initial_pose) {
            const ip = data.initial_pose;
            const el = (id) => document.getElementById(id);
            if (el("init-pose-x")) el("init-pose-x").value = (ip.x || 0).toFixed(2);
            if (el("init-pose-y")) el("init-pose-y").value = (ip.y || 0).toFixed(2);
            if (el("init-pose-yaw")) el("init-pose-yaw").value = Math.round((ip.yaw || 0) * 180 / Math.PI);
          }
        } else {
          _setStatus("Map switch failed: " + (data.error || "unknown"));
        }
      } catch (e) {
        _setStatus("Map switch error: " + e.message);
      }
      btnSwitchMap.textContent = "Switch";
      btnSwitchMap.disabled = false;
    });
  }

  if (btnSaveMapPose) {
    btnSaveMapPose.addEventListener("click", async () => {
      const mapId = mapSwitcher.value;
      if (!mapId) return;
      const el = (id) => document.getElementById(id);
      const x = parseFloat((el("init-pose-x") || {}).value) || 0;
      const y = parseFloat((el("init-pose-y") || {}).value) || 0;
      const yawDeg = parseFloat((el("init-pose-yaw") || {}).value) || 0;
      const yaw = yawDeg * Math.PI / 180;
      // Also save current waypoints
      const wps = typeof Waypoints !== "undefined" && Waypoints.getWaypoints
        ? Waypoints.getWaypoints() : [];
      try {
        await fetch("/api/maps/save", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            map_id: mapId,
            initial_pose: { x, y, yaw },
            waypoints: wps,
          }),
        });
        _setStatus("Saved pose & waypoints for map: " + mapId);
      } catch (e) {
        _setStatus("Save failed: " + e.message);
      }
    });
  }

  if (btnAddMap) {
    btnAddMap.addEventListener("click", async () => {
      const name = prompt("Map name (e.g. Floor 2):");
      if (!name) return;
      const yamlPath = prompt("Full path to .yaml file:", "/home/unitree/go2_ws/map/");
      if (!yamlPath) return;
      const mapId = name.toLowerCase().replace(/[^a-z0-9]/g, "_");
      try {
        await fetch("/api/maps/add", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ map_id: mapId, name, yaml_path: yamlPath }),
        });
        await _loadMapsConfig();
        _setStatus("Added map: " + name);
      } catch (e) {
        _setStatus("Add map failed: " + e.message);
      }
    });
  }
})();
