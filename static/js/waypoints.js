/**
 * Waypoint Manager
 * Add / remove / play through a sequence of waypoints.
 * Supports "map_switch" waypoints that change the nav2 map mid-mission.
 *
 * Waypoint types:
 *   - "nav"        : normal navigation waypoint { x, y, oz, ow }
 *   - "map_switch" : switch map when reached { x, y, oz, ow, switch_to, switch_pose }
 */

/* global MapViewer */
/* exported Waypoints */

const Waypoints = (() => {
  // Each waypoint: { x, y, oz, ow, type: "nav"|"map_switch", switch_to?, switch_pose? }
  let waypoints = [];
  let selectedIdx = -1;
  let playing = false;
  let currentWpIdx = 0;

  function init() {
    document.getElementById("btn-add-wp").addEventListener("click", addCurrentPose);
    document.getElementById("btn-del-wp").addEventListener("click", deleteSelected);
    document.getElementById("btn-play-wp").addEventListener("click", togglePlay);
    document.getElementById("btn-home").addEventListener("click", goHome);
    const btnMapSwitch = document.getElementById("btn-add-map-switch-wp");
    if (btnMapSwitch) btnMapSwitch.addEventListener("click", addMapSwitchPrompt);
  }

  /** Add the robot's current pose as a normal nav waypoint. */
  function addCurrentPose() {
    const pose = MapViewer.getRobotPose();
    if (!pose) { _status("No robot pose available"); return; }
    const yaw = pose.theta || 0;
    waypoints.push({
      x: pose.x, y: pose.y,
      oz: Math.sin(yaw / 2), ow: Math.cos(yaw / 2),
      type: "nav",
    });
    _renderTable();
    _status(`Waypoint ${waypoints.length} added`);
  }

  /** Add a waypoint with explicit values. */
  function addWaypoint(x, y, oz, ow) {
    waypoints.push({ x, y, oz: oz || 0, ow: ow || 1, type: "nav" });
    _renderTable();
  }

  /** Add a map-switch waypoint. When reached during playback, switches the nav2 map. */
  function addMapSwitch(x, y, oz, ow, switchTo, switchPose) {
    waypoints.push({
      x, y, oz: oz || 0, ow: ow || 1,
      type: "map_switch",
      switch_to: switchTo,       // map_id in maps.json
      switch_pose: switchPose,   // { x, y, yaw } — initial pose on the new map
    });
    _renderTable();
    _status(`Map-switch waypoint added → ${switchTo}`);
  }

  /** Prompt user to add a map-switch waypoint at current pose. */
  function addMapSwitchPrompt() {
    const pose = MapViewer.getRobotPose();
    if (!pose) { _status("No robot pose available"); return; }

    const switchTo = prompt("Switch to which map? (map_id from maps.json, e.g. floor2):");
    if (!switchTo) return;
    const sx = prompt("Initial X on new map:", "0.0");
    if (sx === null) return;
    const sy = prompt("Initial Y on new map:", "0.0");
    if (sy === null) return;
    const syaw = prompt("Initial Yaw (degrees) on new map:", "0");
    if (syaw === null) return;

    const yaw = pose.theta || 0;
    addMapSwitch(
      pose.x, pose.y, Math.sin(yaw / 2), Math.cos(yaw / 2),
      switchTo,
      { x: parseFloat(sx), y: parseFloat(sy), yaw: parseFloat(syaw) * Math.PI / 180 }
    );
  }

  function deleteSelected() {
    if (selectedIdx < 0 || selectedIdx >= waypoints.length) return;
    waypoints.splice(selectedIdx, 1);
    selectedIdx = -1;
    _renderTable();
  }

  // ---- Playback -----------------------------------------------------------

  function togglePlay() {
    if (playing) {
      playing = false;
      document.getElementById("btn-play-wp").textContent = "\u25B6";
      _status("Waypoint playback stopped");
      return;
    }
    if (waypoints.length === 0) { _status("No waypoints to play"); return; }

    playing = true;
    currentWpIdx = 0;
    document.getElementById("btn-play-wp").textContent = "\u25A0";
    _playNext();
  }

  function _playNext() {
    if (!playing || currentWpIdx >= waypoints.length) {
      playing = false;
      document.getElementById("btn-play-wp").textContent = "\u25B6";
      _status("Waypoint playback complete");
      _renderTable();
      return;
    }

    const wp = waypoints[currentWpIdx];
    const label = wp.type === "map_switch"
      ? `WP ${currentWpIdx + 1} [MAP→${wp.switch_to}]`
      : `WP ${currentWpIdx + 1}/${waypoints.length}`;
    _status(`Navigating to ${label}`);
    _renderTable();

    // Navigate to the waypoint position first
    MapViewer.navigateTo(wp.x, wp.y, wp.oz, wp.ow);

    // Poll until robot reaches the waypoint
    const checkInterval = setInterval(() => {
      if (!playing) { clearInterval(checkInterval); return; }

      const pose = MapViewer.getRobotPose();
      if (!pose) return;

      const dx = pose.x - wp.x, dy = pose.y - wp.y;
      if (Math.sqrt(dx * dx + dy * dy) < 0.3) {
        clearInterval(checkInterval);

        if (wp.type === "map_switch") {
          // Execute map switch, then continue
          _executeMapSwitch(wp).then(() => {
            currentWpIdx++;
            setTimeout(_playNext, 1000); // extra delay for map to settle
          });
        } else {
          currentWpIdx++;
          setTimeout(_playNext, 500);
        }
      }
    }, 2000);
  }

  /** Switch the nav2 map and set initial pose on the new map. */
  async function _executeMapSwitch(wp) {
    _status(`Switching map → ${wp.switch_to}...`);
    try {
      // Save current waypoints to old map before switching
      const res = await fetch("/api/maps/switch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          map_id: wp.switch_to,
          current_waypoints: [], // don't overwrite — mission waypoints span maps
        }),
      });
      const data = await res.json();
      if (!data.ok) {
        _status(`Map switch failed: ${data.error}`);
        playing = false;
        return;
      }
      _status(`Map loaded: ${data.name || wp.switch_to}`);

      // Update the map switcher dropdown
      const sel = document.getElementById("map-switcher");
      if (sel) sel.value = wp.switch_to;

      // Set initial pose on new map
      if (wp.switch_pose) {
        const sp = wp.switch_pose;
        const oz = Math.sin((sp.yaw || 0) / 2);
        const ow = Math.cos((sp.yaw || 0) / 2);

        // Send via backend (PoseStamped for lidar_localization)
        await fetch("/api/nav2/initial_pose", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ x: sp.x || 0, y: sp.y || 0, oz, ow }),
        });

        // Also update web UI pose
        MapViewer.sendInitialPose(sp.x || 0, sp.y || 0, sp.yaw || 0);
        _status(`Initial pose set on ${wp.switch_to}, continuing mission...`);
      }

      // Wait for localization to converge
      await new Promise(r => setTimeout(r, 3000));
    } catch (e) {
      _status(`Map switch error: ${e.message}`);
      playing = false;
    }
  }

  // ---- Table rendering ----------------------------------------------------

  function _renderTable() {
    const tbody = document.getElementById("waypoint-body");
    tbody.innerHTML = "";
    waypoints.forEach((wp, i) => {
      const tr = document.createElement("tr");
      if (i === selectedIdx) tr.classList.add("selected");
      if (playing && i === currentWpIdx) tr.classList.add("active-wp");

      const isSwitch = wp.type === "map_switch";
      const label = isSwitch ? `\u21C4 ${wp.switch_to || "?"}` : `${i + 1}`;
      const rowStyle = isSwitch ? ' style="background: #2a1a3a; color: #c084fc;"' : "";

      tr.innerHTML = `
        <td${rowStyle}>${label}</td>
        <td>${wp.x.toFixed(3)}</td>
        <td>${wp.y.toFixed(3)}</td>
        <td>${wp.oz.toFixed(3)}</td>
        <td>${wp.ow.toFixed(3)}</td>`;
      tr.addEventListener("click", () => { selectedIdx = i; _renderTable(); });
      tbody.appendChild(tr);
    });
  }

  function goHome() {
    MapViewer.navigateTo(0, 0, 0, 1);
    _status("Navigating to Home (0, 0)");
  }

  function _status(msg) {
    document.getElementById("status-message").textContent = msg;
  }

  // ---- Public API ---------------------------------------------------------

  function getWaypoints() { return waypoints.slice(); }

  function setWaypoints(wps) {
    waypoints = (wps || []).map(w => ({
      x: w.x || 0, y: w.y || 0, oz: w.oz || 0, ow: w.ow || 1,
      type: w.type || "nav",
      switch_to: w.switch_to || undefined,
      switch_pose: w.switch_pose || undefined,
    }));
    selectedIdx = -1;
    _renderTable();
  }

  function clearWaypoints() { waypoints = []; selectedIdx = -1; _renderTable(); }

  return {
    init, addWaypoint, addCurrentPose, addMapSwitch, addMapSwitchPrompt,
    deleteSelected, togglePlay, goHome,
    getWaypoints, setWaypoints, clearWaypoints,
  };
})();
