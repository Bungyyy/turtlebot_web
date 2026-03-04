/**
 * Waypoint Manager
 * Add / remove / play through a sequence of waypoints.
 */

/* global MapViewer */
/* exported Waypoints */

const Waypoints = (() => {
  let waypoints = [];   // [{ x, y, oz, ow }]
  let selectedIdx = -1;
  let playing = false;
  let currentWpIdx = 0;

  function init() {
    document.getElementById("btn-add-wp").addEventListener("click", addCurrentPose);
    document.getElementById("btn-del-wp").addEventListener("click", deleteSelected);
    document.getElementById("btn-play-wp").addEventListener("click", togglePlay);
    document.getElementById("btn-home").addEventListener("click", goHome);
  }

  /** Add the robot's current pose as a waypoint. */
  function addCurrentPose() {
    const pose = MapViewer.getRobotPose();
    if (!pose) {
      _status("No robot pose available");
      return;
    }
    const yaw = pose.theta || 0;
    const oz = Math.sin(yaw / 2);
    const ow = Math.cos(yaw / 2);
    waypoints.push({ x: pose.x, y: pose.y, oz, ow });
    _renderTable();
    _status(`Waypoint ${waypoints.length} added`);
  }

  /** Add a waypoint with explicit values. */
  function addWaypoint(x, y, oz, ow) {
    waypoints.push({ x, y, oz: oz || 0, ow: ow || 1 });
    _renderTable();
  }

  function deleteSelected() {
    if (selectedIdx < 0 || selectedIdx >= waypoints.length) return;
    waypoints.splice(selectedIdx, 1);
    selectedIdx = -1;
    _renderTable();
  }

  function togglePlay() {
    if (playing) {
      playing = false;
      document.getElementById("btn-play-wp").textContent = "\u25B6";
      _status("Waypoint playback stopped");
      return;
    }

    if (waypoints.length === 0) {
      _status("No waypoints to play");
      return;
    }

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
    _status(`Navigating to waypoint ${currentWpIdx + 1}/${waypoints.length}`);
    _renderTable();
    MapViewer.navigateTo(wp.x, wp.y, wp.oz, wp.ow);

    // Simple polling: check if close to goal every 2 s
    const checkInterval = setInterval(() => {
      if (!playing) { clearInterval(checkInterval); return; }

      const pose = MapViewer.getRobotPose();
      if (pose) {
        const dx = pose.x - wp.x;
        const dy = pose.y - wp.y;
        if (Math.sqrt(dx * dx + dy * dy) < 0.3) {
          clearInterval(checkInterval);
          currentWpIdx++;
          setTimeout(_playNext, 500);
        }
      }
    }, 2000);
  }

  function goHome() {
    MapViewer.navigateTo(0, 0, 0, 1);
    _status("Navigating to Home (0, 0)");
  }

  // ---- Table rendering --------------------------------------------------

  function _renderTable() {
    const tbody = document.getElementById("waypoint-body");
    tbody.innerHTML = "";
    waypoints.forEach((wp, i) => {
      const tr = document.createElement("tr");
      if (i === selectedIdx) tr.classList.add("selected");
      if (playing && i === currentWpIdx) tr.classList.add("active-wp");

      tr.innerHTML = `
        <td>${i + 1}</td>
        <td>${wp.x.toFixed(3)}</td>
        <td>${wp.y.toFixed(3)}</td>
        <td>${wp.oz.toFixed(3)}</td>
        <td>${wp.ow.toFixed(3)}</td>`;
      tr.addEventListener("click", () => { selectedIdx = i; _renderTable(); });
      tbody.appendChild(tr);
    });
  }

  function _status(msg) {
    document.getElementById("status-message").textContent = msg;
  }

  return { init, addWaypoint, addCurrentPose, deleteSelected, togglePlay };
})();
