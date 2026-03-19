/**
 * Launch Manager
 * Start/stop ROS2 processes (bringup, SLAM, navigation, rosbridge)
 * from the web UI via backend API.
 */

/* exported LaunchManager */

const LaunchManager = (() => {
  let pollTimer = null;
  let currentStatus = {};

  function init() {
    // Start/stop button handlers (Settings tab)
    document.querySelectorAll("[data-launch]").forEach((btn) => {
      btn.addEventListener("click", () => _toggleProcess(btn.dataset.launch));
    });

    // Save map button in Mode card (SLAM controls)
    const saveBtn = document.getElementById("btn-lm-save-map");
    if (saveBtn) saveBtn.addEventListener("click", saveMap);

    // Start polling process status
    _poll();
    pollTimer = setInterval(_poll, 3000);
  }

  // ---- API calls ----------------------------------------------------------

  async function _apiPost(endpoint, body) {
    try {
      const resp = await fetch(`/api/${endpoint}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      return await resp.json();
    } catch (e) {
      return { ok: false, error: e.message };
    }
  }

  async function _apiGet(endpoint) {
    try {
      const resp = await fetch(`/api/${endpoint}`);
      return await resp.json();
    } catch (e) {
      return null;
    }
  }

  // ---- Process control ----------------------------------------------------

  async function _toggleProcess(name) {
    const running = currentStatus[name] && currentStatus[name].running;
    const btn = document.querySelector(`[data-launch="${name}"]`);
    if (btn) {
      btn.disabled = true;
      btn.textContent = running ? "Stopping..." : "Starting...";
    }

    let result;
    if (running) {
      result = await _apiPost("stop", { name });
    } else {
      const args = _getExtraArgs(name);
      result = await _apiPost("launch", { name, args });
    }

    _setStatusMsg(result.message || result.error || "");

    // Quick refresh
    setTimeout(_poll, 500);
    setTimeout(_poll, 2000);
  }

  function _getExtraArgs(name) {
    if (name === "navigation") {
      const mapSelect = document.getElementById("lm-map-select");
      if (mapSelect && mapSelect.value) {
        return [`map:=${mapSelect.value}`];
      }
    }
    return [];
  }

  // ---- Status polling -----------------------------------------------------

  async function _poll() {
    const status = await _apiGet("processes");
    if (!status) return;
    currentStatus = status;
    _updateUI(status);
    _refreshMaps();
  }

  function _updateUI(status) {
    for (const [name, info] of Object.entries(status)) {
      const row = document.getElementById(`lm-${name}`);
      if (!row) continue;

      const indicator = row.querySelector(".lm-indicator");
      const btn = row.querySelector(`[data-launch="${name}"]`);
      const statusText = row.querySelector(".lm-status-text");

      if (indicator) {
        indicator.className = "lm-indicator " + (info.running ? "running" : "stopped");
      }
      if (btn) {
        btn.disabled = false;
        btn.textContent = info.running ? "Stop" : "Start";
        btn.className = "btn-action " + (info.running ? "btn-stop-proc" : "btn-primary");
      }
      if (statusText) {
        statusText.textContent = info.running ? `Running (PID ${info.pid})` : "Stopped";
      }
    }
  }

  // ---- Map management -----------------------------------------------------

  let _lastMapCount = -1;

  async function _refreshMaps() {
    const data = await _apiGet("maps");
    if (!data || !data.maps) return;

    // Only update DOM if maps changed
    if (data.maps.length === _lastMapCount) return;
    _lastMapCount = data.maps.length;

    const selectors = ["lm-map-select", "lm-loc-map-select"];
    for (const selId of selectors) {
      const select = document.getElementById(selId);
      if (!select) continue;

      const currentVal = select.value;
      select.innerHTML = '<option value="">-- Select map --</option>';
      data.maps.forEach((m) => {
        const opt = document.createElement("option");
        opt.value = m.path;
        opt.textContent = m.name;
        select.appendChild(opt);
      });

      // Restore selection or auto-select last map
      if (currentVal) {
        select.value = currentVal;
      } else if (data.maps.length > 0) {
        select.value = data.maps[data.maps.length - 1].path;
      }
    }
  }

  async function saveMap() {
    const nameInput = document.getElementById("lm-map-name");
    const name = (nameInput && nameInput.value.trim()) || "map";

    _setStatusMsg("Saving map...");
    const result = await _apiPost("save_map", { name });

    if (result.ok) {
      _setStatusMsg(`Map saved: ${name}`);
      _lastMapCount = -1; // Force refresh
      _refreshMaps();
    } else {
      _setStatusMsg(`Map save failed: ${result.error}`);
    }
  }

  // ---- Helpers ------------------------------------------------------------

  function _setStatusMsg(msg) {
    const el = document.getElementById("status-message");
    if (el) el.textContent = msg;
  }

  /** Check if a process is currently running. */
  function isRunning(name) {
    return currentStatus[name] && currentStatus[name].running;
  }

  return { init, isRunning, saveMap };
})();
