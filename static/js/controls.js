/**
 * Unitree Go2 Teleop Controls
 * Handles D-pad, keyboard, and emergency stop.
 *
 * Movement: sends velocity via SocketIO → backend → ros2 topic pub at 10Hz
 *           (Sport API Move, api_id 1008 — required for Go2 AI mode)
 * Also publishes /cmd_vel via rosbridge as fallback for nav2 mode.
 *
 * Sport commands (stand, lie down, etc.): via SocketIO → backend → ros2 topic pub --once
 */

/* global RosBridge, ROSLIB, io */
/* exported Controls */

const Controls = (() => {
  let cmdVelTopic = null;
  let publishInterval = null;
  let currentVel = { vx: 0, vy: 0, vyaw: 0 };
  let enabled = false;
  let estopActive = false;
  let estopBurstTimer = null;
  let socket = null;

  // Sport API IDs
  const API_DAMP      = 1001;
  const API_STOP_MOVE = 1003;
  const API_STAND_UP  = 1004;
  const API_STAND_DOWN = 1005;
  const API_RECOVERY  = 1006;

  function init() {
    // Connect SocketIO for sport commands
    socket = io();
    socket.on("connect", () => console.log("[Controls] SocketIO connected, id:", socket.id));
    socket.on("connect_error", (err) => console.error("[Controls] SocketIO connect error:", err));
    socket.on("disconnect", (reason) => console.warn("[Controls] SocketIO disconnected:", reason));

    // D-pad button events (mouse + touch)
    _bindBtn("btn-forward",  () => _setVel( _linSpeed(), 0, 0));
    _bindBtn("btn-backward", () => _setVel(-_linSpeed(), 0, 0));
    _bindBtn("btn-left",     () => _setVel(0, 0,  _angSpeed()));
    _bindBtn("btn-right",    () => _setVel(0, 0, -_angSpeed()));
    _bindBtn("btn-stop",     () => _stop());

    // Emergency stop
    document.getElementById("btn-emergency-stop").addEventListener("click", _emergencyStop);

    // Unitree Go2 Sport Mode buttons
    document.getElementById("btn-stand-up").addEventListener("click", () => _sportCmd(API_STAND_UP, "StandUp"));
    document.getElementById("btn-stand-down").addEventListener("click", () => _sportCmd(API_STAND_DOWN, "StandDown"));
    document.getElementById("btn-recovery-stand").addEventListener("click", () => _sportCmd(API_RECOVERY, "RecoveryStand"));
    document.getElementById("btn-damp").addEventListener("click", () => _sportCmd(API_DAMP, "Damp"));

    // Speed slider labels
    document.getElementById("linear-speed").addEventListener("input", (e) => {
      document.getElementById("linear-speed-val").textContent = parseFloat(e.target.value).toFixed(2);
    });
    document.getElementById("angular-speed").addEventListener("input", (e) => {
      document.getElementById("angular-speed-val").textContent = parseFloat(e.target.value).toFixed(2);
    });

    // Keyboard
    document.addEventListener("keydown", _onKeyDown);
    document.addEventListener("keyup", _onKeyUp);

    // Enable toggle (optional)
    const toggleEl = document.getElementById("toggle-enable");
    if (toggleEl) {
      toggleEl.addEventListener("change", (e) => {
        enabled = e.target.checked;
        if (!enabled) _stop();
      });
    } else {
      enabled = true;
    }
  }

  function start() {
    if (publishInterval) { clearInterval(publishInterval); publishInterval = null; }
    cmdVelTopic = RosBridge.advertise("/cmd_vel", "geometry_msgs/msg/Twist");

    // Publish cmd_vel at ~10Hz while moving (fallback for nav2 mode)
    publishInterval = setInterval(() => {
      if (enabled && (currentVel.vx !== 0 || currentVel.vy !== 0 || currentVel.vyaw !== 0)) {
        if (cmdVelTopic) {
          cmdVelTopic.publish(new ROSLIB.Message({
            linear:  { x: currentVel.vx, y: currentVel.vy, z: 0 },
            angular: { x: 0, y: 0, z: currentVel.vyaw },
          }));
        }
      }
    }, 100);
  }

  function stop() {
    _stop();
    if (publishInterval) clearInterval(publishInterval);
  }

  // ---- Internal ---------------------------------------------------------

  function _linSpeed() { return parseFloat(document.getElementById("linear-speed").value); }
  function _angSpeed() { return parseFloat(document.getElementById("angular-speed").value); }

  function _setVel(vx, vy, vyaw) {
    if (!enabled) return;
    currentVel = { vx, vy, vyaw };

    // Send to backend via SocketIO → ros2 topic pub at 10Hz (Sport API)
    if (socket) socket.emit("sport_move", currentVel);
  }

  function _stop() {
    currentVel = { vx: 0, vy: 0, vyaw: 0 };

    // Stop the persistent velocity publisher on backend
    if (socket) socket.emit("sport_move", { vx: 0, vy: 0, vyaw: 0 });

    // Also stop via cmd_vel
    if (cmdVelTopic) {
      cmdVelTopic.publish(new ROSLIB.Message({
        linear:  { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      }));
    }
  }

  function _emergencyStop() {
    _stop();
    estopActive = true;

    // Send Damp via sport API (immediate motor soft-stop)
    _sportCmd(API_DAMP, "EMERGENCY STOP");

    // Burst zero cmd_vel (10 msgs over 1 second)
    let burstCount = 0;
    if (estopBurstTimer) clearInterval(estopBurstTimer);
    estopBurstTimer = setInterval(() => {
      if (cmdVelTopic) {
        cmdVelTopic.publish(new ROSLIB.Message({
          linear:  { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        }));
      }
      burstCount++;
      if (burstCount >= 10) {
        clearInterval(estopBurstTimer);
        estopBurstTimer = null;
        estopActive = false;
      }
    }, 100);

    // Cancel Nav2 navigation
    try {
      const actionClient = RosBridge.actionClient(
        "/navigate_to_pose", "nav2_msgs/action/NavigateToPose"
      );
      actionClient.cancel();
    } catch (_) {}

    // Visual feedback
    const btn = document.getElementById("btn-emergency-stop");
    btn.classList.add("estop-flash");
    setTimeout(() => btn.classList.remove("estop-flash"), 1500);

    document.getElementById("status-message").textContent = "EMERGENCY STOP activated";
  }

  // ---- Sport Commands (one-shot) ----------------------------------------

  function _sportCmd(apiId, label) {
    if (socket) {
      socket.emit("sport_cmd", { api_id: apiId, label: label });
    }
    document.getElementById("status-message").textContent = "Sport: " + label;
    console.log("[Controls] Sport command:", label, "api_id:", apiId);
  }

  // ---- Button helpers ---------------------------------------------------

  function _bindBtn(id, onPress) {
    const btn = document.getElementById(id);
    // Mouse
    btn.addEventListener("mousedown", (e) => { e.preventDefault(); btn.classList.add("pressed"); onPress(); });
    btn.addEventListener("mouseup",   () => { btn.classList.remove("pressed"); _stop(); });
    btn.addEventListener("mouseleave",() => { btn.classList.remove("pressed"); _stop(); });
    // Touch
    btn.addEventListener("touchstart", (e) => { e.preventDefault(); btn.classList.add("pressed"); onPress(); });
    btn.addEventListener("touchend",   () => { btn.classList.remove("pressed"); _stop(); });
  }

  // ---- Keyboard ---------------------------------------------------------

  const keyState = {};

  function _onKeyDown(e) {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    if (keyState[e.key]) return;
    keyState[e.key] = true;

    switch (e.key) {
      case "ArrowUp":    case "w": _setVel( _linSpeed(), 0, 0); break;
      case "ArrowDown":  case "s": _setVel(-_linSpeed(), 0, 0); break;
      case "ArrowLeft":  case "a": _setVel(0, 0,  _angSpeed()); break;
      case "ArrowRight": case "d": _setVel(0, 0, -_angSpeed()); break;
      case " ": _emergencyStop(); break;
    }
  }

  function _onKeyUp(e) {
    delete keyState[e.key];
    if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","w","a","s","d"].includes(e.key)) {
      _stop();
    }
  }

  return { init, start, stop };
})();
