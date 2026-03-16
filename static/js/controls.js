/**
 * Unitree Go2 Teleop Controls
 * Handles D-pad, keyboard, and emergency stop.
 * Publishes velocity via Sport API Move (api_id 1008) for AI mode,
 * and also via /cmd_vel as fallback for non-AI modes.
 */

/* global RosBridge, ROSLIB */
/* exported Controls */

const Controls = (() => {
  let cmdVelTopic = null;
  let publishInterval = null;
  let currentTwist = { linear: 0, lateralY: 0, angular: 0 };
  let enabled = false;
  let estopActive = false;
  let estopBurstTimer = null;

  // Sport API constants
  const SPORT_API = "/api/sport/request";
  const SPORT_MSG_TYPE = "unitree_api/msg/Request";
  const API_DAMP          = 1001;
  const API_BALANCE_STAND = 1002;
  const API_STOP_MOVE     = 1003;
  const API_STAND_UP      = 1004;
  const API_STAND_DOWN    = 1005;
  const API_RECOVERY      = 1006;
  const API_MOVE          = 1008;

  function init() {
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
    // Publish at ~10 Hz while moving
    publishInterval = setInterval(() => {
      if (enabled && (currentTwist.linear !== 0 || currentTwist.lateralY !== 0 || currentTwist.angular !== 0)) {
        _publishVelocity(currentTwist.linear, currentTwist.lateralY, currentTwist.angular);
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

  function _setVel(lin, latY, ang) {
    if (!enabled) return;
    currentTwist = { linear: lin, lateralY: latY, angular: ang };
    _publishVelocity(lin, latY, ang);
  }

  function _stop() {
    currentTwist = { linear: 0, lateralY: 0, angular: 0 };
    _publishVelocity(0, 0, 0);
    // Also send StopMove via sport API
    _sportCmd(API_STOP_MOVE, "StopMove");
  }

  function _emergencyStop() {
    _stop();
    estopActive = true;

    // Burst zero-velocity commands (10 msgs over 1 second)
    let burstCount = 0;
    if (estopBurstTimer) clearInterval(estopBurstTimer);
    estopBurstTimer = setInterval(() => {
      _publishVelocity(0, 0, 0);
      burstCount++;
      if (burstCount >= 10) {
        clearInterval(estopBurstTimer);
        estopBurstTimer = null;
        estopActive = false;
      }
    }, 100);

    // Send Damp via sport API (immediate motor soft-stop)
    _sportCmd(API_DAMP, "Damp");

    // Cancel Nav2 navigation
    try {
      const actionClient = RosBridge.actionClient(
        "/navigate_to_pose",
        "nav2_msgs/action/NavigateToPose"
      );
      actionClient.cancel();
    } catch (_) {}

    // Visual feedback
    const btn = document.getElementById("btn-emergency-stop");
    btn.classList.add("estop-flash");
    setTimeout(() => btn.classList.remove("estop-flash"), 1500);

    document.getElementById("status-message").textContent = "EMERGENCY STOP activated";
  }

  /**
   * Publish velocity via both Sport API Move (api_id 1008) and /cmd_vel.
   * Sport API Move is required in AI mode; /cmd_vel is kept as fallback.
   */
  function _publishVelocity(vx, vy, vyaw) {
    // 1. Sport API Move command (works in AI mode)
    _sportMove(vx, vy, vyaw);

    // 2. cmd_vel fallback (works in non-AI / nav2 mode)
    if (cmdVelTopic) {
      cmdVelTopic.publish(new ROSLIB.Message({
        linear:  { x: vx, y: vy, z: 0 },
        angular: { x: 0,  y: 0,  z: vyaw },
      }));
    }
  }

  // ---- Sport API --------------------------------------------------------

  function _sportMove(vx, vy, vyaw) {
    const msg = _buildSportMsg(API_MOVE, { x: vx, y: vy, rx: vyaw });
    RosBridge.publish(SPORT_API, SPORT_MSG_TYPE, msg);
  }

  function _sportCmd(apiId, label) {
    const msg = _buildSportMsg(apiId, {});
    RosBridge.publish(SPORT_API, SPORT_MSG_TYPE, msg);
    document.getElementById("status-message").textContent = "Sport: " + label;
    console.log("[Controls] Sport command:", label, "api_id:", apiId);
  }

  function _buildSportMsg(apiId, params) {
    return {
      header: { identity: { id: Date.now() % 2147483647, api_id: apiId } },
      parameter: JSON.stringify(params),
      api_id: apiId,
    };
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
