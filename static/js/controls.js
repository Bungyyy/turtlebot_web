/**
 * Robot Teleop Controls
 * Handles D-pad, keyboard, and emergency stop.
 * Publishes geometry_msgs/Twist to /cmd_vel.
 */

/* global RosBridge */
/* exported Controls */

const Controls = (() => {
  let cmdVelTopic = null;
  let publishInterval = null;
  let currentTwist = { linear: 0, angular: 0 };
  let enabled = false;

  function init() {
    // D-pad button events (mouse + touch)
    _bindBtn("btn-forward",  () => _setVel( _linSpeed(), 0));
    _bindBtn("btn-backward", () => _setVel(-_linSpeed(), 0));
    _bindBtn("btn-left",     () => _setVel(0,  _angSpeed()));
    _bindBtn("btn-right",    () => _setVel(0, -_angSpeed()));
    _bindBtn("btn-stop",     () => _stop());

    // Emergency stop
    document.getElementById("btn-emergency-stop").addEventListener("click", _emergencyStop);

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

    // Enable toggle (optional — may not exist in simplified UI)
    const toggleEl = document.getElementById("toggle-enable");
    if (toggleEl) {
      toggleEl.addEventListener("change", (e) => {
        enabled = e.target.checked;
        if (!enabled) _stop();
      });
    } else {
      enabled = true; // Always enabled when no toggle exists
    }
  }

  function start() {
    // Clean up previous publisher/interval (handles reconnect)
    if (publishInterval) { clearInterval(publishInterval); publishInterval = null; }
    cmdVelTopic = RosBridge.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
    // Publish at ~10 Hz while moving
    publishInterval = setInterval(() => {
      if (enabled && (currentTwist.linear !== 0 || currentTwist.angular !== 0)) {
        _publishTwist(currentTwist.linear, currentTwist.angular);
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

  function _setVel(lin, ang) {
    if (!enabled) return;
    currentTwist = { linear: lin, angular: ang };
    _publishTwist(lin, ang);
  }

  function _stop() {
    currentTwist = { linear: 0, angular: 0 };
    _publishTwist(0, 0);
  }

  function _emergencyStop() {
    _stop();
    // Cancel Nav2 navigation by publishing empty goal to stop
    RosBridge.publish(
      "/navigate_to_pose/_action/cancel_goal",
      "action_msgs/srv/CancelGoal_Request",
      {}
    );
    document.getElementById("status-message").textContent = "EMERGENCY STOP activated";
  }

  function _publishTwist(linear, angular) {
    if (!cmdVelTopic) return;
    cmdVelTopic.publish(new ROSLIB.Message({
      linear:  { x: linear,  y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    }));
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
    if (keyState[e.key]) return; // prevent repeat
    keyState[e.key] = true;

    switch (e.key) {
      case "ArrowUp":    case "w": _setVel( _linSpeed(), 0); break;
      case "ArrowDown":  case "s": _setVel(-_linSpeed(), 0); break;
      case "ArrowLeft":  case "a": _setVel(0,  _angSpeed()); break;
      case "ArrowRight": case "d": _setVel(0, -_angSpeed()); break;
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
