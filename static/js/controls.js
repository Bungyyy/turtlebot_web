/**
 * Unitree Go2 Teleop Controls — ROS Bridge First
 *
 * Primary:  Publish /cmd_vel directly via rosbridge WebSocket (zero latency)
 * Fallback: HTTP POST /api/sport/move → backend SSH relay
 *
 * Features:
 *   - D-pad with lateral (strafe) movement for quadruped
 *   - Virtual joystick (touch-friendly analog control)
 *   - Browser Gamepad API (PS4/Xbox controller)
 *   - Keyboard (WASD + QE for strafe)
 *   - Sport API commands via rosbridge (StandUp, StandDown, etc.)
 *   - Emergency stop with Damp
 */

/* global RosBridge, ROSLIB */
/* exported Controls */

const Controls = (() => {
  // -- State ------------------------------------------------------------------
  let cmdVelTopic = null;
  let sportPubTopic = null;
  let publishTimer = null;
  let currentVel = { vx: 0, vy: 0, vyaw: 0 };
  let enabled = true;
  let estopActive = false;
  let estopBurstTimer = null;
  let useRosBridge = true;          // true = publish via rosbridge, false = HTTP relay
  let sportMsgType = null;          // resolved sport API message type

  // Joystick state
  let joystickActive = false;
  let joystickVec = { vx: 0, vy: 0 };  // normalized -1..1

  // Gamepad state
  let gamepadIndex = null;
  let gamepadPollTimer = null;

  // Sport API IDs (Unitree Go2)
  const API_DAMP       = 1001;
  const API_STOP_MOVE  = 1003;
  const API_STAND_UP   = 1004;
  const API_STAND_DOWN = 1005;
  const API_RECOVERY   = 1006;
  const API_SIT        = 1008;
  const API_STRETCH    = 1021;
  const API_WAVE       = 1035;

  // ---------------------------------------------------------------------------
  // Init
  // ---------------------------------------------------------------------------

  function init() {
    // D-pad (8 directions: fwd, back, left, right + strafe left/right + diagonals)
    _bindBtn("btn-forward",      () => _setVel( _linSpeed(), 0, 0));
    _bindBtn("btn-backward",     () => _setVel(-_linSpeed(), 0, 0));
    _bindBtn("btn-left",         () => _setVel(0, 0,  _angSpeed()));
    _bindBtn("btn-right",        () => _setVel(0, 0, -_angSpeed()));
    _bindBtn("btn-strafe-left",  () => _setVel(0,  _linSpeed(), 0));
    _bindBtn("btn-strafe-right", () => _setVel(0, -_linSpeed(), 0));
    _bindBtn("btn-stop",         () => _stop());

    // Emergency stop
    document.getElementById("btn-emergency-stop").addEventListener("click", _emergencyStop);

    // Sport mode buttons
    document.getElementById("btn-stand-up").addEventListener("click", () => _sportCmd(API_STAND_UP, "StandUp"));
    document.getElementById("btn-stand-down").addEventListener("click", () => _sportCmd(API_STAND_DOWN, "StandDown"));
    document.getElementById("btn-recovery-stand").addEventListener("click", () => _sportCmd(API_RECOVERY, "RecoveryStand"));
    document.getElementById("btn-damp").addEventListener("click", () => _sportCmd(API_DAMP, "Damp"));

    // Extra sport commands (if buttons exist)
    const btnSit = document.getElementById("btn-sit");
    if (btnSit) btnSit.addEventListener("click", () => _sportCmd(API_SIT, "Sit"));
    const btnStretch = document.getElementById("btn-stretch");
    if (btnStretch) btnStretch.addEventListener("click", () => _sportCmd(API_STRETCH, "Stretch"));
    const btnWave = document.getElementById("btn-wave");
    if (btnWave) btnWave.addEventListener("click", () => _sportCmd(API_WAVE, "Wave"));

    // Speed sliders
    document.getElementById("linear-speed").addEventListener("input", (e) => {
      document.getElementById("linear-speed-val").textContent = parseFloat(e.target.value).toFixed(2);
    });
    document.getElementById("angular-speed").addEventListener("input", (e) => {
      document.getElementById("angular-speed-val").textContent = parseFloat(e.target.value).toFixed(2);
    });

    // Teleop mode toggle (rosbridge vs HTTP relay)
    const modeToggle = document.getElementById("teleop-mode-toggle");
    if (modeToggle) {
      modeToggle.addEventListener("change", (e) => {
        useRosBridge = e.target.checked;
        _updateModeLabel();
      });
    }

    // Keyboard
    document.addEventListener("keydown", _onKeyDown);
    document.addEventListener("keyup", _onKeyUp);

    // Virtual Joystick
    _initJoystick();

    // Gamepad
    _initGamepad();

    // Warmup HTTP relay as fallback
    _warmupRelay();
  }

  // ---------------------------------------------------------------------------
  // Start / Stop (called on rosbridge connect / disconnect)
  // ---------------------------------------------------------------------------

  function start() {
    // Create persistent /cmd_vel publisher via rosbridge
    cmdVelTopic = RosBridge.advertise("/cmd_vel", "geometry_msgs/msg/Twist");

    // Start 10Hz publish loop (publishes current velocity continuously)
    if (publishTimer) clearInterval(publishTimer);
    publishTimer = setInterval(_publishCmdVel, 100);

    // Resolve sport API topic type
    _resolveSportTopicType();

    _updateModeLabel();
    console.log("[Controls] Started — rosbridge teleop active");
  }

  function stop() {
    _stop();
    if (publishTimer) { clearInterval(publishTimer); publishTimer = null; }
    cmdVelTopic = null;
    sportPubTopic = null;
  }

  // ---------------------------------------------------------------------------
  // Core: Publish /cmd_vel
  // ---------------------------------------------------------------------------

  function _publishCmdVel() {
    if (!enabled) return;
    const { vx, vy, vyaw } = currentVel;
    const isMoving = vx !== 0 || vy !== 0 || vyaw !== 0;

    // Only publish when there is actual movement — publishing zeros continuously
    // overrides nav2 commands and causes the robot to jitter.
    if (!isMoving) return;

    // Publish via rosbridge when connected
    if (cmdVelTopic && RosBridge.isConnected()) {
      cmdVelTopic.publish(new ROSLIB.Message({
        linear:  { x: vx, y: vy, z: 0 },
        angular: { x: 0, y: 0, z: vyaw },
      }));
    }

    // ALSO publish via HTTP relay (belt-and-suspenders)
    // The relay runs on the Jetson itself so DDS discovery is guaranteed.
    _httpRelaySend(vx, vy, vyaw);
  }

  // ---------------------------------------------------------------------------
  // Velocity control
  // ---------------------------------------------------------------------------

  function _linSpeed() { return parseFloat(document.getElementById("linear-speed").value); }
  function _angSpeed() { return parseFloat(document.getElementById("angular-speed").value); }
  function _sshHost() {
    const el = document.getElementById("ssh-host");
    return el ? el.value.trim() : "";
  }

  function _setVel(vx, vy, vyaw) {
    if (!enabled) return;
    currentVel = { vx, vy, vyaw };

    // Always send via HTTP relay immediately for guaranteed delivery.
    // The relay runs on the Jetson and has direct DDS access to the robot.
    // Rosbridge cmd_vel is also sent in the 10Hz timer as a secondary path.
    _httpRelaySend(vx, vy, vyaw);

    _updateVelDisplay();
  }

  function _stop() {
    currentVel = { vx: 0, vy: 0, vyaw: 0 };

    // Send stop via rosbridge
    if (cmdVelTopic && RosBridge.isConnected()) {
      cmdVelTopic.publish(new ROSLIB.Message({
        linear:  { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      }));
    }

    // Also send stop via HTTP relay
    _httpRelaySend(0, 0, 0);

    _updateVelDisplay();
  }

  function _updateVelDisplay() {
    const el = document.getElementById("vel-display");
    if (!el) return;
    const { vx, vy, vyaw } = currentVel;
    if (vx === 0 && vy === 0 && vyaw === 0) {
      el.textContent = "Stopped";
      el.className = "vel-display stopped";
    } else {
      el.textContent = `vx:${vx.toFixed(2)} vy:${vy.toFixed(2)} w:${vyaw.toFixed(2)}`;
      el.className = "vel-display moving";
    }
  }

  // ---------------------------------------------------------------------------
  // Sport API Commands — via rosbridge (primary) or HTTP (fallback)
  // ---------------------------------------------------------------------------

  function _resolveSportTopicType() {
    // Try to get the topic type from rosapi
    if (!RosBridge.isConnected()) return;
    RosBridge.callService("/rosapi/topic_type", "rosapi/srv/TopicType", { topic: "/api/sport/request" })
      .then((result) => {
        if (result.type) {
          sportMsgType = result.type;
          console.log("[Controls] Sport topic type resolved via rosbridge:", sportMsgType);
        }
      })
      .catch(() => {
        console.log("[Controls] /api/sport/request not available via rosbridge, using HTTP fallback");
      });
  }

  function _sportCmd(apiId, label) {
    document.getElementById("status-message").textContent = "Sport: " + label;
    console.log("[Controls] Sport command:", label, "api_id:", apiId);

    // Sport API commands ALWAYS use HTTP/SSH — these are critical one-shot
    // commands (StandUp, StandDown, Damp) that MUST be delivered reliably.
    // Rosbridge publisher needs DDS subscriber discovery time which causes
    // the first N messages to be lost, making it unreliable for one-shot use.
    _sportCmdHttp(apiId, label);
  }

  function _sportCmdHttp(apiId, label) {
    const body = { api_id: apiId, label: label };
    const host = _sshHost();
    if (host) body.ssh_host = host;
    const pw = document.getElementById("ssh-password");
    if (pw && pw.value) body.ssh_password = pw.value;
    _httpPost("/api/sport", body)
      .then((res) => {
        if (res.ok) console.log("[Controls] Sport OK:", label, "type:", res.type);
        else console.error("[Controls] Sport FAIL:", label, res.error);
      });
  }

  // ---------------------------------------------------------------------------
  // Emergency Stop
  // ---------------------------------------------------------------------------

  function _emergencyStop() {
    _stop();
    estopActive = true;

    // Send Damp via sport API
    _sportCmd(API_DAMP, "EMERGENCY STOP");

    // Burst zero cmd_vel (10 msgs over 1 second)
    let burstCount = 0;
    if (estopBurstTimer) clearInterval(estopBurstTimer);
    estopBurstTimer = setInterval(() => {
      if (cmdVelTopic && RosBridge.isConnected()) {
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

    // Cancel Nav2 goal handler directly (fastest path)
    try {
      const cancelPub = RosBridge.advertise("/nav2_goal_handler/cancel", "geometry_msgs/msg/PoseStamped");
      cancelPub.publish(new ROSLIB.Message({ header: { frame_id: "cancel" }, pose: {} }));
    } catch (_) {}
    // Also zero cmd_vel_nav (nav2's velocity input before velocity_smoother)
    try {
      const navVelPub = RosBridge.advertise("/cmd_vel_nav", "geometry_msgs/msg/Twist");
      for (let i = 0; i < 5; i++) {
        setTimeout(() => navVelPub.publish(new ROSLIB.Message({
          linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }
        })), i * 50);
      }
    } catch (_) {}
    // Cancel via MapViewer (clears UI state)
    if (typeof MapViewer !== "undefined") MapViewer.cancelNavigation();
    // Backend cancel as backup
    fetch("/api/nav2/cancel", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" }).catch(() => {});

    // Visual feedback
    const btn = document.getElementById("btn-emergency-stop");
    btn.classList.add("estop-flash");
    setTimeout(() => btn.classList.remove("estop-flash"), 1500);

    document.getElementById("status-message").textContent = "EMERGENCY STOP activated";
  }

  // ---------------------------------------------------------------------------
  // Virtual Joystick (touch-friendly analog control)
  // ---------------------------------------------------------------------------

  function _initJoystick() {
    const pad = document.getElementById("joystick-pad");
    if (!pad) return;

    const knob = document.getElementById("joystick-knob");
    let padRect = null;
    let maxRadius = 0;

    function _startDrag(cx, cy) {
      padRect = pad.getBoundingClientRect();
      maxRadius = padRect.width / 2 - 15;
      joystickActive = true;
      _moveDrag(cx, cy);
    }

    function _moveDrag(cx, cy) {
      if (!joystickActive || !padRect) return;
      const ox = cx - (padRect.left + padRect.width / 2);
      const oy = cy - (padRect.top + padRect.height / 2);
      const dist = Math.min(Math.sqrt(ox * ox + oy * oy), maxRadius);
      const angle = Math.atan2(oy, ox);
      const nx = (dist / maxRadius) * Math.cos(angle);
      const ny = (dist / maxRadius) * Math.sin(angle);

      // Position knob
      knob.style.transform = `translate(${nx * maxRadius}px, ${ny * maxRadius}px)`;

      // Map to velocity: X-axis = forward/back (inverted Y), Y-axis = angular (X)
      joystickVec.vx = -ny;  // up = forward
      joystickVec.vy = 0;
      const vyaw = -nx;      // left = turn left (positive angular.z)

      _setVel(
        joystickVec.vx * _linSpeed(),
        joystickVec.vy,
        vyaw * _angSpeed()
      );
    }

    function _endDrag() {
      joystickActive = false;
      joystickVec = { vx: 0, vy: 0 };
      knob.style.transform = "translate(0px, 0px)";
      _stop();
    }

    // Mouse events
    pad.addEventListener("mousedown", (e) => { e.preventDefault(); _startDrag(e.clientX, e.clientY); });
    document.addEventListener("mousemove", (e) => { if (joystickActive) _moveDrag(e.clientX, e.clientY); });
    document.addEventListener("mouseup", () => { if (joystickActive) _endDrag(); });

    // Touch events
    pad.addEventListener("touchstart", (e) => {
      e.preventDefault();
      const t = e.touches[0];
      _startDrag(t.clientX, t.clientY);
    });
    pad.addEventListener("touchmove", (e) => {
      e.preventDefault();
      if (joystickActive) {
        const t = e.touches[0];
        _moveDrag(t.clientX, t.clientY);
      }
    });
    pad.addEventListener("touchend", () => _endDrag());
    pad.addEventListener("touchcancel", () => _endDrag());
  }

  // ---------------------------------------------------------------------------
  // Gamepad API (PS4/Xbox)
  // ---------------------------------------------------------------------------

  function _initGamepad() {
    window.addEventListener("gamepadconnected", (e) => {
      gamepadIndex = e.gamepad.index;
      console.log("[Controls] Gamepad connected:", e.gamepad.id);
      document.getElementById("status-message").textContent = "Gamepad: " + e.gamepad.id;
      const indicator = document.getElementById("gamepad-indicator");
      if (indicator) { indicator.classList.add("connected"); indicator.textContent = "Gamepad: Connected"; }
      if (!gamepadPollTimer) {
        gamepadPollTimer = setInterval(_pollGamepad, 50); // 20Hz
      }
    });

    window.addEventListener("gamepaddisconnected", (e) => {
      if (e.gamepad.index === gamepadIndex) {
        gamepadIndex = null;
        _stop();
        const indicator = document.getElementById("gamepad-indicator");
        if (indicator) { indicator.classList.remove("connected"); indicator.textContent = "Gamepad: None"; }
        if (gamepadPollTimer) { clearInterval(gamepadPollTimer); gamepadPollTimer = null; }
        console.log("[Controls] Gamepad disconnected");
      }
    });
  }

  function _pollGamepad() {
    if (gamepadIndex === null) return;
    const gp = navigator.getGamepads()[gamepadIndex];
    if (!gp) return;

    // Left stick: forward/backward (axis 1) + lateral strafe (axis 0)
    // Right stick: turn (axis 2)
    const deadzone = 0.15;
    const lx = Math.abs(gp.axes[0]) > deadzone ? gp.axes[0] : 0;
    const ly = Math.abs(gp.axes[1]) > deadzone ? gp.axes[1] : 0;
    const rx = Math.abs(gp.axes[2]) > deadzone ? gp.axes[2] : 0;

    const vx = -ly * _linSpeed();     // push up = forward
    const vy = -lx * _linSpeed();     // push left = strafe left
    const vyaw = -rx * _angSpeed();   // push left = turn left

    if (vx !== currentVel.vx || vy !== currentVel.vy || vyaw !== currentVel.vyaw) {
      currentVel = { vx, vy, vyaw };
      if (!useRosBridge || !RosBridge.isConnected()) {
        _httpRelaySend(vx, vy, vyaw);
      }
      _updateVelDisplay();
    }

    // Buttons: A/Cross=StandUp, B/Circle=StandDown, X/Square=Recovery, Y/Triangle=Damp
    if (gp.buttons[0].pressed) _sportCmd(API_STAND_UP, "StandUp");
    if (gp.buttons[1].pressed) _sportCmd(API_STAND_DOWN, "StandDown");
    if (gp.buttons[2].pressed) _sportCmd(API_RECOVERY, "RecoveryStand");
    if (gp.buttons[3].pressed) _sportCmd(API_DAMP, "Damp");

    // Start button = Emergency Stop
    if (gp.buttons[9] && gp.buttons[9].pressed) _emergencyStop();
  }

  // ---------------------------------------------------------------------------
  // Keyboard — WASD + QE for strafe + Arrow keys
  // ---------------------------------------------------------------------------

  const keyState = {};

  function _onKeyDown(e) {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT" || e.target.tagName === "TEXTAREA") return;
    if (keyState[e.key]) return;
    keyState[e.key] = true;
    _updateKeyVelocity();

    if (e.key === " ") { e.preventDefault(); _emergencyStop(); }
  }

  function _onKeyUp(e) {
    delete keyState[e.key];
    const moveKeys = ["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","w","a","s","d","q","e"];
    if (moveKeys.includes(e.key)) {
      _updateKeyVelocity();
    }
  }

  function _updateKeyVelocity() {
    let vx = 0, vy = 0, vyaw = 0;
    const lin = _linSpeed();
    const ang = _angSpeed();

    // Forward/backward
    if (keyState["ArrowUp"] || keyState["w"]) vx += lin;
    if (keyState["ArrowDown"] || keyState["s"]) vx -= lin;

    // Turn
    if (keyState["ArrowLeft"] || keyState["a"]) vyaw += ang;
    if (keyState["ArrowRight"] || keyState["d"]) vyaw -= ang;

    // Strafe (Q/E keys)
    if (keyState["q"]) vy += lin;
    if (keyState["e"]) vy -= lin;

    _setVel(vx, vy, vyaw);
  }

  // ---------------------------------------------------------------------------
  // HTTP Relay (fallback when rosbridge is unavailable)
  // ---------------------------------------------------------------------------

  function _warmupRelay() {
    const body = {};
    const host = _sshHost();
    if (host) body.ssh_host = host;
    fetch("/api/teleop/warmup", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
      .then((r) => r.json())
      .then((res) => {
        if (res.ok) console.log("[Controls] HTTP relay warmed up (fallback ready)");
      })
      .catch(() => {});
  }

  let _lastRelaySend = 0;
  function _httpRelaySend(vx, vy, vyaw) {
    // Throttle to max 10Hz
    const now = Date.now();
    if (now - _lastRelaySend < 100 && !(vx === 0 && vy === 0 && vyaw === 0)) return;
    _lastRelaySend = now;

    const body = { vx, vy, vyaw };
    const host = _sshHost();
    if (host) body.ssh_host = host;
    _httpPost("/api/sport/move", body);
  }

  function _httpPost(url, body) {
    return fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
      .then((r) => r.json())
      .catch((err) => {
        console.error("[Controls] HTTP error:", url, err);
        return { ok: false, error: String(err) };
      });
  }

  // ---------------------------------------------------------------------------
  // Button helpers
  // ---------------------------------------------------------------------------

  function _bindBtn(id, onPress) {
    const btn = document.getElementById(id);
    if (!btn) return;
    btn.addEventListener("mousedown", (e) => { e.preventDefault(); btn.classList.add("pressed"); onPress(); });
    btn.addEventListener("mouseup",   () => { btn.classList.remove("pressed"); _stop(); });
    btn.addEventListener("mouseleave",() => { btn.classList.remove("pressed"); _stop(); });
    btn.addEventListener("touchstart", (e) => { e.preventDefault(); btn.classList.add("pressed"); onPress(); });
    btn.addEventListener("touchend",   () => { btn.classList.remove("pressed"); _stop(); });
  }

  function _updateModeLabel() {
    const el = document.getElementById("teleop-mode-label");
    if (!el) return;
    if (useRosBridge && RosBridge.isConnected()) {
      el.textContent = "ROS Bridge (direct)";
      el.className = "teleop-mode-label mode-rosbridge";
    } else {
      el.textContent = "HTTP Relay (SSH)";
      el.className = "teleop-mode-label mode-relay";
    }
  }

  // ---------------------------------------------------------------------------
  // Public API
  // ---------------------------------------------------------------------------

  function setEnabled(on) {
    enabled = !!on;
    if (!enabled) {
      _stop();
    }
    console.log(`[Controls] Teleop ${enabled ? "enabled" : "disabled"}`);
  }

  return { init, start, stop, setEnabled };
})();
