#!/usr/bin/env python3
"""Unitree Go2 Web Control Interface - Flask Application Server.

Serves the web UI and proxies the camera feed from ROS.
The frontend connects directly to rosbridge_websocket for
all ROS topic/service communication.

Includes a Launch Manager for starting/stopping ROS2 processes
(bringup, SLAM/FAST-LIO2, navigation, rosbridge) from the web UI.
"""

import os
import signal
import subprocess
import threading
import glob as globmod
from pathlib import Path
from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS

app = Flask(__name__)
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0  # disable static file caching
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

# ---------------------------------------------------------------------------
# Configuration – override with environment variables
# ---------------------------------------------------------------------------
ROSBRIDGE_HOST = os.environ.get("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", 9090))
WEB_PORT = int(os.environ.get("WEB_PORT", 5000))
ROBOT_MODEL = os.environ.get("ROBOT_MODEL", "go2")
MAP_SAVE_DIR = os.environ.get("MAP_SAVE_DIR", os.path.expanduser("~/maps"))
SSH_PASSWORD = os.environ.get("SSH_PASSWORD", "123")  # Go2 default password

# ---------------------------------------------------------------------------
# Process Manager — launch / stop ROS2 processes from the web UI
# ---------------------------------------------------------------------------
_processes = {}   # name -> { "proc": Popen, "cmd": str, "log": list }
_proc_lock = threading.Lock()


ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "")
RMW_IMPLEMENTATION = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

# CycloneDDS config – use eth0 on the Go2 (Jetson) network
CYCLONEDDS_URI = os.environ.get("CYCLONEDDS_URI",
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="eth0" priority="default" multicast="default" />'
    '</Interfaces></General></Domain></CycloneDDS>'
)


def _build_env():
    """Build environment with Go2 ROS2 settings (CycloneDDS, etc.)."""
    env = os.environ.copy()
    env["ROBOT_MODEL"] = ROBOT_MODEL
    env["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    env["CYCLONEDDS_URI"] = CYCLONEDDS_URI
    if ROS_DOMAIN_ID:
        env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    return env


# Map of process names to their launch commands
LAUNCH_COMMANDS = {
    "rosbridge": [
        "ros2", "launch", "rosbridge_server",
        "rosbridge_websocket_launch.xml",
    ],
    "bringup": [
        "ros2", "launch", "go2_bringup", "bringup.launch",
    ],
    "slam": [
        "ros2", "launch", "fast_lio", "mapping.launch.py",
        "rviz:=false",
    ],
    "navigation": [
        "ros2", "launch", "go2_navigation", "navigation2.launch.py",
        "use_rviz:=false",
        # map:= argument is appended dynamically
    ],
}


# Map process names to executables that should be killed before re-launch
_KILL_PATTERNS = {
    "rosbridge": ["rosbridge_websocket", "rosapi"],
    "slam": ["fastlio_mapping", "fast_lio", "pointcloud_to_laserscan"],
    "navigation": ["bt_navigator", "controller_server", "planner_server",
                    "behavior_server", "lifecycle_manager_navigation"],
}


def _kill_stale(name):
    """Kill any leftover OS processes from a previous launch of this name."""
    patterns = _KILL_PATTERNS.get(name, [])
    for pat in patterns:
        try:
            subprocess.run(
                ["pkill", "-f", pat],
                timeout=5, capture_output=True,
            )
        except Exception:
            pass


def _launch_process(name, extra_args=None, ssh_host=None, ssh_password=None):
    """Launch a ROS2 process by name. Returns (ok, message).

    If ssh_host is provided (e.g. 'unitree@192.168.123.18'), the command
    is executed on the remote machine via SSH using sshpass for password auth.
    """
    with _proc_lock:
        existing = _processes.get(name)
        if existing and existing["proc"].poll() is None:
            return False, f"{name} is already running"

    # Kill any stale processes from a previous session
    _kill_stale(name)

    cmd = list(LAUNCH_COMMANDS.get(name, []))
    if not cmd:
        return False, f"Unknown process: {name}"

    if extra_args:
        cmd.extend(extra_args)

    # Wrap with SSH if a remote host is specified
    if ssh_host:
        remote_cmd = " ".join(cmd)
        # Source ROS2 environment on the remote machine since SSH
        # non-interactive shells don't load .bashrc
        wrapped = (
            "export PATH=/usr/local/cuda-11.4/bin:$PATH; "
            "export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH; "
            "export CPATH=/usr/local/cuda/include:$CPATH; "
            "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
            "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 2>/dev/null || true; "
            "source ~/3d_ws/install/setup.bash 2>/dev/null; "
            "source ~/go2_ws/install/setup.bash 2>/dev/null; "
            f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; "
            f"export CYCLONEDDS_URI='{CYCLONEDDS_URI}'; "
            + (f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; " if ROS_DOMAIN_ID else "")
            + f"{remote_cmd}"
        )
        password = ssh_password or SSH_PASSWORD
        cmd = ["sshpass", "-p", password,
               "ssh", "-tt", "-o", "StrictHostKeyChecking=no",
               ssh_host, wrapped]

    try:
        env = _build_env()
        proc = subprocess.Popen(
            cmd, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,  # new process group for clean kill
        )
        log_lines = []

        # Background thread to capture output
        def _reader():
            try:
                for line in iter(proc.stdout.readline, b""):
                    decoded = line.decode("utf-8", errors="replace").rstrip()
                    log_lines.append(decoded)
                    if len(log_lines) > 200:
                        log_lines.pop(0)
            except Exception:
                pass

        t = threading.Thread(target=_reader, daemon=True)
        t.start()

        with _proc_lock:
            _processes[name] = {"proc": proc, "cmd": " ".join(cmd), "log": log_lines}

        return True, f"{name} launched (PID {proc.pid})"

    except FileNotFoundError:
        return False, f"Command not found: {cmd[0]}"
    except Exception as e:
        return False, str(e)


def _stop_process(name):
    """Stop a running process. Returns (ok, message)."""
    with _proc_lock:
        info = _processes.get(name)
        if not info or info["proc"].poll() is not None:
            _processes.pop(name, None)
            return False, f"{name} is not running"

    try:
        # Kill the entire process group
        os.killpg(os.getpgid(info["proc"].pid), signal.SIGTERM)
        info["proc"].wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(info["proc"].pid), signal.SIGKILL)
        except Exception:
            pass

    # Don't remove from _processes — keep the log accessible

    return True, f"{name} stopped"


def _process_status():
    """Get status of all managed processes."""
    status = {}
    with _proc_lock:
        for name, info in list(_processes.items()):
            running = info["proc"].poll() is None
            status[name] = {
                "running": running,
                "pid": info["proc"].pid,
                "cmd": info["cmd"],
                "log": info["log"][-20:],  # last 20 lines
            }
            # Keep stopped processes so their logs remain accessible

    # Also report processes we know about but aren't running
    for name in LAUNCH_COMMANDS:
        if name not in status:
            status[name] = {"running": False, "pid": None, "cmd": "", "log": []}

    return status


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.route("/")
def index():
    """Serve the main control panel page."""
    return render_template(
        "index.html",
        rosbridge_host=ROSBRIDGE_HOST,
        rosbridge_port=ROSBRIDGE_PORT,
        robot_model=ROBOT_MODEL,
        ros_domain_id=ROS_DOMAIN_ID,
    )


@app.route("/api/config")
def config():
    """Return current server configuration."""
    return jsonify({
        "rosbridge_host": ROSBRIDGE_HOST,
        "rosbridge_port": ROSBRIDGE_PORT,
        "robot_model": ROBOT_MODEL,
        "map_save_dir": MAP_SAVE_DIR,
        "ros_domain_id": ROS_DOMAIN_ID,
    })


# ---------------------------------------------------------------------------
# Launch Manager API
# ---------------------------------------------------------------------------

@app.route("/api/processes", methods=["GET"])
def get_processes():
    """Get status of all managed processes."""
    return jsonify(_process_status())


@app.route("/api/launch", methods=["POST"])
def launch_process():
    """Launch a ROS2 process."""
    data = request.get_json(silent=True) or {}
    name = data.get("name", "")
    extra_args = data.get("args", [])
    ssh_host = data.get("ssh_host")
    ssh_password = data.get("ssh_password")

    if not name:
        return jsonify({"ok": False, "error": "Missing process name"}), 400

    ok, msg = _launch_process(name, extra_args, ssh_host, ssh_password)
    return jsonify({"ok": ok, "message": msg}), 200 if ok else 409


@app.route("/api/stop", methods=["POST"])
def stop_process():
    """Stop a running process."""
    data = request.get_json(silent=True) or {}
    name = data.get("name", "")

    if not name:
        return jsonify({"ok": False, "error": "Missing process name"}), 400

    ok, msg = _stop_process(name)
    return jsonify({"ok": ok, "message": msg}), 200 if ok else 409


@app.route("/api/ros2/topics", methods=["GET"])
def ros2_topics():
    """Run 'ros2 topic list' and return the available topics."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=10,
            env=_build_env(),
        )
        topics = [t.strip() for t in result.stdout.strip().split("\n") if t.strip()]
        return jsonify({"ok": True, "topics": topics})
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return jsonify({"ok": False, "topics": [], "error": str(e)})


@app.route("/api/process_log/<name>", methods=["GET"])
def process_log(name):
    """Get recent log output for a process."""
    with _proc_lock:
        info = _processes.get(name)
        if not info:
            return jsonify({"log": [], "running": False})
        return jsonify({
            "log": info["log"][-50:],
            "running": info["proc"].poll() is None,
        })


# ---------------------------------------------------------------------------
# Map management
# ---------------------------------------------------------------------------

@app.route("/api/maps", methods=["GET"])
def list_maps():
    """List saved map files."""
    Path(MAP_SAVE_DIR).mkdir(parents=True, exist_ok=True)
    maps = []
    for f in sorted(globmod.glob(os.path.join(MAP_SAVE_DIR, "*.yaml"))):
        name = os.path.splitext(os.path.basename(f))[0]
        maps.append({"name": name, "path": f})
    return jsonify({"maps": maps, "dir": MAP_SAVE_DIR})


@app.route("/api/save_map", methods=["POST"])
def save_map():
    """Save the current map using ros2 map_saver_cli."""
    data = request.get_json(silent=True) or {}
    name = data.get("name", "map")

    # Sanitise filename
    safe_name = "".join(c for c in name if c.isalnum() or c in "-_")
    if not safe_name:
        safe_name = "map"

    Path(MAP_SAVE_DIR).mkdir(parents=True, exist_ok=True)
    filepath = os.path.join(MAP_SAVE_DIR, safe_name)

    # Try map_saver_cli (Nav2)
    try:
        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli",
             "-f", filepath, "--ros-args", "-p", "save_map_timeout:=10.0"],
            capture_output=True, text=True, timeout=30,
        )
        if result.returncode == 0:
            return jsonify({"ok": True, "path": filepath, "method": "map_saver_cli"})
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Fallback: try saving via ros2 service call to slam_toolbox
    try:
        result = subprocess.run(
            ["ros2", "service", "call",
             "/slam_toolbox/save_map",
             "slam_toolbox/srv/SaveMap",
             f'{{"name": {{"data": "{filepath}"}}}}'
             ],
            capture_output=True, text=True, timeout=15,
        )
        if result.returncode == 0:
            return jsonify({"ok": True, "path": filepath, "method": "slam_toolbox"})
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return jsonify({"ok": False, "error": "No map saver available. Ensure nav2_map_server or slam_toolbox is running."}), 500


# ---------------------------------------------------------------------------
# Camera stream proxy (optional – used when rosbridge video is unavailable)
# ---------------------------------------------------------------------------
camera_frame = None
camera_lock = threading.Lock()


@app.route("/api/camera")
def camera_feed():
    """MJPEG stream proxy for the robot camera."""

    def generate():
        while True:
            with camera_lock:
                frame = camera_frame
            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            socketio.sleep(0.05)

    return Response(
        generate(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ---------------------------------------------------------------------------
# Unitree Go2 Sport API – sends commands via SSH → ros2 topic pub on Jetson
# ---------------------------------------------------------------------------

# SSH target for the Go2 Jetson — set from the web UI or default
_sport_ssh_host = os.environ.get("GO2_SSH_HOST", "unitree@192.168.123.18")
_sport_msg_type = None          # cached message type
_sport_move_proc = None         # persistent ssh+ros2 topic pub for velocity
_sport_move_lock = threading.Lock()

# ROS2 env setup to source on the Jetson via SSH (same as bringup)
_JETSON_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
    "source ~/go2_ws/install/setup.bash 2>/dev/null; "
    f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; "
    f"export CYCLONEDDS_URI='{CYCLONEDDS_URI}'; "
    + (f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; " if ROS_DOMAIN_ID else "")
)


def _ssh_cmd(remote_cmd, timeout=8):
    """Run a command on the Jetson via SSH. Returns (returncode, stdout, stderr)."""
    wrapped = _JETSON_ROS_SETUP + remote_cmd
    cmd = [
        "sshpass", "-p", SSH_PASSWORD,
        "ssh", "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=3",
        _sport_ssh_host, wrapped,
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"
    except Exception as e:
        return -1, "", str(e)


def _ssh_popen(remote_cmd):
    """Start a persistent command on the Jetson via SSH. Returns Popen."""
    wrapped = _JETSON_ROS_SETUP + remote_cmd
    cmd = [
        "sshpass", "-p", SSH_PASSWORD,
        "ssh", "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=3",
        _sport_ssh_host, wrapped,
    ]
    return subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def _resolve_sport_type():
    """Discover the message type of /api/sport/request on the Jetson."""
    global _sport_msg_type
    if _sport_msg_type:
        return _sport_msg_type
    rc, stdout, stderr = _ssh_cmd("ros2 topic info /api/sport/request")
    for line in stdout.splitlines():
        if "Type:" in line:
            _sport_msg_type = line.split("Type:")[-1].strip()
            print(f"[Sport] Resolved topic type: {_sport_msg_type}")
            return _sport_msg_type
    print(f"[Sport] Could not resolve type (rc={rc}): stdout={stdout} stderr={stderr}")
    return None


def _sport_yaml(api_id, params):
    """Build YAML message string for ros2 topic pub.

    unitree_api/msg/Request structure:
      RequestHeader header
        RequestIdentity identity (int64 id, int64 api_id)
        RequestLease lease (int64 id)
        RequestPolicy policy (int32 priority, bool noreply)
      string parameter
      uint8[] binary
    """
    import json
    param_str = json.dumps(params) if isinstance(params, dict) else str(params)
    # Match the exact format that works when run manually on the Jetson.
    # Single-quote the whole YAML so the inner single-quotes around parameter
    # are escaped with the '\'' idiom (end sq, escaped sq, start sq).
    escaped_param = param_str.replace("'", "'\\''")
    return (
        "'{header: {identity: {id: 0, api_id: " + str(api_id) + "}}, "
        "parameter: '\"'\"'" + escaped_param + "'\"'\"', "
        "binary: []}'"
    )


def _sport_pub_once(api_id, params):
    """Publish a single sport command on the Jetson."""
    msg_type = _resolve_sport_type()
    if not msg_type:
        return False, "Cannot resolve /api/sport/request type"
    yaml_msg = _sport_yaml(api_id, params)
    # Use --once (matches the manual command that works on the Jetson).
    # Wrap with 'timeout 5' in case --once hangs waiting for discovery.
    remote = (f"timeout 5 ros2 topic pub --once "
              f"/api/sport/request {msg_type} {yaml_msg}")
    rc, stdout, stderr = _ssh_cmd(remote, timeout=8)
    # 'timeout' returns 124 when it kills the child – that's expected/success
    if rc in (0, 124):
        return True, msg_type
    return False, stderr or stdout


def _sport_move_start(vx, vy, vyaw):
    """Start a persistent velocity publisher on the Jetson at 10Hz."""
    global _sport_move_proc
    msg_type = _resolve_sport_type()
    if not msg_type:
        return

    yaml_msg = _sport_yaml(1008, {"x": vx, "y": vy, "rx": vyaw})
    remote = f"ros2 topic pub --rate 10 /api/sport/request {msg_type} {yaml_msg}"

    with _sport_move_lock:
        _kill_sport_move_proc()
        try:
            _sport_move_proc = _ssh_popen(remote)
            print(f"[Sport] Move started: vx={vx} vy={vy} vyaw={vyaw}")
        except Exception as e:
            print(f"[Sport] Move start failed: {e}")


def _kill_sport_move_proc():
    """Kill the persistent velocity SSH process."""
    global _sport_move_proc
    if _sport_move_proc and _sport_move_proc.poll() is None:
        try:
            os.killpg(os.getpgid(_sport_move_proc.pid), signal.SIGTERM)
            _sport_move_proc.wait(timeout=2)
        except Exception:
            try:
                os.killpg(os.getpgid(_sport_move_proc.pid), signal.SIGKILL)
            except Exception:
                pass
    _sport_move_proc = None


def _sport_move_stop():
    """Stop the persistent velocity publisher."""
    with _sport_move_lock:
        _kill_sport_move_proc()


@app.route("/api/sport", methods=["POST"])
def sport_command():
    """Send a one-shot sport mode command to the Go2."""
    data = request.get_json(force=True)
    api_id = data.get("api_id")
    parameter = data.get("parameter", {})

    if api_id is None:
        return jsonify({"ok": False, "error": "api_id required"}), 400

    # Update SSH host from request if provided
    global _sport_ssh_host
    if data.get("ssh_host"):
        _sport_ssh_host = data["ssh_host"]

    _sport_move_stop()

    ok, detail = _sport_pub_once(api_id, parameter)
    label = data.get("label", str(api_id))
    print(f"[Sport] {label} (api_id={api_id}): {'OK' if ok else detail}")
    if ok:
        return jsonify({"ok": True, "api_id": api_id, "type": detail})
    return jsonify({"ok": False, "error": detail}), 500


@app.route("/api/sport/move", methods=["POST"])
def sport_move():
    """Start/stop persistent velocity publishing for teleop."""
    data = request.get_json(force=True)
    vx = float(data.get("vx", 0))
    vy = float(data.get("vy", 0))
    vyaw = float(data.get("vyaw", 0))

    # Update SSH host from request if provided
    global _sport_ssh_host
    if data.get("ssh_host"):
        _sport_ssh_host = data["ssh_host"]

    if vx == 0 and vy == 0 and vyaw == 0:
        _sport_move_stop()
        _sport_pub_once(1003, {})  # StopMove
        return jsonify({"ok": True, "action": "stop"})
    else:
        _sport_move_start(vx, vy, vyaw)
        return jsonify({"ok": True, "action": "move", "vx": vx, "vy": vy, "vyaw": vyaw})


@app.route("/api/sport/topic_type")
def sport_topic_type():
    """Return the resolved message type for /api/sport/request."""
    msg_type = _resolve_sport_type()
    return jsonify({"ok": bool(msg_type), "type": msg_type})


@app.route("/api/sport/debug")
def sport_debug():
    """Debug: show message type definition and test publish on Jetson."""
    msg_type = _resolve_sport_type()
    info = {"type": msg_type, "ssh_host": _sport_ssh_host}

    if msg_type:
        # Get the message definition
        rc, stdout, stderr = _ssh_cmd(f"ros2 interface show {msg_type}")
        info["definition"] = stdout
        info["def_stderr"] = stderr

        # Show what we're publishing
        info["yaml_example"] = _sport_yaml(1004, {})

        # Test publish
        yaml_msg = _sport_yaml(1004, {})
        remote = (f"timeout 5 ros2 topic pub --once "
                  f"/api/sport/request {msg_type} {yaml_msg}")
        rc, stdout, stderr = _ssh_cmd(remote, timeout=8)
        info["pub_rc"] = rc
        info["pub_stdout"] = stdout
        info["pub_stderr"] = stderr
        info["pub_cmd"] = remote

    return jsonify(info)


# ---------------------------------------------------------------------------
# SocketIO events (thin relay – most comms go through roslibjs directly)
# ---------------------------------------------------------------------------

@socketio.on("connect")
def handle_connect():
    print("[WebUI] Client connected")


@socketio.on("disconnect")
def handle_disconnect():
    _sport_move_stop()
    print("[WebUI] Client disconnected")


# ---------------------------------------------------------------------------
# Entry-point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"[WebUI] Starting on http://0.0.0.0:{WEB_PORT}")
    print(f"[WebUI] Expecting rosbridge at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    print(f"[WebUI] Robot model: {ROBOT_MODEL}")
    print(f"[WebUI] Map save dir: {MAP_SAVE_DIR}")
    socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=True)
