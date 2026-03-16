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

# ---------------------------------------------------------------------------
# Process Manager — launch / stop ROS2 processes from the web UI
# ---------------------------------------------------------------------------
_processes = {}   # name -> { "proc": Popen, "cmd": str, "log": list }
_proc_lock = threading.Lock()


ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "30")


def _build_env():
    """Build environment with ROBOT_MODEL and ROS_DOMAIN_ID set."""
    env = os.environ.copy()
    env["ROBOT_MODEL"] = ROBOT_MODEL
    env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    return env


# Map of process names to their launch commands
LAUNCH_COMMANDS = {
    "rosbridge": [
        "ros2", "launch", "rosbridge_server",
        "rosbridge_websocket_launch.xml",
    ],
    "bringup": [
        "ros2", "launch", "go2_bringup", "robot.launch.py",
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


def _launch_process(name, extra_args=None, ssh_host=None):
    """Launch a ROS2 process by name. Returns (ok, message).

    If ssh_host is provided (e.g. 'ubuntu@192.168.1.10'), the command
    is executed on the remote machine via SSH.
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
            "source /opt/ros/humble/setup.bash 2>/dev/null || "
            "source /opt/ros/foxy/setup.bash 2>/dev/null || true; "
            "source ~/go2_ws/install/setup.bash 2>/dev/null || "
            "source ~/unitree_ws/install/setup.bash 2>/dev/null || true; "
            f"export ROBOT_MODEL={ROBOT_MODEL}; "
            f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
            f"{remote_cmd}"
        )
        cmd = ["ssh", "-tt", "-o", "StrictHostKeyChecking=no",
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

    if not name:
        return jsonify({"ok": False, "error": "Missing process name"}), 400

    ok, msg = _launch_process(name, extra_args, ssh_host)
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
# SocketIO events (thin relay – most comms go through roslibjs directly)
# ---------------------------------------------------------------------------

@socketio.on("connect")
def handle_connect():
    print("[WebUI] Client connected")


@socketio.on("disconnect")
def handle_disconnect():
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
