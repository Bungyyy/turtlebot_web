#!/usr/bin/env python3
"""Unitree Go2 Web Teleop — Flask Application Server.

ROS Bridge-first architecture:
  - Frontend publishes /cmd_vel directly via rosbridge WebSocket (primary, zero latency)
  - Backend provides SSH relay as fallback when rosbridge is unavailable
  - Sport API commands (StandUp, StandDown, etc.) via rosbridge or SSH

Features:
  - Launch Manager: start/stop ROS2 processes (bringup, SLAM, navigation, rosbridge)
  - Persistent teleop relay: SSH-based /cmd_vel publisher for fallback
  - Map management: save/load maps for navigation
  - Camera stream proxy
"""

import os
import json
import base64
import shlex
import signal
import subprocess
import threading
import time
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

# Deploy mode: "jetson" = Flask runs on Jetson (all commands local, no SSH needed)
#              "remote" = Flask runs on a PC (commands sent via SSH to Jetson)
DEPLOY_MODE = os.environ.get("DEPLOY_MODE", "jetson")

# ---------------------------------------------------------------------------
# Process Manager — launch / stop ROS2 processes from the web UI
# ---------------------------------------------------------------------------
_processes = {}   # name -> { "proc": Popen, "cmd": str, "log": list }
_proc_lock = threading.Lock()


ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "")
RMW_IMPLEMENTATION = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
GO2_DOMAIN_ID = os.environ.get("GO2_DOMAIN_ID", "")

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
    # Use GO2_DOMAIN_ID if set, otherwise fall back to ROS_DOMAIN_ID
    domain_id = GO2_DOMAIN_ID or ROS_DOMAIN_ID
    if domain_id:
        env["ROS_DOMAIN_ID"] = domain_id
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
    "teleop": [
        "ros2", "launch", "go2_teleop", "teleop.launch",
    ],
    "livox": [
        "ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py",
    ],
    "slam": [
        "ros2", "launch", "fast_lio", "mapping.launch.py",
        "rviz:=false",
    ],
    "pcl_to_scan": [
        "ros2", "run", "pointcloud_to_laserscan",
        "pointcloud_to_laserscan_node",
        "--ros-args",
        "-r", "cloud_in:=/cloud_registered",
        "-p", "min_height:=-0.3",
        "-p", "max_height:=0.3",
        "-p", "range_min:=0.3",
        "-p", "range_max:=30.0",
    ],
    "navigation": [
        "ros2", "launch", "go2_navigation", "navigation2.launch.py",
        "use_rviz:=false",
        # map:= argument is appended dynamically
    ],
    "localization": [
        "ros2", "launch", "go2_navigation", "localization.launch",
    ],
    "transform": [
        "ros2", "launch", "go2_navigation", "transform.launch",
    ],
    "nav_stack": [
        "ros2", "launch", "go2_navigation", "navigation.launch",
    ],
}


# Map process names to executables that should be killed before re-launch
_KILL_PATTERNS = {
    "rosbridge": ["rosbridge_websocket", "rosapi"],
    "teleop": ["teleop_twist_keyboard"],
    "livox": ["livox_ros_driver2", "msg_MID360"],
    "slam": ["fastlio_mapping", "fast_lio"],
    "pcl_to_scan": ["pointcloud_to_laserscan"],
    "navigation": ["bt_navigator", "controller_server", "planner_server",
                    "behavior_server", "lifecycle_manager_navigation"],
    "localization": ["amcl", "map_server", "lifecycle_manager_localization"],
    "transform": ["robot_state_publisher", "joint_state_publisher"],
    "nav_stack": ["bt_navigator", "controller_server", "planner_server",
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

    # In jetson mode, ignore SSH and run locally
    if DEPLOY_MODE == "jetson":
        ssh_host = None

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
    else:
        # Local mode (jetson): wrap with bash to source ROS2 workspace
        # so that go2_bringup, go2_teleop, etc. packages are found
        local_cmd = " ".join(cmd)
        wrapped = (
            "source /opt/ros/humble/setup.bash 2>/dev/null "
            "|| source /opt/ros/iron/setup.bash 2>/dev/null "
            "|| source /opt/ros/jazzy/setup.bash 2>/dev/null "
            "|| true; "
            "source ~/go2_ws/install/setup.bash 2>/dev/null || true; "
            "source ~/3d_ws/install/setup.bash 2>/dev/null || true; "
            + (f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; " if RMW_IMPLEMENTATION else "")
            + (f"export CYCLONEDDS_URI='{CYCLONEDDS_URI}'; " if CYCLONEDDS_URI else "")
            + (f"export ROS_DOMAIN_ID={GO2_DOMAIN_ID}; " if GO2_DOMAIN_ID else "")
            + local_cmd
        )
        cmd = ["bash", "-c", wrapped]

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
        deploy_mode=DEPLOY_MODE,
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
        "teleop_mode": "rosbridge_primary",
        "deploy_mode": DEPLOY_MODE,
    })


@app.route("/api/rosbridge/status")
def rosbridge_status():
    """Check if rosbridge is reachable (for frontend to decide teleop mode)."""
    import socket
    try:
        s = socket.create_connection((ROSBRIDGE_HOST, ROSBRIDGE_PORT), timeout=2)
        s.close()
        return jsonify({"ok": True, "host": ROSBRIDGE_HOST, "port": ROSBRIDGE_PORT})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


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
_sport_move_proc = None         # persistent ssh+ros2 topic pub for velocity (legacy)
_sport_move_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Persistent Teleop Relay — avoids SSH+DDS startup delay on every button press
# ---------------------------------------------------------------------------
# A small Python script is deployed to the Jetson and kept running.  It creates
# a ROS2 node once (paying the DDS discovery cost once) and then reads velocity
# commands as JSON lines from stdin, publishing /cmd_vel at 10Hz.
#
# Flow:  Browser → HTTP POST /api/sport/move → Flask → stdin write → SSH →
#        relay script on Jetson → /cmd_vel at 10Hz

_teleop_relay_proc = None       # persistent subprocess (stdin=PIPE)
_teleop_relay_mode = None       # "local" or "ssh"
_teleop_relay_host = None       # SSH host the relay is connected to
_teleop_relay_lock = threading.Lock()
_teleop_relay_deployed = False

_TELEOP_RELAY_SCRIPT = r'''#!/usr/bin/env python3
"""Teleop relay for Unitree Go2.

Reads JSON velocity from stdin, publishes to BOTH:
  1. /api/sport/request (unitree_api/msg/Request, api_id=1008 Move) — Go2 native
  2. /cmd_vel (geometry_msgs/msg/Twist) — fallback/bridge compatibility

The Go2 firmware ONLY listens to /api/sport/request for movement.
/cmd_vel has 0 subscribers by default on Go2, but we publish it anyway
in case a cmd_vel bridge node is running.

IMPORTANT: rclpy.spin() MUST run in the main thread for timers to fire.
"""
import os, sys, json, threading
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

def main():
    print(f"[relay] PID={os.getpid()}", flush=True)
    print(f"[relay] RMW={os.environ.get('RMW_IMPLEMENTATION','(unset)')}", flush=True)
    print(f"[relay] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID','(unset)')}", flush=True)
    cdds = os.environ.get('CYCLONEDDS_URI','(unset)')
    print(f"[relay] CYCLONEDDS_URI={cdds[:80] if cdds else '(unset)'}", flush=True)

    rclpy.init()
    node = rclpy.create_node('web_teleop_relay')

    # cmd_vel publisher (fallback — 0 subscribers on stock Go2)
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Sport API publisher — Go2 subscriber uses BEST_EFFORT QoS
    sport_pub = None
    try:
        from unitree_api.msg import Request
        sport_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        sport_pub = node.create_publisher(Request, '/api/sport/request', sport_qos)
        print("[relay] Sport API publisher created (unitree_api/msg/Request)", flush=True)
    except ImportError:
        print("[relay] WARNING: unitree_api not found — using cmd_vel only", flush=True)

    vel = [0.0, 0.0, 0.0]  # vx, vy, vyaw
    pub_count = [0]

    def timer_cb():
        vx, vy, vyaw = vel

        # 1. Publish Sport API Move (api_id=1008) — primary path for Go2
        if sport_pub:
            try:
                from unitree_api.msg import Request
                req = Request()
                req.header.identity.api_id = 1008  # Move
                req.parameter = json.dumps({
                    "x": vx, "y": vy,
                    "rx": 0.0, "ry": 0.0, "rz": vyaw
                })
                sport_pub.publish(req)
            except Exception as e:
                if pub_count[0] < 3:
                    print(f"[relay] sport pub error: {e}", flush=True)

        # 2. Publish cmd_vel (fallback)
        m = Twist()
        m.linear.x = vx
        m.linear.y = vy
        m.angular.z = vyaw
        cmd_vel_pub.publish(m)

        pub_count[0] += 1
        if pub_count[0] <= 3 or pub_count[0] % 100 == 0:
            mode = "sport+cmd_vel" if sport_pub else "cmd_vel only"
            print(f"[relay] pub #{pub_count[0]} ({mode}): vx={vx} vy={vy} vyaw={vyaw}", flush=True)

    node.create_timer(0.1, timer_cb)

    # Read stdin in background thread (main thread must run rclpy.spin)
    def stdin_reader():
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
                vel[0] = float(d.get('vx', 0))
                vel[1] = float(d.get('vy', 0))
                vel[2] = float(d.get('vyaw', 0))
                print(f"[relay] recv: vx={vel[0]} vy={vel[1]} vyaw={vel[2]}", flush=True)
            except Exception as e:
                print(f"[relay] parse error: {e}", flush=True)
        print("[relay] stdin closed, shutting down", flush=True)
        rclpy.shutdown()

    threading.Thread(target=stdin_reader, daemon=True).start()

    print("READY", flush=True)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
'''

_LOCAL_RELAY_PATH = "/tmp/_web_teleop_relay.py"


def _write_relay_script_local():
    """Write relay script to /tmp locally."""
    try:
        with open(_LOCAL_RELAY_PATH, "w") as f:
            f.write(_TELEOP_RELAY_SCRIPT)
        return True
    except Exception as e:
        print(f"[Teleop] Failed to write local relay: {e}")
        return False


def _deploy_relay_ssh():
    """Deploy the teleop relay script to the Jetson via SSH (idempotent)."""
    global _teleop_relay_deployed
    if _teleop_relay_deployed:
        return True
    script_b64 = base64.b64encode(_TELEOP_RELAY_SCRIPT.encode()).decode()
    rc, _, stderr = _ssh_cmd(
        f"echo '{script_b64}' | base64 -d > /tmp/_web_teleop_relay.py && "
        f"chmod +x /tmp/_web_teleop_relay.py",
        timeout=8,
    )
    if rc == 0:
        _teleop_relay_deployed = True
        print("[Teleop] Relay script deployed to Jetson via SSH")
        return True
    print(f"[Teleop] Failed to deploy relay via SSH: {stderr}")
    return False


def _wait_for_ready(proc, timeout_sec=15):
    """Wait for the relay process to print READY. Returns True if ready."""
    import select
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if proc.poll() is not None:
            # Process exited — read any remaining output for diagnostics
            try:
                remaining = proc.stdout.read().decode()
                if remaining:
                    print(f"[Teleop] Relay exited early, output: {remaining[:500]}")
            except Exception:
                pass
            try:
                err = proc.stderr.read().decode() if proc.stderr else ""
                if err:
                    print(f"[Teleop] Relay stderr: {err[:500]}")
            except Exception:
                pass
            print(f"[Teleop] Relay process exited with code {proc.returncode}")
            return False
        try:
            rlist, _, _ = select.select([proc.stdout], [], [], 0.3)
            if rlist:
                line = proc.stdout.readline().decode().strip()
                if line:
                    print(f"[Teleop] Relay stdout: {line}")
                if "READY" in line:
                    return True
        except Exception as e:
            print(f"[Teleop] select error: {e}")
            break
    print("[Teleop] Relay did not send READY in time")
    return False


def _start_relay_local():
    """Try starting the relay LOCALLY (same machine as Flask, no SSH)."""
    global _teleop_relay_proc, _teleop_relay_mode

    if not _write_relay_script_local():
        return False

    # Build command that sources ROS2 setup and runs the relay
    # Try multiple ROS2 setup paths (humble, iron, jazzy)
    _domain_id = GO2_DOMAIN_ID or ROS_DOMAIN_ID
    setup_cmd = (
        "source /opt/ros/humble/setup.bash 2>/dev/null "
        "|| source /opt/ros/iron/setup.bash 2>/dev/null "
        "|| source /opt/ros/jazzy/setup.bash 2>/dev/null "
        "|| true; "
        "source ~/go2_ws/install/setup.bash 2>/dev/null || true; "
        + (f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; " if RMW_IMPLEMENTATION else "")
        + (f"export ROS_DOMAIN_ID={_domain_id}; " if _domain_id else "")
        + f"python3 -u {_LOCAL_RELAY_PATH}"
    )

    env = os.environ.copy()
    if RMW_IMPLEMENTATION:
        env["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    if _domain_id:
        env["ROS_DOMAIN_ID"] = _domain_id
    if CYCLONEDDS_URI:
        env["CYCLONEDDS_URI"] = CYCLONEDDS_URI

    print("[Teleop] Trying LOCAL relay (no SSH)...")
    try:
        _teleop_relay_proc = subprocess.Popen(
            ["bash", "-c", setup_cmd],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            preexec_fn=os.setsid,
        )
    except Exception as e:
        print(f"[Teleop] Failed to start local relay: {e}")
        _teleop_relay_proc = None
        return False

    if _wait_for_ready(_teleop_relay_proc, timeout_sec=10):
        _teleop_relay_mode = "local"
        print("[Teleop] LOCAL relay started and ready!")
        threading.Thread(
            target=_read_relay_output, args=(_teleop_relay_proc,), daemon=True
        ).start()
        return True

    # Failed — clean up
    _kill_relay()
    print("[Teleop] Local relay failed")
    return False


def _start_relay_ssh():
    """Start the persistent teleop relay on the Jetson via SSH."""
    global _teleop_relay_proc, _teleop_relay_mode, _teleop_relay_host

    if not _deploy_relay_ssh():
        return False

    remote = "python3 -u /tmp/_web_teleop_relay.py"
    # Use bash -i -c to load .bashrc (needed for CYCLONEDDS_URI etc.)
    inner = _JETSON_ROS_SETUP + remote
    wrapped = f"bash -i -c {shlex.quote(inner)}"
    cmd = [
        "sshpass", "-p", SSH_PASSWORD,
        "ssh", "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=5",
        "-o", "ServerAliveInterval=10",
        _sport_ssh_host, wrapped,
    ]

    print(f"[Teleop] Trying SSH relay to {_sport_ssh_host}...")
    try:
        _teleop_relay_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
    except Exception as e:
        print(f"[Teleop] Failed to start SSH relay: {e}")
        _teleop_relay_proc = None
        return False

    _teleop_relay_host = _sport_ssh_host

    if _wait_for_ready(_teleop_relay_proc, timeout_sec=15):
        _teleop_relay_mode = "ssh"
        print(f"[Teleop] SSH relay started and ready on {_sport_ssh_host}!")
        # Background thread to forward relay stdout for debugging
        _relay_reader = threading.Thread(
            target=_read_relay_output, args=(_teleop_relay_proc,), daemon=True)
        _relay_reader.start()
        return True

    # Failed — clean up
    _kill_relay()
    print("[Teleop] SSH relay failed")
    return False


def _start_relay():
    """Start the teleop relay — try local first, then SSH."""
    global _teleop_relay_proc, _teleop_relay_mode, _teleop_relay_deployed

    # Already running?
    if _teleop_relay_proc and _teleop_relay_proc.poll() is None:
        return True

    _kill_relay()
    # Force re-deploy to pick up any script changes
    _teleop_relay_deployed = False

    # 1) Try local relay (no SSH — works if ROS2 is on the same machine)
    if _start_relay_local():
        return True

    # 2) Fall back to SSH relay
    if _start_relay_ssh():
        return True

    print("[Teleop] All relay methods failed")
    return False


def _read_relay_output(proc):
    """Background thread: forward relay stdout to Flask console."""
    try:
        while proc.poll() is None:
            line = proc.stdout.readline()
            if line:
                print(f"[Relay] {line.decode().rstrip()}")
    except Exception:
        pass


def _send_vel_relay(vx, vy, vyaw):
    """Send a velocity command to the running relay (instant)."""
    global _teleop_relay_proc
    if not _teleop_relay_proc or _teleop_relay_proc.poll() is not None:
        if not _start_relay():
            return False

    try:
        cmd_line = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw}) + "\n"
        _teleop_relay_proc.stdin.write(cmd_line.encode())
        _teleop_relay_proc.stdin.flush()
        return True
    except (BrokenPipeError, OSError) as e:
        print(f"[Teleop] Relay write failed ({_teleop_relay_mode}): {e}")
        _kill_relay()
        return False


def _kill_relay():
    """Kill the teleop relay process."""
    global _teleop_relay_proc, _teleop_relay_host, _teleop_relay_mode
    if _teleop_relay_proc:
        try:
            os.killpg(os.getpgid(_teleop_relay_proc.pid), signal.SIGTERM)
            _teleop_relay_proc.wait(timeout=3)
        except Exception:
            try:
                os.killpg(os.getpgid(_teleop_relay_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        print(f"[Teleop] Relay killed (was {_teleop_relay_mode})")
        _teleop_relay_proc = None
    _teleop_relay_host = None
    _teleop_relay_mode = None

# ROS2 env setup to source on the Jetson via SSH.
# Set domain ID AFTER sourcing setup.bash (it can reset env vars).
_JETSON_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
    "source ~/go2_ws/install/setup.bash 2>/dev/null; "
    f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; "
    + (f"export ROS_DOMAIN_ID={GO2_DOMAIN_ID}; " if GO2_DOMAIN_ID else "")
)


def _local_cmd(cmd_str, timeout=8):
    """Run a ROS2 command locally. Returns (returncode, stdout, stderr)."""
    setup_cmd = (
        "source /opt/ros/humble/setup.bash 2>/dev/null "
        "|| source /opt/ros/iron/setup.bash 2>/dev/null "
        "|| source /opt/ros/jazzy/setup.bash 2>/dev/null "
        "|| true; "
        "source ~/go2_ws/install/setup.bash 2>/dev/null || true; "
        + (f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; " if RMW_IMPLEMENTATION else "")
        + (f"export ROS_DOMAIN_ID={GO2_DOMAIN_ID}; " if GO2_DOMAIN_ID else "")
        + cmd_str
    )
    try:
        result = subprocess.run(
            ["bash", "-c", setup_cmd],
            capture_output=True, text=True, timeout=timeout,
            env=_build_env(),
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"
    except Exception as e:
        return -1, "", str(e)


def _local_popen(cmd_str):
    """Start a persistent local ROS2 command. Returns Popen."""
    setup_cmd = (
        "source /opt/ros/humble/setup.bash 2>/dev/null "
        "|| source /opt/ros/iron/setup.bash 2>/dev/null "
        "|| source /opt/ros/jazzy/setup.bash 2>/dev/null "
        "|| true; "
        "source ~/go2_ws/install/setup.bash 2>/dev/null || true; "
        + (f"export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; " if RMW_IMPLEMENTATION else "")
        + (f"export ROS_DOMAIN_ID={GO2_DOMAIN_ID}; " if GO2_DOMAIN_ID else "")
        + cmd_str
    )
    return subprocess.Popen(
        ["bash", "-c", setup_cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=_build_env(),
        preexec_fn=os.setsid,
    )


def _run_cmd(remote_cmd, timeout=8):
    """Run a ROS2 command — locally in jetson mode, via SSH in remote mode."""
    if DEPLOY_MODE == "jetson":
        return _local_cmd(remote_cmd, timeout)
    return _ssh_cmd(remote_cmd, timeout)


def _run_popen(remote_cmd):
    """Start a persistent ROS2 command — locally in jetson mode, via SSH in remote mode."""
    if DEPLOY_MODE == "jetson":
        return _local_popen(remote_cmd)
    return _ssh_popen(remote_cmd)


def _ssh_cmd(remote_cmd, timeout=8):
    """Run a command on the Jetson via SSH. Returns (returncode, stdout, stderr)."""
    # Use 'bash -i -c' to force interactive mode so that ~/.bashrc is sourced.
    # This picks up env vars (CYCLONEDDS_URI, etc.) needed for DDS discovery.
    inner = _JETSON_ROS_SETUP + remote_cmd
    wrapped = f"bash -i -c {shlex.quote(inner)}"
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
    inner = _JETSON_ROS_SETUP + remote_cmd
    wrapped = f"bash -i -c {shlex.quote(inner)}"
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
    rc, stdout, stderr = _run_cmd("ros2 topic info /api/sport/request")
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
    # Go2 requires parameter: "" for commands without parameters.
    # json.dumps({}) produces '{}' which the robot silently ignores.
    if not params or params == {}:
        param_str = ""
    elif isinstance(params, dict):
        param_str = json.dumps(params)
    else:
        param_str = str(params)
    # Build YAML matching the exact format that works on the Jetson:
    #   '{header: ..., parameter: "", binary: []}'
    # The whole thing is single-quoted for the shell.  Double-quotes are
    # literal inside single-quotes, so we just embed " directly.
    return (
        "'{header: {identity: {id: 0, api_id: " + str(api_id) + "}}, "
        'parameter: "' + param_str + '", '
        "binary: []}'"
    )


def _sport_pub_once(api_id, params):
    """Publish a single sport command on the Jetson."""
    msg_type = _resolve_sport_type()
    if not msg_type:
        return False, "Cannot resolve /api/sport/request type"
    yaml_msg = _sport_yaml(api_id, params)
    # Go2 subscriber uses BEST_EFFORT QoS — match it with --qos-reliability.
    # Publish at 10Hz for 5s — DDS discovery can take 1-2s, so we need enough
    # time for the subscriber to discover us and receive several messages.
    remote = (f"echo '[env] RMW='$RMW_IMPLEMENTATION 'DOMAIN='$ROS_DOMAIN_ID 'CDDS='$CYCLONEDDS_URI; "
              f"timeout 5 ros2 topic pub --rate 10 "
              f"--qos-reliability best_effort "
              f"/api/sport/request {msg_type} {yaml_msg}")
    print(f"[Sport] cmd ({DEPLOY_MODE}): {remote}")
    rc, stdout, stderr = _run_cmd(remote, timeout=8)
    print(f"[Sport] rc={rc} stdout={stdout!r} stderr={stderr!r}")
    # 'timeout' returns 124 when it kills the child – that's expected/success
    if rc in (0, 124):
        return True, msg_type
    return False, stderr or stdout


def _sport_move_start(vx, vy, vyaw):
    """Start a persistent velocity publisher on the Jetson via Sport API Move."""
    global _sport_move_proc

    msg_type = _resolve_sport_type() or "unitree_api/msg/Request"
    param_json = json.dumps({"x": vx, "y": vy, "rx": 0.0, "ry": 0.0, "rz": vyaw})
    yaml_msg = (
        "'{header: {identity: {id: 0, api_id: 1008}}, "
        'parameter: "' + param_json + '", '
        "binary: []}'"
    )
    remote = (f"ros2 topic pub --rate 10 --qos-reliability best_effort "
              f"/api/sport/request {msg_type} {yaml_msg}")

    with _sport_move_lock:
        _kill_sport_move_proc()
        try:
            _sport_move_proc = _run_popen(remote)
            print(f"[Sport] Move started via sport API: vx={vx} vy={vy} vyaw={vyaw}")
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

    # Update SSH host from request if provided (remote mode only)
    global _sport_ssh_host
    if DEPLOY_MODE != "jetson" and data.get("ssh_host"):
        _sport_ssh_host = data["ssh_host"]

    _sport_move_stop()
    # Stop relay velocity too (e.g., Damp should stop all motion)
    with _teleop_relay_lock:
        _send_vel_relay(0, 0, 0)

    ok, detail = _sport_pub_once(api_id, parameter)
    label = data.get("label", str(api_id))
    print(f"[Sport] {label} (api_id={api_id}): {'OK' if ok else detail}")
    if not ok:
        return jsonify({"ok": False, "error": detail}), 500

    # After StandUp (1004), automatically send BalanceStand (1002) so the
    # robot is ready to accept Move commands.
    if api_id == 1004:
        ok2, detail2 = _sport_pub_once(1002, {})
        print(f"[Sport] BalanceStand (api_id=1002): {'OK' if ok2 else detail2}")

    return jsonify({"ok": True, "api_id": api_id, "type": detail})


@app.route("/api/sport/move", methods=["POST"])
def sport_move():
    """Start/stop persistent velocity publishing for teleop.

    Uses a persistent relay script on the Jetson that keeps a ROS2 node alive
    so velocity updates are instant (no SSH/DDS startup per button press).
    Falls back to legacy ros2-topic-pub-per-SSH if the relay fails.
    """
    data = request.get_json(force=True)
    vx = float(data.get("vx", 0))
    vy = float(data.get("vy", 0))
    vyaw = float(data.get("vyaw", 0))

    # Update SSH host from request if provided (remote mode only)
    global _sport_ssh_host
    if DEPLOY_MODE != "jetson" and data.get("ssh_host"):
        _sport_ssh_host = data["ssh_host"]

    is_moving = vx != 0 or vy != 0 or vyaw != 0

    # --- Try persistent relay first (instant response) ---
    with _teleop_relay_lock:
        ok = _send_vel_relay(vx, vy, vyaw)

    if ok:
        action = "stop" if not is_moving else "move"
        return jsonify({"ok": True, "action": action, "method": "relay",
                        "vx": vx, "vy": vy, "vyaw": vyaw})

    # --- Fallback: legacy per-SSH ros2 topic pub via Sport API ---
    print("[Teleop] Relay unavailable, falling back to legacy SSH method")
    if not is_moving:
        _sport_move_stop()
        _sport_pub_once(1003, {})
        return jsonify({"ok": True, "action": "stop", "method": "legacy"})
    else:
        _sport_move_start(vx, vy, vyaw)
        return jsonify({"ok": True, "action": "move", "method": "legacy",
                        "vx": vx, "vy": vy, "vyaw": vyaw})


@app.route("/api/teleop/warmup", methods=["POST"])
def teleop_warmup():
    """Pre-start the teleop relay so the first button press is instant."""
    global _sport_ssh_host
    data = request.get_json(force=True) if request.data else {}
    if data.get("ssh_host"):
        _sport_ssh_host = data["ssh_host"]

    with _teleop_relay_lock:
        ok = False
        if not (_teleop_relay_proc and _teleop_relay_proc.poll() is None):
            ok = _start_relay()
        else:
            ok = True
    return jsonify({"ok": ok, "mode": _teleop_relay_mode})


@app.route("/api/teleop/status")
def teleop_status():
    """Return the current state of the teleop relay."""
    alive = _teleop_relay_proc and _teleop_relay_proc.poll() is None
    return jsonify({
        "relay_running": alive,
        "mode": _teleop_relay_mode,
        "ssh_host": _sport_ssh_host,
        "relay_host": _teleop_relay_host,
        "deploy_mode": DEPLOY_MODE,
    })


@app.route("/api/sport/topic_type")
def sport_topic_type():
    """Return the resolved message type for /api/sport/request."""
    msg_type = _resolve_sport_type()
    return jsonify({"ok": bool(msg_type), "type": msg_type})


@app.route("/api/sport/debug")
def sport_debug():
    """Comprehensive debug: check SSH, env, ROS nodes, topics, and test publish."""
    relay_alive = _teleop_relay_proc and _teleop_relay_proc.poll() is None
    info = {
        "ssh_host": _sport_ssh_host,
        "relay": {
            "running": relay_alive,
            "mode": _teleop_relay_mode,
            "host": _teleop_relay_host,
        },
        "checks": {},
    }

    # 1. Connectivity check (SSH or local)
    rc, stdout, stderr = _run_cmd("echo OK", timeout=5)
    info["checks"]["connection"] = {
        "ok": rc == 0 and "OK" in stdout,
        "rc": rc, "stdout": stdout, "stderr": stderr,
        "mode": DEPLOY_MODE,
    }

    if rc != 0:
        return jsonify(info)

    # 2. Environment variables
    rc, stdout, stderr = _run_cmd(
        "echo RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID "
        "CYCLONE=$CYCLONEDDS_URI FASTRTPS=$FASTRTPS_DEFAULT_PROFILES_FILE"
    )
    info["checks"]["env"] = {"output": stdout, "stderr": stderr}

    # 3. ROS2 daemon / node list
    rc, stdout, stderr = _run_cmd("ros2 node list 2>&1", timeout=10)
    info["checks"]["nodes"] = {
        "rc": rc, "output": stdout.splitlines() if stdout else [],
        "stderr": stderr,
    }

    # 4. ROS2 topic list
    rc, stdout, stderr = _run_cmd("ros2 topic list 2>&1", timeout=10)
    topics = stdout.splitlines() if stdout else []
    info["checks"]["topics"] = {
        "rc": rc, "topics": topics, "stderr": stderr,
        "has_cmd_vel": "/cmd_vel" in topics,
        "has_sport_request": "/api/sport/request" in topics,
    }

    # 5. /cmd_vel topic info (subscribers count)
    if "/cmd_vel" in topics:
        rc, stdout, stderr = _run_cmd("ros2 topic info /cmd_vel -v 2>&1", timeout=10)
        info["checks"]["cmd_vel_info"] = {"rc": rc, "output": stdout, "stderr": stderr}

    # 6. Test publish to /cmd_vel (zero velocity, safe)
    rc, stdout, stderr = _run_cmd(
        "timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        "'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'",
        timeout=8,
    )
    info["checks"]["cmd_vel_pub_test"] = {
        "rc": rc, "stdout": stdout, "stderr": stderr,
        "note": "rc=0 means --once found a subscriber and published OK",
    }

    # 7. Sport API topic type (for StandUp/StandDown)
    msg_type = _resolve_sport_type()
    info["checks"]["sport_api_type"] = {"type": msg_type}

    return jsonify(info)


@app.route("/api/sport/test_move")
def sport_test_move():
    """Test: publish 0.1 m/s forward on /cmd_vel for 3 seconds via SSH.

    This directly tests whether the robot responds to /cmd_vel.
    Visit this URL and watch the robot — it should walk forward slowly.
    """
    rc, stdout, stderr = _run_cmd(
        "timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "
        "'{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'",
        timeout=8,
    )
    return jsonify({
        "rc": rc, "stdout": stdout, "stderr": stderr,
        "note": "Robot should have walked forward slowly for 3 seconds. "
                "rc=124 means timeout killed it (expected/OK).",
    })


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

import atexit
atexit.register(_kill_relay)

if __name__ == "__main__":
    print(f"[WebUI] Starting on http://0.0.0.0:{WEB_PORT}")
    print(f"[WebUI] Deploy mode: {DEPLOY_MODE}")
    if DEPLOY_MODE == "jetson":
        print(f"[WebUI] Running on Jetson — all ROS2 commands execute locally (no SSH)")
    else:
        print(f"[WebUI] Running on remote PC — ROS2 commands sent via SSH to {_sport_ssh_host}")
    print(f"[WebUI] Expecting rosbridge at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    print(f"[WebUI] Robot model: {ROBOT_MODEL}")
    print(f"[WebUI] Map save dir: {MAP_SAVE_DIR}")
    socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=True)
