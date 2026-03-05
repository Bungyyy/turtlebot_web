#!/usr/bin/env python3
"""TurtleBot3 Web Control Interface - Flask Application Server.

Serves the web UI and proxies the camera feed from ROS.
The frontend connects directly to rosbridge_websocket for
all ROS topic/service communication.
"""

import os
import subprocess
import threading
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
    )


@app.route("/api/config")
def config():
    """Return current server configuration."""
    return jsonify(
        {
            "rosbridge_host": ROSBRIDGE_HOST,
            "rosbridge_port": ROSBRIDGE_PORT,
        }
    )


# ---------------------------------------------------------------------------
# Map save endpoint
# ---------------------------------------------------------------------------
MAP_SAVE_DIR = os.environ.get("MAP_SAVE_DIR", os.path.expanduser("~/maps"))


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
    socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=True)
