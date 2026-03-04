#!/usr/bin/env python3
"""TurtleBot3 Web Control Interface - Flask Application Server.

Serves the web UI and proxies the camera feed from ROS.
The frontend connects directly to rosbridge_websocket for
all ROS topic/service communication.
"""

import os
import threading
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
