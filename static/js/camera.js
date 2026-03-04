/**
 * Camera Viewer
 * Subscribes to compressed image topics and renders frames on a canvas.
 */

/* global RosBridge */
/* exported Camera */

const Camera = (() => {
  let canvas, ctx;
  let subscription = null;
  let streaming = false;

  function init() {
    canvas = document.getElementById("camera-canvas");
    ctx = canvas.getContext("2d");
    canvas.classList.add("hidden");

    document.getElementById("btn-camera-toggle").addEventListener("click", toggleStream);
  }

  function toggleStream() {
    if (streaming) {
      stopStream();
    } else {
      startStream();
    }
  }

  function startStream() {
    const topic = document.getElementById("camera-topic").value;
    const isCompressed = topic.includes("compressed");

    document.getElementById("camera-placeholder").style.display = "none";
    canvas.classList.remove("hidden");
    document.getElementById("btn-camera-toggle").textContent = "Stop Stream";
    streaming = true;

    if (isCompressed) {
      subscription = RosBridge.subscribe(
        topic,
        "sensor_msgs/CompressedImage",
        _onCompressedImage,
        { throttle: 66 }
      );
    } else {
      // Raw images via ros2djs or web_video_server fallback
      subscription = RosBridge.subscribe(
        topic,
        "sensor_msgs/Image",
        _onRawImage,
        { throttle: 100 }
      );
    }

    _setNodeStatus("node-camera", true);
  }

  function stopStream() {
    if (subscription) {
      subscription.unsubscribe();
      subscription = null;
    }
    streaming = false;
    canvas.classList.add("hidden");
    document.getElementById("camera-placeholder").style.display = "";
    document.getElementById("btn-camera-toggle").textContent = "Start Stream";
    _setNodeStatus("node-camera", false);
  }

  function _onCompressedImage(msg) {
    const img = new Image();
    img.onload = () => {
      canvas.width = img.width;
      canvas.height = img.height;
      ctx.drawImage(img, 0, 0);
    };
    img.src = "data:image/jpeg;base64," + msg.data;
  }

  function _onRawImage(msg) {
    // sensor_msgs/Image – typically rgb8 or bgr8
    const w = msg.width;
    const h = msg.height;
    canvas.width = w;
    canvas.height = h;

    const imgData = ctx.createImageData(w, h);
    const raw = atob(msg.data);
    const isBGR = msg.encoding === "bgr8";

    for (let i = 0; i < w * h; i++) {
      const si = i * 3;
      const di = i * 4;
      if (isBGR) {
        imgData.data[di]     = raw.charCodeAt(si + 2);
        imgData.data[di + 1] = raw.charCodeAt(si + 1);
        imgData.data[di + 2] = raw.charCodeAt(si);
      } else {
        imgData.data[di]     = raw.charCodeAt(si);
        imgData.data[di + 1] = raw.charCodeAt(si + 1);
        imgData.data[di + 2] = raw.charCodeAt(si + 2);
      }
      imgData.data[di + 3] = 255;
    }
    ctx.putImageData(imgData, 0, 0);
  }

  function _setNodeStatus(nodeId, online) {
    const el = document.getElementById(nodeId);
    if (online) el.classList.add("online");
    else el.classList.remove("online");
  }

  return { init, startStream, stopStream, toggleStream };
})();
