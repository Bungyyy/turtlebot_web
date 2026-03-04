/**
 * ROS Bridge Connection Manager
 * Handles connection to rosbridge_websocket and provides
 * topic/service/action wrappers used by other modules.
 */

/* global ROSLIB */
/* exported RosBridge */

const RosBridge = (() => {
  let ros = null;
  let connected = false;
  const listeners = { connect: [], close: [], error: [] };

  // ---- Public API -------------------------------------------------------

  function connect(host, port) {
    if (ros) {
      ros.close();
    }

    const url = `ws://${host}:${port}`;
    ros = new ROSLIB.Ros({ url });

    ros.on("connection", () => {
      connected = true;
      console.log("[RosBridge] Connected to", url);
      _emit("connect");
    });

    ros.on("close", () => {
      connected = false;
      console.warn("[RosBridge] Connection closed");
      _emit("close");
    });

    ros.on("error", (err) => {
      console.error("[RosBridge] Error:", err);
      _emit("error", err);
    });
  }

  function disconnect() {
    if (ros) ros.close();
  }

  function isConnected() {
    return connected;
  }

  function getRos() {
    return ros;
  }

  /** Subscribe to a topic. Returns the ROSLIB.Topic instance. */
  function subscribe(topicName, msgType, callback, opts) {
    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: msgType,
      throttle_rate: (opts && opts.throttle) || 0,
      queue_length: (opts && opts.queue) || 1,
      queue_size: (opts && opts.queueSize) || 1,
    });
    topic.subscribe(callback);
    return topic;
  }

  /** Publish a single message to a topic. */
  function publish(topicName, msgType, msg) {
    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: msgType,
    });
    topic.publish(new ROSLIB.Message(msg));
    return topic;
  }

  /** Create a persistent publisher. */
  function advertise(topicName, msgType) {
    return new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: msgType,
    });
  }

  /** Call a ROS service. Returns a promise. */
  function callService(srvName, srvType, request) {
    return new Promise((resolve, reject) => {
      const client = new ROSLIB.Service({
        ros,
        name: srvName,
        serviceType: srvType,
      });
      client.callService(
        new ROSLIB.ServiceRequest(request || {}),
        resolve,
        reject
      );
    });
  }

  /** Create an action client. */
  function actionClient(serverName, actionType) {
    return new ROSLIB.ActionClient({
      ros,
      serverName,
      actionName: actionType,
    });
  }

  // ---- Events -----------------------------------------------------------

  function on(event, fn) {
    if (listeners[event]) listeners[event].push(fn);
  }

  function _emit(event, data) {
    (listeners[event] || []).forEach((fn) => fn(data));
  }

  // ---- Return public interface ------------------------------------------

  return {
    connect,
    disconnect,
    isConnected,
    getRos,
    subscribe,
    publish,
    advertise,
    callService,
    actionClient,
    on,
  };
})();
