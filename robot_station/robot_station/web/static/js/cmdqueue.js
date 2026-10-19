/* FAFU station · command queue for a temporarily closed /ws/cmd socket.
 *
 * Pure logic: the clock, the sender and the drop callback are injected, so this
 * module has no DOM or WebSocket dependency and can be tested under Node.
 * app.js owns the socket and all UI side effects.
 *
 * Rules:
 *  - 100 Hz teleop traffic (arm stream / cart / teach / touch / rtt / ping) is
 *    never queued; it is stale by definition.
 *  - Discrete commands are queued for at most ttlMs, then cancelled.
 *  - Estop is special: enqueueing it cancels every other pending command and
 *    it is always flushed first, so a queued "home" can never run before it.
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.CmdQueue = factory();
  }
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  var DEFAULTS = { ttlMs: 3000, maxItems: 16 };

  function shouldDrop(obj) {
    if (!obj || !obj.t) return false;
    var t = obj.t;
    if (t === "cart" || t === "teach" || t === "touch" || t === "rtt" || t === "ping") return true;
    return t === "arm" && !!obj.stream;
  }

  function create(opts) {
    opts = opts || {};
    var ttlMs = Number.isFinite(opts.ttlMs) && opts.ttlMs >= 0 ? opts.ttlMs : DEFAULTS.ttlMs;
    var maxItems = Number.isInteger(opts.maxItems) && opts.maxItems > 0 ? opts.maxItems : DEFAULTS.maxItems;
    var now = typeof opts.now === "function"
      ? opts.now
      : function () {
          return (typeof performance !== "undefined" && performance.now)
            ? performance.now()
            : Date.now();
        };
    var items = [];

    function safeDrop(onDrop, obj, reason) {
      if (typeof onDrop !== "function") return;
      try {
        onDrop(obj, reason);
      } catch (e) {
        /* a UI callback must never break the queue */
      }
    }

    function removeOldestNonEstop(onDrop, reason) {
      for (var i = 0; i < items.length; i += 1) {
        if (items[i].obj.t !== "e") {
          var gone = items.splice(i, 1)[0];
          safeDrop(onDrop, gone.obj, reason);
          return true;
        }
      }
      return false;
    }

    function enqueue(obj, onDrop) {
      if (!obj || typeof obj !== "object" || shouldDrop(obj)) return false;
      var at = now();
      if (obj.t === "e") {
        if (items.some(function (it) { return it.obj.t === "e"; })) return false;
        var cancelled = items.splice(0, items.length);
        cancelled.forEach(function (it) {
          safeDrop(onDrop, it.obj, "急停优先，已取消");
        });
        items.push({ obj: obj, at: at });
        return true;
      }
      items.push({ obj: obj, at: at });
      if (items.length > maxItems) {
        removeOldestNonEstop(onDrop, "排队过多，已丢弃最旧指令");
      }
      return true;
    }

    function sweep(onDrop) {
      var t = now();
      for (var i = items.length - 1; i >= 0; i -= 1) {
        var it = items[i];
        if (it.obj.t !== "e" && t - it.at > ttlMs) {
          items.splice(i, 1);
          safeDrop(onDrop, it.obj, "连接未恢复，已取消");
        }
      }
    }

    function flush(send, onDrop) {
      if (!items.length) return true;
      var ordered = [];
      items.forEach(function (it) { if (it.obj.t === "e") ordered.push(it); });
      items.forEach(function (it) { if (it.obj.t !== "e") ordered.push(it); });
      items.length = 0;
      var t = now();
      for (var i = 0; i < ordered.length; i += 1) {
        var it = ordered[i];
        if (it.obj.t !== "e" && t - it.at > ttlMs) {
          safeDrop(onDrop, it.obj, "连接未恢复，已取消");
          continue;
        }
        var ok = false;
        try {
          ok = !!send(it.obj);
        } catch (e) {
          ok = false;
        }
        if (!ok) {
          items.push(it);
          for (var j = i + 1; j < ordered.length; j += 1) items.push(ordered[j]);
          return false;
        }
      }
      return true;
    }

    function size() {
      return items.length;
    }

    function hasEstop() {
      return items.some(function (it) { return it.obj.t === "e"; });
    }

    function snapshot() {
      return items.map(function (it) { return { t: it.obj.t, at: it.at }; });
    }

    return {
      enqueue: enqueue,
      sweep: sweep,
      flush: flush,
      size: size,
      hasEstop: hasEstop,
      snapshot: snapshot
    };
  }

  return { create: create, shouldDrop: shouldDrop, DEFAULTS: DEFAULTS };
});
