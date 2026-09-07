/* FAFU 机械臂站控 · 浏览器端：控制 JSON WebSocket + 独立视频 WebSocket
 * 客户 PC 本机直连机械臂。
 */
(() => {
  const PAGES = ["home", "arm"];
  const DEFAULT_PAGE = "arm";
  const PAGE_TITLE = { home: "总览", arm: "机械臂" };
  const DEFAULT_LIMITS = [
    [-140.688, 148.140],
    [0.0, 180.0],
    [-0.036, 205.0],
    [-124.956, 90.972],
    [-87.228, 72.972],
    [-95.004, 90.540],
  ];
  let limits = DEFAULT_LIMITS.map((p) => p.slice());
  let gripLimits = [8, 75];
  const GRIP_EFFORT_MIN = 50;
  const GRIP_EFFORT_MAX = 800;
  const GRIP_EFFORT_NM = 0.0053;
  let waypoints = [];
  const VID_HDR = 12;
  let ws = null;
  let vws = null;
  let last = null;
  let info = null;
  let hz = 0;
  let nCmd = 0;
  let hzT0 = performance.now();
  let rtt = null;
  let pingId = 0;
  let pingAt = 0;
  let lastEchoT0 = null;
  let lastRttNote = 0;
  let rttWs = null;
  let rttWorker = null;
  let rttBlobUrl = "";
  let telemWs = null;
  let pendingTelem = null;
  let snapRaf = 0;
  let lastTelemAt = 0;
  const TELEM_MAGIC = 0x314E5453;
  const TELEM_SIZE = 88;
  let live = false;
  let myCid = "";
  let ctrlMode = "Position";
  let spaceMode = "joint";
  let followDirty = false;
  let jointDirty = false;
  let controlTimer = 0;
  let controlRaf = 0;
  let controlHz = 100;
  let lastCtrl = 0;
  let lastDom = 0;
  const CART_KEYS = {
    KeyW: [0.005, 0, 0, 0, 0, 0],
    KeyS: [-0.005, 0, 0, 0, 0, 0],
    KeyA: [0, 0.005, 0, 0, 0, 0],
    KeyD: [0, -0.005, 0, 0, 0, 0],
    KeyQ: [0, 0, 0.005, 0, 0, 0],
    KeyE: [0, 0, -0.005, 0, 0, 0],
    Digit1: [0, 0, 0, 1.7, 0, 0],
    Digit2: [0, 0, 0, -1.7, 0, 0],
    Digit3: [0, 0, 0, 0, 1.7, 0],
    Digit4: [0, 0, 0, 0, -1.7, 0],
    Digit5: [0, 0, 0, 0, 0, 1.7],
    Digit6: [0, 0, 0, 0, 0, -1.7],
  };
  const cartHeld = new Set();
  const CMD_KEYS = {
    KeyH: "home",
    KeyU: "clear",
    Enter: "send",
    NumpadEnter: "send",
    KeyP: "enable",
    KeyL: "disable",
    Space: "grip-toggle",
    KeyF: "follow",
    KeyJ: "joint",
    KeyT: "cartspace",
  };
  const CMD_LABEL = {
    estop: "急停",
    clear: "解除急停",
    home: "复位回零",
    send: "发送位置",
    enable: "使能",
    disable: "去使能",
    "grip-toggle": "夹爪开/合",
    "grip-open": "夹爪打开",
    "grip-close": "夹爪闭合",
    follow: "实时跟随",
    joint: "关节空间",
    cartspace: "笛卡尔空间",
    "cart-move": "笛卡尔遥操作",
    connect: "Connect",
    disconnect: "Disconnect",
  };
  const CART_AXES = [
    { label: "X", unit: "m", min: -0.30, max: 0.75, step: 0.001, digits: 3, def: 0.247 },
    { label: "Y", unit: "m", min: -0.55, max: 0.55, step: 0.001, digits: 3, def: 0.0 },
    { label: "Z", unit: "m", min: -0.20, max: 0.70, step: 0.001, digits: 3, def: 0.169 },
    { label: "滚转", unit: "°", min: -180, max: 180, step: 0.1, digits: 1, def: 0 },
    { label: "俯仰", unit: "°", min: -180, max: 180, step: 0.1, digits: 1, def: 0 },
    { label: "偏航", unit: "°", min: -180, max: 180, step: 0.1, digits: 1, def: 0 },
  ];
  let cartDirty = false;
  const urls = { 0: "", 1: "" };

  const $ = (id) => document.getElementById(id);

  function clipGripEffort(v) {
    const n = Math.round(Number(v));
    if (!Number.isFinite(n)) return 300;
    return Math.max(GRIP_EFFORT_MIN, Math.min(GRIP_EFFORT_MAX, n));
  }
  function currentGripEffort() {
    return clipGripEffort($("gripEffort") ? $("gripEffort").value : 300);
  }
  function paintGripEffortNm() {
    const raw = currentGripEffort();
    if ($("gripEffortUnit")) $("gripEffortUnit").textContent = "raw ≈ " + (raw * GRIP_EFFORT_NM).toFixed(2) + " Nm";
  }
  function sendGripEffort() {
    paintGripEffortNm();
    send({ t: "grip", effort: currentGripEffort() });
  }

  let gripOpenCmd = null;
  let gripToggleAt = 0;
  const GRIP_CMD_HOLD_MS = 800;

  function gripperIsOpen() {
    if (gripOpenCmd != null && (performance.now() - gripToggleAt) < GRIP_CMD_HOLD_MS) {
      return gripOpenCmd;
    }
    if (last && last.arm && typeof last.arm.gripper_open === "boolean") {
      return !!last.arm.gripper_open;
    }
    const deg = Number(($("gripDeg") && $("gripDeg").value) || 0);
    const mid = 0.5 * (gripLimits[0] + gripLimits[1]);
    return deg >= mid;
  }

  function sendGripOpen(open) {
    const next = !!open;
    gripOpenCmd = next;
    gripToggleAt = performance.now();
    send({ t: "grip", open: next, effort: currentGripEffort() });
    paintGrip(next, next ? gripLimits[1] : gripLimits[0]);
    setBtnOn("btnGripOpen", next);
    setBtnOn("btnGripClose", !next);
    pulseBtn(next ? $("btnGripOpen") : $("btnGripClose"));
    flashHot("grip-toggle", next ? "夹爪打开" : "夹爪闭合");
    setAck(next ? "夹爪打开" : "夹爪闭合", true);
  }

  function toggleGripper() {
    sendGripOpen(!gripperIsOpen());
  }

  function currentPage() {
    const raw = (location.hash || "#/" + DEFAULT_PAGE).replace(/^#\/?/, "").split("?")[0];
    return PAGES.indexOf(raw) >= 0 ? raw : DEFAULT_PAGE;
  }
  function ensureDefaultHash() {
    const raw = (location.hash || "").replace(/^#\/?/, "").split("?")[0];
    if (!raw || PAGES.indexOf(raw) < 0) {
      location.hash = "#/" + DEFAULT_PAGE;
      return true;
    }
    return false;
  }
  function showPage() {
    if (ensureDefaultHash()) return;
    const page = currentPage();
    document.querySelectorAll(".page").forEach((el) => el.classList.toggle("on", el.id === "page-" + page));
    document.querySelectorAll("#navTabs a, #pageJump a").forEach((a) => {
      a.classList.toggle("on", a.getAttribute("data-page") === page);
    });
    document.title = "FAFU 机械臂站控 · " + (PAGE_TITLE[page] || page);
    syncStage();
  }

  function syncStage() {
    const twin = $("panel-twin");
    const cams = $("panel-cams");
    const stage = $("panel-stage");
    const twinOn = !!(twin && !twin.classList.contains("off"));
    const camsOn = !!(cams && !cams.classList.contains("off"));
    if (stage) stage.classList.toggle("off", !twinOn && !camsOn);
    if (window.Arm3D) {
      Arm3D.setActive(twinOn && currentPage() === "arm");
      requestAnimationFrame(() => Arm3D.resize());
    }
  }

  function typing() {
    const el = document.activeElement;
    if (!el) return false;
    if (el.isContentEditable) return true;
    const tag = (el.tagName || "").toLowerCase();
    if (tag === "textarea" || tag === "select") return true;
    if (tag !== "input") return false;
    const typ = (el.type || "text").toLowerCase();
    return typ !== "range" && typ !== "checkbox" && typ !== "button" && typ !== "radio";
  }
  function noteHz() {
    nCmd += 1;
    flushHz();
  }
  function flushHz() {
    const now = performance.now();
    if (now - hzT0 < 1000) return;
    hz = nCmd;
    nCmd = 0;
    hzT0 = now;
    if ($("hzTxt")) $("hzTxt").textContent = hz + " Hz";
  }
  function teleopHot() {
    if (cartHeld.size) return true;
    return !!( $("armFollow") && $("armFollow").checked );
  }
  function send(obj) {
    if (obj && (obj.t === "arm" || obj.t === "cart" || obj.t === "teach" || obj.t === "cart_go")) {
      obj.t0 = performance.now();
    }
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }
  function noteRtt(ms) {
    if (!Number.isFinite(ms) || ms < 0 || ms > 250) return;
    rtt = ms;
    if ($("rttTxt")) $("rttTxt").textContent = Number(ms).toFixed(1) + " ms";
    const now = performance.now();
    if (now - lastRttNote < 200) return;
    lastRttNote = now;
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: "rtt", ms: ms }));
  }
  function applyEchoRtt(t0) {
    const stamp = Number(t0);
    if (!Number.isFinite(stamp) || stamp === lastEchoT0) return;
    const ms = performance.now() - stamp;
    if (ms < 0 || ms > 100) return;
    lastEchoT0 = stamp;
    noteRtt(ms);
  }
  function onPong(msg) {
    if (msg && msg.id != null && Number(msg.id) !== pingId) return;
    noteRtt(performance.now() - pingAt);
  }
  function sendPing() {
    if (!live || rttWorker) return;
    pingId += 1;
    pingAt = performance.now();
    const payload = JSON.stringify({ t: "ping", id: pingId, t_us: Math.floor(pingAt * 1000) });
    if (rttWs && rttWs.readyState === 1) {
      rttWs.send(payload);
      return;
    }
    if (ws && ws.readyState === 1) ws.send(payload);
  }
  function sendTouch() {
    if (!live) return;
    send({ t: "touch" });
  }
  function telemFresh() {
    return lastTelemAt && (performance.now() - lastTelemAt) < 220;
  }
  function setLink(ok, extra) {
    $("dotLink").className = "dot " + (ok ? "on" : "off");
    $("linkTxt").textContent = (ok ? "已连接" : "未连接") + (extra ? " · " + extra : "");
  }
  function setAck(text, ok) {
    const cls = "hint " + (ok ? "tone-ok" : "tone-bad");
    ["armAck", "connAck"].forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.textContent = text || "";
      el.className = cls;
    });
  }

  const pendingCmd = Object.create(null);
  const pulseTimers = new WeakMap();

  function pendingMs(op) {
    if (op === "power" || op === "link") return 12000;
    if (op === "home" || op === "path") return 20000;
    return 8000;
  }
  function pulseBtn(el) {
    if (!el) return;
    el.classList.add("flash-press");
    const prev = pulseTimers.get(el);
    if (prev) clearTimeout(prev);
    pulseTimers.set(el, setTimeout(() => {
      el.classList.remove("flash-press");
    }, 280));
  }
  function setBtnOn(id, on) {
    const el = $(id);
    if (!el) return;
    el.classList.toggle("is-on", !!on);
    el.classList.toggle("active", !!on);
    el.setAttribute("aria-pressed", on ? "true" : "false");
  }
  function setBusy(el, on, busyText) {
    if (!el) return;
    el.classList.toggle("busy", !!on);
    el.setAttribute("aria-busy", on ? "true" : "false");
    const idle = el.getAttribute("data-idle");
    if (on && busyText) el.textContent = busyText;
    else if (!on && idle) el.textContent = idle;
  }
  function beginCmd(op, spec) {
    if (pendingCmd[op]) {
      setAck("上一条指令还在等待应答", false);
      return false;
    }
    const id = spec.id;
    const pair = spec.pair || [id];
    pair.forEach((bid) => {
      const el = $(bid);
      if (!el) return;
      pulseBtn(el);
      const mine = bid === id;
      el.classList.toggle("is-on", mine);
      el.classList.toggle("active", mine);
      el.setAttribute("aria-pressed", mine ? "true" : "false");
      if (!mine) el.disabled = true;
    });
    setBusy($(id), true, spec.busy);
    pendingCmd[op] = { id: id, pair: pair, want: spec.want, t0: performance.now() };
    if (spec.hot) flashHot(spec.hot, spec.ack);
    if (spec.ack) setAck(spec.ack, spec.okTone !== false);
    return true;
  }
  function endCmd(op, ok, text) {
    const p = pendingCmd[op];
    if (p) {
      delete pendingCmd[op];
      (p.pair || [p.id]).forEach((bid) => {
        const el = $(bid);
        if (!el) return;
        el.disabled = false;
        setBusy(el, false);
      });
    }
    if (!text) return;
    if (p || !ok) setAck(text, ok);
  }
  function sweepPending() {
    const now = performance.now();
    Object.keys(pendingCmd).forEach((op) => {
      const p = pendingCmd[op];
      if (p && now - p.t0 > pendingMs(op)) {
        endCmd(op, false, "等待应答超时，请看状态或重试");
      }
    });
  }
  function ackText(op, ok, error, want) {
    if (!ok) return "失败：" + (error || "");
    if (op === "link") return want === "disconnect" ? "断开指令已执行" : "连接指令已执行";
    if (op === "power") return want ? "使能指令已执行，见右侧电机状态" : "去使能指令已执行";
    if (op === "home") return "复位已下发";
    if (op === "arm" || op === "cart_go") return "位置已下发";
    if (op === "path") return "轨迹已下发";
    if (op === "grip") return "夹爪指令已下发";
    if (op === "arm_src") return "臂源已切换";
    if (op === "clear") return "已解除急停";
    if (op === "mode") return "已切到 " + (ctrlMode || "");
    if (op === "script") return "已执行映射";
    return "已下发 " + (op || "");
  }
  function paintCmdButtons(a, estop) {
    sweepPending();
    const online = !!(a && a.online);
    const enabled = !!(a && a.enabled);
    const open = !!(a && a.gripper_open);
    if (!pendingCmd.link) {
      setBtnOn("btnArmConnect", online);
      setBtnOn("btnArmDisconnect", !online);
    }
    if (!pendingCmd.power) {
      setBtnOn("btnArmEnable", enabled);
      setBtnOn("btnArmDisable", online && !enabled);
    }
    setBtnOn("btnGripOpen", open);
    setBtnOn("btnGripClose", !open);
    if ($("btnEstop")) $("btnEstop").classList.toggle("is-on", !!estop);
    const follow = $("armFollow");
    const row = follow && follow.closest(".follow-row");
    if (row) row.classList.toggle("is-on", !!(follow && follow.checked));
  }

  let hotFlashTimer = 0;
  function flashHot(id, text) {
    document.querySelectorAll("[data-hot]").forEach((el) => {
      el.classList.toggle("hot", el.getAttribute("data-hot") === id);
    });
    const flash = $("hotkeyFlash");
    if (flash) {
      flash.hidden = false;
      flash.textContent = text || CMD_LABEL[id] || id;
      flash.classList.add("on");
    }
    clearTimeout(hotFlashTimer);
    hotFlashTimer = setTimeout(() => {
      document.querySelectorAll("[data-hot].hot").forEach((el) => el.classList.remove("hot"));
      if (flash) flash.classList.remove("on");
    }, 1400);
  }

  function setHotkeyPop(open) {
    const pop = $("hotkeyPop");
    const btn = $("btnHotkeys");
    if (!pop || !btn) return;
    pop.hidden = !open;
    btn.classList.toggle("open", open);
    btn.setAttribute("aria-expanded", open ? "true" : "false");
  }

  function buildJoints() {
    const box = $("jointList");
    box.innerHTML = "";
    for (let i = 0; i < 6; i++) {
      const [lo, hi] = limits[i] || DEFAULT_LIMITS[i];
      const row = document.createElement("div");
      row.className = "joint";
      row.innerHTML =
        "<label>J" + (i + 1) + "</label>" +
        "<input type='range' min='" + lo + "' max='" + hi + "' step='0.1' value='0' data-j='" + i + "'/>" +
        "<input type='number' step='0.1' value='0' data-jn='" + i + "'/>";
      box.appendChild(row);
    }
    box.oninput = (ev) => {
      const el = ev.target;
      if (el.dataset.j != null) {
        const i = Number(el.dataset.j);
        box.querySelector("[data-jn='" + i + "']").value = Number(el.value).toFixed(1);
      }
      if (el.dataset.jn != null) {
        const i = Number(el.dataset.jn);
        const range = box.querySelector("[data-j='" + i + "']");
        range.value = el.value;
      }
      maybeFollow();
    };
  }
  function readJoints() {
    return limits.map((_, i) => Number($("jointList").querySelector("[data-jn='" + i + "']").value || 0));
  }
  function applyLimits(arm) {
    if (!arm) return;
    if (Array.isArray(arm.limits) && arm.limits.length >= 6) {
      const next = arm.limits.slice(0, 6).map((p) => [Number(p[0]), Number(p[1])]);
      const changed = JSON.stringify(next) !== JSON.stringify(limits);
      limits = next;
      if (changed) buildJoints();
    }
    if (Array.isArray(arm.gripper_limits) && arm.gripper_limits.length === 2) {
      gripLimits = [Number(arm.gripper_limits[0]), Number(arm.gripper_limits[1])];
      const range = $("gripRange");
      const num = $("gripDeg");
      if (range) {
        range.min = gripLimits[0];
        range.max = gripLimits[1];
      }
      if (num) {
        num.min = gripLimits[0];
        num.max = gripLimits[1];
      }
      if (window.Arm3D && Arm3D.setGripRange) Arm3D.setGripRange(gripLimits[0], gripLimits[1]);
    }
  }
  function writeJointInputs(q) {
    (q || []).forEach((val, i) => {
      const r = $("jointList") && $("jointList").querySelector("[data-j='" + i + "']");
      const n = $("jointList") && $("jointList").querySelector("[data-jn='" + i + "']");
      if (!r || !n) return;
      r.value = val;
      n.value = Number(val).toFixed(1);
    });
  }
  function syncJointsFromSnap(arm) {
    if (!arm || !arm.q_deg) return;
    if (document.activeElement && $("jointList") && $("jointList").contains(document.activeElement)) return;
    if ($("armFollow") && $("armFollow").checked) return;
    if (jointDirty) {
      if (arm.moving) return;
      const q = readJoints();
      const near = q.length === 6 && arm.q_deg.length >= 6 &&
        q.every((v, i) => Math.abs(Number(v) - Number(arm.q_deg[i])) < 0.8);
      if (!near) return;
      jointDirty = false;
    }
    writeJointInputs(arm.q_deg);
  }
  function jointsPrimed() {
    return !!(last && last.arm && Array.isArray(last.arm.q_deg) && last.arm.q_deg.length >= 6);
  }
  function wantJointStream() {
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") return false;
    if (!jointsPrimed()) return false;
    return !!( $("armFollow") && $("armFollow").checked );
  }
  function stopFollowStream() {
    if ($("armFollow")) $("armFollow").checked = false;
    followDirty = false;
    const box = $("jointList");
    const el = document.activeElement;
    if (box && el && box.contains(el) && typeof el.blur === "function") el.blur();
    send({ t: "park" });
  }
  function paintGrip(open, deg) {
    const q = (last && last.arm && last.arm.q_deg) || [];
    if (window.Arm3D) Arm3D.setPose(q, deg, open);
  }
  function paintCommandedJoints(q) {
    const a = last && last.arm;
    if (window.Arm3D) Arm3D.setPose(q || [], a && a.gripper_deg, a && a.gripper_open);
  }
  function maybeFollow() {
    followDirty = true;
    if (!($("armFollow") && $("armFollow").checked)) jointDirty = true;
    sendFollowNow();
  }
  function sendFollowNow() {
    if (cartHeld.size) return;
    if (last && last.safety === "ESTOP_LATCHED") return;
    const q = readJoints();
    if (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") {
      paintCommandedJoints(q);
      send({ t: "teach", q: q });
      noteHz();
      return;
    }
    if (!wantJointStream()) return;
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") return;
    paintCommandedJoints(q);
    send({ t: "arm", q: q, speed: Number($("armSpeed").value || 40), stream: true });
    noteHz();
  }
  function followHot() {
    if (spaceMode === "cart") return false;
    if (cartHeld.size) return false;
    if (last && last.safety === "ESTOP_LATCHED") return false;
    const box = $("jointList");
    const el = document.activeElement;
    const sliding = !!(box && el && box.contains(el));
    if (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") return sliding || followDirty;
    if (!wantJointStream()) return false;
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") return false;
    return sliding || followDirty || ($("armFollow") && $("armFollow").checked);
  }
  function controlTick(dtSec) {
    if (!live || !ws || ws.readyState !== 1) return;
    const now = performance.now();
    const minDt = (1000 / controlHz) * 0.55;
    if (now - lastCtrl < minDt) return;
    const dt = Math.min(0.05, lastCtrl ? (now - lastCtrl) / 1000 : (dtSec || 1 / controlHz));
    lastCtrl = now;
    if (cartHeld.size) {
      cartPulse(dt);
      return;
    }
    if (!followHot()) return;
    followDirty = false;
    sendFollowNow();
  }
  function startControlLoop(hz) {
    controlHz = Math.max(20, Math.min(200, Number(hz) || 100));
    if (controlTimer) clearInterval(controlTimer);
    if (controlRaf) cancelAnimationFrame(controlRaf);
    lastCtrl = 0;
    const tick = () => controlTick();
    controlTimer = setInterval(tick, 1000 / controlHz);
    function raf(now) {
      controlRaf = requestAnimationFrame(raf);
      if (!lastCtrl || now - lastCtrl >= (1000 / controlHz) - 2) tick();
    }
    controlRaf = requestAnimationFrame(raf);
  }
  function fmtCart(v, digits, unit) {
    if (v == null || v === "" || Number.isNaN(Number(v))) return "—";
    return Number(v).toFixed(digits) + unit;
  }
  function paintCartPose(a) {
    const xyz = (a && a.ee_m) || [];
    const rpy = (a && a.ee_rpy_deg) || [];
    if ($("cartPose")) {
      $("cartPose").textContent =
        "实测  X " + fmtCart(xyz[0], 3, "") +
        "  Y " + fmtCart(xyz[1], 3, "") +
        "  Z " + fmtCart(xyz[2], 3, "") + " m" +
        "　R " + fmtCart(rpy[0], 1, "") +
        "  P " + fmtCart(rpy[1], 1, "") +
        "  Yaw " + fmtCart(rpy[2], 1, "") + "°";
    }
    syncCartFromSnap(a);
  }
  function clipCartAxis(i, raw) {
    const ax = CART_AXES[i];
    const v = Number(raw);
    if (!Number.isFinite(v)) return ax.def;
    return Math.max(ax.min, Math.min(ax.max, v));
  }
  function writeCartInputs(xyz, rpy) {
    const box = $("cartList");
    if (!box) return;
    const vals = [
      xyz && xyz[0] != null ? Number(xyz[0]) : CART_AXES[0].def,
      xyz && xyz[1] != null ? Number(xyz[1]) : CART_AXES[1].def,
      xyz && xyz[2] != null ? Number(xyz[2]) : CART_AXES[2].def,
      rpy && rpy[0] != null ? Number(rpy[0]) : CART_AXES[3].def,
      rpy && rpy[1] != null ? Number(rpy[1]) : CART_AXES[4].def,
      rpy && rpy[2] != null ? Number(rpy[2]) : CART_AXES[5].def,
    ];
    vals.forEach((val, i) => {
      const ax = CART_AXES[i];
      const clipped = clipCartAxis(i, val);
      const r = box.querySelector("[data-c='" + i + "']");
      const n = box.querySelector("[data-cn='" + i + "']");
      if (!r || !n) return;
      r.value = clipped;
      n.value = Number(clipped).toFixed(ax.digits);
    });
  }
  function syncCartFromSnap(arm) {
    if (!arm) return;
    const box = $("cartList");
    if (!box) return;
    if (document.activeElement && box.contains(document.activeElement)) return;
    if (cartDirty) {
      const pose = readCart();
      if (arm.moving || !cartNearMeasured(arm, pose.xyz, pose.rpy)) return;
      cartDirty = false;
    }
    writeCartInputs(arm.ee_m, arm.ee_rpy_deg);
  }
  function readCart() {
    const box = $("cartList");
    const xyz = [];
    const rpy = [];
    for (let i = 0; i < 6; i++) {
      const n = box && box.querySelector("[data-cn='" + i + "']");
      const v = clipCartAxis(i, n ? n.value : CART_AXES[i].def);
      if (i < 3) xyz.push(v);
      else rpy.push(v);
    }
    return { xyz: xyz, rpy: rpy };
  }
  function cartNearMeasured(arm, xyz, rpy) {
    const m = (arm && arm.ee_m) || [];
    const r = (arm && arm.ee_rpy_deg) || [];
    for (let i = 0; i < 3; i++) {
      if (Math.abs(Number(xyz[i]) - Number(m[i] || 0)) > 0.0015) return false;
    }
    for (let i = 0; i < 3; i++) {
      let d = Math.abs(Number(rpy[i]) - Number(r[i] || 0)) % 360;
      if (d > 180) d = 360 - d;
      if (d > 0.8) return false;
    }
    return true;
  }
  function sendCartPose() {
    pulseBtn($("btnSendArm"));
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") {
      setAck("请先切回 Position 再发送笛卡尔目标", false);
      return;
    }
    const pose = readCart();
    const arm = last && last.arm;
    if (arm && cartNearMeasured(arm, pose.xyz, pose.rpy) && !arm.moving) {
      setAck("目标就是当前末端位姿。先拖滑条再发送，或用 WASD。", false);
      return;
    }
    cartDirty = true;
    send({ t: "cart_go", xyz: pose.xyz, rpy: pose.rpy, speed: Number($("armSpeed").value || 40) });
    flashHot("send", "发送位置");
    setAck("笛卡尔目标已按 Speed 下发", true);
  }
  function bindCartSliders() {
    const box = $("cartList");
    if (!box) return;
    box.innerHTML = "";
    CART_AXES.forEach((ax, i) => {
      const row = document.createElement("div");
      row.className = "joint";
      row.innerHTML =
        "<label>" + ax.label + "</label>" +
        "<input type='range' min='" + ax.min + "' max='" + ax.max + "' step='" + ax.step +
        "' value='" + ax.def + "' data-c='" + i + "'/>" +
        "<input type='number' min='" + ax.min + "' max='" + ax.max + "' step='" + ax.step +
        "' value='" + Number(ax.def).toFixed(ax.digits) + "' data-cn='" + i + "'/>" +
        "<span class='cart-unit'>" + ax.unit + "</span>";
      box.appendChild(row);
    });
    const syncPair = (i, raw) => {
      const ax = CART_AXES[i];
      const v = clipCartAxis(i, raw);
      const r = box.querySelector("[data-c='" + i + "']");
      const n = box.querySelector("[data-cn='" + i + "']");
      if (r) r.value = String(v);
      if (n) n.value = Number(v).toFixed(ax.digits);
    };
    box.addEventListener("input", (ev) => {
      const el = ev.target;
      if (el.dataset.c != null) syncPair(Number(el.dataset.c), el.value);
      if (el.dataset.cn != null) syncPair(Number(el.dataset.cn), el.value);
    });
    box.addEventListener("change", (ev) => {
      const el = ev.target;
      if (el.dataset.c == null && el.dataset.cn == null) return;
      sendCartPose();
    });
  }
  function setSpace(name, fromUi) {
    if (name === "cart" && (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri")) {
      setAck("Gravity 示教请用关节空间。笛卡尔请先切回 Position。", false);
      name = "joint";
    }
    spaceMode = name === "cart" ? "cart" : "joint";
    if (spaceMode === "cart" && $("armFollow")) $("armFollow").checked = false;
    if ($("jointSpace")) $("jointSpace").classList.toggle("off", spaceMode !== "joint");
    if ($("cartSpace")) $("cartSpace").classList.toggle("off", spaceMode !== "cart");
    if ($("spaceBtns")) {
      $("spaceBtns").querySelectorAll("[data-space]").forEach((b) => {
        const on = b.dataset.space === spaceMode;
        b.classList.toggle("active", on);
        b.classList.toggle("is-on", on);
        if (fromUi && on) pulseBtn(b);
      });
    }
    if ($("spaceHint")) {
      $("spaceHint").textContent = spaceMode === "cart"
        ? "笛卡尔：拖 X/Y/Z 与姿态滑条。松手或「发送位置」后按 Speed 走到目标（直线 IK）。键盘 WASD 仍可用。"
        : "关节空间：拖 J1–J6 改目标，点「发送位置」下发；勾选「实时跟随」后拖滑条才发 servo。";
    }
    if (spaceMode === "cart" && last && last.arm) {
      cartDirty = false;
      writeCartInputs(last.arm.ee_m, last.arm.ee_rpy_deg);
    }
    if (fromUi) setAck(spaceMode === "cart" ? "已切到笛卡尔滑条控制" : "已切到关节空间", true);
  }
  function sendPosition() {
    if (spaceMode === "cart") {
      sendCartPose();
      return;
    }
    pulseBtn($("btnSendArm"));
    if (ctrlMode !== "Position") {
      setAck("请先切回 Position 再发送位置", false);
      return;
    }
    stopFollowStream();
    const q = readJoints();
    const cur = (last && last.arm && last.arm.q_deg) || [];
    const same = q.length === 6 && cur.length >= 6 && q.every((v, i) => Math.abs(Number(v) - Number(cur[i])) < 0.2);
    if (same) {
      setAck("目标就是当前角，臂不会动。先拖滑条再发送，或勾选实时跟随。", false);
      return;
    }
    jointDirty = true;
    send({ t: "arm", q: q, speed: Number($("armSpeed").value || 40) });
    flashHot("send", "发送位置");
    setAck("关节目标已下发", true);
  }

  function kv(title, rows) {
    const lines = rows.map(([k, v, cls]) => {
      const c = cls ? " class='" + cls + "'" : "";
      return "<tr><td>" + k + "</td><td" + c + ">" + v + "</td></tr>";
    }).join("");
    return "<div class='kv'><h3>" + title + "</h3><table>" + lines + "</table></div>";
  }
  function metric(k, v, s) {
    return "<div class='metric'><div class='k'>" + k + "</div><div class='v'>" + v + "</div><div class='s'>" + (s || "") + "</div></div>";
  }
  function paintWaypoints() {
    const box = $("wpList");
    if (!box) return;
    if (!waypoints.length) {
      box.innerHTML = "<li>还没有路点</li>";
      return;
    }
    box.innerHTML = waypoints.map((w, i) =>
      "<li>#" + (i + 1) + "　" + w.q.map((x) => Number(x).toFixed(0)).join("，") +
      "°　<input type='number' min='0.2' step='0.1' value='" + Number(w.t || 2).toFixed(1) +
      "' data-wpt='" + i + "' style='width:70px'/> s</li>"
    ).join("");
    box.querySelectorAll("[data-wpt]").forEach((el) => {
      el.addEventListener("change", () => {
        const i = Number(el.dataset.wpt);
        if (waypoints[i]) waypoints[i].t = Number(el.value || 2);
      });
    });
  }

  function render(d) {
    last = d;
    const a = d.arm || {};
    applyLimits(a);
    if (a.ctrl_mode) ctrlMode = a.ctrl_mode;
    paintCtrlMode(a);
    if (currentPage() === "arm") {
      syncJointsFromSnap(a);
      if ($("gripDeg") && a.gripper_deg != null && document.activeElement !== $("gripDeg") && document.activeElement !== $("gripRange")) {
        $("gripDeg").value = Number(a.gripper_deg).toFixed(1);
        if ($("gripRange")) $("gripRange").value = a.gripper_deg;
      }
      if ($("gripEffort") && a.gripper_effort != null && document.activeElement !== $("gripEffort") && document.activeElement !== $("gripEffortRange")) {
        $("gripEffort").value = clipGripEffort(a.gripper_effort);
        if ($("gripEffortRange")) $("gripEffortRange").value = $("gripEffort").value;
        paintGripEffortNm();
      }
    }
    if (!telemFresh() && window.Arm3D) Arm3D.setPose(a.q_deg || [], a.gripper_deg, a.gripper_open);
    paintCartPose(a);
    if (d.echo_t0 != null) applyEchoRtt(d.echo_t0);
    const estop = d.safety === "ESTOP_LATCHED";
    $("estopBanner").classList.toggle("on", estop);
    $("safeTxt").textContent = d.safety || "—";
    if (d.cmd_err) setAck(d.cmd_err, false);
    if ($("kbAck")) {
      if (a.ik_err) {
        $("kbAck").textContent = a.ik_err;
        $("kbAck").className = "hint tone-bad";
        if ($("kbPadHint")) {
          $("kbPadHint").textContent = a.ik_err;
          $("kbPadHint").className = "hint tone-bad";
        }
      } else if ($("kbAck").classList.contains("tone-bad")) {
        $("kbAck").textContent = "按键映射与 SDK test_fafu_keyboard_cartesian 相同。";
        $("kbAck").className = "hint";
        if ($("kbPadHint")) {
          $("kbPadHint").textContent = "拖动只改目标。松手或「发送位置」后 IK + move_j，速度用上方 Speed。键盘 WASD 仍是增量遥操作。";
          $("kbPadHint").className = "hint";
        }
      }
    }
    const now = performance.now();
    if (now - lastDom < 50) return;
    lastDom = now;
    paintHud(d, a, estop);
  }

  function floatModeNeedsDynamics(mode) {
    return mode === "Gravity" || mode === "Gra+Fri" || mode === "Impedance";
  }
  function floatModesOk(arm) {
    if (!arm) return false;
    if (arm.float_ok != null) return !!arm.float_ok;
    if (arm.backend === "sim" || arm.backend === "mock") return true;
    return !!arm.dyn_ready;
  }
  function paintCtrlMode(arm) {
    const a = arm || {};
    const ok = floatModesOk(a);
    const reason = a.float_reason || "需 pinocchio";
    const box = $("modeBtns");
    if (box) {
      box.querySelectorAll("[data-mode]").forEach((btn) => {
        const mode = btn.dataset.mode;
        const blocked = floatModeNeedsDynamics(mode) && !ok;
        btn.disabled = blocked;
        btn.setAttribute("aria-disabled", blocked ? "true" : "false");
        btn.title = blocked ? (mode + "：" + reason + "（真机力矩环未加载）") : "";
        btn.classList.toggle("active", mode === ctrlMode);
      });
    }
    if ($("armCtrlMode")) $("armCtrlMode").textContent = ctrlMode;
    const hints = {
      Position: "Position：关节/笛卡尔滑条 + 发送位置。实时跟随与键盘走 servo。",
      Gravity: "Gravity：真机为可手拖力矩环。仿真用滑条改当前角。夹爪打开。",
      "Gra+Fri": "Gra+Fri：真机重力+摩擦补偿。仿真与 Gravity 相同示教语义。",
      Impedance: "Impedance：真机柔顺保持进入时的姿态。仿真为较慢跟随。",
    };
    if ($("modeHint")) $("modeHint").textContent = hints[ctrlMode] || "";
    const cap = $("modeCapHint");
    if (cap) {
      cap.hidden = ok;
      cap.textContent = ok
        ? ""
        : "Gravity / Gra+Fri / Impedance 已禁用：" + reason + "（真机力矩环未加载）。仿真臂仍可切这些模式做滑条示教。";
    }
  }
  function paintArmSrc(backend) {
    if (pendingCmd.arm_src) return;
    const src = backend === "sim" ? "sim" : (backend === "fafu" ? "fafu" : "");
    document.querySelectorAll("#armSrcBtns [data-arm-src]").forEach((b) => {
      b.classList.toggle("active", b.getAttribute("data-arm-src") === src);
      b.classList.toggle("is-on", b.getAttribute("data-arm-src") === src);
    });
  }
  function setArmSrc(mode) {
    const id = mode === "sim" ? "btnArmSim" : "btnArmLive";
    stopFollowStream();
    if (!beginCmd("arm_src", {
      id: id,
      pair: ["btnArmLive", "btnArmSim"],
      busy: mode === "sim" ? "切换仿真…" : "切换真机…",
      ack: mode === "sim" ? "正在切到仿真臂…" : "正在切到真机 USB…",
      want: mode,
    })) return;
    send({ t: "arm_src", mode: mode });
  }
  function paintMotorStatus(a) {
    const table = $("motorTable");
    const hint = $("motorHint");
    if (!table) return;
    const motors = (a && a.motors) || [];
    if (!motors.length) {
      table.innerHTML = "<tr><td colspan='4'>未读到电机（先 Connect，再点使能）</td></tr>";
      if (hint) hint.textContent = "点「使能」后查看各轴是否在线、有无故障、当前模式。";
      return;
    }
    const head = "<tr><td>轴</td><td>通信</td><td>故障</td><td>模式</td></tr>";
    const rows = motors.map((m) => {
      const online = !!m.online;
      const ok = m.ok !== false && !m.fault;
      const mode = m.mode || "—";
      return (
        "<tr>" +
        "<td>" + (m.name || ("M" + m.id)) + "</td>" +
        "<td class='" + (online ? "st-on" : "st-off") + "'>" + (online ? "在线" : "离线") + "</td>" +
        "<td class='" + (ok ? "st-on" : "st-off") + "'>" + (ok ? "正常" : ("故障 " + (m.fault || ""))) + "</td>" +
        "<td>" + mode + "</td>" +
        "</tr>"
      );
    });
    table.innerHTML = head + rows.join("");
    if (hint) {
      const nOk = motors.filter((m) => m.online && m.ok !== false && !m.fault).length;
      hint.textContent = a && a.enabled
        ? ("已使能 · " + nOk + "/" + motors.length + " 轴在线正常")
        : ("未使能 · " + nOk + "/" + motors.length + " 轴有通信。点「使能」切入位置环。");
    }
  }
  function paintHud(d, a, estop) {
    const cam = d.camera || {};
    $("cliTxt").textContent = String(d.clients || 0);
    if (rtt == null && d.rtt_ms != null) $("rttTxt").textContent = Number(d.rtt_ms).toFixed(1) + " ms";
    const demo = (a.backend || "mock") === "mock" || a.backend === "sim";
    const camLive = (cam.backend || "mock") !== "mock";
    const bits = [demo ? (a.backend === "sim" ? "仿真臂" : "假臂") : "真臂", camLive ? "真相机" : "假相机"];
    if ($("mockTxt")) $("mockTxt").textContent = bits.join(" · ");
    if ($("mockDot")) $("mockDot").className = "dot " + ((demo || !camLive) ? "warn" : "on");
    if (info && $("homeHint")) {
      const ip = info.host_ip + ":" + info.http_port;
      const guard = info.live_serial_allowed ? "" : "　未允许真机串口。";
      $("homeHint").textContent =
        "本站在本机运行。http://" + ip +
        "　USB 直连机械臂与作业相机。" + guard;
    }
    if ($("armModeBadge")) {
      const demo = (a.backend || "mock") === "mock" || a.backend === "sim";
      const online = a.online !== false;
      $("armModeBadge").textContent = !online
        ? "已断开"
        : (demo ? (a.backend === "sim" ? "Sim Mode" : "Demo Mode") : "Live Robot");
      $("armDot").className = "status-dot" + (online ? (demo ? " connected" : " live") : " off");
    }
    if ($("armBackend")) $("armBackend").textContent = a.backend || "—";
    if ($("armLinkTxt")) $("armLinkTxt").textContent = a.online ? "已连接" : "已断开";
    if ($("armEnabled")) $("armEnabled").textContent = a.enabled ? "是" : "否";
    if ($("armGripTxt")) $("armGripTxt").textContent = a.gripper_open ? "Open" : "Close";
    if ($("armRobotName")) $("armRobotName").textContent = "FAFU";
    if (info && $("armServerUrl")) $("armServerUrl").textContent = info.arm_port || "cfg / auto";
    if ($("armCtrlMode")) $("armCtrlMode").textContent = ctrlMode;
    paintArmSrc(a.backend);
    paintMotorStatus(a);
    paintCmdButtons(a, estop);
    if ($("collectBanner")) {
      if (a.backend === "sim") {
        $("collectBanner").classList.add("on");
        $("collectBanner").textContent = "当前是仿真臂：不开 USB，页面操作只驱动仿真。可在右侧切回「真机 USB」。";
      } else if (a.backend === "fafu" && info && info.live_serial_allowed && info.arm_allow_motion) {
        $("collectBanner").classList.remove("on");
        $("collectBanner").textContent = "";
      }
    }
    $("homeMetrics").innerHTML =
      metric("臂", (a.q_deg || []).map((x) => Number(x).toFixed(0) + "°").join("  ") || "—", a.moving ? "运动中" : (a.gripper_open ? "夹爪开" : "夹爪合")) +
      metric("安全", d.safety || "—", estop ? "需解除" : "可运动") +
      metric("相机", camLive ? shortCamName(cam.device || "USB") : "—", camLive ? "已自动识别" : (cam.reason || "未连接"));
    const rows = (a.q_deg || []).map((q, i) => [
      "J" + (i + 1),
      Number(q).toFixed(2) + "°  →  " + Number((a.target_deg || [])[i] || 0).toFixed(2) + "°",
      (a.ok && a.ok[i] === false) ? "tone-bad" : "tone-ok",
    ]);
    rows.push(["夹爪", (a.gripper_open ? "打开" : "闭合") + " · " + Number(a.gripper_effort || 300) + " raw"]);
    rows.push(["后端", a.backend || "—"]);
    rows.push(["使能", a.enabled ? "是" : "否"]);
    $("armKv").innerHTML = kv("实测角度", rows);
    if ($("eeKv")) {
      const tau = (a.tau_raw || []).map((x) => Number(x)).join("  ") || "—";
      const xyz = (a.ee_m && a.ee_m.length === 3)
        ? a.ee_m.map((x) => Number(x).toFixed(3)).join("  ") + " m"
        : "等待快照";
      const rpy = (a.ee_rpy_deg && a.ee_rpy_deg.length === 3)
        ? a.ee_rpy_deg.map((x) => Number(x).toFixed(1)).join("  ") + " °"
        : "—";
      $("eeKv").innerHTML = kv("位姿 / 力矩", [
        ["X Y Z", xyz],
        ["R P Y", rpy],
        ["外力", "无腕部 F/T"],
        ["关节力矩 raw", tau],
        ["动力学", a.dyn_ready ? "URDF 同链" : "无 pinocchio（笛卡尔走站控 IK；重力不可用）"],
        ["力矩模式", a.float_ok ? "可用" : ((a.float_reason || "需 pinocchio") + "（已禁用）")],
        ["重力环", a.grav_active ? "运行中" : "关"],
      ]);
    }
    const roles = cam.roles || ["作业相机"];
    const fps = cam.fps || [];
    roles.forEach((name, i) => {
      const fpsTxt = (fps[i] != null ? Number(fps[i]).toFixed(1) : "—") + " Hz";
      let label = name;
      if (i === 0 && cam.device) label = "作业相机 · " + shortCamName(cam.device);
      [["camRole", "camFps"], ["homeRole", "homeFps"]].forEach(([rk, fk]) => {
        const r = $(rk + i);
        const f = $(fk + i);
        if (r) r.textContent = label;
        if (f) f.textContent = fpsTxt;
      });
    });
    const camOk = camLive && (cam.online || [])[0];
    document.querySelectorAll(".js-cam-overlay").forEach((ov) => {
      if (camOk) ov.style.display = "none";
      else {
        ov.style.display = "";
        ov.textContent = cam.reason || (camLive ? "等待画面" : "未连接 · 假画面");
      }
    });
  }

  function shortCamName(s) {
    return String(s || "")
      .replace(/Intel\(R\)\s*/ig, "")
      .replace(/RealSense\(TM\)\s*/ig, "")
      .replace(/\s+Depth\s*$/i, "")
      .trim() || "USB";
  }

  function applyPng(sid, blob) {
    const url = URL.createObjectURL(blob);
    const prev = urls[sid];
    ["cam" + sid, "homeCam" + sid].forEach((id) => {
      const img = $(id);
      if (img) img.src = url;
    });
    urls[sid] = url;
    if (prev) setTimeout(() => URL.revokeObjectURL(prev), 200);
  }

  function parseTelem(buf) {
    if (!(buf instanceof ArrayBuffer) || buf.byteLength < TELEM_SIZE) return null;
    const v = new DataView(buf);
    if (v.getUint32(0, true) !== TELEM_MAGIC) return null;
    const flags = v.getUint16(8, true);
    const safetyCode = v.getUint8(10);
    const echo = v.getFloat64(12, true);
    const q = [];
    const tgt = [];
    const ee = [];
    for (let i = 0; i < 6; i++) {
      q.push(v.getFloat32(20 + 4 * i, true));
      tgt.push(v.getFloat32(44 + 4 * i, true));
    }
    for (let i = 0; i < 3; i++) ee.push(v.getFloat32(72 + 4 * i, true));
    const names = { 0: "IDLE", 1: "OPERATING", 2: "ESTOP_LATCHED", 3: "WATCHDOG" };
    return {
      seq: v.getUint32(4, true),
      echo_t0: Number.isFinite(echo) ? echo : null,
      cmd_hz: v.getFloat32(84, true),
      safety: names[safetyCode] || "IDLE",
      arm: {
        online: !!(flags & 1),
        enabled: !!(flags & 2),
        moving: !!(flags & 4),
        gripper_open: !!(flags & 8),
        ik_err: (flags & 16) ? (last && last.arm && last.arm.ik_err) || "IK" : "",
        q_deg: q,
        target_deg: tgt,
        gripper_deg: v.getFloat32(68, true),
        ee_m: ee,
      },
    };
  }
  function mergeSnap(patch) {
    const prev = last || {};
    const arm = Object.assign({}, prev.arm || {}, patch.arm || {});
    return Object.assign({}, prev, patch, { arm: arm });
  }
  function flushSnap() {
    snapRaf = 0;
    const raw = pendingTelem;
    pendingTelem = null;
    if (!raw) return;
    const t = parseTelem(raw);
    if (!t) return;
    last = mergeSnap(t);
    lastTelemAt = performance.now();
    const a = last.arm || {};
    if (window.Arm3D) Arm3D.setPose(a.q_deg || [], a.gripper_deg, a.gripper_open);
    if (last.echo_t0 != null) applyEchoRtt(last.echo_t0);
    if (teleopHot()) return;
    if (currentPage() === "arm") syncJointsFromSnap(a);
    paintCartPose(a);
    const estop = last.safety === "ESTOP_LATCHED";
    $("estopBanner").classList.toggle("on", estop);
    $("safeTxt").textContent = last.safety || "—";
  }
  function connectBus() {
    if (ws) try { ws.close(); } catch (e) {}
    ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/cmd");
    ws.onopen = () => setLink(true, "");
    ws.onclose = () => {
      setLink(false, "");
      if (live) setTimeout(connectBus, 800);
    };
    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.t === "pong") {
        onPong(msg);
        return;
      }
      if (msg.t === "hello" && msg.cid) myCid = msg.cid;
      if (msg.t === "snap") return;
      if (msg.t === "ack") {
        const want = pendingCmd[msg.op] && pendingCmd[msg.op].want;
        if (msg.op === "clear") {
          endCmd("clear", !!msg.ok, msg.ok ? "已解除急停" : (msg.error || "失败"));
          if (!msg.ok) alert(msg.error || "失败");
        } else if (msg.op === "script") {
          endCmd("script", !!msg.ok, msg.ok ? "已执行映射" : ("失败：" + (msg.error || "")));
          if ($("scriptAck")) {
            $("scriptAck").textContent = msg.ok ? "已执行映射" : ("失败：" + (msg.error || ""));
            $("scriptAck").className = "hint " + (msg.ok ? "tone-ok" : "tone-bad");
          }
        } else if (msg.op === "cart" || msg.op === "teach") {
          if (!msg.ok) {
            setAck("失败：" + (msg.error || ""), false);
            if ($("kbAck") && msg.op === "cart") {
              $("kbAck").textContent = msg.error || "键盘失败";
              $("kbAck").className = "hint tone-bad";
            }
          } else if (msg.op === "cart" && $("kbAck") && $("kbAck").classList.contains("tone-bad")) {
            $("kbAck").textContent = "已恢复";
            $("kbAck").className = "hint tone-ok";
          }
        } else if (msg.op === "power") {
          endCmd("power", !!msg.ok, ackText("power", !!msg.ok, msg.error, want));
          if (msg.ok && last && last.arm) paintMotorStatus(last.arm);
        } else if (msg.op === "arm_src") {
          endCmd("arm_src", !!msg.ok, ackText("arm_src", !!msg.ok, msg.error, want));
        } else {
          endCmd(msg.op, !!msg.ok, ackText(msg.op, !!msg.ok, msg.error, want));
        }
      }
    };
  }
  function connectTelem() {
    if (telemWs) try { telemWs.close(); } catch (e) {}
    telemWs = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws");
    telemWs.binaryType = "arraybuffer";
    telemWs.onclose = () => { if (live) setTimeout(connectTelem, 800); };
    telemWs.onmessage = (ev) => {
      if (ev.data instanceof ArrayBuffer) {
        pendingTelem = ev.data;
        if (!snapRaf) snapRaf = requestAnimationFrame(flushSnap);
        return;
      }
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.t === "hud" && msg.d) {
        if (teleopHot()) {
          last = mergeSnap(msg.d);
          if (msg.d.cmd_err) setAck(msg.d.cmd_err, false);
          const a = last.arm || {};
          if (a.ik_err && $("kbAck")) {
            $("kbAck").textContent = a.ik_err;
            $("kbAck").className = "hint tone-bad";
          }
          return;
        }
        render(mergeSnap(msg.d));
      }
    };
  }
  function connectRttSocket() {
    if (rttWs) try { rttWs.close(); } catch (e) {}
    rttWs = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/rtt");
    rttWs.onclose = () => { if (live && !rttWorker) setTimeout(connectRttSocket, 1000); };
    rttWs.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.t === "pong") onPong(msg);
    };
  }
  function connectRtt() {
    if (rttWorker) try { rttWorker.terminate(); } catch (e) {}
    rttWorker = null;
    if (rttBlobUrl) try { URL.revokeObjectURL(rttBlobUrl); } catch (e) {}
    rttBlobUrl = "";
    if (rttWs) try { rttWs.close(); } catch (e) {}
    rttWs = null;
    const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/rtt";
    const src = [
      "let ws, pingId=0, pingAt=0, timer=0;",
      "function ping(){ if(!ws||ws.readyState!==1) return; pingId++; pingAt=performance.now(); ws.send(JSON.stringify({t:'ping',id:pingId,t_us:Math.floor(pingAt*1000)})); }",
      "function connect(){",
      "  ws=new WebSocket(" + JSON.stringify(url) + ");",
      "  ws.onmessage=function(ev){ try{ var m=JSON.parse(ev.data);}catch(e){return;} if(m.t==='pong'&&m.id===pingId) postMessage({ms:performance.now()-pingAt}); };",
      "  ws.onclose=function(){ if(timer){clearInterval(timer);timer=0;} setTimeout(connect,1000); };",
      "  ws.onopen=function(){ if(!timer) timer=setInterval(ping,250); ping(); };",
      "}",
      "connect();",
    ].join("");
    try {
      rttBlobUrl = URL.createObjectURL(new Blob([src], { type: "application/javascript" }));
      rttWorker = new Worker(rttBlobUrl);
      rttWorker.onmessage = (ev) => { if (ev.data && ev.data.ms != null) noteRtt(ev.data.ms); };
      rttWorker.onerror = () => { try { rttWorker.terminate(); } catch (e) {} rttWorker = null; connectRttSocket(); };
    } catch (e) {
      connectRttSocket();
    }
  }
  function connectVideo() {
    if (vws) try { vws.close(); } catch (e) {}
    vws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/video");
    vws.binaryType = "arraybuffer";
    vws.onmessage = (ev) => {
      if (cartHeld.size) return;
      if ($("armFollow") && $("armFollow").checked) return;
      const cams = $("panel-cams");
      if (currentPage() === "arm" && cams && cams.classList.contains("off")) return;
      const buf = ev.data;
      if (!(buf instanceof ArrayBuffer) || buf.byteLength < VID_HDR) return;
      const v = new DataView(buf);
      if (v.getUint8(0) !== 0xA1) return;
      const sid = v.getUint8(1);
      const len = v.getUint32(8);
      const payload = buf.slice(VID_HDR, VID_HDR + len);
      applyPng(sid, new Blob([payload], { type: "image/png" }));
    };
    vws.onclose = () => { if (live) setTimeout(connectVideo, 1000); };
  }

  async function boot() {
    live = true;
    ensureDefaultHash();
    showPage();
    syncStage();
    const r = await fetch("/api/info");
    if (r.ok) info = await r.json();
    if (info) $("brandSub").textContent = "控制 " + info.control_hz + "Hz · 视频 " + info.video_hz + "Hz · " + info.host_ip;
    if ($("collectBanner")) {
      const liveArm = !!(info && info.live_serial_allowed);
      const motion = !!(info && info.arm_allow_motion);
      $("collectBanner").classList.toggle("on", !(liveArm && motion));
      $("collectBanner").textContent = liveArm
        ? (motion ? "" : "已允许开串口，但 arm_allow_motion=false：真臂只读，不会使能或下发。")
        : "未允许真机串口：本站不开真臂 USB。当前运动只作用于仿真/假臂。";
      if (liveArm && motion) $("collectBanner").classList.remove("on");
    }
    await loadScripts();
    startControlLoop(info && info.control_hz);
    connectBus();
    connectTelem();
    connectRtt();
    connectVideo();
  }

  $("btnEstop").onclick = () => fireEstop();
  $("btnClear").onclick = () => fireClearEstop();
  function syncSpeed(fromRange) {
    const range = $("armSpeedRange");
    const num = $("armSpeed");
    if (fromRange) num.value = range.value;
    else range.value = num.value;
  }
  $("armSpeedRange").addEventListener("input", () => syncSpeed(true));
  $("armSpeed").addEventListener("input", () => syncSpeed(false));
  $("btnSendArm").onclick = sendPosition;
  function fireEstop() {
    cartHeld.clear();
    stopFollowStream();
    send({ t: "e" });
    pulseBtn($("btnEstop"));
    if ($("btnEstop")) $("btnEstop").classList.add("is-on");
    flashHot("estop", "急停");
    setAck("急停：电机保持当前姿态（不是断电）", false);
  }
  function fireClearEstop() {
    if (!beginCmd("clear", {
      id: "btnClear",
      busy: "解除中…",
      hot: "clear",
      ack: "已请求解除急停",
    })) return;
    send({ t: "clear" });
  }
  function goHome() {
    pulseBtn($("btnHome"));
    if (ctrlMode !== "Position") { setAck("请先切回 Position", false); return; }
    if (!beginCmd("home", {
      id: "btnHome",
      busy: "复位中…",
      hot: "home",
      ack: "复位：按当前姿态连续回零",
    })) return;
    stopFollowStream();
    jointDirty = false;
    send({ t: "home", speed: Number($("armSpeed").value || 40) });
  }
  function firePower(on) {
    const id = on ? "btnArmEnable" : "btnArmDisable";
    if (!beginCmd("power", {
      id: id,
      pair: ["btnArmEnable", "btnArmDisable"],
      busy: on ? "正在使能…" : "正在去使能…",
      hot: on ? "enable" : "disable",
      ack: on ? "正在使能，随后显示各轴在线状态…" : "正在去使能…",
      want: on,
    })) return;
    send({ t: "power", on: on });
  }
  function fireLink(action) {
    const connect = action === "connect";
    const id = connect ? "btnArmConnect" : "btnArmDisconnect";
    if (!beginCmd("link", {
      id: id,
      pair: ["btnArmConnect", "btnArmDisconnect"],
      busy: connect ? "正在连接…" : "正在断开…",
      hot: connect ? "connect" : "disconnect",
      ack: connect ? "正在连接…" : "正在断开…",
      want: action,
    })) return;
    send({ t: "link", action: action });
  }
  function fireHotCmd(name) {
    if (name === "home") {
      goHome();
      return;
    }
    if (name === "clear") {
      fireClearEstop();
      return;
    }
    if (name === "send") {
      sendPosition();
      return;
    }
    if (name === "enable") {
      firePower(true);
      return;
    }
    if (name === "disable") {
      firePower(false);
      return;
    }
    if (name === "grip-toggle") {
      toggleGripper();
      return;
    }
    if (name === "grip-open") {
      sendGripOpen(true);
      return;
    }
    if (name === "grip-close") {
      sendGripOpen(false);
      return;
    }
    if (name === "follow") {
      const box = $("armFollow");
      if (!box) return;
      box.checked = !box.checked;
      const row = box.closest(".follow-row");
      if (row) row.classList.toggle("is-on", box.checked);
      if (!box.checked) {
        stopFollowStream();
        flashHot("follow", "跟随关");
        setAck("已关闭实时跟随", true);
      } else {
        jointDirty = false;
        sendFollowNow();
        flashHot("follow", "跟随开");
        setAck("已打开实时跟随", true);
      }
      return;
    }
    if (name === "joint") {
      setSpace("joint", true);
      flashHot("joint", "关节空间");
      return;
    }
    if (name === "cartspace") {
      setSpace("cart", true);
      flashHot("cartspace", "笛卡尔空间");
    }
  }
  $("btnHome").onclick = goHome;
  if ($("btnHomeCart")) $("btnHomeCart").onclick = goHome;
  if ($("armFollow")) {
    $("armFollow").addEventListener("change", () => {
      const row = $("armFollow").closest(".follow-row");
      if (row) row.classList.toggle("is-on", $("armFollow").checked);
      if (!$("armFollow").checked) send({ t: "park" });
      else jointDirty = false;
    });
  }
  $("btnGripOpen").onclick = () => {
    fireHotCmd("grip-open");
  };
  $("btnGripClose").onclick = () => {
    fireHotCmd("grip-close");
  };
  if ($("btnArmEnable")) {
    $("btnArmEnable").onclick = () => firePower(true);
  }
  if ($("btnArmDisable")) {
    $("btnArmDisable").onclick = () => firePower(false);
  }
  if ($("btnArmConnect")) {
    $("btnArmConnect").onclick = () => fireLink("connect");
  }
  if ($("btnArmDisconnect")) {
    $("btnArmDisconnect").onclick = () => fireLink("disconnect");
  }
  function sendGripDeg() {
    const deg = Number($("gripDeg").value || 0);
    const mid = 0.5 * (gripLimits[0] + gripLimits[1]);
    const open = deg >= mid;
    gripOpenCmd = open;
    gripToggleAt = performance.now();
    paintGrip(open, deg);
    send({ t: "grip", deg: deg, effort: currentGripEffort() });
  }
  if ($("gripRange") && $("gripDeg")) {
    $("gripRange").addEventListener("input", () => {
      $("gripDeg").value = $("gripRange").value;
      const deg = Number($("gripRange").value);
      const mid = 0.5 * (gripLimits[0] + gripLimits[1]);
      paintGrip(deg >= mid, deg);
    });
    $("gripDeg").addEventListener("input", () => {
      $("gripRange").value = $("gripDeg").value;
      const deg = Number($("gripDeg").value);
      const mid = 0.5 * (gripLimits[0] + gripLimits[1]);
      paintGrip(deg >= mid, deg);
    });
    $("gripRange").addEventListener("change", sendGripDeg);
    $("gripDeg").addEventListener("change", sendGripDeg);
  }
  if ($("gripEffortRange") && $("gripEffort")) {
    $("gripEffortRange").addEventListener("input", () => {
      $("gripEffort").value = $("gripEffortRange").value;
      paintGripEffortNm();
    });
    $("gripEffort").addEventListener("input", () => {
      $("gripEffortRange").value = $("gripEffort").value;
      paintGripEffortNm();
    });
    $("gripEffortRange").addEventListener("change", sendGripEffort);
    $("gripEffort").addEventListener("change", sendGripEffort);
    paintGripEffortNm();
  }

  $("hostToggles").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-panel]");
    if (!btn) return;
    pulseBtn(btn);
    btn.classList.toggle("on");
    const el = $("panel-" + btn.dataset.panel);
    if (el) el.classList.toggle("off", !btn.classList.contains("on"));
    syncStage();
  });
  if ($("spaceBtns")) {
    $("spaceBtns").addEventListener("click", (ev) => {
      const btn = ev.target.closest("button[data-space]");
      if (!btn) return;
      setSpace(btn.dataset.space, true);
    });
  }
  if ($("armSrcBtns")) {
    $("armSrcBtns").addEventListener("click", (ev) => {
      const btn = ev.target.closest("button[data-arm-src]");
      if (!btn) return;
      setArmSrc(btn.dataset.armSrc);
    });
  }
  $("modeBtns").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-mode]");
    if (!btn || btn.disabled) return;
    const next = btn.dataset.mode;
    const arm = (last && last.arm) || {};
    if (floatModeNeedsDynamics(next) && !floatModesOk(arm)) {
      setAck(next + "：" + (arm.float_reason || "需 pinocchio") + "（真机力矩环未加载）", false);
      return;
    }
    ctrlMode = next;
    paintCtrlMode(arm);
    pulseBtn(btn);
    if (ctrlMode !== "Position" && $("armFollow")) $("armFollow").checked = false;
    if (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") setSpace("joint");
    send({ t: "mode", mode: ctrlMode });
    setAck("已切到 " + ctrlMode, true);
  });
  $("btnWpAdd").onclick = () => {
    pulseBtn($("btnWpAdd"));
    waypoints.push({ q: readJoints(), t: 2 });
    paintWaypoints();
    setAck("已记录路点 #" + waypoints.length, true);
  };
  $("btnWpClear").onclick = () => {
    pulseBtn($("btnWpClear"));
    waypoints = [];
    paintWaypoints();
    setAck("已清空路点", true);
  };
  $("btnWpRun").onclick = async () => {
    pulseBtn($("btnWpRun"));
    if (ctrlMode !== "Position") { setAck("请先切回 Position", false); return; }
    if (!waypoints.length) { setAck("还没有路点", false); return; }
    if (!beginCmd("path", {
      id: "btnWpRun",
      busy: "运行中…",
      ack: "轨迹已交给后端…",
    })) return;
    send({
      t: "path",
      q: waypoints.map((w) => w.q),
      dt: waypoints.map((w) => Number(w.t || 2)),
      speed: Number($("armSpeed").value || 40),
    });
  };
  async function loadScripts() {
    const sel = $("scriptSel");
    if (!sel) return;
    try {
      const r = await fetch("/api/scripts");
      if (!r.ok) return;
      const body = await r.json();
      const items = body.scripts || [];
      sel.innerHTML = items.map((it) => {
        const name = it.name || it;
        const mapped = it.mapped ? " · 可映射" : " · 仅列表";
        return "<option value='" + name + "'>" + name + mapped + "</option>";
      }).join("");
    } catch (e) {}
  }
  $("btnScriptRun").onclick = () => {
    const name = $("scriptSel").value;
    if (!beginCmd("script", {
      id: "btnScriptRun",
      busy: "请求中…",
      ack: "已请求 " + name,
    })) return;
    send({ t: "script", name: name });
    if ($("scriptAck")) {
      $("scriptAck").textContent = "已请求 " + name;
      $("scriptAck").className = "hint tone-ok";
    }
  };
  if ($("btnScriptRefresh")) $("btnScriptRefresh").onclick = () => {
    pulseBtn($("btnScriptRefresh"));
    loadScripts();
  };

  function cartPulse(dtSec) {
    if (!cartHeld.size) return;
    if (last && last.safety === "ESTOP_LATCHED") return;
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") return;
    const dt = dtSec || 1 / controlHz;
    const scale = 10 * dt;
    let dxyz = [0, 0, 0];
    let drpy = [0, 0, 0];
    cartHeld.forEach((code) => {
      const step = CART_KEYS[code];
      if (!step) return;
      dxyz[0] += step[0] * scale; dxyz[1] += step[1] * scale; dxyz[2] += step[2] * scale;
      drpy[0] += step[3] * scale; drpy[1] += step[4] * scale; drpy[2] += step[5] * scale;
    });
    send({ t: "cart", dxyz: dxyz, drpy: drpy, speed: Number($("armSpeed").value || 40) });
    lastCtrl = performance.now();
    noteHz();
  }
  function syncSlidersAfterCart() {
    if (cartHeld.size || !last || !last.arm) return;
    writeJointInputs(last.arm.q_deg);
    writeCartInputs(last.arm.ee_m, last.arm.ee_rpy_deg);
  }

  window.addEventListener("hashchange", showPage);
  window.addEventListener("keydown", (ev) => {
    if (!live) return;
    if (ev.code === "Escape") {
      ev.preventDefault();
      fireEstop();
      return;
    }
    if (typing()) return;
    const cmd = CMD_KEYS[ev.code];
    if (cmd) {
      if (ev.repeat) return;
      ev.preventDefault();
      fireHotCmd(cmd);
      return;
    }
    if (!CART_KEYS[ev.code]) return;
    ev.preventDefault();
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") {
      setAck("请先切回 Position 再用键盘", false);
      return;
    }
    cartHeld.add(ev.code);
    if (!ev.repeat) {
      flashHot("cart-move", "笛卡尔 " + ev.key.toUpperCase());
      cartPulse();
    }
  });
  window.addEventListener("keyup", (ev) => {
    if (!CART_KEYS[ev.code]) return;
    cartHeld.delete(ev.code);
    syncSlidersAfterCart();
    if (!cartHeld.size) send({ t: "park" });
  });
  window.addEventListener("blur", () => {
    const had = cartHeld.size > 0;
    cartHeld.clear();
    if (had) send({ t: "park" });
  });
  if ($("btnHotkeys")) {
    $("btnHotkeys").onclick = (ev) => {
      ev.stopPropagation();
      const pop = $("hotkeyPop");
      setHotkeyPop(!!(pop && pop.hidden));
    };
  }
  document.addEventListener("click", (ev) => {
    const bar = $("hotkeyBar");
    if (!bar || bar.contains(ev.target)) return;
    setHotkeyPop(false);
  });

  setInterval(() => { if (live) sendTouch(); flushHz(); }, 100);
  setInterval(() => { if (live && !rttWorker) sendPing(); }, 250);

  buildJoints();
  bindCartSliders();
  paintWaypoints();
  showPage();
  boot();
})();
