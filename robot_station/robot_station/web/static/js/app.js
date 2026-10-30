/* FAFU 机械臂站控 · 浏览器端：控制 JSON WebSocket + 实时关节状态
 * 客户 PC 本机直连机械臂。
 */
(() => {
  const PAGES = ["home", "arm/control", "work/waypoints", "work/trajectories", "work/program"];
  const PAGE_ID = {"home":"home", "arm/control":"arm", "work/waypoints":"waypoints", "work/trajectories":"trajectories", "work/program":"program"};
  let shownPage = "";
  let lastWorkPage = "work/program";
  const DEFAULT_PAGE = "arm/control";
  const PAGE_TITLE = {"home":"总览与连接", "arm/control":"手动控制", "work/waypoints":"路点", "work/trajectories":"示教轨迹", "work/program":"程序"};
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
  let ws = null;
  let last = null;
  let lastPowerSeq = null;
  let lastPowerFeedbackAt = 0;
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
  let teachContext = "";
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
    { label: "R", unit: "°", min: -180, max: 180, step: 0.1, digits: 1, def: 0 },
    { label: "P", unit: "°", min: -180, max: 180, step: 0.1, digits: 1, def: 0 },
    { label: "Y", unit: "°", min: -180, max: 180, step: 0.1, digits: 1, def: 0 },
  ];
  let cartDirty = false;
  let cartPrimed = false;
  const cartFollow = {timer:0, pending:null, dirty:false, lastSent:0, serial:0};
  const CART_HINT = "目标值独立保留，实时位姿显示在 3D 下方。键盘仍用于增量控制。";

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
    const factor = last && last.arm && last.arm.gripper_nm_per_raw;
    if ($("gripEffortUnit")) $("gripEffortUnit").textContent = factor ? "raw ≈ " + (raw * factor).toFixed(2) + " Nm" : "raw";
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
    if ($(open ? "btnGripOpen" : "btnGripClose")?.disabled) return;
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

  function setWorkMenu(open, restoreFocus = false) {
    $("workNavMenu").hidden = !open;
    $("workNavSelect").setAttribute("aria-expanded", String(open));
    if (restoreFocus) $("workNavSelect").focus();
  }
  function currentPage() {
    const raw = (location.hash || "#/" + DEFAULT_PAGE).replace(/^#\/?/, "").split("?")[0];
    if (raw === "arm/connection") return "home";
    return raw === "arm" ? "arm/control" : (PAGES.indexOf(raw) >= 0 ? raw : DEFAULT_PAGE);
  }
  function ensureDefaultHash() {
    const raw = (location.hash || "").replace(/^#\/?/, "").split("?")[0];
    if (raw === "arm/connection") { location.hash = "#/home"; return true; }
    if (!raw || raw === "arm" || PAGES.indexOf(raw) < 0) {
      location.hash = "#/" + DEFAULT_PAGE;
      return true;
    }
    return false;
  }
  function releaseManual(park = true) {
    const had = cartHeld.size || followDirty || ($("armFollow") && $("armFollow").checked);
    cancelCartFollow();
    cartHeld.clear();
    followDirty = false;
    if ($("armFollow")) $("armFollow").checked = false;
    $("armFollow")?.closest(".follow-row")?.classList.remove("is-on");
    if (park && had && ws && ws.readyState === 1) send({t:"park"});
  }
  function showPage() {
    if (ensureDefaultHash()) return;
    const page = currentPage();
    if (page !== shownPage && $("simTeachEnabled")) $("simTeachEnabled").checked = false;
    if (shownPage === "arm/control" && page !== shownPage) releaseManual();
    shownPage = page;
    setWorkMenu(false);
    const group = page.split("/")[0];
    document.querySelectorAll(".page").forEach(el => el.classList.toggle("on", el.id === "page-" + PAGE_ID[page]));
    document.querySelectorAll("#navTabs > a").forEach(a => a.classList.toggle("on", a.dataset.group === group));
    $("workNav").classList.toggle("on", group === "work");
    if (group === "work") lastWorkPage = page;
    $("workNavSelect").value = lastWorkPage;
    $("workNavLabel").textContent = PAGE_TITLE[lastWorkPage];
    $("workNavEntry").href = "#/" + lastWorkPage;
    $("workNavEntry").title = "打开" + PAGE_TITLE[lastWorkPage];
    document.querySelectorAll("#workNavMenu a").forEach(a => {
      if (a.dataset.workPage === page) a.setAttribute("aria-current", "page");
      else a.removeAttribute("aria-current");
    });
    document.title = "FAFU 机械臂站控 · " + PAGE_TITLE[page];
    syncStage();
    window.dispatchEvent(new CustomEvent("station:page", {detail:page}));
  }

  function syncStage() {
    const twin = $("panel-twin");
    const stage = $("panel-stage");
    const twinOn = !!(twin && !twin.classList.contains("off"));
    if (stage) stage.classList.toggle("off", !twinOn);
    if (window.Arm3D) {
      Arm3D.setActive(twinOn && currentPage() === "arm/control");
    }
  }

  function typing() {
    if (!$("workNavMenu").hidden) return true;
    const el = document.activeElement;
    if (!el) return false;
    if (el.closest("#workNav, #commandNotice, #hotkeyBar, #btnTheme")) return true;
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
    return spaceMode === "joint" && !!$("armFollow")?.checked;
  }
  const CMD_QUEUE_LIB = window.CmdQueue;
  if (!CMD_QUEUE_LIB) console.error("cmdqueue.js 未加载：断线指令将不排队");
  const cmdQueue = CMD_QUEUE_LIB
    ? CMD_QUEUE_LIB.create({ ttlMs: 3000, maxItems: 16 })
    : { enqueue: () => false, flush: () => true, sweep: () => {}, size: () => 0, hasEstop: () => false };
  const QUEUED_UI_OPS = new Set([
    "arm", "cart_go", "home", "power", "link", "arm_src", "clear", "path",
    "replay", "rec_start", "rec_stop", "rec_delete", "rec_rename", "script",
    "grip", "mode", "grav_urdf",
  ]);

  function dropQueued(obj, reason) {
    const op = obj && obj.t;
    if (!op) return;
    // A queued target that is cancelled or preempted by estop must not keep
    // the sliders marked dirty forever.
    if (op === "arm") jointDirty = false;
    if (op === "cart_go") cartDirty = false;
    if (pendingCmd[op]) {
      endCmd(op, false, reason);
      return;
    }
    if (QUEUED_UI_OPS.has(op)) setAck(reason, false);
  }

  function withSession(obj) {
    return {...obj, session:last?.connection?.session};
  }
  function send(obj) {
    obj = withSession(obj);
    if (obj && (obj.t === "arm" || obj.t === "cart" || obj.t === "teach" || obj.t === "cart_go")) {
      obj.t0 = performance.now();
    }
    if (ws && ws.readyState === 1) {
      try {
        ws.send(JSON.stringify(obj));
        return true;
      } catch (e) {}
    }
    if (obj.t !== "e") {
      dropQueued(obj, "控制连接已断开，旧指令不会补发，请重新操作");
      return false;
    }
    if (!CMD_QUEUE_LIB || CMD_QUEUE_LIB.shouldDrop(obj)) return false;
    cmdQueue.enqueue(obj, dropQueued);
    return false;
  }

  let flushTimer = 0;
  function flushCmdQueue() {
    if (!ws || ws.readyState !== 1) return;
    const ok = cmdQueue.flush((obj) => {
      try {
        ws.send(JSON.stringify(obj));
        return true;
      } catch (e) {
        return false;
      }
    }, dropQueued);
    if (!ok && cmdQueue.size() && ws && ws.readyState === 1 && !flushTimer) {
      flushTimer = setTimeout(() => {
        flushTimer = 0;
        flushCmdQueue();
      }, 250);
    }
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
  function deviceState() {
    const a=last?.arm || {},c=last?.connection || {};
    const connected=!!(ws && ws.readyState===1);
    const fresh=!!(connected && telemWs?.readyState===1 && lastPowerFeedbackAt && performance.now()-lastPowerFeedbackAt<2000);
    const phase=c.status || (a.online?"connected":"disconnected");
    return {connected,fresh,phase,online:!!(fresh && phase==="connected" && a.online),
      kind:c.kind || (a.backend==="sim" || a.backend==="mock"?"simulation":"serial"),
      port:c.port || a.serial_port || ""};
  }
  function setLink(ok, extra) {
    const cls="dot "+(ok?"on":"off"),text=(ok?"已连接":"未连接")+(extra?" · "+extra:"");
    if ($("dotLink").className!==cls) $("dotLink").className=cls;
    if ($("linkTxt").textContent!==text) $("linkTxt").textContent=text;
  }
  function paintGlobalStatus() {
    const s=deviceState(),a=last?.arm || {};
    // These indicators must update even when teleoperation skips the heavy DOM render.
    if (!s.online || !a.enabled || last?.connection?.enable_required) releaseManual(false);
    setLink(s.online);
    const switching=["connecting","disconnecting"].includes(s.phase);
    const label=!s.fresh?"设备状态未知":switching?(s.phase==="connecting"?"正在连接设备":"正在断开设备"):
      !s.online?"无活动设备":s.kind==="simulation"?"本机仿真":s.port?"串口 · "+s.port:"串口机械臂";
    if ($("mockTxt").textContent!==label) $("mockTxt").textContent=label;
    $("mockDot").className="dot "+(!s.fresh || switching?"warn":!s.online?"":s.kind==="simulation"?"warn":"on");
    const readonly=s.online && s.kind==="serial" && info && !info.arm_allow_motion;
    const banner=!s.fresh?(s.connected?"设备状态未更新 · 暂停控制，等待最新反馈。":"站控通信已断开 · 正在重新连接。"):
      switching?"设备正在切换连接，请等待完成。":!s.online?"设备未连接 · 在「总览与连接」选择串口机械臂或本机仿真。":
      s.kind==="simulation"?"本机仿真 · 当前操作仅作用于模拟机械臂。":readonly?"串口已连接 · 当前为只读连接，不接受使能与运动。":"";
    $("collectBanner").classList.toggle("on",!!banner);
    if ($("collectBanner").textContent!==banner) $("collectBanner").textContent=banner;
    paintSafety(last);
  }
  function setAck(text, ok, options = {}) {
    if (text) window.StationToast?.show(text, {tone:ok ? "success" : "error", ...options});
    const cls = "hint " + (ok ? "tone-ok" : "tone-bad");
    ["armAck"].forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.textContent = text || "";
      el.className = cls;
    });
  }
  function vendorEnableNoise(text, arm) {
    const t = String(text || "");
    if (!/motor_reset|enable failed|使能失败/.test(t)) return false;
    return !!arm?.enabled;
  }
  function paintCmdErr(d, a) {
    const err = (d && d.cmd_err) || "";
    if (pendingCmd.power && String(err).startsWith("电机去使能未完成")) {
      endCmd("power", false, err);
      paintPowerButton();
    }
    const cur = ($("armAck") && $("armAck").textContent) || "";
    if (vendorEnableNoise(err, a) || (a && a.enabled && vendorEnableNoise(cur, a))) {
      if (vendorEnableNoise(cur, a)) setAck("", true);
      return;
    }
    if (err) setAck(err, false);
  }

  const pendingCmd = Object.create(null);
  const pulseTimers = new WeakMap();

  function pendingMs(op) {
    if (op === "power" || op === "link" || op === "arm_src" || op === "clear") return 25000;
    if (op === "home" || op === "path" || op === "replay") return 20000;
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
    pendingCmd[op] = { id: id, pair: pair, want: spec.want, pick: spec.pick, t0: performance.now() };
    if (spec.hot) flashHot(spec.hot, spec.ack);
    if (spec.ack) setAck(spec.ack, spec.okTone !== false, {tone:"info", pending:true, force:true});
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
    if (op === "power") return want ? "机械臂已使能" : "机械臂已去使能，电机不再提供保持力矩";
    if (op === "home") return "复位已下发";
    if (op === "arm" || op === "cart_go") return "位置已下发";
    if (op === "path") return "轨迹已下发";
    if (op === "rec_start") return "已开始录制";
    if (op === "rec_stop") return "已停止录制";
    if (op === "rec_delete") return "已删除轨迹";
    if (op === "rec_rename") return "已重命名轨迹";
    if (op === "replay") return "回放已下发";
    if (op === "grip") return "夹爪指令已下发";
    if (op === "arm_src") return "臂源已切换";
    if (op === "clear") return "已解除急停";
    if (op === "mode") return "已切到 " + (ctrlMode || "");
    if (op === "grav_urdf") return "已切换重力 URDF";
    if (op === "script") return "已执行映射";
    return "已下发 " + (op || "");
  }
  function paintPowerButton() {
    const button = $("btnPowerToggle"), a = last && last.arm;
    const motorEnabled = !!a?.enabled, needsConfirmation = !!last?.connection?.enable_required;
    const {connected,fresh,online}=deviceState();
    const enabled = online && motorEnabled && !needsConfirmation, pending = pendingCmd.power;
    const estop = last && last.safety === "ESTOP_LATCHED";
    const programBusy = ["running", "paused"].includes(last && last.program && last.program.status);
    const oldHost = enabled && a.backend === "fafu" && !(last.capabilities && last.capabilities.motor_disable);
    const known = online;
    const powerStatus = $("powerStatus"), powerStatusText = $("powerStatusText");
    const statusState = !known ? "unknown" : motorEnabled ? "on" : "off";
    const statusText = !known ? "使能未知" : motorEnabled ? "已使能" : "未使能";
    if (powerStatus && powerStatusText) {
      powerStatus.dataset.state = statusState;
      if (powerStatusText.textContent !== statusText) powerStatusText.textContent = statusText;
      powerStatus.title = known ? "机械臂" + statusText : "等待机械臂最新使能反馈";
    }
    button.dataset.powerState = pending ? "pending" : !known ? "unknown" : enabled ? "on" : "off";
    button.dataset.idle = known && enabled ? "去使能" : known && motorEnabled && needsConfirmation ? "确认使能" : "使能";
    button.disabled = !!(pending || !known || estop || programBusy || oldHost);
    button.textContent = pending ? (pending.want ? "正在使能…" : "正在去使能…") : button.dataset.idle;
    button.setAttribute("data-hot", enabled ? "disable" : "enable");
    setBtnOn("btnPowerToggle", enabled);
    button.title = pending ? "等待设备状态确认，请勿重复点击" : !connected || !fresh ? "等待最新设备状态" :
      !online ? "请先连接机械臂" : estop ? "请先解除急停" : programBusy ? "请先停止程序" :
      oldHost ? "请重启站控以加载真正的电机去使能功能" :
      enabled ? "点击去使能，电机将不再提供保持力矩" : motorEnabled && needsConfirmation ? "电机已处于使能状态，确认后以当前实测姿态接管控制" : "当前未使能，点击使能";
  }
  function paintCmdButtons(a, estop) {
    sweepPending();
    const online = deviceState().online;
    const open = online && !!a?.gripper_open;
    if (pendingCmd.power) {
      const p = pendingCmd.power;
      const confirmed = p.want ? a?.enabled && !last?.connection?.enable_required : !a?.enabled;
      if (p && online && confirmed) {
        endCmd("power", true, ackText("power", true, "", p.want));
      }
    }
    paintPowerButton();
    setBtnOn("btnGripOpen", open);
    setBtnOn("btnGripClose", online && !open);
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
      if (flash) { flash.classList.remove("on"); flash.hidden = true; flash.textContent = ""; }
    }, 1400);
  }

  function layoutHotkeyPop() {
    const pop = $("hotkeyPop"), bar = $("hotkeyBar");
    if (!pop || pop.hidden || !bar) return;
    const anchor = bar.getBoundingClientRect();
    const width = pop.getBoundingClientRect().width;
    const left = Math.max(12, Math.min(anchor.right - width, document.documentElement.clientWidth - width - 12));
    const top = document.querySelector(".top").getBoundingClientRect().bottom + 8;
    pop.style.left = (left - anchor.left) + "px";
    pop.style.top = (top - anchor.top) + "px";
    pop.style.maxHeight = Math.max(0, innerHeight - top - 12) + "px";
  }

  function setHotkeyPop(open) {
    const pop = $("hotkeyPop");
    const btn = $("btnHotkeys");
    if (!pop || !btn) return;
    pop.hidden = !open;
    btn.classList.toggle("open", open);
    btn.setAttribute("aria-expanded", open ? "true" : "false");
    if (open) layoutHotkeyPop();
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
    try {
      return limits.map(([min,max],i) => window.StationNumbers.read($("jointList").querySelector("[data-jn='"+i+"']"),{label:"J"+(i+1)+" 目标角度",min,max}));
    } catch (_) {return null;}
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
      if (!q) return;
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
    if (currentPage() !== "arm/control") return false;
    if (spaceMode !== "joint") return false;
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") return false;
    if (!jointsPrimed()) return false;
    return !!( $("armFollow") && $("armFollow").checked );
  }
  function stopFollowStream(park = true) {
    cancelCartFollow();
    if ($("armFollow")) $("armFollow").checked = false;
    $("armFollow")?.closest(".follow-row")?.classList.remove("is-on");
    followDirty = false;
    const box = $("jointList");
    const el = document.activeElement;
    if (box && el && box.contains(el) && typeof el.blur === "function") el.blur();
    if (park) send({ t: "park" });
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
    if ((ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") && !simulatedTeachEnabled()) return;
    followDirty = true;
    if (!($("armFollow") && $("armFollow").checked)) jointDirty = true;
    sendFollowNow();
  }
  function sendFollowNow() {
    if (cartHeld.size) return;
    if (last && last.safety === "ESTOP_LATCHED") return;
    const q = readJoints();
    if (!q) {
      if ($("armFollow")?.checked) stopFollowStream();
      return;
    }
    if (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") {
      if (!simulatedTeachEnabled()) return;
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
    if (currentPage() !== "arm/control") return false;
    if (spaceMode === "cart") return false;
    if (cartHeld.size) return false;
    if (last && last.safety === "ESTOP_LATCHED") return false;
    const box = $("jointList");
    const el = document.activeElement;
    const sliding = !!(box && el && box.contains(el));
    if (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") return simulatedTeachEnabled() && (sliding || followDirty);
    if (!wantJointStream()) return false;
    if (ctrlMode !== "Position" && ctrlMode !== "Impedance") return false;
    return sliding || followDirty || ($("armFollow") && $("armFollow").checked);
  }
  function controlTick(dtSec) {
    if (currentPage() !== "arm/control" || (last && last.program && ["running","paused"].includes(last.program.status))) return;
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
    if (v == null || v === "" || !Number.isFinite(Number(v))) return "—";
    return Number(v).toFixed(digits) + unit;
  }
  function paintCartPose(a) {
    const xyz = (a && a.ee_m) || [];
    const rpy = (a && a.ee_rpy_deg) || [];
    const online=deviceState().online;
    const values=[xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]];
    ["tcpX","tcpY","tcpZ","tcpR","tcpP","tcpYaw"].forEach((id,i)=>{
      const element=$(id);if(!element)return;
      const text=online?fmtCart(values[i],i<3?3:1,""):"—";
      if(element.textContent!==text)element.textContent=text;
    });
    if($("tcpReadoutState"))$("tcpReadoutState").textContent=online?"实时位姿":"等待反馈";
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
      window.StationNumbers.clear(n);
    });
  }
  function syncCartFromSnap(arm) {
    if (cartPrimed || cartDirty || !arm?.online || arm.ee_m?.length!==3 || arm.ee_rpy_deg?.length!==3) return;
    writeCartInputs(arm.ee_m, arm.ee_rpy_deg);
    cartPrimed=true;
  }
  function readCart() {
    const box = $("cartList");
    const xyz = [];
    const rpy = [];
    try {for (let i = 0; i < 6; i++) {
      const n = box && box.querySelector("[data-cn='" + i + "']");
      const ax=CART_AXES[i];
      const v = window.StationNumbers.read(n,{label:ax.label+" 目标"+(i<3?"位置":"姿态"),min:ax.min,max:ax.max});
      if (i < 3) xyz.push(v);
      else rpy.push(v);
    }}catch(_){return null;}
    return { xyz: xyz, rpy: rpy };
  }
  function cartNearMeasured(arm, xyz, rpy) {
    const m = (arm && arm.ee_m) || [];
    const r = (arm && arm.ee_rpy_deg) || [];
    if(m.length!==3 || r.length!==3)return false;
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
    if(!pose){setAck("请填写完整且在范围内的 XYZ、RPY 目标",false);return;}
    const arm = last && last.arm;
    if (live && arm && cartNearMeasured(arm, pose.xyz, pose.rpy) && !arm.moving) {
      setAck("已在目标位姿，无需移动。", true);
      return;
    }
    stopFollowStream(false);
    const sent = send({ t: "cart_go", xyz: pose.xyz, rpy: pose.rpy, speed: Number($("armSpeed").value || 40) });
    cartDirty = sent || !!CMD_QUEUE_LIB;
    flashHot("send", "发送位置");
    setAck(sent ? (arm?.moving ? "已更新目标，从当前位姿按设定速度移动" : "笛卡尔目标已按设定速度下发") : "未连接：仅保留最新目标，连接恢复后下发", sent);
  }
  function bindCartSliders() {
    const box = $("cartList");
    if (!box) return;
    box.innerHTML = "";
    CART_AXES.forEach((ax, i) => {
      const row = document.createElement("div");
      row.className = "joint";
      row.dataset.axis=["x","y","z","r","p","yaw"][i];
      const description=ax.label+" 目标"+(i<3?"位置":"姿态");
      row.innerHTML =
        "<label for='cartTarget"+i+"'>" + ax.label + "</label>" +
        "<input type='range' min='" + ax.min + "' max='" + ax.max + "' step='" + ax.step +
        "' value='" + ax.def + "' data-c='" + i + "' aria-label='"+description+"滑条'/>" +
        "<input type='number' min='" + ax.min + "' max='" + ax.max + "' step='" + ax.step +
        "' value='" + Number(ax.def).toFixed(ax.digits) + "' data-cn='" + i + "' id='cartTarget"+i+"' aria-label='"+description+"（"+ax.unit+"）'/>" +
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
      if (el.dataset.c == null && el.dataset.cn == null) return;
      cartDirty=true;
      if (el.dataset.c != null) syncPair(Number(el.dataset.c), el.value);
      if (el.dataset.cn != null && el.value!=="" && Number.isFinite(Number(el.value))) box.querySelector("[data-c='"+el.dataset.cn+"']").value=el.value;
      queueCartFollow();
    });
    box.addEventListener("change", (ev) => {
      const el = ev.target;
      if (el.dataset.c == null && el.dataset.cn == null) return;
      cartDirty=true;
      const i=Number(el.dataset.c ?? el.dataset.cn), ax=CART_AXES[i];
      try{syncPair(i,window.StationNumbers.read(el,{label:ax.label+" 目标",min:ax.min,max:ax.max}));}
      catch(_){return;}
      queueCartFollow();
    });
  }
  function wantCartFollow(){
    return spaceMode==="cart" && currentPage()==="arm/control" && !!$("armFollow")?.checked && live &&
      !!last?.capabilities?.cartesian_follow && !cartHeld.size &&
      ["Position","Impedance"].includes(ctrlMode) && last?.arm?.online && last.arm.enabled &&
      last.safety!=="ESTOP_LATCHED" && !["running","paused"].includes(last.program?.status);
  }
  function cancelCartFollow(){
    clearTimeout(cartFollow.timer);cartFollow.timer=0;cartFollow.dirty=false;
    if(cartFollow.pending)clearTimeout(cartFollow.pending.timeout);
    cartFollow.pending=null;
  }
  function queueCartFollow(){
    if(!wantCartFollow())return;
    cartFollow.dirty=true;
    if(cartFollow.timer || cartFollow.pending)return;
    cartFollow.timer=setTimeout(flushCartFollow,Math.max(0,100-(performance.now()-cartFollow.lastSent)));
  }
  function flushCartFollow(){
    cartFollow.timer=0;
    if(!wantCartFollow() || !cartFollow.dirty || cartFollow.pending)return;
    const pose=readCart();if(!pose)return;
    cartFollow.dirty=false;
    if(!last.arm.moving && cartNearMeasured(last.arm,pose.xyz,pose.rpy))return;
    const request_id="cart-follow-"+(++cartFollow.serial);
    const timeout=setTimeout(()=>{if(cartFollow.pending?.id===request_id){stopFollowStream();setAck("笛卡尔跟随应答超时，已关闭实时跟随",false);}},3000);
    cartFollow.pending={id:request_id,timeout};cartFollow.lastSent=performance.now();
    try{ws.send(JSON.stringify(withSession({t:"cart_go",...pose,speed:Number($("armSpeed").value||40),stream:true,request_id,t0:performance.now()})));}
    catch(_){stopFollowStream();setAck("连接已断开，已关闭实时跟随",false);}
  }
  function handleCartFollowAck(msg){
    if(!cartFollow.pending || msg.request_id!==cartFollow.pending.id)return false;
    clearTimeout(cartFollow.pending.timeout);cartFollow.pending=null;
    if(!msg.ok){stopFollowStream();setAck("笛卡尔跟随失败："+(msg.error||"目标无法执行"),false);}
    else if(cartFollow.dirty)queueCartFollow();
    return true;
  }
  function useCurrentCartPose(){
    const arm=last?.arm;if(!arm?.online || arm.ee_m?.length!==3 || arm.ee_rpy_deg?.length!==3)return;
    if($("armFollow")?.checked)stopFollowStream();
    writeCartInputs(arm.ee_m,arm.ee_rpy_deg);cartDirty=false;cartPrimed=true;
  }
  function setSpace(name, fromUi) {
    if (name === "cart" && (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri")) {
      setAck("Gravity 示教请用关节空间。笛卡尔请先切回 Position。", false);
      name = "joint";
    }
    const nextSpace=name === "cart" ? "cart" : "joint";
    if(spaceMode!==nextSpace)releaseManual();
    spaceMode=nextSpace;
    $("manualControlTitle").textContent=spaceMode==="cart"?"笛卡尔控制":"关节控制";
    if($("armFollow")){
      const unsupported=spaceMode==="cart" && !last?.capabilities?.cartesian_follow;
      $("armFollow").disabled=unsupported || ["Gravity","Gra+Fri"].includes(ctrlMode) || ["running","paused"].includes(last?.program?.status);
      $("armFollow").title=unsupported?"重启更新后的站控服务以启用笛卡尔实时跟随":"";
    }
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
        ? "拖动只编辑 XYZ、RPY 目标，点击「发送位置」后运动。开启实时跟随时，拖动即运动。"
        : "拖动滑条设置目标，点击「发送位置」后运动。开启实时跟随时，拖动即运动。";
    }
    if (spaceMode === "cart" && !cartDirty && last?.arm?.online) {
      cartDirty = false;
      writeCartInputs(last.arm.ee_m, last.arm.ee_rpy_deg);
      cartPrimed=true;
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
    const q = readJoints();
    if (!q) {setAck("请填写完整且在关节限位内的六轴目标角度",false);return;}
    const arm = last && last.arm;
    const cur = (arm && (arm.feedback_deg?.length >= 6 ? arm.feedback_deg : arm.q_deg)) || [];
    const same = q.length === 6 && cur.length >= 6 && q.every((v, i) => Math.abs(Number(v) - Number(cur[i])) < 0.2);
    const sameLive = arm?.q_deg?.length >= 6 && q.every((v,i)=>Math.abs(Number(v)-Number(arm.q_deg[i]))<0.2);
    // A validated target replaces the old motion directly. A separate park
    // could restore an obsolete servo target before this command is accepted.
    stopFollowStream(false);
    if (live && same && sameLive && !arm.moving) {
      jointDirty = false;
      setAck("已在目标位置，无需移动。", true);
      return;
    }
    const sent = send({ t: "arm", q: q, speed: Number($("armSpeed").value || 40) });
    jointDirty = sent || !!CMD_QUEUE_LIB;
    flashHot("send", "发送位置");
    setAck(sent ? (arm?.moving ? "已更新目标，从当前位置按设定速度移动" : "关节目标已下发，按设定速度移动") : "未连接：仅保留最新目标，连接恢复后下发", sent);
  }

  function kv(title, rows) {
    const lines = rows.map(([k, v, cls]) => {
      const c = cls ? " class='" + cls + "'" : "";
      return "<tr><td>" + k + "</td><td" + c + ">" + v + "</td></tr>";
    }).join("");
    return "<div class='kv'><h3>" + title + "</h3><table>" + lines + "</table></div>";
  }
  function acceptSnapshot(d) {
    if (last?.connection?.session && d.connection?.session!==last.connection.session) {
      releaseManual(false);
      jointDirty=false;
      pendingTelem=null;
    }
    if (d.seq !== lastPowerSeq) {
      lastPowerSeq = d.seq;
      lastPowerFeedbackAt = performance.now();
    }
    last = d;
    paintGlobalStatus();
    paintMotorStatus(d.arm || {});
    paintCmdButtons(d.arm || {},d.safety==="ESTOP_LATCHED");
    window.dispatchEvent(new CustomEvent("station:snapshot", {detail:d}));
  }
  function render(d) {
    acceptSnapshot(d);
    const a = d.arm || {};
    applyLimits(a);
    if (a.ctrl_mode) ctrlMode = a.ctrl_mode;
    paintCtrlMode(a);
    if (currentPage() === "arm/control") {
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
    paintSafety(d);
    paintCmdErr(d, a);
    if ($("kbAck")) {
      if (a.ik_err) {
        $("kbAck").textContent = a.ik_err;
        $("kbAck").className = "hint tone-bad";
        if ($("kbPadHint")) {
          $("kbPadHint").textContent = a.ik_err;
          $("kbPadHint").className = "hint tone-bad";
        }
      } else if ($("kbAck").classList.contains("tone-bad")) {
        $("kbAck").textContent = "松开按键停止，快捷键仅在手动控制页生效。";
        $("kbAck").className = "hint";
        if ($("kbPadHint")) {
          $("kbPadHint").textContent = CART_HINT;
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
    const reason = a.float_reason || "真机力矩环未加载";
    const box = $("modeBtns");
    if (box) {
      box.querySelectorAll("[data-mode]").forEach((btn) => {
        const mode = btn.dataset.mode;
        const blocked = (floatModeNeedsDynamics(mode) && !ok) || !!a.recording;
        btn.disabled = blocked;
        btn.setAttribute("aria-disabled", blocked ? "true" : "false");
        btn.title = a.recording ? "录制期间控制模式已锁定，请先停止录制" : blocked ? (mode + "：" + reason + "（真机力矩环未加载）") : "";
        btn.classList.toggle("active", mode === ctrlMode);
      });
    }
    const hints = {
      Position: "Position：关节/笛卡尔滑条 + 发送位置。实时跟随与键盘走 servo。",
      Gravity: "Gravity：真机以重力补偿接受手动引导，关节控件显示实测位置。仿真需明确开启「仿真拖动」后才能用滑条模拟手动拖臂。",
      "Gra+Fri": "Gra+Fri：与 Gravity 相同，另加示教摩擦补偿。切入同样有短时软件阻尼，稳定后保持 G(q)+摩擦。仿真与 Gravity 相同示教语义。",
      Impedance: "Impedance：真机柔顺保持进入时的姿态（不跟随拖动）。仿真为较慢跟随。",
    };
    if ($("modeHint")) $("modeHint").textContent = hints[ctrlMode] || "";
    const cap = $("modeCapHint");
    if (cap) {
      cap.hidden = ok;
      cap.textContent = ok
        ? ""
        : "Gravity / Gra+Fri / Impedance 已禁用：" + reason + "（真机力矩环未加载）。仿真臂仍可切这些模式做滑条示教。";
    }
    paintGravUrdf(a);
    paintTeachControls(a);
  }
  function simulatedTeachEnabled() {
    return last?.arm?.backend === "sim" && !!$("simTeachEnabled")?.checked;
  }
  function paintTeachControls(a) {
    const online=deviceState().online;
    const floating = ctrlMode === "Gravity" || ctrlMode === "Gra+Fri";
    const simulation = a.backend === "sim";
    const context = [ctrlMode,a.backend,!!a.recording,a.rec_teach].join(":");
    if (context !== teachContext) {
      teachContext = context;
      $("simTeachEnabled").checked = false;
      if (floating || a.recording) {
        releaseManual(); jointDirty = false;
        writeJointInputs(a.q_deg || []);
      }
    }
    const busy = ["running","paused"].includes(last?.program?.status);
    const readOnly = floating && !(simulation && simulatedTeachEnabled());
    $("manualTeachStatus").hidden = !online || !(floating || a.recording);
    $("simTeachOption").hidden = !online || !(floating && simulation);
    $("simTeachEnabled").disabled = !online || busy;
    $("manualTeachMessage").textContent = floating
      ? (a.recording ? "拖臂录制中 · 控制模式已锁定。" : "当前为手动拖臂模式。") + (simulation ? "关节角默认只读；开启下方仿真拖动后，滑条才会改变仿真姿态并被录制。" : "请用手引导机械臂，滑条仅显示实测位置，不下发关节目标。")
      : "界面操控录制中 · 可以发送位置或使用实时跟随；停止录制前不能切换控制模式。";
    $("jointList").querySelectorAll("input").forEach(input => {input.disabled = readOnly || busy;});
    $("cartList").querySelectorAll("input").forEach(input => {input.disabled = floating || busy;});
    $("btnCartUseCurrent").disabled=!online || floating || busy;
    for (const id of ["btnSendArm","btnHome","armFollow"]) $(id).disabled = !online || floating || busy || !!pendingCmd[id === "btnHome" ? "home" : "arm"];
    const cartUnsupported=spaceMode==="cart" && !last?.capabilities?.cartesian_follow;
    if(cartUnsupported)$("armFollow").disabled=true;
    $("armFollow").title=cartUnsupported?"重启更新后的站控服务以启用笛卡尔实时跟随":"";
    $("spaceBtns").querySelector('[data-space="cart"]').disabled = floating || busy;
    if (floating && spaceMode !== "joint") setSpace("joint");
    for (const id of ["gripRange","gripDeg","btnGripOpen","btnGripClose"]) $(id).disabled = !online || floating || busy;
    $("gravUrdf").disabled = !!a.recording || busy;
  }
  const urdfList = { files: [], dir: "", selected: "" };
  function paintGravUrdf(arm) {
    const sel = $("gravUrdf");
    if (!sel) return;
    const snapFiles = (arm && arm.urdf_files) || [];
    const files = snapFiles.length ? snapFiles : urdfList.files;
    const cur = (arm && arm.urdf) || urdfList.selected || "";
    const dir = urdfList.dir || "";
    if (document.activeElement === sel) return;
    const sig = files.join("|") + "::" + cur + "::" + dir;
    if (sel.dataset.sig === sig) return;
    sel.dataset.sig = sig;
    sel.innerHTML = files.length
      ? files.map((n) => "<option value='" + n + "'>" + n + "</option>").join("")
      : "<option value=''>urdf 目录为空</option>";
    if (cur && Array.from(sel.options).some((o) => o.value === cur)) sel.value = cur;
    sel.title = sel.value;
    const hint = $("gravUrdfHint");
    if (hint) {
      const where = dir || "robot_station\\urdf";
      hint.textContent = files.length
        ? ("当前：" + (cur || files[0]) + "。把 6 轴 URDF 放到 " + where)
        : ("目录为空：" + where + "。放入 .urdf 后刷新页面。");
    }
  }
  async function loadUrdfList() {
    try {
      const r = await fetch("/api/urdf");
      if (!r.ok) return;
      const body = await r.json();
      urdfList.files = (body.files || []).map((it) => it.name || it).filter(Boolean);
      urdfList.dir = body.dir || "";
      urdfList.selected = body.selected || "";
      paintGravUrdf((last && last.arm) || {});
    } catch (e) {}
  }
  function paintMotorStatus(a) {
    const table = $("motorTable");
    const hint = $("motorHint");
    if (!table) return;
    const state=deviceState();
    if (!state.online) {
      const text=state.fresh?"设备未连接，连接后读取电机反馈。":"设备状态未更新，暂不显示旧电机反馈。";
      table.replaceChildren();
      if (hint) hint.textContent=text;
      return;
    }
    const motors = (a && a.motors) || [];
    if (!motors.length) {
      const why = (a && a.link_err) ? String(a.link_err) : "";
      table.innerHTML = "<tr><td colspan='4'>" + (why ? ("未读到电机：" + why) : "未读到电机（先 Connect，再点使能）") + "</td></tr>";
      if (hint) hint.textContent = why || "点「使能」后查看各轴是否在线、有无故障、当前模式。";
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
        "<td class='" + (ok ? "st-on" : "st-off") + "'>" + (ok ? "正常" : (m.fault_text || ("故障 " + (m.fault || "")))) + "</td>" +
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
    $("cliTxt").textContent = String(d.clients || 0);
    if (rtt == null && d.rtt_ms != null) $("rttTxt").textContent = Number(d.rtt_ms).toFixed(1) + " ms";
    // The connection card has one renderer in connections.js, including offline feedback.
    if (a.link_err && deviceState().online && String(a.link_err).indexOf("离线：") !== 0) setAck(a.link_err, false);
    paintMotorStatus(a);
    paintCmdButtons(a, estop);
    paintArmFeedback(a);
  }
  function paintArmFeedback(a) {
    if (!deviceState().online) {
      const text=deviceState().fresh?"设备未连接，等待反馈":"设备状态未更新";
      $("armKv").innerHTML=kv("实测角度",[["状态",text]]);
      if ($("eeKv")) $("eeKv").innerHTML=kv("力矩与控制状态",[["状态",text]]);
      return;
    }
    const measured = a.feedback_deg?.length >= 6 ? a.feedback_deg : (a.q_deg || []);
    const rows = measured.map((q, i) => [
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
      $("eeKv").innerHTML = kv("力矩与控制状态", [
        ["外力", "无腕部 F/T"],
        ["关节力矩 raw", tau],
        ["动力学", a.dyn_ready ? "pinocchio URDF" : "站控 URDF（笛卡尔 IK；重力 G(q)）"],
        ["重力 URDF", a.urdf || "—"],
        ["力矩模式", a.float_ok ? "可用" : ((a.float_reason || "真机力矩环未加载") + "（已禁用）")],
        ["重力环", a.grav_active ? "运行中" : "关"],
        ["录制", a.recording ? ((a.rec_frames || 0) + " 帧") : (a.replay_active ? "回放中" : "关")],
      ]);
    }
  }

  function paintSafety(d) {
    const estop = !!(d && d.safety === "ESTOP_LATCHED");
    const banner = $("estopBanner");
    if (banner) banner.classList.toggle("on", estop);
    if ($("safeTxt")) $("safeTxt").textContent = deviceState().fresh?((d && d.safety) || "—"):"状态未知";
    if ($("btnEstop")) $("btnEstop").classList.toggle("is-on", estop);
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
    const names = { 0: "IDLE", 1: "OPERATING", 2: "ESTOP_LATCHED", 3: "WATCHDOG_TRIP" };
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
    if (last && ((t.seq-(last.seq>>>0))|0)<0) return;
    last = mergeSnap(t);
    lastTelemAt = performance.now();
    paintGlobalStatus();
    const a = last.arm || {};
    if (window.Arm3D) Arm3D.setPose(a.q_deg || [], a.gripper_deg, a.gripper_open);
    if (last.echo_t0 != null) applyEchoRtt(last.echo_t0);
    paintSafety(last);
    paintCartPose(a);
    if (teleopHot()) return;
    if (currentPage() === "arm/control") syncJointsFromSnap(a);
    const estop = last.safety === "ESTOP_LATCHED";
    $("estopBanner").classList.toggle("on", estop);
  }
  function connectBus() {
    const prev = ws;
    if (prev) {
      prev.onclose = null;
      try { prev.close(); } catch (e) {}
    }
    const sock = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/cmd");
    ws = sock;
    sock.onopen = () => { if (ws === sock) flushCmdQueue(); };
    sock.onclose = () => {
      if (ws !== sock) return;
      releaseManual(false);
      paintGlobalStatus();
      if (live) setTimeout(() => { if (ws === sock) connectBus(); }, 800);
    };
    sock.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.t === "pong") {
        onPong(msg);
        return;
      }
      if (msg.t === "hello" && msg.cid) myCid = msg.cid;
      if (msg.t === "snap") return;
      if (msg.t === "ack") {
        window.dispatchEvent(new CustomEvent("station:ack", {detail:msg}));
        if(handleCartFollowAck(msg))return;
        if (msg.op === "workbench") return;
        const want = pendingCmd[msg.op] && pendingCmd[msg.op].want;
        if (msg.op === "clear") {
          endCmd("clear", !!msg.ok, msg.ok ? "已解除急停" : (msg.error || "失败"));
        } else if (msg.op === "script") {
          endCmd("script", !!msg.ok, msg.ok ? "已执行映射" : ("失败：" + (msg.error || "")));
          if ($("scriptAck")) {
            $("scriptAck").textContent = msg.ok ? "已执行映射" : ("失败：" + (msg.error || ""));
            $("scriptAck").className = "hint " + (msg.ok ? "tone-ok" : "tone-bad");
          }
        } else if (msg.op === "cart" || msg.op === "teach") {
          if (!msg.ok && !vendorEnableNoise(msg.error, last && last.arm)) {
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
          const arm = last && last.arm;
          const ok = !!msg.ok || vendorEnableNoise(msg.error, arm);
          const confirmed = want ? arm?.enabled && !last?.connection?.enable_required : !arm?.enabled;
          if (!ok || (pendingCmd.power && arm && confirmed)) {
            endCmd("power", ok, ackText("power", ok, ok ? "" : msg.error, want));
          }
          paintPowerButton();
          if (ok && arm) paintMotorStatus(arm);
        } else if (msg.op === "arm_src") {
          endCmd("arm_src", !!msg.ok, ackText("arm_src", !!msg.ok, msg.error, want));
        } else {
          const arm = last && last.arm;
          const ok = !!msg.ok || vendorEnableNoise(msg.error, arm);
          const pick = pendingCmd[msg.op] && pendingCmd[msg.op].pick;
          endCmd(msg.op, ok, ackText(msg.op, ok, ok ? "" : msg.error, want));
          if (ok && (msg.op === "rec_start" || msg.op === "rec_stop" || msg.op === "rec_delete" || msg.op === "rec_rename")) {
            loadTrajList(pick);
          }
          if (ok && msg.op === "grav_urdf") loadUrdfList();
        }
      }
    };
  }
  function connectTelem() {
    const prev = telemWs;
    if (prev) {
      prev.onclose = null;
      try { prev.close(); } catch (e) {}
    }
    const sock = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws");
    telemWs = sock;
    sock.binaryType = "arraybuffer";
    sock.onclose = () => {
      if (telemWs!==sock) return;
      pendingTelem=null;
      lastPowerFeedbackAt=0;
      paintGlobalStatus();
      if (live) setTimeout(() => { if (telemWs===sock) connectTelem(); },800);
    };
    sock.onmessage = (ev) => {
      if (telemWs!==sock) return;
      if (ev.data instanceof ArrayBuffer) {
        pendingTelem = ev.data;
        if (!snapRaf) snapRaf = requestAnimationFrame(flushSnap);
        return;
      }
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.t === "hud" && msg.d) {
        if (teleopHot()) {
          acceptSnapshot(mergeSnap(msg.d));
          paintSafety(last);
          paintCmdErr(msg.d, last.arm || {});
          const a = last.arm || {};
          paintCartPose(a);
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
    const prev = rttWs;
    if (prev) {
      prev.onclose = null;
      try { prev.close(); } catch (e) {}
    }
    const sock = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/rtt");
    rttWs = sock;
    sock.onclose = () => { if (rttWs === sock && live && !rttWorker) setTimeout(() => { if (rttWs === sock) connectRttSocket(); }, 1000); };
    sock.onmessage = (ev) => {
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
  function nudgeSockets() {
    if (!live) return;
    if (!ws || ws.readyState > 1) connectBus();
    if (!telemWs || telemWs.readyState > 1) connectTelem();
    if (!rttWorker && (!rttWs || rttWs.readyState > 1)) connectRttSocket();
  }

  async function boot() {
    live = true;
    ensureDefaultHash();
    showPage();
    syncStage();
    try {
      const ac = new AbortController();
      const timer = setTimeout(() => ac.abort(), 2000);
      const r = await fetch("/api/info", { signal: ac.signal });
      clearTimeout(timer);
      if (r.ok) info = await r.json();
    } catch (e) {}
    if ($("collectBanner")) {
      $("collectBanner").classList.add("on");
      $("collectBanner").textContent = "等待设备状态 · 在「总览与连接」管理连接。";
    }
    await loadTrajList();
    await loadUrdfList();
    startControlLoop(info && info.control_hz);
    connectBus();
    connectTelem();
    connectRtt();
    document.addEventListener("visibilitychange", () => {
      if (!document.hidden) nudgeSockets();
    });
    window.addEventListener("focus", nudgeSockets);
  }

  $("btnEstop").onclick = () => fireEstop();
  $("btnClear").onclick = () => fireClearEstop();
  function syncSpeed(fromRange) {
    const range = $("armSpeedRange");
    const num = $("armSpeed");
    if (fromRange) num.value = range.value;
    else range.value = num.value;
    const fill = 100 * (Number(range.value) - Number(range.min)) / (Number(range.max) - Number(range.min));
    range.style.setProperty("--speed-fill", fill + "%");
  }
  $("armSpeedRange").addEventListener("input", () => syncSpeed(true));
  $("armSpeed").addEventListener("input", () => syncSpeed(false));
  syncSpeed(true);
  $("btnSendArm").onclick = sendPosition;
  function fireEstop() {
    cartHeld.clear();
    stopFollowStream();
    const sent = send({ t: "e" });
    pulseBtn($("btnEstop"));
    if ($("btnEstop")) $("btnEstop").classList.add("is-on");
    flashHot("estop", "急停");
    setAck(sent ? "急停：电机已刹车（不是位置保持，也不是断电卸力）" : "急停尚未送到，连接恢复后补发", false);
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
    stopFollowStream(false);
    jointDirty = false;
    send({ t: "home", speed: Number($("armSpeed").value || 40) });
  }
  function firePower(on) {
    paintPowerButton();
    if ($("btnPowerToggle").disabled) { setAck($("btnPowerToggle").title, false); return; }
    if (!last?.connection?.enable_required && !!last.arm.enabled === !!on) { setAck(on ? "机械臂已使能" : "机械臂已去使能", true); return; }
    if (!on) releaseManual();
    const id = "btnPowerToggle";
    if (!beginCmd("power", {
      id: id,
      busy: on ? "正在使能…" : "正在去使能…",
      hot: on ? "enable" : "disable",
      ack: on ? "正在使能，随后显示各轴在线状态…" : "正在去使能…",
      want: on,
    })) return;
    paintPowerButton();
    try { ws.send(JSON.stringify(withSession({ t: "power", on: !!on }))); }
    catch (error) { endCmd("power", false, "设备连接已断开，请重连后重试"); paintPowerButton(); }
  }
  function fireLink(action) { window.StationConnection?.perform(action); }
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
      if (!box || box.disabled) return;
      box.checked = !box.checked;
      box.dispatchEvent(new Event("change"));
      if (!box.checked) {
        flashHot("follow", "跟随关");
        setAck("已关闭实时跟随", true);
      } else {
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
  $("simTeachEnabled").onchange = () => {
    jointDirty = false; followDirty = false;
    writeJointInputs(last?.arm?.q_deg || []);
    paintTeachControls(last?.arm || {});
  };
  if ($("btnHomeCart")) $("btnHomeCart").onclick = goHome;
  if ($("armFollow")) {
    $("armFollow").addEventListener("change", () => {
      const row = $("armFollow").closest(".follow-row");
      if (row) row.classList.toggle("is-on", $("armFollow").checked);
      if (!$("armFollow").checked) stopFollowStream();
      else if(spaceMode==="cart"){
        cancelCartFollow();
        writeCartInputs(last?.arm?.ee_m,last?.arm?.ee_rpy_deg);cartPrimed=true;cartDirty=false;
      }else jointDirty = false;
    });
  }
  $("btnCartUseCurrent").onclick=useCurrentCartPose;
  $("btnGripOpen").onclick = () => {
    fireHotCmd("grip-open");
  };
  $("btnGripClose").onclick = () => {
    fireHotCmd("grip-close");
  };
  $("btnPowerToggle").onclick = () => firePower(!last?.arm?.enabled || !!last?.connection?.enable_required);
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
  $("modeBtns").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-mode]");
    if (!btn || btn.disabled) return;
    const next = btn.dataset.mode;
    const arm = (last && last.arm) || {};
    if (floatModeNeedsDynamics(next) && !floatModesOk(arm)) {
      setAck(next + "：" + (arm.float_reason || "真机力矩环未加载"), false);
      return;
    }
    ctrlMode = next;
    paintCtrlMode(arm);
    pulseBtn(btn);
    stopFollowStream(false);
    if (ctrlMode === "Gravity" || ctrlMode === "Gra+Fri") setSpace("joint");
    send({ t: "mode", mode: ctrlMode });
    setAck("已切到 " + ctrlMode, true);
  });
  if ($("gravUrdf")) {
    $("gravUrdf").addEventListener("change", () => {
      const name = $("gravUrdf").value;
      if (!name) { setAck("urdf 目录为空", false); return; }
      if (!beginCmd("grav_urdf", { ack: "正在切换重力 URDF…" })) return;
      send({ t: "grav_urdf", name: name });
    });
  }
  async function loadTrajList(prefer) {
    window.dispatchEvent(new CustomEvent("station:trajectories", {detail:{prefer}}));
  }
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

  $("workNavSelect").onclick = ev => {
    const open = $("workNavMenu").hidden;
    setWorkMenu(open);
    if (open && ev.detail === 0) $("workNavMenu").querySelector("a").focus();
  };
  $("workNavEntry").onclick = () => setWorkMenu(false);
  document.querySelectorAll("#workNavMenu a").forEach(a => a.onclick = () => setWorkMenu(false, true));
  $("workNav").addEventListener("keydown", ev => {
    const open = !$("workNavMenu").hidden;
    if ((ev.key === "Escape" || ev.key === "Tab") && open) {
      setWorkMenu(false, true);
      return; // Escape still reaches the existing global emergency-stop handler.
    }
    if (!["ArrowDown", "ArrowUp"].includes(ev.key) && !(open && ["Home", "End"].includes(ev.key))) return;
    ev.preventDefault(); ev.stopPropagation();
    setWorkMenu(true);
    const items = Array.from($("workNavMenu").querySelectorAll("a"));
    const current = items.indexOf(document.activeElement);
    const next = ev.key === "Home" ? 0 : ev.key === "End" ? items.length - 1 :
      current < 0 ? (ev.key === "ArrowUp" ? items.length - 1 : 0) :
      (current + (ev.key === "ArrowUp" ? -1 : 1) + items.length) % items.length;
    items[next].focus();
  });
  document.addEventListener("click", ev => { if (!$("workNav").contains(ev.target)) setWorkMenu(false); });
  document.addEventListener("focusin", ev => { if (!$("workNav").contains(ev.target)) setWorkMenu(false); });
  window.addEventListener("resize", () => setWorkMenu(false));
  window.addEventListener("hashchange", showPage);
  window.addEventListener("keydown", (ev) => {
    if (!live) return;
    if (ev.code === "Escape") {
      ev.preventDefault();
      if ($("confirmDialog").open) $("confirmDialog").close("cancel");
      fireEstop();
      return;
    }
    if (currentPage() !== "arm/control" || typing()) return;
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
    if (!CART_KEYS[ev.code] || !cartHeld.has(ev.code)) return;
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
  window.addEventListener("resize", layoutHotkeyPop);
  if (window.ResizeObserver) new ResizeObserver(layoutHotkeyPop).observe(document.querySelector(".top"));

  setInterval(() => {
    if (live) sendTouch();
    flushHz();cmdQueue.sweep(dropQueued);paintGlobalStatus();paintPowerButton();
    if (!deviceState().online) {
      const arm=last?.arm || {};
      paintMotorStatus(arm);paintArmFeedback(arm);paintCartPose(arm);paintTeachControls(arm);
      setBtnOn("btnGripOpen",false);setBtnOn("btnGripClose",false);
    }
  }, 100);
  setInterval(() => { if (live && !rttWorker) sendPing(); }, 250);

  window.StationUI = {
    snapshot: () => last,
    connected: () => !!(ws && ws.readyState === 1),
    deviceState,
    readTargets: () => limits.map(([min,max],i) => window.StationNumbers.read($("jointList").querySelector("[data-jn='"+i+"']"),{label:"J"+(i+1)+" 目标角度",min,max})),
    releaseManual: releaseManual,
    page: currentPage,
    refreshStage: syncStage,
    notice: setAck,
    sendImmediate: msg => {
      if (!ws || ws.readyState !== 1) throw new Error("站控连接尚未就绪，请稍后重试");
      ws.send(JSON.stringify(withSession(msg)));
    }
  };
  buildJoints();
  bindCartSliders();
  showPage();
  boot();
})();
