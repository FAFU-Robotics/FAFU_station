/* Overview reads existing telemetry; connection checks only enumerate devices. */
(() => {
  "use strict";
  const $ = id => document.getElementById(id);
  const ui = window.StationUI;
  const escape = value => String(value ?? "").replace(/[&<>"']/g, x => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[x]));
  let snapshot = ui.snapshot(), diagnostics = null, busy = false, checked = 0, checkError = "";
  const metric = (label, value, detail, tone = "") => `<div class="metric"><div class="k">${escape(label)}</div><div class="v ${tone}">${escape(value)}</div><div class="s">${escape(detail)}</div></div>`;
  const row = (label, value) => `<dt>${escape(label)}</dt><dd>${value}</dd>`;
  const status = (ok, text) => `<span class="${ok ? "tone-ok" : "tone-warn"}">${escape(text)}</span>`;
  const states = {idle:"未运行", running:"运行中", paused:"已暂停", completed:"已完成", stopped:"已停止", error:"执行失败"};

  function paint() {
    if (ui.page() !== "home") return;
    const d = snapshot || {}, a = d.arm || {}, p = d.program || {};
    const {connected,fresh,online,kind}=ui.deviceState(), enabled = online && a.enabled;
    const simulated = kind === "simulation";
    const estop = d.safety === "ESTOP_LATCHED";
    const motors = a.motors || [], good = motors.filter(m => m.online && m.ok !== false && !m.fault).length;
    const task = ["running", "paused", "error"].includes(p.status) ? (states[p.status] || p.status)
      : a.recording ? "录制中" : a.replay_active ? "回放中" : a.moving ? "手动运动" : "空闲";
    const latency = d.rtt_ms == null ? "等待测量" : Number(d.rtt_ms).toFixed(1) + " ms";
    $("homeMetrics").innerHTML =
      metric("设备连接", online ? "已连接" : fresh ? "未连接" : "状态未知", online ? (simulated ? "本机仿真" : a.serial_port || "串口机械臂") : fresh ? "在下方选择目标并连接" : "等待最新状态", online ? "tone-ok" : "tone-warn") +
      metric("电机与使能", online ? (enabled ? "已使能" : "未使能") : "—", online ? (enabled && d.connection?.enable_required ? "等待确认使能，再接管当前姿态" : motors.length ? `${good}/${motors.length} 轴在线正常` : simulated ? "仿真后端" : "等待电机反馈") : "连接后读取各轴反馈", enabled ? "tone-ok" : "tone-warn") +
      metric("急停状态", fresh ? (estop ? "已锁存" : "未触发") : "—", fresh ? (d.safety_reason || (estop ? "需解除急停" : "运动前仍需确认连接与使能")) : "等待安全状态", estop ? "tone-bad" : "") +
      metric("当前任务", fresh ? task : "—", fresh ? (p.name || a.traj_file || (enabled ? a.ctrl_mode || "Position" : "等待使能")) : "等待任务状态", p.status === "error" ? "tone-bad" : "") +
      metric("通信", fresh ? latency : connected ? "状态超时" : "已断开", fresh ? "设备状态实时更新" : "设备状态暂未更新", fresh ? "" : "tone-warn");

    const notes = [];
    if (!fresh) notes.push("设备状态未更新，请检查连接；必要时重新打开站控。");
    else {
      if (online && simulated) notes.push("当前为仿真设备，操作只作用于模拟机械臂。");
      if (!online) notes.push(d.connection?.status === "error" ? d.connection.error || a.link_err || "连接失败，请检查所选设备。" : "设备未连接，请在左侧选择串口机械臂或本机仿真后连接。");
      else if (a.link_err) notes.push(a.link_err);
      if (estop) notes.push("急停已锁存；确认现场状态后再解除。");
      if (online && motors.length && good < motors.length) notes.push(`${motors.length - good} 个轴离线或有故障，详情见电机状态。`);
      if (online && !a.enabled) notes.push("设备未使能，检查各轴后按需使能。");
      if (p.error) notes.push("程序：" + p.error);
      else if (["running", "paused"].includes(p.status)) notes.push(`程序${states[p.status]}，手动运动接口由程序占用。`);
      if (!notes.length) notes.push("连接与电机状态正常，可进入手动控制或选择作业功能。");
    }
    $("homeNotices").innerHTML = notes.map(note => `<li>${escape(note)}</li>`).join("");
    $("deviceScanSummary").textContent = online && simulated ? "当前已连接本机仿真。下方仅列出可选串口，检测不会改变当前连接。" :
      online && a.serial_port ? "机械臂当前连接：" + a.serial_port : "选择下方已识别的串口，或在左侧手动输入端口；也可选择本机仿真。";
    let checks = row("网页服务", status(true, "已响应") + escape(location.origin));
    checks += row("控制通道", status(fresh, fresh ? "在线" : connected ? "等待最新状态" : "已断开"));
    if (diagnostics) {
      const motion = diagnostics.motion;
      checks += row("运动进程", status(motion.connected && fresh, !fresh ? "状态未更新" : motion.mode === "in_process" ? "同进程服务" : motion.connected ? "已连接" : "未连接") +
        (motion.mode === "in_process" ? "无需独立 TCP 端口" : escape(`${motion.host}:${motion.port}`)));
      checks += row("SDK", status(diagnostics.serial.sdk_loaded, diagnostics.serial.sdk_loaded ? "已加载" : "未就绪"));
      const serialPort = a.serial_port || diagnostics.serial.active_port;
      checks += row("设备串口", escape(!online ? "未连接设备" : simulated ? "本机仿真，无需串口" : serialPort || "设备未报告端口"));
      checks += row("串口访问", status(!!diagnostics.permissions?.serial, diagnostics.permissions?.serial ? "已开放" : "此进程未开放，需通过「启动真机」启动站控"));
    }
    if (checkError) checks += row("检查结果", status(false, checkError));
    $("connectionChecksList").innerHTML = checks;
  }

  async function refresh() {
    if (busy) return;
    busy = true;
    $("btnRefreshDiagnostics").disabled = true;
    $("serialScanTime").textContent = "检查中…";
    const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 8000);
    try {
      const response = await fetch("/api/diagnostics", {cache:"no-store", signal:controller.signal});
      if (!response.ok) throw new Error("检查服务不可用，请刷新页面或重启预览服务");
      diagnostics = await response.json();
      window.dispatchEvent(new CustomEvent("station:diagnostics",{detail:diagnostics}));
      checked = Date.now(); checkError = "";
      const serial = diagnostics.serial;
      $("serialPorts").innerHTML = serial.error ? `<div class="serial-empty"><b>设备检查失败</b><p>${escape(serial.error)}</p></div>` : serial.ports.length ? serial.ports.map(port =>
        `<div class="serial-port"><b>${escape(port.port)}</b><span>${escape(port.description || "无设备描述")}</span>` +
        (port.vid ? `<span>VID:PID ${escape(port.vid)}:${escape(port.pid)}</span>` : "") +
        `<span class="${port.candidate ? "port-candidate" : ""}">${port.candidate ? "候选调试板" : "其它串口"}</span></div>`
      ).join("") : '<div class="serial-empty"><b>未检测到串口设备</b><p>连接机械臂 USB 并打开设备电源后刷新。仍未识别时，请检查 USB 线和串口驱动。</p></div>';
      $("serialScanTime").textContent = new Date(diagnostics.checked_at).toLocaleTimeString();
    } catch (error) {
      checkError = error.name === "AbortError" ? "检查超时，请重试" : error.message;
      diagnostics = null;
      $("serialScanTime").textContent = "检查失败";
      $("serialPorts").textContent = checkError;
    } finally {
      clearTimeout(timeout); busy = false; $("btnRefreshDiagnostics").disabled = false; paint();
    }
  }
  function enterHome() {
    if (ui.page() !== "home") return;
    paint();
    if (!diagnostics || Date.now() - checked > 30000) refresh();
  }
  $("btnRefreshDiagnostics").onclick = refresh;
  window.addEventListener("station:page", enterHome);
  window.addEventListener("station:snapshot", ev => {
    snapshot = ev.detail; paint();
  });
  setInterval(paint, 1000);
  enterHome();
})();
