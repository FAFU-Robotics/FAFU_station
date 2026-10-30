/* Selecting a target never opens it. One explicit connection owns motion. */
(() => {
  "use strict";
  const $=id=>document.getElementById(id),ui=window.StationUI;
  const kind=$("connectionKind"),port=$("connectionPort"),custom=$("connectionCustomPort");
  const connect=$("btnArmConnect"),disconnect=$("btnArmDisconnect"),hint=$("connectionTargetHint");
  let snap=ui.snapshot(),diagnostics=null,pending=null,timer=0,sequence=0,seenSession="",draftTouched=false;
  const selectedPort=()=>port.value==="__manual"?custom.value.trim():port.value;
  const serialPermissionHint="此站控进程未开放串口访问。请通过「启动真机」启动站控，再在此页面连接串口；同一界面也支持本机仿真。";
  function choosePort(value) {
    if (!value) {port.value="";custom.value="";return;}
    if (Array.from(port.options).some(option=>option.value===value)) port.value=value;
    else {port.value="__manual";custom.value=value;}
  }
  function rebuildPorts() {
    const manual=port.value==="__manual";
    const selected=selectedPort() || (!draftTouched?diagnostics?.serial?.configured_port:"") || "";
    port.replaceChildren(new Option("请选择串口",""));
    const ports=diagnostics?.serial?.ports || [];
    ports.forEach(item=>port.add(new Option(item.port+" · "+(item.description||"串口设备")+(item.candidate?" · 候选控制板":""),item.port)));
    port.add(new Option("手动输入端口…","__manual"));
    const candidates=ports.filter(item=>item.candidate);
    if (manual) port.value="__manual";
    else choosePort(selected || (candidates.length===1?candidates[0].port:""));
  }
  function feedback(text,ok=true,options={}) {
    $("connAck").textContent=text;
    $("connAck").className="hint "+(ok?"tone-ok":"tone-bad");
    ui.notice(text,ok,options);
  }
  function finish() {
    if (!pending?.ack) return;
    const connection=snap?.connection;
    if (!connection || connection.session===pending.before || ["connecting","disconnecting"].includes(connection.status)) return;
    const action=pending.action;
    clearTimeout(timer);pending=null;
    if (connection.status!==(action==="disconnect"?"disconnected":"connected")) {
      feedback(connection.error || "连接状态已变化，请检查当前设备后重试。",false);
      return;
    }
    feedback(action==="disconnect"?"设备已断开":connection.kind==="simulation"?"已连接本机仿真，请按需使能":"已连接 "+connection.port+"，请检查反馈后使能");
  }
  function paintDetails(c,a,online,fresh,working) {
    // A draft endpoint is not a connected device. Never reuse its last motor feedback.
    const confirmed=c.status==="connected";
    const target=confirmed?c:working?(pending?.target || c):{kind:kind.value,port:selectedPort()};
    const serial=target.kind==="serial";
    const suffix=online?"":confirmed?"（状态未更新）":working?"（切换中）":"（待连接）";
    $("armConnectionKind").textContent=(serial?"串口机械臂":"本机仿真")+suffix;
    $("armServerUrl").textContent=serial?(target.port || (online?a.serial_port:"") || "未选择串口")+suffix:
      "本机仿真，无需串口"+suffix;
    const motors=a.motors || [],count=motors.filter(m=>m.online).length;
    $("armLinkTxt").textContent=!fresh?"状态未更新":online?
      (motors.length && count<motors.length?"已连接 · "+count+"/"+motors.length+" 轴在线":"已连接"):
      ({connecting:"正在连接…",disconnecting:"正在断开…",error:"连接失败"}[c.status] || "未连接");
    $("armEnabled").textContent=online?(a.enabled?(c.enable_required?"已使能 · 等待确认":"已使能"):"未使能"):"—";
    $("armGripTxt").textContent=online && a.has_gripper && a.gripper_open!=null?(a.gripper_open?"打开":"闭合"):"—";
    $("armRobotName").textContent=online?(serial?"FAFU":"FAFU（仿真）"):"—";
    $("armAxisCount").textContent=online && a.n?(a.has_gripper?(a.n+1)+"（"+a.n+" + 夹爪）":String(a.n)):"—";
    $("armCtrlMode").textContent=online?(a.ctrl_mode || "—"):"—";
    $("connectionFeedbackHint").hidden=!!online;
    $("connectionFeedbackHint").textContent=fresh?"设备反馈将在连接后读取。":"设备状态未更新，暂不显示旧反馈。";
  }
  function paint() {
    const c=snap?.connection || {},a=snap?.arm || {};
    const {fresh,online}=ui.deviceState();
    const supported=!!snap?.capabilities?.connection_profiles;
    const working=!!pending || ["connecting","disconnecting"].includes(c.status);
    const serial=kind.value==="serial";
    const permission=diagnostics?.permissions?.serial;
    const task=["running","paused"].includes(snap?.program?.status) || a.recording || a.replay_active;
    kind.disabled=port.disabled=custom.disabled=c.status==="connected" || working;
    $("serialConnectionFields").hidden=!serial;
    custom.hidden=port.value!=="__manual";
    // A denied serial permission has an explanatory click action, never a hardware request.
    const blocked=!supported?"请重启站控加载新版连接功能":!fresh?"等待最新设备状态":working?"等待连接操作完成":c.status==="connected"?"请先断开当前设备":
      serial && !selectedPort()?"请选择或手动输入串口":serial && permission==null?"正在读取串口访问权限":"";
    connect.disabled=!!blocked;
    disconnect.disabled=!supported || !fresh || working || c.status!=="connected" || !!task;
    connect.textContent=pending?.action==="connect" || c.status==="connecting"?"正在连接…":serial?"连接串口":"连接仿真";
    connect.title=blocked || (serial && permission===false?"此进程未开放串口访问，点击查看连接方式":"连接所选设备，之后再单独使能");
    disconnect.textContent=pending?.action==="disconnect" || c.status==="disconnecting"?"正在断开…":"断开";
    const labels={connected:"已连接",disconnected:"未连接",connecting:"正在连接…",disconnecting:"正在断开…",error:"连接失败"};
    const endpoint=c.kind==="simulation"?"本机仿真":c.port?"串口 · "+c.port:"串口机械臂";
    const partial=online && (a.motors || []).some(m=>!m.online);
    $("armModeBadge").textContent=!fresh?"等待设备状态":online?"已连接 · "+endpoint+(partial?" · 部分轴离线":""):
      working?(pending?.action==="disconnect" || c.status==="disconnecting"?"正在断开…":"正在连接…"):(labels[c.status==="connected"?"disconnected":c.status]||"未连接");
    $("armDot").className="status-dot "+(!fresh?"":online?(partial?"warn":"connected"):working?"warn":"off");
    hint.textContent=!supported?"请重启站控加载新版连接功能，再刷新页面。":!fresh?"等待最新连接状态…":
      working?"等待连接操作完成，请勿重复操作。":online?"当前目标已锁定。切换串口或仿真前，请先断开。":
      serial?(permission===false?serialPermissionHint:permission==null?"正在读取串口访问权限…":"选择或手动输入 FAFU 控制板所在串口，点击「连接串口」或按回车。连接后再单独使能。"):
      "本机运行模拟机械臂，不打开串口。点击连接后再单独使能。";
    hint.classList.toggle("tone-warn",!!(fresh && !online && !working && serial && permission===false));
    paintDetails(c,a,online,fresh,working);
    if (!pending && c.status==="error" && c.kind===kind.value && (!serial || c.port===selectedPort())) {
      $("connAck").textContent=c.error || a.link_err || "连接失败，请检查端口和设备状态。";
      $("connAck").className="hint tone-bad";
    }
  }
  function perform(action) {
    paint();
    if ((action==="connect"?connect:disconnect).disabled) return;
    if (action==="connect" && kind.value==="serial" && diagnostics?.permissions?.serial!==true) {
      feedback(selectedPort()+" 尚未连接。"+serialPermissionHint,false,{force:true});
      return;
    }
    const packet={t:"link",action,request_id:"connection-"+(++sequence)};
    if (action==="connect") {
      packet.kind=kind.value;
      if (packet.kind==="serial") packet.port=selectedPort();
    }
    ui.releaseManual();
    pending={action,id:packet.request_id,before:snap.connection.session,ack:false,
      target:action==="connect"?{kind:packet.kind,port:packet.port}:snap.connection};
    feedback(action==="connect"?"正在连接所选设备…":"正在断开设备…",true,{tone:"info",pending:true,force:true});
    try {ui.sendImmediate(packet);}
    catch (error) {pending=null;feedback(error.message,false);paint();return;}
    timer=setTimeout(()=>{pending=null;feedback("未收到连接确认，请检查最新设备状态后重试。",false);paint();},28000);
    paint();
  }
  $("connectionForm").onsubmit=event=>{event.preventDefault();perform("connect");};
  disconnect.onclick=()=>perform("disconnect");
  function changeDraft() {draftTouched=true;$("connAck").textContent="";paint();}
  kind.onchange=changeDraft;
  port.onchange=()=>{changeDraft();if(port.value==="__manual")custom.focus();};
  custom.oninput=changeDraft;
  window.addEventListener("station:diagnostics",event=>{diagnostics=event.detail;rebuildPorts();paint();});
  window.addEventListener("station:ack",event=>{
    const ack=event.detail;
    if (ack.op!=="link" || !pending || ack.request_id!==pending.id) return;
    if (!ack.ok) {clearTimeout(timer);pending=null;feedback(ack.error||"连接操作失败",false);}
    else {pending.ack=true;finish();}
    paint();
  });
  window.addEventListener("station:snapshot",event=>{
    const next=event.detail;
    snap=next;
    const c=snap.connection;
    if (c?.session && c.session!==seenSession && (c.status==="connected" || !draftTouched)) {
      seenSession=c.session;kind.value=c.kind;if(c.kind==="serial")choosePort(c.port);draftTouched=false;
    }
    finish();paint();
  });
  setInterval(paint,500);
  window.StationConnection={perform};
  paint();
})();
