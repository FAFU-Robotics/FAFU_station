"use strict";
// Exercise the complete page so app.js and connections.js cannot fight over the UI.
// WebSockets and device discovery terminate in this browser fixture, never hardware.
const assert=require("node:assert/strict"),fs=require("node:fs"),http=require("node:http"),os=require("node:os"),path=require("node:path");
const {spawn,execFileSync}=require("node:child_process");
const root=path.resolve(__dirname,"../../robot_station/web/static");
const browser=[process.env.CHROME_BIN,"C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe","C:/Program Files/Google/Chrome/Application/chrome.exe","/usr/bin/chromium","/usr/bin/google-chrome"].find(p=>p&&fs.existsSync(p));
if(!browser){console.log("SKIP: Chrome/Chromium not installed");process.exit(0);}
const fixture=`
window.fixture={errors:[],sent:[],sockets:[],paused:false,permission:true,
  snap:{seq:1,safety:"OPERATING",clients:1,program:{status:"idle"},capabilities:{connection_profiles:1,motor_disable:true,cartesian_follow:true},
    connection:{kind:"simulation",port:"",status:"disconnected",session:"initial",enable_required:true},
    arm:{online:false,enabled:true,backend:"sim",n:6,has_gripper:true,gripper_open:true,ctrl_mode:"Gravity",q_deg:[0,40,40,0,0,0],target_deg:[0,40,40,0,0,0],ok:[true,true,true,true,true,true],motors:[]}}};
const copy=value=>JSON.parse(JSON.stringify(value));
fixture.diagnostics=()=>({permissions:{serial:fixture.permission,motion:true},motion:{mode:"in_process",connected:true},checked_at:new Date().toISOString(),serial:{sdk_loaded:true,error:"",configured_port:"COM3",ports:[{port:"COM3",description:"Fixture serial device",candidate:true}]}});
const nativeFetch=fetch;window.fetch=(url,...args)=>{
  const data=url==="/api/info"?{arm:"sim",arm_allow_motion:true,live_serial_allowed:true,control_hz:100}:url==="/api/diagnostics"?fixture.diagnostics():url==="/api/workspace"?{waypoints:[],programs:[]}:String(url).startsWith("/api/")?{files:[]}:null;
  return data?Promise.resolve({ok:true,json:async()=>copy(data)}):nativeFetch(url,...args);
};
fixture.publish=()=>{fixture.snap.seq++;for(const socket of fixture.sockets)if(socket.readyState===1&&socket.url.endsWith("/ws"))socket.onmessage?.({data:JSON.stringify({t:"hud",d:fixture.snap})});};
fixture.pose=(seq,online)=>{const data=new ArrayBuffer(88),v=new DataView(data);v.setUint32(0,0x314E5453,true);v.setUint32(4,seq>>>0,true);v.setUint16(8,online?3:0,true);v.setUint8(10,1);for(const s of fixture.sockets)if(s.readyState===1&&s.url.endsWith("/ws"))s.onmessage?.({data});};
window.Worker=class{constructor(){throw new Error("Use the fixture RTT socket");}};
window.WebSocket=class{
  constructor(url){this.url=url;this.readyState=0;fixture.sockets.push(this);setTimeout(()=>{if(fixture.commandDown&&url.endsWith("/ws/cmd")){this.readyState=3;this.onclose?.({});return;}this.readyState=1;this.onopen?.({});if(url.endsWith("/ws/cmd"))this.deliver({t:"hello",cid:"fixture"});fixture.publish();},0);}
  deliver(message){this.onmessage?.({data:JSON.stringify(message)});}
  close(){this.readyState=3;this.onclose?.({});}
  send(raw){const message=JSON.parse(raw);fixture.sent.push(message);
    if(message.t==="ping"){this.deliver({...message,t:"pong"});return;}
    if(message.t!=="link")return;
    const c=fixture.snap.connection,a=fixture.snap.arm;
    c.session="session-"+fixture.snap.seq;c.status=message.action==="connect"?"connecting":"disconnecting";fixture.publish();
    setTimeout(()=>{if(message.action==="connect"){
      c.kind=message.kind;c.port=message.port||"";c.status="connected";
      Object.assign(a,{online:true,enabled:false,backend:message.kind==="serial"?"fafu":"sim",serial_port:c.port,ctrl_mode:"Position"});
    }else{c.status="disconnected";a.online=false;/* Deliberately retain stale motor feedback. */}
      fixture.publish();this.deliver({t:"ack",op:"link",ok:true,request_id:message.request_id});},100);
  }
};
fixture.edit=(id,value)=>{const el=document.getElementById(id);el.value=value;el.dispatchEvent(new Event(el.tagName==="SELECT"?"change":"input",{bubbles:true}));};
addEventListener("error",event=>fixture.errors.push(event.message));addEventListener("unhandledrejection",event=>fixture.errors.push(String(event.reason)));
setInterval(()=>{if(!fixture.paused)fixture.publish();},80);
`;
const html=fs.readFileSync(path.join(root,"index.html"),"utf8").replace("<head>",'<head><script src="/fixture.js"></script>');
const server=http.createServer((req,res)=>{
  if(req.url==="/"||req.url==="/fixture.js"){res.setHeader("Content-Type",req.url==="/"?"text/html; charset=utf-8":"text/javascript; charset=utf-8");res.end(req.url==="/"?html:fixture);return;}
  const file=path.resolve(root,decodeURIComponent(req.url.split("?")[0]).replace(/^\/static\//,""));
  if(!file.startsWith(root+path.sep)||!fs.existsSync(file)||!fs.statSync(file).isFile()){res.writeHead(404);res.end();return;}
  res.setHeader("Content-Type",file.endsWith(".css")?"text/css":file.endsWith(".js")?"text/javascript":"application/octet-stream");res.end(fs.readFileSync(file));
});
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function until(fn){const deadline=Date.now()+15000;while(Date.now()<deadline){const value=await fn();if(value)return value;await delay(25);}throw new Error("Timed out waiting for connection UI");}
async function main(){
  await new Promise(resolve=>server.listen(0,"127.0.0.1",resolve));
  const profile=fs.mkdtempSync(path.join(os.tmpdir(),"fafu-connections-"));
  const child=spawn(browser,["--headless=new","--no-first-run","--no-default-browser-check","--disable-background-networking","--disable-extensions","--use-angle=swiftshader","--enable-unsafe-swiftshader","--remote-debugging-port=0","--user-data-dir="+profile,"about:blank"],{stdio:"ignore",windowsHide:true});
  let socket,cdp;
  try{
    const port=await until(()=>{try{return fs.readFileSync(path.join(profile,"DevToolsActivePort"),"utf8").split("\n")[0].trim();}catch(error){if(["ENOENT","EACCES","EBUSY","EPERM"].includes(error.code))return false;throw error;}});
    const pages=await(await fetch("http://127.0.0.1:"+port+"/json/list")).json();
    socket=new WebSocket(pages.find(p=>p.type==="page").webSocketDebuggerUrl);await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
    let serial=0;const pending=new Map();socket.onmessage=event=>{const msg=JSON.parse(event.data),task=pending.get(msg.id);if(task){pending.delete(msg.id);msg.error?task.reject(new Error(msg.error.message)):task.resolve(msg.result);}};
    cdp=(method,params={})=>new Promise((resolve,reject)=>{const id=++serial;pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});
    const evaluate=async expression=>{const result=await cdp("Runtime.evaluate",{expression,returnByValue:true,awaitPromise:true});if(result.exceptionDetails)throw new Error(result.exceptionDetails.exception?.description||result.exceptionDetails.text);return result.result.value;};
    const wait=expression=>until(()=>evaluate(expression));
    await cdp("Emulation.setDeviceMetricsOverride",{width:1440,height:1050,deviceScaleFactor:1,mobile:false});
    await cdp("Page.navigate",{url:"http://127.0.0.1:"+server.address().port+"/#/home"});
    await wait('window.StationUI?.connected() && document.getElementById("armModeBadge").textContent==="未连接" && document.getElementById("connectionPort").options.length===3');
    await evaluate('fixture.labels=[];new MutationObserver(()=>fixture.labels.push(document.getElementById("armModeBadge").textContent)).observe(document.getElementById("armModeBadge"),{childList:true,subtree:true,characterData:true})');
    await delay(1250);
    assert.deepEqual(await evaluate('[...new Set(fixture.labels)]'),["未连接"]);
    assert.equal(await evaluate('document.getElementById("armGripTxt").textContent'),"—");
    assert.equal(await evaluate('document.getElementById("armCtrlMode").textContent'),"—");
    assert.equal(await evaluate('document.getElementById("mockTxt").textContent'),"无活动设备");
    console.log("ok - disconnected status stays stable across telemetry and periodic paints; stale feedback is hidden");

    await evaluate('fixture.edit("connectionKind","serial");fixture.edit("connectionPort","__manual");fixture.edit("connectionCustomPort","")');
    assert.equal(await evaluate('document.getElementById("btnArmConnect").disabled'),true);
    await evaluate('fixture.edit("connectionCustomPort","COM27")');
    assert.equal(await evaluate('document.getElementById("armServerUrl").textContent'),"COM27（待连接）");
    assert.equal(await evaluate('StationUI.snapshot().arm.backend'),"sim");
    await evaluate('document.getElementById("btnRefreshDiagnostics").click()');
    await wait('!document.getElementById("btnRefreshDiagnostics").disabled');
    assert.equal(await evaluate('document.getElementById("connectionPort").value'),"__manual");
    assert.equal(await evaluate('document.getElementById("connectionCustomPort").value'),"COM27");
    await evaluate('document.getElementById("connectionCustomPort").focus()');
    await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Enter",code:"Enter",text:"\r",windowsVirtualKeyCode:13});
    await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Enter",code:"Enter",windowsVirtualKeyCode:13});
    await wait('StationUI.snapshot().connection.status==="connected" && !document.getElementById("btnArmDisconnect").disabled');
    assert.equal(await evaluate('document.getElementById("armServerUrl").textContent'),"COM27");
    assert.equal(await evaluate('document.getElementById("connectionCustomPort").disabled'),true);
    assert.deepEqual(await evaluate('fixture.sent.filter(m=>m.t==="link").map(m=>[m.action,m.kind,m.port])'),[["connect","serial","COM27"]]);
    assert.equal(await evaluate('fixture.sent.some(m=>["power","home","arm","cart_go"].includes(m.t))'),false);
    console.log("ok - manual port survives discovery refresh; Enter connects the exact port once without enabling or motion");

    await evaluate('fixture.snap.arm.enabled=true;fixture.snap.arm.ctrl_mode="Gravity";document.getElementById("btnArmDisconnect").click()');
    await wait('StationUI.snapshot().connection.status==="disconnected" && !document.getElementById("connectionKind").disabled');
    await delay(650);
    assert.deepEqual(await evaluate('["armEnabled","armGripTxt","armCtrlMode"].map(id=>document.getElementById(id).textContent)'),["—","—","—"]);
    await evaluate('fixture.permission=false;document.getElementById("btnRefreshDiagnostics").click()');
    await wait('document.getElementById("connectionTargetHint").textContent.includes("未开放串口访问")');
    await evaluate('fixture.requestsBefore=fixture.sent.filter(m=>m.t==="link").length;document.getElementById("btnArmConnect").click()');
    await wait('document.getElementById("toastMessage").textContent.includes("COM27 尚未连接")');
    assert.equal(await evaluate('fixture.sent.filter(m=>m.t==="link").length===fixture.requestsBefore'),true);
    assert.equal(await evaluate('StationUI.snapshot().connection.status'),"disconnected");
    await evaluate('fixture.edit("connectionPort","__manual");fixture.edit("connectionCustomPort","");document.getElementById("btnRefreshDiagnostics").click()');
    await wait('!document.getElementById("btnRefreshDiagnostics").disabled');
    assert.equal(await evaluate('document.getElementById("connectionPort").value'),"__manual");
    assert.equal(await evaluate('document.getElementById("connectionCustomPort").value'),"");
    console.log("ok - restricted preview explains serial access without sending a connection or changing device type");

    await evaluate('fixture.edit("connectionKind","simulation");document.getElementById("btnArmConnect").click()');
    await wait('StationUI.snapshot().connection.status==="connected" && StationUI.snapshot().connection.kind==="simulation" && !document.getElementById("btnArmDisconnect").disabled');
    assert.equal(await evaluate('document.getElementById("armServerUrl").textContent'),"本机仿真，无需串口");
    await evaluate('fixture.paused=true');
    await wait('document.getElementById("armModeBadge").textContent==="等待设备状态" && document.getElementById("mockTxt").textContent==="设备状态未知"');
    assert.equal(await evaluate('document.getElementById("armEnabled").textContent'),"—");
    assert.equal(await evaluate('document.getElementById("armConnectionKind").textContent'),"本机仿真（状态未更新）");
    assert.equal(await evaluate('document.getElementById("mockTxt").textContent'),"设备状态未知");
    assert.equal(await evaluate('document.getElementById("safeTxt").textContent'),"状态未知");
    assert.equal(await evaluate('document.getElementById("btnCartUseCurrent").disabled'),true);
    await evaluate('fixture.paused=false');
    await wait('document.getElementById("armModeBadge").textContent==="已连接 · 本机仿真"');
    await evaluate('fixture.snap.arm.enabled=true;fixture.snap.connection.enable_required=false;fixture.snap.arm.motors=[{id:1,name:"J1",online:true,ok:true,mode:"position"}];fixture.publish()');
    await wait('document.getElementById("powerStatusText").textContent==="已使能"');
    await evaluate('location.hash="#/arm/control"');
    await wait('StationUI.page()==="arm/control"');
    await evaluate('fixture.snap.arm.q_deg[0]=10;fixture.snap.arm.target_deg[0]=20;fixture.snap.arm.feedback_deg=[7.5,40,40,0,0,0];fixture.publish()');
    await wait('document.querySelector("#armKv tr td:nth-child(2)").textContent==="7.50°  →  20.00°"');
    await evaluate('fixture.snap.arm.feedback_deg=null;fixture.publish()');
    await wait('document.querySelector("#armKv tr td:nth-child(2)").textContent==="10.00°  →  20.00°"');
    await evaluate('fixture.snap.arm.q_deg[0]=0;fixture.snap.arm.target_deg[0]=0;fixture.publish()');
    console.log("ok - measured joint table uses motor feedback and supports legacy snapshots without it");
    await evaluate('document.getElementById("armFollow").checked=true;fixture.snap.arm.motors[0].online=false;fixture.publish()');
    await wait('document.getElementById("motorTable").textContent.includes("离线")');
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),true);
    await evaluate('document.getElementById("armFollow").checked=true;fixture.snap.connection.status="disconnected";fixture.snap.connection.session="remote-disconnect";fixture.snap.arm.online=false;fixture.publish()');
    await delay(300);
    assert.equal(await evaluate('document.getElementById("mockTxt").textContent'),"无活动设备");
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),false);
    assert.equal(await evaluate('document.getElementById("collectBanner").textContent.includes("本机仿真 · 当前操作")'),false);
    assert.equal(await evaluate('document.getElementById("motorTable").textContent.includes("在线")'),false);
    assert.equal(await evaluate('document.getElementById("armKv").textContent.includes("0.00°")'),false);
    assert.equal(await evaluate('document.getElementById("btnGripOpen").classList.contains("is-on")'),false);
    // A lifecycle disconnect remains authoritative even if an older pose says online.
    await evaluate('fixture.snap.arm.online=true;fixture.publish()');await delay(150);
    assert.equal(await evaluate('document.getElementById("mockTxt").textContent'),"无活动设备");
    assert.equal(await evaluate('document.getElementById("powerStatusText").textContent'),"使能未知");
    assert.equal(await evaluate('document.getElementById("btnPowerToggle").disabled'),true);
    await evaluate('location.hash="#/home"');
    await wait('StationUI.page()==="home"');
    assert.equal(await evaluate('document.querySelector("#homeMetrics .v").textContent'),"未连接");
    console.log("ok - remote disconnect during follow clears the target badge, motor feedback and follow intent; stale online flags cannot revive them");

    await evaluate('fixture.snap.connection.status="connected";fixture.snap.connection.session="restore-simulation";fixture.publish()');
    await wait('document.getElementById("mockTxt").textContent==="本机仿真"');
    await evaluate('fixture.paused=true;fixture.poseSeq=StationUI.snapshot().seq;fixture.pose(fixture.poseSeq-1,false)');
    await delay(180);
    assert.equal(await evaluate('StationUI.snapshot().seq===fixture.poseSeq && StationUI.snapshot().arm.online'),true);
    await evaluate('fixture.pose(fixture.poseSeq,false)');
    await wait('document.getElementById("mockTxt").textContent==="无活动设备"');
    await evaluate('fixture.paused=false');
    await wait('document.getElementById("mockTxt").textContent==="本机仿真"');
    console.log("ok - obsolete binary poses are ignored and current link-loss poses update the header immediately");
    await evaluate('document.getElementById("armFollow").checked=true;fixture.commandDown=true;fixture.sockets.filter(s=>s.url.endsWith("/ws/cmd")&&s.readyState===1).forEach(s=>s.close())');
    await delay(250);
    assert.equal(await evaluate('document.getElementById("mockTxt").textContent'),"设备状态未知");
    assert.equal(await evaluate('document.getElementById("linkTxt").textContent'),"未连接");
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),false);
    assert.equal(await evaluate('document.getElementById("btnPowerToggle").disabled'),true);
    assert.equal(await evaluate('document.getElementById("btnHome").disabled'),true);
    await evaluate('fixture.homesBefore=fixture.sent.filter(m=>m.t==="home").length;document.getElementById("btnHome").click()');
    await evaluate('fixture.commandDown=false');
    await wait('StationUI.connected() && document.getElementById("mockTxt").textContent==="本机仿真"');
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),false);
    assert.equal(await evaluate('fixture.sent.filter(m=>m.t==="home").length'),await evaluate('fixture.homesBefore'));
    console.log("ok - control-socket loss invalidates status despite continuing telemetry, and reconnection does not resume follow");
    for(const theme of ["light","dark"]){
      await evaluate('StationTheme.set('+JSON.stringify(theme)+');document.querySelector(".connection-details").open=true');
      for(const width of [1440,900,390]){
        await cdp("Emulation.setDeviceMetricsOverride",{width,height:1050,deviceScaleFactor:1,mobile:false});await delay(80);
        assert.equal(await evaluate('document.documentElement.scrollWidth<=innerWidth+1'),true,theme+" "+width);
      }
    }
    assert.deepEqual(await evaluate('fixture.errors'),[]);
    console.log("ok - simulation uses its own endpoint, stale telemetry hides feedback, both themes fit desktop and mobile");
  }finally{
    if(cdp)await Promise.race([cdp("Browser.close").catch(()=>{}),delay(2000)]);if(socket)socket.close();
    if(process.platform==="win32"&&child.exitCode===null){try{execFileSync("taskkill",["/PID",String(child.pid),"/T","/F"],{windowsHide:true,stdio:"ignore",timeout:5000});}catch{child.kill();}}else child.kill();server.close();
    await new Promise(resolve=>{if(child.exitCode!==null)resolve();else child.once("exit",resolve);});
    assert.ok(path.resolve(profile).startsWith(path.resolve(os.tmpdir())+path.sep)&&path.basename(profile).startsWith("fafu-connections-"));fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:200});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;server.close();});
