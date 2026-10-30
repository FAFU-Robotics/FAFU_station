"use strict";

// Exercise the real DOM and native dialog with a mocked station, never hardware.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const http = require("node:http");
const os = require("node:os");
const path = require("node:path");
const {spawn,execFileSync} = require("node:child_process");
const root = path.resolve(__dirname, "../../robot_station/web/static");
const browser = [process.env.CHROME_BIN,
  "C:/Program Files/Google/Chrome/Application/chrome.exe",
  "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
  "/usr/bin/chromium", "/usr/bin/chromium-browser", "/usr/bin/google-chrome",
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"].find(p => p && fs.existsSync(p));
if (!browser) { console.log("SKIP: Chrome/Chromium not installed (or set CHROME_BIN)"); process.exit(0); }

const fixture = `
  const copy = value => JSON.parse(JSON.stringify(value));
  window.testStation = {
    points:[{name:"取料点",q:[0,40,40,0,0,0],t:2},{name:"放料点",q:[15,40,40,0,0,0],t:5},{name:"路点_3",q:[7.56,40,40,0,0,0],t:2}],
    sent:[], saves:[], waiting:[], holdSaves:false, holdCatalog:false, catalogs:[], errors:[], seq:0, failPath:false, holdPaths:false, waitingPaths:[],
    reply(msg, ok=true) {
      let data = {};
      if (ok && msg.action === "points_save") {this.points=copy(msg.points);data={waypoints:copy(msg.points)};}
      if (msg.action === "point_capture") data={q:[1,42,43,4,5,6]};
      window.dispatchEvent(new CustomEvent("station:ack",{detail:{request_id:msg.request_id,ok,data,error:"模拟保存失败"}}));
    },
    ack(ok=true) {this.reply(this.waiting.shift(),ok);}
  };
  window.fetch = async url => {
    const data = url === "/api/workspace" ? {waypoints:copy(testStation.points),programs:["取放"]} : {files:[]};
    if (url === "/api/workspace" && testStation.holdCatalog) await new Promise(resolve => testStation.catalogs.push(resolve));
    return {ok:true,json:async()=>data};
  };
  window.StationUI = {
    page:()=>"work/waypoints",snapshot:()=>null,releaseManual:()=>{},readTargets:()=>[10,20,30,40,50,60],notice:()=>{},
    sendImmediate(msg) {
      testStation.sent.push(copy(msg));
      if (msg.t === "path") {
        if(testStation.holdPaths) testStation.waitingPaths.push(msg);
        else queueMicrotask(()=>testStation.reply(msg,!testStation.failPath));
        return;
      }
      if (msg.t !== "workbench") return;
      if (msg.action === "points_save") {
        testStation.saves.push(copy(msg.points));
        if(testStation.holdSaves) {testStation.waiting.push(msg);return;}
      }
      queueMicrotask(()=>testStation.reply(msg));
    }
  };
  window.addEventListener("error",ev=>testStation.errors.push(ev.message));
  window.addEventListener("unhandledrejection",ev=>testStation.errors.push(String(ev.reason)));
  document.querySelectorAll(".page").forEach(page=>page.classList.toggle("on",page.id==="page-waypoints"));
  document.getElementById("workNavSelect").value="work/waypoints";
  window.edit = (selector,value) => {const input=document.querySelector(selector);input.value=value;input.dispatchEvent(new Event("change"));};
  window.choose = index => document.querySelector('[data-point-select="'+index+'"]').onclick();
  window.feedback = (q,moving=false,extra={}) => window.dispatchEvent(new CustomEvent("station:snapshot",{detail:{seq:++testStation.seq,program:{status:"idle"},arm:{online:true,q_deg:q,feedback_deg:q,moving},...extra}}));
  window.click = selector => document.querySelector(selector).click();
`;
const html = fs.readFileSync(path.join(root,"index.html"),"utf8")
  .replace(/<script\b[^>]*>[\s\S]*?<\/script>/g, "")
  .replace("</body>", '<script src="/fixture.js"></script><script src="/static/js/numbers.js"></script><script src="/static/js/dialog.js"></script><script src="/static/js/workbench.js"></script></body>');
const server = http.createServer((req,res) => {
  if(req.url === "/") {res.setHeader("Content-Type","text/html; charset=utf-8");res.end(html);return;}
  if(req.url === "/fixture.js") {res.setHeader("Content-Type","text/javascript; charset=utf-8");res.end(fixture);return;}
  const relative = decodeURIComponent(req.url.split("?")[0]).replace(/^\/static\//,"");
  const file = path.resolve(root,relative);
  if(!file.startsWith(root+path.sep)||!fs.existsSync(file)||!fs.statSync(file).isFile()) {res.writeHead(404);res.end();return;}
  res.setHeader("Content-Type",file.endsWith(".css")?"text/css":"text/javascript");res.end(fs.readFileSync(file));
});
const delay = ms => new Promise(resolve=>setTimeout(resolve,ms));
async function until(fn) {
  const end=Date.now()+7000;
  while(Date.now()<end) {const result=await fn();if(result)return result;await delay(20);}
  throw new Error("Timed out waiting for browser state");
}

async function main() {
  await new Promise(resolve=>server.listen(0,"127.0.0.1",resolve));
  const profile = fs.mkdtempSync(path.join(os.tmpdir(),"fafu-waypoints-"));
  const child = spawn(browser,["--headless=new","--disable-gpu","--no-first-run","--no-default-browser-check",
    "--disable-background-networking","--disable-extensions","--remote-debugging-port=0","--user-data-dir="+profile,"about:blank"],
    {stdio:["ignore","ignore","pipe"],windowsHide:true});
  let browserLog = "";
  child.stderr.on("data", data => {browserLog += data.toString();});
  let socket, closeBrowser;
  try {
    const portFile=path.join(profile,"DevToolsActivePort");
    try {await until(()=>fs.existsSync(portFile));}
    catch(error) {throw new Error(error.message+"; browser exit="+child.exitCode+"; "+browserLog);}
    const port=await until(()=>{
      try {
        const value=fs.readFileSync(portFile,"utf8").split("\n")[0].trim();
        return /^\d+$/.test(value)&&Number(value)>0 ? value : false;
      } catch(error) {
        if(["EBUSY","EACCES","EPERM","ENOENT"].includes(error.code))return false;
        throw error;
      }
    });
    const pages=await (await fetch("http://127.0.0.1:"+port+"/json/list")).json();
    socket=new WebSocket(pages.find(p=>p.type==="page").webSocketDebuggerUrl);
    await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
    let serial=0;
    const pending=new Map();
    socket.onmessage=event=>{
      const msg=JSON.parse(event.data),task=pending.get(msg.id);
      if(task){pending.delete(msg.id);if(msg.error)task.reject(new Error(msg.error.message));else task.resolve(msg.result);}
    };
    const cdp=(method,params={})=>new Promise((resolve,reject)=>{const id=++serial;pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});
    closeBrowser=()=>cdp("Browser.close");
    const evaluate=async expression=>{
      const result=await cdp("Runtime.evaluate",{expression,returnByValue:true,awaitPromise:true});
      if(result.exceptionDetails)throw new Error(result.exceptionDetails.exception?.description||result.exceptionDetails.text);
      return result.result.value;
    };
    const wait=async expression=>{
      try{return await until(()=>evaluate(expression));}
      catch(error){throw new Error(error.message+": "+expression+"\n"+await evaluate('JSON.stringify({program:window.StationUI?.snapshot()?.program,arm:window.StationUI?.snapshot()?.arm,motion:document.getElementById("wpMotionState")?.textContent,ack:document.getElementById("armAck")?.textContent})'));}
    };
    const run=async(name,fn)=>{await fn();console.log("ok - "+name);};
    await cdp("Emulation.setDeviceMetricsOverride",{width:1440,height:1080,deviceScaleFactor:1,mobile:false});
    const realStation = process.env.FAFU_REAL_STATION_URL;
    await cdp("Page.navigate",{url:realStation ? realStation+"/#/work/waypoints" : "http://127.0.0.1:"+server.address().port+"/"});
    const screenshot=async name=>{
      if(!process.env.FAFU_UI_SCREENSHOTS)return;
      // Explicit bounds avoid Chromium cropping the right edge when it hides
      // the scrollbar and reflows a full-page capture at the wider viewport.
      const size=await evaluate('({width:innerWidth,height:document.documentElement.scrollHeight})');
      const {data}=await cdp("Page.captureScreenshot",{format:"png",captureBeyondViewport:true,clip:{x:0,y:0,...size,scale:1}});
      fs.writeFileSync(path.join(process.env.FAFU_UI_SCREENSHOTS,name+".png"),Buffer.from(data,"base64"));
    };
    if (realStation) {
      await wait('window.StationUI?.connected() && window.StationUI.snapshot()?.arm?.online && window.StationUI.snapshot()?.arm?.enabled');
      if(process.env.FAFU_UI_AUDIT){
        await require("./ui_audit_scenarios.js")({evaluate,wait,cdp,screenshot,delay});return;
      }
      if(process.env.FAFU_CARTESIAN_LAYOUT_ONLY){
        await require("./cartesian_scenarios.js").captureLayout({evaluate,wait,run,cdp,screenshot});
        console.log("Cartesian layout review passed");return;
      }
      if(process.env.FAFU_MANUAL_ONLY){
        await require("./manual_scenarios.js")({evaluate,wait,run,delay,cdp,screenshot});
        console.log("manual browser tests passed");return;
      }
      await evaluate('document.getElementById("btnWpAdd").click()');
      await wait('document.querySelectorAll("[data-point-select]").length===1 && document.getElementById("wpSaveState").dataset.state==="saved"');
      const start = await evaluate('StationUI.snapshot().arm.q_deg[0]');
      await evaluate('const input=document.querySelector("[data-point-q=\\"0\\"]");input.value='+JSON.stringify(start+6)+';input.dispatchEvent(new Event("change"))');
      await wait('document.getElementById("wpSaveState").dataset.state==="saved"');
      await evaluate('document.getElementById("btnWpRun").click()');
      await wait('document.getElementById("wpMotionState").dataset.state==="arrived"');
      assert.ok(Math.abs(await evaluate('StationUI.snapshot().arm.q_deg[0]')-start-6)<0.6);
      await screenshot("waypoints-simulation");
      await evaluate('document.getElementById("btnWpStop").click()');
      await wait('document.getElementById("wpMotionState").textContent.includes("已停止")');
      await evaluate('location.hash="#/arm/control"');
      await wait('document.getElementById("page-arm").classList.contains("on")');
      const beforeInvalid=await evaluate('StationUI.snapshot().arm.q_deg[0]');
      await evaluate('const manualInput=document.querySelector("[data-jn=\\"0\\"]");window.manualValue=manualInput.value;manualInput.focus();manualInput.value="";manualInput.dispatchEvent(new Event("input",{bubbles:true}));document.getElementById("btnSendArm").click()');
      await wait('document.querySelector("[data-jn=\\"0\\"]").getAttribute("aria-invalid")==="true"');
      await delay(200);
      assert.ok(Math.abs(await evaluate('StationUI.snapshot().arm.q_deg[0]')-beforeInvalid)<0.6);
      await evaluate('location.hash="#/work/waypoints"');
      await wait('document.getElementById("page-waypoints").classList.contains("on")');
      await evaluate('document.getElementById("btnWpManual").click()');
      await wait('document.getElementById("btnWpDraftCancel")');
      assert.equal(await evaluate('Array.from(document.querySelectorAll("[data-point-q]")).every(x=>x.value==="")'),true);
      const manualAngles=[beforeInvalid+3,40,40,0,0,0];
      for(let j=0;j<6;j++) {
        await cdp("Input.insertText",{text:String(manualAngles[j])});
        await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
        await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
        const library=await evaluate('(async()=>await(await fetch("/api/workspace")).json())()');
        if(j<5)assert.equal(library.waypoints.length,1,"axis "+(j+1)+" must not persist an incomplete point");
      }
      await wait('document.querySelectorAll("[data-point-select]").length===2 && document.getElementById("wpSaveState").dataset.state==="saved"');
      const persisted=await evaluate('(async()=>await(await fetch("/api/workspace")).json())()');
      assert.deepEqual(persisted.waypoints[1].q,manualAngles);
      assert.ok(Math.abs(await evaluate('StationUI.snapshot().arm.q_deg[0]')-beforeInvalid)<0.6);
      await evaluate('window.waypointBeforeReload=true');
      await cdp("Page.reload");
      await wait('!window.waypointBeforeReload && window.StationUI?.connected() && document.querySelectorAll("[data-point-select]").length===2 && !document.getElementById("btnWpManual").disabled');
      await evaluate('document.getElementById("btnWpManual").click()');
      await wait('document.getElementById("btnWpDraftCancel")');
      assert.equal(await evaluate('Array.from(document.querySelectorAll("[data-point-q]")).every(x=>x.value==="")'),true);
      await evaluate('document.getElementById("btnWpDraftCancel").click()');
      assert.equal(await evaluate('document.querySelectorAll("[data-point-select]").length'),2);
      console.log("ok - full station UI records, saves, moves and confirms arrival against the simulated robot");
    } else {
      await wait('document.querySelectorAll("[data-point-select]").length===3');
      await require("./waypoints_scenarios.js")({evaluate,wait,cdp,screenshot,run,delay});
      assert.deepEqual(await evaluate('testStation.errors'),[]);
    }
    console.log("waypoint browser tests passed");
  } finally {
    if(closeBrowser)await Promise.race([closeBrowser().catch(()=>{}),delay(2000)]);
    if(socket)socket.close();
    if(process.platform==="win32"&&child.exitCode===null){
      try{execFileSync("taskkill",["/PID",String(child.pid),"/T","/F"],{windowsHide:true,stdio:"ignore",timeout:5000});}
      catch{child.kill();}
    }else child.kill();
    server.close();
    await new Promise(resolve=>{if(child.exitCode!==null)resolve();else child.once("exit",resolve);});
    // Only remove the temporary browser profile created by this test.
    assert.ok(path.resolve(profile).startsWith(path.resolve(os.tmpdir())+path.sep));
    assert.ok(path.basename(profile).startsWith("fafu-waypoints-"));
    fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:200});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;server.close();});
