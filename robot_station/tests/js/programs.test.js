"use strict";
// Real DOM and WebGL; all commands terminate in the fixture, never hardware.
const assert=require("node:assert/strict"),fs=require("node:fs"),http=require("node:http"),os=require("node:os"),path=require("node:path");
const {spawn,execFileSync}=require("node:child_process");
const root=path.resolve(__dirname,"../../robot_station/web/static");
const browser=[process.env.CHROME_BIN,"C:/Program Files/Google/Chrome/Application/chrome.exe","C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe","/usr/bin/chromium","/usr/bin/google-chrome"].find(p=>p&&fs.existsSync(p));
if(!browser){console.log("SKIP: Chrome/Chromium not installed");process.exit(0);}
const fixture=`
const copy=x=>JSON.parse(JSON.stringify(x));
window.testStation={programs:{"取放示例":{name:"取放示例",nodes:[{id:"loop1",type:"loop",count:2,children:[{id:"move1",type:"movej",point:"取料点",speed:20}]},{id:"wait1",type:"wait",seconds:1}]},"返回":{name:"返回",nodes:[{id:"return1",type:"movej",point:"放置点",speed:20}]}},sent:[],errors:[],notices:[],fail:false,snap:{seq:1,program:{status:"idle"},arm:{q_deg:[90,40,40,0,0,0],feedback_deg:[90,40,40,0,0,0],gripper_deg:50,gripper_limits:[0,105]}}};
const points=[{name:"取料点",q:[15,40,40,0,0,0]},{name:"放置点",q:[-20,30,50,0,0,0]}];
const nativeFetch=fetch;window.fetch=(url,...args)=>url==="/api/workspace"?Promise.resolve({ok:true,json:async()=>({waypoints:copy(points),programs:Object.keys(testStation.programs)})}):url==="/api/traj"?Promise.resolve({ok:true,json:async()=>({files:[{name:"示教.jsonl",frames:101,s:1}]})}):nativeFetch(url,...args);
window.emit=(q,status="idle")=>{testStation.snap={seq:testStation.snap.seq+1,program:{status},arm:{...testStation.snap.arm,q_deg:q,feedback_deg:q}};dispatchEvent(new CustomEvent("station:snapshot",{detail:copy(testStation.snap)}));};
window.emitProgram=program=>{testStation.snap={...testStation.snap,seq:testStation.snap.seq+1,program};dispatchEvent(new CustomEvent("station:snapshot",{detail:copy(testStation.snap)}));};
window.StationUI={page:()=>location.hash.slice(2),snapshot:()=>copy(testStation.snap),notice:text=>testStation.notices.push(text),releaseManual:()=>{},sendImmediate(msg){
  testStation.sent.push(copy(msg));let data={},error="";
  if(testStation.fail){testStation.fail=false;error="模拟保存失败";}
  else if(msg.action==="load")data=copy(testStation.programs[msg.name]);
  else if(msg.action==="save"){data={name:msg.name,nodes:copy(msg.nodes),script:"# fixture"};testStation.programs[msg.name]=copy(data);}
  else if(msg.action==="delete")delete testStation.programs[msg.name];
  else if(msg.action==="parse")data={nodes:copy(msg.nodes||testStation.programs["结构测试"].nodes),script:"# fixture"};
  else if(msg.action==="traj_process")data={frames:Array.from({length:101},(_,i)=>({t:i/100,pos:[i/100,0.7,0.7,0,0,0],gpos:0.5}))};
  const deliver=()=>dispatchEvent(new CustomEvent("station:ack",{detail:{request_id:msg.request_id,ok:!error,error,data}}));
  if(msg.action==="traj_process"&&testStation.holdTrajectory)testStation.replyTrajectory=deliver;else queueMicrotask(deliver);
}};
window.showPage=()=>{document.querySelectorAll(".page").forEach(el=>el.classList.toggle("on",el.id==="page-"+(location.hash.includes("program")?"program":"arm")));window.Arm3D?.setActive(location.hash.includes("control"));dispatchEvent(new CustomEvent("station:page",{detail:StationUI.page()}));};
location.hash="#/work/program";showPage();addEventListener("hashchange",showPage);
window.click=s=>document.querySelector(s).click();window.edit=(s,value)=>{const el=document.querySelector(s);el.value=value;el.dispatchEvent(new Event(el.tagName==="SELECT"?"change":"input",{bubbles:true}));};
window.add=(type,selector="#btnProgAdd")=>{click(selector);click('[data-add-type="'+type+'"]');};
window.dragStep=(ident,targetSelector)=>{const source=Array.from(document.querySelectorAll("[data-node]")).find(button=>button.dataset.node===ident),target=document.querySelector(targetSelector),dataTransfer=new DataTransfer();source.dispatchEvent(new DragEvent("dragstart",{bubbles:true,dataTransfer}));target.dispatchEvent(new DragEvent("dragover",{bubbles:true,dataTransfer,cancelable:true}));target.dispatchEvent(new DragEvent("drop",{bubbles:true,dataTransfer,cancelable:true}));};
addEventListener("error",ev=>testStation.errors.push(ev.message));addEventListener("unhandledrejection",ev=>testStation.errors.push(String(ev.reason)));
`;
const metrics=`const original=THREE.WebGLRenderer;window.stats={contexts:0,renders:0};THREE.WebGLRenderer=function(options){const r=new original(options);stats.contexts++;const draw=r.render.bind(r);r.render=(scene,camera)=>{stats.renders++;stats.scene=scene;return draw(scene,camera);};return r;};window.firstJoint=()=>stats.scene?.children.find(x=>x.isGroup)?.children.find(x=>x.isGroup)?.quaternion.z;`;
const html=fs.readFileSync(path.join(root,"index.html"),"utf8").replace(/<script\b[^>]*>[\s\S]*?<\/script>/g,"").replace("</body>",'<script src="/fixture.js"></script><script src="/static/vendor/three.min.js"></script><script src="/metrics.js"></script><script src="/static/js/arm3d.js"></script><script src="/static/js/numbers.js"></script><script src="/static/js/dialog.js"></script><script src="/static/js/workbench.js"></script></body>');
const server=http.createServer((req,res)=>{
  if(["/","/fixture.js","/metrics.js"].includes(req.url)){res.setHeader("Content-Type",req.url==="/"?"text/html; charset=utf-8":"text/javascript; charset=utf-8");res.end(req.url==="/"?html:req.url==="/fixture.js"?fixture:metrics);return;}
  const file=path.resolve(root,decodeURIComponent(req.url.split("?")[0]).replace(/^\/static\//,""));
  if(!file.startsWith(root+path.sep)||!fs.existsSync(file)||!fs.statSync(file).isFile()){res.writeHead(404);res.end();return;}
  res.setHeader("Content-Type",file.endsWith(".css")?"text/css":file.endsWith(".bin")?"application/octet-stream":"text/javascript");res.end(fs.readFileSync(file));
});
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function until(fn){const end=Date.now()+15000;while(Date.now()<end){const value=await fn();if(value)return value;await delay(25);}throw new Error("Timed out waiting for program UI");}
async function main(){
  await new Promise(resolve=>server.listen(0,"127.0.0.1",resolve));
  const profile=fs.mkdtempSync(path.join(os.tmpdir(),"fafu-programs-"));
  const child=spawn(browser,["--headless=new","--no-first-run","--no-default-browser-check","--disable-background-networking","--disable-extensions","--use-angle=swiftshader","--enable-unsafe-swiftshader","--remote-debugging-port=0","--user-data-dir="+profile,"about:blank"],{stdio:"ignore",windowsHide:true});
  let socket,closeBrowser;
  try{
    const portFile=path.join(profile,"DevToolsActivePort");
    const port=await until(()=>{try{const value=fs.readFileSync(portFile,"utf8").split("\n")[0].trim();return /^\d+$/.test(value)&&Number(value)>0?value:false;}catch(e){if(["ENOENT","EACCES","EBUSY","EPERM"].includes(e.code))return false;throw e;}});
    const pages=await(await fetch("http://127.0.0.1:"+port+"/json/list")).json();socket=new WebSocket(pages.find(p=>p.type==="page").webSocketDebuggerUrl);await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
    let serial=0;const pending=new Map();socket.onmessage=ev=>{const msg=JSON.parse(ev.data),task=pending.get(msg.id);if(task){pending.delete(msg.id);msg.error?task.reject(new Error(msg.error.message)):task.resolve(msg.result);}};
    const cdp=(method,params={})=>new Promise((resolve,reject)=>{const id=++serial;pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});closeBrowser=()=>cdp("Browser.close");
    const evaluate=async expression=>{const r=await cdp("Runtime.evaluate",{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw new Error(r.exceptionDetails.exception?.description||r.exceptionDetails.text);return r.result.value;};
    const wait=expr=>until(()=>evaluate(expr)),run=async(name,fn)=>{await fn();console.log("ok - "+name);};
    const screenshot=async name=>{if(!process.env.FAFU_UI_SCREENSHOTS)return;const {data}=await cdp("Page.captureScreenshot",{format:"png",captureBeyondViewport:true});fs.writeFileSync(path.join(process.env.FAFU_UI_SCREENSHOTS,name+".png"),Buffer.from(data,"base64"));};
    await cdp("Emulation.setDeviceMetricsOverride",{width:1440,height:1100,deviceScaleFactor:1,mobile:false});
    if(process.env.FAFU_REAL_STATION_URL){
      await cdp("Page.enable");await cdp("Page.addScriptToEvaluateOnNewDocument",{source:'window.programErrors=[];addEventListener("error",e=>programErrors.push(e.message));addEventListener("unhandledrejection",e=>programErrors.push(String(e.reason)));window.click=s=>document.querySelector(s).click();window.edit=(s,value)=>{const el=document.querySelector(s);el.value=value;el.dispatchEvent(new Event(el.tagName==="SELECT"?"change":"input",{bubbles:true}));};window.add=(type,selector="#btnProgAdd")=>{click(selector);click("[data-add-type="+type+"]");};'});
      await cdp("Page.navigate",{url:process.env.FAFU_REAL_STATION_URL+"/#/work/program"});
      await wait('window.StationUI?.connected() && document.querySelectorAll("[data-program]").length===2');
      await evaluate('click("[data-program=取放示例]")');
      await wait('document.getElementById("progName").value==="取放示例" && document.getElementById("progSelectedPose").textContent.includes("J1 15.0")');
      await wait('!!document.querySelector("#prog3dHost canvas") && document.getElementById("prog3dOverlay").textContent.includes("CAD")');
      await evaluate('window.sharedProgramCanvas=document.querySelector("#prog3dHost canvas");click("#btnProgNew")');
      await wait('!document.querySelector("[data-node]") && document.getElementById("progDirty").textContent.includes("新程序")');
      await evaluate('edit("#progName","集成程序");add("loop");add("movej", "[aria-label=添加重复内步骤]");add("gripper","[aria-label=在重复结束后添加]");click("#btnProgSave")');
      await wait('!!document.querySelector("[data-program=集成程序]") && document.getElementById("progDirty").textContent==="已保存" && !document.getElementById("btnProgNew").disabled');
      assert.equal(await evaluate('document.getElementById("progPreview").hidden'),true);
      await evaluate('click("#progTree .tree-block [data-node]")');
      await evaluate('location.hash="#/arm/control"');await wait('!!document.querySelector("#arm3dHost canvas")');
      assert.equal(await evaluate('document.querySelector("#arm3dHost canvas")===sharedProgramCanvas'),true);
      await evaluate('location.hash="#/work/program"');await wait('!!document.querySelector("#prog3dHost canvas")');
      assert.equal(await evaluate('document.querySelector("#prog3dHost canvas")===sharedProgramCanvas'),true);
      await evaluate('window.programReloadPending=true');
      await cdp("Page.reload");await wait('!window.programReloadPending && window.StationUI?.connected() && !!document.querySelector("[data-program=集成程序]")');
      await evaluate('document.querySelector("[data-program=集成程序]").click()');await wait('document.querySelectorAll("#progTree>ol>li[role=treeitem]").length===2');
      assert.equal(await evaluate('document.querySelectorAll("#progTree .tree-block li[role=treeitem]").length'),1);
      await screenshot("programs-station");
      await evaluate('click("#btnProgNew")');await wait('!document.querySelector("[data-node]")');
      await evaluate('edit("#progName","运行状态验证");add("wait");const input=document.querySelector("[data-parameter=seconds]");input.value=3;input.dispatchEvent(new Event("change",{bubbles:true}));click("#btnProgSave")');
      await wait('document.getElementById("progDirty").textContent==="已保存" && !document.getElementById("btnProgRun").disabled');await evaluate('click("#btnProgRun")');
      await wait('StationUI.snapshot().program.status==="running" && document.getElementById("progExecution").dataset.status==="running" && document.getElementById("progCurrentStep").textContent.includes("等待 3")');
      assert.equal(await evaluate('document.getElementById("globalTask").hidden'),true);
      assert.match(await evaluate('document.getElementById("progCurrentStep").textContent'),/等待 3/);
      await screenshot("programs-running-station");await evaluate('click("#btnProgPause")');
      await wait('StationUI.snapshot().program.status==="paused" && document.getElementById("progExecution").dataset.status==="paused"');
      await evaluate('location.hash="#/arm/control"');await wait('!document.getElementById("globalTask").hidden');
      assert.equal(await evaluate('document.getElementById("globalTask").dataset.status'),"paused");
      await evaluate('document.getElementById("btnGlobalProgramStop").click()');await wait('StationUI.snapshot().program.status==="stopped" && document.getElementById("globalTask").hidden');
      await evaluate('location.hash="#/work/program"');await wait('document.getElementById("progExecution").dataset.status==="stopped"');
      await evaluate('click("#btnProgRun")');await wait('StationUI.snapshot().program.status==="completed" && document.getElementById("progExecution").dataset.status==="completed"');
      assert.equal(await evaluate('StationUI.snapshot().program.completed'),1);
      assert.equal(await evaluate('document.getElementById("btnProgStop").hidden && document.getElementById("btnProgPause").hidden'),true);
      assert.equal(await evaluate('document.getElementById("progAck").textContent'),"");
      await screenshot("programs-completed-station");
      assert.deepEqual(await evaluate('programErrors'),[]);console.log("ok - full station program editing, run/pause, cross-page stop, restart and completion");return;
    }
    await cdp("Page.navigate",{url:"http://127.0.0.1:"+server.address().port+"/"});
    await wait('document.querySelectorAll("[data-program]").length===2');
    await run("sidebar owns file controls and only motion steps allocate and show 3D",async()=>{
      assert.equal(await evaluate('document.getElementById("progName").closest(".program-library")!==null'),true);
      assert.equal(await evaluate('!!document.getElementById("progInsert")||!!document.getElementById("progAddType")||!!document.querySelector(".program-toolbar")'),false);
      assert.equal(await evaluate('stats.contexts'),0);
      assert.equal(await evaluate('document.getElementById("btnProgCopy").disabled && document.getElementById("btnProgUndo").disabled'),true);
      await evaluate('click("[data-program=取放示例]")');await wait('document.getElementById("progName").value==="取放示例"');
      assert.equal(await evaluate('document.getElementById("progName").readOnly && document.getElementById("progPreview").hidden'),true);
      await evaluate('click("[data-node=move1]")');await wait('Math.abs(firstJoint()-Math.sin(15*Math.PI/360))<1e-6');
      await evaluate('edit("#progParams select","放置点")');await wait('Math.abs(firstJoint()-Math.sin(-20*Math.PI/360))<1e-6');
      await evaluate('edit("#progPreviewMode","live")');await wait('Math.abs(firstJoint()-Math.sin(90*Math.PI/360))<1e-6');
      await evaluate('emit([30,40,40,0,0,0])');await wait('Math.abs(firstJoint()-Math.sin(30*Math.PI/360))<1e-6');
      await evaluate('edit("#progPreviewMode","target")');await wait('Math.abs(firstJoint()-Math.sin(-20*Math.PI/360))<1e-6');
      assert.equal(await evaluate('document.getElementById("prog3dHost").clientHeight<=240'),true);assert.equal(await evaluate('stats.contexts'),1);
      assert.equal(await evaluate('document.querySelector(".program-execution").getBoundingClientRect().bottom<=document.querySelector(".program-tree-card").getBoundingClientRect().top'),true);
      assert.equal(await evaluate('testStation.sent.some(x=>["path","arm","grip"].includes(x.t)||["run","step"].includes(x.action))'),false);
      await screenshot("programs-target-preview");
      for(const selector of ["[data-node=wait1]","[data-node=loop1]"]){await evaluate('click('+JSON.stringify(selector)+')');assert.equal(await evaluate('document.getElementById("progPreview").hidden'),true);}
      const beforeCancel=await evaluate('document.querySelectorAll(".tree-node").length');
      await evaluate('click("#btnProgAdd")');await screenshot("programs-add-step");await evaluate('click("#btnProgAddCancel")');
      assert.equal(await evaluate('document.querySelectorAll(".tree-node").length'),beforeCancel);
      await evaluate('add("gripper")');assert.equal(await evaluate('document.getElementById("progPreview").hidden'),true);
      await delay(150);const idle=await evaluate('stats.renders');await evaluate('emit([60,40,40,0,0,0])');await delay(150);assert.equal(await evaluate('stats.renders'),idle);
    });
    await run("explicit save-as protects originals and inline insertion handles loop, if and else",async()=>{
      await evaluate('click("#btnProgCopyFile");edit("#progCopyName","返回");document.getElementById("progCopyForm").requestSubmit()');await wait('document.getElementById("progCopyError").textContent.includes("同名")');
      await evaluate('edit("#progCopyName","目标预览示例");document.getElementById("progCopyForm").requestSubmit()');await wait('!!document.querySelector("[data-program=目标预览示例]") && !document.getElementById("progCopyDialog").open');
      assert.equal(await evaluate('testStation.programs["取放示例"].nodes[0].children[0].point'),"取料点");
      await evaluate('click("#btnProgNew")');await wait('!document.querySelector("[data-node]") && document.getElementById("progDirty").textContent.includes("新程序")');
      await evaluate('edit("#progName","结构测试");add("loop");add("movej","[aria-label=添加重复内步骤]");add("wait",".tree-block>.tree-branch>ol>.tree-insert-row:last-child button");add("gripper","[aria-label=在重复结束后添加]");click("#btnProgSave")');
      await wait('!!testStation.programs["结构测试"]');
      assert.deepEqual(await evaluate('testStation.programs["结构测试"].nodes.map(x=>x.type)'),["loop","gripper"]);
      assert.deepEqual(await evaluate('testStation.programs["结构测试"].nodes[0].children.map(x=>x.type)'),["movej","wait"]);
      await evaluate('add("if",".tree-block>.tree-branch>ol>.tree-insert-row:last-child button");add("wait","[aria-label=添加条件成立时的步骤]");add("gripper","[aria-label=添加否则分支的步骤]");add("comment","[aria-label=在判断结束后添加]");add("wait","#progTree>ol>.tree-insert-row [aria-label=在重复结束后添加]");add("comment");click("#btnProgSave")');
      await wait('testStation.programs["结构测试"].nodes.length===4');
      assert.deepEqual(await evaluate('testStation.programs["结构测试"].nodes.map(x=>x.type)'),["loop","wait","gripper","comment"]);
      assert.deepEqual(await evaluate('testStation.programs["结构测试"].nodes[0].children.map(x=>x.type)'),["movej","wait","if","comment"]);
      assert.deepEqual(await evaluate('testStation.programs["结构测试"].nodes[0].children[2].children.map(x=>x.type)'),["wait"]);
      assert.deepEqual(await evaluate('testStation.programs["结构测试"].nodes[0].children[2].otherwise.map(x=>x.type)'),["gripper"]);
      assert.equal(await evaluate('document.getElementById("btnProgUndo").disabled'),false,"保存不应清空撤销历史");
      await evaluate('click("#btnProgUndo");click("#btnProgSave")');await wait('testStation.programs["结构测试"].nodes.length===3');
      await screenshot("programs-structured-editor");
    });
    await run("save errors preserve drafts, switching protects edits, and running locks edits",async()=>{
      await evaluate('add("wait");testStation.fail=true;click("#btnProgSave")');await wait('document.getElementById("progFileNotice").textContent.includes("模拟保存失败")');
      assert.match(await evaluate('document.getElementById("progDirty").textContent'),/未保存/);
      await evaluate('click("[data-program=返回]")');await wait('document.getElementById("confirmDialog").open');await evaluate('click("#confirmCancel")');
      assert.equal(await evaluate('document.getElementById("progName").value'),"结构测试");
      await evaluate('emit([0,40,40,0,0,0],"running")');assert.equal(await evaluate('document.getElementById("btnProgAdd").disabled && document.querySelector("[data-program]").disabled'),true);
      await evaluate('emit([0,40,40,0,0,0]);click("#btnProgSave")');await wait('document.getElementById("progDirty").textContent==="已保存"');
      await evaluate('click("#btnProgScript")');await wait('!document.getElementById("programScriptView").hidden');assert.equal(await evaluate('document.querySelector(".program-library").getBoundingClientRect().width>0'),true);
      await evaluate('click("#btnProgTree")');await wait('!document.getElementById("programWorkspace").hidden');
    });
    await run("dragging moves existing steps across branches without allowing cyclic trees",async()=>{
      await evaluate('window.rootGripperId=testStation.programs["结构测试"].nodes.find(node=>node.type==="gripper").id;dragStep(rootGripperId,".else-branch>ol>.tree-insert-row:last-child button");click("#btnProgSave")');
      await wait('!testStation.programs["结构测试"].nodes.some(node=>node.type==="gripper")');
      assert.equal(await evaluate('testStation.programs["结构测试"].nodes[0].children[2].otherwise.length'),2);
      await evaluate('dragStep(rootGripperId,"#progTree>ol>.tree-insert-row:last-child button");click("#btnProgSave")');
      await wait('testStation.programs["结构测试"].nodes.at(-1).id===rootGripperId');
      assert.equal(await evaluate('testStation.programs["结构测试"].nodes[0].children[2].otherwise.length'),1);
      const count=await evaluate('document.querySelectorAll(".tree-node").length');
      await evaluate('dragStep(testStation.programs["结构测试"].nodes[0].id,".else-branch>ol>.tree-insert-row:last-child button")');
      assert.equal(await evaluate('document.querySelectorAll(".tree-node").length'),count);
      assert.equal(await evaluate('document.getElementById("btnProgSave").disabled'),true);
      assert.match(await evaluate('document.getElementById("progAck").textContent'),/不能/);
    });
    await run("trajectory step shows its path and a collapsed preview stops rendering",async()=>{
      await evaluate('testStation.holdTrajectory=true;add("replay")');await wait('!!testStation.replyTrajectory');
      await evaluate('window.replayButtonId=document.querySelector(".tree-node.selected").dataset.node;document.querySelectorAll("#progTree>ol>li[role=treeitem]>.tree-node")[2].click()');
      assert.equal(await evaluate('document.getElementById("progPreview").hidden'),true);
      await delay(100);const late=await evaluate('stats.renders');await evaluate('testStation.holdTrajectory=false;testStation.replyTrajectory()');await delay(100);assert.equal(await evaluate('stats.renders'),late,"过期的轨迹响应不得重新打开预览");
      await evaluate('Array.from(document.querySelectorAll("[data-node]")).find(button=>button.dataset.node===replayButtonId).click()');await wait('document.getElementById("progPreviewHint").textContent.includes("末端路径")');
      await evaluate('document.getElementById("progPreview").open=false');await delay(150);const count=await evaluate('stats.renders');await evaluate('emit([50,40,40,0,0,0])');await delay(150);assert.equal(await evaluate('stats.renders'),count);
      await evaluate('document.getElementById("progPreview").open=true');await wait('stats.renders>'+count);await screenshot("programs-library-and-tree");
      await cdp("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});await delay(150);assert.equal(await evaluate('document.documentElement.scrollWidth<=innerWidth'),true);await screenshot("programs-mobile");
    });
    await run("execution panel distinguishes states and hides duplicate banners and editing affordances",async()=>{
      await cdp("Emulation.setDeviceMetricsOverride",{width:1440,height:1100,deviceScaleFactor:1,mobile:false});
      await evaluate('window.currentStep=testStation.programs["结构测试"].nodes[0].children[0];Array.from(document.querySelectorAll("[data-node]")).find(button=>button.dataset.node===currentStep.id).click();emitProgram({status:"running",name:"结构测试",completed:4,current_id:currentStep.id,current_type:"movej"});window.scrollTo(0,0)');
      assert.equal(await evaluate('document.getElementById("globalTask").hidden'),true);
      assert.match(await evaluate('document.getElementById("progCurrentStep").textContent'),/正在执行：MoveJ/);
      assert.equal(await evaluate('document.getElementById("progProgress").textContent'),"已完成 4 步");
      assert.equal(await evaluate('document.querySelector(".tree-tools").hidden && document.getElementById("btnProgRun").hidden && !document.getElementById("btnProgPause").hidden && !document.getElementById("btnProgStop").hidden && document.getElementById("progSpeedScale").hidden'),true);
      assert.equal(await evaluate('getComputedStyle(document.querySelector(".tree-insert-row")).display'),"none");
      assert.equal(await evaluate('document.querySelector(".tree-node.executing .tree-step-status").textContent'),"执行中");
      await screenshot("programs-running");
      await cdp("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});await delay(100);
      assert.equal(await evaluate('document.documentElement.scrollWidth<=innerWidth'),true);await screenshot("programs-running-mobile");
      await cdp("Emulation.setDeviceMetricsOverride",{width:1440,height:1100,deviceScaleFactor:1,mobile:false});
      await evaluate('emitProgram({status:"paused",name:"结构测试",completed:4,current_id:currentStep.id,current_type:"movej"})');
      assert.equal(await evaluate('document.getElementById("progExecution").dataset.status'),"paused");
      assert.equal(await evaluate('document.getElementById("btnProgPause").hidden && !document.getElementById("btnProgRun").hidden && !document.getElementById("btnProgStep").hidden && !document.getElementById("btnProgStop").hidden'),true);
      assert.match(await evaluate('document.getElementById("btnProgRun").textContent'),/继续运行/);
      assert.equal(await evaluate('document.querySelector(".tree-node.executing .tree-step-status").textContent'),"已暂停");
      await screenshot("programs-paused");
      await evaluate('emitProgram({status:"completed",name:"结构测试",completed:5})');
      assert.equal(await evaluate('document.getElementById("progExecution").dataset.status'),"completed");
      assert.equal(await evaluate('document.getElementById("btnProgPause").hidden && document.getElementById("btnProgStop").hidden && !document.querySelector(".tree-tools").hidden'),true);
      assert.equal(await evaluate('!!document.querySelector(".tree-node.executing")'),false);
      assert.equal(await evaluate('document.getElementById("btnProgRun").textContent'),"▶ 运行");
      await screenshot("programs-completed");
      await evaluate('emitProgram({status:"error",name:"结构测试",completed:4,error:"动作超时，程序已停止"});emitProgram({status:"error",name:"结构测试",completed:4,error:"动作超时，程序已停止"})');
      assert.match(await evaluate('document.getElementById("progCurrentStep").textContent'),/动作超时/);
      assert.equal(await evaluate('testStation.notices.filter(text=>text==="动作超时，程序已停止").length'),1,"相同错误不能随快照重复通知");
      assert.equal(await evaluate('!document.getElementById("btnProgStop").hidden && !document.getElementById("btnProgStop").disabled'),true);
    });
    await run("other pages retain a compact task and stop control with immediate route updates",async()=>{
      await evaluate('emitProgram({status:"running",name:"结构测试",completed:4,current_id:currentStep.id,current_type:"movej"});location.hash="#/arm/control"');
      await wait('!document.getElementById("globalTask").hidden');
      assert.equal(await evaluate('document.getElementById("globalTaskLabel").textContent'),"结构测试");
      assert.equal(await evaluate('document.getElementById("globalTask").dataset.status'),"running");
      assert.equal(await evaluate('!document.getElementById("btnGlobalProgramStop").hidden'),true);
      assert.match(await evaluate('document.querySelector("#globalTask a").getAttribute("href")'),/work\/program/);
      await screenshot("programs-cross-page-status");
      await evaluate('location.hash="#/work/program"');await wait('document.getElementById("globalTask").hidden');
      await evaluate('emitProgram({status:"idle"});testStation.snap.arm={...testStation.snap.arm,recording:true,traj_file:"现场录制.jsonl",rec_s:1.5};emitProgram({status:"idle"})');
      assert.equal(await evaluate('!document.getElementById("globalTask").hidden && document.getElementById("globalTask").dataset.status==="recording"'),true);
      await evaluate('document.getElementById("btnGlobalProgramStop").click()');assert.equal(await evaluate('testStation.sent.at(-1).t'),"rec_stop");
      await evaluate('location.hash="#/work/trajectories"');await wait('document.getElementById("globalTask").hidden');
      await evaluate('testStation.snap.arm.recording=false;emitProgram({status:"idle"});location.hash="#/work/program"');
      await wait('document.getElementById("page-program").classList.contains("on")');
    });
    assert.deepEqual(await evaluate('testStation.errors'),[]);console.log("program browser tests passed");
  }finally{
    if(closeBrowser)await Promise.race([closeBrowser().catch(()=>{}),delay(2000)]);if(socket)socket.close();
    if(process.platform==="win32"&&child.exitCode===null){try{execFileSync("taskkill",["/PID",String(child.pid),"/T","/F"],{windowsHide:true,stdio:"ignore",timeout:5000});}catch{child.kill();}}else child.kill();server.close();
    await new Promise(resolve=>{if(child.exitCode!==null)resolve();else child.once("exit",resolve);});
    assert.ok(path.resolve(profile).startsWith(path.resolve(os.tmpdir())+path.sep)&&path.basename(profile).startsWith("fafu-programs-"));fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:200});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;server.close();});
