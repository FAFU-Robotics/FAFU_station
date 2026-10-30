"use strict";
// Real DOM, WebGL and native dialogs; all robot commands terminate in this fixture.
const assert=require("node:assert/strict"),fs=require("node:fs"),http=require("node:http"),os=require("node:os"),path=require("node:path");
const {spawn,execFileSync}=require("node:child_process");
const root=path.resolve(__dirname,"../../robot_station/web/static");
const browser=[process.env.CHROME_BIN,"C:/Program Files/Google/Chrome/Application/chrome.exe","C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe","/usr/bin/chromium","/usr/bin/google-chrome"].find(p=>p&&fs.existsSync(p));
if(!browser){console.log("SKIP: Chrome/Chromium not installed (or set CHROME_BIN)");process.exit(0);}
const fixture=`
  const copy=x=>JSON.parse(JSON.stringify(x));
  const samples=Array.from({length:801},(_,i)=>({t:i/100,pos:[Math.sin(i/150)*0.5,0.7+Math.sin(i/180)*0.3,0.7+Math.cos(i/150)*0.2,Math.sin(i/100)*0.2,0,0],gpos:0.8,vel:[0,0,0,0,0,0]}));
  const persisted=JSON.parse(sessionStorage.getItem("trajectory-fixture")||"null");
  const endpointPose=()=>samples.at(-1).pos.map(q=>q*180/Math.PI);
  window.testStation={files:persisted?.files||[{name:"抓取与放置.jsonl",frames:801,s:8,teach:"drag",mtime:1793260800},{name:"料盘返回.jsonl",frames:801,s:8,teach:"soft"}],histories:persisted?.histories||{},editorVersion:sessionStorage.getItem("old-editor")?1:3,sent:[],errors:[],arm:{online:true,enabled:true,q_deg:endpointPose(),gripper_deg:0.8*180/Math.PI,recording:false,replay_active:false},fail:false,hold:false,waiting:[]};
  window.emit=()=>window.dispatchEvent(new CustomEvent("station:snapshot",{detail:{arm:copy(testStation.arm),program:{status:"idle"},trajectory:{endpoint_hold_id:testStation.endpoint||""}}}));
  window.fetchOriginal=window.fetch;
  window.fetch=async (url,...args)=>url==="/api/traj"?{ok:true,json:async()=>({files:copy(testStation.files)})}:fetchOriginal(url,...args);
  window.StationUI={connected:()=>true,page:()=>location.hash.slice(2),snapshot:()=>({arm:testStation.arm}),releaseManual:()=>{},notice:()=>{},sendImmediate(msg){
    testStation.sent.push(copy(msg));
    const reply=()=>{
      let data={},error="";
      const file=msg.file||"";
      if(testStation.fail){testStation.fail=false;error="模拟操作失败";}
      else if(msg.action==="traj_process"){
        let rows=copy(samples),duration=8,selection={start:0,end:8};
        const history=testStation.histories[file]||{operations:[],future:[]};
        const operations=msg.operations??history.operations,future=msg.future??(msg.operations?[]:history.future);
        for(const op of operations){
          const before=duration;
          if(op.type==="trim"){rows=rows.filter(x=>x.t>=op.start&&x.t<=op.end);rows.forEach(x=>x.t-=op.start);duration=op.end-op.start;}
          if(op.type==="speed"){
            if(op.scope==="selection"&&testStation.editorVersion>=3){const delta=(op.end-op.start)/op.factor-(op.end-op.start);rows.forEach(x=>{if(x.t>op.end)x.t+=delta;else if(x.t>=op.start)x.t=op.start+(x.t-op.start)/op.factor;});duration+=delta;}
            else {rows.forEach(x=>x.t/=op.factor);duration/=op.factor;}
          }
          selection=op.type==="select"&&testStation.editorVersion>=3?{start:op.start,end:op.end}:op.scope==="selection"&&testStation.editorVersion>=3?{start:op.start,end:op.end+duration-before}:{start:0,end:duration};
        }
        let name=file;
        if(msg.save){name=(msg.name||"编辑副本")+".jsonl";testStation.files.unshift({name,frames:rows.length,s:duration,teach:"soft"});testStation.histories[name]=copy({operations,future});}
        data={name,frames:rows,count:rows.length,duration,max_speed_deg_s:25,preview_stride:1,operations,future,selection,editor_version:testStation.editorVersion,endpoint_hold:true};
      }else if(msg.action==="traj_record"){
        const name=(msg.name||"新录制")+".jsonl";
        testStation.arm={online:true,enabled:true,recording:true,traj_file:name,rec_s:1.25,rec_frames:125,q_deg:endpointPose(),gripper_deg:0.8*180/Math.PI};
        testStation.files.unshift({name,frames:125,s:1.25,teach:msg.teach});data={name};
      }else if(msg.action==="traj_record_stop"){
        testStation.arm.recording=false;data={name:testStation.arm.traj_file};
      }else if(msg.action==="traj_endpoint"){
        testStation.endpoint=msg.hold_id;testStation.arm.moving=true;data={active:true};
      }else if(msg.action==="traj_endpoint_touch"){
        data={active:testStation.endpoint===msg.hold_id};
      }else if(msg.action==="traj_endpoint_stop"){
        if(testStation.endpoint===msg.hold_id){testStation.endpoint="";testStation.arm.moving=false;}data={active:false};
      }else if(msg.action==="traj_rename"){
        const name=msg.name+".jsonl";testStation.files.find(x=>x.name===file).name=name;testStation.histories[name]=testStation.histories[file];data={name};
      }else if(msg.action==="traj_delete"){testStation.files=testStation.files.filter(x=>x.name!==file);}
      else if(msg.action==="traj_execute"){testStation.arm.replay_active=true;}
      else if(msg.action==="stop"){testStation.arm.replay_active=false;}
      sessionStorage.setItem("trajectory-fixture",JSON.stringify({files:testStation.files,histories:testStation.histories}));
      window.dispatchEvent(new CustomEvent("station:ack",{detail:{request_id:msg.request_id,ok:!error,data,error}}));
      setTimeout(emit,10);
    };
    if(testStation.hold)testStation.waiting.push(reply);else queueMicrotask(reply);
  }};
  window.addEventListener("error",ev=>testStation.errors.push(ev.message));
  window.addEventListener("unhandledrejection",ev=>testStation.errors.push(String(ev.reason)));
  window.click=selector=>document.querySelector(selector).click();
  window.edit=(selector,value)=>{const input=document.querySelector(selector);input.value=value;input.dispatchEvent(new Event("change"));};
  window.showPage=()=>{document.querySelectorAll(".page").forEach(x=>x.classList.toggle("on",x.id==="page-"+(location.hash.includes("trajectories")?"trajectories":"arm")));window.Arm3D?.setActive(!location.hash.includes("trajectories"));};
  location.hash="#/work/trajectories";showPage();window.addEventListener("hashchange",showPage);
`;
const metrics=`
  window.renderStats={contexts:0,meshes:0,normals:0,joints:0,segments:0,renders:0,scenes:[],times:[]};
  const nativeRenderer=THREE.WebGLRenderer;
  THREE.WebGLRenderer=function(options){
    const renderer=new nativeRenderer(options);renderStats.contexts++;
    const render=renderer.render.bind(renderer);
    renderer.render=(scene,camera)=>{renderStats.renders++;renderStats.times.push(performance.now());renderStats.lastScene=scene;renderStats.lastCamera=camera;if(!renderStats.scenes.includes(scene))renderStats.scenes.push(scene);return render(scene,camera);};
    return renderer;
  };
  for(const [type,method,key] of [[THREE.BufferGeometry,"computeVertexNormals","normals"],[THREE.Quaternion,"setFromAxisAngle","joints"],[CanvasRenderingContext2D,"lineTo","segments"]]){
    const original=type.prototype[method];type.prototype[method]=function(...args){renderStats[key]++;return original.apply(this,args);};
  }
  const nativeFetch=window.fetch;window.fetch=(url,...args)=>{if(String(url).endsWith(".bin"))renderStats.meshes++;return nativeFetch(url,...args);};
  window.meshIds=scene=>{const ids=[];scene.traverse(object=>{if(object.isMesh)ids.push(object.geometry.uuid);});return ids.sort();};
  window.firstJoint=scene=>scene.children.find(object=>object.isGroup).children.find(object=>object.isGroup).quaternion.toArray();
`;
const html=fs.readFileSync(path.join(root,"index.html"),"utf8").replace(/<script\b[^>]*>[\s\S]*?<\/script>/g,"").replace("</body>",'<script src="/static/js/theme.js"></script><script src="/fixture.js"></script><script src="/static/js/dialog.js"></script><script src="/static/vendor/three.min.js"></script><script src="/metrics.js"></script><script src="/static/js/arm3d.js"></script><script src="/static/js/trajectories.js"></script></body>');
const server=http.createServer((req,res)=>{
  if(req.url==="/"){res.setHeader("Content-Type","text/html; charset=utf-8");res.end(html);return;}
  if(req.url==="/fixture.js"){res.setHeader("Content-Type","text/javascript; charset=utf-8");res.end(fixture);return;}
  if(req.url==="/metrics.js"){res.setHeader("Content-Type","text/javascript; charset=utf-8");res.end(metrics);return;}
  const file=path.resolve(root,decodeURIComponent(req.url.split("?")[0]).replace(/^\/static\//,""));
  if(!file.startsWith(root+path.sep)||!fs.existsSync(file)||!fs.statSync(file).isFile()){res.writeHead(404);res.end();return;}
  res.setHeader("Content-Type",file.endsWith(".css")?"text/css":file.endsWith(".bin")?"application/octet-stream":"text/javascript");res.end(fs.readFileSync(file));
});
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function until(fn){const end=Date.now()+15000;while(Date.now()<end){const result=await fn();if(result)return result;await delay(25);}throw new Error("Timed out waiting for browser state");}
async function main(){
  await new Promise(resolve=>server.listen(0,"127.0.0.1",resolve));
  const profile=fs.mkdtempSync(path.join(os.tmpdir(),"fafu-trajectories-"));
  const child=spawn(browser,["--headless=new","--no-first-run","--no-default-browser-check","--disable-background-networking","--disable-extensions","--use-angle=swiftshader","--enable-unsafe-swiftshader","--remote-debugging-port=0","--user-data-dir="+profile,"about:blank"],{stdio:"ignore",windowsHide:true});
  let socket, closeBrowser;
  try{
    const portFile=path.join(profile,"DevToolsActivePort");await until(()=>fs.existsSync(portFile));
    const port=fs.readFileSync(portFile,"utf8").split("\n")[0];
    const pages=await(await fetch("http://127.0.0.1:"+port+"/json/list")).json();
    socket=new WebSocket(pages.find(p=>p.type==="page").webSocketDebuggerUrl);await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
    let serial=0;const pending=new Map();
    socket.onmessage=event=>{const msg=JSON.parse(event.data),task=pending.get(msg.id);if(task){pending.delete(msg.id);if(msg.error)task.reject(new Error(msg.error.message));else task.resolve(msg.result);}};
    const cdp=(method,params={})=>new Promise((resolve,reject)=>{const id=++serial;pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});
    closeBrowser=()=>cdp("Browser.close");
    const evaluate=async expression=>{const result=await cdp("Runtime.evaluate",{expression,returnByValue:true,awaitPromise:true});if(result.exceptionDetails)throw new Error(result.exceptionDetails.exception?.description||result.exceptionDetails.text);return result.result.value;};
    const wait=async expression=>{
      try{return await until(async()=>{try{return await evaluate(expression);}catch(error){if(/context was destroyed|Cannot find context|Inspected target navigated/.test(error.message))return false;throw error;}});}
      catch(error){
        const context=await evaluate('({arm:window.StationUI?.snapshot?.()?.arm,notice:document.getElementById("commandNotice")?.textContent,errors:window.uiTestErrors,targets:Array.from(document.querySelectorAll("[data-jn]")).map(x=>x.value),sendDisabled:document.getElementById("btnSendArm")?.disabled,draft:document.getElementById("trajDraftState")?.textContent,trajAck:document.getElementById("trajAck")?.textContent,undoDisabled:document.getElementById("btnTrajUndo")?.disabled,sent:window.testStation?.sent?.slice(-5)})');
        throw new Error(error.message+"; condition="+expression+"; context="+JSON.stringify(context));
      }
    };
    const run=async(name,fn)=>{await fn();console.log("ok - "+name);};
    const screenshot=async(name,selector=null)=>{if(!process.env.FAFU_UI_SCREENSHOTS)return;const clip=selector?await evaluate('(()=>{const r=document.querySelector('+JSON.stringify(selector)+').getBoundingClientRect();return {x:r.x+scrollX,y:r.y+scrollY,width:r.width,height:r.height,scale:1};})()'):null;const {data}=await cdp("Page.captureScreenshot",{format:"png",captureBeyondViewport:true,...(clip?{clip}:{})});fs.writeFileSync(path.join(process.env.FAFU_UI_SCREENSHOTS,name+".png"),Buffer.from(data,"base64"));};
    const drag=async(from,to)=>{
      await cdp("Input.dispatchMouseEvent",{type:"mouseMoved",x:from.x,y:from.y});
      await cdp("Input.dispatchMouseEvent",{type:"mousePressed",x:from.x,y:from.y,button:"left",clickCount:1});
      for(let i=1;i<=8;i++)await cdp("Input.dispatchMouseEvent",{type:"mouseMoved",x:from.x+(to.x-from.x)*i/8,y:from.y+(to.y-from.y)*i/8,button:"left",buttons:1});
      await cdp("Input.dispatchMouseEvent",{type:"mouseReleased",x:to.x,y:to.y,button:"left",clickCount:1});
    };
    await cdp("Emulation.setDeviceMetricsOverride",{width:1600,height:1200,deviceScaleFactor:1,mobile:false});
    if(process.env.FAFU_REAL_STATION_URL){
      await cdp("Page.enable");
      await cdp("Page.addScriptToEvaluateOnNewDocument",{source:'window.uiTestErrors=[];addEventListener("error",e=>uiTestErrors.push(e.message));addEventListener("unhandledrejection",e=>uiTestErrors.push(String(e.reason)));'});
      await cdp("Page.navigate",{url:process.env.FAFU_REAL_STATION_URL+"/#/work/trajectories"});
      await wait('window.StationUI?.connected() && document.getElementById("trajMeta").textContent.includes("201 帧")');
      await wait('document.querySelector("#traj3dHost canvas") && document.getElementById("traj3dOverlay").textContent.includes("CAD")');
      const initial=await evaluate('StationUI.snapshot().arm.q_deg');
      await evaluate('document.getElementById("btnTrajPreview").click()');await delay(200);
      await evaluate('document.getElementById("btnTrajPreview").click()');
      assert.deepEqual(await evaluate('StationUI.snapshot().arm.q_deg'),initial);
      await screenshot("trajectories-station");
      await evaluate('document.getElementById("trajIn").value="0.2";document.getElementById("trajOut").value="1.8";document.getElementById("btnTrajTrim").click()');
      await wait('document.getElementById("trajDraftState").dataset.editCount==="1"');
      await evaluate('document.getElementById("trajSaveName").value="integration-edited";document.getElementById("btnTrajSave").click()');
      await wait('document.getElementById("trajSelectedTitle").textContent==="integration-edited"');
      await screenshot("trajectories-history");
      await evaluate('document.getElementById("trajIn").value="0.3";document.getElementById("trajOut").value="0.8";document.getElementById("trajIn").dispatchEvent(new Event("change"));document.getElementById("trajEditTool").value="smooth";document.getElementById("trajEditTool").dispatchEvent(new Event("change"));document.getElementById("btnTrajApply").click()');
      await wait('document.getElementById("trajDraftState").dataset.editCount==="2" && !document.getElementById("btnTrajSave").disabled');
      assert.match(await evaluate('document.getElementById("trajHistory").textContent'),/选区 0.300–0.800/);
      await screenshot("trajectories-selection");
      await evaluate('document.getElementById("trajSaveName").value="integration-scoped";document.getElementById("btnTrajSave").click()');
      await wait('document.getElementById("trajSelectedTitle").textContent==="integration-scoped" && !document.getElementById("btnTrajNew").disabled');
      await wait('!document.getElementById("btnTrajEndpoint").disabled');
      const holdButton=await evaluate('(()=>{const b=document.getElementById("btnTrajEndpoint");b.scrollIntoView({block:"center"});const r=b.getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()');
      await cdp("Input.dispatchMouseEvent",{type:"mousePressed",...holdButton,button:"left",clickCount:1});
      await wait('StationUI.snapshot().arm.moving');await delay(150);
      await cdp("Input.dispatchMouseEvent",{type:"mouseReleased",...holdButton,button:"left",clickCount:1});
      await wait('!StationUI.snapshot().arm.moving && document.getElementById("btnTrajEndpoint").getAttribute("aria-pressed")==="false"');
      const stopped=await evaluate('StationUI.snapshot().arm.q_deg');await delay(300);
      assert.ok((await evaluate('StationUI.snapshot().arm.q_deg')).every((q,i)=>Math.abs(q-stopped[i])<0.2),"松开移动按钮后仿真姿态应保持");
      await cdp("Input.dispatchMouseEvent",{type:"mousePressed",...holdButton,button:"left",clickCount:1});
      await wait('document.getElementById("trajEndpointLabel").textContent.includes("已到末点") && document.getElementById("btnTrajEndpoint").disabled');
      await cdp("Input.dispatchMouseEvent",{type:"mouseReleased",...holdButton,button:"left",clickCount:1});
      await evaluate('document.getElementById("btnTrajNew").click()');
      await wait('document.getElementById("trajSelectedTitle").textContent.includes("选择轨迹")');
      await evaluate('document.getElementById("trajInputMode").value="soft";document.getElementById("trajRecordName").value="integration-recorded";document.getElementById("btnRecStart").click()');
      await wait('StationUI.snapshot().arm.recording');await delay(200);
      await wait('!document.getElementById("btnRecStart").disabled');
      await evaluate('document.getElementById("btnRecStart").click()');
      await wait('document.getElementById("trajSelectedTitle").textContent==="integration-recorded"');
      await wait('!document.getElementById("btnTrajContinue").disabled');
      await evaluate('document.getElementById("btnTrajContinue").click()');
      await wait('StationUI.snapshot().arm.recording');await delay(150);
      await wait('!document.getElementById("btnRecStart").disabled');
      await evaluate('document.getElementById("btnRecStart").click()');
      await wait('!StationUI.snapshot().arm.recording && document.getElementById("trajSelectedTitle").textContent.includes("续录")');
      await wait('!document.getElementById("btnTrajExecute").disabled');
      await evaluate('document.getElementById("btnTrajExecute").click()');
      await wait('StationUI.snapshot().arm.replay_active');
      await evaluate('document.getElementById("btnTrajStop").click()');
      await wait('!StationUI.snapshot().arm.replay_active');
      await evaluate('location.hash="#/arm/control"');
      await wait('document.getElementById("page-arm").classList.contains("on") && !document.getElementById("btnHome").disabled');
      await evaluate('document.getElementById("armSpeed").value="40";document.getElementById("btnHome").click()');
      await wait('StationUI.snapshot().arm.q_deg.every(q=>Math.abs(q)<0.3) && !StationUI.snapshot().arm.moving');
      await evaluate('document.getElementById("armSpeed").value="10";for(const [j,q] of [[0,30],[1,40],[2,40]]){const input=document.querySelectorAll("[data-jn]")[j];input.value=q;input.dispatchEvent(new Event("input",{bubbles:true}));}document.getElementById("btnSendArm").click()');
      await wait('StationUI.snapshot().arm.moving');
      await delay(150);
      const afterHome=await evaluate('StationUI.snapshot().arm.q_deg');
      assert.ok(afterHome[1]<8,"发送位置必须按速度插值，不能回到复位前的旧目标");
      await evaluate('StationUI.sendImmediate({t:"workbench",action:"stop",request_id:"test-stop"})');
      await wait('!StationUI.snapshot().arm.moving');
      await evaluate('location.hash="#/work/trajectories"');
      await wait('!document.getElementById("btnTrajNew").disabled');
      await evaluate('document.getElementById("btnTrajNew").click()');
      await wait('document.getElementById("trajSelectedTitle").textContent.includes("选择轨迹")');
      await evaluate('document.getElementById("trajInputMode").value="drag";document.getElementById("trajRecordName").value="integration-drag";document.getElementById("btnRecStart").click()');
      await wait('StationUI.snapshot().arm.recording && StationUI.snapshot().arm.ctrl_mode==="Gravity"');
      await evaluate('location.hash="#/arm/control"');
      await wait('document.querySelectorAll("[data-jn]")[0].disabled');
      assert.equal(await evaluate('document.querySelector("[data-mode=Position]").disabled'),true);
      assert.equal(await evaluate('document.getElementById("simTeachEnabled").checked'),false);
      const beforeDrag=await evaluate('StationUI.snapshot().arm.q_deg[0]');
      await evaluate('{const target=document.querySelectorAll("[data-jn]")[0];target.value=50;target.dispatchEvent(new Event("input",{bubbles:true}));}');
      await delay(100);
      assert.ok(Math.abs((await evaluate('StationUI.snapshot().arm.q_deg[0]'))-beforeDrag)<1e-4,"未启用仿真拖动时姿态应保持；允许 JSON 与 float32 遥测的舍入差异");
      await evaluate('document.getElementById("simTeachEnabled").click()');
      await wait('!document.querySelectorAll("[data-jn]")[0].disabled');
      await evaluate('{const target=document.querySelectorAll("[data-jn]")[0];target.value=15;target.dispatchEvent(new Event("input",{bubbles:true}));}');
      await wait('Math.abs(StationUI.snapshot().arm.q_deg[0]-15)<0.2');
      assert.equal(await evaluate('StationUI.snapshot().arm.ctrl_mode'),"Gravity");
      await screenshot("manual-teach-simulation");
      await evaluate('location.hash="#/work/trajectories"');
      await wait('document.getElementById("btnRecStart").textContent.includes("停止并保存")');
      await evaluate('document.getElementById("btnRecStart").click()');
      await wait('!StationUI.snapshot().arm.recording && StationUI.snapshot().arm.ctrl_mode==="Position"');
      assert.deepEqual(await evaluate('uiTestErrors'),[]);
      console.log("ok - station UI: edit/save, recording, continuation, execution, home/send speed, and explicit simulated drag");
      return;
    }
    await cdp("Page.navigate",{url:"http://127.0.0.1:"+server.address().port+"/"});
    await wait('document.getElementById("trajMeta").textContent.includes("801")');
    await run("offline Chinese font loads and light-mode secondary and disabled text meet contrast targets",async()=>{
      await evaluate('document.fonts.load(\'14px "Noto Sans SC"\',"机械臂 Aa 12")');
      assert.equal(await evaluate('document.fonts.check(\'14px "Noto Sans SC"\',"机械臂 Aa 12")'),true);
      await evaluate('StationTheme.set("light")');await delay(250);
      const contrasts=await evaluate(`(()=>{
        const css=getComputedStyle(document.documentElement),rgb=name=>{let h=css.getPropertyValue(name).trim().replace('#','');if(h.length===3)h=[...h].map(c=>c+c).join('');return h.match(/../g).map(n=>parseInt(n,16)/255);},lum=name=>rgb(name).map(v=>v<=.04045?v/12.92:((v+.055)/1.055)**2.4).reduce((sum,v,i)=>sum+v*[.2126,.7152,.0722][i],0),ratio=(a,b)=>{const x=lum(a),y=lum(b);return (Math.max(x,y)+.05)/(Math.min(x,y)+.05);};
        return [['--text','--card'],['--muted','--card'],['--dim','--bg'],['--disabled-text','--disabled-bg']].map(([a,b])=>({pair:a+'/'+b,ratio:ratio(a,b)}));
      })()`);
      for(const result of contrasts)assert.ok(result.ratio>=4.5,JSON.stringify(result));
      assert.equal(await evaluate('getComputedStyle(document.getElementById("btnTrajEndpoint")).opacity'),"1");
      await screenshot("trajectories-light");
      await evaluate('StationTheme.set("dark")');await delay(250);
    });
    await run("real 3D canvas and independent playback never command hardware",async()=>{
      await wait('document.querySelector("#traj3dHost canvas") && document.getElementById("traj3dOverlay").textContent.includes("CAD") && renderStats.renders>0');
      assert.equal(await evaluate('document.querySelectorAll("#btnRecStop").length'),0);
      const count=await evaluate('testStation.sent.length');
      await evaluate('click("#btnTrajPreview")');
      await wait('!document.getElementById("trajClock").textContent.startsWith("0.00")');
      await evaluate('click("#btnTrajPreview")');
      assert.equal(await evaluate('testStation.sent.length'),count);
      assert.equal(await evaluate('document.getElementById("trajClock").textContent.startsWith("0.00")'),false);
      await screenshot("trajectories-desktop");
    });
    await run("idle views stop drawing and playback reuses the cached joint curves",async()=>{
      await delay(650);
      const idle=await evaluate('renderStats.renders');
      await delay(250);
      assert.equal(await evaluate('renderStats.renders'),idle,"静止视图不应持续渲染");
      const segments=await evaluate('renderStats.segments');
      await evaluate('click("#btnTrajPreview")');await delay(450);await evaluate('click("#btnTrajPreview")');
      const drawn=await evaluate('renderStats.segments');
      assert.ok(drawn-segments<60,"播放时应复用曲线底图，只更新播放头");
      assert.equal(await evaluate('renderStats.contexts'),1,"首次打开轨迹页不应初始化隐藏的主视图");
      console.log("metrics - idle renders: 0; curve segments during playback: "+(drawn-segments));
    });
    await run("page switches share one context and CAD buffers while preserving independent poses",async()=>{
      await delay(650);
      await evaluate('window.previewScene=renderStats.lastScene;window.previewPose=firstJoint(previewScene);window.previewCamera=renderStats.lastCamera.position.toArray()');
      const normals=await evaluate('renderStats.normals');
      const hidden=await evaluate('({renders:renderStats.renders,joints:renderStats.joints})');
      await evaluate('for(let i=0;i<100;i++)Arm3D.setPose([i,40,40,0,0,0],50)');
      await delay(150);
      assert.deepEqual(await evaluate('({renders:renderStats.renders,joints:renderStats.joints})'),hidden,"隐藏主视图应只保存最新状态");
      await evaluate('location.hash="#/arm/control"');
      await wait('document.querySelector("#arm3dHost canvas") && renderStats.scenes.length===2');
      assert.equal(await evaluate('renderStats.contexts'),1);
      assert.equal(await evaluate('renderStats.meshes'),7);
      assert.equal(await evaluate('renderStats.normals'),normals,"第二个视图不应重复解析网格或拆分夹爪");
      assert.deepEqual(await evaluate('meshIds(renderStats.lastScene)'),await evaluate('meshIds(previewScene)'),"两个场景应引用同一套 CAD 几何体");
      assert.ok(Math.abs((await evaluate('firstJoint(renderStats.lastScene)'))[2]-Math.sin(99*Math.PI/360))<1e-6,"重新显示主视图应使用最新姿态");
      await evaluate('location.hash="#/work/trajectories"');
      await wait('document.querySelector("#traj3dHost canvas") && renderStats.lastScene===previewScene');
      assert.deepEqual(await evaluate('firstJoint(previewScene)'),await evaluate('previewPose'));
      assert.deepEqual(await evaluate('renderStats.lastCamera.position.toArray()'),await evaluate('previewCamera'));
      assert.equal(await evaluate('document.querySelectorAll("#arm3dHost canvas,#traj3dHost canvas").length'),1);
      console.log("metrics - WebGL contexts: 1; CAD fetches: 7; CAD geometries shared across both views");
    });
    await run("high-rate feedback is coalesced and hidden windows suspend all pose work",async()=>{
      await evaluate('location.hash="#/arm/control"');
      await wait('document.querySelector("#arm3dHost canvas") && renderStats.lastScene!==previewScene');
      await evaluate('window.feed={start:performance.now(),renders:renderStats.renders,n:0};window.feedTimer=setInterval(()=>Arm3D.setPose([++feed.n%60,40,40,0,0,0],50),2)');
      await delay(650);
      const rate=await evaluate('clearInterval(feedTimer);({elapsed:performance.now()-feed.start,renders:renderStats.renders-feed.renders})');
      assert.ok(rate.renders>0);
      assert.ok(rate.renders<=Math.ceil(rate.elapsed*30/1000)+2,"高频反馈不能突破显示帧率上限");
      await delay(150);
      await evaluate('Object.defineProperty(document,"hidden",{configurable:true,value:true});document.dispatchEvent(new Event("visibilitychange"))');
      const hidden=await evaluate('({renders:renderStats.renders,joints:renderStats.joints})');
      await evaluate('for(let i=0;i<100;i++)Arm3D.setPose([i+24,40,40,0,0,0],50)');
      await delay(150);
      assert.deepEqual(await evaluate('({renders:renderStats.renders,joints:renderStats.joints})'),hidden);
      await evaluate('delete document.hidden;document.dispatchEvent(new Event("visibilitychange"))');
      await wait('Math.abs(firstJoint(renderStats.lastScene)[2]-Math.sin(123*Math.PI/360))<1e-6');
      await evaluate('location.hash="#/work/trajectories"');
      await wait('document.querySelector("#traj3dHost canvas") && renderStats.lastScene===previewScene');
      console.log("metrics - high-rate feedback: "+rate.renders+" draws / "+Math.round(rate.elapsed)+" ms; hidden window draws and joint updates: 0");
    });
    await run("video-style selection supports drag creation, trim handles, movement, keys and touch",async()=>{
      const historyBefore=await evaluate('Number(document.getElementById("trajDraftState").dataset.historyCount)');
      const plot=await evaluate('(()=>{const r=document.getElementById("trajChart").getBoundingClientRect();return {x:r.x+28,y:r.y+r.height/2,width:r.width-36};})()');
      await drag({x:plot.x+plot.width/8,y:plot.y},{x:plot.x+plot.width*5/8,y:plot.y});
      assert.equal(await evaluate('Number(document.getElementById("trajDraftState").dataset.historyCount)'),historyBefore+1,"完整拖动只记录一步，不能把每次 pointermove 都存入历史");
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajIn").value)')-1)<0.03);
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-5)<0.03);
      const edge=await evaluate('(()=>{const r=document.getElementById("trajRangeEnd").getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()');
      await drag(edge,{x:plot.x+plot.width*6/8,y:edge.y});
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-6)<0.04);
      const middle=await evaluate('(()=>{const r=document.getElementById("trajRange").getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()');
      await drag(middle,{x:middle.x+plot.width*0.5/8,y:middle.y});
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajIn").value)')-1.5)<0.04);
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-6.5)<0.04);
      await evaluate('document.getElementById("trajRangeStart").focus()');
      const before=await evaluate('Number(document.getElementById("trajIn").value)');
      await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"ArrowRight",code:"ArrowRight"});
      await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"ArrowRight",code:"ArrowRight"});
      assert.ok(await evaluate('Number(document.getElementById("trajIn").value)')>before);
      await evaluate('document.getElementById("trajScrub").value=2;document.getElementById("trajScrub").dispatchEvent(new Event("input"));window.dispatchEvent(new KeyboardEvent("keydown",{code:"KeyI"}));document.getElementById("trajScrub").value=3.5;document.getElementById("trajScrub").dispatchEvent(new Event("input"));window.dispatchEvent(new KeyboardEvent("keydown",{code:"KeyO"}));');
      assert.deepEqual(await evaluate('[Number(document.getElementById("trajIn").value),Number(document.getElementById("trajOut").value)]'),[2,3.5]);
      await cdp("Emulation.setTouchEmulationEnabled",{enabled:true});
      await cdp("Input.dispatchTouchEvent",{type:"touchStart",touchPoints:[{x:plot.x+plot.width*0.25,y:plot.y}]});
      await cdp("Input.dispatchTouchEvent",{type:"touchMove",touchPoints:[{x:plot.x+plot.width*0.5,y:plot.y}]});
      await cdp("Input.dispatchTouchEvent",{type:"touchEnd",touchPoints:[]});
      await cdp("Emulation.setTouchEmulationEnabled",{enabled:false});
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-4)<0.04);
      await evaluate('document.getElementById("trajRangeEnd").dispatchEvent(new MouseEvent("dblclick",{bubbles:true}))');
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-4)<0.04,"双击手柄不能意外全选");
      await evaluate('click("#btnTrajSelectAll")');
      assert.equal(await evaluate('document.getElementById("btnTrajExecute").disabled'),false,"仅改变选区不应要求重新保存运动数据才能执行");
      await evaluate('window.dispatchEvent(new KeyboardEvent("keydown",{code:"KeyZ",ctrlKey:true}))');
      await wait('!document.getElementById("btnTrajRedo").disabled');
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-4)<0.04);
      await evaluate('window.dispatchEvent(new KeyboardEvent("keydown",{code:"KeyZ",ctrlKey:true,shiftKey:true}))');
      await wait('Number(document.getElementById("trajOut").value)===8 && !document.getElementById("btnTrajUndo").disabled');
      await evaluate('document.querySelectorAll("[data-traj-restore]")[1].click()');
      await wait('document.getElementById("trajDraftState").dataset.historyCount==="1" && !document.getElementById("btnTrajRestoreSaved").disabled');
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-5)<0.04);
      await evaluate('click("#btnTrajRestoreSaved")');
      await wait('document.getElementById("trajDraftState").dataset.historyCount==="0" && !document.getElementById("btnTrajApply").disabled');
    });
    await run("native numeric input commits one history entry even after a drag keeps the input focused",async()=>{
      await evaluate('const input=document.getElementById("trajOut");input.closest("details").open=true;input.focus()');
      const plot=await evaluate('(()=>{const r=document.getElementById("trajChart").getBoundingClientRect();return {x:r.x+28,y:r.y+r.height/2,width:r.width-36};})()');
      await drag({x:plot.x,y:plot.y},{x:plot.x+plot.width/2,y:plot.y});
      assert.equal(await evaluate('document.activeElement.id'),"trajOut");
      assert.equal(await evaluate('document.getElementById("trajDraftState").dataset.historyCount'),"1");
      await evaluate('document.getElementById("trajOut").select()');
      await cdp("Input.insertText",{text:"8"});
      assert.equal(await evaluate('document.getElementById("trajDraftState").dataset.historyCount'),"1","输入中间状态不能逐字进入历史");
      await evaluate('document.getElementById("btnTrajSelectAll").focus()');
      assert.equal(await evaluate('document.getElementById("trajDraftState").dataset.historyCount'),"2");
      await evaluate('click("#btnTrajUndo")');
      await wait('document.getElementById("trajDraftState").dataset.historyCount==="1" && !document.getElementById("btnTrajApply").disabled');
      assert.ok(Math.abs(await evaluate('Number(document.getElementById("trajOut").value)')-4)<0.04);
      await evaluate('click("#btnTrajRedo")');
      await wait('document.getElementById("trajDraftState").dataset.historyCount==="2" && !document.getElementById("btnTrajApply").disabled');
      assert.equal(await evaluate('Number(document.getElementById("trajOut").value)'),8);
      await evaluate('click("#btnTrajRestoreSaved");document.getElementById("trajOut").closest("details").open=false');
      await wait('document.getElementById("trajDraftState").dataset.historyCount==="0" && !document.getElementById("btnTrajApply").disabled');
    });
    await run("selection preview is local; all processing uses the visible range and full range means select all",async()=>{
      assert.equal(await evaluate('document.getElementById("trajEditScope")'),null);
      const commands=await evaluate('testStation.sent.length');
      await evaluate('edit("#trajIn",0.05);edit("#trajOut",0.15);click("#btnTrajPreviewSelection")');
      await wait('document.getElementById("trajClock").textContent.startsWith("0.15") && document.getElementById("btnTrajPreviewSelection").textContent.includes("预览选区")');
      assert.equal(await evaluate('testStation.sent.length'),commands);
      for(const type of ["idle","smooth","speed","resample","reverse"]){
        await evaluate('edit("#trajIn",1);edit("#trajOut",2);edit("#trajEditTool",'+JSON.stringify(type)+');click("#btnTrajApply")');
        await wait('document.getElementById("trajDraftState").dataset.editCount==="1" && !document.getElementById("btnTrajUndo").disabled');
        const op=await evaluate('testStation.sent.filter(x=>x.action==="traj_process").at(-1).operations.at(-1)');
        assert.equal(op.type,type);assert.equal(op.scope,"selection");assert.equal(op.start,1);assert.equal(op.end,2);
        assert.deepEqual(await evaluate('[Number(document.getElementById("trajIn").value),Number(document.getElementById("trajOut").value)]'),[1,2]);
        await evaluate('click("#btnTrajUndo")');await wait('document.getElementById("trajDraftState").dataset.editCount==="0" && !document.getElementById("btnTrajApply").disabled');
      }
      await evaluate('click("#btnTrajSelectAll");edit("#trajEditTool","smooth");click("#btnTrajApply")');
      await wait('document.getElementById("trajDraftState").dataset.editCount==="1" && !document.getElementById("btnTrajUndo").disabled');
      const all=await evaluate('testStation.sent.filter(x=>x.action==="traj_process").at(-1).operations.at(-1)');
      assert.equal(all.scope,"selection");assert.equal(all.start,0);assert.equal(all.end,8);
      await evaluate('click("#btnTrajUndo")');await wait('!document.getElementById("btnTrajApply").disabled');
      await evaluate('edit("#trajEditTool","merge")');
      assert.match(await evaluate('document.getElementById("btnTrajApply").textContent'),/末尾/);
      await evaluate('edit("#trajEditTool","idle");click("#btnTrajSelectAll")');
      await screenshot("trajectories-timeline");
    });
    await run("trim, undo, redo, failed operation recovery, and save a copy",async()=>{
      await evaluate('edit("#trajIn",1);edit("#trajOut",6);click("#btnTrajTrim")');
      await wait('document.getElementById("trajDraftState").dataset.editCount==="1"');
      assert.equal(await evaluate('document.getElementById("btnTrajExecute").disabled'),true);
      await evaluate('click("#btnTrajUndo")');await wait('document.getElementById("trajDraftState").dataset.editCount==="0"');
      await evaluate('click("#btnTrajRedo")');await wait('document.getElementById("trajDraftState").dataset.editCount==="1"');
      await evaluate('testStation.fail=true;click("#btnTrajApply")');await wait('document.getElementById("trajAck").textContent.includes("模拟操作失败")');
      assert.equal(await evaluate('document.getElementById("btnTrajApply").disabled'),false);
      assert.equal(await evaluate('document.querySelectorAll("#trajHistory [data-history-kind=edit]").length'),1);
      await evaluate('edit("#trajSaveName","裁剪结果");click("#btnTrajSave")');
      await wait('document.getElementById("trajSelectedTitle").textContent==="裁剪结果"');
      assert.equal(await evaluate('testStation.files.some(x=>x.name==="抓取与放置.jsonl")'),true);
    });
    await run("saved history survives reload and restores any earlier step",async()=>{
      await evaluate('window.trajectoryBeforeReload=true');
      await cdp("Page.reload");
      await wait('!window.trajectoryBeforeReload && typeof window.click==="function" && document.getElementById("trajSelectedTitle").textContent==="裁剪结果"');
      assert.equal(await evaluate('document.getElementById("btnTrajUndo").disabled'),false);
      await evaluate('click("#btnTrajUndo")');await wait('document.getElementById("trajDraftState").dataset.editCount==="0"');
      assert.equal(await evaluate('document.querySelectorAll("#trajHistory [data-history-kind=edit].undone").length'),1);
      await evaluate('click("#btnTrajRestoreSaved")');await wait('document.getElementById("trajDraftState").textContent.includes("已保存")');
      await evaluate('edit("#trajEditTool","smooth");click("#btnTrajApply")');await wait('document.querySelectorAll("#trajHistory [data-history-kind=edit]").length===2');
      await evaluate('document.querySelectorAll("[data-traj-restore]")[0].click()');await wait('document.querySelectorAll("#trajHistory [data-history-kind=edit].undone").length===2');
      await evaluate('click("#btnTrajRestoreSaved")');await wait('document.getElementById("btnRecRename").disabled===false');
    });
    await run("file management lives in library and deletion defaults to cancel",async()=>{
      await evaluate('edit("#trajLibraryName","重命名结果");click("#btnRecRename")');await wait('document.getElementById("trajSelectedTitle").textContent==="重命名结果"');
      await evaluate('click("#btnRecDelete")');await wait('document.getElementById("confirmDialog").open');
      assert.equal(await evaluate('document.activeElement.id'),"confirmCancel");
      await evaluate('click("#confirmCancel")');await wait('!document.getElementById("confirmDialog").open');
      assert.equal(await evaluate('testStation.files.some(x=>x.name==="重命名结果.jsonl")'),true);
    });
    await run("one record toggle, live library item, page changes and stop-save",async()=>{
      await evaluate('click("#btnTrajNew")');await wait('document.getElementById("trajSelectedTitle").textContent.includes("选择轨迹")');
      await evaluate('edit("#trajRecordName","现场录制");edit("#trajInputMode","soft");click("#btnRecStart")');
      await wait('document.getElementById("btnRecStart").textContent.includes("停止并保存")');
      assert.match(await evaluate('document.querySelector(".traj-file.recording").textContent'),/现场录制/);
      await evaluate('location.hash="#/arm/control"');await delay(50);await evaluate('location.hash="#/work/trajectories"');
      await wait('document.getElementById("page-trajectories").classList.contains("on")');
      assert.equal(await evaluate('testStation.arm.recording'),true);
      await evaluate('click("#btnRecStart")');await wait('document.getElementById("trajSelectedTitle").textContent==="现场录制"');
      assert.equal(await evaluate('testStation.arm.recording'),false);
    });
    await run("endpoint motion requires holding, stops on release/blur, and disables at arrival",async()=>{
      assert.equal(await evaluate('document.getElementById("btnTrajEndpoint").disabled'),true);
      assert.match(await evaluate('document.getElementById("btnTrajContinue").textContent'),/继续录制轨迹/);
      assert.equal(await evaluate('document.querySelectorAll("#btnTrajEndpoint svg,#btnTrajContinue svg").length'),2);
      await evaluate('testStation.arm.q_deg[0]-=10');await delay(220);await evaluate('emit()');
      await wait('!document.getElementById("btnTrajEndpoint").disabled');
      const before=await evaluate('testStation.sent.length');
      await evaluate('StationTheme.set("light")');await delay(250);
      await screenshot("trajectories-continuation-light",".traj-continuation");
      await evaluate('StationTheme.set("dark")');await delay(250);
      await evaluate('click("#btnTrajEndpoint")');
      assert.equal(await evaluate('testStation.sent.length'),before,"普通 click 不能启动机械臂运动");
      const button=await evaluate('(()=>{const b=document.getElementById("btnTrajEndpoint");b.scrollIntoView({block:"center"});const r=b.getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()');
      const press=()=>cdp("Input.dispatchMouseEvent",{type:"mousePressed",...button,button:"left",clickCount:1});
      const release=()=>cdp("Input.dispatchMouseEvent",{type:"mouseReleased",...button,button:"left",clickCount:1});
      await press();await wait('testStation.sent.some(x=>x.action==="traj_endpoint_touch")');
      const hold=await evaluate('testStation.endpoint');assert.ok(hold);
      await release();await wait('!testStation.endpoint');
      assert.equal(await evaluate('testStation.sent.filter(x=>x.action==="traj_endpoint_stop").at(-1).hold_id'),hold);
      await press();await wait('testStation.endpoint');
      await evaluate('window.dispatchEvent(new Event("blur"))');await wait('!testStation.endpoint');await release();
      await evaluate('testStation.hold=true');await press();await release();
      await evaluate('testStation.hold=false;testStation.waiting.splice(0).forEach(reply=>reply())');
      await wait('!testStation.endpoint && !testStation.arm.moving');
      await evaluate('document.getElementById("btnTrajEndpoint").focus()');
      await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:" ",code:"Space"});await wait('testStation.endpoint');
      await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:" ",code:"Space"});await wait('!testStation.endpoint');
      await press();await wait('testStation.endpoint');
      await evaluate('testStation.arm.q_deg[0]+=10;emit()');
      await wait('!testStation.endpoint && document.getElementById("btnTrajEndpoint").disabled');await release();
      assert.match(await evaluate('document.getElementById("trajEndpointLabel").textContent'),/已到末点/);
      assert.equal(await evaluate('document.getElementById("btnTrajContinue").disabled'),false);
    });
    await run("continuation and hardware playback send only explicit commands",async()=>{
      await evaluate('click("#btnTrajContinue")');await wait('testStation.arm.recording');
      assert.match(await evaluate('testStation.sent.filter(x=>x.action==="traj_record").at(-1).file'),/现场录制/);
      await evaluate('click("#btnRecStart")');await wait('!testStation.arm.recording && !document.getElementById("btnTrajExecute").disabled');
      await evaluate('click("#btnTrajExecute")');await wait('!document.getElementById("btnTrajStop").hidden');
      assert.equal(await evaluate('testStation.sent.at(-1).rate'),0.5);
      await evaluate('click("#btnTrajStop")');await wait('!testStation.arm.replay_active');
    });
    await run("dirty selection requires confirmation and mobile fits viewport",async()=>{
      await evaluate('edit("#trajIn",1);edit("#trajOut",2)');
      assert.equal(await evaluate('document.getElementById("trajDraftState").dataset.editCount'),"0");
      await evaluate('click("#btnTrajNew")');await wait('document.getElementById("confirmDialog").open');
      await evaluate('click("#confirmCancel")');await wait('!document.getElementById("confirmDialog").open');
      assert.ok(await evaluate('document.querySelectorAll("#trajHistory [data-history-kind=selection]").length')>0);
      await cdp("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
      await delay(100);assert.equal(await evaluate('document.documentElement.scrollWidth<=innerWidth'),true);
      await screenshot("trajectories-mobile");
    });
    await run("old motion services cannot silently process a selected range as the whole trajectory",async()=>{
      await evaluate('edit("#trajEditTool","speed");edit("#trajFactor",2);testStation.editorVersion=1;click("#btnTrajApply")');
      await wait('document.getElementById("trajAck").textContent.includes("旧版") && !document.getElementById("btnTrajApply").disabled');
      assert.match(await evaluate('document.getElementById("trajMeta").textContent'),/8.00 s/);
      assert.deepEqual(await evaluate('[Number(document.getElementById("trajIn").value),Number(document.getElementById("trajOut").value)]'),[1,2]);
      assert.equal(await evaluate('document.getElementById("trajDraftState").dataset.editCount'),"0");
      await evaluate('testStation.editorVersion=3;click("#btnTrajRestoreSaved")');
      await wait('document.getElementById("trajDraftState").dataset.historyCount==="0"');
      await evaluate('sessionStorage.setItem("old-editor","1");window.trajectoryBeforeReload=true');
      await cdp("Page.reload");
      await wait('!window.trajectoryBeforeReload && document.getElementById("trajAck").textContent.includes("旧版")');
      assert.equal(await evaluate('document.getElementById("btnTrajApply").disabled'),true);
      const before=await evaluate('testStation.sent.length');
      await evaluate('click("#btnTrajApply");click("#btnTrajSave")');
      assert.equal(await evaluate('testStation.sent.length'),before);
    });
    assert.deepEqual(await evaluate('testStation.errors'),[]);console.log("trajectory browser tests passed");
  }finally{
    if(closeBrowser)await Promise.race([closeBrowser().catch(()=>{}),delay(2000)]);
    if(socket)socket.close();
    if(process.platform==="win32"&&child.exitCode===null){
      try{execFileSync("taskkill",["/PID",String(child.pid),"/T","/F"],{windowsHide:true,stdio:"ignore",timeout:5000});}
      catch{child.kill();}
    }else child.kill();
    server.close();
    await new Promise(resolve=>{if(child.exitCode!==null)resolve();else child.once("exit",resolve);});
    assert.ok(path.resolve(profile).startsWith(path.resolve(os.tmpdir())+path.sep));assert.ok(path.basename(profile).startsWith("fafu-trajectories-"));
    fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:200});
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;server.close();});
