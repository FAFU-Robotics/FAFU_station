/* File library, non-destructive operation history and a robot-independent player. */
(() => {
  "use strict";
  const $ = id => document.getElementById(id);
  const esc = value => String(value ?? "").replace(/[&<>"']/g, x => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[x]));
  const stem = name => String(name || "").replace(/\.jsonl$/i, "");
  const state = {files:[], file:"", data:null, ops:[], redo:[], savedOps:[], savedRedo:[], arm:{}, program:{}, working:false,
    playing:false, previewSelection:false, selection:{start:0,end:0}, time:0, previous:0, raf:0, generation:0, loaded:false, initialized:false};
  const pending = new Map();
  const PREVIEW_FRAME_MS = 1000 / 30;
  const EDITOR_VERSION = 3;
  const UPDATE_EDITOR = "运动服务仍为旧版，请重启站控加载新版轨迹编辑服务，再刷新页面。";
  let serial = 0, libraryAt = 0, refreshGeneration = 0, libraryMarkup = "", historyMarkup = "";
  let numericSelectionBefore=null,keySelectionBefore=null;
  let endpointHold=null;
  const dirty = () => JSON.stringify(state.ops) !== JSON.stringify(state.savedOps);
  const motionOps = ops => ops.filter(op=>op.type!=="select");
  const dataDirty = () => JSON.stringify(motionOps(state.ops))!==JSON.stringify(motionOps(state.savedOps));
  const editorReady = () => Number(state.data?.editor_version)>=EDITOR_VERSION;
  function requireEditor(data=state.data) {
    if(Number(data?.editor_version)<EDITOR_VERSION||!Number.isFinite(Number(data?.editor_version)))throw new Error(UPDATE_EDITOR);
  }
  const taskActive = () => state.arm.recording || state.arm.replay_active || ["running","paused"].includes(state.program.status);
  const locked = () => state.working || taskActive() || !!endpointHold;
  function endpointPose() {
    const end=state.data?.frames?.at(-1),q=state.arm.feedback_deg||state.arm.q_deg;
    if(!end||!q?.length)return {known:false,reached:false,ready:false};
    const joint=Math.max(...end.pos.map((value,i)=>Math.abs(value*180/Math.PI-q[i])));
    const grip=end.gpos==null?0:state.arm.gripper_deg==null?Infinity:Math.abs(end.gpos*180/Math.PI-state.arm.gripper_deg);
    return {known:true,reached:joint<=0.25,ready:joint<=3&&grip<=3,joint,grip};
  }
  const duration = () => state.data?.duration || 0;
  const visible = () => $("page-trajectories").classList.contains("on");
  const clamp = (value, min, max) => Math.max(min,Math.min(max,value));
  const timeText = value => Number(value).toFixed(3);
  const minimumSpan = () => Math.min(0.001,Math.max(1e-6,duration()/10000));
  function readSelection(strict = true) {
    const start=Number($("trajIn").value),end=Number($("trajOut").value);
    const valid=state.data && $("trajIn").value.trim() && $("trajOut").value.trim() &&
      Number.isFinite(start) && Number.isFinite(end) && start>=0 && start<end && end<=duration()+1e-6;
    if(!valid){if(strict)throw new Error("请在时间轴上选择有效区间，起点必须小于终点。");return state.data?state.selection:{start:0,end:0};}
    return {start,end:Math.min(end,duration())};
  }
  function setSelection(start,end) {
    const total=duration(),gap=minimumSpan();
    start=clamp(start,0,Math.max(0,total-gap));end=clamp(end,start+gap,total);
    state.selection={start,end};
    $("trajIn").value=String(Number(start.toFixed(6)));$("trajOut").value=String(Number(end.toFixed(6)));
    renderSelection();chart();
  }
  function commitSelection(before) {
    // Inputs update the live range before change/blur; compare with the range
    // actually represented by history, including after switching gestures.
    before=state.data?.selection||before;
    const range=readSelection();
    if(Math.abs(range.start-before.start)<1e-6&&Math.abs(range.end-before.end)<1e-6)return;
    if(!editorReady()){setSelection(before.start,before.end);notice(UPDATE_EDITOR,false);return;}
    if(state.ops.length>=200){setSelection(before.start,before.end);notice("每份轨迹最多保留 200 步历史。",false);return;}
    state.ops=[...state.ops,{type:"select",start:range.start,end:range.end}];
    state.redo=[];state.data.selection={...range};paint();
    notice("选区已加入历史："+timeText(range.start)+"–"+timeText(range.end)+" s。");
  }
  function renderSelection() {
    const {start,end}=readSelection(false),total=duration(),has=!!state.data;
    $("trajRange").style.left=(total?start/total*100:0)+"%";
    $("trajRange").style.width=(total?(end-start)/total*100:100)+"%";
    $("trajRangeLabel").textContent=has?timeText(end-start)+" s":"选择轨迹";
    const whole=has&&start<1e-6&&Math.abs(end-total)<1e-6;
    $("trajSelectionSummary").textContent=has?(whole?"已全选 ":"已选 ")+timeText(start)+"–"+timeText(end)+" s · 时长 "+timeText(end-start)+" s":"尚未选择轨迹";
    for(const [id,value,min,max] of [["trajRangeStart",start,0,end-minimumSpan()],["trajRangeEnd",end,start+minimumSpan(),total]]){
      $(id).setAttribute("aria-valuemin",String(Math.max(0,min)));$(id).setAttribute("aria-valuemax",String(Math.max(0,max)));
      $(id).setAttribute("aria-valuenow",String(value));$(id).setAttribute("aria-valuetext",timeText(value)+" 秒");
      $(id).disabled=!editorReady()||locked();
    }
    $("trajRange").tabIndex=editorReady()&&!locked()?0:-1;
    scopeHint();
  }
  function scopeHint() {
    const kind=$("trajEditTool").value,merge=kind==="merge";
    const selection=readSelection(false);
    const whole=state.data&&selection.start<1e-6&&Math.abs(selection.end-duration())<1e-6;
    $("btnTrajApply").textContent=merge?"追加到末尾":whole?"应用到全选轨迹":"应用到选区";
    const range=timeText(selection.start)+"–"+timeText(selection.end)+" s "+(whole?"（当前已全选整条轨迹）":"选区");
    const hints={idle:"移除范围内的持续静止，保留夹爪动作；后续时间顺延。",smooth:"只平滑范围内的关节角，保留区间两端和夹爪动作。",speed:"改变范围内的动作时长并重算速度，后续片段整体移动。",resample:"只调整范围内的采样密度，保持起终点和总时长。",reverse:"反转范围内的关节及夹爪动作；局部反向会添加衔接过渡，后续时间顺延。",merge:"追加到整条轨迹末尾，使用过渡连接两端。"};
    $("trajToolHint").textContent=(merge?"处理范围：轨迹末尾。":"处理范围："+range+"。")+hints[kind];
  }
  const notice = (message, ok = true) => {
    $("trajAck").textContent = message;
    $("trajAck").className = "hint" + (ok ? "" : " error");
    if (!ok) window.StationUI.notice(message, false);
  };
  function request(action, data = {}) {
    return new Promise((resolve, reject) => {
      const request_id = "traj-" + (++serial);
      const timer = setTimeout(() => {pending.delete(request_id); reject(new Error("轨迹请求超时，请检查站控连接"));}, 15000);
      pending.set(request_id, {resolve,reject,timer});
      try { window.StationUI.sendImmediate({t:"workbench", action, request_id, ...data}); }
      catch (error) {clearTimeout(timer);pending.delete(request_id);reject(error);}
    });
  }
  window.addEventListener("station:ack", event => {
    const msg = event.detail, task = pending.get(msg.request_id);
    if (!task) return;
    clearTimeout(task.timer); pending.delete(msg.request_id);
    if (msg.ok) task.resolve(msg.data || {}); else task.reject(new Error(msg.error || "操作失败"));
  });
  const handle = fn => async event => {
    if (state.working) return;
    state.working = true; paint();
    try { await fn(event); } catch (error) {notice(error.message || String(error), false);}
    finally {state.working = false;paint();}
  };
  async function discard() {
    return !dirty() || await window.StationDialog.confirm({title:"放弃当前编辑？",message:"当前轨迹草稿尚未保存。继续将放弃这些编辑，原始文件保留。",confirmText:"放弃编辑",destructive:true});
  }
  function library() {
    const query = $("trajSearch").value.trim().toLowerCase();
    const files = state.files.filter(x => x.name.toLowerCase().includes(query));
    const liveName = state.arm.recording ? state.arm.traj_file : "";
    if (liveName && !files.some(x => x.name === liveName) && liveName.toLowerCase().includes(query)) {
      files.unshift({name:liveName,frames:state.arm.rec_frames,s:state.arm.rec_s,teach:state.arm.rec_teach});
    }
    const markup = files.length ? files.map(file => {
      const live = file.name === liveName;
      return "<button type='button' role='option' class='traj-file" + (file.name === state.file ? " selected" : "") + (live ? " recording" : "") + "' aria-selected='" + (file.name === state.file) + "' data-file='" + esc(file.name) + "'" + (locked() ? " disabled" : "") + "><b>" + esc(stem(file.name)) + "</b><small" + (live ? " class='live'" : "") + ">" + (live ? "● 正在录制 · " : "") + Number(live ? state.arm.rec_s : file.s).toFixed(1) + " s · " + Number(live ? state.arm.rec_frames : file.frames) + " 帧</small><small>" + (file.teach === "drag" ? "手动拖臂" : "界面操控") + (file.mtime ? " · " + new Date(file.mtime*1000).toLocaleDateString() : "") + "</small></button>";
    }).join("") : "<div class='traj-library-empty'>" + (query ? "没有匹配的轨迹" : "还没有轨迹文件<br>点击「新建录制」采集第一个动作") + "</div>";
    if (markup !== libraryMarkup) {$("trajLibrary").innerHTML=markup;libraryMarkup=markup;}
  }
  async function refresh(prefer) {
    const generation = ++refreshGeneration;
    const response = await fetch("/api/traj");
    if (!response.ok) throw new Error("无法读取轨迹库");
    const body = await response.json();
    if (generation !== refreshGeneration) return;
    state.files = body.files || []; state.loaded = true; library(); toolOptions();
    const wanted = prefer || state.file || state.files.find(x => x.frames >= 2)?.name;
    if (wanted && !taskActive() && window.StationUI.connected() && (!state.data || (prefer && prefer !== state.file))) {
      await select(wanted, false);
    }
    if (window.StationUI.connected()) state.initialized = true;
    paint();
  }
  async function select(file, confirm = true) {
    if (file === state.file && state.data) return;
    if (confirm && !await discard()) return;
    pause();
    const generation = ++state.generation;
    const data = await request("traj_process", {file});
    if (generation !== state.generation) return;
    state.file = file; state.ops = data.operations || []; state.redo = (data.future || []).slice().reverse();
    state.savedOps = JSON.parse(JSON.stringify(state.ops));
    state.savedRedo = JSON.parse(JSON.stringify(state.redo));
    $("trajLibraryName").value = stem(file);
    $("trajSaveName").value = stem(file) + "_编辑";
    setData(data); notice(data.history_warning || "已载入轨迹，可在 3D 中预览或编辑。", !data.history_warning);
    if(!editorReady())notice(UPDATE_EDITOR,false);
  }
  let viewer = null;
  function initViewer() {
    if (!visible() || viewer || !window.Arm3D?.create) return;
    viewer = window.Arm3D.create("traj3dHost", "traj3dOverlay");
    viewer.setActive(true);
    if (state.data) viewer.setPath(state.data.frames);
    // Data and recording feedback can arrive before the workspace is shown.
    if (state.arm.gripper_limits?.length===2) viewer.setGripRange(...state.arm.gripper_limits);
    if (state.arm.recording) viewer.setPose(state.arm.q_deg,state.arm.gripper_deg,state.arm.gripper_open);
    else seek(state.time);
  }
  function setData(data) {
    numericSelectionBefore=null;keySelectionBefore=null;
    pause();state.data = data;state.time = 0;
    const selection=data.selection||{start:0,end:duration()};
    setSelection(selection.start,selection.end);
    $("trajRuler").innerHTML=Array.from({length:6},(_,i)=>"<span style='left:"+(i*20)+"%'>"+timeText(duration()*i/5)+" s</span>").join("");
    $("trajScrub").max = String(duration());$("trajScrub").value = "0";
    if (viewer) viewer.setPath(data.frames);
    seek(0);paint();
  }
  function showRecording(name) {
    pause(); state.file=name;state.data=null;state.ops=[];state.redo=[];state.savedOps=[];
    viewer?.setPath([]);seek(0);
  }
  function sample(t) {
    const frames = state.data?.frames || [];
    if (!frames.length) return null;
    let lo=0,hi=frames.length-1;
    while (lo+1<hi) {const mid=(lo+hi)>>1;if(frames[mid].t<=t)lo=mid;else hi=mid;}
    const a=frames[lo],b=frames[hi],u=Math.max(0,Math.min(1,(t-a.t)/(b.t-a.t||1)));
    return {pos:a.pos.map((q,i)=>q+(b.pos[i]-q)*u),gpos:a.gpos==null?b.gpos:a.gpos+((b.gpos??a.gpos)-a.gpos)*u};
  }
  function seek(t) {
    state.time = Math.max(0,Math.min(duration(),t));
    $("trajScrub").value = String(state.time);
    $("trajClock").textContent = state.time.toFixed(2) + " / " + duration().toFixed(2) + " s";
    $("trajTimelinePlayhead").style.left=(duration()?state.time/duration()*100:0)+"%";
    const pose=sample(state.time);
    if (viewer && pose) viewer.setPose(pose.pos.map(q=>q*180/Math.PI),pose.gpos==null?null:pose.gpos*180/Math.PI);
    chart();
  }
  function pause() {
    state.playing=false;state.previewSelection=false;cancelAnimationFrame(state.raf);state.raf=0;state.previous=0;
    $("btnTrajPreview").textContent="▶ 预览";
    $("btnTrajPreviewSelection").textContent="▶ 预览选区";
  }
  function playPreview(selectionOnly=false) {
    if(state.playing){pause();paint();return;}
    if(!state.data||state.working||state.arm.recording)return;
    const bounds=selectionOnly?readSelection():{start:0,end:duration()};
    if(state.time<bounds.start||state.time>=bounds.end)seek(bounds.start);
    state.previewSelection=selectionOnly;state.playing=true;state.previous=0;
    $("btnTrajPreview").textContent="Ⅱ 暂停";
    if(selectionOnly)$("btnTrajPreviewSelection").textContent="Ⅱ 暂停选区";
    state.raf=requestAnimationFrame(frame);paint();
  }
  function frame(now) {
    if (!state.playing || !visible() || document.hidden) {pause();return;}
    if (state.previous && now-state.previous < PREVIEW_FRAME_MS-0.1) {
      state.raf=requestAnimationFrame(frame);return;
    }
    const dt=state.previous?Math.min(0.1,(now-state.previous)/1000):0;state.previous=now;
    let time=state.time+dt*Number($("trajPreviewRate").value);
    const bounds=state.previewSelection?readSelection(false):{start:0,end:duration()};
    if (time>=bounds.end) {if($("trajLoop").checked)time=bounds.start;else{seek(bounds.end);pause();return;}}
    seek(time);state.raf=requestAnimationFrame(frame);
  }
  let chartCache = null, chartPalette = null;
  function plotColors() {
    if (!chartPalette) {
      const style=getComputedStyle(document.documentElement),color=name=>style.getPropertyValue(name).trim();
      chartPalette={muted:color("--muted"),line:color("--line"),accent:color("--cyan"),
        mask:color("--plot-mask"),selection:color("--plot-selection"),playhead:color("--playhead"),
        joints:Array.from({length:6},(_,i)=>color("--joint-"+(i+1)))};
    }
    return chartPalette;
  }
  function chart() {
    if (!visible() || document.hidden) return;
    const canvas=$("trajChart"),ctx=canvas.getContext("2d");
    const width=canvas.clientWidth,height=canvas.clientHeight;
    if (!ctx || width<1) return;
    const dpr=Math.min(devicePixelRatio||1,2);
    if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);}
    ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,canvas.width,canvas.height);
    const frames=state.data?.frames || [],end=duration();
    if(!frames.length || !end){chartCache=null;return;}
    const {start,end:finish}=readSelection(false);
    const colors=plotColors();
    const key=[width,height,dpr].join(":");
    const x=t=>28+(width-36)*t/end;
    if(!chartCache || chartCache.data!==state.data || chartCache.key!==key){
      const image=chartCache?.image || document.createElement("canvas");
      image.width=canvas.width;image.height=canvas.height;
      const layer=image.getContext("2d");layer.setTransform(dpr,0,0,dpr,0,0);
      let low=Infinity,high=-Infinity;
      frames.forEach(row=>row.pos.forEach(q=>{low=Math.min(low,q);high=Math.max(high,q);}));
      const pad=Math.max(0.1,(high-low)*0.1);low-=pad;high+=pad;
      const y=q=>8+(height-20)*(1-(q-low)/(high-low));
      layer.font="9px Consolas";layer.fillStyle=colors.muted;
      for(let i=0;i<=2;i++){const q=low+(high-low)*i/2;layer.fillText(Math.round(q*180/Math.PI)+"°",1,y(q));layer.strokeStyle=colors.line;layer.beginPath();layer.moveTo(28,y(q));layer.lineTo(width,y(q));layer.stroke();}
      colors.joints.forEach((color,j)=>{
        layer.strokeStyle=color;layer.lineWidth=1;layer.beginPath();frames.forEach((row,i)=>{if(i)layer.lineTo(x(row.t),y(row.pos[j]));else layer.moveTo(x(row.t),y(row.pos[j]));});layer.stroke();
      });
      chartCache={data:state.data,key,image};
    }
    ctx.drawImage(chartCache.image,0,0);ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.fillStyle=colors.mask;ctx.fillRect(28,0,x(start)-28,height);ctx.fillRect(x(finish),0,width-8-x(finish),height);
    ctx.fillStyle=colors.selection;ctx.fillRect(x(start),0,x(finish)-x(start),height);
    ctx.strokeStyle=colors.accent;ctx.beginPath();ctx.moveTo(x(start),0);ctx.lineTo(x(start),height);ctx.moveTo(x(finish),0);ctx.lineTo(x(finish),height);ctx.stroke();
    ctx.strokeStyle=colors.playhead;ctx.beginPath();ctx.moveTo(x(state.time),0);ctx.lineTo(x(state.time),height);ctx.stroke();
  }
  function toolOptions() {
    const kind=$("trajEditTool").value;
    const field=(label,id,value,step,min,max)=>"<label>"+label+"<input type='number' id='"+id+"' value='"+value+"' step='"+step+"' min='"+min+"' max='"+max+"'/></label>";
    const options={
      idle:field("角度阈值 / °","trajThreshold",0.5,0.1,0.1,10)+field("最短静止 / s","trajMinimum",0.5,0.1,0.1,10),
      smooth:field("窗口 / 帧","trajWindow",9,2,3,101),
      speed:field("动作倍率","trajFactor",1,0.1,0.1,2),
      resample:field("采样率 / Hz","trajHz",100,10,10,100),
      reverse:field("衔接过渡 / s","trajReverseTransition",1,0.1,0.2,30),
      merge:"<label>追加轨迹<select id='trajMergeFile'><option value=''>请选择</option>"+state.files.filter(x=>x.frames>=2).map(x=>"<option value='"+esc(x.name)+"'>"+esc(stem(x.name))+"</option>").join("")+"</select></label>"+field("过渡 / s","trajTransition",1,0.1,0.2,30)
    };
    $("trajToolOptions").innerHTML=options[kind];
    scopeHint();
  }
  const describe=op=>{
    const text={select:"选区 "+timeText(op.start)+"–"+timeText(op.end)+" s",trim:"保留 "+timeText(op.start)+"–"+timeText(op.end)+" s",delete:"删除 "+timeText(op.start)+"–"+timeText(op.end)+" s · 1 s 过渡",idle:"去除静止 · "+op.threshold+"° / "+op.minimum+" s",smooth:"关节平滑 · "+op.window+" 帧",speed:"动作倍率 · "+op.factor+"×",resample:"重采样 · "+op.hz+" Hz",reverse:"反向轨迹",merge:"追加 "+stem(op.file)+" 到末尾 · "+op.transition+" s 过渡"}[op.type];
    return text+(["select","trim","delete","merge"].includes(op.type)?"":op.scope==="selection"?" · 选区 "+timeText(op.start)+"–"+timeText(op.end)+" s":" · 整条轨迹");
  };
  async function apply(ops,redo=[]) {
    requireEditor();
    const data=await request("traj_process",{file:state.file,operations:ops});
    requireEditor(data);
    state.ops=ops;state.redo=redo;setData(data);notice("草稿已更新，先预览，再保存。");
  }
  function paint() {
    const has=!!state.data,busy=locked(),arm=state.arm,recording=!!arm.recording;
    $("trajActivity").textContent=recording?"● 录制中":arm.replay_active?"执行中":state.working?"处理中…":state.playing?"3D 预览中":"就绪";
    $("trajActivity").classList.toggle("recording",recording);
    $("trajSelectedTitle").textContent=recording?stem(arm.traj_file)+" · 正在录制":has?stem(state.file)+(dataDirty()?" · 编辑草稿":dirty()?" · 选区历史":""):"选择轨迹，开始预览";
    $("trajMeta").textContent=recording?"显示当前机械臂姿态 · "+Number(arm.rec_s||0).toFixed(1)+" s · "+(arm.rec_frames||0)+" 帧":has?duration().toFixed(2)+" s · "+state.data.count+" 帧 · 峰值 "+state.data.max_speed_deg_s.toFixed(1)+"°/s"+(state.data.preview_stride>1?" · 预览已抽样，保存保留全部帧":""):"3D 预览仅在屏幕中播放";
    $("trajEmpty").hidden=has||recording;
    $("trajDraftState").textContent=dirty()?state.ops.length+" 步历史 · 未保存":state.ops.length?"已保存 · "+state.ops.length+" 步历史":"原始文件";
    $("trajDraftState").dataset.editCount=String(motionOps(state.ops).length);
    $("trajDraftState").dataset.selectionCount=String(state.ops.length-motionOps(state.ops).length);
    $("trajDraftState").dataset.historyCount=String(state.ops.length);
    const timeline=[...state.ops,...state.redo.slice().reverse()];
    const historyRow=(index,label,kind)=>"<li data-history-kind='"+kind+"' class='"+(index>state.ops.length?"undone":"")+"'><button type='button' data-traj-restore='"+index+"'"+(busy||!editorReady()||index===state.ops.length?" disabled":"")+(index===state.ops.length?" aria-current='step'":"")+"><span>"+index+". "+esc(label)+"</span><small>"+(index===state.ops.length?"当前版本":index>state.ops.length?"已撤销 · 可恢复":"恢复至此")+"</small></button></li>";
    const renderedHistory=has?historyRow(0,"原始录制","initial")+timeline.map((op,i)=>historyRow(i+1,describe(op),op.type==="select"?"selection":"edit")).join(""):"";
    if(renderedHistory!==historyMarkup){$("trajHistory").innerHTML=renderedHistory;historyMarkup=renderedHistory;}
    $("btnRecStart").textContent=recording?"■ 停止并保存":"● 开始录制";
    $("btnRecStart").classList.toggle("recording",recording);
    $("btnRecStart").disabled=state.working||(!recording&&taskActive());
    $("trajRecordName").disabled=busy;$("trajInputMode").disabled=busy;
    $("trajRecordStatus").textContent=recording?"正在录制 · "+Number(arm.rec_s||0).toFixed(1)+" s · "+(arm.rec_frames||0)+" 帧 · "+(arm.traj_file||""):"准备好后开始，停止时自动保存。";
    $("trajInputHint").textContent=$("trajInputMode").value==="drag"?"开始后进入重力补偿，请扶稳机械臂，再用手引导动作。":"使用手动控制中的关节或笛卡尔操作录制。换页后会继续采集。";
    $("trajExecutionHint").textContent=dataDirty()?"当前显示编辑草稿，请先保存后执行。":"执行整条已保存文件："+(state.file||"请先选择轨迹");
    ["btnRecRename","btnRecDelete","btnTrajCopy","trajLibraryName","btnTrajEndpoint","btnTrajContinue","btnTrajExport"].forEach(id=>$(id).disabled=!has||busy||dataDirty());
    const endpoint=endpointPose(),holdSupported=state.data?.endpoint_hold===true;
    $("btnTrajEndpoint").disabled=!has||dataDirty()||state.working||taskActive()||!holdSupported||!arm.online||!arm.enabled||endpoint.reached;
    $("btnTrajEndpoint").classList.toggle("reached",endpoint.reached);
    $("btnTrajEndpoint").setAttribute("aria-pressed",String(!!endpointHold));
    $("trajEndpointLabel").textContent=endpoint.reached?"关节已到末点":endpointHold?"正在移动 · 松开停止":"按住移动到末点";
    $("btnTrajContinue").disabled=!has||busy||dataDirty()||!arm.online||!arm.enabled||!endpoint.ready;
    $("trajEndpointStatus").textContent=!has?"选择轨迹后，可将机械臂移动到末点。":!holdSupported?"运动服务需重启，才能使用按住移动。":dataDirty()?"先保存当前编辑，再移动到末点。":!arm.online||!arm.enabled?"请先连接并使能机械臂。":endpointHold?"机械臂正在运动；松开、移出按钮或切换窗口即停止。":endpoint.ready?"已接近末点，可以继续录制轨迹。":endpoint.reached?"关节已到末点，请在手动控制中对齐夹爪。":"按住时机械臂以 10°/s 运动，松开立即停止。";
    ["btnTrajTrim","btnTrajCut","btnTrajApply","btnTrajSave","trajIn","trajOut","btnTrajMarkIn","btnTrajMarkOut","trajEditTool","trajSaveName"].forEach(id=>$(id).disabled=!editorReady()||busy);
    $("btnTrajUndo").disabled=!editorReady()||!state.ops.length||busy;$("btnTrajRedo").disabled=!editorReady()||!state.redo.length||busy;
    $("btnTrajRestoreSaved").disabled=!editorReady()||!dirty()||busy;
    $("btnTrajExecute").disabled=!has||busy||dataDirty();
    $("btnTrajExecute").hidden=!!arm.replay_active;$("btnTrajStop").hidden=!arm.replay_active;
    $("btnTrajStop").disabled=state.working;
    $("btnTrajPreview").disabled=!has||state.working||recording;$("btnTrajRewind").disabled=!has;
    $("trajScrub").disabled=!has||recording;
    $("btnTrajNew").disabled=busy;$("btnTrajRefresh").disabled=state.working;
    $("btnTrajSelectAll").disabled=!editorReady()||busy;
    $("btnTrajPreviewSelection").disabled=!has||busy;
    renderSelection();
    library();
  }
  $("trajLibrary").onclick=handle(async event=>{const button=event.target.closest("[data-file]");if(button)await select(button.dataset.file);});
  $("trajSearch").oninput=library;
  $("btnTrajRefresh").onclick=handle(()=>refresh());
  $("btnTrajNew").onclick=handle(async()=>{if(!await discard())return;state.initialized=true;state.ops=[];state.redo=[];state.savedOps=[];state.file="";state.data=null;pause();viewer?.setPath([]);$("trajLibraryName").value="";$("trajRecordName").value="";$("trajRecordKind").textContent="新建录制";$("trajRecordName").focus();seek(0);notice("填写名称和输入方式，然后开始录制。");});
  $("btnRecStart").onclick=handle(async()=>{
    pause();
    if(state.arm.recording){const data=await request("traj_record_stop");state.arm={...state.arm,recording:false};state.data=null;await refresh(data.name);notice("录制已保存。");}
    else {if(!await discard())return;window.StationUI.releaseManual();const data=await request("traj_record",{name:$("trajRecordName").value,teach:$("trajInputMode").value});state.arm={...state.arm,recording:true,traj_file:data.name,rec_frames:0,rec_s:0};showRecording(data.name);$("trajRecordKind").textContent="新建录制";notice("开始录制，停止时自动保存。");refresh().catch(error=>notice(error.message,false));}
  });
  $("trajInputMode").onchange=paint;
  $("btnRecRename").onclick=handle(async()=>{const name=$("trajLibraryName").value.trim();if(!name)throw new Error("请输入文件名");const data=await request("traj_rename",{file:state.file,name});state.file=data.name;await refresh();notice("轨迹已重命名。");});
  $("btnRecDelete").onclick=handle(async()=>{if(!await window.StationDialog.confirm({title:"删除轨迹？",message:"将永久删除「"+state.file+"」。引用它的程序也将无法回放该文件。",confirmText:"删除文件",destructive:true}))return;await request("traj_delete",{file:state.file});state.file="";state.data=null;state.ops=[];state.redo=[];state.savedOps=[];pause();viewer?.setPath([]);seek(0);await refresh();notice("轨迹已删除。");});
  $("btnTrajCopy").onclick=handle(async()=>{requireEditor();const data=await request("traj_process",{file:state.file,operations:state.ops,future:state.redo.slice().reverse(),save:true,name:stem(state.file)+"_副本"});requireEditor(data);state.data=null;await refresh(data.name);notice("已创建完整副本。");});
  $("btnTrajContinue").onclick=handle(async()=>{pause();window.StationUI.releaseManual();const data=await request("traj_record",{file:state.file,teach:$("trajInputMode").value,name:stem(state.file)+"_续录"});state.arm={...state.arm,recording:true,traj_file:data.name};showRecording(data.name);$("trajRecordKind").textContent="从末尾续录";notice("正在续录，原始轨迹保留。");refresh().catch(error=>notice(error.message,false));});
  function stopEndpoint() {
    const hold=endpointHold;
    if(!hold)return;
    endpointHold=null;clearInterval(hold.timer);paint();
    request("traj_endpoint_stop",{hold_id:hold.id}).catch(error=>notice(error.message,false));
  }
  function startEndpoint(event,key=null) {
    const button=$("btnTrajEndpoint");
    if(button.disabled||endpointHold)return;
    event.preventDefault();pause();window.StationUI.releaseManual();
    const hold={id:"endpoint-"+Date.now()+"-"+(++serial),key,pointer:key?null:event.pointerId,timer:0,ready:false,pending:false};
    endpointHold=hold;
    if(!key)button.setPointerCapture(event.pointerId);
    request("traj_endpoint",{file:state.file,hold_id:hold.id}).then(data=>{
      if(endpointHold!==hold){request("traj_endpoint_stop",{hold_id:hold.id}).catch(()=>{});return;}
      if(!data.active){stopEndpoint();return;}
      hold.ready=true;
    }).catch(error=>{if(endpointHold===hold){stopEndpoint();notice(error.message,false);}});
    hold.timer=setInterval(()=>{
      if(endpointHold!==hold||!hold.ready||hold.pending)return;
      hold.pending=true;
      request("traj_endpoint_touch",{hold_id:hold.id}).then(data=>{if(!data.active&&endpointHold===hold)stopEndpoint();}).catch(error=>{if(endpointHold===hold){stopEndpoint();notice(error.message,false);}}).finally(()=>{hold.pending=false;});
    },100);
    paint();
  }
  const endpointButton=$("btnTrajEndpoint");
  endpointButton.onclick=event=>event.preventDefault();
  endpointButton.onpointerdown=event=>{if(event.button===0)startEndpoint(event);};
  endpointButton.onpointermove=event=>{
    if(!endpointHold||endpointHold.pointer!==event.pointerId)return;
    const r=endpointButton.getBoundingClientRect();
    if(event.clientX<r.left||event.clientX>r.right||event.clientY<r.top||event.clientY>r.bottom)stopEndpoint();
  };
  endpointButton.onlostpointercapture=event=>{if(endpointHold?.pointer===event.pointerId)stopEndpoint();};
  endpointButton.onkeydown=event=>{if(["Space","Enter"].includes(event.code)){event.preventDefault();if(!event.repeat)startEndpoint(event,event.code);}};
  window.addEventListener("keyup",event=>{if(endpointHold?.key===event.code)stopEndpoint();});
  for(const type of ["pointerup","pointercancel"])window.addEventListener(type,event=>{if(endpointHold?.pointer===event.pointerId)stopEndpoint();});
  window.addEventListener("blur",stopEndpoint);
  window.addEventListener("pagehide",stopEndpoint);
  window.addEventListener("hashchange",stopEndpoint);
  document.addEventListener("visibilitychange",()=>{if(document.hidden)stopEndpoint();});
  window.addEventListener("beforeunload",stopEndpoint);
  $("btnTrajExecute").onclick=handle(async()=>{pause();window.StationUI.releaseManual();await request("traj_execute",{file:state.file,rate:Number($("trajRobotRate").value),speed:Number($("trajApproachSpeed").value)});state.arm={...state.arm,replay_active:true};notice("正在执行已保存轨迹。");});
  $("btnTrajStop").onclick=handle(async()=>{await request("stop");state.arm={...state.arm,replay_active:false};notice("机械臂执行已停止。");});
  $("btnTrajPreview").onclick=()=>playPreview();
  $("btnTrajPreviewSelection").onclick=()=>{try{playPreview(true);}catch(error){notice(error.message,false);}};
  $("btnTrajRewind").onclick=()=>{pause();seek(0);paint();};
  $("trajScrub").oninput=()=>{pause();seek(Number($("trajScrub").value));};
  $("btnTrajFit").onclick=()=>viewer?.fitCamera();
  $("btnTrajSelectAll").onclick=()=>{pause();const before=readSelection(false);setSelection(0,duration());commitSelection(before);};
  $("btnTrajMarkIn").onclick=()=>{pause();const before=readSelection(false);setSelection(state.time,Math.max(before.end,state.time+minimumSpan()));commitSelection(before);};
  $("btnTrajMarkOut").onclick=()=>{pause();const before=readSelection(false);setSelection(Math.min(before.start,state.time-minimumSpan()),state.time);commitSelection(before);};
  for(const id of ["trajIn","trajOut"]){
    $(id).onfocus=()=>{numericSelectionBefore??={...state.selection};};
    $(id).onblur=()=>{numericSelectionBefore=null;};
    $(id).oninput=()=>{pause();numericSelectionBefore??={...state.selection};try{state.selection=readSelection();renderSelection();chart();}catch(error){$("trajSelectionSummary").textContent=error.message;}};
    $(id).onchange=()=>{try{const before=numericSelectionBefore||{...state.selection},range=readSelection();setSelection(range.start,range.end);commitSelection(before);numericSelectionBefore=null;}catch(error){notice(error.message,false);}};
  }
  $("trajEditTool").onchange=toolOptions;
  const selectedEdit = build => async event=>{
    if(state.working)return;
    try{
      requireEditor();
      const op=build(); // Capture the chosen interval before repainting or awaiting an RPC.
      if(op.type!=="merge"){
        const before={...state.selection};setSelection(op.start,op.end);commitSelection(before);
      }
      const operations=[...state.ops,op];
      await handle(()=>apply(operations))(event);
    }catch(error){notice(error.message,false);}
  };
  ["Trim","Cut"].forEach(kind=>$("btnTraj"+kind).onclick=selectedEdit(()=>({type:kind==="Trim"?"trim":"delete",...readSelection(),transition:1})));
  $("btnTrajApply").onclick=selectedEdit(()=>{
    const type=$("trajEditTool").value,scope=type==="merge"?"all":"selection";
    const values={idle:()=>({threshold:Number($("trajThreshold").value),minimum:Number($("trajMinimum").value)}),smooth:()=>({window:Number($("trajWindow").value)}),speed:()=>({factor:Number($("trajFactor").value)}),resample:()=>({hz:Number($("trajHz").value)}),reverse:()=>({transition:Number($("trajReverseTransition").value)}),merge:()=>({file:$("trajMergeFile").value,transition:Number($("trajTransition").value)})};
    return {type,scope,...(scope==="selection"?readSelection():{}),...values[type]()};
  });
  $("btnTrajUndo").onclick=handle(()=>apply(state.ops.slice(0,-1),[...state.redo,state.ops.at(-1)]));
  $("btnTrajRedo").onclick=handle(()=>apply([...state.ops,state.redo.at(-1)],state.redo.slice(0,-1)));
  $("trajHistory").onclick=handle(event=>{const button=event.target.closest("[data-traj-restore]");if(!button)return;const timeline=[...state.ops,...state.redo.slice().reverse()],index=Number(button.dataset.trajRestore);return apply(timeline.slice(0,index),timeline.slice(index).reverse());});
  $("btnTrajRestoreSaved").onclick=handle(()=>apply(JSON.parse(JSON.stringify(state.savedOps)),JSON.parse(JSON.stringify(state.savedRedo))));
  $("btnTrajSave").onclick=handle(async()=>{requireEditor();const data=await request("traj_process",{file:state.file,operations:state.ops,future:state.redo.slice().reverse(),save:true,name:$("trajSaveName").value.trim()});requireEditor(data);state.ops=[];state.redo=[];state.savedOps=[];state.data=null;await refresh(data.name);notice("已保存新轨迹及编辑历史，重新打开后仍可撤销或恢复。");});
  $("btnTrajExport").onclick=()=>{const link=document.createElement("a");link.href="/api/traj/download?name="+encodeURIComponent(state.file);link.download=state.file;link.click();};
  let selectionDrag=null;
  function pointerTime(event,surface) {
    const box=surface.getBoundingClientRect(),plot=surface.id==="trajChart";
    return clamp((event.clientX-box.left-(plot?28:0))/Math.max(1,box.width-(plot?36:0)),0,1)*duration();
  }
  function beginSelection(event) {
    if(!editorReady()||locked()||event.button!==0)return;
    pause();
    const surface=event.currentTarget,range=readSelection(false),edge=event.target.closest(".traj-range-handle");
    const mode=edge?(edge.id==="trajRangeStart"?"start":"end"):
      surface.id==="trajTimelineTrack"&&event.target.closest("#trajRange")&&!event.shiftKey?"move":"create";
    selectionDrag={surface,pointer:event.pointerId,mode,anchor:pointerTime(event,surface),start:range.start,end:range.end,x:event.clientX,moved:false};
    if(edge)edge.focus({preventScroll:true});
    surface.setPointerCapture(event.pointerId);event.preventDefault();
  }
  function moveSelection(event) {
    const drag=selectionDrag;
    if(!drag||drag.pointer!==event.pointerId)return;
    if(!drag.moved&&Math.abs(event.clientX-drag.x)<3)return;
    drag.moved=true;
    const time=pointerTime(event,drag.surface),gap=minimumSpan();
    if(drag.mode==="start")setSelection(Math.min(time,drag.end-gap),drag.end);
    else if(drag.mode==="end")setSelection(drag.start,Math.max(time,drag.start+gap));
    else if(drag.mode==="move"){
      const delta=clamp(time-drag.anchor,-drag.start,duration()-drag.end);
      setSelection(drag.start+delta,drag.end+delta);
    }else setSelection(Math.min(drag.anchor,time),Math.max(drag.anchor,time));
    seek(drag.mode==="end"?readSelection(false).end:readSelection(false).start);
  }
  function endSelection(event) {
    const drag=selectionDrag;
    if(!drag||drag.pointer!==event.pointerId)return;
    if(event.type==="pointercancel")setSelection(drag.start,drag.end);
    else if(!drag.moved)seek(pointerTime(event,drag.surface));
    else commitSelection({start:drag.start,end:drag.end});
    selectionDrag=null;
    try{drag.surface.releasePointerCapture(event.pointerId);}catch{}
    paint();
  }
  for(const id of ["trajChart","trajTimelineTrack"]){
    $(id).addEventListener("pointerdown",beginSelection);
    $(id).addEventListener("pointermove",moveSelection);
    $(id).addEventListener("pointerup",endSelection);
    $(id).addEventListener("pointercancel",endSelection);
  }
  for(const id of ["trajRangeStart","trajRangeEnd","trajRange"]){
    $(id).addEventListener("keydown",event=>{
      if(event.target!==$(id)||locked()||!editorReady()||!["ArrowLeft","ArrowRight","Home","End"].includes(event.key))return;
      event.preventDefault();pause();
      keySelectionBefore??={...state.selection};
      const range=readSelection(false),step=Math.max(0.001,Math.min(0.1,duration()/Math.max(1,state.data.count-1)))*(event.shiftKey?10:1);
      const delta=(event.key==="ArrowLeft"?-1:1)*step,gap=minimumSpan();
      if(id==="trajRangeStart")setSelection(event.key==="Home"?0:event.key==="End"?range.end-gap:clamp(range.start+delta,0,range.end-gap),range.end);
      else if(id==="trajRangeEnd")setSelection(range.start,event.key==="End"?duration():event.key==="Home"?range.start+gap:clamp(range.end+delta,range.start+gap,duration()));
      else {const shift=event.key==="Home"?-range.start:event.key==="End"?duration()-range.end:clamp(delta,-range.start,duration()-range.end);setSelection(range.start+shift,range.end+shift);}
    });
  }
  window.addEventListener("keyup",event=>{
    if(keySelectionBefore&&["ArrowLeft","ArrowRight","Home","End"].includes(event.key)){
      const before=keySelectionBefore;keySelectionBefore=null;commitSelection(before);
    }
  });
  window.addEventListener("keydown",event=>{
    if(!visible()||locked()||!editorReady()||event.altKey||event.target.closest?.("input,select,textarea,[contenteditable]"))return;
    if(event.ctrlKey||event.metaKey){
      if(event.code==="KeyZ"||event.code==="KeyY"){
        event.preventDefault();$(event.code==="KeyY"||event.shiftKey?"btnTrajRedo":"btnTrajUndo").click();
      }
      return;
    }
    if(event.code==="KeyI"||event.code==="KeyO"){event.preventDefault();$(event.code==="KeyI"?"btnTrajMarkIn":"btnTrajMarkOut").click();}
  });
  window.addEventListener("station:snapshot",event=>{
    const wasRecording=state.arm.recording,wasPlaying=state.arm.replay_active;
    state.arm=event.detail.arm||{};state.program=event.detail.program||{};
    // HUD data may lag the start ACK across the motion/web bridge. Lease
    // status comes from its own heartbeat; pose/connection failures stop here.
    if(endpointHold?.ready&&(endpointPose().reached||!state.arm.online||!state.arm.enabled))stopEndpoint();
    if(!wasRecording&&state.arm.recording)showRecording(state.arm.traj_file);
    if(viewer&&state.arm.gripper_limits?.length===2)viewer.setGripRange(...state.arm.gripper_limits);
    if(state.arm.recording){pause();if(viewer&&visible())viewer.setPose(state.arm.q_deg,state.arm.gripper_deg,state.arm.gripper_open);}
    if((wasRecording&&!state.arm.recording)||(wasPlaying&&!state.arm.replay_active)) {
      if(wasRecording){state.data=null;state.ops=[];state.redo=[];state.savedOps=[];}
      refresh(wasRecording?state.arm.traj_file:undefined).catch(error=>notice(error.message,false));
    }
    if(visible()&&performance.now()-libraryAt>200){libraryAt=performance.now();paint();}
    if((!state.loaded||!state.initialized)&&visible()&&!state.working)handle(()=>refresh())();
  });
  window.addEventListener("station:trajectories",event=>{if(!state.working)refresh(event.detail?.prefer).catch(error=>notice(error.message,false));});
  window.addEventListener("hashchange",()=>{initViewer();viewer?.setActive(visible());if(!visible())pause();else{viewer?.resize();chart();if(!state.loaded)refresh().catch(error=>notice(error.message,false));}});
  window.addEventListener("station:theme",()=>{chartCache=null;chartPalette=null;chart();});
  document.addEventListener("visibilitychange",()=>{if(document.hidden)pause();else if(visible()){chart();paint();}});
  window.addEventListener("beforeunload",event=>{if(dirty()){event.preventDefault();event.returnValue="";}});
  if(typeof ResizeObserver!=="undefined")new ResizeObserver(chart).observe($("trajChart"));
  initViewer();toolOptions();paint();
  refresh().catch(error=>notice(error.message,false));
})();
