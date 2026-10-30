/* Program tree and script views share one data model and one motion service. */
(() => {
  "use strict";
  const $ = id => document.getElementById(id);
  const escape = value => String(value ?? "").replace(/[&<>"']/g, x => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[x]));
  const clone = value => JSON.parse(JSON.stringify(value));
  const state = {nodes:[], points:[], trajectories:[], programs:[], loadedName:null, programBusy:false, selected:null, dirty:false,
    pointsDirty:false, pointsRevision:0, pointSelection:null, pointCapturing:false, pointsImporting:false,
    pointStarting:false, pointDraft:null, pointDraftTouched:false, pointsSaveError:false, pointMotion:null, pointSnapshotSeq:null,
    pointLastSnapshotAt:0, sourceDirty:false, scriptView:false, history:[], snapshot:null, running:false};
  const pending = new Map();
  const insertionPoints = new Map();
  let pendingInsertion = null;
  let draggedStep = null;
  let lastProgramError = "";
  const RUN_LABELS={idle:"就绪",running:"运行中",paused:"已暂停",completed:"已完成",stopped:"已停止",error:"执行出错"};
  const RUN_ICONS={idle:"M10 3a7 7 0 1 0 0 14a7 7 0 1 0 0-14M10 7v3l2 2",running:"M7 4l9 6-9 6Z",paused:"M7 5v10M13 5v10",completed:"M4 10l4 4 8-8",stopped:"M5 5h10v10H5Z",error:"M10 3L2 17h16L10 3ZM10 8v4M10 14v.1",recording:"M10 4a6 6 0 1 0 0 12a6 6 0 1 0 0-12",replay:"M7 4l9 6-9 6Z"};
  let pointsSaveTask = null;
  let serial = 0;
  const id = () => "n" + Date.now().toString(36) + (++serial).toString(36);
  const active = () => ["running", "paused"].includes((state.snapshot?.program || {}).status);
  const notice = (text, ok = true, target = "progAck") => {
    if (target === "progAck" && window.StationUI.page() === "work/waypoints") target = "wpAck";
    $(target).textContent = text;
    $(target).className = "hint " + (ok ? "tone-ok" : "tone-bad") + (target === "wpAck" ? " waypoint-notice" : "");
    if (!ok || !["work/program", "work/waypoints"].includes(window.StationUI.page())) window.StationUI.notice(text, ok);
  };
  const handle = fn => async ev => {
    try { await fn(ev); } catch (error) { notice(error.message || String(error), false); }
  };
  function request(action, data = {}) {
    return sendRequest({t:"workbench", action, ...data});
  }
  function sendRequest(message) {
    return new Promise((resolve, reject) => {
      const key = "wb" + (++serial);
      const timer = setTimeout(() => {pending.delete(key); reject(new Error("请求超时，请检查站控连接"));}, 12000);
      pending.set(key, {resolve, reject, timer});
      try { window.StationUI.sendImmediate({...message, request_id:key}); }
      catch (error) {clearTimeout(timer); pending.delete(key); reject(error);}
    });
  }
  window.addEventListener("station:ack", ev => {
    const msg = ev.detail;
    const task = pending.get(msg.request_id);
    if (!task) return;
    clearTimeout(task.timer);
    pending.delete(msg.request_id);
    if (msg.ok) task.resolve(msg.data || {}); else task.reject(new Error(msg.error || "操作失败"));
  });

  function locate(ident, nodes = state.nodes, parent = null) {
    for (let i = 0; i < nodes.length; i++) {
      const node = nodes[i];
      if (node.id === ident) return {node, array:nodes, index:i, parent};
      for (const branch of ["children", "otherwise"]) {
        if (node[branch]) {
          const result = locate(ident, node[branch], {node, array:nodes, index:i});
          if (result) return result;
        }
      }
    }
    return null;
  }
  function remember() {
    if (active()) throw new Error("请先停止程序，再编辑步骤");
    state.history.push(JSON.stringify(state.nodes));
    if (state.history.length > 30) state.history.shift();
  }
  function markDirty() {
    state.dirty = true;
    if($("progFileNotice").classList.contains("tone-ok"))$("progFileNotice").textContent="";
    paintFileState();
  }
  function change(fn, parameters = true) {
    remember(); fn(); $("progAck").textContent="";markDirty(); paintTree(); if (parameters) paintParameters(); else updateSelectedPose();
  }
  function label(node) {
    switch (node.type) {
      case "movej": return "MoveJ → " + (node.point || "关节位置") + " · " + node.speed + "°/s";
      case "gripper": return node.open ? "打开夹爪" : "闭合夹爪";
      case "wait": return "等待 " + node.seconds + " 秒";
      case "loop": return "重复 " + node.count + " 次";
      case "if": return "如果 " + ({gripper_open:"夹爪打开",gripper_closed:"夹爪闭合",enabled:"已使能",online:"已连接"}[node.condition]);
      case "replay": return "回放 → " + (node.file || "选择轨迹");
      default: return "注释 · " + (node.text || "");
    }
  }
  function paintTree() {
    let count = 0, gap = 0;
    insertionPoints.clear();
    function insertButton(parent, branch, before, text, empty = false) {
      const key="gap"+(++gap);
      insertionPoints.set(key,{parent,branch,before,label:text});
      return "<button type='button' class='tree-insert"+(empty?" empty-insert":"")+"' data-add-at='"+key+"' aria-label='"+escape(text)+"'><span>＋</span>"+escape(text)+"</button>";
    }
    function sequence(nodes, parent = null, branch = "children", emptyLabel = "添加第一个步骤") {
      let html="<ol role='group'>";
      if(!nodes.length)return html+"<li role='none' class='tree-insert-row'>"+insertButton(parent,branch,null,emptyLabel,true)+"</li></ol>";
      html+="<li role='none' class='tree-insert-row'>"+insertButton(parent,branch,nodes[0].id,parent?"在分支开头添加":"在程序开头添加")+"</li>";
      nodes.forEach((node,index) => {
        count++;
        html += "<li role='treeitem' aria-selected='"+(node.id===state.selected)+"'><button type='button' class='tree-node" + (node.id === state.selected ? " selected" : "") +
          "' data-node='" + escape(node.id) + "' draggable='true'><span class='tree-drag-handle' aria-hidden='true'>⠿</span><span class='node-kind' aria-hidden='true'>" + ({movej:"↗",gripper:"◇",wait:"◷",loop:"↻",if:"?",replay:"▷",comment:"#"}[node.type] || "·") +
          "</span><span class='tree-node-label'>" + escape(label(node)) + "</span><span class='tree-step-status' hidden>执行中</span></button>";
        if(node.type==="loop")html+="<div class='tree-block'><div class='tree-branch'><p class='tree-branch-label'>重复执行</p>"+sequence(node.children,node.id,"children","添加重复内步骤")+"</div></div><div class='tree-block-end'>重复结束</div>";
        if(node.type==="if")html+="<div class='tree-block'><div class='tree-branch'><p class='tree-branch-label'>条件成立时</p>"+sequence(node.children,node.id,"children","添加条件成立时的步骤")+"</div><div class='tree-branch else-branch'><p class='tree-branch-label'>否则</p>"+sequence(node.otherwise,node.id,"otherwise","添加否则分支的步骤")+"</div></div><div class='tree-block-end'>判断结束</div>";
        html+="</li><li role='none' class='tree-insert-row'>"+insertButton(parent,branch,nodes[index+1]?.id||null,node.type==="loop"?"在重复结束后添加":node.type==="if"?"在判断结束后添加":"在此添加步骤")+"</li>";
      });
      return html+"</ol>";
    }
    $("progTree").innerHTML = state.nodes.length ? sequence(state.nodes) : "<div class='tree-empty'><div class='empty-state'><b>从一个步骤开始</b><p>在需要的位置点 ＋，然后选择动作。<br>重复与条件的内部也可以直接添加。</p></div>"+insertButton(null,"children",null,"添加第一个步骤",true)+"</div>";
    $("progNodeCount").textContent = count + " 个节点";
    highlight((state.snapshot?.program || {}).current_id);
    updateButtons();
  }
  function openAddDialog(position) {
    if(active()||state.programBusy)return;
    pendingInsertion={...position};
    const parent=position.parent?locate(position.parent)?.node:null;
    const scope=parent?(parent.type==="loop"?"重复内部":position.branch==="otherwise"?"否则分支":"条件成立分支"):"程序最外层";
    $("progAddContext").textContent=parent?scope+" · "+position.label:position.label;
    $("progAddDialog").showModal();
  }
  function focusSelectedStep(focus = true){
    const tree=$("progTree"), selected=tree.querySelector(".tree-node.selected");
    if(!selected)return;
    if(focus)selected.focus({preventScroll:true});
    const row=selected.getBoundingClientRect(), viewport=tree.getBoundingClientRect();
    if(row.bottom>viewport.bottom)tree.scrollTop+=row.bottom-viewport.bottom+8;
    else if(row.top<viewport.top)tree.scrollTop-=viewport.top-row.top+8;
  }
  function field(title, element) {
    const wrapper = document.createElement("label");
    wrapper.className = "parameter-field";
    const text = document.createElement("span"); text.textContent = title;
    wrapper.append(text, element); $("progParams").appendChild(wrapper);
    return element;
  }
  function numeric(title, key, value, min, max, step = 1) {
    const input = document.createElement("input");
    Object.assign(input, {type:"number", min:String(min), max:String(max), step:String(step), value:String(value)});
    window.StationNumbers.bind(input);
    input.dataset.parameter = key;
    input.onchange = handle(() => {
      let number;
      try {number = window.StationNumbers.read(input,{label:title,min,max,integer:key === "count" || key === "effort"});}
      catch (error) {input.value=locate(state.selected).node[key];window.StationNumbers.clear(input);throw error;}
      change(() => {locate(state.selected).node[key] = number;}, false);
    });
    return field(title, input);
  }
  function selection(title, options, value, onChange) {
    const select = document.createElement("select");
    for (const [key, name] of options) {const option = new Option(name, key); select.add(option);}
    select.value = value;
    select.onchange = handle(() => onChange(select.value));
    return field(title, select);
  }
  function parameterButton(title, fn) {
    const button = document.createElement("button"); button.type = "button"; button.textContent = title;
    button.onclick = handle(fn); $("progParams").appendChild(button); return button;
  }
  function paintParameters() {
    const found = locate(state.selected);
    const box = $("progParams"); box.innerHTML = "";
    if (!found) {box.innerHTML = "<p class='empty-state'>从左侧添加或选中一个步骤。</p>"; $("progParamTitle").textContent = "步骤参数"; updateSelectedPose(); return;}
    const node = found.node;
    $("progParamTitle").textContent = label(node);
    if (node.type === "movej") {
      const names = state.points.map(point => [point.name, point.name]);
      if (node.point && !state.points.some(p => p.name === node.point)) names.unshift([node.point, node.point + "（路点已缺失）"]);
      selection("目标", [["", "直接填写关节角度"], ...names], node.point || "", point => change(() => {
        if (point) {node.point = point; delete node.q;}
        else {delete node.point; node.q = clone(state.snapshot?.arm?.feedback_deg || state.snapshot?.arm?.q_deg || [0,40,40,0,0,0]);}
      }));
      numeric("关节速度 °/s", "speed", node.speed, 5, 80);
      if (!node.point) {
        const grid = document.createElement("div"); grid.className = "joint-parameter-grid";
        node.q.forEach((q, index) => {
          const label = document.createElement("label"); label.textContent = "J" + (index + 1) + " °";
          const input = document.createElement("input"); input.type = "number"; input.step = "0.1"; input.value = q;
          window.StationNumbers.bind(input);
          input.onchange = handle(() => {
            const [min,max] = pointJointRange(index);
            let value;
            try {value=window.StationNumbers.read(input,{label:"J"+(index+1)+" 角度",min,max});}
            catch(error) {input.value=node.q[index];window.StationNumbers.clear(input);throw error;}
            change(() => {node.q[index] = value;}, false);
          });
          label.appendChild(input); grid.appendChild(label);
        });
        box.appendChild(grid);
      }
      parameterButton("使用当前实测关节位置", async () => {
        const data = await request("point_capture"); change(() => {delete node.point; node.q = data.q;});
      });
    } else if (node.type === "gripper") {
      selection("动作", [["true", "打开"],["false", "闭合"]], String(node.open), value => change(() => {node.open = value === "true";}));
      numeric("夹持力矩上限 raw", "effort", node.effort, 50, 800, 10);
      const hint = document.createElement("p"); hint.className = "hint";
      const factor = state.snapshot?.arm?.gripper_nm_per_raw;
      hint.textContent = factor ? "按当前电机型号估算约 " + (node.effort * factor).toFixed(2) + " Nm" : "以电机原始力矩上限 raw 设置。";
      box.appendChild(hint);
    } else if (node.type === "wait") numeric("等待时间（秒）", "seconds", node.seconds, 0, 3600, 0.1);
    else if (node.type === "loop") numeric("重复次数", "count", node.count, 1, 1000);
    else if (node.type === "if") {
      selection("判断条件", [["gripper_open","夹爪处于打开状态"],["gripper_closed","夹爪处于闭合状态"],["enabled","机械臂已使能"],["online","机械臂已连接"]], node.condition, value => change(() => {node.condition = value;}));
    } else if (node.type === "replay") {
      selection("轨迹文件", [["","请选择轨迹"], ...state.trajectories.map(x => [x.name,x.name])], node.file, value => change(() => {node.file = value;}));
      numeric("回放倍率", "rate", node.rate, 0.5, 1, 0.05);
      numeric("走近起点速度 °/s", "speed", node.speed, 5, 80);
    } else {
      const input = document.createElement("textarea"); input.value = node.text; input.maxLength = 500;
      input.onchange = handle(() => change(() => {node.text = input.value;}, false)); field("注释", input);
    }
    const note = document.createElement("p"); note.className = "hint";
    note.textContent = node.type === "movej" ? "MoveJ 按关节空间运动，程序会等待位置到达后执行下一步。" : "修改后点击保存。可使用单步验证每个动作。";
    box.appendChild(note);
    updateSelectedPose(); updateButtons();
  }
  function updateSelectedPose() {
    const node = locate(state.selected)?.node;
    const q = node?.type === "movej" ? (node.q || state.points.find(p => p.name === node.point)?.q) : null;
    $("progSelectedPose").textContent = q ? "所选目标：" + q.map((x,i) => "J"+(i+1)+" "+Number(x).toFixed(1)+"°").join(" · ") : "选中 MoveJ 步骤查看目标路点。";
    if(node)$("progParamTitle").textContent=label(node);
    const eligible=(node?.type==="movej" && Array.isArray(q) && q.length===6)||(node?.type==="replay" && !!node.file);
    $("progPreview").hidden=!eligible;
    const on=!!eligible && window.StationUI.page()==="work/program" && !state.scriptView && $("progPreview").open;
    if(on && !programViewer && window.Arm3D?.create)programViewer=Arm3D.create("prog3dHost","prog3dOverlay");
    if(programViewer && on!==previewActive){programViewer.setActive(on);previewActive=on;}
    if(!eligible && previewKey){previewKey="";previewGeneration++;previewFrames=null;}
    if(on)updateProgramPreview(node,q);
  }
  let programViewer=null, previewActive=false, previewKey="", previewGeneration=0, previewFrames=null;
  function syncProgramPreview() {
    updateSelectedPose();
  }
  function updateProgramPreview(node,q) {
    if(!programViewer || window.StationUI.page()!=="work/program" || state.scriptView || !$("progPreview").open)return;
    const arm=state.snapshot?.arm||{}, live=arm.feedback_deg||arm.q_deg||[0,40,40,0,0,0];
    if(arm.gripper_limits?.length===2)programViewer.setGripRange(...arm.gripper_limits);
    const mode=$("progPreviewMode").value;
    const file=state.trajectories.find(file=>file.name===node?.file);
    const key=mode+":"+(node?.id||"")+":"+(node?.file||"")+":"+(file?.mtime||"")+":"+(file?.frames||"");
    if(key!==previewKey){previewKey=key;previewGeneration++;previewFrames=null;programViewer.setPath([]);programViewer.fitCamera();}
    if(mode==="live"){
      programViewer.setPose(live,arm.gripper_deg,arm.gripper_open);
      $("progPreviewHint").textContent="当前实测姿态 · 切换到「所选步骤目标」查看动作目标。";return;
    }
    if(node?.type==="replay" && node.file){
      if(!previewFrames){
        previewFrames=[];const generation=previewGeneration;
        $("progPreviewHint").textContent="正在载入轨迹预览…";
        request("traj_process",{file:node.file}).then(data=>{
          if(generation!==previewGeneration)return;
          previewFrames=data.frames||[];programViewer.setPath(previewFrames);
          if(previewFrames.length)programViewer.setPose(previewFrames[0].pos.map(x=>x*180/Math.PI),previewFrames[0].gpos==null?arm.gripper_deg:previewFrames[0].gpos*180/Math.PI);
          $("progPreviewHint").textContent="所选轨迹的起点与末端路径 · 仅屏幕预览。";
        }).catch(error=>{if(generation===previewGeneration)$("progPreviewHint").textContent="无法预览："+error.message;});
      }
      return;
    }
    programViewer.setPose(q,arm.gripper_deg,arm.gripper_open);
    $("progPreviewHint").textContent="MoveJ 目标姿态 · 参数修改即时更新，仅屏幕预览。";
  }
  function updateButtons() {
    const busy = active() || state.programBusy;
    const executing=active(), status=state.snapshot?.program?.status||"idle";
    $("progTree").dataset.mode=executing?"execution":"edit";
    $("progTree").dataset.status=status;
    $("btnProgAdd").hidden=executing;
    $("page-program").querySelector(".tree-tools").hidden=executing;
    $("page-program").querySelector(".tree-drag-hint").hidden=executing;
    $("progSpeedScale").hidden=executing;
    $("btnProgRun").hidden=status==="running";
    $("btnProgPause").hidden=status!=="running";
    $("btnProgStep").hidden=status==="running";
    const canStop=executing||status==="error";
    $("btnProgStop").hidden=!canStop;
    for (const ident of ["btnProgAdd","btnProgUp","btnProgDown","btnProgCopy","btnProgRemove","btnProgUndo","btnProgNew","btnProgDelete","btnProgSave","btnProgCopyFile","btnProgCopySave","progCopyName","btnProgApplyScript","progName","progSearch","progScript","progSpeedScale"]) $(ident).disabled = busy;
    const selected=locate(state.selected);
    $("btnProgUp").disabled=busy||!selected||selected.index===0;
    $("btnProgDown").disabled=busy||!selected||selected.index===selected.array.length-1;
    for(const ident of ["btnProgCopy","btnProgRemove"])$(ident).disabled=busy||!selected;
    $("btnProgUndo").disabled=busy||!state.history.length;
    $("btnProgSave").disabled=busy||!!(state.loadedName&&!state.dirty);
    $("btnProgCopyCancel").disabled=state.programBusy;
    $("progTree").querySelectorAll("[data-add-at]").forEach(button=>button.disabled=busy);
    $("progTree").querySelectorAll("[data-node]").forEach(button=>button.draggable=!busy);
    $("progAddDialog").querySelectorAll("[data-add-type]").forEach(button=>button.disabled=busy);
    $("btnProgDelete").disabled=busy || !state.loadedName;
    $("progLibrary").querySelectorAll("button").forEach(button=>button.disabled=busy);
    $("progParams").querySelectorAll("input,select,button,textarea").forEach(x => {x.disabled = busy;});
    const hasSteps=state.nodes.length || (state.sourceDirty && $("progScript").value.trim()) || state.snapshot?.program?.status==="paused";
    $("btnProgRun").disabled = state.programBusy || !hasSteps || (state.snapshot?.program?.status === "running");
    setText("btnProgRun",status==="paused"?"▶ 继续运行":"▶ 运行");
    setText("btnProgStep",status==="paused"?"下一步":"单步");
    $("btnProgPause").disabled = state.snapshot?.program?.status !== "running";
    $("btnProgStep").disabled = state.programBusy || !hasSteps || state.snapshot?.program?.status === "running";
    $("btnProgStop").disabled = !canStop;
  }
  function highlight(ident) {
    $("progTree").querySelectorAll("[data-node]").forEach(button => {
      const current=button.dataset.node===ident&&active();
      button.classList.toggle("executing",current);
      const marker=button.querySelector(".tree-step-status");marker.hidden=!current;
      const text=state.snapshot?.program?.status==="paused"?"已暂停":"执行中";if(marker.textContent!==text)marker.textContent=text;
    });
  }
  function setText(id,value){const element=$(id);if(element.textContent!==value)element.textContent=value;}
  function completedText(program){return "已完成 "+Math.max(0,Number(program.completed)||0)+" 步";}
  function paintRunStatus(program={}){
    const status=Object.prototype.hasOwnProperty.call(RUN_LABELS,program.status)?program.status:"idle", root=$("progExecution");
    if(root.dataset.status!==status){root.dataset.status=status;$("progRunIconPath").setAttribute("d",RUN_ICONS[status]);}
    setText("progRunState",RUN_LABELS[status]);$("progRunState").dataset.status=status;
    setText("progRunName",program.name||"");$("progRunName").hidden=!program.name;$("progRunName").title=program.name||"";
    const node=locate(program.current_id)?.node;
    const step=node?label(node):({movej:"MoveJ · 关节运动",gripper:"夹爪动作",wait:"等待",replay:"轨迹回放"}[program.current_type]||"");
    const detail=status==="running"?(step?"正在执行："+step:"正在准备下一步"):status==="paused"?(step?"暂停于："+step:"点击继续运行，或执行下一步"):status==="completed"?"本次程序执行结束":status==="stopped"?"本次执行已停止":status==="error"?(program.error||"程序执行中断"):"点击运行，或用单步逐项验证";
    setText("progCurrentStep",detail);$("progCurrentStep").title=detail;
    $("progCurrentStep").hidden=status==="completed";
    setText("progProgress",status==="completed"?"共执行 "+Math.max(0,Number(program.completed)||0)+" 步":completedText(program));$("progProgress").hidden=status==="idle";
  }
  function paintGlobalTask(data=state.snapshot||{}){
    const program=data.program||{}, arm=data.arm||{}, busy=["running","paused"].includes(program.status);
    let task=null;
    if(busy)task={status:program.status,state:program.status==="paused"?"程序已暂停":"程序运行中",name:program.name||"未命名程序",detail:completedText(program),page:"work/program",link:"返回程序",stop:"停止程序"};
    else if(arm.recording)task={status:"recording",state:"录制中",name:(arm.traj_file||"示教轨迹").replace(/\.jsonl$/,""),detail:"已录制 "+(Number(arm.rec_s)||0).toFixed(1)+" s",page:"work/trajectories",link:"查看示教",stop:"停止录制"};
    else if(arm.replay_active)task={status:"replay",state:"回放中",name:(arm.traj_file||"示教轨迹").replace(/\.jsonl$/,""),detail:"机械臂正在执行轨迹",page:"work/trajectories",link:"查看示教",stop:"停止运动"};
    else if(program.error)task={status:"error",state:"程序出错",name:program.name||"程序",detail:program.error,page:"work/program",link:"查看程序",stop:"停止运动"};
    const root=$("globalTask");root.hidden=!task||window.StationUI.page()===task.page;
    if(!task)return;
    if(root.dataset.status!==task.status){root.dataset.status=task.status;$("globalTaskIconPath").setAttribute("d",RUN_ICONS[task.status]);}
    setText("globalTaskState",task.state);setText("globalTaskLabel",task.name);setText("globalTaskDetail",task.detail);
    $("globalTaskLabel").title=task.name;$("globalTaskDetail").title=task.detail;
    const link=root.querySelector("a");link.href="#/"+task.page;link.textContent=task.link;
    setText("btnGlobalProgramStop",task.stop);$("btnGlobalProgramStop").hidden=!task.stop;
  }
  function createNode(type) {
    const q = clone(state.snapshot?.arm?.feedback_deg || state.snapshot?.arm?.q_deg || [0,40,40,0,0,0]);
    const values = {movej:{speed:20,...(state.points.length ? {point:state.points[0].name} : {q})}, gripper:{open:true,effort:300},
      wait:{seconds:0.5},loop:{count:2,children:[]},if:{condition:"gripper_open",children:[],otherwise:[]},
      replay:{file:state.trajectories[0]?.name || "",rate:1,speed:20},comment:{text:"说明"}};
    return {id:id(),type,...values[type]};
  }
  function addNode(type, position) {
    if(!["movej","gripper","wait","loop","if","replay","comment"].includes(type))throw new Error("请选择有效的步骤类型");
    const owner=position.parent?locate(position.parent)?.node:null;
    const array=position.parent?owner?.[position.branch]:state.nodes;
    if(!array)throw new Error("插入位置已改变，请重新选择");
    const index=position.before?array.findIndex(node=>node.id===position.before):array.length;
    if(index<0)throw new Error("插入位置已改变，请重新选择");
    change(() => {
      const node = createNode(type);
      array.splice(index,0,node);
      state.selected = node.id;
    });
  }
  function moveStep(ident, position){
    if(active()||state.programBusy)throw new Error("请先停止程序，再移动步骤");
    const source=locate(ident), owner=position.parent?locate(position.parent)?.node:null;
    const target=position.parent?owner?.[position.branch]:state.nodes;
    if(!source||!target)throw new Error("移动位置已改变，请重新拖动");
    function contains(node,key){return node.id===key||[...(node.children||[]),...(node.otherwise||[])].some(child=>contains(child,key));}
    if(position.parent&&contains(source.node,position.parent))throw new Error("不能把一个结构放入它自己或它的内部步骤");
    let index=position.before?target.findIndex(node=>node.id===position.before):target.length;
    if(index<0)throw new Error("移动位置已改变，请重新拖动");
    if(target===source.array){
      if(index===source.index||index===source.index+1)return;
      if(source.index<index)index--;
    }
    change(()=>{source.array.splice(source.index,1);target.splice(index,0,source.node);state.selected=source.node.id;});
    focusSelectedStep();
  }
  async function applyScript() {
    notice("正在检查脚本…");
    const data = await request("parse", {script:$("progScript").value});
    remember(); state.nodes = data.nodes; state.sourceDirty = false; state.selected = state.nodes[0]?.id || null;
    $("progScript").value = data.script; markDirty(); paintTree(); paintParameters();
    notice("脚本检查通过，已同步到程序树");
  }
  async function setView(script) {
    if (!script && state.sourceDirty) await applyScript();
    if (script && !state.sourceDirty) $("progScript").value = (await request("parse", {nodes:state.nodes})).script;
    state.scriptView = script;
    $("programWorkspace").hidden = script; $("programScriptView").hidden = !script;
    $("btnProgTree").classList.toggle("on", !script); $("btnProgScript").classList.toggle("on", script);
    syncProgramPreview();
  }
  async function confirmDiscard() {
    return !(state.dirty || state.sourceDirty) || await window.StationDialog.confirm({title:"放弃修改？",
      message:"当前程序有未保存的修改，放弃后无法恢复。", confirmText:"放弃修改", destructive:true});
  }
  function setProgram(data, preserve = false, saved = true) {
    const previous = preserve ? state.selected : null;
    state.nodes = data.nodes || []; state.selected = locate(previous) ? previous : (state.nodes[0]?.id || null);
    state.dirty = state.sourceDirty = false;if(!preserve)state.history=[];
    state.loadedName=saved?data.name:null;
    $("progName").value = data.name || nextProgramName(); $("progScript").value = data.script || "";
    $("progName").readOnly=!!state.loadedName;
    $("progFileNotice").textContent="";
    paintProgramLibrary();paintFileState();paintTree();paintParameters();
    if(!preserve)$("progTree").scrollTop=0;
  }
  function nextProgramName(){let n=1;while(state.programs.includes("程序_"+n))n++;return "程序_"+n;}
  function paintProgramLibrary(){
    const query=$("progSearch").value.trim().toLowerCase(), names=state.programs.filter(name=>name.toLowerCase().includes(query));
    $("progLibraryCount").textContent=state.programs.length+" 个";
    $("progLibrary").innerHTML=names.map(name=>"<button type='button' role='option' aria-selected='"+(name===state.loadedName)+"' class='program-file"+(name===state.loadedName?" selected":"")+"' data-program='"+escape(name)+"'><span class='program-file-icon' aria-hidden='true'>▤</span><span class='program-file-name'>"+escape(name)+"</span>"+(name===state.loadedName?"<small data-current-marker>编辑中</small>":"")+"</button>").join("")||"<p class='hint'>"+(query?"没有匹配的程序":"还没有已保存程序，先新建并保存。")+"</p>";
  }
  function paintFileState(){
    $("progDirty").textContent=state.dirty?(state.loadedName?"● 有未保存修改":"● 新程序 · 尚未保存"):(state.loadedName?"已保存":"新程序 · 尚未保存");
    $("progDirty").dataset.dirty=String(state.dirty||!state.loadedName);
    $("btnProgSave").textContent=state.programBusy?"正在处理…":"保存程序";
    const marker=$("progLibrary").querySelector("[data-current-marker]");if(marker)marker.textContent=state.dirty?"未保存":"编辑中";
  }
  function fileNotice(message,ok=true){
    $("progFileNotice").textContent=message;$("progFileNotice").className="program-file-notice "+(ok?"tone-ok":"tone-bad");
  }
  const fileHandle=fn=>async event=>{try{await fn(event);}catch(error){fileNotice(error.message||String(error),false);notice(error.message||String(error),false);}};
  async function saveProgram(name){
    return programTask(async()=>{
      if(state.sourceDirty)await applyScript();if(state.pointsDirty)await savePoints();
      const data=await request("save",{name,nodes:state.nodes});
      $("progSearch").value="";setProgram(data,true);await refreshAssets();fileNotice("已保存到程序库");
      return data;
    });
  }
  async function programTask(fn){
    if(active()||state.programBusy)throw new Error("请先停止程序或等待当前操作完成");
    state.programBusy=true;paintFileState();updateButtons();
    try{return await fn();}finally{state.programBusy=false;paintFileState();updateButtons();}
  }
  async function refreshAssets() {
    const revision = state.pointsRevision;
    const refreshPoints = !state.pointsDirty && !pointsSaveTask && !state.pointsImporting && !state.pointStarting;
    const [catalogResponse, trajResponse] = await Promise.all([fetch("/api/workspace"),fetch("/api/traj")]);
    if (!catalogResponse.ok || !trajResponse.ok) throw new Error("无法读取路点或程序库");
    const catalog = await catalogResponse.json();
    if (refreshPoints && !state.pointsDirty && !state.pointsImporting && !state.pointStarting && revision === state.pointsRevision && !$("wpEditor").contains(document.activeElement)) {
      const selectedName = state.pointSelection?.name;
      state.points = catalog.waypoints || [];
      state.pointSelection = state.pointDraft || state.points.find(point => point.name === selectedName) || null;
      paintPoints();
    }
    state.trajectories = (await trajResponse.json()).files || [];
    state.programs=catalog.programs||[];paintProgramLibrary();
    if(!state.loadedName&&!state.dirty&&!state.nodes.length)$("progName").value=nextProgramName();
    paintParameters();paintFileState();updateButtons();
  }
  function pointsChanged() {
    state.pointsDirty = true;
    state.pointsSaveError = false;
    state.pointsRevision++;
    notice("", true, "wpAck");
    paintPointCount();
    paintPointSaveState();
  }
  function savePoints() {
    if (pointsSaveTask) return pointsSaveTask;
    if (!state.pointsDirty) return Promise.resolve();
    // Serialize writes and keep newer edits when an older acknowledgement arrives.
    pointsSaveTask = (async () => {
      try {
        while (state.pointsDirty) {
          const revision = state.pointsRevision;
          await request("points_save", {points:clone(state.points)});
          if (revision === state.pointsRevision) state.pointsDirty = false;
        }
        state.pointsSaveError = false;
        paintPointCount();
        paintPointSaveState();
      } catch (error) {
        state.pointsSaveError = true; paintPointSaveState();
        throw new Error("自动保存失败，修改尚未保存：" + error.message + "。请检查连接后重试修改。");
      } finally { pointsSaveTask = null; }
    })();
    return pointsSaveTask;
  }
  function pointName() {
    let index = state.points.length + 1;
    while (state.points.some(point => point.name === "路点_"+index)) index++;
    return "路点_"+index;
  }
  function validPointName(name, point) {
    return name && name.length <= 64 && !/[\\/:*?"<>|\x00-\x1f]/.test(name) && ![".",".."].includes(name) && !state.points.some(p => p !== point && p.name === name);
  }
  function pointJointRange(index) {
    return state.snapshot?.arm?.limits?.[index] || [-360,360];
  }
  function pointMotionActive() {
    return ["preparing","waiting","moving"].includes(state.pointMotion?.status);
  }
  async function discardPointDraft() {
    const draft = state.pointDraft;
    if (draft && state.pointDraftTouched && !await window.StationDialog.confirm({title:"放弃新建路点？",message:"继续将放弃这个路点尚未保存的修改。",confirmText:"放弃添加",destructive:true})) return false;
    state.pointDraft = null;
    state.pointDraftTouched = false;
    if (state.pointSelection === draft) state.pointSelection = null;
    return true;
  }
  async function selectPoint(point) {
    if (state.pointSelection === point) return;
    if (!await discardPointDraft()) return;
    state.pointSelection = point;
    if (!state.pointsSaveError) notice("", true, "wpAck");
    if (!pointMotionActive()) state.pointMotion = null;
    paintPoints();
  }
  async function capturePoint() {
    if (state.pointCapturing || state.pointsImporting) return;
    state.pointCapturing = true; updatePointControls();
    try {
      if (!await discardPointDraft()) return;
      paintPoints();
      const q = (await request("point_capture")).q;
      const point = {name:pointName(),q,t:2};
      state.points.push(point); state.pointSelection = point;
      state.pointMotion = null; pointsChanged(); paintPoints(); await savePoints();
      notice("已记录当前位置："+point.name, true, "wpAck");
    } finally { state.pointCapturing = false; updatePointControls(); }
  }
  function paintPointCount() {
    $("wpCount").textContent = state.points.length + " 个路点";
  }
  function paintPointSaveState() {
    const element = $("wpSaveState");
    const status = state.pointDraft ? "draft" : state.pointsSaveError ? "error" : state.pointsDirty ? "dirty" : "saved";
    element.dataset.state = status;
    const filled = state.pointDraft ? Array.from($("wpEditor").querySelectorAll("[data-point-q]")).filter(input=>input.value.trim()).length : 0;
    element.textContent = !state.pointSelection && !state.pointsDirty ? "" : ({draft:"已填写 "+filled+"/6",error:"保存失败",dirty:"正在保存…",saved:"已自动保存"}[status]);
  }
  function updatePointControls() {
    const locked = state.pointsImporting || pointMotionActive();
    $("btnWpAdd").disabled = locked || state.pointCapturing;
    $("btnWpManual").disabled = locked || state.pointCapturing || !!state.pointDraft;
    $("btnWpClear").disabled = locked || !state.points.length;
    $("btnWpImport").disabled = locked || state.pointCapturing;
    $("btnWpExport").disabled = locked;
    $("btnWpRun").disabled = locked || !state.points.includes(state.pointSelection) || state.pointStarting || active();
    $("wpList").querySelectorAll("button").forEach(element => {element.disabled = state.pointsImporting;});
    $("wpEditor").querySelectorAll("input,button").forEach(element => {element.disabled = locked;});
    $("wpRunTarget").textContent = state.pointDraft ? "未完成路点" : state.pointSelection?.name || "尚未选择";
  }
  function paintPointList() {
    $("wpList").innerHTML = state.points.length ? state.points.map((point,index) =>
      "<li><button type='button' class='waypoint-item"+(point === state.pointSelection ? " selected" : "")+"' aria-pressed='"+(point === state.pointSelection)+"' data-point-select='"+index+"'><b>"+escape(point.name)+"</b><small>"+
      point.q.slice(0,3).map((q,j)=>"J"+(j+1)+" "+Number(q).toFixed(1)+"°").join(" · ")+"</small></button></li>").join("") : "<li class='waypoint-library-empty'>还没有路点<br/>记录当前位置，或手动添加。</li>";
    $("wpList").querySelectorAll("[data-point-select]").forEach(button => button.onclick = handle(() => selectPoint(state.points[Number(button.dataset.pointSelect)])));
  }
  function paintPoints() {
    paintPointCount();
    if (state.pointSelection !== state.pointDraft && !state.points.includes(state.pointSelection)) state.pointSelection = null;
    paintPointList(); paintPointEditor(); paintPointSaveState(); paintPointMotion(); updatePointControls();
  }
  function paintPointEditor() {
    const point = state.pointSelection, draft = point && point === state.pointDraft;
    $("wpEditorEmpty").hidden = !!point;
    $("wpEditor").hidden = !point;
    $("wpDetailTitle").textContent = draft ? "新建路点" : "位置参数";
    if (!point) {$("wpEditor").innerHTML = "";return;}
    $("wpEditor").innerHTML = "<div class='waypoint-heading'><label>路点名称<input data-point-name value='"+escape(point.name)+"' maxlength='64' aria-label='路点名称'/></label>"+
      (draft ? "<button type='button' class='destructive-subtle' id='btnWpDraftCancel'>取消添加</button>" : "<button type='button' class='destructive-subtle' id='btnWpDelete'>删除路点</button>")+"</div><div class='waypoint-joints'>"+
      point.q.map((q,j)=>{const [min,max]=pointJointRange(j);return "<label>J"+(j+1)+" <span>°</span><input type='number' min='"+min+"' max='"+max+"' step='any' required data-point-q='"+j+"' value='"+(q === null ? "" : q)+"' placeholder='填写角度'/></label>";}).join("")+"</div><div class='waypoint-editor-note'>"+
      (draft ? "<p class='hint'>填写六轴角度，Tab / Enter 切换输入框。六轴全部有效并结束输入后自动保存。</p>" : "<p class='hint'>名称与角度修改后自动保存 · 选择和编辑不会触发运动</p>")+"</div>";
    const nameInput = $("wpEditor").querySelector("[data-point-name]");
    nameInput.onchange = handle(async () => {
        const input = nameInput;
        const name = input.value.trim();
        if (!validPointName(name, point)) {
          input.value = point.name;
          throw new Error("路点名称须为 1–64 个字符，不能重复或包含路径及特殊文件字符");
        }
        point.name = name; input.value = name; await commitPointEdit(point);
    });
    $("wpEditor").querySelectorAll("[data-point-q]").forEach(input => {
      const j = Number(input.dataset.pointQ);
      window.StationNumbers.bind(input);
      input.onchange = handle(async () => {
        if (point === state.pointDraft && !input.value.trim()) {
          point.q[j]=null;paintPointSaveState();return;
        }
        const [min,max] = pointJointRange(j);
        let value;
        try {value=window.StationNumbers.read(input,{label:"J"+(j+1)+" 角度",min,max});}
        catch(error) {input.value=point.q[j] ?? "";window.StationNumbers.clear(input);throw error;}
        point.q[j] = value; await commitPointEdit(point);
      });
    });
    $("wpEditor").querySelectorAll("input").forEach(input => {
      input.oninput = () => {if(state.pointDraft === point) {state.pointDraftTouched=true;paintPointSaveState();}};
      // Custom decimal paste may not emit change; validate when focus leaves too.
      input.onblur = handle(async () => {
        if(!input.isConnected || $("confirmDialog").open)return;
        if(state.pointDraft === point) {await input.onchange();return;}
        let changed;
        if(input.dataset.pointQ !== undefined) {
          const j=Number(input.dataset.pointQ),[min,max]=pointJointRange(j);
          changed=window.StationNumbers.read(input,{label:"J"+(j+1)+" 角度",min,max}) !== point.q[j];
        } else changed=input.value.trim() !== point.name;
        if(changed)await input.onchange();
      });
      input.onkeydown = handle(async ev => {
        if(ev.key !== "Enter") return;
        ev.preventDefault();
        if(state.pointDraft === point) {
          const inputs=Array.from($("wpEditor").querySelectorAll("input"));
          const next=inputs[inputs.indexOf(input)+1] || inputs.find(field=>!field.value.trim()) || input;
          input.blur();
          if(state.pointDraft === point) {next.focus();if(next.select)next.select();}
          else $("wpEditor").querySelector("[data-point-q='"+(input.dataset.pointQ || "0")+"']")?.focus();
        } else input.blur();
      });
    });
    if (!draft) $("btnWpDelete").onclick = handle(async () => {
      if (!await window.StationDialog.confirm({title:"删除路点？", message:"确定删除路点「"+point.name+"」？此操作无法撤销。\n引用此路点的程序需要重新选择位置。", confirmText:"删除路点", destructive:true})) return;
      const index = state.points.indexOf(point);
      if (index < 0) return;
      state.points.splice(index,1); pointsChanged(); paintPoints(); await savePoints();
    });
    else {
      $("btnWpDraftCancel").onpointerdown = event => event.preventDefault();
      $("btnWpDraftCancel").onclick = handle(async () => {if(await discardPointDraft()) {state.pointSelection=null;paintPoints();}});
    }
  }
  async function commitPointEdit(point) {
    if (point === state.pointDraft) {
      state.pointDraftTouched=true;paintPointSaveState();
      const fields=Array.from($("wpEditor").querySelectorAll("[data-point-q]"));
      if(fields.length !== 6 || fields.some(input=>!input.value.trim()))return;
      const name=$("wpEditor").querySelector("[data-point-name]").value.trim();
      if(!validPointName(name,point))throw new Error("路点名称不能重复或包含路径及特殊文件字符");
      const q=fields.map((input,j)=>{const [min,max]=pointJointRange(j);return window.StationNumbers.read(input,{label:"J"+(j+1)+" 角度",min,max});});
      point.name=name;point.q=q;
      state.points.push(point);state.pointDraft=null;state.pointDraftTouched=false;paintPointEditor();
    }
    if (!state.points.includes(point)) return;
    if (!pointMotionActive()) state.pointMotion = null;
    pointsChanged(); paintPointList(); paintPointMotion(); updatePointControls(); await savePoints();
  }
  function beginManualPoint() {
    if (state.pointDraft) return;
    state.pointDraft = {name:pointName(),q:Array(6).fill(null),t:2};
    state.pointDraftTouched = false;
    state.pointSelection = state.pointDraft; state.pointMotion = null;
    if (!state.pointsSaveError) notice("", true, "wpAck");
    paintPoints(); const input=$("wpEditor").querySelector("[data-point-q]");input.focus();input.select();
  }
  function paintPointMotion() {
    const motion = state.pointMotion;
    $("wpMotionState").dataset.state = motion?.status || "idle";
    $("wpMotionState").textContent = motion?.text || (state.pointDraft ? "六轴角度填写完整后自动保存，添加本身不会触发运动。" : state.pointSelection ? "就绪 · 按设定速度前往所选位置" : "选择路点后可前往，选择本身不会触发运动。");
  }
  function finishPointMotion(status, text) {
    if (!state.pointMotion) return;
    Object.assign(state.pointMotion,{status,text,cancelled:true});
    paintPointMotion(); updatePointControls();
  }
  function trackPointMotion(data, fresh) {
    const motion = state.pointMotion, arm = data.arm || {};
    if (!motion || motion.status !== "moving" || !fresh) return;
    if (arm.online === false) {finishPointMotion("error","连接已中断，运动状态未知");return;}
    if (data.safety === "ESTOP_LATCHED" || active() || arm.replay_active) {finishPointMotion("stopped","前往「"+motion.name+"」的运动已中止");return;}
    const q = arm.feedback_deg || arm.q_deg;
    const near = Array.isArray(q) && q.length >= 6 && motion.q.every((value,index) => Number.isFinite(q[index]) && Math.abs(q[index]-value) <= 0.5);
    const now = Date.now();
    if (arm.moving) {motion.seenMoving = true;motion.stableAt = 0;motion.stoppedAt = 0;return;}
    if (near) {
      motion.stoppedAt = 0;
      motion.stableAt ||= now;
      if (now-motion.stableAt >= 150) finishPointMotion("arrived","已到达「"+motion.name+"」");
    } else {
      motion.stableAt = 0;
      if (motion.seenMoving || now-motion.acceptedAt > 2000) {
        motion.stoppedAt ||= now;
        if (now-motion.stoppedAt >= 400) finishPointMotion("stopped","运动已停止，未到达「"+motion.name+"」");
      }
    }
  }
  async function runSelectedPoint() {
    const point = state.pointSelection;
    if (!point || !state.points.includes(point)) throw new Error("请先选择目标路点");
    const speed = window.StationNumbers.read($("wpRunSpeed"),{label:"速度",min:5,max:80});
    if (state.pointStarting) return;
    const motion = {status:"preparing",text:"正在准备前往「"+point.name+"」…",name:point.name,cancelled:false};
    state.pointMotion = motion; paintPointMotion();
    state.pointStarting = true; updatePointControls();
    try {
      if (state.pointsDirty) await savePoints();
      if (motion.cancelled) return;
      if (active()) throw new Error("请先停止正在运行的程序");
      if (state.pointSelection !== point || !state.points.includes(point)) throw new Error("目标路点已变化，请重新选择");
      window.StationUI.releaseManual();
      motion.q = clone(point.q); motion.status = "waiting"; motion.text = "正在下发前往「"+point.name+"」的指令…"; paintPointMotion();
      await sendRequest({t:"path",q:[motion.q],speed});
      if (!motion.cancelled) {
        Object.assign(motion,{status:"moving",text:"正在前往「"+point.name+"」",acceptedAt:Date.now(),stableAt:0,seenMoving:false});
        state.pointLastSnapshotAt = Date.now(); paintPointMotion();
      }
    } catch (error) {
      if (!motion.cancelled) finishPointMotion("error","前往失败："+error.message);
      throw error;
    } finally { state.pointStarting = false; updatePointControls(); }
  }
  async function runProgram(single = false) {
    $("progAck").textContent="";
    window.StationUI.releaseManual();
    if (state.sourceDirty) await applyScript();
    if (state.pointsDirty) await savePoints();
    const program = state.snapshot?.program || {};
    if (program.status === "paused") await request(single ? "step" : "resume");
    else {
      if (!state.nodes.length) throw new Error("请先添加程序步骤");
      const checked = await request("parse", {nodes:state.nodes}); state.nodes = checked.nodes; paintTree(); paintParameters();
      await request(single ? "step" : "run", {name:$("progName").value, nodes:state.nodes, speed_scale:Number($("progSpeedScale").value)/100});
    }
    // An earlier stop/pause ACK can arrive while parsing this new run.
    $("progAck").textContent="";
  }
  function snapshot(data) {
    const fresh = data.seq == null || data.seq !== state.pointSnapshotSeq;
    if (fresh) {state.pointSnapshotSeq = data.seq;state.pointLastSnapshotAt = Date.now();}
    state.snapshot = data;
    trackPointMotion(data, fresh);
    const program = data.program || {};
    paintRunStatus(program);
    if(program.error&&program.error!==lastProgramError)notice(program.error,false);
    if(!program.error&&lastProgramError)for(const id of ["progAck","wpAck"])if($(id).textContent===lastProgramError)$(id).textContent="";
    lastProgramError=program.error||"";
    $("progLivePose").textContent = (data.arm?.feedback_deg || data.arm?.q_deg || []).map((q,i)=>"J"+(i+1)+" "+Number(q).toFixed(1)+"°").join("　") || "等待反馈";
    const busy = active();
    $("manualProgramHint").hidden = !busy;
    if (busy && !state.running) window.StationUI.releaseManual();
    state.running = busy;
    highlight(program.current_id); updateButtons(); updatePointControls();
    paintGlobalTask(data);
    updateSelectedPose();
  }

  $("btnProgAdd").onclick = () => openAddDialog({parent:null,branch:"children",before:null,label:"在程序末尾添加步骤"});
  $("progTree").onclick = event => {
    const add=event.target.closest("[data-add-at]");
    if(add){const position=insertionPoints.get(add.dataset.addAt);if(position)openAddDialog(position);return;}
    const button=event.target.closest("[data-node]");
    if(button){state.selected=button.dataset.node;paintTree();paintParameters();}
  };
  function clearDrag(){
    draggedStep=null;$("progTree").querySelectorAll(".drop-target,.dragging").forEach(element=>element.classList.remove("drop-target","dragging"));
  }
  $("progTree").addEventListener("dragstart",event=>{
    const button=event.target.closest("[data-node]");
    if(!button||active()||state.programBusy){event.preventDefault();return;}
    draggedStep=button.dataset.node;event.dataTransfer.effectAllowed="move";event.dataTransfer.setData("application/x-fafu-step",draggedStep);button.classList.add("dragging");
  });
  $("progTree").addEventListener("dragover",event=>{
    const target=event.target.closest("[data-add-at]");
    if(!draggedStep||!target||active()||state.programBusy)return;
    event.preventDefault();event.dataTransfer.dropEffect="move";
    $("progTree").querySelectorAll(".drop-target").forEach(element=>{if(element!==target)element.classList.remove("drop-target");});target.classList.add("drop-target");
  });
  $("progTree").addEventListener("dragleave",event=>{
    const target=event.target.closest("[data-add-at]");if(target&&!target.contains(event.relatedTarget))target.classList.remove("drop-target");
  });
  $("progTree").addEventListener("drop",event=>{
    const target=event.target.closest("[data-add-at]"), ident=draggedStep;
    if(!ident||!target)return;
    event.preventDefault();const position=insertionPoints.get(target.dataset.addAt);clearDrag();
    try{if(position)moveStep(ident,position);}catch(error){notice(error.message,false);}
  });
  $("progTree").addEventListener("dragend",clearDrag);
  window.addEventListener("resize",()=>focusSelectedStep(false));
  $("btnProgAddCancel").onclick=()=>$("progAddDialog").close();
  $("progAddDialog").onclose=()=>{if(!$("progAddDialog").open)pendingInsertion=null;};
  $("progAddDialog").onclick=event=>{
    const type=event.target.closest("[data-add-type]")?.dataset.addType;
    if(!type||!pendingInsertion)return;
    try{
      addNode(type,pendingInsertion);$("progAddDialog").close();
      focusSelectedStep();
    }catch(error){$("progAddContext").textContent=error.message;}
  };
  for (const [button, offset] of [["btnProgUp",-1],["btnProgDown",1]]) $(button).onclick = handle(() => change(() => {
    const item = locate(state.selected); if (!item) return;
    const next = item.index+offset; if (next<0 || next>=item.array.length) return;
    [item.array[item.index],item.array[next]]=[item.array[next],item.array[item.index]];
  }));
  $("btnProgCopy").onclick = handle(() => change(() => {
    const item=locate(state.selected); if (!item) return;
    const node=clone(item.node); function renew(n){n.id=id();(n.children||[]).forEach(renew);(n.otherwise||[]).forEach(renew);} renew(node);
    item.array.splice(item.index+1,0,node);state.selected=node.id;
  }));
  $("btnProgRemove").onclick = handle(() => change(() => {const item=locate(state.selected);if(item){item.array.splice(item.index,1);state.selected=null;}}));
  $("btnProgUndo").onclick = handle(() => {if(active())return;if(state.history.length){state.nodes=JSON.parse(state.history.pop());if(!locate(state.selected))state.selected=state.nodes[0]?.id||null;markDirty();paintTree();paintParameters();focusSelectedStep();}});
  $("btnProgTree").onclick = handle(() => setView(false));
  $("btnProgScript").onclick = handle(() => setView(true));
  $("btnProgApplyScript").onclick = handle(applyScript);
  $("progScript").oninput = () => {state.sourceDirty=true;markDirty();updateButtons();};
  $("progName").oninput = () => {if(!state.loadedName){markDirty();updateButtons();}};
  $("progSearch").oninput=()=>{paintProgramLibrary();updateButtons();};
  $("progLibrary").onclick=fileHandle(async event=>{
    const name=event.target.closest("[data-program]")?.dataset.program;if(!name)return;
    if(name===state.loadedName && !state.dirty)return;
    if(await confirmDiscard())await programTask(async()=>{setProgram(await request("load",{name}));await setView(false);});
  });
  $("btnProgNew").onclick = fileHandle(async () => {if(await confirmDiscard()) {$("progSearch").value="";setProgram({name:nextProgramName()},false,false);await setView(false);markDirty();$("progName").focus();$("progName").select();}});
  $("btnProgSave").onclick = fileHandle(async () => {
    const name=state.loadedName||$("progName").value.trim();if(!name)throw new Error("请填写程序名称");
    if(state.programs.includes(name)&&name!==state.loadedName&&!await window.StationDialog.confirm({title:"覆盖已有程序？",message:"程序库中已有「"+name+"」，保存将替换其内容。",confirmText:"覆盖保存",destructive:true}))return;
    await saveProgram(name);
  });
  $("btnProgCopyFile").onclick=()=>{
    if(active()||state.programBusy)return;
    const base=($("progName").value.trim()||"程序").slice(0,54);let name=base+"_副本",n=2;
    while(state.programs.includes(name))name=base+"_副本"+(n++);
    $("progCopyName").value=name;$("progCopyError").textContent="";$("progCopyDialog").showModal();$("progCopyName").focus();$("progCopyName").select();
  };
  $("btnProgCopyCancel").onclick=()=>$("progCopyDialog").close();
  $("progCopyDialog").oncancel=event=>{if(state.programBusy)event.preventDefault();};
  $("progCopyForm").onsubmit=async event=>{
    event.preventDefault();const name=$("progCopyName").value.trim();
    if(!name){$("progCopyError").textContent="请填写副本名称";return;}
    if(state.programs.includes(name)){$("progCopyError").textContent="已有同名程序，请使用另一个名称";return;}
    try{await saveProgram(name);$("progCopyDialog").close();}catch(error){$("progCopyError").textContent=error.message;}
  };
  $("btnProgDelete").onclick = fileHandle(async () => {
    const name=state.loadedName;if(!name)throw new Error("当前草稿尚未保存");
    if(!await window.StationDialog.confirm({title:"删除程序？",message:"确定删除程序「"+name+"」？此操作无法撤销。"+(state.dirty?"\n当前未保存的修改也将被放弃。":""),confirmText:"删除程序",destructive:true}))return;
    await programTask(async()=>{await request("delete",{name});await refreshAssets();setProgram({name:nextProgramName()},false,false);fileNotice("已删除「"+name+"」");});
  });
  $("progPreview").ontoggle=syncProgramPreview;
  $("progPreviewMode").onchange=()=>{previewKey="";updateSelectedPose();};
  $("btnProgRefresh").onclick = handle(async () => {await refreshAssets();notice("路点、轨迹和程序库已刷新");});
  $("btnProgRun").onclick = handle(() => runProgram(false));
  $("btnProgStep").onclick = handle(() => runProgram(true));
  $("btnProgPause").onclick = handle(async () => {await request("pause");notice("已暂停并保持姿态；继续时重新执行当前步骤，轨迹回放会重新走到起点。");});
  async function stopMotion() {
    if (pointMotionActive()) finishPointMotion("stopped","正在停止运动…");
    try {
      await request("stop");
      finishPointMotion("stopped","运动已停止并保持当前位置");
      notice("运动已停止并保持当前位置");
    } catch (error) {finishPointMotion("error","停止请求失败："+error.message);throw error;}
  }
  for (const ident of ["btnProgStop","btnWpStop"]) $(ident).onclick = handle(stopMotion);
  $("btnGlobalProgramStop").onclick = handle(async () => {if(state.snapshot?.arm?.recording && !active())window.StationUI.sendImmediate({t:"rec_stop"});else await stopMotion();});
  $("progSpeedScale").oninput = () => {$("progSpeedLabel").textContent=$("progSpeedScale").value+"%";};
  $("btnWpAdd").onclick = handle(capturePoint);
  $("btnWpManual").onclick = handle(beginManualPoint);
  window.StationNumbers.bind($("wpRunSpeed"));
  $("btnWpClear").onclick = handle(async () => {
    if (!state.points.length) return;
    if (!await window.StationDialog.confirm({title:"清空路点库？",message:"确定删除全部 "+state.points.length+" 个路点？此操作无法撤销。\n引用这些路点的程序需要重新选择位置。",confirmText:"清空列表",destructive:true})) return;
    state.points=[]; state.pointSelection=state.pointDraft; pointsChanged(); paintPoints(); await savePoints();
  });
  $("btnWpRun").onclick = handle(runSelectedPoint);
  $("btnWpImport").onclick = () => $("wpImportFile").click();
  $("wpImportFile").onchange = handle(async ev => {
    const file=ev.target.files[0];if(!file)return;if(file.size>1024*1024){ev.target.value="";throw new Error("路点文件超过 1 MB");}
    if (!await discardPointDraft()) {ev.target.value="";return;}
    state.pointsImporting=true;paintPoints();
    try {
      const parsed=JSON.parse(await file.text());const points=Array.isArray(parsed)?parsed:parsed.waypoints;
      await savePoints();
      const result=await request("points_save",{points});state.points=result.waypoints;state.pointSelection=null;state.pointMotion=null;state.pointsRevision++;state.pointsDirty=false;state.pointsSaveError=false;paintPoints();notice("路点已导入",true,"wpAck");
    } finally {state.pointsImporting=false;ev.target.value="";updatePointControls();}
  });
  $("btnWpExport").onclick = handle(async () => {
    if(state.pointsDirty)await savePoints();
    const blob=new Blob([JSON.stringify({version:1,waypoints:state.points},null,2)],{type:"application/json"});
    const url=URL.createObjectURL(blob);const anchor=document.createElement("a");anchor.href=url;anchor.download="FAFU-waypoints.json";anchor.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  });
  window.addEventListener("station:snapshot", ev => snapshot(ev.detail));
  window.addEventListener("station:page", ev => {
    syncProgramPreview();
    paintGlobalTask();
    if(ev.detail==="work/program")refreshAssets().catch(error=>notice(error.message,false));
  });
  window.addEventListener("beforeunload", ev => {if(state.dirty||state.pointsDirty||state.sourceDirty||state.pointDraftTouched){ev.preventDefault();ev.returnValue="";}});
  setInterval(() => {
    if (state.pointMotion?.status === "moving" && Date.now()-state.pointLastSnapshotAt > 3000) finishPointMotion("error","反馈已中断，运动状态未知，请检查连接");
  }, 500);
  paintProgramLibrary();paintFileState();paintTree();paintParameters();paintPoints();syncProgramPreview();paintRunStatus();paintGlobalTask();
  if(window.StationUI.snapshot())snapshot(window.StationUI.snapshot());
  refreshAssets().catch(error=>notice(error.message,false));
})();
