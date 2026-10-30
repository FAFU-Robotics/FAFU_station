"use strict";
const assert = require("node:assert/strict");

module.exports = async function({evaluate,wait,cdp,screenshot,run,delay}) {
  const paths = () => evaluate('testStation.sent.filter(x=>x.t==="path").length');
  const saved = () => wait('document.getElementById("wpSaveState").dataset.state==="saved"');
  const select = index => evaluate('choose('+index+')');
  const motionMessage = () => evaluate('document.getElementById("wpMotionState").textContent');
  const lastPath = async () => {const {request_id,...message}=await evaluate('testStation.sent.filter(x=>x.t==="path").at(-1)');assert.match(request_id,/^wb/);return message;};
  const pointerClick = async selector => {
    const rect=await evaluate('(()=>{const element=document.querySelector('+JSON.stringify(selector)+');element.scrollIntoView({block:"center"});const r=element.getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()');
    await cdp("Input.dispatchMouseEvent",{type:"mousePressed",...rect,button:"left",clickCount:1});
    await cdp("Input.dispatchMouseEvent",{type:"mouseReleased",...rect,button:"left",clickCount:1});
  };
  await run("one shared selection controls the editor and single-point execution",async()=>{
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
    assert.equal(await evaluate('document.querySelector("#wpNewName,#wpSource,#wpRunPoint,#btnWpSave,[data-point-time],[data-point-go],[data-point-update]")'),null);
    await screenshot("waypoints-empty-selection");
    await select(1);
    assert.equal(await paths(),0);
    assert.equal(await evaluate('document.querySelectorAll("[data-point-name]").length'),1);
    assert.equal(await evaluate('document.getElementById("wpRunTarget").textContent'),"放料点");
    assert.equal(await evaluate('document.querySelector("[data-point-select=\\"1\\"]").getAttribute("aria-pressed")'),"true");
    await screenshot("waypoints");
    await evaluate('click("#btnWpRun")');
    await wait('document.getElementById("wpMotionState").dataset.state==="moving"');
    assert.deepEqual(await lastPath(),{t:"path",q:[[15,40,40,0,0,0]],speed:20});
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
  });
  await run("arrival is confirmed by stable measured feedback, never command acceptance",async()=>{
    await evaluate('feedback([0,40,40,0,0,0])');
    assert.match(await motionMessage(),/正在前往/);
    await evaluate('feedback([8,40,40,0,0,0],true);feedback([15,40,40,0,0,0])');
    assert.match(await motionMessage(),/正在前往/);
    await delay(180);
    await evaluate('feedback([15,40,40,0,0,0])');
    assert.match(await motionMessage(),/已到达/);
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),false);
  });
  await run("invalid speed and rejected commands cannot report successful motion",async()=>{
    const before=await paths();
    for(const invalid of ["","1e1","Infinity","0x10","4","81"]) await evaluate('edit("#wpRunSpeed",'+JSON.stringify(invalid)+');click("#btnWpRun")');
    assert.equal(await paths(),before);
    await evaluate('edit("#wpRunSpeed","20");testStation.failPath=true;click("#btnWpRun")');
    await wait('document.getElementById("wpMotionState").dataset.state==="error"');
    assert.match(await motionMessage(),/前往失败/);
    await evaluate('testStation.failPath=false');
  });
  await run("rapid autosaves preserve newer values and the focused editor input",async()=>{
    await select(0);
    const before=await evaluate('testStation.saves.length');
    await evaluate('testStation.holdSaves=true;edit("[data-point-name]","取料新位置");window.jointInput=document.querySelector("[data-point-q=\\"0\\"]");jointInput.focus();edit("[data-point-q=\\"0\\"]","12.5")');
    assert.equal(await evaluate('testStation.saves.length'),before+1);
    await evaluate('testStation.ack()');
    await wait('testStation.saves.length==='+String(before+2));
    assert.equal(await evaluate('testStation.saves.at(-1)[0].q[0]'),12.5);
    await evaluate('testStation.ack();testStation.holdSaves=false');
    await saved();
    assert.equal(await evaluate('document.activeElement===jointInput && jointInput.isConnected'),true);
    assert.equal(await evaluate('document.querySelector("[data-point-select=\\"0\\"] b").textContent'),"取料新位置");
  });
  await run("save errors remain visible and block motion until retry succeeds",async()=>{
    const before=await paths();
    await evaluate('testStation.holdSaves=true;edit("[data-point-q=\\"1\\"]","41");testStation.ack(false)');
    await wait('document.getElementById("wpSaveState").dataset.state==="error"');
    await evaluate('click("#btnWpRun");testStation.ack(false)');
    await wait('!document.getElementById("btnWpRun").disabled');
    assert.equal(await paths(),before);
    await evaluate('click("#btnWpRun");testStation.ack();testStation.holdSaves=false');
    await wait('document.getElementById("wpMotionState").dataset.state==="moving"');
    await saved();
    await evaluate('click("#btnWpStop")');
    await wait('!document.getElementById("btnWpRun").disabled');
    await evaluate('testStation.holdSaves=true;edit("[data-point-q=\\"1\\"]","42");testStation.ack();testStation.holdSaves=false');
    await saved();
  });
  await run("invalid or duplicate edits do not overwrite saved values",async()=>{
    const before=await evaluate('testStation.saves.length');
    await evaluate('edit("[data-point-name]","放料点");edit("[data-point-q=\\"0\\"]","")');
    assert.equal(await evaluate('testStation.saves.length'),before);
    assert.equal(await evaluate('document.querySelector("[data-point-name]").value'),"取料新位置");
    assert.equal(await evaluate('Number(jointInput.value)'),12.5);
  });
  await run("angles reject scientific notation, malformed decimals, blanks and out-of-range values",async()=>{
    const before=await evaluate('testStation.saves.length');
    for (const invalid of ["1e2","-1E-1","NaN","Infinity","0x10","1..2","--1","","-",".","1,2","361"]) {
      await evaluate('edit("[data-point-q=\\"0\\"]",'+JSON.stringify(invalid)+')');
      assert.equal(await evaluate('Number(document.querySelector("[data-point-q=\\"0\\"]").value)'),12.5,invalid);
    }
    assert.equal(await evaluate('testStation.saves.length'),before);
    await evaluate('window.numberInput=document.querySelector("[data-point-q=\\"0\\"]");numberInput.focus();numberInput.select()');
    for (const key of ["e","E","+"]) {
      await cdp("Input.dispatchKeyEvent",{type:"keyDown",key,text:key});
      await cdp("Input.dispatchKeyEvent",{type:"keyUp",key});
      assert.equal(await evaluate('numberInput.value'),"12.5");
    }
    await evaluate('const badPaste=new DataTransfer();badPaste.setData("text","1e3");numberInput.dispatchEvent(new ClipboardEvent("paste",{clipboardData:badPaste,bubbles:true,cancelable:true}))');
    assert.equal(await evaluate('numberInput.value'),"12.5");
    await evaluate('const valid=new DataTransfer();valid.setData("text","  -12.5  ");numberInput.dispatchEvent(new ClipboardEvent("paste",{clipboardData:valid,bubbles:true,cancelable:true}));numberInput.dispatchEvent(new Event("change"))');
    await saved();
    assert.equal(await evaluate('testStation.points[0].q[0]'),-12.5);
    await evaluate('edit("[data-point-q=\\"0\\"]","12.5")');
    await saved();
    await evaluate('const native=document.getElementById("gripDeg");native.focus();window.nativeValue=native.value');
    await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"e",text:"e"});
    await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"e"});
    assert.equal(await evaluate('document.getElementById("gripDeg").value===nativeValue'),true);
  });
  await run("program numeric parameters reject blanks and fractional loop counts",async()=>{
    await evaluate('click("#btnProgAdd");click("[data-add-type=wait]");edit("[data-parameter=\\"seconds\\"]","")');
    assert.equal(await evaluate('Number(document.querySelector("[data-parameter=\\"seconds\\"]").value)'),0.5);
    await evaluate('click("#btnProgAdd");click("[data-add-type=loop]");edit("[data-parameter=\\"count\\"]","2.5")');
    assert.equal(await evaluate('Number(document.querySelector("[data-parameter=\\"count\\"]").value)'),2);
  });
  await run("an older catalog response cannot replace a newer edit",async()=>{
    await evaluate('jointInput.blur();testStation.holdCatalog=true;click("#btnProgRefresh");edit("[data-point-name]","最新取料点")');
    await wait('testStation.points[0].name==="最新取料点"');
    await evaluate('testStation.holdCatalog=false;testStation.catalogs.shift()()');
    await wait('document.getElementById("wpAck").textContent.includes("已刷新")');
    assert.equal(await evaluate('document.querySelector("[data-point-name]").value'),"最新取料点");
  });
  await run("recording the current pose automatically names and selects the new point",async()=>{
    const before=await paths();
    await evaluate('click("#btnWpAdd")');
    await wait('testStation.points.length===4 && !document.getElementById("btnWpAdd").disabled');
    assert.deepEqual(await evaluate('testStation.points[3]'),{name:"路点_4",q:[1,42,43,4,5,6],t:2});
    assert.equal(await evaluate('document.getElementById("wpRunTarget").textContent'),"路点_4");
    assert.equal(await paths(),before);
  });
  await run("manual creation starts empty and Tab saves exactly once after all six axes",async()=>{
    const before=await evaluate('testStation.saves.length');
    await evaluate('click("#btnWpManual")');
    assert.equal(await evaluate('document.querySelectorAll("[data-point-q]").length'),6);
    assert.equal(await evaluate('Array.from(document.querySelectorAll("[data-point-q]")).every(x=>x.value==="")'),true);
    assert.equal(await evaluate('document.getElementById("btnWpDraftTarget")'),null);
    assert.equal(await evaluate('document.getElementById("btnWpManual").textContent'),"手动添加");
    assert.equal(await evaluate('testStation.saves.length'),before);
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
    await screenshot("waypoints-manual");
    await cdp("Input.insertText",{text:"11"});
    await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
    await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
    assert.equal(await evaluate('document.activeElement.dataset.pointQ'),"1");
    assert.equal(await evaluate('testStation.saves.length'),before);
    assert.equal(await evaluate('testStation.points.length'),4);
    assert.match(await evaluate('document.getElementById("wpSaveState").textContent'),/1\/6/);
    await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Tab",code:"Tab",windowsVirtualKeyCode:9,modifiers:8});
    await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Tab",code:"Tab",windowsVirtualKeyCode:9,modifiers:8});
    assert.equal(await evaluate('document.activeElement.dataset.pointQ'),"0");
    await evaluate('edit("[data-point-name]","手动位置")');
    await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
    await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
    for(let j=1;j<6;j++) {
      assert.equal(await evaluate('document.activeElement.dataset.pointQ'),String(j));
      await cdp("Input.insertText",{text:String(11+10*j)});
      assert.equal(await evaluate('testStation.saves.length'),before);
      await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
      await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Tab",code:"Tab",windowsVirtualKeyCode:9});
      if(j<5)assert.equal(await evaluate('testStation.saves.length'),before);
    }
    await wait('testStation.points.length===5');
    assert.deepEqual(await evaluate('testStation.points[4].q'),[11,21,31,41,51,61]);
    assert.equal(await evaluate('testStation.points[4].name'),"手动位置");
    await saved();
    assert.equal(await evaluate('testStation.saves.length'),before+1);
    await evaluate('click("#btnWpManual");document.querySelector("[data-point-q]").dispatchEvent(new KeyboardEvent("keydown",{key:"Enter",bubbles:true,cancelable:true}))');
    assert.equal(await evaluate('testStation.points.length'),5);
    assert.equal(await evaluate('document.activeElement.dataset.pointQ'),"1");
    await evaluate('edit("[data-point-q=\\"5\\"]","0");edit("[data-point-q=\\"0\\"]","0");edit("[data-point-q=\\"1\\"]","20");edit("[data-point-q=\\"2\\"]","30");edit("[data-point-q=\\"3\\"]","0")');
    assert.equal(await evaluate('testStation.points.length'),5);
    await evaluate('edit("[data-point-q=\\"4\\"]","0")');
    await wait('testStation.points.length===6');
    assert.deepEqual(await evaluate('testStation.points[5].q'),[0,20,30,0,0,0]);
  });
  await run("cancel is colored; untouched drafts cancel directly and edits require confirmation",async()=>{
    await evaluate('click("#btnWpManual")');
    assert.equal(await evaluate('document.getElementById("btnWpDraftCancel").classList.contains("destructive-subtle")'),true);
    await evaluate('click("#btnWpDraftCancel")');
    assert.equal(await evaluate('document.getElementById("confirmDialog").open'),false);
    await evaluate('click("#btnWpManual");const draftInput=document.querySelector("[data-point-q]");draftInput.value="8";draftInput.dispatchEvent(new Event("input",{bubbles:true}))');
    await pointerClick("#btnWpDraftCancel");
    assert.equal(await evaluate('document.getElementById("confirmDialog").open'),true);
    await evaluate('click("#confirmCancel")');
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
    const beforeCancel=await evaluate('testStation.saves.length');
    await evaluate('document.querySelector("[data-point-q=\\"5\\"]").focus();for(let j=1;j<6;j++){const field=document.querySelector("[data-point-q=\\""+j+"\\"]");field.value="20";field.dispatchEvent(new Event("input",{bubbles:true}));}');
    await pointerClick("#btnWpDraftCancel");
    assert.equal(await evaluate('testStation.saves.length'),beforeCancel);
    await evaluate('click("#confirmAccept")');
    await wait('document.getElementById("wpEditor").hidden');
    assert.equal(await evaluate('testStation.points.length'),6);
  });
  await run("blank or invalid final axes block completion instead of using zero",async()=>{
    const before=await evaluate('testStation.saves.length');
    await evaluate('click("#btnWpManual");for(let j=0;j<5;j++)edit("[data-point-q=\\""+j+"\\"]",String(j===0?18:20))');
    for(const invalid of ["","-","1e2","361"]) {
      await evaluate('edit("[data-point-q=\\"5\\"]",'+JSON.stringify(invalid)+')');
      assert.equal(await evaluate('testStation.saves.length'),before);
      assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
    }
    await evaluate('const finalAxis=document.querySelector("[data-point-q=\\"5\\"]");finalAxis.focus();finalAxis.value="0";finalAxis.dispatchEvent(new Event("input",{bubbles:true}))');
    assert.equal(await evaluate('testStation.saves.length'),before);
    await cdp("Input.dispatchKeyEvent",{type:"keyDown",key:"Enter",code:"Enter",windowsVirtualKeyCode:13});
    await cdp("Input.dispatchKeyEvent",{type:"keyUp",key:"Enter",code:"Enter",windowsVirtualKeyCode:13});
    await wait('testStation.points.length===7');
    assert.equal(await evaluate('testStation.points[6].q[0]'),18);
    assert.equal(await evaluate('document.getElementById("btnWpDraftCancel")'),null);
    await saved();
    await evaluate('click("#btnWpDelete");click("#confirmAccept")');
    await wait('testStation.points.length===6');
  });
  await run("selection changes and stop requests cancel motion waiting on an autosave",async()=>{
    const before=await paths();
    await select(0);
    await evaluate('testStation.holdSaves=true;edit("[data-point-q=\\"0\\"]","13");click("#btnWpRun")');
    await select(1);
    await evaluate('testStation.ack();testStation.holdSaves=false');
    await wait('document.getElementById("wpAck").textContent.includes("目标路点已变化")');
    assert.equal(await paths(),before);
    await evaluate('testStation.holdSaves=true;edit("[data-point-q=\\"0\\"]","16");click("#btnWpRun");click("#btnWpStop");testStation.ack();testStation.holdSaves=false');
    await wait('!document.getElementById("btnWpRun").disabled');
    assert.equal(await paths(),before);
  });
  await run("disconnect and stop never masquerade as arrival",async()=>{
    await evaluate('click("#btnWpRun")');
    await wait('document.getElementById("wpMotionState").dataset.state==="moving"');
    await evaluate('feedback([0,40,40,0,0,0],false,{arm:{online:false}})');
    assert.match(await motionMessage(),/状态未知/);
    await evaluate('feedback([0,40,40,0,0,0]);click("#btnWpRun")');
    await wait('document.getElementById("wpMotionState").dataset.state==="moving"');
    await evaluate('click("#btnWpStop")');
    await wait('document.getElementById("wpMotionState").textContent.includes("已停止")');
    await evaluate('feedback([16,40,40,0,0,0])');
    assert.doesNotMatch(await motionMessage(),/已到达/);
  });
  await run("late acceptance cannot revive a stopped motion, and missing feedback becomes unknown",async()=>{
    await evaluate('testStation.holdPaths=true;click("#btnWpRun")');
    await wait('document.getElementById("wpMotionState").dataset.state==="waiting"');
    await evaluate('click("#btnWpStop")');
    await wait('document.getElementById("wpMotionState").textContent.includes("已停止")');
    await evaluate('testStation.reply(testStation.waitingPaths.shift());testStation.holdPaths=false');
    await wait('!document.getElementById("btnWpRun").disabled');
    assert.doesNotMatch(await motionMessage(),/正在前往|已到达/);
    await evaluate('click("#btnWpRun")');
    await wait('document.getElementById("wpMotionState").textContent.includes("反馈已中断")');
    assert.match(await motionMessage(),/状态未知/);
  });
  await run("delete cancellation preserves data; confirmed deletion clears the shared target",async()=>{
    const before=await evaluate('testStation.saves.length');
    await select(0);
    await evaluate('click("#btnWpDelete")');
    assert.equal(await evaluate('document.activeElement.id'),"confirmCancel");
    await screenshot("waypoint-delete-confirm");
    await evaluate('click("#confirmCancel")');
    assert.equal(await evaluate('testStation.saves.length'),before);
    await evaluate('click("#btnWpDelete");click("#confirmAccept")');
    await wait('testStation.points.length===5');
    assert.equal(await evaluate('document.getElementById("wpRunTarget").textContent'),"尚未选择");
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
  });
  await run("mobile layout fits and keeps execution visible while scrolling",async()=>{
    await cdp("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await select(0);
    await screenshot("waypoints-mobile");
    assert.equal(await evaluate('document.documentElement.scrollWidth <= innerWidth'),true);
    await evaluate('scrollTo(0,document.body.scrollHeight)');
    assert.equal(await evaluate('document.getElementById("btnWpRun").getBoundingClientRect().bottom<=innerHeight'),true);
    await evaluate('click("#btnWpClear");click("#confirmCancel")');
    assert.equal(await evaluate('testStation.points.length'),5);
    await evaluate('click("#btnWpClear");click("#confirmAccept")');
    await wait('testStation.points.length===0');
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled && document.getElementById("btnWpClear").disabled'),true);
  });
  await run("legacy import keeps its data but does not select a motion target",async()=>{
    await evaluate(`click("#btnWpManual");const invalid=new DataTransfer();invalid.items.add(new File(["{bad json"],"points.json",{type:"application/json"}));const invalidInput=document.getElementById("wpImportFile");invalidInput.files=invalid.files;invalidInput.dispatchEvent(new Event("change"));`);
    await wait('!document.getElementById("btnWpManual").disabled && document.getElementById("wpAck").classList.contains("tone-bad")');
    assert.equal(await evaluate('document.querySelectorAll("[data-point-name]").length'),0);
    await evaluate(`const transfer=new DataTransfer();transfer.items.add(new File([JSON.stringify({waypoints:[{name:"导入位置",q:[2,40,40,0,0,0],t:17}]})],"points.json",{type:"application/json"}));const input=document.getElementById("wpImportFile");input.files=transfer.files;input.dispatchEvent(new Event("change"));`);
    await wait('testStation.points.length===1 && document.getElementById("wpAck").textContent.includes("已导入")');
    assert.equal(await evaluate('testStation.points[0].t'),17);
    assert.equal(await evaluate('document.getElementById("btnWpRun").disabled'),true);
    await select(0);
    await evaluate('click("#btnWpRun")');
    assert.deepEqual(await lastPath(),{t:"path",q:[[2,40,40,0,0,0]],speed:20});
  });
};
