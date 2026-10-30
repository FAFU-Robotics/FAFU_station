"use strict";
const assert = require("node:assert/strict");

// Uses the real browser app and a local simulated station; never hardware.
module.exports = async function({evaluate, wait, run, delay, cdp, screenshot}) {
  await evaluate(`
    location.hash="#/arm/control";
    window.manualCommands=[];window.manualSamples=[];window.manualErrors=[];
    const originalSend=WebSocket.prototype.send;
    WebSocket.prototype.send=function(data){
      if(typeof data==="string")try{const command=JSON.parse(data);if(["arm","home","park","cart_go"].includes(command.t)){manualCommands.push(command);window.manualSocket=this;}}catch(_){}
      return originalSend.call(this,data);
    };
    addEventListener("error",e=>manualErrors.push(e.message));
    addEventListener("unhandledrejection",e=>manualErrors.push(String(e.reason)));
    addEventListener("station:snapshot",e=>{const arm=e.detail.arm;if(arm)manualSamples.push({t:e.detail.t_mono*1000,q:arm.feedback_deg||arm.q_deg});});
    window.manualGoal=q=>q.forEach((value,i)=>{const field=document.querySelector("[data-jn='"+i+"']");field.value=value;field.dispatchEvent(new Event("input",{bubbles:true}));});
    window.manualSpeed=value=>{const field=document.getElementById("armSpeed");field.value=value;field.dispatchEvent(new Event("input",{bubbles:true}));};
  `);
  await wait('document.getElementById("page-arm").classList.contains("on") && !document.getElementById("btnSendArm").disabled && !StationUI.snapshot().arm.moving');

  await run("manual no-op validates before sending any control command", async()=>{
    await evaluate('manualGoal(StationUI.snapshot().arm.feedback_deg);manualCommands.length=0;document.getElementById("btnSendArm").click()');
    assert.deepEqual(await evaluate('manualCommands'), []);
    assert.match(await evaluate('document.getElementById("armAck").textContent'), /已在目标位置/);
    assert.equal(await evaluate('document.getElementById("armAck").classList.contains("tone-ok")'), true);
  });

  await run("old measured feedback cannot suppress a different current target",async()=>{
    await evaluate('window.auditGoal=StationUI.snapshot().arm.q_deg.slice();auditGoal[0]+=12;StationUI.snapshot().arm.feedback_deg=auditGoal.slice();manualGoal(auditGoal);manualCommands.length=0;document.getElementById("btnSendArm").click()');
    assert.equal(await evaluate('manualCommands.filter(command=>command.t==="arm").length'),1,"stale feedback must not be mistaken for the current pose");
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.feedback_deg[0]-auditGoal[0])<0.2');
  });

  await run("joint follow keeps the live TCP readout updating",async()=>{
    await evaluate('window.liveYaw=StationUI.snapshot().arm.feedback_deg[0]+5;document.getElementById("armFollow").click();manualGoal([liveYaw,40,40,0,0,0])');
    await wait('Math.abs(StationUI.snapshot().arm.q_deg[0]-liveYaw)<0.2');
    await delay(350);
    assert.ok(Math.abs(Number(await evaluate('document.getElementById("tcpYaw").textContent'))-await evaluate('liveYaw'))<0.3,"RPY readout must update during joint follow");
    await evaluate('document.getElementById("armFollow").click()');
    await wait('!StationUI.snapshot().arm.moving');
  });

  await run("home then send stays at home on no-op and interpolates the next target", async()=>{
    await evaluate('manualSpeed(40);manualGoal([30,40,40,0,0,0]);document.getElementById("btnSendArm").click()');
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.feedback_deg[0]-30)<0.2');
    await evaluate('document.getElementById("btnHome").click()');
    await wait('!StationUI.snapshot().arm.moving && StationUI.snapshot().arm.feedback_deg.every(q=>Math.abs(q)<0.1)');
    await evaluate('manualGoal([0,0,0,0,0,0]);manualCommands.length=0;document.getElementById("btnSendArm").click()');
    assert.deepEqual(await evaluate('manualCommands'), []);
    await delay(200);
    assert.ok(await evaluate('StationUI.snapshot().arm.feedback_deg.every(q=>Math.abs(q)<0.1)'));
    await evaluate('manualSpeed(10);manualGoal([15,0,0,0,0,0]);document.getElementById("btnSendArm").click()');
    assert.deepEqual(await evaluate('manualCommands.map(command=>command.t)'), ["arm"]);
    await delay(150);
    const midway=await evaluate('StationUI.snapshot().arm.feedback_deg[0]');
    assert.ok(midway<5, "sending a target must not teleport the simulated arm");
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.feedback_deg[0]-15)<0.2');
  });

  await run("a moving target can be replaced immediately, including by the current pose", async()=>{
    await evaluate('manualGoal([60,0,0,0,0,0]);document.getElementById("btnSendArm").click()');
    await wait('StationUI.snapshot().arm.moving && StationUI.snapshot().arm.feedback_deg[0]>16');
    await evaluate('manualCommands.length=0;manualGoal(StationUI.snapshot().arm.feedback_deg);document.getElementById("btnSendArm").click()');
    assert.equal(await evaluate('manualCommands.filter(command=>command.t==="arm").length'), 1);
    await evaluate('manualSamples.length=0;manualGoal([-20,0,0,0,0,0]);document.getElementById("btnSendArm").click()');
    await wait('Math.abs(StationUI.snapshot().arm.target_deg[0]+20)<0.2');
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.feedback_deg[0]+20)<0.2');
    assert.equal(await evaluate('manualCommands.some(command=>command.t==="park")'), false);
    const samples=await evaluate('manualSamples');
    assert.ok(samples.length>=3);
    for(let i=1;i<samples.length;i++){
      const maxStep=10*(samples[i].t-samples[i-1].t)/1000+0.5;
      assert.ok(Math.abs(samples[i].q[0]-samples[i-1].q[0])<=maxStep, "replacement must respect the configured speed: "+JSON.stringify({previous:samples[i-1],current:samples[i],maxStep}));
      assert.ok(samples[i].q[0]<30, "the superseded goal must never run to completion");
    }
  });
  await require("./cartesian_scenarios.js")({evaluate,wait,run,delay,cdp,screenshot});
  assert.deepEqual(await evaluate('manualErrors'), []);
};
