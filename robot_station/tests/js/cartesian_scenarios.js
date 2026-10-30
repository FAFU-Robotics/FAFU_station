"use strict";
const assert=require("node:assert/strict");

module.exports=async function({evaluate,wait,run,delay,cdp,screenshot}){
  const count=()=>evaluate('manualCommands.filter(command=>command.t==="cart_go").length');
  const followCount=()=>evaluate('manualCommands.filter(command=>command.t==="cart_go" && command.stream).length');
  const targets=()=>evaluate('Array.from(document.querySelectorAll("[data-cn]")).map(field=>field.value)');
  await evaluate(`
    manualSpeed(40);manualGoal([0,40,40,0,0,0]);document.getElementById("btnSendArm").click();
    window.cartTarget=(index,value)=>{const field=document.querySelector("[data-cn='"+index+"']");field.value=value;field.dispatchEvent(new Event("input",{bubbles:true}));field.dispatchEvent(new Event("change",{bubbles:true}));};
    window.cartCurrent=()=>{const arm=StationUI.snapshot().arm;return [...arm.ee_m,...arm.ee_rpy_deg];};
  `);
  await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.feedback_deg[0])<0.1 && Math.abs(StationUI.snapshot().arm.feedback_deg[1]-40)<0.1');
  await evaluate('document.querySelector("[data-space=cart]").click()');

  await run("Cartesian targets remain drafts until send and never track incoming feedback",async()=>{
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),false);
    assert.equal(await evaluate('document.getElementById("armFollow").closest(".space-panel")'),null);
    const before=await count();
    await evaluate('cartTarget(0,Number(document.querySelector("[data-cn=\\"0\\"]").value)-0.02)');
    const draft=await targets();
    await delay(250);
    assert.equal(await count(),before,"input/change must not send with follow off");
    await evaluate('StationUI.sendImmediate({t:"arm",q:[5,40,40,0,0,0],speed:40})');
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.feedback_deg[0]-5)<0.1');
    await delay(100);
    assert.deepEqual(await targets(),draft,"feedback must not overwrite a pending Cartesian target");
    assert.ok(Math.abs(Number(await evaluate('document.getElementById("tcpY").textContent'))-await evaluate('StationUI.snapshot().arm.ee_m[1]'))<0.002);
    await evaluate('document.getElementById("btnCartUseCurrent").click()');
    const current=await evaluate('cartCurrent().map((value,i)=>value.toFixed(i<3?3:1))');
    assert.deepEqual(await targets(),current);
    assert.equal(await count(),before,"copying the current pose must not start motion");
    await evaluate('cartTarget(0,"");document.getElementById("btnSendArm").click()');
    assert.equal(await count(),before,"a blank target must not become zero");
    await evaluate('document.getElementById("btnCartUseCurrent").click();window.explicitX=cartCurrent()[0]-0.02;cartTarget(0,explicitX);document.getElementById("btnSendArm").click()');
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.ee_m[0]-explicitX)<0.004');
    assert.equal(await count(),before+1);
  });

  await run("Cartesian follow starts only after an edit and coalesces pending IK requests",async()=>{
    const before=await followCount();
    await wait('!document.getElementById("armFollow").disabled');
    await evaluate('manualSpeed(10);document.getElementById("armFollow").click()');
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),true);
    await delay(150);
    assert.equal(await followCount(),before,"enabling follow alone must not send an old draft");
    await evaluate(`
      window.heldCartAcks=[];window.holdCartAcks=true;
      const original=manualSocket.onmessage;
      manualSocket.onmessage=function(event){const message=JSON.parse(event.data);if(holdCartAcks && message.request_id?.startsWith("cart-follow-")){heldCartAcks.push(event);return;}original.call(this,event);};
      window.releaseCartAcks=()=>{holdCartAcks=false;heldCartAcks.splice(0).forEach(event=>original.call(manualSocket,event));};
      window.followX=cartCurrent()[0]-0.015;cartTarget(0,followX);
    `);
    await wait('heldCartAcks.length===1');
    await evaluate('for(let i=0;i<100;i++)cartTarget(0,followX-(i+1)*0.0001);window.finalFollowX=Number(document.querySelector("[data-cn=\\"0\\"]").value)');
    await delay(250);
    assert.equal(await followCount(),before+1,"only one IK request may be outstanding");
    await evaluate('releaseCartAcks()');
    await wait('manualCommands.filter(command=>command.t==="cart_go" && command.stream).length==='+String(before+2));
    assert.ok(Math.abs(await evaluate('manualCommands.filter(command=>command.t==="cart_go" && command.stream).at(-1).xyz[0]-finalFollowX'))<1e-6);
    await wait('!StationUI.snapshot().arm.moving && Math.abs(StationUI.snapshot().arm.ee_m[0]-finalFollowX)<0.004');
    await evaluate('cartTarget(0,cartCurrent()[0]-0.03)');
    await wait('StationUI.snapshot().arm.moving');
    await evaluate('window.followStopStamp=StationUI.snapshot().t_mono;document.getElementById("armFollow").click()');
    await wait('!StationUI.snapshot().arm.moving && StationUI.snapshot().t_mono>followStopStamp');
    const stopped=await evaluate('StationUI.snapshot().arm.feedback_deg');
    await delay(250);
    const after=await evaluate('StationUI.snapshot().arm.feedback_deg');
    assert.ok(after.every((value,i)=>Math.abs(value-stopped[i])<0.2),"unchecking follow must hold the arm: "+JSON.stringify({stopped,after}));
    const sent=await followCount();
    await evaluate('cartTarget(0,cartCurrent()[0]-0.01)');await delay(150);
    assert.equal(await followCount(),sent);
    await evaluate('document.getElementById("armFollow").click();cartTarget(0,cartCurrent()[0]-0.01);location.hash="#/home"');
    await wait('document.getElementById("page-home").classList.contains("on")');
    const afterRoute=await followCount();await delay(200);
    assert.equal(await followCount(),afterRoute,"leaving the page must cancel queued follow updates");
    assert.equal(await evaluate('document.getElementById("armFollow").checked'),false);
    assert.equal(await evaluate('document.getElementById("armFollow").closest(".follow-row").classList.contains("is-on")'),false);
  });

  await captureLayout({evaluate,wait,run,cdp,screenshot});
};

async function captureLayout({evaluate,wait,run,cdp,screenshot}){
  await run("live XYZ and RPY use one colored readout beside the 3D view in both themes",async()=>{
    await evaluate('location.hash="#/arm/control"');
    await wait('document.getElementById("page-arm").classList.contains("on")');
    await evaluate('document.querySelector("[data-space=cart]").click()');
    await wait('!!document.querySelector("#arm3dHost canvas") && document.getElementById("arm3dOverlay").textContent.startsWith("CAD 网格")');
    assert.deepEqual(await evaluate('Array.from(document.querySelectorAll("#cartList label")).map(label=>label.textContent)'),["X","Y","Z","R","P","Y"]);
    assert.equal(await evaluate('document.getElementById("cartPose").closest("#panel-twin")!==null'),true);
    assert.doesNotMatch(await evaluate('document.getElementById("eeKv").textContent'),/X Y Z|R P Y/);
    await cdp("Emulation.setDeviceMetricsOverride",{width:1680,height:1100,deviceScaleFactor:1,mobile:false});
    // Include negative and wide values in visual review; follow is off.
    await evaluate('[-0.142,0.215,0.069,128,22.8,-161.2].forEach((value,i)=>{const input=document.querySelector("[data-cn=\\""+i+"\\"]");input.value=value;input.dispatchEvent(new Event("input",{bubbles:true}));input.dispatchEvent(new Event("change",{bubbles:true}));})');
    for(const theme of ["light","dark"]){
      await evaluate('StationTheme.set('+JSON.stringify(theme)+')');
      const colors=await evaluate('["tcpX","tcpY","tcpZ"].map(id=>getComputedStyle(document.getElementById(id)).color)');
      assert.equal(new Set(colors).size,3);
      await screenshot("cartesian-"+theme);
    }
    await cdp("Emulation.setDeviceMetricsOverride",{width:390,height:1000,deviceScaleFactor:1,mobile:false});
    assert.equal(await evaluate('document.documentElement.scrollWidth<=innerWidth+1'),true);
    await screenshot("cartesian-mobile");
  });
}
module.exports.captureLayout=captureLayout;
