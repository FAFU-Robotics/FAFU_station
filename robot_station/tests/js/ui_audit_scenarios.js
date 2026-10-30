"use strict";
const assert=require("node:assert/strict");

// Read-only browser review of the real application, backed by temporary simulation data.
module.exports=async function({evaluate,wait,cdp,screenshot,delay}){
  await evaluate('window.auditErrors=[];addEventListener("error",e=>auditErrors.push(e.message));addEventListener("unhandledrejection",e=>auditErrors.push(String(e.reason)))');
  const prefix=process.env.FAFU_UI_AUDIT;
  const layouts=[{name:"desktop",width:1440,height:1000,theme:"light"},{name:"tablet",width:1024,height:1000,theme:"light"},{name:"mobile",width:390,height:900,theme:"light"},{name:"dark",width:1440,height:1000,theme:"dark"}];
  const overflow=[];
  for(const layout of layouts){
    await cdp("Emulation.setDeviceMetricsOverride",{width:layout.width,height:layout.height,deviceScaleFactor:1,mobile:false});
    await evaluate('StationTheme.set('+JSON.stringify(layout.theme)+')');
    for(const page of ["home","arm/control","work/waypoints","work/trajectories","work/program"]){
      await evaluate('location.hash="#/'+page+'"');
      await wait('StationUI.page()==='+JSON.stringify(page));
      if(page==="arm/control"){
        await evaluate('document.querySelector("[data-space=joint]").click()');
        await wait('document.getElementById("arm3dOverlay").textContent.startsWith("CAD 网格")');
      }
      if(page==="work/program"){
        await wait('document.querySelector("[data-program]")');
        await evaluate('document.querySelector("[data-program]").click()');
        await wait('document.querySelector("#progTree [data-node]")');
      }
      await delay(100);
      const over=await evaluate('document.documentElement.scrollWidth>innerWidth+1');
      if(over)overflow.push({layout:layout.name,page});
      await screenshot(prefix+"-"+layout.name+"-"+page.replaceAll("/","-"));
      if(page==="arm/control"){
        await evaluate('document.querySelector("[data-space=cart]").click()');
        await screenshot(prefix+"-"+layout.name+"-cartesian");
      }
    }
  }
  console.log("UI_AUDIT "+JSON.stringify({overflow,errors:await evaluate('auditErrors')}));
  assert.deepEqual(overflow,[]);
  assert.deepEqual(await evaluate('auditErrors'),[]);
};
