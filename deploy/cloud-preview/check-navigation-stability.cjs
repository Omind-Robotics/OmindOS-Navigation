'use strict';
// Runs real navigation in an isolated backend runtime. Faults delay/drop responses, never poses/results.
const fs = require('fs'), assert = require('assert');
const {chromium} = require(process.env.OMINDOS_PLAYWRIGHT || 'playwright');
const base = process.env.CHECK_PREVIEW_URL || 'https://cloud.omindos.cn/';
const out = process.env.CHECK_OUTPUT || '/tmp/omindos-navigation-stability';
const creds = process.env.CHECK_AUTH_FILE ? JSON.parse(fs.readFileSync(process.env.CHECK_AUTH_FILE)) : undefined;
const scrub=value=>String(value).replace(/Authorization[^\r\n]*/gi,'Authorization: [REDACTED]').replace(/Basic [A-Za-z0-9+/=]+/g,'Basic [REDACTED]');
// route.fetch uses Node's resolver, so the loopback-only candidate API is
// fetched directly rather than relying on Chrome's test host mapping.
const taskResponse=route=>route.fetch(process.env.CHECK_BACKEND_URL?{
  url:process.env.CHECK_BACKEND_URL+new URL(route.request().url()).pathname.slice(new URL(base).pathname.length),timeout:5000
}:{timeout:5000});
fs.mkdirSync(out, {recursive:true});
const report = {base, checks:[], runs:[], requests:[], page_errors:[]};
let browser, page;
async function waitReady() { await page.waitForFunction(() => !!profile && !!currentPose && !operation && readyRevision===revision && sceneEngine?.meshStats.loaded===20 && sceneEngine.meshStats.failed===0, null, {timeout:60000}); }
async function result(timeout=30000) {
  await page.waitForFunction(() => !operation && !navigationJob, null, {timeout});
  return page.evaluate(() => ({status:document.getElementById('navigation-status').textContent,
    ui:document.getElementById('status').textContent, stopped:emergencyStop, epoch:motionEpoch,
    ticket:motionTicket, base:currentPose.base_pose, parked:!!navigationData?.parking_confirmed}));
}
async function navigate(x,y=0,yaw=0) {
  await page.fill('#navigation-x',String(x)); await page.fill('#navigation-y',String(y));
  await page.fill('#navigation-yaw',String(yaw)); await page.click('#navigation-run');
}
(async()=>{try{
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'/usr/bin/google-chrome',headless:true,
    args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader',
      ...(new URL(base).port==='19443'?['--host-resolver-rules=MAP cloud.omindos.cn 127.0.0.1']:[])]});
  const context=await browser.newContext({viewport:{width:1440,height:1000},
    ...(creds?{extraHTTPHeaders:{Authorization:'Basic '+Buffer.from(creds.username+':'+creds.password).toString('base64')}}:{})});
  page=await context.newPage();page.setDefaultTimeout(15000);
  const started=new WeakMap();let navigationStarts=0;
  page.on('request',r=>{started.set(r,Date.now());if(r.method()==='POST'&&r.url().endsWith('/v1/navigation-start'))navigationStarts++;});
  page.on('response',r=>{if(/\/v1\/(navigation|workbench-state|workbench-stop)/.test(r.url()))
    report.requests.push({path:new URL(r.url()).pathname,code:r.status(),ms:Date.now()-started.get(r.request())});});
  page.on('pageerror',e=>report.page_errors.push(scrub(e)));
  await page.goto(base,{waitUntil:'domcontentloaded'});await waitReady();
  if(await page.locator('.cloud-layout-tabs [data-panel=navigation]').count()) await page.click('.cloud-layout-tabs [data-panel=navigation]');
  report.model=await page.evaluate(()=>({parts:sceneEngine?.meshStats,webgl:!!sceneEngine,profile:profile.profile_mode}));
  if(process.argv.includes('--baseline')){
    // The task response is delayed while independent safety-state GETs remain healthy.
    let held=false;
    await page.route('**/v1/navigation-task/*',async route=>{
      try{const response=await taskResponse(route);
        if(!held){held=true;await new Promise(r=>setTimeout(r,3400));}
        await route.fulfill({response});
      }catch(e){await route.abort('failed').catch(()=>{});}
    });
    await navigate(.9,.1,20); report.runs.push({case:'slow_task_response',...await result(30000)});
    report.status='observed';
    await page.screenshot({path:out+'/baseline-desktop.png'});
  }else{
    for(const [width,height] of process.argv.includes('--faults')||process.argv.includes('--network')?[]:[[1440,1000],[390,844],[844,390]]){
      await page.setViewportSize({width,height});
      for(const [x,y,yaw] of [[.55,0,0],[.8,.15,20],[.45,0,0]]){
        await navigate(x,y,yaw);const r=await result();
        assert(r.status.includes('已到位并确认停车'),JSON.stringify(r));assert(r.parked&&!r.stopped);
        report.runs.push({case:'continuous',viewport:[width,height],...r});
      }
      assert(await page.locator('#scene').isVisible());
      await page.screenshot({path:out+'/navigation-'+width+'x'+height+'.png'});
      report.checks.push(width+'x'+height+' three consecutive navigations');
      console.log('PASS '+width+'x'+height+' consecutive navigation');
    }
    const pass=name=>{report.checks.push(name);console.log('PASS '+name);};
    const assertParked=r=>{assert(r.status.includes('已到位并确认停车'),JSON.stringify(r));assert(r.parked&&!r.stopped);};
    if(process.argv.includes('--faults')||process.argv.includes('--network')){
      async function delayTaskOnce(ms){
        let held=false;
        await page.route('**/v1/navigation-task/*',async route=>{
          try{const response=await taskResponse(route);
            if(!held){held=true;await new Promise(resolve=>setTimeout(resolve,ms));}
            await route.fulfill({response});
          }catch(e){await route.abort('failed').catch(()=>{});}
        });
      }
      if(!process.argv.includes('--network')){
      await delayTaskOnce(3400);
      await navigate(.8,.1,20);let r=await result();assertParked(r);
      report.runs.push({case:'slow_task_response',...r});pass('3.4s result delay with independent heartbeat reaches confirmed parking');
      await page.unroute('**/v1/navigation-task/*');

      let dropped=false;
      await page.route('**/v1/navigation-task/*',async route=>{
        if(!dropped){dropped=true;return route.abort('connectionreset');}await route.continue();
      });
      const startsBeforeRetry=navigationStarts;
      await navigate(.55,0,0);r=await result();assertParked(r);
      assert.equal(navigationStarts,startsBeforeRetry+1,'read retry must never replay navigation-start');
      report.runs.push({case:'read_retry',...r});pass('one failed result GET retries without replaying navigation-start');
      await page.unroute('**/v1/navigation-task/*');

      let unavailable=false;const startsBeforeProxyError=navigationStarts;
      await page.route('**/v1/navigation-task/*',route=>{
        if(!unavailable){unavailable=true;return route.fulfill({status:503,contentType:'text/html',body:'<html>upstream temporarily unavailable</html>'});}
        return route.continue();
      });
      await navigate(.65,0,0);r=await result();assertParked(r);
      assert.equal(navigationStarts,startsBeforeProxyError+1,'proxy-error retry must never replay navigation-start');
      report.runs.push({case:'proxy_html_retry',...r});pass('one HTML HTTP503 result response retries and recovers without duplicate start');
      await page.unroute('**/v1/navigation-task/*');

      await delayTaskOnce(11000);
      const startsBeforeTimeout=navigationStarts;
      await navigate(.75,0,0);r=await result();assertParked(r);
      assert.equal(navigationStarts,startsBeforeTimeout+1,'read timeout must never replay navigation-start');
      report.runs.push({case:'bounded_read_timeout',...r});pass('hung result request times out and safe GET retry recovers');
      await page.unroute('**/v1/navigation-task/*');

      await delayTaskOnce(3400);
      await navigate(1.6,0,0);
      await page.waitForFunction(()=>!!navigationJob);
      await page.click('#navigation-stop');
      await page.waitForFunction(()=>!operation&&!navigationJob);
      const canceled=await page.evaluate(()=>({...currentPose.base_pose}));
      await page.waitForTimeout(3700);
      assert.deepStrictEqual(await page.evaluate(()=>currentPose.base_pose),canceled);
      assert((await page.locator('#navigation-status').innerText()).includes('取消'));
      pass('cancel aborts pending read and delayed response never updates pose');
      await page.unroute('**/v1/navigation-task/*');
      await navigate(.7,0,0);assertParked(await result());pass('explicit restart after cancellation works');

      await delayTaskOnce(3400);
      await navigate(1.6,0,0);await page.waitForFunction(()=>!!navigationJob);
      await page.click('#stage-stop');
      await page.waitForFunction(()=>emergencyStop&&!safetyPending&&!operation);
      const frozen=await page.evaluate(()=>({...currentPose.base_pose}));
      await page.waitForTimeout(3700);
      assert.deepStrictEqual(await page.evaluate(()=>currentPose.base_pose),frozen);
      await page.click('#reset-stop');await page.waitForFunction(()=>!emergencyStop&&!safetyPending);
      await page.waitForTimeout(600);
      assert(await page.evaluate(()=>!navigationJob&&!animation&&!operation));
      assert.deepStrictEqual(await page.evaluate(()=>currentPose.base_pose),frozen);
      pass('estop freezes pose; reset never resumes old navigation');
      await page.unroute('**/v1/navigation-task/*');
      await navigate(.6,0,0);assertParked(await result());pass('new explicit navigation after reset succeeds');

      await navigate(100,0,0);r=await result();
      assert(r.status.includes('导航拒绝'),JSON.stringify(r));
      report.runs.push({case:'planner_rejection',...r});
      await navigate(.75,0,0);assertParked(await result());pass('planner rejects out-of-map goal and subsequent valid request succeeds');
      }

      await navigate(1.6,0,0);await page.waitForFunction(()=>!!navigationJob);
      const taskId=await page.evaluate(()=>navigationJob);
      await context.setOffline(true);
      await page.waitForFunction(()=>emergencyStop&&!safetyPending&&!operation,null,{timeout:15000});
      // Legacy task polling renews the lease, so the independent observer must
      // not accidentally keep a disconnected client alive during this check.
      await page.waitForTimeout(3500);
      await context.setOffline(false);
      const deadline=Date.now()+5000;let state;
      do{state=await (await context.request.get((process.env.CHECK_BACKEND_URL||base)+'v1/navigation-task/'+taskId,{timeout:5000})).json();if(['failed','canceled'].includes(state.status))break;await page.waitForTimeout(100);}while(Date.now()<deadline);
      assert(['failed','canceled'].includes(state.status),JSON.stringify(state));
      assert(Object.values(state.navigation.cmd_vel).every(v=>v===0));
      assert(!state.navigation.parking_confirmed);
      pass('real browser network loss stops backend with zero motion and no parking claim');
      await page.waitForTimeout(1000);
      assert(await page.evaluate(()=>emergencyStop&&!navigationJob));
      await page.click('#reset-stop');await page.waitForFunction(()=>!emergencyStop&&!safetyPending);
      await navigate(.65,0,0);assertParked(await result());pass('network recovery still requires manual reset and a new start');
      await page.screenshot({path:out+'/fault-recovery-desktop.png'});
    }
    assert.deepStrictEqual(report.page_errors,[]);
    report.status='passed';
  }
}catch(e){report.status='failed';report.error=scrub(e);process.exitCode=1;
  if(page){report.final=await result(1000).catch(()=>null);await page.screenshot({path:out+'/failure.png'}).catch(()=>{});}
}finally{fs.writeFileSync(out+'/report.json',JSON.stringify(report,null,2));
  if(page)await page.unrouteAll({behavior:'ignoreErrors'}).catch(()=>{});
  if(browser)await browser.close();
  console.log(JSON.stringify({...report,requests:undefined},null,2));
}})();
