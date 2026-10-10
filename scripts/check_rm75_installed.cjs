// Real customer meshes, real browser and real executable. No product state injection.
'use strict';
const fs=require('fs'),path=require('path'),assert=require('assert');
const {spawn}=require('child_process');
const {chromium}=require('playwright');
const executable=path.resolve(process.argv[2]),output=path.resolve(process.argv[3]);
fs.mkdirSync(output,{recursive:true});
const store=path.join(output,'state');fs.mkdirSync(store,{recursive:true});
const checks=[],errors=[],meshFailures=[];
let processHandle,browser,page;
function check(name){checks.push(name);console.log('PASS '+name);}
const report={scope:'actual_customer_CAD_browser_and_executable',checks,
  hardware_validated:false,dynamics_validated:false,meshFailures};
async function start(){
  let log='';processHandle=spawn(executable,['--no-browser','--port','0','--store',store],
    {cwd:output,env:{...process.env,PYTHONPATH:'',PYTHONHOME:''}});
  processHandle.stdout.on('data',d=>log+=d);processHandle.stderr.on('data',d=>log+=d);
  for(let i=0;i<600;i++){
    const m=log.match(/http:\/\/127\.0\.0\.1:\d+\//);
    if(m)return m[0].replace(/\/$/,'');
    if(processHandle.exitCode!==null)throw Error('Executable exited: '+log);
    await new Promise(r=>setTimeout(r,100));
  }
  throw Error('Executable startup timed out: '+log);
}
async function stop(){
  if(!processHandle||processHandle.exitCode!==null)return;
  const p=processHandle;p.kill('SIGTERM');
  await Promise.race([new Promise(r=>p.once('exit',r)),new Promise(r=>setTimeout(r,10000))]);
  if(p.exitCode===null)p.kill('SIGKILL');
}
async function wait(expression,seconds=60){
  const until=Date.now()+seconds*1000;
  while(Date.now()<until){if(await page.evaluate('('+expression+')'))return;await page.waitForTimeout(100);}
  throw Error(expression+' timed out: '+await page.locator('#status').innerText());
}
async function api(base,route,data){
  const r=await fetch(base+route,data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  assert(r.ok,route+' HTTP '+r.status);return r.json();
}
(async()=>{
try{
  let base=await start();
  browser=await chromium.launch({executablePath:process.env.CHROMIUM_EXECUTABLE||'/usr/bin/google-chrome',headless:true,
    args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  page=await browser.newPage({viewport:{width:1600,height:1000}});
  page.on('pageerror',e=>errors.push(String(e)));
  page.on('response',r=>{if(r.url().includes('/meshes/')&&!r.ok())meshFailures.push(r.url()+':'+r.status());});
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await wait('!operation && readyRevision===revision && !!profile');
  assert.equal(await page.locator('#cad-model option').count(),3);
  await page.selectOption('#cad-model','b');await page.click('#load-cad-model');
  await wait("!operation && profile.profile_mode==='kinematic_preview' && readyRevision===revision");
  await wait('sceneEngine && sceneEngine.meshStats.requested===20 && sceneEngine.meshStats.loaded===20 && sceneEngine.meshStats.failed===0',180);
  check('all_20_actual_CAD_visual_instances_rendered');
  assert.deepEqual(JSON.parse(await page.inputValue('#planning-mapping')).tcp_links,['left_tool0','right_tool0']);
  const tree=await page.evaluate('({links:model.links.length,joints:model.joints.length,meshes:sceneEngine.meshStats})');
  report.model=tree;check('actual_joint_and_TCP_mapping');
  await page.click('[data-focus="left"]');
  assert(await page.evaluate("view.center.every((v,i)=>Math.abs(v-currentPose.link_transforms.left_tool0[i][3])<1e-9)"));
  check('actual_TCP_focus_button');
  await page.click('#fit');
  await page.screenshot({path:path.join(output,'actual-rm75-desktop.png')});
  const record=await api(base,'/v1/workbench-model/b');
  report.robot_id=record.profile.robot_id;
  await page.click('#apply-program');
  await wait("!operation && document.getElementById('apply-note').textContent.includes('已写入程序')");
  const active=await api(base,'/v1/workbench-active-profile'),saved=active.active_profile_id;
  assert(Object.values(active.profile.dynamics.joints).every(j=>j.zero_offset_rad===null));
  check('UI_apply_preserves_unknown_physical_parameters');
  // Drive three actual joint target controls, then plan through the normal UI.
  await page.click('#planning-start-pose');
  const startPose=await page.evaluate('({...currentPose.positions})');
  for(const [joint,delta]of [['waist_pitch',.04],['left_joint_7',.12],['right_joint_7',-.12]]){
    await page.selectOption('#part',joint);
    await page.locator('#editor input[type=range]').evaluate((el,value)=>{el.value=String(value);el.dispatchEvent(new Event('input',{bubbles:true}));},startPose[joint]+delta);
    await wait('!operation && readyRevision===revision');
  }
  await page.click('#planning-joint');await wait('!operation && animation===null && !planningJob');
  assert((await page.locator('#planning-status').innerText()).includes('规划通过'));
  const end=await page.evaluate('({...currentPose.positions})');
  for(const [joint,delta]of [['waist_pitch',.04],['left_joint_7',.12],['right_joint_7',-.12]])assert(Math.abs(end[joint]-startPose[joint]-delta)<1e-9);
  check('UI_both_arms_and_waist_joint_path_playback');
  for(const [id,value]of [['planning-x','.005'],['planning-y','0'],['planning-z','0']])await page.fill('#'+id,value);
  assert((await page.locator('#cooperative-demo').innerText()).includes('规划并播放'));
  await page.click('#cooperative-demo');await wait('!operation && animation===null && !planningJob');
  assert((await page.locator('#planning-status').innerText()).includes('规划通过'));
  check('UI_coupled_two_TCP_object_translation');
  await page.click('#planning-object');await wait('animation!==null');
  await page.click('#emergency-stop');await wait('emergencyStop && !safetyPending && animation===null');
  assert(await page.isDisabled('#jog-plus'));
  await page.click('#reset-stop');await wait('!emergencyStop && !safetyPending');
  assert(await page.evaluate('animation===null'));
  check('UI_stop_during_planned_playback_and_no_auto_resume');
  await page.setViewportSize({width:390,height:844});await page.waitForTimeout(300);
  assert(await page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'));
  await page.screenshot({path:path.join(output,'actual-rm75-mobile.png')});
  check('actual_CAD_mobile_layout_390px');
  await stop();base=await start();await page.goto(base,{waitUntil:'domcontentloaded'});
  await wait('!operation && readyRevision===revision && profile.profile_mode===\'kinematic_preview\'');
  assert.equal(await page.inputValue('#profile-id'),saved);
  await wait('sceneEngine && sceneEngine.meshStats.loaded===20 && sceneEngine.meshStats.failed===0',180);
  check('saved_customer_profile_and_all_meshes_restore_after_restart');
  assert.deepEqual(errors,[]);assert.deepEqual(meshFailures,[]);
  report.status='passed';report.check_count=checks.length;report.page_errors=errors;
  report.renderer='Google Chrome / SwiftShader; no GPU throughput claim';
  console.log(JSON.stringify(report,null,2));
}catch(e){report.status='failed';report.error=String(e);report.page_errors=errors;
  if(page)await page.screenshot({path:path.join(output,'failure.png')}).catch(()=>{});
  console.error(e);process.exitCode=1;
}finally{if(browser)await browser.close();await stop();fs.writeFileSync(path.join(output,'ACTUAL_CAD_BROWSER.json'),JSON.stringify(report,null,2)+'\n');}
})();
