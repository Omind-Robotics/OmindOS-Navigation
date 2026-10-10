// Verifies the actual local development UI. No physical robot is contacted.
const {chromium}=require('playwright');
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {spawn}=require('node:child_process'),os=require('node:os');
(async()=>{
 const out=process.argv[2]||'docs/validation/quadruped-cad';fs.mkdirSync(out,{recursive:true});
 const store=fs.mkdtempSync(path.join(os.tmpdir(),'omindos-cad-check-'));
 const server=process.env.WORKBENCH_ROOT?spawn(process.env.PYTHON||'python',['scripts/robot_workbench.py','--store',store,'--port','0'],{cwd:process.env.WORKBENCH_ROOT}):null;
 const url=server?await new Promise((resolve,reject)=>{let output='';server.stdout.on('data',data=>{output+=data;const m=output.match(/http:\/\/127\.0\.0\.1:\d+\//);if(m)resolve(m[0]);});server.on('error',reject);server.once('exit',()=>reject(Error('workbench exited')));}):process.env.WORKBENCH_URL||'http://127.0.0.1:8097/';
 const browser=await chromium.launch({headless:true,executablePath:process.env.CHROMIUM_EXECUTABLE||undefined,
   args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-dev-shm-usage']});
 const page=await browser.newPage({viewport:{width:1600,height:1060},deviceScaleFactor:1});
 page.setDefaultTimeout(30000);
 const errors=[],failed=[],motionRequests=[];
 page.on('pageerror',e=>errors.push(String(e)));
 page.on('response',r=>{if(!r.ok()&&!r.url().includes('/v1/runtime/status')&&!r.url().includes('favicon'))failed.push([r.status(),r.url()]);});
 page.on('request',r=>{if(r.method()==='POST'&&/robot-jog|robot-sine|runtime\/run|navigation-start|planning-start/.test(r.url()))motionRequests.push(r.url());});
 try{
  await page.goto(url,{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.getElementById('cad-scene').dataset.loaded==='true',null,{timeout:120000});
  assert.equal(await page.locator('main').isVisible(),false);
  const metadata=await page.evaluate(()=>({parts:document.getElementById('cad-scene').dataset.parts,bounds:JSON.parse(document.getElementById('cad-scene').dataset.bounds),size:document.getElementById('cad-size').textContent}));
  console.log(JSON.stringify(metadata));assert(Number(metadata.parts)>900);
  const manifest=await page.evaluate(async()=>await(await fetch('/cad/v11/manifest.json')).json());
  console.log(JSON.stringify({expected_bounds:manifest.bounds_m}));const model=await page.evaluate(async()=>await(await fetch('/cad/v11/kinematics.json')).json());assert.equal(Number(metadata.parts),manifest.visible_instances+model.assembly.additions.length);
  for(let r=0;r<2;r++)for(let c=0;c<3;c++)assert(Math.abs(metadata.bounds[r][c]-manifest.bounds_m[r][c])<1e-5,'CAD bounds must survive GLB load');
  for(const v of ['front','side','left','top','iso'])await page.locator(`[data-cad-view=${v}]`).click();
  await page.locator('#cad-palette').selectOption('blue');
  await page.locator('#cad-palette').selectOption('graphite');
  const box=await page.locator('#cad-scene').boundingBox();
  await page.mouse.move(box.x+box.width*.5,box.y+box.height*.5);await page.mouse.down();await page.mouse.move(box.x+box.width*.57,box.y+box.height*.55,{steps:8});await page.mouse.up();
  await page.mouse.wheel(0,-120);await page.locator('#cad-fit').click();
  let picked=false;
  for(const [x,y] of [[.5,.5],[.45,.5],[.55,.45],[.5,.4],[.6,.6]]){
   await page.mouse.click(box.x+box.width*x,box.y+box.height*y);
   if(!(await page.locator('#cad-part-name').textContent()).includes('点击')){picked=true;break;}
  }
  assert(picked,'CAD mesh picking');
  assert.equal(motionRequests.length,0,'CAD view must not issue motion requests');
  await page.waitForFunction(()=>document.getElementById('cad-scene').dataset.joints==='12');
  for(const leg of ['FL','FR','RR']){
   await page.locator(`[data-cad-leg=${leg}]`).click();
   assert.equal(await page.locator('#cad-q0').isEnabled(),true);
   assert.equal(await page.locator('#cad-q1').isEnabled(),true);
   assert.equal(await page.locator('#cad-q2').isEnabled(),true);
  }
  await page.locator('[data-cad-leg=RL]').click();
  for(const i of [0,1,2])assert(await page.locator('#cad-q'+i).isEnabled());
  await page.locator('summary').filter({hasText:'轴线、零位与映射依据'}).click();
  const downloadEvent=page.waitForEvent('download');await page.locator('#cad-export-calibration').click();
  const download=await downloadEvent;const calibration=JSON.parse(fs.readFileSync(await download.path()));
  assert.equal(calibration.calibration.joints.length,12);assert(calibration.calibration.joints.every(j=>j.encoder_zero_rad===null&&j.motor_direction===null));
  assert.equal(calibration.assembly_revision,'v1.1-assembly-r1');
  await page.locator('summary').filter({hasText:'轴线、零位与映射依据'}).click();
  await page.locator('#cad-show-axes').uncheck();
  await page.locator('[data-cad-view=left]').click();
  const imageBefore=await page.locator('#cad-scene').screenshot();
  for(const [i,value]of [[0,'5'],[1,'15'],[2,'-12']]){
   await page.locator('#cad-q'+i+'-value').fill(value);await page.locator('#cad-q'+i+'-value').dispatchEvent('change');
  }
  assert.deepEqual(await page.evaluate(()=>JSON.parse(document.getElementById('cad-scene').dataset.jointAngles).RL),[5,15,-12]);
  const imageAfter=await page.locator('#cad-scene').screenshot();
  fs.writeFileSync(path.join(out,'CAD-Zero.png'),imageBefore);fs.writeFileSync(path.join(out,'CAD-Motion.png'),imageAfter);
  const sharp=require('sharp');const rawA=await sharp(imageBefore).raw().toBuffer(),rawB=await sharp(imageAfter).raw().toBuffer();
  assert.equal(rawA.length,rawB.length);let changed=0;for(let i=0;i<rawA.length;i++)if(Math.abs(rawA[i]-rawB[i])>15)changed++;
  console.log('changed fraction',changed/rawA.length);assert(changed/rawA.length>.003,'moving actual CAD must change rendered pixels');
  await page.locator('#cad-q0-value').fill('99');await page.locator('#cad-q0-value').dispatchEvent('change');
  assert.equal(await page.locator('#cad-q0-value').inputValue(),'5.0');
  await page.locator('#cad-demo').click();await page.locator('#cad-motion-stop').click();
  const held=await page.locator('#cad-scene').getAttribute('data-joint-angles');
  await page.waitForTimeout(120);assert.equal(await page.locator('#cad-scene').getAttribute('data-joint-angles'),held);
  await page.locator('#cad-reset').click();
  for(const q of Object.values(await page.evaluate(()=>JSON.parse(document.getElementById('cad-scene').dataset.jointAngles))))assert.deepEqual(q,[0,0,0]);
  await page.locator('#cad-demo').click();await page.locator('#parameters-tab').click();
  assert(await page.locator('#cad-motion-stop').isDisabled());

  await page.waitForFunction(()=>document.getElementById('model-name').textContent.includes('MEVIUS2'));
  await page.waitForFunction(()=>document.getElementById('scene').dataset.meshLoaded==='13');
  await page.locator('#robot-id').fill('cad-roundtrip-check');await page.locator('#robot-id').dispatchEvent('change');
  await page.locator('#cad-tab').click();await page.locator('#parameters-tab').click();
  assert.equal(await page.locator('#robot-id').inputValue(),'cad-roundtrip-check');
  await page.locator('#apply-program').click();
  await page.waitForFunction(()=>document.getElementById('apply-note').textContent.startsWith('已写入'));
  const active=await page.evaluate(async()=>await(await fetch('/v1/workbench-active-profile')).json());
  assert.equal(active.profile.robot_id,'cad-roundtrip-check');
  await page.locator('#cad-tab').click();
  const after=await page.evaluate(async()=>await(await fetch('/v1/workbench-active-profile')).json());
  assert.equal(after.active_profile_id,active.active_profile_id);
  await page.locator('#cad-fit').click();
  await page.locator('#cad-reset').click();
  await page.locator('[data-cad-leg=RL]').click();
  await page.locator('[data-cad-view=left]').click();
  const clearBox=await page.locator('#cad-scene').boundingBox();await page.mouse.click(clearBox.x+15,clearBox.y+15);
  await page.locator('#cad-q1-value').fill('12');await page.locator('#cad-q1-value').dispatchEvent('change');
  await page.locator('#cad-q2-value').fill('-10');await page.locator('#cad-q2-value').dispatchEvent('change');
  await page.locator('#cad-show-axes').check();
  await page.screenshot({path:path.join(out,'OriginMind-Quadruped-V11-Joints.png'),fullPage:true});
  await page.setViewportSize({width:390,height:844});
  await page.locator('#cad-fit').click();
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'mobile overflow');
  await page.screenshot({path:path.join(out,'OriginMind-Quadruped-V11-Mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);assert.deepEqual(motionRequests,[]);
  const report={stage:process.env.INSTALLED_PACKAGE?'installed-full-package':'source-development-preview',hardware_validated:false,ubuntu_installer_updated:!!process.env.INSTALLED_PACKAGE,
   browser:browser.version(),metadata,errors,failed,motionRequests,rendered_motion_changed_fraction:changed/rawA.length,
   checks:['cad_mesh_render','cad_bounds_preserved','five_camera_views','palette_change','orbit_zoom_fit','part_picking',
    'cad_no_motion_requests','parameter_reference_meshes','draft_survives_tab_roundtrip',
    'parameter_apply','cad_does_not_replace_active_profile','390px_no_horizontal_overflow',
    'twelve_axes_enabled','corrected_assembly_loaded','calibration_export_preserves_unknown_encoder_zeros','three_left_rear_controls_enabled',
    'rendered_cad_moves','out_of_range_pose_rejected','preview_stop_holds_pose','cad_zero_reset','leave_cad_stops_animation']};
  fs.writeFileSync(path.join(out,'BROWSER_VALIDATION.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify(report));
 }catch(error){
  await page.screenshot({path:path.join(out,'failure.png')}).catch(()=>{});
  fs.writeFileSync(path.join(out,'FAILURE.json'),JSON.stringify({error:String(error),errors,failed},null,2));
  throw error;
 }finally{await browser.close();server?.kill();}
})().catch(e=>{console.error(e);process.exit(1)});
