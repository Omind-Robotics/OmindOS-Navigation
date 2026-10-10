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
  console.log(JSON.stringify({expected_bounds:manifest.bounds_m}));assert.equal(Number(metadata.parts),manifest.visible_instances);
  for(let r=0;r<2;r++)for(let c=0;c<3;c++)assert(Math.abs(metadata.bounds[r][c]-manifest.bounds_m[r][c])<1e-5,'CAD bounds must survive GLB load');
  for(const v of ['front','side','top','iso'])await page.locator(`[data-cad-view=${v}]`).click();
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
  await page.locator('#parameters-tab').click();
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
  await page.screenshot({path:path.join(out,'OriginMind-Quadruped-V11-Workbench.png')});
  await page.setViewportSize({width:390,height:844});
  await page.locator('#cad-fit').click();
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'mobile overflow');
  await page.screenshot({path:path.join(out,'OriginMind-Quadruped-V11-Mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  const report={stage:'source-development-preview',hardware_validated:false,ubuntu_installer_updated:false,
   browser:browser.version(),metadata,errors,failed,motionRequests,
   checks:['cad_mesh_render','cad_bounds_preserved','four_camera_views','palette_change','orbit_zoom_fit','part_picking',
    'cad_no_motion_requests','parameter_reference_meshes','draft_survives_tab_roundtrip',
    'parameter_apply','cad_does_not_replace_active_profile','390px_no_horizontal_overflow']};
  fs.writeFileSync(path.join(out,'BROWSER_VALIDATION.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify(report));
 }catch(error){
  await page.screenshot({path:path.join(out,'failure.png')}).catch(()=>{});
  fs.writeFileSync(path.join(out,'FAILURE.json'),JSON.stringify({error:String(error),errors,failed},null,2));
  throw error;
 }finally{await browser.close();server?.kill();}
})().catch(e=>{console.error(e);process.exit(1)});
