// Actual local workbench UI capture; use a clean store and the pinned mesh model.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');

(async () => {
  const output = process.argv[2] || 'docs/media/quadruped-reference';
  fs.mkdirSync(output, {recursive:true});
  const browser = await chromium.launch({headless:true, args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  try {
    const page = await browser.newPage({viewport:{width:1800,height:1120},deviceScaleFactor:1});
    page.setDefaultTimeout(20000);
    const errors=[], meshFailures=[];
    page.on('pageerror',error=>errors.push(String(error)));
    page.on('response',r=>{if(r.url().includes('/meshes/')&&!r.ok())meshFailures.push(r.url());});
    await page.goto('http://127.0.0.1:8085/',{waitUntil:'domcontentloaded'});
    await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('四足参考模型已载入'));
    await page.waitForFunction(()=>document.querySelector('#scene').dataset.meshLoaded==='13',null,{timeout:30000});
    assert.equal(await page.locator('#scene').getAttribute('data-renderer'),'webgl');
    assert.equal(await page.locator('#scene').getAttribute('data-mesh-failed'),'0');
    assert.equal(await page.locator('#jog-joint option').count(),12);
    for(const selector of ['.navigation-controls','.planning-controls','.cooperative-controls','.reference-photo','.chassis-controls'])
      assert.equal(await page.locator(selector).isVisible(),false);
    await page.locator('#part').selectOption('FL_hip_joint');
    await page.locator('#fit').click();
    await page.waitForTimeout(800);
    console.log('Complete reference model rendered; capturing viewport');
    await page.screenshot({path:path.join(output,'workbench.png'),timeout:60000});
    // Check configuration use and an actual point-one-degree preview after capturing the clean state.
    await page.locator('#apply-program').click();
    await page.waitForFunction(()=>document.querySelector('#apply-note').textContent.includes('已'));
    const before = Number(await page.locator('#jog-angle').textContent().then(x=>parseFloat(x)));
    await page.locator('#jog-plus').click();
    await page.waitForFunction(before=>parseFloat(document.querySelector('#jog-angle').textContent)>before+.09,before);
    await page.locator('#emergency-stop').click();
    await page.waitForFunction(()=>document.body.dataset.estop==='true');
    assert.equal(await page.locator('#jog-plus').isDisabled(),true);
    assert.deepEqual(errors,[]);assert.deepEqual(meshFailures,[]);
    const metadata={stage:'source-workbench-preview',hardware_validated:false,installer_updated:false,
      upstream_commit:'4f09680bb575574377b903bbc971bdb2695d507b',source_commit:process.env.GITHUB_SHA,
      viewport:{width:1800,height:1120},title:await page.title(),mesh_loaded:13,mesh_failed:0,
      active_joints:12,page_errors:errors,mesh_request_errors:meshFailures,
      checks:['complete_mesh_webgl_render','12_joints','quadruped_only_controls','apply_configuration','point_one_degree_jog','stop_latches_motion']};
    fs.writeFileSync(path.join(output,'CAPTURE.json'),JSON.stringify(metadata,null,2)+'\n');
    console.log(JSON.stringify(metadata));
  } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exit(1);});
