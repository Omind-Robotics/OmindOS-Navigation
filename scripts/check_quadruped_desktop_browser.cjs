// Run only against a local compiled quadruped candidate. No signed-in session.
const assert = require('node:assert/strict');
const {spawn} = require('node:child_process');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {chromium} = require('playwright');

(async () => {
  const executable = path.resolve(process.argv[2]);
  const output = path.resolve(process.argv[3]);
  fs.mkdirSync(output,{recursive:true});
  const store = fs.mkdtempSync(path.join(os.tmpdir(),'quadruped-browser-'));
  const server = spawn(executable,['--no-browser','--port','0','--store',store]);
  let browser;
  const checks=[];
  try {
    const base = await new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>reject(Error('server timeout')),30000);
      server.stdout.on('data',d=>{const m=String(d).match(/http:\/\/127\.0\.0\.1:\d+/);if(m){clearTimeout(timer);resolve(m[0]);}});
      server.on('exit',code=>{clearTimeout(timer);reject(Error('server exited '+code));});
    });
    browser=await chromium.launch({headless:true,executablePath:process.env.CHROMIUM_EXECUTABLE||undefined,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
    const page=await browser.newPage({viewport:{width:1440,height:1000}});
    const errors=[];page.on('pageerror',e=>errors.push(String(e)));
    await page.goto(base);
    await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('四足参考模型已载入'));
    assert.equal(await page.locator('#jog-joint option').count(),12);
    assert.equal(await page.locator('#connections-panel').isVisible(),false);
    assert.equal(await page.locator('.chassis-controls').isVisible(),false);
    checks.push('quadruped_default_loaded_12_joints_no_manipulation_only_controls');
    await page.locator('#apply-program').click();
    await page.waitForFunction(()=>document.querySelector('#apply-note').textContent.includes('已'));
    await page.locator('#jog-plus').click();
    await page.waitForFunction(()=>parseFloat(document.querySelector('#jog-angle').textContent)>0.09);
    checks.push('browser_point_one_degree_jog');
    await page.locator('#emergency-stop').click();
    await page.waitForFunction(()=>document.body.dataset.estop==='true');
    assert.equal(await page.locator('#jog-plus').isDisabled(),true);
    checks.push('browser_stop_locks_motion');
    await page.screenshot({path:path.join(output,'quadruped-desktop.png'),fullPage:true});
    await page.setViewportSize({width:390,height:844});
    await page.screenshot({path:path.join(output,'quadruped-mobile.png'),fullPage:true});
    assert.deepEqual(errors,[]);
    checks.push('desktop_and_mobile_render_without_page_errors');
    fs.writeFileSync(path.join(output,'browser.json'),JSON.stringify({status:'PASS',checks,errors},null,2));
    console.log(JSON.stringify({status:'PASS',checks}));
  } finally {
    if(browser)await browser.close();
    server.kill();fs.rmSync(store,{recursive:true,force:true});
  }
})().catch(e=>{console.error(e);process.exit(1);});

