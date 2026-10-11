'use strict';
const fs = require('fs'), assert = require('assert');
const {chromium} = require(process.env.OMINDOS_PLAYWRIGHT || 'playwright');
const root=process.env.OMINDOS_PREVIEW_ROOT || '/opt/omindos-cloud-preview';
const credentials=JSON.parse(fs.readFileSync(root+'/secrets/access.json','utf8'));
const report={checks:[],page_errors:[],failed_requests:[]};
let browser,page;
function pass(name){report.checks.push(name);console.log('PASS '+name);}
async function ready(expression,timeout=60000){
 const until=Date.now()+timeout;
 while(Date.now()<until){if(await page.evaluate('('+expression+')'))return;await page.waitForTimeout(100);}
 throw Error('Timed out: '+expression);
}
(async()=>{try{
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader','--host-resolver-rules=MAP omindos.cn 127.0.0.1']});
 const context=await browser.newContext({viewport:{width:1440,height:1000},httpCredentials:credentials});
 page=await context.newPage();
 page.setDefaultTimeout(20000);
 page.on('pageerror',e=>report.page_errors.push(String(e)));
 page.on('requestfailed',r=>report.failed_requests.push(r.url()+':'+r.failure().errorText));
 page.on('response',r=>{if(r.url().includes('/_cloud-preview/')&&r.status()>=400)report.failed_requests.push(r.url()+':HTTP_'+r.status());});
 await page.goto('https://omindos.cn/_cloud-preview/',{waitUntil:'domcontentloaded'});
 await ready('!operation && readyRevision===revision && !!profile');
 assert((await page.title()).includes('云端体验'));
 assert(await page.locator('.cloud-preview-note').isVisible());
 pass('HTTPS_authenticated_workbench_loads');

 await page.fill('#navigation-x','0.7');
 await page.fill('#navigation-y','0.2');
 await page.click('#navigation-run');
 await ready("!operation && !navigationJob && document.getElementById('navigation-status').textContent.includes('已到位并确认停车')");
 report.navigation_status=await page.locator('#navigation-status').innerText();
 pass('real_cloud_navigation_via_UI');
 await page.screenshot({path:root+'/validation/navigation-desktop.png'});
 await page.selectOption('#cad-model','b');
 await page.click('#load-cad-model');
 await ready("!operation && profile.profile_mode==='kinematic_preview' && readyRevision===revision");
 await ready('sceneEngine && sceneEngine.meshStats.requested===20 && sceneEngine.meshStats.loaded===20 && sceneEngine.meshStats.failed===0',120000);
 assert(await page.evaluate(()=>{const c=document.getElementById('scene'),g=c.getContext('webgl2')||c.getContext('webgl');return !!g&&!g.isContextLost();}));
 pass('20_actual_CAD_visuals_rendered');
 for(const [id,value]of [['planning-x','.005'],['planning-y','0'],['planning-z','0']])await page.fill('#'+id,value);
 await page.click('#cooperative-demo');
 await ready("!operation && animation===null && !planningJob && document.getElementById('planning-status').textContent.includes('规划通过')",90000);
 report.planning_status=await page.locator('#planning-status').innerText();
 pass('real_cloud_dual_TCP_planning_and_playback_via_UI');
 await page.click('#fit');
 await page.screenshot({path:root+'/validation/dual-arm-desktop.png'});
 await page.setViewportSize({width:390,height:844});
 assert(await page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'));
 pass('390px_mobile_no_horizontal_overflow');
 await page.screenshot({path:root+'/validation/mobile.png'});
 assert.deepStrictEqual(report.page_errors,[]);
 report.aborted_requests=report.failed_requests.filter(x=>x.endsWith(':net::ERR_ABORTED'));
 // Browser may cancel a superseded mesh request during model changes; all 20 active visuals must still load.
 assert.deepStrictEqual(report.failed_requests.filter(x=>!x.endsWith(':net::ERR_ABORTED')),[]);
 report.status='passed';
}catch(error){report.status='failed';report.error=String(error);process.exitCode=1;if(page){report.ui_status=await page.locator('#status').innerText().catch(()=>null);report.navigation_status=await page.locator('#navigation-status').innerText().catch(()=>null);await page.screenshot({path:root+'/validation/failure.png'}).catch(()=>{});}}
finally{if(browser)await browser.close();fs.writeFileSync(root+'/validation/browser.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));}
})();
