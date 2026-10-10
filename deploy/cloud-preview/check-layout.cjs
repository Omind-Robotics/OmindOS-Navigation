'use strict';
const fs=require('fs'),assert=require('assert');
const {chromium}=require(process.env.OMINDOS_PLAYWRIGHT || 'playwright');
const root=process.env.OMINDOS_PREVIEW_ROOT || '/opt/omindos-cloud-preview',candidate=root+'/layout-candidate';
const credentials=JSON.parse(fs.readFileSync(root+'/secrets/access.json'));
const report={checks:[],errors:[],mode:process.env.LIVE_LAYOUT==='1'?'live':'candidate'};
let browser,page;
const pass=n=>{report.checks.push(n);console.log('PASS '+n);};
async function ready(){for(let i=0;i<300;i++){if(await page.evaluate('!!profile && !operation && readyRevision===revision'))return;await page.waitForTimeout(100);}throw Error('model did not initialize');}
async function canvasVisible(){return page.evaluate(()=>{const r=document.getElementById('scene').getBoundingClientRect();return r.width>100&&r.height>=70&&r.top>=0&&r.bottom<=innerHeight;});}
(async()=>{try{
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader','--host-resolver-rules=MAP omindos.cn 127.0.0.1']});
 const context=await browser.newContext({viewport:{width:390,height:844}});
 if(report.mode==='candidate'){
  await context.route('**/_cloud-preview/preview-bootstrap.js',r=>r.fulfill({path:candidate+'/preview-bootstrap.js',contentType:'text/javascript'}));
  await context.route('**/_cloud-preview/preview-layout.css',r=>r.fulfill({path:candidate+'/preview-layout.css',contentType:'text/css'}));
 }
 page=await context.newPage();page.setDefaultTimeout(20000);
 page.on('pageerror',e=>report.errors.push(String(e)));
 await page.goto('https://omindos.cn/_cloud-preview/',{waitUntil:'domcontentloaded'});
 await page.fill('input[name=username]',credentials.username);
 await page.fill('input[name=password]',credentials.password);
 await page.click('button[type=submit]');
 await page.waitForSelector('.cloud-layout-tabs');
 await ready();
 assert(await canvasVisible());
 await page.locator('#navigation-speed').scrollIntoViewIfNeeded();
 assert(await canvasVisible());
 const old=await page.inputValue('#navigation-speed');
 await page.fill('#navigation-speed','0.2');
 assert.equal(await page.inputValue('#navigation-speed'),'0.2');
 assert(await canvasVisible());
 await page.fill('#navigation-speed',old);
 pass('390px_navigation_inputs_and_model_visible_together');
 await page.screenshot({path:candidate+'/mobile-navigation.png'});
 await page.click('.cloud-layout-tabs [data-panel=parameters]');
 await page.locator('#part').scrollIntoViewIfNeeded();
 assert(await canvasVisible());
 const options=await page.locator('#part option').evaluateAll(xs=>xs.map(x=>x.value));
 assert(options.length>1);
 await page.selectOption('#part',options[1]);
 assert(await canvasVisible());
 assert(await page.locator('#apply-program').isVisible());
 pass('robot_parameter_controls_scroll_without_losing_model');
 await page.screenshot({path:candidate+'/mobile-parameters.png'});
 await page.click('.cloud-layout-tabs [data-panel=motion]');
 await page.locator('#cooperative-demo').scrollIntoViewIfNeeded();
 assert(await canvasVisible());
 pass('dual_arm_controls_and_model_visible_together');
 for(const [w,h]of [[360,740],[768,1024],[844,390]]){
  await page.setViewportSize({width:w,height:h});
  await page.click('.cloud-layout-tabs [data-panel=navigation]');
  await page.locator('#navigation-speed').scrollIntoViewIfNeeded();
  assert(await canvasVisible(),w+'x'+h+' canvas');
  assert(await page.evaluate('document.documentElement.scrollWidth<=innerWidth+1'));
  assert(await page.locator('#stage-stop').isVisible());
  pass(w+'x'+h+'_model_controls_and_stop_visible');
 }
 await page.setViewportSize({width:1440,height:1000});
 await page.waitForFunction(()=>!document.body.classList.contains('cloud-compact'));
 assert(await page.locator('.actions .navigation-controls').isVisible());
 assert(await page.locator('.viewport .view-controls #fit').isVisible());
 assert(await page.locator('.parameters').isVisible());
 assert.equal(await page.locator('#navigation-speed').count(),1);
 pass('desktop_layout_restored_with_original_control_nodes');
 await page.screenshot({path:candidate+'/desktop.png'});
 await page.setViewportSize({width:390,height:844});
 await page.waitForFunction(()=>document.body.classList.contains('cloud-compact'));
 assert(await canvasVisible());
 assert.equal(await page.locator('#navigation-speed').count(),1);
 assert.deepStrictEqual(report.errors,[]);
 pass('return_to_mobile_keeps_one_copy_and_no_script_errors');
 report.status='passed';
}catch(e){report.status='failed';report.error=String(e);process.exitCode=1;if(page)await page.screenshot({path:candidate+'/layout-failure.png'}).catch(()=>{});}
finally{if(browser)await browser.close();fs.writeFileSync(candidate+'/layout-'+report.mode+'.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));}
})();
