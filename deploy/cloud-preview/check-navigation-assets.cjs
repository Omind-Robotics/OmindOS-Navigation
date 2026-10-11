'use strict';
const fs=require('fs'),assert=require('assert');
const {chromium}=require(process.env.OMINDOS_PLAYWRIGHT||'playwright');
const base=process.env.CHECK_PREVIEW_URL,out=process.env.CHECK_OUTPUT;
const creds=JSON.parse(fs.readFileSync(process.env.CHECK_AUTH_FILE));
fs.mkdirSync(out,{recursive:true});
const report={checks:[],page_errors:[]};let browser,page,release;
async function initialized(){await page.waitForFunction(()=>!!profile&&!operation&&readyRevision===revision,null,{timeout:60000});}
async function navigate(){await page.fill('#navigation-x','.55');await page.fill('#navigation-y','0');await page.fill('#navigation-yaw','0');await page.click('#navigation-run');}
(async()=>{try{
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader',...(new URL(base).port==='19443'?['--host-resolver-rules=MAP cloud.omindos.cn 127.0.0.1']:[])]});
 for(const kind of ['cancel','timeout','failed']){
  const context=await browser.newContext({viewport:{width:1440,height:1000},extraHTTPHeaders:{Authorization:'Basic '+Buffer.from(creds.username+':'+creds.password).toString('base64')}});
  page=await context.newPage();page.setDefaultTimeout(15000);
  page.on('pageerror',e=>report.page_errors.push(String(e)));
  let intercepted=false,starts=0;
  const held=new Promise(resolve=>release=resolve);
  // A failed packed mesh normally recovers through the existing STL fallback.
  // Exercise genuine unavailable geometry by failing that fallback as well.
  if(kind==='failed')await page.route('**/preview-meshes/*',route=>route.fulfill({status:404,body:'unavailable test asset'}));
  await page.route('**/preview-meshes-v2/*',async route=>{
   if(!intercepted){intercepted=true;if(kind==='failed')return route.abort('connectionreset');await held;}
   await route.continue().catch(()=>{});
  });
  page.on('request',r=>{if(r.url().endsWith('/v1/navigation-start'))starts++;});
  await page.goto(base,{waitUntil:'domcontentloaded'});await initialized();
  await page.click('.cloud-layout-tabs [data-panel=navigation]');
  if(kind==='failed')await page.waitForFunction(()=>sceneEngine.meshStats.failed>0);
  else await page.waitForFunction(()=>sceneEngine.meshStats.requested>sceneEngine.meshStats.loaded+sceneEngine.meshStats.failed);
  await navigate();
  if(kind==='cancel'){
   await page.waitForFunction(()=>document.getElementById('navigation-status').textContent.includes('正在加载'));
   await page.click('#cloud-motion-stop');
   await page.waitForFunction(()=>!operation&&!navigationJob);
  }else{
   const text=kind==='timeout'?'仍在加载':'资源加载失败';
   await page.waitForFunction(text=>!operation&&!navigationJob&&document.getElementById('navigation-status').textContent.includes(text),text,{timeout:20000});
  }
  assert.equal(starts,0,'No navigation API before complete approved visuals');
  report.checks.push(kind+' during model preparation creates no navigation task');
  console.log('PASS '+report.checks.at(-1));
  release();
  await page.unroute('**/preview-meshes-v2/*');
  if(kind==='failed'){await page.reload({waitUntil:'domcontentloaded'});await initialized();await page.click('.cloud-layout-tabs [data-panel=navigation]');}
  await page.waitForFunction(()=>sceneEngine.meshStats.loaded===20&&sceneEngine.meshStats.failed===0,null,{timeout:60000});
  await navigate();
  await page.waitForFunction(()=>!operation&&!navigationJob&&document.getElementById('navigation-status').textContent.includes('已到位并确认停车'),null,{timeout:30000});
  report.checks.push(kind+' recovers only after model ready and explicit new start');
  await page.screenshot({path:out+'/'+kind+'-recovered.png'});
  console.log('PASS '+report.checks.at(-1));
  await context.close();page=null;
 }
 assert.deepStrictEqual(report.page_errors,[]);report.status='passed';
}catch(e){report.status='failed';report.error=String(e).replace(/Authorization[^\r\n]*/gi,'Authorization: [REDACTED]');process.exitCode=1;
 if(page){report.ui=await page.locator('#navigation-status').innerText().catch(()=>null);await page.screenshot({path:out+'/failure.png'}).catch(()=>{});}
}finally{release?.();fs.writeFileSync(out+'/report.json',JSON.stringify(report,null,2));if(browser)await browser.close();console.log(JSON.stringify(report,null,2));}})();
