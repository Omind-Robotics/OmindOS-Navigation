'use strict';
// Use existing authorized credentials; preserve an inherited public browser session.
const fs=require('fs'),assert=require('assert');
const {chromium}=require(process.env.OMINDOS_PLAYWRIGHT||'playwright');
const base=process.env.CHECK_PREVIEW_URL,out=process.env.CHECK_OUTPUT;
const creds=JSON.parse(fs.readFileSync(process.env.CHECK_AUTH_FILE));
const report={base,checks:[]};let browser;
fs.mkdirSync(out,{recursive:true});
(async()=>{try{
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,
  args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1440,height:1000}});
 await context.addCookies([{name:'__Secure-omindos-preview',value:'inherited-public-session',
  domain:new URL(base).hostname,path:'/',secure:true,httpOnly:true,sameSite:'Strict'}]);
 const page=await context.newPage();page.setDefaultTimeout(15000);
 await page.goto(base+'login',{waitUntil:'domcontentloaded'});
 // Seed only browser-owned settings/cache, never a pose, model or planner result.
 await page.evaluate(async()=>{
  localStorage.setItem('omindos-navigation-config','inherited-public-setting');
  await caches.open('omindos-preview-pwa-inherited-session');
 });
 await page.fill('#username',creds.username);await page.fill('#password',creds.password);
 await Promise.all([page.waitForURL(url=>url.pathname===new URL(base).pathname),page.click('button[type=submit]')]);
 await page.waitForFunction(()=>!!profile&&!!currentPose&&!operation&&readyRevision===revision&&sceneEngine?.meshStats.loaded===20,null,{timeout:60000});
 const cookies=await context.cookies();
 const session=cookies.find(c=>c.name==='__Secure-omindos-navigation-preview');
 assert(session&&session.path===new URL(base).pathname&&session.secure&&session.httpOnly&&session.sameSite==='Strict');
 assert.equal(cookies.find(c=>c.name==='__Secure-omindos-preview').value,'inherited-public-session');
 report.checks.push('form login uses separate Secure HttpOnly Strict cookie; inherited public cookie unchanged');
 assert.equal((await context.request.get(base+'v1/workbench-state')).status(),200);
 report.checks.push('authenticated browser session accesses real backend');
 await page.click('.cloud-layout-tabs [data-panel=navigation]');
 const config=await page.evaluate(()=>({config:navigationConfig(),obstacles:JSON.parse(document.getElementById('planning-obstacles').value)}));
 await page.setInputFiles('#navigation-import',{name:'scope-test.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(config))});
 await page.waitForFunction(()=>document.getElementById('navigation-status').textContent.includes('导航配置已导入'));
 const storage=await page.evaluate(()=>({publicValue:localStorage.getItem('omindos-navigation-config'),
  candidateValue:localStorage.getItem('omindos-navigation-preview:omindos-navigation-config')}));
 assert.equal(storage.publicValue,'inherited-public-setting');assert.deepStrictEqual(JSON.parse(storage.candidateValue),config.config);
 report.checks.push('real configuration import writes only preview storage; public setting unchanged');
 await page.waitForFunction(async()=>{const r=await navigator.serviceWorker.getRegistration();return r?.active?.state==='activated';},null,{timeout:20000});
 const cache=await page.evaluate(async()=>({keys:await caches.keys(),scope:(await navigator.serviceWorker.getRegistration()).scope}));
 assert.equal(cache.scope,base);assert(cache.keys.includes('omindos-preview-pwa-inherited-session'));
 assert(cache.keys.some(key=>key.startsWith('omindos-navigation-preview-pwa-')));
 report.checks.push('preview service worker activates in its subpath without deleting public offline cache');
 await page.screenshot({path:out+'/authenticated-preview.png'});
 report.status='passed';
}catch(e){report.status='failed';report.error=String(e).split(creds.password).join('[REDACTED]').replace(/Authorization[^\r\n]*/gi,'Authorization: [REDACTED]');process.exitCode=1;
}finally{fs.writeFileSync(out+'/report.json',JSON.stringify(report,null,2));if(browser)await browser.close();console.log(JSON.stringify(report,null,2));}})();
