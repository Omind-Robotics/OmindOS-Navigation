// Run against the real ROS-backed workbench, with isolated test configuration storage.
const assert=require('node:assert/strict');
const puppeteer=require('puppeteer');
const fs=require('node:fs');
(async()=>{
  const browser=await puppeteer.launch({headless:true,executablePath:process.env.CHROMIUM_EXECUTABLE||'/usr/bin/google-chrome',args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu']});
  const checks=[];
  try{
    const page=await browser.newPage(),errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    await page.setViewport({width:1440,height:1000});
    await page.goto(process.env.WORKBENCH_URL||'http://127.0.0.1:18095/');
    await page.waitForFunction(()=>document.getElementById('status').textContent.includes('教学示例'));
    await page.waitForFunction(()=>document.getElementById('runtime-status').textContent.includes('已连接'));
    await page.click('#runtime-load');
    await page.waitForFunction(()=>document.getElementById('runtime-profile').textContent.includes('ROS 2 配置'));
    const first=await page.$eval('#profile-id',e=>e.value);assert.equal(first.length,64);
    checks.push('current_parameters_saved_and_loaded_into_ros2');
    await page.$eval('#duration',e=>e.value='2');await page.click('#runtime-run');
    await page.waitForFunction(()=>document.getElementById('runtime-status').textContent.includes('试运行中'));
    await page.waitForFunction(()=>Math.abs(currentPose.positions.left_elbow)>.01);
    checks.push('ros2_state_drives_visible_robot_pose');
    await page.click('#runtime-stop');
    await page.waitForFunction(()=>document.getElementById('runtime-status').textContent.includes('已停止'));
    const stopped=await page.evaluate(async()=>await(await fetch('/v1/runtime/status')).json());
    assert.equal(stopped.state,'STOPPED');assert.equal(stopped.hardware_active,false);
    checks.push('stop_reaches_runtime_and_no_hardware_activation');
    // A local preview must not be overwritten by background ROS status polling.
    await page.click('#run');await page.waitForFunction(()=>document.getElementById('status').textContent.includes('运动学试运行完成'));
    assert.ok(Math.abs(await page.evaluate(()=>currentPose.positions.left_elbow)+.65)<.001);
    checks.push('local_preview_remains_independent_of_ros_status_polling');
    await page.$eval('#robot-id',e=>{e.value='browser-revised-profile';e.dispatchEvent(new Event('change',{bubbles:true}));});
    await page.waitForFunction(()=>document.getElementById('status').textContent.startsWith('模型已更新'));
    await page.click('#runtime-run');
    await page.waitForFunction(()=>document.getElementById('runtime-status').textContent.includes('已加载版本不同'));
    checks.push('edited_parameters_require_explicit_runtime_reload');
    await page.click('#runtime-load');
    await page.waitForFunction(id=>document.getElementById('profile-id').value!==id,{},first);
    const second=await page.$eval('#profile-id',e=>e.value);assert.equal(second.length,64);
    const old=await page.evaluate(async id=>await(await fetch('/v1/robot-profiles/'+id)).json(),first);
    assert.notEqual(old.robot_id,'browser-revised-profile');
    checks.push('reload_uses_new_version_and_old_version_remains_readable');
    await page.setViewport({width:390,height:844});
    assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.click('#runtime-run');
    await page.waitForFunction(()=>document.getElementById('runtime-status').textContent.includes('试运行中'));
    await page.click('#runtime-stop');
    await page.waitForFunction(()=>document.getElementById('runtime-status').textContent.includes('已停止'));
    checks.push('mobile_runtime_controls_and_no_horizontal_overflow');
    if(process.env.WORKBENCH_SCREENSHOT_DIR){
      fs.mkdirSync(process.env.WORKBENCH_SCREENSHOT_DIR,{recursive:true});
      await page.screenshot({path:process.env.WORKBENCH_SCREENSHOT_DIR+'/ros-workbench-mobile.png',fullPage:true});
      await page.setViewport({width:1440,height:1000});
      await page.screenshot({path:process.env.WORKBENCH_SCREENSHOT_DIR+'/ros-workbench-desktop.png',fullPage:true});
    }
    assert.deepEqual(errors,[]);checks.push('no_browser_script_errors');
    console.log(JSON.stringify({status:'PASS',browser:await browser.version(),checks},null,2));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
