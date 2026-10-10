#!/usr/bin/env python3
"""Exercise real workbench UI with real joint geometry and a tiny transfer mesh.

This is NOT acceptance of the large customer's CAD rendering or an installer.
"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'scripts')]
from test_workbench_rm75_integration import BundleRegistryTests
from playwright.sync_api import sync_playwright


def run(output):
    output.mkdir(parents=True,exist_ok=True)
    helper=BundleRegistryTests();helper.setUp();checks=[];errors=[];process=None
    report={'scope':'source_browser_UI_with_actual_joint_geometry_and_synthetic_transfer_mesh',
            'actual_customer_visuals_tested':False,'compiled_installer_tested':False,
            'hardware_validated':False,'dynamics_validated':False,'checks':checks}
    log=(output/'server.log').open('w')
    def start(store):
        with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
        p=subprocess.Popen([sys.executable,str(ROOT/'scripts/robot_workbench.py'),'--store',str(store),
                            '--port',str(port),'--model-bundle',str(helper.root)],
                           cwd=ROOT,stdout=log,stderr=log,env={**os.environ,'PYTHONPATH':str(ROOT/'scripts')})
        url=f'http://127.0.0.1:{port}'
        for _ in range(150):
            if p.poll() is not None:raise RuntimeError('workbench process exited; inspect server.log')
            try:
                with urllib.request.urlopen(url+'/v1/workbench-state',timeout=1):return p,url
            except OSError:time.sleep(.1)
        p.terminate();p.wait(timeout=5);raise RuntimeError('workbench startup timeout')
    try:
        with tempfile.TemporaryDirectory() as td, sync_playwright() as driver:
            process,url=start(Path(td))
            browser=driver.chromium.launch(headless=True,args=['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader'])
            page=browser.new_page(viewport={'width':1440,'height':1000})
            page.on('pageerror',lambda e:errors.append(str(e)))
            def wait(expression, timeout=30):
                # Poll from the test driver; never enable unsafe-eval in the app.
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    if page.evaluate('() => ('+expression+')'):
                        return
                    page.wait_for_timeout(50)
                detail=page.locator('#status').inner_text()
                raise AssertionError('UI wait timed out: '+expression+'; status='+detail)
            page.goto(url,wait_until='networkidle')
            wait('!operation && readyRevision===revision && !!profile')
            assert page.locator('#cad-model option').count()==3
            checks.append('three_explicit_CAD_variants_visible')
            page.select_option('#cad-model','b');page.click('#load-cad-model')
            wait("!operation && profile.robot_id==='rm75-integration-b' && readyRevision===revision")
            assert page.input_value('#profile-mode')=='kinematic_preview'
            mapping=json.loads(page.input_value('#planning-mapping'))
            assert mapping['tcp_links']==['left_tool0','right_tool0']
            checks.append('model_load_and_actual_TCP_mapping')
            wait('sceneEngine && sceneEngine.meshStats.requested>0 && sceneEngine.meshStats.loaded===sceneEngine.meshStats.requested && sceneEngine.meshStats.failed===0')
            assert page.evaluate("""() => {
                const c=document.getElementById('scene');
                const gl=c.getContext('webgl2') || c.getContext('webgl');
                return !!gl && !gl.isContextLost() && c.width>0 && c.height>0;
            }""")
            checks.append('verified_local_transfer_mesh_loaded_by_real_renderer')
            page.click('#apply-program')
            wait("!operation && document.getElementById('apply-note').textContent.includes('已写入程序')")
            saved_id=page.input_value('#profile-id');assert len(saved_id)==64
            with urllib.request.urlopen(url+'/v1/workbench-active-profile') as r:active=json.load(r)
            assert active['active_profile_id']==saved_id
            assert all(v['zero_offset_rad']is None for v in active['profile']['dynamics']['joints'].values())
            checks.append('UI_apply_preserves_unknown_motor_parameters')
            page.select_option('#jog-joint','waist_pitch')
            before=page.evaluate('() => currentPose.positions.waist_pitch')
            page.click('#jog-plus')
            wait('!operation && animation===null')
            after=page.evaluate('() => currentPose.positions.waist_pitch')
            assert abs(after-before-math.radians(.1))<1e-10
            checks.append('UI_tenth_degree_joint_jog')
            page.click('#emergency-stop');wait('emergencyStop && !safetyPending')
            assert page.is_disabled('#jog-plus')
            page.click('#reset-stop');wait('!emergencyStop && !safetyPending')
            assert page.evaluate('() => animation===null')
            checks.append('UI_stop_latch_and_reset_without_auto_resume')
            page.fill('#robot-id','rm75-ui-history');page.click('#save')
            wait("!operation && document.getElementById('saved').textContent.includes('已保存配置')")
            history_id=page.input_value('#profile-id');assert history_id!=saved_id
            page.fill('#profile-id',saved_id);page.click('#load')
            wait("!operation && profile.robot_id==='rm75-integration-b'")
            checks.append('UI_version_save_and_history_reload')
            process.terminate();process.wait(timeout=5);process=None
            process,url=start(Path(td));page.goto(url,wait_until='networkidle')
            wait("!operation && profile.robot_id==='rm75-integration-b' && readyRevision===revision")
            assert page.input_value('#profile-id')==saved_id
            assert page.input_value('#profile-mode')=='kinematic_preview'
            checks.append('UI_saved_configuration_restored_after_service_restart')
            page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(250)
            assert page.evaluate('() => document.documentElement.scrollWidth<=window.innerWidth+1')
            checks.append('mobile_390px_no_horizontal_page_overflow')
            assert not errors,errors
            checks.append('no_JavaScript_page_errors')
            report.update(status='passed',check_count=len(checks),page_errors=errors,
                          renderer='Chromium SwiftShader; no GPU performance acceptance')
            browser.close()
    except Exception as e:
        report.update(status='failed',error=str(e),check_count=len(checks),page_errors=errors)
        raise
    finally:
        if process is not None and process.poll()is None:process.terminate();process.wait(timeout=5)
        log.close();helper.tearDown()
        (output/'BROWSER_VALIDATION.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args().output)
