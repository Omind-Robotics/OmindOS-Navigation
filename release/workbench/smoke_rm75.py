"""Acceptance of bundled customer models through the compiled executable API."""
import argparse
import hashlib
import json
from pathlib import Path
import queue
import re
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request


def run(executable, output):
    executable=executable.resolve();package=executable.parent
    bundle=package/'config/robot_models/rm75-cad'
    manifest=json.loads((bundle/'manifest.json').read_text())
    checks=[];report={'scope':'compiled_RM75_bundle_API', 'checks':checks,
                      'hardware_validated':False,'dynamics_validated':False}
    with tempfile.TemporaryDirectory(prefix='rm75-package-')as td:
        store=Path(td)/'state'
        def start():
            p=subprocess.Popen([str(executable),'--no-browser','--port','0','--store',str(store)],
                               cwd=td,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            lines=queue.Queue()
            def read():
                for line in p.stdout:lines.put(line)
                lines.put(None)
            threading.Thread(target=read,daemon=True).start()
            try:
                while True:
                    line=lines.get(timeout=60)
                    if line is None:raise RuntimeError('compiled workbench exited')
                    match=re.search(r'http://127\.0\.0\.1:\d+/',line)
                    if match:return p,match.group().rstrip('/')
            except BaseException:
                p.terminate();p.wait(timeout=10);raise
        def call(route,data=None,raw=False):
            req=urllib.request.Request(base+route,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(req,timeout=60)as response:
                return response.read()if raw else json.load(response)
        p,base=start()
        try:
            assert {m['id']for m in call('/v1/workbench-models')['models']}=={'b','6fb','6f'}
            checks.append('three_customer_models_auto_registered_without_external_bundle')
            for rel,record in manifest['assets'].items():
                data=call('/meshes/rm75-cad/'+Path(rel).name,raw=True)
                assert len(data)==record['size_bytes']
                assert hashlib.sha256(data).hexdigest()==record['sha256']
            checks.append('all_actual_visual_meshes_served_with_matching_SHA256')
            for variant in ('b','6fb','6f'):
                record=call('/v1/workbench-model/'+variant);profile=record['profile'];q=record['poses']['preview']
                applied=call('/v1/workbench-active-profile',profile)
                assert applied['workbench_applied']and not applied['runtime_applied']
                assert all(v['zero_offset_rad']is None for v in profile['dynamics']['joints'].values())
                for move in ({'mode':'joint','target':{**q,'waist_pitch':.04,'left_joint_7':.12,'right_joint_7':-.12}},
                             {'mode':'object','translation_m':[.005,0,0]}):
                    task=call('/v1/planning-start',{'profile':profile,'start':q,'mapping':record['mapping'],**move})
                    deadline=time.monotonic()+65
                    while time.monotonic()<deadline:
                        result=call('/v1/planning-task/'+task['task_id'])
                        if result['status']!='running':break
                        time.sleep(.05)
                    assert result['status']=='succeeded',result
                    assert result['result']['validated_samples']>=2
                    assert not result['result']['hardware_executed']
                checks.append(variant+'_both_arms_waist_and_coupled_TCP_planning')
            stop=call('/v1/workbench-stop',{'engaged':True})
            try:
                call('/v1/robot-jog',{'joint':'waist_pitch','direction':1})
            except urllib.error.HTTPError as e:assert e.code==423
            else:raise AssertionError('stop did not latch')
            checks.append('software_stop_rejects_motion_423')
        finally:p.terminate();p.wait(timeout=10)
        p,base=start()
        try:
            active=call('/v1/workbench-active-profile')
            assert active['profile']==profile
            checks.append('customer_configuration_survives_process_restart')
        finally:p.terminate();p.wait(timeout=10)
    report.update(status='passed',check_count=len(checks),model_manifest_sha256=hashlib.sha256((bundle/'manifest.json').read_bytes()).hexdigest())
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable',type=Path);parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args();run(args.executable,args.report)
