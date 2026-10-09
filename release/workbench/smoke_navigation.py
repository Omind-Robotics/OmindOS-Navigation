"""Verify actual navigation and manipulation from the isolated executable."""
import argparse
import json
from pathlib import Path
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request,urlopen


def main():
    parser=argparse.ArgumentParser();parser.add_argument('executable',type=Path)
    parser.add_argument('--source',action='store_true');parser.add_argument('--report',type=Path)
    args=parser.parse_args();exe=args.executable.resolve();checks=[];measurements=[]
    with tempfile.TemporaryDirectory() as folder:
        with (Path(folder)/'log').open('w') as log:
            command=[sys.executable,str(exe)] if args.source else [str(exe)]
            process=subprocess.Popen(command+['--no-browser','--port','0','--store',folder],cwd=folder,stdout=subprocess.PIPE,stderr=log,text=True)
            lines=queue.Queue()
            def read():
                for line in process.stdout:lines.put(line)
                lines.put(None)
            threading.Thread(target=read,daemon=True).start()
            try:
                while True:
                    line=lines.get(timeout=45)
                    if line is None:raise RuntimeError((Path(folder)/'log').read_text())
                    match=re.search(r'http://127\.0\.0\.1:\d+',line)
                    if match:base=match.group();break
                def request(path,payload=None,expected=200):
                    req=Request(base+path,data=None if payload is None else json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
                    try:
                        with urlopen(req,timeout=20) as r:code,body=r.status,json.load(r)
                    except HTTPError as e:code,body=e.code,json.load(e)
                    assert code==expected,(code,body)
                    return body
                def wait(task,timeout=25):
                    deadline=time.monotonic()+timeout
                    while time.monotonic()<deadline:
                        task=request('/v1/navigation-task/'+task['task_id'])
                        if task['status'] in {'parked','succeeded','failed','canceled'}:return task
                        if task['status']=='manipulation_planning':
                            assert task['navigation']['parking_confirmed']
                            assert task['navigation']['stable_stop_s']>=.6
                        time.sleep(.1)
                    raise RuntimeError('Navigation acceptance timed out')
                profile=request('/example.json');config=request('/v1/navigation-config')['config']
                assert request('/v1/navigation-config')['drive_types']==['differential','omnidirectional']
                checks.append('two_drive_types_exposed')
                start={}
                for side,q in [('left',[1.57,1.3,-1.57,1.1,.1,.5,0]),('right',[-1.57,1.3,1.57,1.1,-.1,.5,0])]:start.update({f'{side}_joint_{i+1}':v for i,v in enumerate(q)})
                payload={'profile':profile,'start':start,'config':config,'goal':{'x_m':.12,'y_m':.1,'yaw_rad':.1},'manipulation':{'translation_m':[.005,0,0]}}
                for kind in ('differential','omnidirectional'):
                    data={**payload,'config':{**config,'drive_type':kind}}
                    task=request('/v1/navigation-start',data)
                    request('/v1/planning-start',{'profile':profile,'mode':'object','translation_m':[.01,0,0]},423)
                    request('/v1/robot-jog',{'profile':profile,'joint':'waist_pitch','direction':1},423)
                    result=wait(task)
                    assert result['status']=='succeeded',result
                    nav=result['navigation'];arm=result['result']
                    assert nav['drive_type']==kind and nav['parking_confirmed'] and nav['stable_stop_s']>=.6
                    assert all(abs(v)<1e-9 for v in nav['cmd_vel'].values())
                    assert arm['validated_samples']>1 and arm['max_tcp_position_error_m']<=.001
                    assert arm['navigation_map_checked']
                    assert all(f['base_pose']==nav['base_pose'] for f in arm['frames'])
                    measurements.append({'drive_type':kind,'base_pose':nav['base_pose'],'stable_stop_s':nav['stable_stop_s'],'arm_samples':arm['validated_samples'],'max_tcp_error_m':arm['max_tcp_position_error_m']})
                    checks.append(kind+'_arrival_parking_then_actual_runtime2')
                checks+=['concurrent_arm_and_jog_blocked','zero_command_and_observed_stop','fixed_base_during_arm_planning']
                checks.append('imported_map_guard_present_in_arm_planning')
                request('/v1/workbench-active-profile',profile)
                stale=request('/v1/navigation-task/'+result['task_id']);assert stale['status']=='canceled' and stale['result'] is None
                checks.append('parameter_epoch_invalidates_completed_task')
                blocked={**payload,'obstacles':[{'size_m':[.1,.1,.1],'position_m':[.12,.1,.1]}]}
                result=wait(request('/v1/navigation-start',blocked));assert result['status']=='failed' and result['result'] is None
                checks.append('blocked_goal_no_partial_success')
                unknown={**payload,'config':{**config,'map':{**config['map'],'data':[-1]*10000}}}
                result=wait(request('/v1/navigation-start',unknown));assert result['status']=='failed'
                checks.append('unknown_map_fail_closed')
                task=request('/v1/navigation-start',payload);result=request('/v1/navigation-cancel',{'task_id':task['task_id']})
                assert result['status']=='canceled' and result['result'] is None
                checks.append('cancel_prevents_automatic_arm_task')
                task=request('/v1/navigation-start',payload);request('/v1/workbench-stop',{'engaged':True})
                result=request('/v1/navigation-task/'+task['task_id']);assert result['status']=='canceled' and result['result'] is None
                request('/v1/navigation-start',payload,423)
                checks.append('estop_cancels_navigation_and_locks_restart')
                state=request('/v1/workbench-state');request('/v1/workbench-stop',{'engaged':False,'expected_epoch':state['motion_epoch']})
                checks.append('reset_does_not_resume_task')
            finally:process.terminate();process.wait(timeout=10)
    report={'status':'passed','source_mode':args.source,'checks':checks,'measurements':measurements,'hardware_executed':False,'localization_source':'geometric_simulator'}
    if args.report:args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
