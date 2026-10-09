"""Acceptance of actual planning endpoints from the executable in isolation."""
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
import urllib.request


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('executable',type=Path)
    parser.add_argument('--source',action='store_true')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    exe=args.executable.resolve()
    command=[sys.executable,str(exe)] if args.source else [str(exe)]
    checks=[]
    with tempfile.TemporaryDirectory() as folder:
        log=(Path(folder)/'log').open('w')
        process=subprocess.Popen(command+['--no-browser','--port','0','--store',folder],cwd=folder,
                                 stdout=subprocess.PIPE,stderr=log,text=True)
        lines=queue.Queue()
        def read():
            for line in process.stdout:lines.put(line)
            lines.put(None)
        threading.Thread(target=read,daemon=True).start()
        try:
            while True:
                line=lines.get(timeout=45)
                if line is None:raise RuntimeError('workbench exited: '+(Path(folder)/'log').read_text())
                match=re.search(r'http://127\.0\.0\.1:\d+',line)
                if match:base=match.group();break
            def request(path,payload=None):
                req=urllib.request.Request(base+path,data=None if payload is None else json.dumps(payload).encode(),
                                           headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(req,timeout=15) as response:return json.load(response)
            def wait(task):
                deadline=time.monotonic()+65
                while time.monotonic()<deadline:
                    task=request('/v1/planning-task/'+task['task_id'])
                    if task['status']!='running':return task
                    time.sleep(.1)
                raise RuntimeError('Planning timeout')
            profile=request('/example.json'); mapping=request('/v1/planning-config')['mapping']
            start={}
            for side,q in [('left',[1.57,1.3,-1.57,1.1,.1,.5,0]),('right',[-1.57,1.3,1.57,1.1,-.1,.5,0])]:
                start.update({f'{side}_joint_{i+1}':v for i,v in enumerate(q)})
            pose=request('/v1/robot-model',{'urdf_xml':profile['kinematics']['urdf_xml'],'positions':start})['pose']
            start=pose['positions']
            payload={'profile':profile,'mode':'object','mapping':mapping,'start':start,'translation_m':[.01,0,0]}
            task=wait(request('/v1/planning-start',payload))
            assert task['status']=='succeeded',task
            result=task['result']
            assert result['validated_samples']>1 and result['max_tcp_position_error_m']<=.001
            assert result['max_relative_position_error_m']<=.002
            assert result['collision_geometry']=='conservative_oriented_boxes' and result['timed'] is False
            assert all(f['base_pose']==pose['base_pose'] for f in result['frames'])
            checks+=['actual_coupled_solver','both_tcp_and_relative_constraints','fixed_base','sampled_collision_scope']
            tcp=[pose['link_transforms']['left_tip'][i][3] for i in range(3)]
            blocked={**payload,'obstacles':[{'size_m':[.04,.04,.04],'position_m':tcp}]}
            task=wait(request('/v1/planning-start',blocked))
            assert task['status']=='failed' and task['result'] is None
            checks.append('obstacle_blocks_path_without_partial_success')
            goal={**start,'waist_pitch':.02}
            task=wait(request('/v1/planning-start',{**payload,'mode':'joint','target':goal}))
            assert task['status']=='succeeded',task
            assert task['result']['frames'][-1]['positions']['waist_pitch']==.02
            checks.append('joint_path_planning')
            request('/v1/workbench-active-profile',profile)
            stale=request('/v1/planning-task/'+task['task_id'])
            assert stale['status']=='canceled' and stale['result'] is None
            checks.append('parameter_write_invalidates_completed_path')
            task=request('/v1/planning-start',payload)
            canceled=request('/v1/planning-cancel',{'task_id':task['task_id']})
            assert canceled['status']=='canceled' and canceled['result'] is None
            checks.append('calculation_cancel')
            task=request('/v1/planning-start',payload)
            request('/v1/workbench-stop',{'engaged':True})
            task=wait(task)
            assert task['status']=='canceled' and task['result'] is None
            checks.append('software_stop_invalidates_planning')
        finally:
            process.terminate();process.wait(timeout=10);log.close()
    report={'status':'passed','source_mode':args.source,'checks':checks,
            'hardware_executed':False,'dynamics_validated':False,'windows_validated':False}
    if args.report:args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
