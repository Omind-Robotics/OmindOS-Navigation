#!/usr/bin/env python3
"""Validate a cleaned image against the frozen preview.5 test fixtures."""
import argparse, hashlib, json, os, re, shutil, subprocess
from pathlib import Path

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--image',required=True)
    p.add_argument('--base-build',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    image=json.loads(subprocess.check_output(['docker','image','inspect',a.image]))[0]
    baseline=json.loads(subprocess.check_output(['docker','image','inspect','omindos/control-preview:0.2.0-preview.5']))[0]
    env=['-e','ROS_DOMAIN_ID=228','-e','ROS_LOCALHOST_ONLY=1','-e','PYTHONDONTWRITEBYTECODE=1']
    common=['docker','run','--rm','--init','--network','none','--cap-drop','ALL',
            '--security-opt','no-new-privileges','-v',str(a.base_build.resolve())+':/baseline:ro',
            '-v',str(out)+':/qa',*env,'--entrypoint','bash',image['Id'],'-lc']
    def run(name,script):
        with (out/(name+'.log')).open('w') as log:
            subprocess.run(common+['set -e; source /opt/ros/humble/setup.bash; '+script],
                           check=True,stdout=log,stderr=subprocess.STDOUT,timeout=600)
        print(name+' PASS',flush=True)
    run('native-audit','/opt/omindos-runtime/bin/python /baseline/audit-native.py')
    run('regression',"""cp -a /baseline/native-tests /qa/native-tests
cp -a /opt/omindos/scripts/. /qa/native-tests/scripts/
export PYTHONPATH=/qa/native-tests/scripts:$PYTHONPATH
/opt/omindos-runtime/bin/python -m unittest discover -s /qa/native-tests/tests -v""")
    run('ros-profile',"""export PYTHONPATH=/qa/native-tests/scripts:$PYTHONPATH
/opt/omindos-runtime/bin/python /qa/native-tests/scripts/check_robot_profile_ros.py""")
    run('dynamics','omindos-runtime validate-dynamics --source /opt/omindos/reference --output /qa/dynamics.json')
    run('ros-quadruped',"""export PYTHONPATH=/opt/omindos/scripts:$PYTHONPATH
omindos-runtime quadruped --port 18107 --token-file /tmp/quad.token > /qa/worker.log 2>&1 & worker=$!
trap 'kill "$worker" ${bridge:-} 2>/dev/null || true' EXIT
for n in $(seq 1 200); do test -f /tmp/quad.token && break; kill -0 "$worker"; sleep .1; done
omindos-runtime quadruped-ros --port 18107 --token-file /tmp/quad.token > /qa/bridge.log 2>&1 & bridge=$!
/opt/omindos-runtime/bin/python /baseline/source/scripts/check_quadruped_ros2.py --output /qa/ros-quadruped.json""")
    code="""import hashlib,json
from pathlib import Path
r=Path('/opt/omindos')
print(json.dumps({str(p.relative_to(r)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(r.rglob('*')) if p.is_file()}))
"""
    def app_hashes(ident):
        return json.loads(subprocess.check_output(['docker','run','--rm','--network','none',
            '--entrypoint','/opt/omindos-runtime/bin/python',ident,'-c',code],text=True))
    before=app_hashes(baseline['Id']);after=app_hashes(image['Id'])
    assert before==after,'Application/model bytes changed'
    report={'status':'PASS','image_id':image['Id'],'baseline_image_id':baseline['Id'],
            'unchanged_application_files':len(after),'unchanged_application_hashes':after,
            'compiled_regression_count':int(re.search(r'Ran (\d+) tests',(out/'regression.log').read_text()).group(1)),
            'native_audit':json.loads((out/'native-audit.log').read_text().splitlines()[-1]),
            'dynamics':json.loads((out/'dynamics.json').read_text()),
            'ros_profile_log':(out/'ros-profile.log').read_text(),
            'ros_quadruped':json.loads((out/'ros-quadruped.json').read_text()),
            'hardware_validated':False,'network':'isolated container loopback'}
    (out/'VALIDATION.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['status','image_id','unchanged_application_files','compiled_regression_count']}))

if __name__=='__main__':main()

