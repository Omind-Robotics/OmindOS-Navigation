#!/usr/bin/env python3
"""Exercise the actual compiled executable from a directory without source files."""
import json
import math
import os
from pathlib import Path
import platform
import queue
import re
import subprocess
import sys
import tempfile
import threading
import urllib.error
import urllib.request


def main():
    executable = Path(sys.argv[1]).resolve()
    subprocess.run([str(executable), '--verify'], check=True, timeout=30)
    checks = ['manifest_verification', 'compiled_executable_without_python_command']
    with tempfile.TemporaryDirectory(prefix='omindos-customer-') as temporary:
        folder = Path(temporary)
        store = folder / 'saved-configurations'
        def start():
            log = (folder / 'stderr.log').open('w', encoding='utf-8')
            process = subprocess.Popen([str(executable), '--no-browser', '--port', '0', '--store', str(store)],
                                       cwd=folder, stdout=subprocess.PIPE, stderr=log, text=True, encoding='utf-8', errors='replace',
                                       env={k:v for k,v in os.environ.items() if k not in ('PYTHONHOME','PYTHONPATH')})
            lines = queue.Queue()
            def read():
                for line in process.stdout:
                    lines.put(line)
                lines.put(None)
            threading.Thread(target=read, daemon=True).start()
            try:
                while True:
                    line = lines.get(timeout=45)
                    if line is None:
                        raise RuntimeError('Executable exited: '+(folder/'stderr.log').read_text(encoding='utf-8'))
                    match = re.search(r'http://127\.0\.0\.1:\d+/', line)
                    if match:
                        return process, log, match.group().rstrip('/')
            except BaseException:
                process.terminate(); process.wait(timeout=10); log.close()
                raise
        def request(base, path, data=None):
            req = urllib.request.Request(base+path, data=None if data is None else json.dumps(data).encode(),
                                         headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(req, timeout=15) as response:
                content = response.read()
                return json.loads(content) if 'application/json' in response.headers['Content-Type'] else content
        process, log, base = start()
        try:
            for path in ('/', '/workbench.js', '/robot-scene.js', '/workbench.css'):
                assert len(request(base,path)) > 100
            checks.append('all_local_assets_served')
            profile = request(base,'/example.json')
            profile['robot_id'] = 'release-smoke'
            applied = request(base,'/v1/workbench-active-profile',profile)
            assert applied['workbench_applied'] and not applied['runtime_applied']
            for velocity, angular, expected in ((.15,0,.225),(-.15,0,-.225),(0,.4,.6),(0,-.4,-.6)):
                result = request(base,'/v1/robot-wheel-base',{'linear_m_s':velocity,'angular_rad_s':angular,'duration_s':3})
                assert not result['runtime_applied']
                key = 'x_m' if velocity else 'yaw_rad'
                assert math.isclose(result['frames'][-1]['base_pose'][key],expected,abs_tol=1e-10)
            checks.append('four_wheel_forward_reverse_left_right')
            stop = request(base,'/v1/workbench-stop',{'engaged':True})
            try:
                request(base,'/v1/robot-wheel-base',{'linear_m_s':.1})
                raise AssertionError('Stopped server accepted motion')
            except urllib.error.HTTPError as error:
                assert error.code == 423
            reset = request(base,'/v1/workbench-stop',{'engaged':False,'expected_epoch':stop['motion_epoch']})
            assert not reset['emergency_stop']
            checks.append('estop_rejects_motion_and_manual_reset')
        finally:
            process.terminate(); process.wait(timeout=10); log.close()
        process, log, base = start()
        try:
            active = request(base,'/v1/workbench-active-profile')
            assert active['profile'] == profile
            assert active['active_profile_id'] == applied['active_profile_id']
            checks.append('saved_configuration_survives_executable_restart')
        finally:
            process.terminate(); process.wait(timeout=10); log.close()
    report = {'status':'PASS','system':platform.system(),'machine':platform.machine(), 'checks':checks,
              'hardware_validated':False,'ros_required':False}
    (executable.parent/'VALIDATION.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
