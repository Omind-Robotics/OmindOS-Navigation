#!/usr/bin/env python3
"""Exercise the actual compiled executable from a directory without source files."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
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
import time
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable', type=Path)
    parser.add_argument('--source', action='store_true', help='Development-only source smoke, not a binary acceptance result')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    executable = args.executable.resolve()
    command = [sys.executable, str(executable)] if args.source else [str(executable)]
    checks = []
    if not args.source:
        subprocess.run(command + ['--verify'], check=True, timeout=30)
        checks += ['manifest_verification', 'compiled_executable_without_python_command']
    with tempfile.TemporaryDirectory(prefix='omindos-customer-') as temporary:
        folder = Path(temporary)
        store = folder / 'saved-configurations'
        def start(extra=()):
            log = (folder / 'stderr.log').open('w', encoding='utf-8')
            process = subprocess.Popen(command + ['--no-browser', '--port', '0', '--store', str(store), *extra],
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
            for path in ('/', '/workbench.js', '/robot-scene.js', '/workbench.css', '/runtime.js'):
                assert len(request(base,path)) > 100
            checks.append('all_local_assets_served')
            try:
                request(base, '/v1/runtime/status')
                raise AssertionError('Unconfigured ROS bridge reported success')
            except urllib.error.HTTPError as error:
                assert error.code == 503
            checks.append('unconfigured_runtime_reported_without_blocking_local_preview')
            profile = request(base,'/example.json')
            profile['robot_id'] = 'release-smoke'
            options = request(base,'/v1/connection-options')
            assert set(options['devices']) == {'left_arm','right_arm','waist','chassis'}
            assert options['adapter_status'] == 'not_integrated' and options['hardware_connected'] is False
            profile['connections'] = {'schema_version':1,'devices':{
                'left_arm':{'selected':'sdk_tcp','profiles':{'sdk_tcp':{'host':'192.0.2.10','port':8080}}},
                'right_arm':{'selected':'json_tcp','profiles':{'json_tcp':{'host':'192.0.2.11','port':8081}}},
                'waist':{'selected':'unselected','profiles':{'canopen':{'node_id':3},'ethercat':{'slave_position':0}}},
                'chassis':{'selected':'unselected','profiles':{'rs232':{'device':'COM3'},'mqtt':{'host':'broker.test'}}}}}
            applied = request(base,'/v1/workbench-active-profile',profile)
            assert applied['workbench_applied'] and not applied['runtime_applied']
            assert applied['hardware_connected'] is False
            checks.append('connection_options_and_independent_drafts_saved_without_connecting')
            for velocity, angular, expected in ((.15,0,.225),(-.15,0,-.225),(0,.4,.6),(0,-.4,-.6)):
                result = request(base,'/v1/robot-wheel-base',{'linear_m_s':velocity,'angular_rad_s':angular,'duration_s':3})
                assert not result['runtime_applied']
                key = 'x_m' if velocity else 'yaw_rad'
                assert math.isclose(result['frames'][-1]['base_pose'][key],expected,abs_tol=1e-10)
            checks.append('four_wheel_forward_reverse_left_right')
            stop = request(base,'/v1/workbench-stop',{'engaged':True})
            profile['connections']['devices']['waist']['selected'] = 'canopen'
            applied = request(base,'/v1/workbench-active-profile',profile)
            assert applied['emergency_stop'] and not applied['hardware_connected']
            checks.append('saving_connection_selection_preserves_emergency_stop')
            try:
                request(base,'/v1/robot-wheel-base',{'linear_m_s':.1})
                raise AssertionError('Stopped server accepted motion')
            except urllib.error.HTTPError as error:
                assert error.code == 423
            reset = request(base,'/v1/workbench-stop',{'engaged':False,'expected_epoch':applied['motion_epoch']})
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
            checks.append('all_connection_drafts_and_selected_protocol_survive_restart')
        finally:
            process.terminate(); process.wait(timeout=10); log.close()
        # This fixture tests the compiled desktop CLI and bridge transport, not ROS DDS.
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
        from robot_profile_runtime_core import ProfileRuntime, RuntimeErrorResponse
        core = ProfileRuntime(store)
        core_lock = threading.Lock()
        class PreviewFixture(BaseHTTPRequestHandler):
            def respond(self, code, body):
                raw = json.dumps(body, allow_nan=False).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers(); self.wfile.write(raw)
            def do_GET(self):
                with core_lock:
                    core.tick()
                    self.respond(200, core.snapshot())
            def do_POST(self):
                payload = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                with core_lock:
                    try:
                        self.respond(200, core.command(self.path.rsplit('/',1)[1], payload))
                    except RuntimeErrorResponse as error:
                        self.respond(error.code, {'error': str(error)})
            def log_message(self, *unused): pass
        fixture = ThreadingHTTPServer(('127.0.0.1', 0), PreviewFixture)
        worker = threading.Thread(target=fixture.serve_forever, daemon=True)
        worker.start()
        process, log, base = start(['--runtime-url', f'http://127.0.0.1:{fixture.server_port}'])
        try:
            assert request(base, '/v1/runtime/status')['state'] == 'UNLOADED'
            registered = request(base, '/v1/robot-profiles', profile)
            loaded = request(base, '/v1/runtime/load', {'profile_id': registered['profile_id']})
            assert loaded['runtime_applied'] and not loaded['hardware_active']
            token = loaded['lease_token']
            target = loaded['pose']['positions'].copy(); target['left_joint_4'] = .2
            run = {'profile_id':registered['profile_id'], 'lease_token':token, 'target':target, 'duration_s':2}
            assert request(base, '/v1/runtime/run', run)['state'] == 'RUNNING'
            time.sleep(.12)
            moving = request(base, '/v1/runtime/heartbeat', {'lease_token':token})
            assert moving['pose']['positions']['left_joint_4'] > 0
            checks.append('desktop_cli_runtime_url_load_run_heartbeat_and_pose_feedback')
            stopped = request(base, '/v1/workbench-stop', {'engaged':True})
            assert stopped['ros_preview_stop_confirmed']
            state = request(base, '/v1/runtime/status')
            assert state['state'] == 'STOPPED' and not state['hardware_active']
            held = state['pose']['positions']
            try:
                request(base, '/v1/runtime/run', run)
                raise AssertionError('Stopped desktop accepted a ROS preview run')
            except urllib.error.HTTPError as error:
                assert error.code == 423
            request(base, '/v1/workbench-stop', {'engaged':False, 'expected_epoch':stopped['motion_epoch']})
            state = request(base, '/v1/runtime/status')
            assert state['state'] == 'STOPPED' and state['pose']['positions'] == held
            checks.append('estop_forwards_to_ros_preview_and_reset_never_resumes')
        finally:
            fixture.shutdown(); fixture.server_close(); worker.join(timeout=2)
            if sys.exc_info()[0] is not None:
                process.terminate(); process.wait(timeout=10); log.close()
        try:
            request(base, '/v1/runtime/status')
            raise AssertionError('Unavailable preview service reported success')
        except urllib.error.HTTPError as error:
            assert error.code == 503
            assert len(request(base, '/runtime.js')) > 100
            checks.append('missing_runtime_reported_while_local_assets_remain_available')
        finally:
            process.terminate(); process.wait(timeout=10); log.close()
        invalid = subprocess.run(command + ['--no-browser', '--port', '0', '--store', str(store),
                                            '--runtime-url', 'http://192.0.2.1:8086'],
                                 capture_output=True, text=True, timeout=20)
        assert invalid.returncode != 0 and '--runtime-url must be' in invalid.stderr
        checks.append('desktop_rejects_non_loopback_runtime_url')
    report = {'status':'PASS','system':platform.system(),'machine':platform.machine(), 'checks':checks,
              'hardware_validated':False,'ros_required':False,'ros_runtime_bundled':False,
              'native_ros_validation':False,'runtime_bridge_test':'localhost_protocol_fixture',
              'compiled_executable':not args.source}
    destination = args.report or executable.parent/'VALIDATION.json'
    destination.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()


