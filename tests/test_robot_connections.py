import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from robot_connections import CATALOG, validate_connections
from robot_parameter_api import register_robot_parameters, load_robot_profile
from robot_workbench import make_handler, ThreadingHTTPServer
from workbench_state import WorkbenchState
from test_robot_workbench import fixture


def settings():
    return {'schema_version': 1, 'devices': {
        'left_arm': {'selected': 'sdk_tcp', 'profiles': {'sdk_tcp': {'host': '192.0.2.10', 'port': 8080}, 'json_tcp': {'port': 8081}}},
        'right_arm': {'selected': 'json_tcp', 'profiles': {'json_tcp': {'host': '192.0.2.11', 'port': 8082}}},
        'waist': {'selected': 'canopen', 'profiles': {'canopen': {'interface': 'can0', 'node_id': 3}, 'ethercat': {'slave_position': 0}}},
        'chassis': {'selected': 'unselected', 'profiles': {'rs232': {'device': 'COM3', 'baud_rate': None}, 'mqtt': {'host': 'broker.test'}}},
    }}


class ConnectionTests(unittest.TestCase):
    def test_legacy_profile_hash_and_content_are_unchanged(self):
        profile = fixture()
        expected = hashlib.sha256(json.dumps(profile, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()).hexdigest()
        with tempfile.TemporaryDirectory() as folder:
            saved = register_robot_parameters(profile, folder)
            self.assertEqual(saved['profile_id'], expected)
            self.assertEqual(load_robot_profile(expected, folder), profile)
            self.assertNotIn('connections', profile)

    def test_all_protocol_options_can_be_saved_as_incomplete_drafts(self):
        for device, spec in CATALOG.items():
            for selected in ['unselected', *spec['protocols']]:
                with self.subTest(device=device, selected=selected):
                    result = validate_connections({'schema_version': 1, 'devices': {device: {
                        'selected': selected, 'profiles': {key: {} for key in spec['protocols']}}}})
                    self.assertFalse(result['hardware_connected'])
                    self.assertEqual(result['adapter_status'], 'not_integrated')

    def test_active_and_inactive_options_and_independent_arms_survive_restart(self):
        profile = fixture(); profile['connections'] = settings()
        with tempfile.TemporaryDirectory() as folder:
            state = WorkbenchState(folder); first = state.apply(profile)
            changed = copy.deepcopy(profile); changed['connections']['devices']['waist']['selected'] = 'ethercat'
            second = state.apply(changed)
            self.assertNotEqual(first['profile_id'], second['profile_id'])
            self.assertEqual(WorkbenchState(folder).active_profile(), changed)
            self.assertEqual(load_robot_profile(first['profile_id'], folder), profile)
            self.assertFalse(second['runtime_applied'])

    def test_invalid_hidden_options_and_unknown_fields_are_rejected(self):
        bad = []
        for value in [0, 65536, True, 2.5, '8080']:
            config = settings(); config['devices']['left_arm']['profiles']['json_tcp']['port'] = value; bad.append(config)
        config = settings(); config['schema_version'] = True; bad.append(config)
        config = settings(); config['devices']['waist']['selected'] = 'mqtt'; bad.append(config)
        config = settings(); config['devices']['chassis']['profiles']['mqtt']['password'] = 'do-not-store'; bad.append(config)
        config = settings(); config['devices']['left_arm']['profiles']['sdk_tcp']['host'] = 'bad\naddress'; bad.append(config)
        for config in bad:
            with self.subTest(config=config), self.assertRaises(ValueError):
                validate_connections(config)

    def test_http_options_save_reload_and_reject_invalid_import_without_unlocking(self):
        with tempfile.TemporaryDirectory() as folder:
            server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(folder))
            worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
            base = f'http://127.0.0.1:{server.server_port}'
            def request(path, payload=None):
                data = None if payload is None else json.dumps(payload).encode()
                try:
                    with urllib.request.urlopen(urllib.request.Request(base+path, data=data, headers={'Content-Type':'application/json'}), timeout=5) as response:
                        return response.status, json.load(response)
                except urllib.error.HTTPError as error:
                    return error.code, json.load(error)
            try:
                code, options = request('/v1/connection-options'); self.assertEqual(code, 200)
                self.assertEqual(set(options['devices']), {'left_arm', 'right_arm', 'waist', 'chassis'})
                self.assertEqual(options['adapter_status'], 'not_integrated')
                request('/v1/workbench-stop', {'engaged': True})
                profile = fixture(); profile['connections'] = settings()
                code, saved = request('/v1/workbench-active-profile', profile)
                self.assertEqual(code, 200); self.assertTrue(saved['emergency_stop']); self.assertFalse(saved['hardware_connected'])
                self.assertEqual(request('/v1/workbench-active-profile')[1]['profile'], profile)
                invalid = copy.deepcopy(profile); invalid['connections']['devices']['waist']['selected'] = 'invalid'
                self.assertEqual(request('/v1/robot-model', {'urdf_xml': profile['kinematics']['urdf_xml'], 'connections': invalid['connections']})[0], 400)
                self.assertEqual(request('/v1/workbench-active-profile', invalid)[0], 400)
                self.assertEqual(request('/v1/workbench-active-profile')[1]['profile'], profile)
                self.assertEqual(request('/v1/robot-jog', {'joint': 'hinge', 'direction': 1})[0], 423)
            finally:
                server.shutdown(); worker.join(5); server.server_close()
