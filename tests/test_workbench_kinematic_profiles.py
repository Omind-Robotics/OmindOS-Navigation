"""Explicit kinematic-only saves must not weaken legacy/runtime validation."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import robot_parameter_api as API
import workbench_state as WS
from robot_workbench import make_handler
from test_robot_parameter_api import fixture as full_fixture


def preview_fixture(with_drafts=True):
    # Synthetic geometry exercises missing physical metadata, not RM75 calibration.
    xml = ('<robot name="preview"><link name="base"/><link name="arm"/>'
           '<joint name="hinge" type="revolute"><parent link="base"/>'
           '<child link="arm"/><axis xyz="0 1 0"/>'
           '<limit lower="-1" upper="1" effort="0" velocity="0.2"/>'
           '</joint></robot>')
    profile = {'profile_mode': 'kinematic_preview', 'robot_id': 'preview-a',
               'kinematics': {'urdf_xml': xml}}
    if with_drafts:
        profile['dynamics'] = {'joints': {'hinge': {
            'motor_model': '', 'reduction_ratio': None, 'direction': None,
            'zero_offset_rad': None, 'torque_limit_nm': None,
            'velocity_limit_rad_s': None}}}
    return profile


class KinematicProfileTests(unittest.TestCase):
    def test_save_restart_preserves_unknowns_without_inserting_physics(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = preview_fixture()
            original = copy.deepcopy(profile)
            state = WS.WorkbenchState(folder)
            saved = state.apply(profile)
            self.assertEqual(profile, original)
            self.assertEqual(saved['validation_scope'], 'kinematic_configuration_only')
            self.assertFalse(saved['dynamics_validated'])
            self.assertFalse(saved['hardware_validated'])
            self.assertFalse(saved['runtime_applied'])
            self.assertTrue(saved['workbench_applied'])
            restarted = WS.WorkbenchState(folder)
            self.assertEqual(restarted.profile_id, saved['profile_id'])
            self.assertEqual(restarted.active_profile(), original)
            self.assertFalse(restarted.snapshot()['hardware_connected'])

    def test_geometry_only_profile_does_not_require_drive_drafts(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = preview_fixture(False)
            WS.WorkbenchState(folder).apply(profile)
            self.assertEqual(WS.WorkbenchState(folder).active_profile(), profile)
            self.assertNotIn('dynamics', profile)

    def test_old_versions_and_identical_saves_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            state = WS.WorkbenchState(folder)
            old = preview_fixture()
            first = state.apply(old)
            self.assertEqual(first['profile_id'], state.apply(old)['profile_id'])
            new = copy.deepcopy(old)
            new['robot_id'] = 'preview-b'
            second = state.apply(new)
            self.assertNotEqual(first['profile_id'], second['profile_id'])
            self.assertEqual(WS._load_workbench_profile(first['profile_id'], folder), old)
            self.assertEqual(WS.WorkbenchState(folder).active_profile(), new)

    def test_full_parameter_registry_and_hash_are_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = full_fixture()
            original = API.register_robot_parameters(profile, folder)
            saved = WS.WorkbenchState(folder).apply(profile)
            self.assertEqual(saved['profile_id'], original['profile_id'])
            self.assertEqual(WS.WorkbenchState(folder).active_profile(), profile)
            self.assertEqual(API.load_robot_profile(saved['profile_id'], folder), profile)

    def test_explicit_opt_in_is_required(self):
        for mode in (None, '', 'kinematic', 'full_parameters', True):
            profile = preview_fixture()
            if mode is None:
                profile.pop('profile_mode')
            else:
                profile['profile_mode'] = mode
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                with self.assertRaises(ValueError):
                    WS.WorkbenchState(folder).apply(profile)
                self.assertFalse((Path(folder) / 'workbench-active.json').exists())

    def test_runtime_registry_still_rejects_kinematic_preview(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = preview_fixture()
            saved = WS.WorkbenchState(folder).apply(profile)
            with self.assertRaises(ValueError):
                API.validate_parameters(profile)
            with self.assertRaises(ValueError):
                API.load_robot_profile(saved['profile_id'], folder)
            with self.assertRaises(ValueError):
                API.register_robot_parameters(profile, folder)

    def test_invalid_draft_values_and_mapping_are_rejected(self):
        cases = []
        for field, value in (('direction', True), ('direction', 0),
                             ('reduction_ratio', -1), ('torque_limit_nm', '2'),
                             ('zero_offset_rad', float('nan')),
                             ('velocity_limit_rad_s', float('inf')),
                             ('motor_model', 123)):
            profile = preview_fixture()
            profile['dynamics']['joints']['hinge'][field] = value
            cases.append(profile)
        wrong = preview_fixture()
        wrong['dynamics']['joints']['wrong'] = wrong['dynamics']['joints'].pop('hinge')
        cases.append(wrong)
        for profile in cases:
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                WS._validate_workbench_profile(profile)

    def test_finite_partial_drafts_are_preserved_not_promoted(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = preview_fixture()
            profile['dynamics']['joints']['hinge']['velocity_limit_rad_s'] = 0.1
            saved = WS.WorkbenchState(folder).apply(profile)
            self.assertFalse(saved['dynamics_validated'])
            self.assertEqual(WS.WorkbenchState(folder).active_profile(), profile)

    def test_bad_geometry_and_unsafe_identifiers_are_rejected(self):
        cases = []
        for old, new in (('axis xyz="0 1 0"', 'axis xyz="0 0 0"'),
                         ('child link="arm"', 'child link="missing"'),
                         ('lower="-1"', 'lower="2"')):
            p = preview_fixture()
            p['kinematics']['urdf_xml'] = p['kinematics']['urdf_xml'].replace(old, new)
            cases.append(p)
        p = preview_fixture()
        p['kinematics']['urdf_xml'] = '<!DOCTYPE robot>' + p['kinematics']['urdf_xml']
        cases.append(p)
        p = preview_fixture()
        p['robot_id'] = '../unsafe'
        cases.append(p)
        p = preview_fixture()
        p['hardware_validated'] = True
        cases.append(p)
        for profile in cases:
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                WS._validate_workbench_profile(profile)

    def test_tampered_saved_profile_is_rejected_on_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = preview_fixture()
            saved = WS.WorkbenchState(folder).apply(profile)
            profile['robot_id'] = 'tampered'
            (Path(folder) / (saved['profile_id'] + '.json')).write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError, 'checksum'):
                WS.WorkbenchState(folder)

    def test_apply_does_not_release_stop_and_invalidates_old_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            state = WS.WorkbenchState(folder)
            old_epoch = state.check_motion()
            state.set_stop({'engaged': True})
            state.apply(preview_fixture())
            self.assertTrue(state.snapshot()['emergency_stop'])
            with self.assertRaises(WS.MotionLocked):
                state.check_motion(old_epoch)
            self.assertEqual(list(Path(folder).glob('.kinematic-*')), [])

    def test_existing_http_apply_and_restart_read_round_trip(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = preview_fixture()
            saved_id = None
            for restart in (False, True):
                server = API.ThreadingHTTPServer(('127.0.0.1', 0), make_handler(folder))
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                base = 'http://127.0.0.1:' + str(server.server_port)
                try:
                    if not restart:
                        req = urllib.request.Request(base + '/v1/workbench-active-profile',
                            data=json.dumps(profile).encode(),
                            headers={'Content-Type': 'application/json'})
                        with urllib.request.urlopen(req, timeout=5) as response:
                            self.assertEqual(response.status, 200)
                            saved_id = json.load(response)['profile_id']
                    with urllib.request.urlopen(base + '/v1/workbench-active-profile', timeout=5) as response:
                        body = json.load(response)
                    self.assertEqual(body['profile'], profile)
                    self.assertEqual(body['active_profile_id'], saved_id)
                    self.assertFalse(body['hardware_connected'])
                finally:
                    server.shutdown()
                    thread.join(timeout=5)
                    server.server_close()


if __name__ == '__main__':
    unittest.main()
