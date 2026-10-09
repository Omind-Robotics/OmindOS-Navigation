import copy
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from robot_kinematics import build_model
from robot_sequence import sequence_trial
from robot_assets import approved_mesh_url


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())

    def test_analytic_arc_wheels_and_simultaneous_joint_motion(self):
        result = sequence_trial({'profile': self.profile, 'segments': [
            {'duration_s': 4, 'target': {'waist_pitch': .1}, 'linear_m_s': .1, 'angular_rad_s': .2},
            {'duration_s': 2, 'target': {'left_outer_finger_joint': .5}}]})
        # Cosine pulse integrates to 2s at peak speed: arc radius .5, heading .4.
        end = result['frames'][-1]
        self.assertAlmostEqual(end['base_pose']['x_m'], .5*math.sin(.4))
        self.assertAlmostEqual(end['base_pose']['y_m'], .5*(1-math.cos(.4)))
        self.assertAlmostEqual(end['positions']['left_front_wheel_joint'], (.1-.2*.295)*2/.1)
        self.assertAlmostEqual(end['positions']['waist_pitch'], .1)
        self.assertAlmostEqual(end['positions']['left_outer_finger_joint'], .5)
        midpoint = min(result['frames'], key=lambda f: abs(f['time_s']-2))
        self.assertAlmostEqual(midpoint['positions']['waist_pitch'], .05)
        self.assertGreater(midpoint['wheel_velocity_rad_s']['right_front_wheel_joint'], 0)
        self.assertFalse(result['contact_dynamics_validated'])

    def test_velocity_limits_and_external_meshes_are_not_bypassed(self):
        with self.assertRaisesRegex(ValueError, 'velocity'):
            sequence_trial({'profile': self.profile, 'segments': [{'duration_s': .5, 'target': {'waist_pitch': .5}}]})
        for value in ['https://example.com/a.STL', '/etc/passwd', 'package://RM75-B/meshes/../link_1.STL']:
            self.assertIsNone(approved_mesh_url(value))
        model = build_model(self.profile['kinematics']['urdf_xml'])
        self.assertEqual(sum(g['url'] is not None for l in model['links'] for g in l['visuals'] if g['type']=='mesh'), 16)

    def test_choreography_rejects_changed_structure(self):
        recipe = json.loads((ROOT/'web/robot-workbench/cooperative-demo.json').read_text())
        changed = copy.deepcopy(self.profile)
        changed['kinematics']['urdf_xml'] = changed['kinematics']['urdf_xml'].replace('xyz="0 0 0.1725"', 'xyz="0 0 0.1800"')
        # Change a guaranteed joint axis without touching the original manufacturer's file.
        changed['kinematics']['urdf_xml'] = changed['kinematics']['urdf_xml'].replace('axis xyz="0 0 1"', 'axis xyz="1 0 0"', 1)
        with self.assertRaisesRegex(ValueError, '参考关节结构'):
            sequence_trial({'profile': changed, **{k: recipe[k] for k in ('reference_signature','segments')}})


if __name__ == '__main__':
    unittest.main()
