"""Reference asset trust boundary and compatibility with the shared workbench."""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'release/workbench'))
from robot_assets import approved_mesh_url, MEVIUS2_FILES
from robot_kinematics import build_model, pose
from robot_parameter_api import validate_parameters
from fetch_mevius2_workbench import verified
from quadruped_variant import demo_profile


class ReferenceTests(unittest.TestCase):
    def test_reference_assets_and_customer_model_are_distinct(self):
        profile = demo_profile(ROOT)
        validate_parameters(profile)
        model = build_model(profile['kinematics']['urdf_xml'])
        meshes = [g for link in model['links'] for g in link['visuals'] if g['type'] == 'mesh']
        self.assertEqual(len(meshes), 13)
        self.assertTrue(all(g['url'].startswith('/meshes/mevius2/') for g in meshes))
        self.assertEqual(model['warnings'], [])
        self.assertEqual(len(profile['dynamics']['joints']), 12)
        self.assertTrue(all(d['motor_model'] == 'MEVIUS2-REFERENCE-NOT-CUSTOMER'
                            for d in profile['dynamics']['joints'].values()))
        q = {n: .7 if '_hip_' in n else -1.2 if '_knee_' in n else 0
             for n in profile['dynamics']['joints']}
        frame = pose(model, q)
        feet = [frame['link_transforms'][leg + '_foot_link'][2][3] for leg in ('FR','FL','BR','BL')]
        self.assertLess(max(feet) - min(feet), 1e-8)

    def test_mesh_paths_cannot_escape_the_catalog(self):
        for name in MEVIUS2_FILES:
            self.assertEqual(approved_mesh_url('package://mevius2/models/meshes/' + name), '/meshes/mevius2/' + name)
        for path in ('../LICENSE', 'base_link.STL?x=1', '/base_link.STL', 'unknown.STL', 'FR_calf_link.STL/../../LICENSE'):
            self.assertIsNone(approved_mesh_url('package://mevius2/models/meshes/' + path))
        for path in ('file:///etc/passwd', 'https://example.com/base_link.STL'):
            self.assertIsNone(approved_mesh_url(path))

    def test_every_pinned_file_is_verified_and_a_changed_byte_is_rejected(self):
        root = ROOT / 'config/robot_models/mevius2'
        manifest = json.loads((root / 'UPSTREAM.json').read_text())
        for name, entry in manifest['files'].items():
            data = (root / name).read_bytes()
            self.assertTrue(verified(data, entry), name)
            self.assertFalse(verified(bytes([data[0] ^ 1]) + data[1:], entry), name)


if __name__ == '__main__':
    unittest.main()
