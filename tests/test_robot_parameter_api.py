import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("robot_api", ROOT / "scripts/robot_parameter_api.py")
API = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(API)


def fixture():
    # Synthetic two-body rotary mechanism; not a MEVIUS2 hardware configuration.
    body = '<inertial><mass value="1"/><inertia ixx=".01" iyy=".02" izz=".02" ixy="0" ixz="0" iyz="0"/></inertial>'
    xml = ('<robot name="fixture"><link name="base">' + body + '</link><link name="arm">' + body +
           '</link><joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>'
           '<axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint></robot>')
    return {"robot_id": "fixture-a", "kinematics": {"urdf_xml": xml}, "dynamics": {"joints": {
        "hinge": {"motor_model": "synthetic-test-motor", "reduction_ratio": 5.0, "direction": 1,
                  "zero_offset_rad": 0.0, "torque_limit_nm": 8.0, "velocity_limit_rad_s": 1.0}}}}


class ParameterAPITests(unittest.TestCase):
    def test_versions_preserve_old_parameters_and_idempotence(self):
        with tempfile.TemporaryDirectory() as folder:
            original = fixture()
            one = API.register_robot_parameters(original, folder)
            self.assertFalse(one["runtime_applied"])
            self.assertEqual(one, API.register_robot_parameters(original, folder))
            changed = copy.deepcopy(original)
            changed["dynamics"]["joints"]["hinge"]["zero_offset_rad"] = .05
            two = API.register_robot_parameters(changed, folder)
            self.assertNotEqual(one["profile_id"], two["profile_id"])
            self.assertEqual(API.load_robot_profile(one["profile_id"], folder), original)
            self.assertEqual(API.load_robot_profile(two["profile_id"], folder), changed)

    def test_invalid_physics_and_joint_mapping_rejected(self):
        cases = []
        for old, new in [('mass value="1"', 'mass value="-1"'), ('ixx=".01"', 'ixx="nan"'),
                         ('ixy="0"', 'ixy="1"'), ('lower="-1"', 'lower="2"'),
                         ('child link="arm"', 'child link="missing"'),
                         ('axis xyz="0 1 0"', 'axis xyz="0 0 0"')]:
            payload = fixture()
            payload["kinematics"]["urdf_xml"] = payload["kinematics"]["urdf_xml"].replace(old, new)
            cases.append(payload)
        bad = fixture()
        bad["dynamics"]["joints"]["wrong-name"] = bad["dynamics"]["joints"].pop("hinge")
        cases.append(bad)
        for payload in cases:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                API.validate_parameters(payload)

    def test_invalid_drive_values_and_excess_limits_rejected(self):
        for field, value in [("reduction_ratio", 0), ("reduction_ratio", True),
                             ("direction", 0), ("direction", True),
                             ("zero_offset_rad", float("nan")), ("torque_limit_nm", 11),
                             ("velocity_limit_rad_s", 3), ("motor_model", "")]:
            payload = fixture()
            payload["dynamics"]["joints"]["hinge"][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                API.validate_parameters(payload)

    def test_dtd_and_unsafe_identifiers_rejected(self):
        payload = fixture()
        payload["kinematics"]["urdf_xml"] = '<!DOCTYPE robot [<!ENTITY x "x">]>' + payload["kinematics"]["urdf_xml"]
        with self.assertRaises(ValueError):
            API.validate_parameters(payload)
        payload = fixture()
        payload["robot_id"] = "../../robot"
        with self.assertRaises(ValueError):
            API.validate_parameters(payload)

    def test_changed_stored_content_is_detected(self):
        with tempfile.TemporaryDirectory() as folder:
            result = API.register_robot_parameters(fixture(), folder)
            changed = fixture()
            changed["robot_id"] = "tampered"
            (Path(folder) / (result["profile_id"] + ".json")).write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, "checksum"):
                API.load_robot_profile(result["profile_id"], folder)

    def test_http_register_read_and_bad_request(self):
        with tempfile.TemporaryDirectory() as folder:
            server = API.ThreadingHTTPServer(("127.0.0.1", 0), API.make_handler(folder))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            base = f"http://127.0.0.1:{server.server_port}/v1/robot-profiles"
            def post(payload):
                return urllib.request.urlopen(urllib.request.Request(base, data=json.dumps(payload).encode(),
                    headers={"Content-Type": "application/json"}), timeout=5)
            try:
                with post(fixture()) as response:
                    self.assertEqual(response.status, 201)
                    created = json.load(response)
                with urllib.request.urlopen(base + "/" + created["profile_id"], timeout=5) as response:
                    self.assertEqual(json.load(response), fixture())
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    post({"robot_id": "missing-parameters"})
                self.assertEqual(caught.exception.code, 400)
                self.assertIn("error", json.load(caught.exception))
            finally:
                server.shutdown()
                thread.join(timeout=5)
                server.server_close()


if __name__ == "__main__":
    unittest.main()
