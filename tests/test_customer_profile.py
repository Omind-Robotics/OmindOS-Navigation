import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("profile_check", ROOT / "scripts/check_customer_profile.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CustomerProfileTests(unittest.TestCase):
    def setUp(self):
        self.template = json.loads((ROOT / "config/customer/quadruped.example.json").read_text())

    def test_all_three_templates_valid_but_not_hardware_ready(self):
        for filename in (ROOT / "config/customer").glob("*.example.json"):
            profile = json.loads(filename.read_text())
            report = MODULE.check_profile(profile)
            self.assertEqual(report["status"], "CONFIGURATION_VALID")
            self.assertFalse(report["hardware_configuration_complete"])
            self.assertEqual(MODULE.check_profile(profile, True)["status"], "HARDWARE_DETAILS_REQUIRED")

    def test_complete_declaration_never_claims_hardware_acceptance(self):
        profile = copy.deepcopy(self.template)
        hardware = profile["hardware"]
        for field in MODULE.TEXT_FIELDS:
            hardware[field] = "synthetic test fixture"
        hardware.update(computer_architecture="linux/amd64", watchdog_timeout_ms=200,
                        physical_estop_available=True, locomotion_controller_ready=True)
        report = MODULE.check_profile(profile, True)
        self.assertTrue(report["hardware_configuration_complete"])
        self.assertFalse(report["hardware_validated"])
        self.assertFalse(report["motion_authorized"])

    def test_arm_computer_does_not_silently_use_amd64_package(self):
        self.template["hardware"]["computer_architecture"] = "linux/arm64"
        self.assertEqual(MODULE.check_profile(self.template)["status"], "INVALID")

    def test_invalid_timeout_rejected(self):
        for value in (True, 0, -1, float("nan"), float("inf"), "200", 10**1000):
            self.template["hardware"]["watchdog_timeout_ms"] = value
            self.assertEqual(MODULE.check_profile(self.template)["status"], "INVALID")

    def test_bad_types_and_unknown_fields_rejected(self):
        for changes in ({"schema_version": True}, {"platform": []}, {"hardware": []},
                        {"allow_motion": True}, {"platform": "tracked"}):
            profile = copy.deepcopy(self.template)
            profile.update(changes)
            self.assertEqual(MODULE.check_profile(profile)["status"], "INVALID")

    def test_false_controller_or_estop_not_complete(self):
        self.template["hardware"]["physical_estop_available"] = False
        self.template["hardware"]["locomotion_controller_ready"] = False
        report = MODULE.check_profile(self.template, True)
        self.assertIn("hardware.physical_estop_available", report["missing"])
        self.assertIn("hardware.locomotion_controller_ready", report["missing"])

    def test_cli_fails_for_unconfigured_hardware(self):
        result = subprocess.run([sys.executable, str(ROOT / "scripts/check_customer_profile.py"),
                                 str(ROOT / "config/customer/quadruped.example.json"),
                                 "--require-hardware"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(json.loads(result.stdout)["status"], "HARDWARE_DETAILS_REQUIRED")


if __name__ == "__main__":
    unittest.main()
