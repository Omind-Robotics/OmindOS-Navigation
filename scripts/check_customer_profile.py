#!/usr/bin/env python3
"""Check unified customer configuration; never connect to or command hardware."""
import argparse
import json
import math
from pathlib import Path


PLATFORMS = {"quadruped", "wheeled_diff", "wheeled_omni"}
ROOT_KEYS = {"schema_version", "platform", "runtime_architecture", "hardware"}
HARDWARE_KEYS = {
    "model", "computer_architecture", "control_protocol", "controller_sdk_version",
    "adapter", "odometry_source", "stop_semantics", "watchdog_timeout_ms",
    "physical_estop_available", "locomotion_controller_ready",
}
TEXT_FIELDS = {
    "model", "control_protocol", "controller_sdk_version", "adapter",
    "odometry_source", "stop_semantics",
}


def finite_positive(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value) and value > 0
    except OverflowError:
        return False


def check_profile(document, require_hardware=False):
    errors, missing = [], []
    if not isinstance(document, dict):
        return {"status": "INVALID", "errors": ["profile must be an object"],
                "missing": [], "hardware_validated": False, "motion_authorized": False}
    if set(document) != ROOT_KEYS:
        errors.append("profile keys must be exactly: " + ", ".join(sorted(ROOT_KEYS)))
    if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        errors.append("schema_version must be integer 1")
    platform = document.get("platform")
    if not isinstance(platform, str) or platform not in PLATFORMS:
        errors.append("unknown platform")
    # The existing customer runtime has only been built for Linux amd64.
    if document.get("runtime_architecture") != "linux/amd64":
        errors.append("only linux/amd64 runtime is currently available")
    hardware = document.get("hardware")
    if not isinstance(hardware, dict):
        errors.append("hardware must be an object")
        hardware = {}
    if set(hardware) != HARDWARE_KEYS:
        errors.append("hardware keys must be exactly: " + ", ".join(sorted(HARDWARE_KEYS)))
    for field in sorted(TEXT_FIELDS):
        value = hardware.get(field)
        if value is None or value == "":
            missing.append("hardware." + field)
        elif not isinstance(value, str) or not value.strip():
            errors.append("hardware." + field + " must be nonempty text or null")
    architecture = hardware.get("computer_architecture")
    if architecture is None:
        missing.append("hardware.computer_architecture")
    elif architecture != "linux/amd64":
        errors.append("computer architecture does not match the available linux/amd64 runtime")
    timeout = hardware.get("watchdog_timeout_ms")
    if timeout is None:
        missing.append("hardware.watchdog_timeout_ms")
    elif not finite_positive(timeout):
        errors.append("hardware.watchdog_timeout_ms must be a finite positive number")
    for field in ("physical_estop_available", "locomotion_controller_ready"):
        value = hardware.get(field)
        if value is None or value is False:
            missing.append("hardware." + field)
        elif type(value) is not bool:
            errors.append("hardware." + field + " must be boolean or null")
    status = "INVALID" if errors else (
        "HARDWARE_DETAILS_REQUIRED" if require_hardware and missing else "CONFIGURATION_VALID")
    return {
        "status": status, "platform": platform,
        "hardware_configuration_complete": not errors and not missing,
        "errors": errors, "missing": missing,
        # A complete declaration is not hardware acceptance and never enables motion.
        "hardware_validated": False, "motion_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("--require-hardware", action="store_true",
                        help="fail until required hardware details are supplied")
    args = parser.parse_args()
    try:
        document = json.loads(args.profile.read_text(encoding="utf-8"))
        report = check_profile(document, args.require_hardware)
    except (OSError, ValueError) as error:
        report = {"status": "INVALID", "errors": [str(error)],
                  "hardware_validated": False, "motion_authorized": False}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "CONFIGURATION_VALID" else 2


if __name__ == "__main__":
    raise SystemExit(main())
