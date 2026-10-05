import math
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from go2_command_gate_core import GateConfig, evaluate_gate  # noqa: E402


CONFIG = GateConfig()
ZERO = (0.0,) * 6


def decide(command=ZERO, command_age=0.01, enabled=True, enable_age=0.01,
           emergency_stop=False, estop_age=0.01, unlocked=False):
    return evaluate_gate(
        command, command_age, enabled, enable_age,
        emergency_stop, estop_age, CONFIG, unlocked)


def test_defaults_fail_closed_without_safety_heartbeats():
    decision = decide(enable_age=None, estop_age=None)
    assert decision.status == 'SAFETY_HEARTBEAT_TIMEOUT'
    assert not decision.input_valid
    assert not decision.motion_authorized
    assert decision.output == ZERO


def test_emergency_stop_and_disarm_have_priority():
    assert decide(emergency_stop=True).status == 'EMERGENCY_STOP'
    assert decide(enabled=False).status == 'DISARMED'


def test_rejects_stale_nonfinite_unsupported_and_out_of_bounds_commands():
    assert decide(command_age=0.151).status == 'COMMAND_TIMEOUT'
    assert decide(command=(math.nan, 0, 0, 0, 0, 0)).status == (
        'NONFINITE_COMMAND_REJECTED')
    assert decide(command=(0, 0.001, 0, 0, 0, 0)).status == (
        'UNSUPPORTED_DOF_REJECTED')
    assert decide(command=(0.051, 0, 0, 0, 0, 0)).status == (
        'COMMAND_LIMIT_REJECTED')
    assert decide(command=(0, 0, 0, 0, 0, 0.301)).status == (
        'COMMAND_LIMIT_REJECTED')


def test_valid_input_remains_motion_locked_in_h4():
    decision = decide(command=(0.05, 0, 0, 0, 0, -0.30))
    assert decision.status == 'MOTION_STAGE_LOCKED'
    assert decision.input_valid
    assert not decision.motion_authorized
    assert decision.output == ZERO


def test_pure_future_unlock_path_still_returns_zero_output():
    decision = decide(command=(-0.05, 0, 0, 0, 0, 0.30), unlocked=True)
    assert decision.status == 'READY'
    assert decision.input_valid
    assert decision.motion_authorized
    assert decision.output == ZERO
