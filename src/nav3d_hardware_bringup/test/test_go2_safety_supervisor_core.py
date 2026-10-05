import math
from pathlib import Path
import struct
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from go2_safety_supervisor_core import B_MASK  # noqa: E402
from go2_safety_supervisor_core import L2_MASK  # noqa: E402
from go2_safety_supervisor_core import SafetyConfig  # noqa: E402
from go2_safety_supervisor_core import SafetySupervisorCore  # noqa: E402
from go2_safety_supervisor_core import START_MASK  # noqa: E402
from go2_safety_supervisor_core import decode_remote  # noqa: E402


CONFIG = SafetyConfig()


def remote(buttons=0, axes=(0.0, 0.0, 0.0, 0.0), prefix=(85, 81)):
    data = bytearray(40)
    data[0:2] = bytes(prefix)
    data[2] = buttons & 0xff
    data[3] = (buttons >> 8) & 0xff
    struct.pack_into('<f', data, 4, axes[0])
    struct.pack_into('<f', data, 8, axes[1])
    struct.pack_into('<f', data, 12, axes[2])
    struct.pack_into('<f', data, 20, axes[3])
    return data


def test_remote_decode_matches_accepted_unitree_layout():
    neutral = decode_remote(remote(), CONFIG)
    assert neutral.online and neutral.neutral
    assert not neutral.takeover_input
    start = decode_remote(remote(START_MASK), CONFIG)
    assert start.start_only and not start.takeover_input
    soft_estop = decode_remote(remote(L2_MASK | B_MASK), CONFIG)
    assert soft_estop.soft_estop_input and soft_estop.takeover_input


def test_invalid_and_nonfinite_remote_fail_neutral_check():
    try:
        decode_remote(bytes(20), CONFIG)
        assert False, 'short remote data must be rejected'
    except ValueError:
        pass
    sample = decode_remote(remote(axes=(math.nan, 0, 0, 0)), CONFIG)
    assert not sample.neutral
    assert sample.takeover_input


def test_start_requires_new_process_neutral_baseline_and_fresh_edge():
    core = SafetySupervisorCore(CONFIG)
    core.observe_lowstate(remote(START_MASK), 0.0)
    assert core.evaluate(0.01).status == 'REMOTE_NEUTRAL_BASELINE_REQUIRED'
    core.observe_lowstate(remote(), 0.02)
    assert core.evaluate(0.03).status == 'REMOTE_START_REQUIRED'
    core.observe_lowstate(remote(START_MASK), 0.04)
    decision = core.evaluate(0.05)
    assert decision.remote_motion_permit
    assert decision.status == 'OPERATOR_REQUEST_TIMEOUT'


def test_stage_lock_is_final_enable_term():
    core = SafetySupervisorCore(CONFIG)
    core.observe_lowstate(remote(), 1.0)
    core.observe_lowstate(remote(START_MASK), 1.01)
    core.observe_operator(True, 1.02)
    locked = core.evaluate(1.03, motion_stage_unlocked=False)
    assert locked.status == 'MOTION_STAGE_LOCKED'
    assert not locked.emergency_stop
    assert not locked.control_enabled
    unlocked = core.evaluate(1.03, motion_stage_unlocked=True)
    assert unlocked.status == 'ENABLED'
    assert unlocked.control_enabled


def test_any_other_remote_input_after_permit_latches_takeover():
    core = SafetySupervisorCore(CONFIG)
    core.observe_lowstate(remote(), 2.0)
    core.observe_lowstate(remote(START_MASK), 2.01)
    core.observe_lowstate(remote(), 2.02)
    _, events = core.observe_lowstate(remote(buttons=1), 2.03)
    decision = core.evaluate(2.04)
    assert 'REMOTE_TAKEOVER_LATCHED' in events
    assert decision.status == 'REMOTE_TAKEOVER_LATCHED'
    assert decision.remote_takeover
    assert decision.emergency_stop


def test_takeover_reset_requires_fresh_disarm_and_neutral_hold():
    core = SafetySupervisorCore(CONFIG)
    core.observe_lowstate(remote(), 3.0)
    core.observe_lowstate(remote(START_MASK), 3.01)
    core.observe_lowstate(remote(buttons=1), 3.02)
    core.observe_lowstate(remote(), 3.03)
    core.observe_operator(False, 3.04)
    core.observe_lowstate(remote(), 3.49)
    assert core.evaluate(3.50).remote_takeover
    core.observe_operator(False, 4.02)
    core.observe_lowstate(remote(), 4.03)
    decision = core.evaluate(4.04)
    assert not decision.remote_takeover
    assert decision.status == 'REMOTE_START_REQUIRED'


def test_lowstate_and_operator_timeouts_fail_closed():
    core = SafetySupervisorCore(CONFIG)
    core.observe_lowstate(remote(), 5.0)
    core.observe_lowstate(remote(START_MASK), 5.01)
    core.observe_operator(True, 5.02)
    assert core.evaluate(5.31).status == 'LOWSTATE_TIMEOUT'
    assert core.evaluate(5.31).emergency_stop

    core.observe_lowstate(remote(), 6.0)
    core.observe_lowstate(remote(START_MASK), 6.01)
    core.observe_operator(True, 6.02)
    core.observe_lowstate(remote(), 6.32)
    decision = core.evaluate(6.33)
    assert decision.status == 'OPERATOR_REQUEST_TIMEOUT'
    assert not decision.control_enabled
