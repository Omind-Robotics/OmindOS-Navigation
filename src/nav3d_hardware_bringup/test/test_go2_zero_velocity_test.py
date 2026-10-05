import ast
import importlib.util
import math
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
SCRIPT_PATH = SCRIPT_DIR / 'go2_zero_velocity_test.py'
sys.path.insert(0, str(SCRIPT_DIR))
SPEC = importlib.util.spec_from_file_location('go2_zero_test', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def stationary_state(**changes):
    state = {
        'remote_online': True,
        'remote_neutral': True,
        'mode': 0,
        'gait_type': 0,
        'velocity': (0.0, 0.0, 0.0),
        'yaw_speed': 0.0,
    }
    state.update(changes)
    return state


def test_stationary_validation_accepts_only_safe_idle_state():
    assert MODULE.stationary_failures(stationary_state()) == []
    assert MODULE.stationary_failures(
        stationary_state(remote_neutral=False))
    assert MODULE.stationary_failures(
        stationary_state(velocity=(0.051, 0.0, 0.0)))
    assert MODULE.stationary_failures(
        stationary_state(yaw_speed=0.051))
    assert MODULE.stationary_failures(
        stationary_state(velocity=(math.nan, 0.0, 0.0)))
    assert MODULE.stationary_failures(stationary_state(mode=1))


def test_runtime_constants_are_fixed_to_zero_stage_limits():
    assert MODULE.ZERO_COMMAND_SECONDS == 0.50
    assert MODULE.ZERO_COMMAND_RATE_HZ == 20.0
    assert MODULE.MAXIMUM_LINEAR_SPEED == 0.05
    assert MODULE.MAXIMUM_YAW_SPEED == 0.05
    assert MODULE.CONFIRM_TOKEN == 'ZERO_MOVE_AND_STOP_ONCE'


def test_source_has_only_literal_zero_move_and_one_stop_call_site():
    tree = ast.parse(SCRIPT_PATH.read_text(encoding='utf-8'))
    move_calls = []
    stop_calls = []
    switch_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not isinstance(function, ast.Attribute):
            continue
        if function.attr == 'Move':
            move_calls.append(node)
        elif function.attr == 'StopMove':
            stop_calls.append(node)
        elif function.attr == 'SwitchJoystick':
            switch_calls.append(node)
    assert len(move_calls) == 1
    assert len(move_calls[0].args) == 3
    assert all(
        isinstance(argument, ast.Constant) and argument.value == 0.0
        for argument in move_calls[0].args)
    assert len(stop_calls) == 1
    assert switch_calls == []
