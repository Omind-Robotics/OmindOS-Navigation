import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'lio_base_odom_adapter.py'
SPEC = importlib.util.spec_from_file_location('lio_base_odom_adapter', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_identity_pose_is_unchanged():
    position, orientation = MODULE.compose_base_pose(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert position == pytest.approx((1.0, 2.0, 3.0))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_base_to_imu_round_trip_returns_identity_base_pose():
    calibration = Path(__file__).parents[1] / 'config' / (
        'go2_mid360_extrinsics.yaml')
    position_bi, orientation_bi = MODULE._load_base_to_imu(calibration)
    position, orientation = MODULE.compose_base_pose(
        position_bi, orientation_bi, position_bi, orientation_bi)
    assert position == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-9)
    assert orientation == pytest.approx(
        (0.0, 0.0, 0.0, 1.0), abs=1.0e-9)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError, match='zero quaternion'):
        MODULE.quat_normalize((0.0, 0.0, 0.0, 0.0))
