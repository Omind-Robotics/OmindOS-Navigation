from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from go2_base_controller_ros2 import _clamp  # noqa: E402
from go2_base_controller_ros2 import _shape_nonzero_command  # noqa: E402


def test_clamp_preserves_values_inside_limit():
    assert _clamp(0.2, 0.4) == 0.2
    assert _clamp(-0.2, 0.4) == -0.2
    assert _clamp(0.0, 0.0) == 0.0


def test_clamp_limits_positive_and_negative_values():
    assert _clamp(0.9, 0.4) == 0.4
    assert _clamp(-0.9, 0.4) == -0.4
    assert _clamp(0.1, 0.0) == 0.0


def test_shape_nonzero_command_keeps_stop_as_zero():
    assert _shape_nonzero_command(0.0, 0.4, 0.4) == 0.0
    assert _shape_nonzero_command(0.00001, 0.4, 0.4) == 0.0
    assert _shape_nonzero_command(-0.00001, 0.4, 0.4) == 0.0


def test_shape_nonzero_command_uses_requested_effective_speed():
    assert _shape_nonzero_command(0.1, 0.4, 0.4) == 0.4
    assert _shape_nonzero_command(-0.1, 0.4, 0.4) == -0.4
    assert _shape_nonzero_command(0.9, 0.4, 0.4) == 0.4
    assert _shape_nonzero_command(-0.9, 0.4, 0.4) == -0.4
