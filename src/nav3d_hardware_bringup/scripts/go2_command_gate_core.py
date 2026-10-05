"""Pure fail-closed decision logic for the ROS 2 Go2 H4 dry-run gate."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


@dataclass(frozen=True)
class GateConfig:
    command_timeout_seconds: float = 0.15
    safety_timeout_seconds: float = 0.15
    max_linear_velocity_m_s: float = 0.05
    max_angular_velocity_rad_s: float = 0.30
    zero_epsilon: float = 1.0e-6


@dataclass(frozen=True)
class GateDecision:
    status: str
    input_valid: bool
    motion_authorized: bool
    output: tuple


def _fresh(age: Optional[float], timeout: float) -> bool:
    return age is not None and 0.0 <= age <= timeout


def validate_config(config: GateConfig) -> None:
    values = (
        config.command_timeout_seconds,
        config.safety_timeout_seconds,
        config.max_linear_velocity_m_s,
        config.max_angular_velocity_rad_s,
        config.zero_epsilon,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('gate parameters must be finite and positive')
    if config.command_timeout_seconds > 0.15:
        raise ValueError('command watchdog must not exceed 0.15 seconds')
    if config.safety_timeout_seconds > 0.15:
        raise ValueError('safety watchdog must not exceed 0.15 seconds')
    if config.max_linear_velocity_m_s > 0.05:
        raise ValueError('H4 linear limit must not exceed 0.05 m/s')
    if config.max_angular_velocity_rad_s > 0.30:
        raise ValueError('H4 angular limit must not exceed 0.30 rad/s')


def evaluate_gate(
        command: Sequence[float], command_age: Optional[float],
        enabled: bool, enable_age: Optional[float],
        emergency_stop: bool, estop_age: Optional[float],
        config: GateConfig, motion_stage_unlocked: bool = False,
) -> GateDecision:
    """Evaluate one gate cycle; returned output is fail-closed all-zero."""
    if len(command) != 6:
        raise ValueError('command must contain six Twist components')
    validate_config(config)
    values = tuple(float(value) for value in command)
    safety_fresh = (
        _fresh(enable_age, config.safety_timeout_seconds)
        and _fresh(estop_age, config.safety_timeout_seconds)
    )
    command_fresh = _fresh(command_age, config.command_timeout_seconds)
    finite = all(math.isfinite(value) for value in values)
    unsupported = finite and any(
        abs(value) > config.zero_epsilon for value in values[1:5]
    )
    in_bounds = finite and (
        abs(values[0]) <= config.max_linear_velocity_m_s
        and abs(values[5]) <= config.max_angular_velocity_rad_s
    )

    if not safety_fresh:
        status = 'SAFETY_HEARTBEAT_TIMEOUT'
    elif emergency_stop:
        status = 'EMERGENCY_STOP'
    elif not enabled:
        status = 'DISARMED'
    elif not command_fresh:
        status = 'COMMAND_TIMEOUT'
    elif not finite:
        status = 'NONFINITE_COMMAND_REJECTED'
    elif unsupported:
        status = 'UNSUPPORTED_DOF_REJECTED'
    elif not in_bounds:
        status = 'COMMAND_LIMIT_REJECTED'
    elif not motion_stage_unlocked:
        status = 'MOTION_STAGE_LOCKED'
    else:
        status = 'READY'

    input_valid = bool(
        safety_fresh and not emergency_stop and enabled and command_fresh
        and finite and not unsupported and in_bounds
    )
    motion_authorized = input_valid and motion_stage_unlocked
    # H4 is diagnostic-only. Keeping the pure evaluator output zero also makes
    # an accidental future use fail closed until an explicit SDK boundary is
    # separately designed and reviewed.
    return GateDecision(
        status=status,
        input_valid=input_valid,
        motion_authorized=motion_authorized,
        output=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
