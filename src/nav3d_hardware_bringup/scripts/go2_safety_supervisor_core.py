"""Pure remote-priority safety state machine for the ROS 2 Go2 bridge."""

from dataclasses import dataclass
import math
import struct
from typing import Optional, Sequence, Tuple


START_MASK = 1 << 2
L2_MASK = 1 << 5
B_MASK = 1 << 9


@dataclass(frozen=True)
class SafetyConfig:
    lowstate_timeout_seconds: float = 0.25
    operator_timeout_seconds: float = 0.30
    axis_takeover_threshold: float = 0.15
    takeover_reset_hold_seconds: float = 1.0
    remote_online_prefix: Tuple[int, int] = (85, 81)


@dataclass(frozen=True)
class RemoteSample:
    button_bits: int
    axes: Tuple[float, float, float, float]
    online: bool
    neutral: bool
    start_only: bool
    takeover_input: bool
    soft_estop_input: bool


@dataclass(frozen=True)
class SafetyDecision:
    status: str
    lowstate_ok: bool
    remote_online: bool
    remote_baseline_ready: bool
    remote_motion_permit: bool
    remote_takeover: bool
    remote_soft_estop_input: bool
    operator_fresh: bool
    operator_requested: bool
    emergency_stop: bool
    control_enabled: bool


def validate_config(config: SafetyConfig) -> None:
    values = (
        config.lowstate_timeout_seconds,
        config.operator_timeout_seconds,
        config.axis_takeover_threshold,
        config.takeover_reset_hold_seconds,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('safety parameters must be finite and positive')
    if config.lowstate_timeout_seconds > 0.25:
        raise ValueError('LowState timeout must not exceed 0.25 seconds')
    if config.operator_timeout_seconds > 0.30:
        raise ValueError('operator timeout must not exceed 0.30 seconds')
    if config.axis_takeover_threshold > 0.15:
        raise ValueError('axis takeover threshold must not exceed 0.15')
    if len(config.remote_online_prefix) != 2 or any(
            value < 0 or value > 255
            for value in config.remote_online_prefix):
        raise ValueError('remote online prefix must contain two bytes')


def decode_remote(raw: Sequence[int], config: SafetyConfig) -> RemoteSample:
    validate_config(config)
    data = bytes(raw)
    if len(data) < 24:
        raise ValueError('wireless_remote must contain at least 24 bytes')
    button_bits = data[2] | (data[3] << 8)
    axes = (
        struct.unpack_from('<f', data, 4)[0],
        struct.unpack_from('<f', data, 8)[0],
        struct.unpack_from('<f', data, 12)[0],
        struct.unpack_from('<f', data, 20)[0],
    )
    finite_axes = all(math.isfinite(value) for value in axes)
    axes_neutral = (
        finite_axes
        and max(abs(value) for value in axes)
        <= config.axis_takeover_threshold
    )
    online = data[:2] == bytes(config.remote_online_prefix)
    start_only = axes_neutral and button_bits == START_MASK
    neutral = axes_neutral and button_bits == 0
    takeover_input = online and not (neutral or start_only)
    soft_estop_input = bool(
        online and button_bits & L2_MASK and button_bits & B_MASK)
    return RemoteSample(
        button_bits=button_bits,
        axes=axes,
        online=online,
        neutral=neutral,
        start_only=start_only,
        takeover_input=takeover_input,
        soft_estop_input=soft_estop_input,
    )


class SafetySupervisorCore:
    def __init__(self, config: SafetyConfig):
        validate_config(config)
        self.config = config
        self.last_lowstate_time: Optional[float] = None
        self.last_remote_online = False
        self.last_remote_neutral = False
        self.last_remote_soft_estop = False
        self.remote_baseline_ready = False
        self.remote_motion_permit = False
        self.last_start_pressed = False
        self.takeover_latched = False
        self.neutral_since: Optional[float] = None
        self.last_operator_request = False
        self.last_operator_time: Optional[float] = None

    def observe_operator(self, requested: bool, now: float) -> None:
        self.last_operator_request = bool(requested)
        self.last_operator_time = float(now)

    def observe_invalid_lowstate(self, now: float) -> None:
        self.last_lowstate_time = float(now)
        self.last_remote_online = False
        self.last_remote_neutral = False
        self.last_remote_soft_estop = False
        self.remote_baseline_ready = False
        self.remote_motion_permit = False
        self.last_start_pressed = False
        self.neutral_since = None

    def observe_lowstate(self, raw: Sequence[int], now: float):
        sample = decode_remote(raw, self.config)
        events = []
        self.last_lowstate_time = float(now)
        self.last_remote_online = sample.online
        self.last_remote_neutral = sample.neutral
        self.last_remote_soft_estop = sample.soft_estop_input

        if not sample.online:
            self.remote_baseline_ready = False
            self.remote_motion_permit = False
            self.last_start_pressed = False
        elif not self.remote_baseline_ready:
            if sample.neutral:
                self.remote_baseline_ready = True
                self.last_start_pressed = False
                events.append('REMOTE_BASELINE_READY')
        elif (sample.start_only and not self.last_start_pressed
              and not self.takeover_latched):
            self.remote_motion_permit = True
            events.append('REMOTE_START_PERMIT_ARMED')

        if sample.takeover_input and self.remote_motion_permit:
            self.remote_motion_permit = False
            self.takeover_latched = True
            self.neutral_since = None
            events.append('REMOTE_TAKEOVER_LATCHED')
        elif sample.neutral:
            if self.neutral_since is None:
                self.neutral_since = float(now)
        else:
            self.neutral_since = None

        if self.remote_baseline_ready:
            self.last_start_pressed = bool(sample.button_bits & START_MASK)
        return sample, tuple(events)

    @staticmethod
    def _fresh(now: float, stamp: Optional[float], timeout: float) -> bool:
        return (
            stamp is not None
            and 0.0 <= now - stamp <= timeout
        )

    def evaluate(self, now: float,
                 motion_stage_unlocked: bool = False) -> SafetyDecision:
        now = float(now)
        lowstate_ok = self._fresh(
            now, self.last_lowstate_time,
            self.config.lowstate_timeout_seconds)
        operator_fresh = self._fresh(
            now, self.last_operator_time,
            self.config.operator_timeout_seconds)
        operator_requested = (
            operator_fresh and self.last_operator_request)
        remote_online = lowstate_ok and self.last_remote_online
        remote_soft_estop = (
            lowstate_ok and self.last_remote_soft_estop)

        if not lowstate_ok or not remote_online:
            self.remote_baseline_ready = False
            self.remote_motion_permit = False

        can_reset_takeover = (
            self.takeover_latched
            and operator_fresh
            and not self.last_operator_request
            and remote_online
            and self.last_remote_neutral
            and self.neutral_since is not None
            and now - self.neutral_since
            >= self.config.takeover_reset_hold_seconds
        )
        if can_reset_takeover:
            self.takeover_latched = False

        takeover = self.takeover_latched
        remote_motion_permit = self.remote_motion_permit
        emergency_stop = bool(
            not lowstate_ok
            or not remote_online
            or not remote_motion_permit
            or takeover
        )
        control_enabled = bool(
            motion_stage_unlocked
            and lowstate_ok
            and remote_online
            and remote_motion_permit
            and operator_requested
            and not emergency_stop
        )

        if not lowstate_ok:
            status = 'LOWSTATE_TIMEOUT'
        elif not remote_online:
            status = 'REMOTE_OFFLINE'
        elif not self.remote_baseline_ready:
            status = 'REMOTE_NEUTRAL_BASELINE_REQUIRED'
        elif takeover:
            status = 'REMOTE_TAKEOVER_LATCHED'
        elif not remote_motion_permit:
            status = 'REMOTE_START_REQUIRED'
        elif not operator_fresh:
            status = 'OPERATOR_REQUEST_TIMEOUT'
        elif not operator_requested:
            status = 'OPERATOR_DISARMED'
        elif not motion_stage_unlocked:
            status = 'MOTION_STAGE_LOCKED'
        else:
            status = 'ENABLED'

        return SafetyDecision(
            status=status,
            lowstate_ok=lowstate_ok,
            remote_online=remote_online,
            remote_baseline_ready=self.remote_baseline_ready,
            remote_motion_permit=remote_motion_permit,
            remote_takeover=takeover,
            remote_soft_estop_input=remote_soft_estop,
            operator_fresh=operator_fresh,
            operator_requested=operator_requested,
            emergency_stop=emergency_stop,
            control_enabled=control_enabled,
        )
