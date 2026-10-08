"""Vendor-neutral joint position contract. No device I/O or motor protocol.

Angles are joint-side radians. Timestamps use the receiver's monotonic clock;
remote transports must translate source age, never stamp old data as fresh.
"""
from dataclasses import dataclass, asdict
import math

SCHEMA = 'omindos.joint_position.v1'


def finite_vector(values, count):
    if not isinstance(values, (tuple, list)) or len(values) != count:
        raise ValueError('joint_vector_length')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
        raise ValueError('joint_vector_not_finite')
    return tuple(values)


def joint_order(names, expected):
    if not isinstance(names, (tuple, list)) or any(not isinstance(n, str) or not n for n in names):
        raise ValueError('invalid_joint_names')
    if len(names) != len(expected) or len(set(names)) != len(names) or set(names) != set(expected):
        raise ValueError('joint_names_mismatch')
    return [names.index(n) for n in expected]


@dataclass(frozen=True)
class PositionCommand:
    session_id: str
    sequence: int
    issued_monotonic_s: float
    valid_for_s: float
    joint_names: tuple
    position: tuple
    schema: str = SCHEMA
    unit: str = 'rad'

    def __post_init__(self):
        if not isinstance(self.joint_names,(list,tuple)) or not isinstance(self.position,(list,tuple)):
            raise ValueError('joint_command_vectors')
        object.__setattr__(self,'joint_names',tuple(self.joint_names))
        object.__setattr__(self,'position',tuple(self.position))

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        # Exact fields avoid silently ignoring a misspelled or incompatible field.
        if not isinstance(data, dict) or set(data) != set(cls.__dataclass_fields__):
            raise ValueError('position_command_fields')
        return cls(**{**data, 'joint_names':tuple(data['joint_names']), 'position':tuple(data['position'])})


@dataclass(frozen=True)
class JointFeedback:
    joint_names: tuple
    position: tuple
    velocity: tuple
    faults: tuple
    sampled_monotonic_s: float

    def ordered(self, expected, now, max_age_s=.1):
        order = joint_order(self.joint_names, expected)
        q = finite_vector(self.position, len(expected))
        dq = finite_vector(self.velocity, len(expected))
        faults = finite_vector(self.faults, len(expected))
        finite_vector([now, self.sampled_monotonic_s], 2)
        if not 0 <= now-self.sampled_monotonic_s <= max_age_s:
            raise ValueError('joint_feedback_expired')
        if any(faults):
            raise ValueError('joint_feedback_fault')
        return tuple(q[i] for i in order), tuple(dq[i] for i in order)


class PositionCommandGuard:
    """Validate whole batches before output; deadline/invalid commands latch.

    The receiving adapter must call watchdog independently while holding a
    command. A Python simulation of this rule is not a hardware watchdog.
    """
    def __init__(self, names, limits, max_valid_for_s=.05):
        self.names = tuple(names)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError('invalid_configured_joint_names')
        joint_order(self.names, self.names)
        if len(limits) != len(self.names):
            raise ValueError('joint_limit_count')
        self.limits = tuple(finite_vector(pair, 2) for pair in limits)
        if any(lo >= hi for lo, hi in self.limits):
            raise ValueError('invalid_joint_limits')
        finite_vector([max_valid_for_s], 1)
        if not 0 < max_valid_for_s <= .1:
            raise ValueError('invalid_command_timeout')
        self.max_valid_for_s = max_valid_for_s
        self.active = False
        self.fault = None
        self.session_id = None
        self.used_sessions = set()
        self.sequence = -1
        self.last_issued = -math.inf
        self.deadline = 0.

    def arm(self, session_id, now):
        finite_vector([now], 1)
        if self.fault or self.active:
            raise ValueError('joint_adapter_not_disarmed')
        if not isinstance(session_id, str) or not session_id or session_id in self.used_sessions:
            raise ValueError('new_joint_session_required')
        self.session_id = session_id
        self.used_sessions.add(session_id)
        self.sequence = -1
        self.last_issued = now
        self.deadline = now+self.max_valid_for_s
        self.active = True

    def disarm(self):
        self.active = False

    def _fail(self, reason):
        self.active = False
        self.fault = str(reason)
        raise ValueError(self.fault)

    def watchdog(self, now):
        if self.fault:
            raise ValueError('joint_adapter_fault_latched:'+self.fault)
        if self.active:
            try:
                finite_vector([now], 1)
                if not self.last_issued <= now <= self.deadline:
                    raise ValueError('joint_command_deadline')
            except (TypeError, ValueError) as error:
                self._fail(error)

    def accept(self, command, now):
        if not self.active or self.fault:
            raise ValueError('joint_adapter_inactive')
        try:
            # A new packet cannot revive an already expired session.
            self.watchdog(now)
            if not isinstance(command, PositionCommand) or command.schema != SCHEMA or command.unit != 'rad':
                raise ValueError('joint_command_schema_or_unit')
            if command.session_id != self.session_id:
                raise ValueError('joint_command_session')
            if type(command.sequence) is not int or command.sequence <= self.sequence or command.sequence < 0:
                raise ValueError('joint_command_replayed')
            finite_vector([command.issued_monotonic_s, command.valid_for_s], 2)
            if not 0 < command.valid_for_s <= self.max_valid_for_s:
                raise ValueError('joint_command_validity')
            if command.issued_monotonic_s < self.last_issued or not 0 <= now-command.issued_monotonic_s <= command.valid_for_s:
                raise ValueError('joint_command_expired_or_future')
            order = joint_order(command.joint_names, self.names)
            raw = finite_vector(command.position, len(self.names))
            positions = tuple(raw[i] for i in order)
            if any(not lo <= q <= hi for q, (lo, hi) in zip(positions, self.limits)):
                raise ValueError('joint_target_outside_limit')
            self.sequence = command.sequence
            self.last_issued = command.issued_monotonic_s
            self.deadline = command.issued_monotonic_s+command.valid_for_s
            return positions
        except (TypeError, ValueError, AttributeError) as error:
            self._fail(error)
