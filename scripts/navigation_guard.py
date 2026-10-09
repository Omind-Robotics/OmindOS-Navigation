"""ROS-independent command boundary for platform previews, not a hardware driver."""
from dataclasses import dataclass
import json
import math
from pathlib import Path


ZERO = (0.0,) * 6
TIMEOUTS = {'command': 0.25, 'pose': 0.75, 'odom': 0.5, 'plan': 2.0}


@dataclass(frozen=True)
class PlatformProfile:
    name: str
    kind: str
    max_vx: float
    max_vy: float
    max_wz: float


def load_profile(name, path=None):
    source = Path(path) if path else Path(__file__).with_name('platform_profiles.json')
    document = json.loads(source.read_text(encoding='utf-8'))
    if document.get('schema_version') != 1:
        raise ValueError('Unsupported platform profile schema')
    if name not in document.get('profiles', {}):
        raise ValueError(f'Unknown platform profile: {name}')
    data = document['profiles'][name]
    if data.get('kind') not in ('differential', 'omnidirectional', 'quadruped_body'):
        raise ValueError('Unknown platform kind')
    limits = [data.get(key) for key in ('max_vx', 'max_vy', 'max_wz')]
    if not all(type(v) in (int, float) and math.isfinite(v) for v in limits):
        raise ValueError('Velocity limits must be finite numbers')
    vx, vy, wz = limits
    if vx <= 0 or vy < 0 or wz <= 0:
        raise ValueError('Invalid velocity limits')
    if data['kind'] == 'differential' and vy != 0:
        raise ValueError('Differential platforms cannot accept lateral velocity')
    return PlatformProfile(name, data['kind'], vx, vy, wz)


def stamp_is_fresh(stamp_seconds, now_seconds, timeout):
    return (math.isfinite(stamp_seconds) and math.isfinite(now_seconds)
            and stamp_seconds > 0 and 0 <= now_seconds - stamp_seconds <= timeout)


class PlatformGuard:
    def __init__(self, profile, enabled=False):
        self.profile = profile
        self.enabled = enabled
        self.estop_latched = False
        self.command = ZERO
        self.receipts = {}
        self.valid = {}

    def observe(self, source, valid, now):
        if source not in TIMEOUTS:
            raise ValueError('Unknown observation source')
        self.receipts[source] = now
        self.valid[source] = bool(valid) and math.isfinite(now)

    def receive_command(self, values, now):
        try:
            values = tuple(float(v) for v in values)
            valid = len(values) == 6 and all(math.isfinite(v) for v in values)
        except (ValueError, TypeError, OverflowError):
            values, valid = ZERO, False
        if valid:
            valid = all(abs(values[i]) <= 1e-9 for i in (2, 3, 4))
            if self.profile.max_vy == 0:
                valid = valid and abs(values[1]) <= 1e-9
        self.command = values if valid else ZERO
        self.observe('command', valid, now)

    def emergency_stop(self, active):
        self.estop_latched = self.estop_latched or bool(active)

    def evaluate(self, now):
        if self.estop_latched:
            return 'EMERGENCY_STOP', ZERO
        if not self.enabled:
            return 'DISARMED', ZERO
        if not math.isfinite(now):
            return 'CLOCK_INVALID', ZERO
        for source, timeout in TIMEOUTS.items():
            if source not in self.receipts:
                return source.upper() + '_MISSING', ZERO
            if not self.valid[source]:
                return source.upper() + '_INVALID', ZERO
            if not 0 <= now - self.receipts[source] <= timeout:
                return source.upper() + '_TIMEOUT', ZERO
        vx, vy, _, _, _, wz = self.command
        # Scale the whole planar command together to preserve its curvature.
        scale = max(1.0, abs(vx) / self.profile.max_vx,
                    abs(wz) / self.profile.max_wz)
        if self.profile.max_vy:
            scale = max(scale, abs(vy) / self.profile.max_vy)
        else:
            vy = 0.0
        output = (vx / scale, vy / scale, 0.0, 0.0, 0.0, wz / scale)
        return ('LIMITED' if scale > 1 else 'READY'), output
