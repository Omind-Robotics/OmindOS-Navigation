"""MEVIUS2 policy lifecycle; sensor-driven control, no hardware I/O.

Observation layout: haraduka/mevius2, Copyright (c) 2025 Kento Kawaharazuka,
MIT (LICENSES/MEVIUS2-MIT.txt). Model/weights must come from the pinned checkout.
"""
import hashlib
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

COMMIT = '4f09680bb575574377b903bbc971bdb2695d507b'
POLICY_SHA256 = '392b4d3f30e856caedcb0ec25f125201e97b28795bc9342ee2fc1d8125656510'
NAMES = [f'{leg}_{joint}_joint' for leg in ('BL', 'BR', 'FL', 'FR')
         for joint in ('collar', 'hip', 'knee')]
DEFAULT = [0., .7, -1.2] * 4
SYMMETRY = [1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1]
LIMITS = [(-.7854, .7854), (-1.0472, 2.618), (-2.8508, -.7812)] * 4


def simulation_control_profile(path):
    raw=Path(path).read_bytes(); data=json.loads(raw)
    if data.get('scope')!='simulation_only' or data.get('reference_commit')!=COMMIT or data.get('policy_sha256')!=POLICY_SHA256:
        raise ValueError('simulation control profile identity mismatch')
    kp,kd=vector(data['kp'],12),vector(data['kd'],12)
    if any(not 0<x<=500 for x in kp) or any(not 0<x<=20 for x in kd):
        raise ValueError('invalid simulation PD gains')
    return {'kp':kp,'kd':kd,'sha256':hashlib.sha256(raw).hexdigest()}


def vector(values, length):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f'expected {length} numeric values')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('non-finite or non-numeric value')
    return list(values)


@dataclass
class Sensors:
    q: list
    dq: list
    quaternion_xyzw: list
    gyro_body: list
    motor_time: float
    imu_time: float

    def validate(self, now):
        for timestamp in (self.motor_time, self.imu_time):
            if not math.isfinite(timestamp) or not 0 <= now-timestamp <= .1:
                raise ValueError('stale_or_future_sensor')
        vector(self.q, 12); vector(self.dq, 12)
        vector(self.quaternion_xyzw, 4); vector(self.gyro_body, 3)
        if abs(sum(x*x for x in self.quaternion_xyzw)-1) > .02:
            raise ValueError('invalid_imu_quaternion')
        # Small tolerance permits the reference keyframe's soft joint limits.
        if any(not low-.08 <= q <= high+.08 for q, (low, high) in zip(self.q, LIMITS)):
            raise ValueError('measured_joint_outside_limit')


def observation(s, command):
    x, y, z, w = s.quaternion_xyzw
    gravity = [2*(w*y-x*z), -2*(y*z+w*x), 2*(x*x+y*y)-1]
    return ([v*.25 for v in s.gyro_body] + gravity +
            [command[0]*2, command[1]*2, command[2]*.25] +
            [(q-d)*sign for q, d, sign in zip(s.q, DEFAULT, SYMMETRY)] +
            [dq*.05*sign for dq, sign in zip(s.dq, SYMMETRY)] +
            [float(sum(v*v for v in command) < .03**2)])


class PinnedPolicy:
    def __init__(self, source):
        self.source = Path(source).resolve()
        head = subprocess.check_output(['git', '-C', str(self.source), 'rev-parse', 'HEAD'], text=True).strip()
        if head != COMMIT:
            raise ValueError('reference commit mismatch')
        subprocess.run(['git', '-C', str(self.source), 'diff', '--exit-code', 'HEAD', '--'], check=True,
                       stdout=subprocess.DEVNULL)
        path = self.source/'models/policy.pt'
        if hashlib.sha256(path.read_bytes()).hexdigest() != POLICY_SHA256:
            raise ValueError('policy hash mismatch')
        import torch
        self.torch = torch
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(path), map_location='cpu').eval()

    def __call__(self, obs):
        torch = self.torch
        with torch.inference_mode():
            output = self.policy(torch.tensor([obs], dtype=torch.float32).clamp(-100, 100))
        if tuple(output.shape) != (1, 12):
            raise ValueError('policy output shape mismatch')
        return vector(output[0].tolist(), 12)


class Controller:
    """Call tick at 200 Hz. Policy runs at 50 Hz. Stop preserves active balance.

    Fault/disarm removes torque; this is NOT a controlled landing maneuver.
    Faults never auto-resume. Recreate the controller after investigating a fault.
    """
    def __init__(self, policy, effort_limits=None, kp=None, kd=None):
        self.policy = policy
        self.effort = vector(effort_limits or [60.]*12, 12)
        if any(not 0 < x <= 120 for x in self.effort):
            raise ValueError('invalid joint effort limit')
        self.kp=vector([50.]*12 if kp is None else kp,12)
        self.kd=vector([2.]*12 if kd is None else kd,12)
        if any(not 0 < x <= 500 for x in self.kp) or any(not 0 < x <= 20 for x in self.kd):
            raise ValueError('invalid host PD gains')
        self.state, self.reason = 'DISARMED', 'startup'
        self.target = DEFAULT.copy()
        self.command = [0., 0., 0.]
        self.last_command = -math.inf
        self.last_tick = self.last_policy = None
        self.started = 0.
        self.start_q = []
        self.inferences = 0

    def fault(self, reason):
        self.state, self.reason = 'FAULT', str(reason)
        self.command = [0., 0., 0.]

    def action(self, operation, sensors, now):
        if operation == 'estop':
            self.fault('operator_estop'); return
        if operation == 'disarm':
            if self.state != 'FAULT':
                self.state, self.reason = 'DISARMED', 'operator_disarm'
            self.command = [0., 0., 0.]; return
        if self.state == 'FAULT':
            raise ValueError('fault_latched_restart_required')
        sensors.validate(now)
        if operation == 'arm' and self.state == 'DISARMED':
            self.state, self.reason = 'STANDBY', 'explicit_arm'
            self.target = sensors.q.copy()
            self.last_tick, self.last_policy = now, None
        elif operation == 'stand' and self.state == 'STANDBY':
            self.state, self.reason = 'STANDUP', 'standup_3_seconds'
            self.started, self.start_q = now, sensors.q.copy()
        elif operation == 'walk' and self.state == 'HOLD':
            self.state, self.reason = 'LOCOMOTION', 'explicit_walk'
            self.command, self.last_command = [0., 0., 0.], now
        elif operation == 'stop' and self.state in ('HOLD', 'LOCOMOTION'):
            self.state, self.reason = 'HOLD', 'operator_zero_velocity_hold'
            self.command = [0., 0., 0.]
        else:
            raise ValueError('action_not_allowed_in_current_state')

    def velocity(self, command, now):
        command = vector(command, 3)
        if any(abs(v) > limit for v, limit in zip(command, [.3, .2, .5])):
            raise ValueError('command_outside_preview_envelope')
        if self.state != 'LOCOMOTION':
            raise ValueError('explicit_walk_required')
        if not math.isfinite(now) or now < self.last_command:
            raise ValueError('invalid_command_time')
        self.command, self.last_command = command, now

    def tick_targets(self, sensors, now):
        """Advance the policy/state machine and return joint angles, or None.

        None means disabled/faulted, never a request to drive every joint to 0.
        No torque or actuator protocol is calculated in this path.
        """
        if self.state in ('DISARMED', 'FAULT'):
            return None
        try:
            if not math.isfinite(now) or self.last_tick is None or not 0 <= now-self.last_tick <= .1:
                raise ValueError('control_deadline_missed')
            sensors.validate(now)
            x, y, _, _ = sensors.quaternion_xyzw
            if 1-2*(x*x+y*y) < .6:
                raise ValueError('excessive_tilt')
            self.last_tick = now
            if self.state == 'STANDUP':
                ratio = min(1., (now-self.started)/3.)
                self.target = [a+ratio*(b-a) for a,b in zip(self.start_q, DEFAULT)]
                if ratio == 1:
                    self.state, self.reason = 'HOLD', 'standing_policy_active'
            if self.state in ('HOLD', 'LOCOMOTION'):
                if self.state == 'LOCOMOTION' and now-self.last_command > .3:
                    self.state, self.reason = 'HOLD', 'velocity_command_timeout'
                    self.command = [0., 0., 0.]
                if self.last_policy is None or now-self.last_policy >= .02-1e-8:
                    actions = vector(self.policy(observation(sensors, self.command)), 12)
                    self.target = [max(lo, min(hi, d+.2*max(-100, min(100, a*s))))
                                   for a,s,d,(lo,hi) in zip(actions, SYMMETRY, DEFAULT, LIMITS)]
                    self.last_policy = now
                    self.inferences += 1
            return self.target.copy()
        except Exception as error:
            self.fault(error)
            return None

    def tick(self, sensors, now):
        """Legacy host-PD path for torque-driver baseline tests."""
        target = self.tick_targets(sensors, now)
        if target is None:
            return [0.]*12
        return [max(-limit, min(limit, kp*(t-q)-kd*dq)) for t,q,dq,limit,kp,kd in
                zip(target, sensors.q, sensors.dq, self.effort,self.kp,self.kd)]

    def status(self):
        return {'state':self.state, 'reason':self.reason, 'command':self.command,
                'target':self.target, 'policy_inferences':self.inferences,
                'baseline_host_kp':self.kp,'baseline_host_kd':self.kd,
                'hardware_validated':False}
