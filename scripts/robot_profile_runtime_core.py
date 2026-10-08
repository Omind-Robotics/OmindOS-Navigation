"""Versioned robot profiles applied to a fixed-base joint preview, never hardware."""
import math
import secrets
import time

from robot_parameter_api import load_robot_profile
from robot_kinematics import build_model, pose, positions, trial


class RuntimeErrorResponse(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class ProfileRuntime:
    def __init__(self, store_dir, clock=time.monotonic):
        self.store_dir, self.clock = store_dir, clock
        self.profile_id = self.profile = self.model = None
        self.state, self.reason = 'UNLOADED', 'startup_requires_explicit_load'
        self.q, self.velocity = {}, {}
        self.plan = None
        self.lease_token = None
        self.last_heartbeat = 0.0
        self.lease_timeout = 2.0
        self.generation = 0
        self.started = 0.0
        self.duration = 0.0
        self.elapsed = 0.0
        self.start_q = self.target_q = {}

    def _fail(self, code, message):
        raise RuntimeErrorResponse(code, message)

    def _lease(self, payload):
        token = payload.get('lease_token')
        if not isinstance(token, str) or not self.lease_token or not secrets.compare_digest(token, self.lease_token):
            self._fail(409, 'preview session changed; explicitly load the configuration again')

    def tick(self):
        if self.state != 'RUNNING':
            return
        now = self.clock()
        if not math.isfinite(now) or now < self.started or now - self.last_heartbeat > self.lease_timeout:
            self.state, self.reason = 'STOPPED', 'preview_client_heartbeat_lost'
            self.velocity = {n: 0.0 for n in self.q}
            return
        u = min(1.0, max(0.0, (now-self.started)/self.duration))
        self.elapsed = u*self.duration
        smooth = u*u*(3-2*u)
        self.q = {n:self.start_q[n]+(self.target_q[n]-self.start_q[n])*smooth for n in self.start_q}
        self.velocity = {n:(self.target_q[n]-self.start_q[n])*6*u*(1-u)/self.duration for n in self.start_q}
        if u == 1.0:
            self.state, self.reason = 'READY', 'trajectory_complete'

    def command(self, op, payload):
        if not isinstance(payload, dict):
            self._fail(400, 'expected JSON object')
        allowed = {'load':{'profile_id'}, 'run':{'profile_id','lease_token','target','duration_s'},
                   'stop':{'lease_token'}, 'heartbeat':{'lease_token'}}
        if op not in allowed or set(payload) != allowed[op]:
            self._fail(400, 'unexpected command fields')
        self.tick()
        if op == 'load':
            if self.state == 'RUNNING':
                self._fail(409, 'stop the preview before loading another configuration')
            try:
                candidate = load_robot_profile(payload['profile_id'], self.store_dir)
                model = build_model(candidate['kinematics']['urdf_xml'])
                q = positions(model, {})
            except FileNotFoundError:
                self._fail(404, 'profile not found in the runtime store; use the same --store directory')
            # Commit only after all parsing and validation have succeeded.
            self.profile_id, self.profile, self.model = payload['profile_id'], candidate, model
            self.q, self.velocity = q, {n:0.0 for n in q}
            self.state, self.reason = 'READY', 'profile_loaded_into_ros2_joint_preview'
            self.plan = None
            self.duration = 0.0
            self.elapsed = 0.0
            self.generation += 1
            self.lease_token = secrets.token_urlsafe(24)
            self.last_heartbeat = self.clock()
            return {**self.snapshot(), 'lease_token':self.lease_token}
        self._lease(payload)
        if op == 'heartbeat':
            self.last_heartbeat = self.clock()
        elif op == 'stop':
            self.state, self.reason = 'STOPPED', 'operator_stop'
            self.velocity = {n:0.0 for n in self.q}
        elif op == 'run':
            if self.state == 'RUNNING':
                self._fail(409, 'preview is already running')
            if self.profile_id != payload['profile_id']:
                self._fail(409, 'configuration differs from the loaded profile; load it first')
            if not isinstance(payload['target'], dict) or set(payload['target']) != set(self.q):
                self._fail(400, 'target must contain every active joint by name')
            # Recheck the immutable version before executing, including disk tampering.
            checked = load_robot_profile(self.profile_id, self.store_dir)
            candidate = trial({'profile':checked,'start':self.q,'target':payload['target'],
                               'duration_s':payload['duration_s']})
            self.plan = candidate
            self.start_q, self.target_q = self.q.copy(), candidate['frames'][-1]['positions']
            self.duration = candidate['duration_s']
            self.started = self.last_heartbeat = self.clock()
            self.elapsed = 0.0
            self.state, self.reason = 'RUNNING', 'kinematic_trajectory_running'
        return self.snapshot()

    def snapshot(self):
        frame = pose(self.model, self.q) if self.model else None
        motor = {} if not self.profile else {n:(q-self.profile['dynamics']['joints'][n]['zero_offset_rad'])
                *self.profile['dynamics']['joints'][n]['reduction_ratio']
                /self.profile['dynamics']['joints'][n]['direction'] for n,q in self.q.items()}
        return {'state':self.state,'reason':self.reason,'profile_id':self.profile_id,
                'generation':self.generation,'runtime_applied':self.profile is not None,
                'runtime_target':'ros2_joint_preview','hardware_active':False,'hardware_validated':False,
                'model':self.model,'pose':frame,'velocities_rad_s':self.velocity,
                'motor_positions_rad':motor,'elapsed_s':self.elapsed,'duration_s':self.duration,
                'gravity_limit_exceeded':self.plan['gravity_limit_exceeded'] if self.plan else [],
                'lease_timeout_s':self.lease_timeout,
                'scope':'fixed_base_kinematics_and_static_gravity'}
