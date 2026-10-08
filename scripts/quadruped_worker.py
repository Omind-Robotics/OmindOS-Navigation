#!/usr/bin/env python3
"""200 Hz control worker. Default MuJoCo; explicit calibrated CAN opt-in.

Loopback JSON bridge lets ROS Humble and the pinned Torch/MuJoCo environment
use separate Python versions. All actions execute on the control thread.
"""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import queue
import secrets
import signal
import threading
import time

from quadruped_controller import Controller, PinnedPolicy, NAMES, LIMITS, simulation_control_profile
from quadruped_backends import MujocoBackend, RobStrideBackend, SimulatedPositionBackend
from joint_interface import PositionCommand


class Worker:
    def __init__(self, backend, controller, control_profile_id=None):
        self.backend, self.controller = backend, controller
        self.control_profile_id = control_profile_id
        self.commands = queue.Queue(maxsize=16)
        self.boot_id = secrets.token_hex(16)
        self.snapshot = {'boot_id':self.boot_id,'state':'STARTING','hardware_active':False}
        self.last_sensors = None
        self.closed = False
        self.position_sequence = 0

    def submit(self, body):
        if body.get('boot_id') != self.boot_id:
            raise ValueError('worker_session_changed')
        age = time.time()-body.get('sent_at',0)
        if not 0 <= age <= .1:
            raise ValueError('stale_or_future_request')
        operation = body.get('operation')
        sample_age = body.get('sample_age_s',0.)
        if not isinstance(sample_age,(int,float)) or not math.isfinite(sample_age) or not 0 <= sample_age <= .15:
            raise ValueError('invalid_sample_age')
        if operation == 'imu':
            if not self.backend.hardware:
                raise ValueError('simulation_has_its_own_IMU')
            # Immutable tuple swap; no actuator access on the HTTP thread.
            if age+sample_age > .1: raise ValueError('stale_IMU_sample')
            self.backend.update_imu(body['quaternion'],body['gyro'],time.monotonic()-age-sample_age)
            return {'accepted':True}
        event = threading.Event()
        deadline = time.monotonic() + (1.5 if operation == 'arm' else .15)
        item = (body,event,{},deadline)
        self.commands.put_nowait(item)
        if not event.wait(1.6 if operation == 'arm' else .2):
            raise TimeoutError('control_request_expired')
        result = item[2]
        if 'error' in result:
            raise ValueError(result['error'])
        return result

    def execute(self, body, deadline):
        now = time.monotonic()
        if now > deadline:
            raise ValueError('control_request_expired_before_execution')
        op = body['operation']
        if op in ('estop','disarm'):
            self.controller.action(op,None,now)
            self.backend.disarm()
            return
        if op == 'arm':
            if self.controller.state != 'DISARMED':
                raise ValueError('controller_not_disarmed')
            try:
                self.backend.arm(deadline)
                now = time.monotonic()
                if now > deadline: raise ValueError('arm_deadline_expired')
                self.controller.action(op,self.backend.sensors(now),now)
                return
            except Exception as error:
                self.backend.disarm()
                self.controller.fault(error)
                raise
        sensors = self.backend.sensors(now)
        if op == 'velocity':
            # Do not rejuvenate a command delayed inside the request queue.
            age = time.time()-body['sent_at']+body.get('sample_age_s',0.)
            if not 0 <= age <= .15:
                raise ValueError('velocity_expired_in_queue')
            self.controller.velocity(body['velocity'], now-age)
        else:
            self.controller.action(op,sensors,now)

    def step(self):
        for _ in range(8):
            try:
                body,event,result,deadline = self.commands.get_nowait()
            except queue.Empty:
                break
            try:
                self.execute(body,deadline)
                result.update(self.controller.status())
            except Exception as error:
                result['error'] = str(error)
            finally:
                event.set()
        now = time.monotonic()
        try:
            if not self.backend.hardware or self.controller.state not in ('DISARMED','FAULT'):
                sensors = self.backend.sensors(now)
                self.last_sensors = sensors
                if getattr(self.backend, 'command_mode', None) == 'joint_position':
                    positions = self.controller.tick_targets(sensors,now)
                    if positions is None:
                        self.backend.disarm()
                        if self.controller.state != 'FAULT': self.backend.advance(now)
                    elif not self.closed:
                        command = PositionCommand(self.backend.command_session, self.position_sequence,
                                                  now, .05, tuple(NAMES), tuple(positions))
                        self.backend.apply_positions(command,now)
                        self.position_sequence += 1
                else:
                    effort = self.controller.tick(sensors,now)
                    if self.controller.state == 'FAULT':
                        self.backend.disarm()
                    elif self.backend.hardware or not self.closed:
                        self.backend.apply(effort)
        except Exception as error:
            self.controller.fault(error)
            self.backend.disarm()
        s = self.last_sensors
        self.snapshot = {**self.controller.status(), **self.backend.telemetry(),
            'boot_id':self.boot_id, 'joint_names':NAMES,
            'hardware_active':self.backend.hardware and self.controller.state not in ('DISARMED','FAULT'),
            'backend':'robstride04' if self.backend.hardware else 'mujoco',
            'control_mode':getattr(self.backend,'command_mode','host_effort_test'),
            'control_profile_sha256':self.control_profile_id,
            'sensor_age_s':max(0.,now-min(s.motor_time,s.imu_time)) if s else None,
            'monotonic_time':now, 'q':s.q if s else [], 'dq':s.dq if s else [],
            'quaternion_xyzw':s.quaternion_xyzw if s else [], 'gyro_body':s.gyro_body if s else []}


def handler(worker, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass

        def reply(self,code,body):
            data=json.dumps(body,allow_nan=False).encode()
            self.send_response(code); self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data))); self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError): pass

        def authorized(self):
            return hmac.compare_digest(self.headers.get('X-Omindos-Token',''),token)

        def do_GET(self):
            if not self.authorized(): return self.reply(403,{'error':'unauthorized'})
            if self.path != '/status': return self.reply(404,{'error':'unknown_path'})
            self.reply(200,worker.snapshot)

        def do_POST(self):
            if not self.authorized(): return self.reply(403,{'error':'unauthorized'})
            if self.path != '/command': return self.reply(404,{'error':'unknown_path'})
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 4096: raise ValueError('invalid_body_length')
                body=json.loads(self.rfile.read(length))
                if not isinstance(body,dict): raise ValueError('expected_object')
                self.reply(200,worker.submit(body))
            except (ValueError,TypeError,KeyError,TimeoutError,queue.Full) as error:
                self.reply(409,{'error':str(error)})
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--backend',choices=['mujoco','robstride04'],default='mujoco')
    parser.add_argument('--calibration',type=Path)
    parser.add_argument('--allow-hardware',action='store_true')
    parser.add_argument('--control-mode',choices=['joint-position','host-effort-test'],default='joint-position')
    parser.add_argument('--control-profile',type=Path)
    parser.add_argument('--mass-sensitivity-case',choices=['nominal_proximal','high_mass_proximal','distal_stress'])
    parser.add_argument('--simulate-rs04-frames',action='store_true')
    parser.add_argument('--port',type=int,default=18097)
    parser.add_argument('--token-file',type=Path,required=True)
    args=parser.parse_args()
    if args.control_mode=='joint-position' and (args.backend!='mujoco' or args.simulate_rs04_frames):
        parser.error('generic joint-position currently has a simulation adapter only; '
                     'the existing RS04 torque driver requires explicit --control-mode host-effort-test')
    if args.backend!='mujoco' and (args.control_profile or args.mass_sensitivity_case or args.simulate_rs04_frames):
        parser.error('candidate control profiles and mass sensitivity are simulation-only')
    gains=simulation_control_profile(args.control_profile) if args.control_profile else {}
    policy=PinnedPolicy(args.source)
    if args.backend == 'robstride04' and args.calibration is None:
        parser.error('--calibration required')
    backend=(MujocoBackend(policy.source) if args.backend=='mujoco' else
             RobStrideBackend(args.calibration,args.allow_hardware))
    if args.mass_sensitivity_case:
        from rs04_simulated_bus import modify_mass_sensitivity
        cases={'nominal_proximal':(.54,'proximal',1.),'high_mass_proximal':(.58,'proximal',2.),
               'distal_stress':(.58,'distal_stress',2.)}
        backend.sensitivity=modify_mass_sensitivity(backend,*cases[args.mass_sensitivity_case])
    if args.simulate_rs04_frames:
        from rs04_simulated_bus import RS04SimulationBackend
        backend=RS04SimulationBackend(backend)
    if args.control_mode=='joint-position':
        # The reference model has compliant mechanical stops; allow its existing
        # 0.08 rad sensor tolerance when capturing the folded pose during arm.
        backend=SimulatedPositionBackend(backend,NAMES,[(lo-.08,hi+.08) for lo,hi in LIMITS],
                                         gains.get('kp',[50.]*12),gains.get('kd',[2.]*12))
    worker=Worker(backend,Controller(policy,backend.effort_limits,kp=gains.get('kp'),kd=gains.get('kd')),
                  control_profile_id=gains.get('sha256'))
    token=secrets.token_hex(32)
    fd=os.open(args.token_file,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
    with os.fdopen(fd,'w') as stream: stream.write(token)
    os.chmod(args.token_file,0o600)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),handler(worker,token))
    server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print(json.dumps({'ready':True,'backend':args.backend,'port':args.port,'boot_id':worker.boot_id}),flush=True)
    target=time.monotonic()
    def shutdown_signal(signum,frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown_signal)
    try:
        while True:
            worker.step()
            target+=.005
            delay=target-time.monotonic()
            if delay < -.02: target=time.monotonic()
            elif delay > 0: time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        worker.closed=True
        backend.close()
        server.shutdown(); server.server_close()


if __name__=='__main__': main()
