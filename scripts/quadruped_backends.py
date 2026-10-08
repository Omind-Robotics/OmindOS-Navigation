"""Physics and real CAN backends for the same sensor/effort controller."""
import hashlib
import json
import time

from quadruped_controller import COMMIT, NAMES, Sensors, vector


class MujocoBackend:
    hardware = False

    def __init__(self, source):
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        self.model = mujoco.MjModel.from_xml_path(str(source/'models/scene.xml'))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.qids = [self.model.jnt_qposadr[self.model.joint(n).id] for n in NAMES]
        self.vids = [self.model.jnt_dofadr[self.model.joint(n).id] for n in NAMES]
        self.aids = [self.model.actuator(n).id for n in NAMES]
        self.base_id = self.model.body('base_link').id
        self.effort_limits = [60.]*12
        self.calibration_id = None
        if abs(self.model.opt.timestep-.005) > 1e-10:
            raise ValueError('expected 200 Hz reference physics')

    def sensors(self, now):
        quat = self.data.sensor('body_quat_sensor').data
        return Sensors(self.data.qpos[self.qids].tolist(), self.data.qvel[self.vids].tolist(),
                       quat[[1,2,3,0]].tolist(), self.data.sensor('body_gyro_sensor').data.tolist(), now, now)

    def arm(self, deadline=None):
        pass

    def apply(self, effort):
        self.data.ctrl[self.aids] = effort
        self.mj.mj_step(self.model, self.data)
        if not self.np.isfinite(self.data.qpos).all() or not self.np.isfinite(self.data.qvel).all():
            raise RuntimeError('non_finite_physics')

    def disarm(self):
        self.data.ctrl[:] = 0

    def telemetry(self):
        body = self.data.body('base_link')
        return {'base_xyz':body.xpos.tolist(), 'upright':float(body.xmat[8]),
                'body_velocity':self.data.sensor('body_vel_sensor').data.tolist(),
                'physics_time':float(self.data.time), 'model':'pinned_MEVIUS2_RS03_reference',
                'customer_RS04_dynamics_validated':False}

    def close(self):
        self.disarm()


class RobStrideBackend:
    hardware = True

    def __init__(self, config_path, allow_hardware=False, clock=time.monotonic, transport_factory=None):
        from robstride04_driver import MotorSession, SocketCAN, validate_calibration
        raw = config_path.read_bytes()
        config = json.loads(raw)
        validate_calibration(config)
        if not allow_hardware:
            raise ValueError('hardware requires explicit --allow-hardware')
        if [j['name'] for j in config['joints']] != NAMES:
            raise ValueError('calibration joint order must match named policy order')
        # A purchased motor model does not establish compatibility of this policy.
        record = config.get('policy_validation') or {}
        if record.get('reference_commit') != COMMIT or not record.get('record_id') or record.get('approved') is not True:
            raise ValueError('customer dynamics/policy validation record required')
        if config.get('imu_frame') != 'base_link' or not config.get('imu_mount_calibration_record'):
            raise ValueError('calibrated base_link IMU required')
        self.clock = clock
        self.effort_limits = [j['effort_limit_nm'] for j in config['joints']]
        self.calibration_id = hashlib.sha256(raw).hexdigest()
        self.imu = None
        factory = transport_factory or SocketCAN
        transport = factory(sorted({j['channel'] for j in config['joints']}))
        self.session = MotorSession(config, transport, clock)

    def update_imu(self, quaternion, gyro, received):
        q, g = vector(quaternion, 4), vector(gyro, 3)
        if abs(sum(x*x for x in q)-1) > .02:
            raise ValueError('invalid IMU quaternion')
        self.imu = q, g, received

    def arm(self, deadline=None):
        def authorize():
            if deadline is not None and self.clock() > deadline:
                raise ValueError('arm_deadline_expired')
            if self.imu is None or not 0 <= self.clock()-self.imu[2] <= .1:
                raise ValueError('fresh calibrated IMU required before motor enable')
            x,y,_,_=self.imu[0]
            if 1-2*(x*x+y*y)<.6:
                raise ValueError('excessive_tilt_before_motor_enable')
        authorize()
        self.session.arm(authorize)

    def sensors(self, now):
        if not self.session.active:
            raise ValueError('motors are disarmed')
        for _ in range(256):
            if not self.session.poll(): break
        else:
            raise RuntimeError('CAN_receive_flood')
        q, dq, stamp = self.session.sample()
        if self.imu is None:
            raise RuntimeError('IMU unavailable')
        quat, gyro, imu_time = self.imu
        return Sensors(q, dq, quat, gyro, stamp, imu_time)

    def apply(self, effort):
        self.session.cycle(effort)

    def disarm(self):
        self.session._send_all_stop()

    def telemetry(self):
        return {'model':'customer_RS04', 'calibration_id':self.calibration_id,
                'motor_fault':self.session.fault, 'customer_RS04_dynamics_validated':False}

    def close(self):
        self.session.close()
