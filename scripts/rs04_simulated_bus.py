"""RS04 wire-format emulator backed by MuJoCo, for software-in-the-loop only.

This module never opens an operating-system CAN/serial interface. It decodes
actual MotorSession frames, advances the same contact dynamics, and encodes
quantized position/velocity/torque feedback. Firmware behavior is an explicit
test model, not evidence about a real RS04 firmware build.
"""
import math
import struct

from quadruped_controller import NAMES, LIMITS, Sensors
from robstride04_driver import Frame, MotorSession


def calibration_fixture():
    directions = [1,-1,-1,1,1,1,-1,-1,-1,-1,1,1]
    ids = [10,11,12,7,8,9,4,5,6,1,2,3]
    return {'motor_model':'ROBSTRIDE04','protocol':'private_29bit',
            'robot_serial':'SIMULATED_ONLY','calibration_record':'synthetic-zero-offsets',
            'joints':[{'name':name,'channel':'sim0' if i<6 else 'sim1','motor_id':ids[i],
                'direction':directions[i],'offset_rad':0.,'external_ratio':1.,
                'lower':LIMITS[i][0],'upper':LIMITS[i][1],'effort_limit_nm':60.,
                'velocity_limit_rad_s':15.,'temperature_limit_c':80.}
                for i,name in enumerate(NAMES)]}


def modify_mass_sensitivity(sim, delta_per_motor, distribution, armature_scale=1.):
    """Explicit surrogate: spread added mass over existing body COM/inertia.

    Parent/proximal allocation: 4 motors on base, 4 on scapulae, 4 on thighs.
    Distal stress allocation: 4 on scapulae, 4 on thighs, 4 on calves.
    Neither allocation is a reconstructed customer CAD assembly.
    """
    if distribution not in ('proximal','distal_stress') or not 0 <= delta_per_motor <= 1:
        raise ValueError('invalid sensitivity case')
    if not 1 <= armature_scale <= 3:
        raise ValueError('invalid armature stress factor')
    before=float(sim.model.body_subtreemass[sim.base_id])
    allocations={}
    if distribution=='proximal': allocations['base_link']=4*delta_per_motor
    for leg in ('BL','BR','FL','FR'):
        for segment in (('scapula','thigh') if distribution=='proximal' else ('scapula','thigh','calf')):
            allocations[f'{leg}_{segment}_link']=delta_per_motor
    for name,delta in allocations.items():
        bid=sim.model.body(name).id
        old=sim.model.body_mass[bid]
        sim.model.body_mass[bid]=old+delta
        sim.model.body_inertia[bid]*=(old+delta)/old
    sim.model.dof_armature[sim.vids]*=armature_scale
    sim.mj.mj_setConst(sim.model,sim.data)
    sim.mj.mj_resetDataKeyframe(sim.model,sim.data,0)
    sim.mj.mj_forward(sim.model,sim.data)
    return {'base_mass_kg':before,'modified_mass_kg':float(sim.model.body_subtreemass[sim.base_id]),
            'added_mass_kg':12*delta_per_motor,'distribution':distribution,
            'armature_factor':armature_scale,'allocations_kg':allocations,
            'customer_model':False,'inertia_method':'scale_existing_body_inertia_at_unchanged_COM'}


class SimulatedBus:
    def __init__(self, sim, config, clock):
        self.sim,self.config,self.clock=sim,config,clock
        self.queue=[]
        self.joints={(j['channel'],j['motor_id']):(i,j) for i,j in enumerate(config['joints'])}
        self.mode={key:0 for key in self.joints}
        self.params={key:{0x7005:0,0x7028:0} for key in self.joints}
        self.last_rx={key:clock() for key in self.joints}
        self.effort=[0.]*12
        self.frames_sent=0; self.frames_feedback=0; self.closed=False
        self.drop_feedback=set(); self.faults={}; self.stop_failures=set()
        self.raw_peak_velocity=0.; self.watchdog_trips=0
        self.stopped_addresses=set()

    @staticmethod
    def _encode(value,extent):
        return int((max(-extent,min(extent,value))+extent)*65535/(2*extent))

    def _reply(self,key):
        if key in self.drop_feedback: return
        i,j=self.joints[key]; channel,motor=key
        p=float(self.sim.data.qpos[self.sim.qids[i]])*j['direction']
        v=float(self.sim.data.qvel[self.sim.vids[i]])*j['direction']
        self.raw_peak_velocity=max(self.raw_peak_velocity,abs(v))
        tau=self.effort[i]*j['direction']
        data=struct.pack('>4H',self._encode(p,12.57),self._encode(v,15.),self._encode(tau,120.),250)
        ident=(2<<24)|(self.mode[key]<<22)|(self.faults.get(key,0)<<16)|(motor<<8)|253
        self.queue.append((channel,Frame(ident,data),self.clock()))
        self.frames_feedback+=1

    def send(self,channel,packet):
        if self.closed: raise OSError('simulated bus closed')
        self.frames_sent+=1
        kind=packet.arbitration_id>>24
        motor=packet.arbitration_id&255
        key=channel,motor
        if key not in self.joints: return
        if len(packet.data)!=8: raise ValueError('invalid emulator DLC')
        i,j=self.joints[key]
        self.last_rx[key]=self.clock()
        if kind==1:
            raw_p,raw_v,raw_kp,raw_kd=struct.unpack('>4H',packet.data)
            p=raw_p*25.14/65535-12.57; v=raw_v*30/65535-15
            kp=raw_kp*5000/65535; kd=raw_kd*100/65535
            tau=((packet.arbitration_id>>8)&65535)*240/65535-120
            actual_p=float(self.sim.data.qpos[self.sim.qids[i]])*j['direction']
            actual_v=float(self.sim.data.qvel[self.sim.vids[i]])*j['direction']
            motor_tau=tau+kp*(p-actual_p)+kd*(v-actual_v)
            self.effort[i]=(max(-120.,min(120.,motor_tau))*j['direction'] if self.mode[key]==2 else 0.)
        elif kind==3:
            if self.faults.get(key,0): raise RuntimeError('attempted enable with simulated motor fault')
            self.mode[key]=2
        elif kind==4:
            if key in self.stop_failures: raise OSError('injected stop write failure')
            if any(packet.data): raise AssertionError('emulator refuses fault-clear commands')
            self.mode[key]=0; self.effort[i]=0.; self.stopped_addresses.add(key)
        elif kind==18:
            index=int.from_bytes(packet.data[:2],'little')
            if index not in (0x7005,0x7028): raise AssertionError('unexpected parameter write')
            self.params[key][index]=int.from_bytes(packet.data[4:],'little')
        elif kind==17:
            index=int.from_bytes(packet.data[:2],'little')
            value=self.params[key][index]
            data=struct.pack('<H2xI',index,value)
            self.queue.append((channel,Frame((17<<24)|(motor<<8)|253,data),self.clock()))
            return
        else:
            raise AssertionError(f'unexpected CAN command type {kind}')
        self._reply(key)

    def receive(self,timeout=0.):
        if self.queue: return self.queue.pop(0)
        if timeout: self.clock.now+=timeout
        return None

    def advance(self):
        for key in self.joints:
            timeout=self.params[key][0x7028]/20000
            if self.mode[key]==2 and timeout and self.clock()-self.last_rx[key]>timeout:
                self.mode[key]=0; self.effort[self.joints[key][0]]=0.; self.watchdog_trips+=1
        self.sim.apply(self.effort)
        self.clock.now+=.005

    def close(self): self.closed=True


class SimClock:
    def __init__(self): self.now=0.
    def __call__(self): return self.now


class DriverInLoop:
    def __init__(self,sim):
        self.sim=sim; self.clock=SimClock()
        self.bus=SimulatedBus(sim,calibration_fixture(),self.clock)
        self.driver=MotorSession(calibration_fixture(),self.bus,self.clock)
        self.driver.arm()

    def sensors(self):
        for _ in range(256):
            if not self.driver.poll(): break
        q,dq,stamp=self.driver.sample()
        imu=self.sim.sensors(self.clock())
        return Sensors(q,dq,imu.quaternion_xyzw,imu.gyro_body,stamp,imu.imu_time)

    def step(self,effort):
        self.driver.cycle(effort)
        self.bus.advance()

    def close(self): self.driver.close()


class RS04SimulationBackend:
    """Runtime backend: ROS actions exercise MotorSession against virtual motors.

    Physics and the motor-emulator clock advance together; this does not emulate
    real bus latency or certify host scheduling. No real device constructors.
    """
    hardware=False

    def __init__(self,physics):
        self.physics=physics; self.clock=SimClock()
        self.bus=SimulatedBus(physics,calibration_fixture(),self.clock)
        self.driver=MotorSession(calibration_fixture(),self.bus,self.clock)
        self.effort_limits=physics.effort_limits

    def arm(self,deadline=None): self.driver.arm()

    def sensors(self,now):
        imu=self.physics.sensors(now)
        if not self.driver.active: return imu
        for _ in range(256):
            if not self.driver.poll(): break
        q,dq,stamp=self.driver.sample()
        return Sensors(q,dq,imu.quaternion_xyzw,imu.gyro_body,now-(self.clock()-stamp),now)

    def apply(self,effort):
        if self.driver.active: self.driver.cycle(effort)
        else: self.bus.effort=[0.]*12
        self.bus.advance()

    def disarm(self): self.driver._send_all_stop()

    def telemetry(self):
        return {**self.physics.telemetry(),'transport':'simulated_RS04_CAN_frames',
                'simulated_CAN_tx_frames':self.bus.frames_sent,
                'simulated_CAN_feedback_frames':self.bus.frames_feedback,
                'motor_session_active':self.driver.active,'motor_session_fault':self.driver.fault}

    def close(self): self.driver.close()
