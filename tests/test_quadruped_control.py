from pathlib import Path
import struct
import sys
import unittest
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from quadruped_controller import Controller, Sensors, DEFAULT, NAMES, observation
from robstride04_driver import (Frame, MotorSession, feedback, control, frame,
                               write_parameter, validate_calibration, HOST_ID)
from quadruped_worker import Worker
from quadruped_backends import RobStrideBackend


class Clock:
    def __init__(self): self.now = 1.
    def __call__(self): return self.now


def calibrated_fixture():
    return {'motor_model':'ROBSTRIDE04','protocol':'private_29bit',
        'robot_serial':'FAKE_BUS_ONLY','calibration_record':'test-fixture-not-hardware',
        'joints':[{'name':n,'channel':'test0' if i<6 else 'test1','motor_id':i+1,
                   'direction':1,'offset_rad':0.,'external_ratio':1.,'lower':-3.2,'upper':3.2,
                   'effort_limit_nm':30.,'velocity_limit_rad_s':10.,'temperature_limit_c':80.}
                  for i,n in enumerate(NAMES)]}


class FakeBus:
    def __init__(self, clock):
        self.clock = clock
        self.sent, self.queue, self.params, self.modes = [], [], {}, {}
        self.bad_readback = False
        self.drop_motor = None
        self.closed = False

    def reply(self, channel, motor, mode=None, fault=0):
        mode = self.modes.get((channel,motor),0) if mode is None else mode
        ident = (2<<24)|(mode<<22)|(fault<<16)|(motor<<8)|HOST_ID
        data = struct.pack('>4H',32767,32767,32767,250)
        self.queue.append((channel,Frame(ident,data),self.clock()))

    def send(self, channel, packet):
        self.sent.append((channel,packet))
        kind, motor = packet.arbitration_id>>24, packet.arbitration_id & 255
        if motor == self.drop_motor: return
        if kind == 4: self.modes[channel,motor] = 0
        if kind == 3: self.modes[channel,motor] = 2
        if kind == 18:
            index = int.from_bytes(packet.data[:2],'little')
            self.params[channel,motor,index] = int.from_bytes(packet.data[4:],'little')
        if kind == 17:
            index = int.from_bytes(packet.data[:2],'little')
            value = self.params[channel,motor,index] + int(self.bad_readback)
            reply = Frame((17<<24)|(motor<<8)|HOST_ID,struct.pack('<H2xI',index,value))
            self.queue.append((channel,reply,self.clock()))
        else: self.reply(channel,motor)

    def receive(self, timeout=0):
        if self.queue: return self.queue.pop(0)
        self.clock.now += timeout
        return None

    def close(self): self.closed = True


class ProtocolTests(unittest.TestCase):
    def session(self):
        clock = Clock(); bus = FakeBus(clock)
        return MotorSession(calibrated_fixture(),bus,clock),bus,clock

    def test_rs04_golden_min_mid_max(self):
        p = control(12,-12.57,-15,0,0,-120)
        self.assertEqual((p.arbitration_id,p.data.hex()),(0x0100000c,'0000000000000000'))
        p = control(12,12.57,15,5000,100,120)
        self.assertEqual((p.arbitration_id,p.data.hex()),(0x01ffff0c,'ffffffffffffffff'))
        p = control(1)
        self.assertEqual((p.arbitration_id,p.data.hex()),(0x017fff01,'7fff7fff00000000'))
        with self.assertRaises(ValueError): control(1,velocity=15.1)
        with self.assertRaises(ValueError): control(1,torque=float('nan'))

    def test_all_six_fault_bits_and_run_mode(self):
        for fault in [1,2,4,8,16,32]:
            f = feedback(Frame((2<<24)|(2<<22)|(fault<<16)|(12<<8)|253,b'\xff'*6+b'\x03\x20'))
            self.assertEqual((f['fault'],f['mode'],f['motor_id']),(fault,2,12))
            self.assertEqual((f['velocity'],f['torque'],f['temperature']),(15,120,80))

    def test_wrong_host_and_malformed_frame(self):
        self.assertIsNone(feedback(Frame((2<<24)|(12<<8)|252,b'\x00'*8)))
        with self.assertRaises(ValueError): feedback(Frame(0,b'\x00'))

    def test_watchdog_bytes_and_no_persistent_or_zero_writes(self):
        f=write_parameter(1,0x7028,2000)
        self.assertEqual(f.arbitration_id,0x1200fd01)
        self.assertEqual(f.data.hex(),'28700000d0070000')
        for index in [0x7029,0x702b,0x700b]:
            with self.assertRaises(ValueError): write_parameter(1,index,0)

    def test_missing_calibration_and_duplicate_address_rejected(self):
        config=calibrated_fixture(); config['joints'][0]['offset_rad']=None
        with self.assertRaises(ValueError): validate_calibration(config)
        config=calibrated_fixture(); config['joints'][1]['motor_id']=1
        with self.assertRaises(ValueError): validate_calibration(config)

    def test_constructor_sends_nothing_and_arm_verifies_watchdog(self):
        session,bus,_=self.session()
        self.assertFalse(bus.sent)
        session.arm()
        self.assertTrue(session.active)
        self.assertEqual(len([k for k,v in bus.params.items() if k[2]==0x7028 and v==2000]),12)
        self.assertTrue(all(((p.arbitration_id>>24) in (1,3,4,17,18)) for _,p in bus.sent))

    def test_readback_mismatch_prevents_every_enable(self):
        session,bus,_=self.session(); bus.bad_readback=True
        with self.assertRaisesRegex(RuntimeError,'readback'): session.arm()
        self.assertFalse(any(p.arbitration_id>>24==3 for _,p in bus.sent))
        self.assertFalse(session.active)

    def test_missing_motor_prevents_enable(self):
        session,bus,_=self.session(); bus.drop_motor=12
        with self.assertRaises(RuntimeError): session.arm()
        self.assertFalse(any(p.arbitration_id>>24==3 for _,p in bus.sent))

    def test_external_ratio_and_direction_torque_mapping(self):
        config=calibrated_fixture(); config['joints'][0].update(direction=-1,external_ratio=2.,velocity_limit_rad_s=5.)
        clock=Clock(); bus=FakeBus(clock); session=MotorSession(config,bus,clock)
        session.arm(); session.cycle([20.]+[0.]*11)
        packet=[p for _,p in bus.sent if p.arbitration_id>>24==1][-12]
        raw=(packet.arbitration_id>>8)&65535
        self.assertAlmostEqual(raw*240/65535-120,-10.,places=2)
        self.assertEqual(packet.data[4:],b'\x00'*4)

    def test_stale_cycle_latches_and_stops_all(self):
        session,bus,clock=self.session(); session.arm(); clock.now+=.11
        with self.assertRaisesRegex(RuntimeError,'deadline'): session.cycle([0.]*12)
        self.assertEqual([p.arbitration_id>>24 for _,p in bus.sent[-12:]],[4]*12)
        with self.assertRaises(RuntimeError): session.arm()

    def test_nan_rejects_entire_batch_before_motion(self):
        session,bus,_=self.session(); session.arm(); count=len(bus.sent)
        with self.assertRaises(RuntimeError): session.cycle([0.]*11+[float('nan')])
        self.assertFalse(any(p.arbitration_id>>24==1 for _,p in bus.sent[count:]))

    def test_fault_frame_causes_stop(self):
        session,bus,_=self.session(); session.arm(); bus.reply('test0',1,fault=32)
        with self.assertRaises(RuntimeError): session.cycle([0.]*12)
        self.assertFalse(session.active)

    def test_wrong_bus_identity_cannot_refresh_known_motor(self):
        session,bus,clock=self.session(); session.arm(); old=session.latest['test0',1][1]
        clock.now+=.02; bus.reply('unrelated',1); session.poll()
        self.assertEqual(session.latest['test0',1][1],old)

    def test_close_stops_and_closes_transport(self):
        session,bus,_=self.session(); session.arm(); session.close()
        self.assertFalse(session.active); self.assertTrue(bus.closed)

    def test_arm_authorization_expiry_prevents_enable(self):
        session,bus,_=self.session()
        def expired(): raise RuntimeError('arm expired')
        with self.assertRaisesRegex(RuntimeError,'expired'): session.arm(expired)
        self.assertFalse(any(p.arbitration_id>>24==3 for _,p in bus.sent))

    def test_failure_on_one_bus_still_stops_other_motors(self):
        session,bus,_=self.session(); session.arm(); send=bus.send
        def broken(channel,packet):
            if packet.arbitration_id & 255 == 1: raise OSError('bus down')
            send(channel,packet)
        bus.send=broken; session.close()
        self.assertEqual([p.arbitration_id&255 for _,p in bus.sent[-11:]],list(range(2,13)))
        self.assertTrue(bus.closed)


class ControllerTests(unittest.TestCase):
    def sensor(self,now=1.): return Sensors(DEFAULT.copy(),[0.]*12,[0.,0.,0.,1.],[0.,0.,0.],now,now)
    def controller(self): return Controller(lambda obs:[0.]*12)
    def standing(self):
        c=self.controller(); c.action('arm',self.sensor(),1.); c.action('stand',self.sensor(),1.)
        for i in range(1,602):
            t=1+i*.005; c.tick(self.sensor(t),t)
        self.assertEqual(c.state,'HOLD')
        return c

    def test_observation_order_and_gravity(self):
        obs=observation(self.sensor(),[.2,.1,.3])
        self.assertEqual(len(obs),34); self.assertEqual(obs[3:6],[0.,-0.,-1.])
        self.assertEqual(obs[6:9],[.4,.2,.075]); self.assertEqual(obs[9:33],[0.]*24)

    def test_startup_disarmed_no_effort(self):
        c=self.controller(); self.assertEqual(c.tick(self.sensor(),1.),[0.]*12)
        with self.assertRaises(ValueError): c.velocity([.2,0,0],1.)

    def test_velocity_timeout_requires_explicit_resume(self):
        c=self.standing(); c.action('walk',self.sensor(4.01),4.01); c.velocity([.2,0,0],4.01)
        for i in range(1,70):
            t=4.01+i*.005; c.tick(self.sensor(t),t)
        self.assertEqual((c.state,c.reason),('HOLD','velocity_command_timeout'))
        with self.assertRaises(ValueError): c.velocity([.2,0,0],4.36)

    def test_stop_keeps_balance_and_estop_latches(self):
        c=self.standing(); c.action('walk',self.sensor(4.01),4.01)
        c.action('stop',self.sensor(4.02),4.02); n=c.inferences
        c.tick(self.sensor(4.03),4.03); self.assertGreater(c.inferences,n)
        c.action('estop',self.sensor(4.04),4.04)
        self.assertEqual(c.tick(self.sensor(4.05),4.05),[0.]*12)
        with self.assertRaises(ValueError): c.action('arm',self.sensor(4.06),4.06)

    def test_stale_imu_and_bad_quaternion(self):
        for change in ('old','zero'):
            c=self.standing(); s=self.sensor(4.01)
            if change=='old': s.imu_time=3.
            else: s.quaternion_xyzw=[0.]*4
            c.tick(s,4.01); self.assertEqual(c.state,'FAULT')

    def test_tilt_and_policy_nan_fault(self):
        c=self.standing(); s=self.sensor(4.02); s.quaternion_xyzw=[1.,0.,0.,0.]
        c.tick(s,4.02); self.assertEqual(c.reason,'excessive_tilt')
        c=self.standing(); c.policy=lambda obs:[float('nan')]*12
        c.tick(self.sensor(4.04),4.04); self.assertEqual(c.state,'FAULT')

    def test_torque_limited_before_backend(self):
        c=self.controller(); c.action('arm',self.sensor(),1.); s=self.sensor(1.005)
        s.dq=[100.]*12
        self.assertEqual(c.tick(s,1.005),[-60.]*12)


class WorkerGateTests(unittest.TestCase):
    class Backend:
        hardware=False
        def __init__(self): self.armed=0; self.stopped=0
        def arm(self,deadline=None): self.armed+=1
        def disarm(self): self.stopped+=1
        def sensors(self,now): return Sensors(DEFAULT.copy(),[0.]*12,[0.,0.,0.,1.],[0.,0.,0.],now,now)

    def test_expired_queued_arm_does_not_touch_backend(self):
        backend=self.Backend(); worker=Worker(backend,Controller(lambda obs:[0.]*12))
        with self.assertRaisesRegex(ValueError,'expired'):
            worker.execute({'operation':'arm'},time.monotonic()-1)
        self.assertEqual(backend.armed,0)

    def test_repeated_arm_does_not_disable_running_controller(self):
        backend=self.Backend(); worker=Worker(backend,Controller(lambda obs:[0.]*12))
        worker.execute({'operation':'arm'},time.monotonic()+1)
        with self.assertRaises(ValueError): worker.execute({'operation':'arm'},time.monotonic()+1)
        self.assertEqual((backend.armed,backend.stopped,worker.controller.state),(1,0,'STANDBY'))

    def test_old_worker_epoch_rejected_before_queue(self):
        worker=Worker(self.Backend(),Controller(lambda obs:[0.]*12))
        with self.assertRaisesRegex(ValueError,'session'):
            worker.submit({'operation':'arm','boot_id':'old','sent_at':time.time()})
        self.assertTrue(worker.commands.empty())

    def test_customer_template_cannot_open_any_can_interface(self):
        root=Path(__file__).resolve().parents[1]
        def forbidden(channels): self.fail('attempted to open hardware')
        with self.assertRaises(ValueError):
            RobStrideBackend(root/'config/robot_profiles/robstride04.calibration-template.json',
                             allow_hardware=True,transport_factory=forbidden)


if __name__=='__main__': unittest.main()
