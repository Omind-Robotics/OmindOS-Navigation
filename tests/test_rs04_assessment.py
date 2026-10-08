import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from check_rs04_hardware import inventory,read_assessment,ReadOnlyTransport,addresses
from robstride04_driver import Frame,frame,read_parameter,MotorSession
from quadruped_controller import Controller,Sensors,DEFAULT,simulation_control_profile
from test_quadruped_control import calibrated_fixture,Clock,FakeBus


class ParameterBus:
    def __init__(self,clock):
        self.clock=clock; self.queue=[]; self.sent=[]; self.closed=False
        self.missing=None; self.active=False; self.nonfinite=False
    def send(self,channel,packet):
        self.sent.append(packet)
        motor=packet.arbitration_id&255; index=int.from_bytes(packet.data[:2],'little')
        if motor==self.missing: return
        if self.active:
            self.queue.append((channel,Frame((2<<24)|(2<<22)|(motor<<8)|253,b'\x00'*8),self.clock()))
        values={0x7005:0,0x7019:0.,0x701B:0.,0x701C:48.,0x7028:2000}
        value=values[index]
        if self.nonfinite and index==0x7019: value=float('nan')
        data=struct.pack('<H2x',index)+(struct.pack('<I',value) if index in (0x7005,0x7028) else struct.pack('<f',value))
        self.queue.append((channel,Frame((17<<24)|(motor<<8)|253,data),self.clock()))
    def receive(self,timeout):
        if self.queue: return self.queue.pop(0)
        self.clock.now+=timeout
        return None
    def close(self): self.closed=True


class AssessmentTests(unittest.TestCase):
    def test_inventory_empty_without_opening_devices(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            r=inventory(root/'sys',root/'dev')
            self.assertFalse(r['hardware_connection_observed'])
    def test_inventory_classifies_can_and_usb_serial_separately(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); net=root/'sys/class/net/can0'; net.mkdir(parents=True)
            (net/'type').write_text('280\n'); (net/'operstate').write_text('down\n')
            (root/'dev').mkdir(); (root/'dev/ttyUSB0').touch()
            r=inventory(root/'sys',root/'dev')
            self.assertEqual(r['can_interfaces'],[{'name':'can0','state':'down'}])
            self.assertFalse(r['usb_serial_candidates'][0]['SocketCAN_compatibility_confirmed'])
    def test_probe_requires_explicit_disabled_confirmation(self):
        bus=ParameterBus(Clock())
        with self.assertRaises(ValueError): read_assessment(calibrated_fixture(),bus)
        self.assertFalse(bus.sent)
    def test_readonly_rejects_enable_stop_motion_and_writes(self):
        bus=ParameterBus(Clock()); wrapper=ReadOnlyTransport(bus,addresses(calibrated_fixture()))
        for kind in (0,1,3,4,6,7,18,22):
            with self.assertRaises(ValueError): wrapper.send('test0',frame(kind,1))
        with self.assertRaises(ValueError): wrapper.send('test0',read_parameter(1,0x702b))
        self.assertFalse(bus.sent)
    def test_complete_assessment_sends_only_parameter_reads(self):
        clock=Clock(); bus=ParameterBus(clock)
        result=read_assessment(calibrated_fixture(),bus,True,clock)
        self.assertEqual((result['status'],result['responsive_motors']),('READS_COMPLETE',12))
        self.assertTrue(all(p.arbitration_id>>24==17 for p in bus.sent)); self.assertEqual(len(bus.sent),60)
        self.assertTrue(bus.closed); self.assertFalse(result['hardware_motion_validated'])
        self.assertEqual(result['motors'][0]['parameters']['bus_voltage_v'],48.)
    def test_missing_motor_is_incomplete(self):
        clock=Clock(); bus=ParameterBus(clock); bus.missing=12
        result=read_assessment(calibrated_fixture(),bus,True,clock)
        self.assertEqual((result['status'],result['responsive_motors']),('INCOMPLETE',11))
    def test_observed_running_motor_aborts_without_stop_write(self):
        clock=Clock(); bus=ParameterBus(clock); bus.active=True
        result=read_assessment(calibrated_fixture(),bus,True,clock)
        self.assertEqual(result['error'],'motor_not_disabled_reading_aborted')
        self.assertEqual(len(bus.sent),1); self.assertTrue(bus.closed)
    def test_nonfinite_parameter_fails(self):
        clock=Clock(); bus=ParameterBus(clock); bus.nonfinite=True
        result=read_assessment(calibrated_fixture(),bus,True,clock)
        self.assertEqual(result['error'],'non_finite_parameter_value')
    def test_feedback_saturation_latches_motor_fault(self):
        for pos,vel in [(0,32767),(65535,32767),(32767,0),(32767,65535)]:
            clock=Clock(); bus=FakeBus(clock); driver=MotorSession(calibrated_fixture(),bus,clock)
            driver.arm()
            bus.queue.append(('test0',Frame((2<<24)|(2<<22)|(1<<8)|253,
                struct.pack('>4H',pos,vel,32767,250)),clock()))
            with self.assertRaisesRegex(RuntimeError,'encoding_saturated'): driver.cycle([0.]*12)
            self.assertFalse(driver.active)
            self.assertEqual([p.arbitration_id>>24 for _,p in bus.sent[-12:]],[4]*12)
    def test_adjustable_host_damping_is_applied(self):
        controller=Controller(lambda obs:[0.]*12,kd=[3.]*12)
        s=Sensors(DEFAULT.copy(),[0.]*12,[0.,0.,0.,1.],[0.]*3,1.,1.)
        controller.action('arm',s,1.); s.dq=[1.]*12
        self.assertEqual(controller.tick(s,1.),[-3.]*12)
    def test_candidate_profile_pins_policy_and_remains_simulation_only(self):
        p=Path(__file__).resolve().parents[1]/'config/robot_profiles/rs04-simulation-control.json'
        result=simulation_control_profile(p)
        self.assertEqual(result['kd'],[3.]*12); self.assertEqual(len(result['sha256']),64)
        with tempfile.TemporaryDirectory() as directory:
            broken=json.loads(p.read_text()); broken['scope']='hardware'
            out=Path(directory)/'profile.json'; out.write_text(json.dumps(broken))
            with self.assertRaises(ValueError): simulation_control_profile(out)
    def test_invalid_gains_rejected(self):
        for kd in ([0.]*12,[float('nan')]*12,[21.]*12):
            with self.assertRaises(ValueError): Controller(lambda obs:[0.]*12,kd=kd)


if __name__=='__main__': unittest.main()
