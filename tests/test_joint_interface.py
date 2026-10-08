from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from joint_interface import PositionCommand, PositionCommandGuard, JointFeedback
from quadruped_controller import Controller, Sensors, DEFAULT, NAMES, LIMITS
from quadruped_backends import SimulatedPositionBackend
from quadruped_worker import Worker


class PositionContractTests(unittest.TestCase):
    def guard(self, count=2):
        guard=PositionCommandGuard([f'j{i}' for i in range(count)],[(-2,2)]*count)
        guard.arm('session',1.)
        return guard

    def command(self, **changes):
        return replace(PositionCommand('session',0,1.,.05,('j0','j1'),(.2,-.3)),**changes)

    def test_generic_one_twelve_and_fifteen_joints(self):
        for count in (1,12,15):
            g=self.guard(count)
            p=PositionCommand('session',0,1.,.05,tuple(f'j{i}' for i in range(count)),(.1,)*count)
            self.assertEqual(g.accept(p,1.),(.1,)*count)

    def test_reordered_names_map_angles(self):
        self.assertEqual(self.guard().accept(self.command(joint_names=('j1','j0')),1.),(-.3,.2))

    def test_partial_duplicate_unknown_joints_reject_whole_batch(self):
        for names,positions in [(('j0',),(.1,)),(('j0','j0'),(.1,.2)),(('j0','other'),(.1,.2))]:
            g=self.guard()
            with self.assertRaises(ValueError): g.accept(self.command(joint_names=names,position=positions),1.)
            self.assertFalse(g.active); self.assertEqual(g.sequence,-1)

    def test_nonfinite_boolean_and_out_of_limit(self):
        for value in (float('nan'),float('inf'),True,2.1):
            with self.assertRaises(ValueError): self.guard().accept(self.command(position=(.1,value)),1.)

    def test_degrees_and_unknown_schema_rejected(self):
        for p in (self.command(unit='deg'),self.command(schema='other')):
            with self.assertRaises(ValueError): self.guard().accept(p,1.)

    def test_source_age_future_and_overlong_validity_rejected(self):
        for p in (self.command(issued_monotonic_s=.9),self.command(issued_monotonic_s=1.01),
                  self.command(valid_for_s=.2),self.command(valid_for_s=float('nan'))):
            with self.assertRaises(ValueError): self.guard().accept(p,1.)

    def test_replay_and_prior_session_latch(self):
        g=self.guard(); g.accept(self.command(),1.)
        with self.assertRaisesRegex(ValueError,'replayed'): g.accept(self.command(),1.01)
        self.assertFalse(g.active)
        with self.assertRaisesRegex(ValueError,'session'):
            self.guard().accept(self.command(session_id='old'),1.)

    def test_disarm_requires_new_session_and_cannot_reset_fault(self):
        g=self.guard(); g.disarm()
        with self.assertRaises(ValueError): g.arm('session',1.01)
        g.arm('second',1.01)
        with self.assertRaises(ValueError): g.accept(self.command(),1.01)
        g.disarm()
        with self.assertRaises(ValueError): g.arm('third',1.02)

    def test_fresh_packet_cannot_revive_expired_stream(self):
        g=self.guard(); g.accept(self.command(),1.)
        with self.assertRaisesRegex(ValueError,'deadline'):
            g.accept(self.command(sequence=1,issued_monotonic_s=1.06),1.06)
        self.assertFalse(g.active)

    def test_packet_roundtrip_and_exact_fields(self):
        p=self.command(); self.assertEqual(PositionCommand.from_dict(json.loads(json.dumps(p.to_dict()))),p)
        for change in ({'torque':[1.,1.]},{'extra':0}):
            with self.assertRaises(ValueError): PositionCommand.from_dict({**p.to_dict(),**change})

    def test_feedback_named_mapping_age_faults_and_finite_values(self):
        f=JointFeedback(('j1','j0'),(1.,2.),(3.,4.),(0,0),1.)
        self.assertEqual(f.ordered(('j0','j1'),1.),((2.,1.),(4.,3.)))
        for bad in (replace(f,faults=(0,1)),replace(f,sampled_monotonic_s=.8),
                    replace(f,velocity=(0.,float('nan'))),replace(f,joint_names=('j0','j0'))):
            with self.assertRaises(ValueError): bad.ordered(('j0','j1'),1.)


class FakePhysics:
    hardware=False
    effort_limits=[60.]*12
    def __init__(self): self.sent=[]; self.stops=0; self.velocity=[0.]*12
    def arm(self,deadline=None): pass
    def sensors(self,now): return Sensors(DEFAULT.copy(),self.velocity.copy(),[0.,0.,0.,1.],[0.,0.,0.],now,now)
    def apply(self,effort): self.sent.append(effort)
    def disarm(self): self.stops+=1
    def close(self): self.disarm()
    def telemetry(self): return {'model':'test_fixture'}


class PositionRuntimeTests(unittest.TestCase):
    def adapter(self,physics=None,clock=time.monotonic):
        return SimulatedPositionBackend(physics or FakePhysics(),NAMES,LIMITS,[50.]*12,[2.]*12,clock=clock)

    def test_policy_returns_angles_and_no_command_when_disabled(self):
        c=Controller(lambda obs:[0.]*12); s=FakePhysics().sensors(1.)
        self.assertIsNone(c.tick_targets(s,1.))
        c.action('arm',s,1.); s.dq=[10.]*12
        self.assertEqual(c.tick_targets(s,1.),DEFAULT)
        c.action('estop',None,1.); self.assertIsNone(c.tick_targets(s,1.))

    def test_simulated_joint_loop_computes_and_limits_effort(self):
        p=FakePhysics(); p.velocity=[100.]*12
        a=self.adapter(p,lambda:1.); a.arm()
        command=PositionCommand(a.command_session,0,1.,.05,tuple(NAMES),tuple(DEFAULT))
        a.apply_positions(command,1.)
        self.assertEqual(p.sent[-1],[-60.]*12)
        self.assertNotIn('torque',a.telemetry()['joint_command'])

    def test_watchdog_removes_output_and_latches(self):
        p=FakePhysics(); a=self.adapter(p,lambda:1.); a.arm()
        command=PositionCommand(a.command_session,0,1.,.05,tuple(NAMES),tuple(DEFAULT))
        a.apply_positions(command,1.)
        with self.assertRaisesRegex(ValueError,'deadline'): a.advance(1.051)
        self.assertFalse(a.guard.active); self.assertEqual(a.last_effort,[0.]*12)
        self.assertGreater(p.stops,0); self.assertIsNone(a.telemetry()['joint_command'])
        with self.assertRaises(ValueError): a.arm()

    def test_invalid_batch_does_not_apply_any_joint(self):
        p=FakePhysics(); a=self.adapter(p,lambda:1.); a.arm()
        command=PositionCommand(a.command_session,0,1.,.05,tuple(NAMES),tuple(DEFAULT[:-1]+[float('nan')]))
        with self.assertRaises(ValueError): a.apply_positions(command,1.)
        self.assertEqual(p.sent,[]); self.assertFalse(a.guard.active)

    def test_bad_feedback_latches_adapter_even_without_host_controller(self):
        p=FakePhysics(); a=self.adapter(p,lambda:1.); a.arm()
        command=PositionCommand(a.command_session,0,1.,.05,tuple(NAMES),tuple(DEFAULT))
        a.apply_positions(command,1.)
        p.velocity[-1]=float('nan')
        with self.assertRaises(ValueError): a.advance(1.01)
        self.assertFalse(a.guard.active); self.assertIsNotNone(a.guard.fault)
        with self.assertRaises(ValueError): a.arm()

    def test_worker_default_position_path_never_calls_host_torque(self):
        a=self.adapter(); c=Controller(lambda obs:[0.]*12); w=Worker(a,c)
        w.step(); self.assertIsNone(w.snapshot['joint_command'])
        w.execute({'operation':'arm'},time.monotonic()+1)
        with patch.object(c,'tick',side_effect=AssertionError('host PD used')):
            w.step()
        self.assertEqual(w.snapshot['state'],'STANDBY')
        self.assertEqual(w.snapshot['joint_command']['position'],tuple(DEFAULT))
        w.execute({'operation':'estop'},time.monotonic()+1); w.step()
        self.assertEqual(w.snapshot['state'],'FAULT'); self.assertIsNone(w.snapshot['joint_command'])

    def test_simulated_position_adapter_cannot_wrap_hardware(self):
        p=FakePhysics(); p.hardware=True
        with self.assertRaisesRegex(ValueError,'simulation_only'): self.adapter(p)

    def test_cli_unknown_position_hardware_rejected_before_policy_or_device_load(self):
        script=Path(__file__).resolve().parents[1]/'scripts/quadruped_worker.py'
        r=subprocess.run([sys.executable,str(script),'--source','/no-policy',
                          '--backend','robstride04','--allow-hardware','--token-file','/no-token'],
                         text=True,capture_output=True)
        self.assertEqual(r.returncode,2)
        self.assertIn('simulation adapter only',r.stderr)


if __name__=='__main__': unittest.main()
