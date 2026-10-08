import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from robot_parameter_api import register_robot_parameters
from robot_profile_runtime_core import ProfileRuntime,RuntimeErrorResponse
from robot_profile_runtime import RuntimeMailbox
from robot_runtime_client import RuntimeClient


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.now=10.0
        self.profile=json.loads((Path(__file__).resolve().parents[1]/'config/robot_profiles/workbench-demo.json').read_text())
        self.id=register_robot_parameters(self.profile,self.folder.name)['profile_id']
        self.core=ProfileRuntime(self.folder.name,lambda:self.now)

    def load(self):
        result=self.core.command('load',{'profile_id':self.id})
        self.token=result['lease_token']
        return result

    def start_preview(self,duration=3):
        target=self.core.q.copy(); target['left_elbow']=.4
        return self.core.command('run',{'profile_id':self.id,'lease_token':self.token,'target':target,'duration_s':duration})

    def heartbeat(self):
        self.core.command('heartbeat',{'lease_token':self.token})

    def test_explicit_load_real_joint_positions_motor_mapping_and_restart(self):
        self.assertEqual(self.core.snapshot()['state'],'UNLOADED')
        result=self.load()
        self.assertTrue(result['runtime_applied']); self.assertFalse(result['hardware_active'])
        self.start_preview()
        self.now+=1; self.heartbeat(); self.core.tick()
        result=self.core.snapshot()
        self.assertAlmostEqual(result['pose']['positions']['left_elbow'],.4*(1/3)**2*(3-2/3))
        drive=self.profile['dynamics']['joints']['left_elbow']
        self.assertAlmostEqual(result['motor_positions_rad']['left_elbow'],(self.core.q['left_elbow']-drive['zero_offset_rad'])*drive['reduction_ratio']/drive['direction'])
        self.assertNotEqual(result['pose']['joint_positions']['left_wrist'],self.load_pose()['joint_positions']['left_wrist'])
        restarted=ProfileRuntime(self.folder.name)
        self.assertEqual(restarted.snapshot()['state'],'UNLOADED')
        self.assertFalse(restarted.snapshot()['runtime_applied'])

    def load_pose(self):
        other=ProfileRuntime(self.folder.name)
        return other.command('load',{'profile_id':self.id})['pose']

    def test_load_during_run_rejected_then_stop_holds_pose(self):
        self.load(); self.start_preview(); self.now+=.7; self.heartbeat()
        with self.assertRaisesRegex(RuntimeErrorResponse,'stop the preview'):
            self.core.command('load',{'profile_id':self.id})
        self.core.command('stop',{'lease_token':self.token})
        q=self.core.q.copy(); elapsed=self.core.snapshot()['elapsed_s']
        self.now+=20; self.core.tick()
        self.assertEqual(self.core.q,q); self.assertEqual(self.core.snapshot()['elapsed_s'],elapsed)
        self.assertTrue(all(v==0 for v in self.core.velocity.values()))

    def test_heartbeat_loss_holds_and_does_not_resume_automatically(self):
        self.load(); self.start_preview(); self.now+=.5; self.core.tick(); before=self.core.q.copy()
        self.now+=3; self.core.tick()
        self.assertEqual(self.core.state,'STOPPED'); self.assertEqual(self.core.q,before)
        self.heartbeat(); self.core.tick(); self.assertEqual(self.core.state,'STOPPED')

    def test_completion_and_velocity_limit_rejection(self):
        self.load()
        with self.assertRaisesRegex(ValueError,'velocity limits'): self.start_preview(.01)
        self.assertEqual(self.core.state,'READY')
        self.start_preview(2)
        for _ in range(4): self.now+=.5; self.heartbeat()
        self.assertEqual(self.core.state,'READY')
        self.assertAlmostEqual(self.core.q['left_elbow'],.4)
        self.assertTrue(all(abs(v)<1e-12 for v in self.core.velocity.values()))

    def test_different_version_and_session_require_reload(self):
        self.load(); old_token=self.token
        changed=copy.deepcopy(self.profile); changed['robot_id']='robot-b'
        second=register_robot_parameters(changed,self.folder.name)['profile_id']
        with self.assertRaisesRegex(RuntimeErrorResponse,'differs'):
            self.core.command('run',{'profile_id':second,'lease_token':self.token,'target':self.core.q,'duration_s':2})
        result=self.core.command('load',{'profile_id':second})
        self.assertNotEqual(result['lease_token'],old_token)
        with self.assertRaisesRegex(RuntimeErrorResponse,'session changed'):
            self.core.command('stop',{'lease_token':old_token})

    def test_missing_or_tampered_profile_does_not_replace_loaded_version(self):
        self.load()
        with self.assertRaises(RuntimeErrorResponse):
            self.core.command('load',{'profile_id':'0'*64})
        self.assertEqual(self.core.profile_id,self.id)
        path=Path(self.folder.name)/(self.id+'.json')
        changed=copy.deepcopy(self.profile); changed['robot_id']='tampered'; path.write_text(json.dumps(changed))
        with self.assertRaisesRegex(ValueError,'checksum'): self.start_preview()
        self.assertEqual(self.core.state,'READY')

    def test_target_names_and_values_validated(self):
        self.load()
        for target in ({},{'bad_joint':0},{**self.core.q,'left_elbow':float('nan')},{**self.core.q,'left_elbow':100}):
            with self.assertRaises(ValueError):
                self.core.command('run',{'profile_id':self.id,'lease_token':self.token,'target':target,'duration_s':2})
        self.assertEqual(self.core.state,'READY')

    def test_expired_http_command_never_executes_late(self):
        mailbox=RuntimeMailbox()
        code,_=mailbox.submit('load',{'profile_id':self.id},timeout=.01)
        self.assertEqual(code,503)
        mailbox.process(self.core)
        self.assertEqual(self.core.state,'UNLOADED')

    def test_bridge_rejects_remote_urls_and_embedded_credentials(self):
        for url in ('https://127.0.0.1:8086','http://example.com:8086','http://127.0.0.1:8086/x','http://a@localhost:8086','http://localhost:8086?x=1'):
            with self.assertRaises(ValueError): RuntimeClient(url)


if __name__=='__main__': unittest.main()
