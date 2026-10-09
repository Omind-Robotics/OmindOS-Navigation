import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from robot_planning import ProfileRobot, PlanningJobs, PlanningFailure, default_mapping, plan, overlap
from workbench_state import WorkbenchState
import numpy as np


class PlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())

    def test_coupled_path_uses_profile_tcp_and_keeps_base(self):
        base={'x_m':.3,'y_m':-.2,'yaw_rad':.4}
        start={}
        for side,q in [('left',[1.57,1.3,-1.57,1.1,.1,.5,0]),('right',[-1.57,1.3,1.57,1.1,-.1,.5,0])]:
            start.update({f'{side}_joint_{i+1}':v for i,v in enumerate(q)})
        result=plan({'profile':self.profile,'mode':'object','translation_m':[.01,0,0],
                     'base_pose':base,'start':start},lambda:None)
        self.assertGreater(result['validated_samples'],1)
        self.assertLess(result['max_tcp_position_error_m'],.001)
        self.assertLess(result['max_relative_position_error_m'],.002)
        self.assertTrue(all(f['base_pose']==base for f in result['frames']))
        self.assertFalse(result['timed'])

    def test_customer_mount_and_tool_changes_affect_planning(self):
        changed=copy.deepcopy(self.profile)
        # Actual TCP and shoulder geometry are read from the complete URDF.
        import xml.etree.ElementTree as ET
        root=ET.fromstring(changed['kinematics']['urdf_xml'])
        root.find("joint[@name='left_tool_fixed']/origin").set('xyz','0.015 0 -0.24')
        root.find("joint[@name='left_arm_mount']/origin").set('xyz','0 0.12 0.4')
        changed['kinematics']['urdf_xml']=ET.tostring(root,encoding='unicode')
        result=plan({'profile':changed,'mode':'object','translation_m':[.005,0,0]},lambda:None)
        original=ProfileRobot(self.profile,{},None,default_mapping(),[],lambda:None)
        new=ProfileRobot(changed,{},None,default_mapping(),[],lambda:None)
        self.assertGreater(np.linalg.norm(new.tcps(np.zeros(15))[0]-original.tcps(np.zeros(15))[0]),.01)
        self.assertLess(result['max_tcp_position_error_m'],.001)

    def test_obstacle_at_tcp_rejects_path(self):
        r=ProfileRobot(self.profile,{},None,default_mapping(),[],lambda:None)
        tcp=r.tcps(np.zeros(15))[0][:3,3].tolist()
        with self.assertRaises(PlanningFailure):
            plan({'profile':self.profile,'mode':'object','translation_m':[.01,0,0],
                  'obstacles':[{'size_m':[.02,.02,.02],'position_m':tcp}]},lambda:None)

    def test_thin_mid_edge_obstacle_is_sampled(self):
        r=ProfileRobot(self.profile,{},None,default_mapping(),[],lambda:None)
        # A narrow obstacle appears only during a sampled joint edge.
        from robot_planning import joint_path
        class EdgeRobot:
            limits=np.array([[-1,1]]*15)
            last_collision=['arm','thin_obstacle']
            check=staticmethod(lambda:None)
            clear=staticmethod(lambda q:not .045<q[0]<.065)
        # All paths between -0.1 and +0.1 cross this slab. No partial success.
        with self.assertRaises(PlanningFailure):
            joint_path(EdgeRobot(),[-.1]*15,[.1]*15)

    def test_missing_mesh_fails_closed(self):
        p=copy.deepcopy(self.profile)
        p['kinematics']['urdf_xml']=p['kinematics']['urdf_xml'].replace('base_link.STL','missing.STL')
        with self.assertRaises(ValueError):
            ProfileRobot(p,{},None,default_mapping(),[],lambda:None)

    def test_joint_path_uses_limits_and_nonplanning_joint_lock(self):
        r=ProfileRobot(self.profile,{},None,default_mapping(),[],lambda:None)
        goal={**r.start,'waist_pitch':.02}
        result=plan({'profile':self.profile,'mode':'joint','target':goal},lambda:None)
        self.assertEqual(result['frames'][-1]['positions']['waist_pitch'],.02)
        goal['left_outer_finger_joint']=.1
        with self.assertRaises(ValueError):
            plan({'profile':self.profile,'mode':'joint','target':goal},lambda:None)

    def test_mapping_rejects_wrong_tcp_branch(self):
        mapping=default_mapping(); mapping['tcp_links'].reverse()
        with self.assertRaises(ValueError):
            ProfileRobot(self.profile,{},None,mapping,[],lambda:None)

    def test_oriented_boxes_separate_and_contain(self):
        box=(np.zeros(3),np.eye(3),np.ones(3))
        self.assertTrue(overlap(box,(np.zeros(3),np.eye(3),np.full(3,.01))))
        self.assertFalse(overlap(box,(np.array([3,0,0]),np.eye(3),np.ones(3))))

    def test_async_cancel_and_parameter_epoch_invalidate(self):
        with tempfile.TemporaryDirectory() as folder:
            state=WorkbenchState(folder); jobs=PlanningJobs(state)
            task=jobs.submit({'profile':self.profile,'mode':'object','translation_m':[.02,0,0]})
            jobs.cancel(task['task_id'])
            self.assertEqual(jobs.snapshot(task['task_id'])['status'],'canceled')
            time.sleep(.05)
            task=jobs.submit({'profile':self.profile,'mode':'object','translation_m':[.01,0,0]})
            state.set_stop({'engaged':True})
            deadline=time.monotonic()+5
            while jobs.snapshot(task['task_id'])['status']=='running' and time.monotonic()<deadline:
                time.sleep(.02)
            snapshot=jobs.snapshot(task['task_id'])
            self.assertEqual(snapshot['status'],'canceled')
            self.assertIsNone(snapshot['result'])
            with self.assertRaises(ValueError):
                jobs.submit({'profile':self.profile,'mode':'joint'})

    def test_completed_plan_invalid_after_parameter_write(self):
        with tempfile.TemporaryDirectory() as folder:
            state=WorkbenchState(folder); jobs=PlanningJobs(state)
            task=jobs.submit({'profile':self.profile,'mode':'joint','target':{}})
            deadline=time.monotonic()+5
            while jobs.snapshot(task['task_id'])['status']=='running' and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertEqual(jobs.snapshot(task['task_id'])['status'],'succeeded')
            state.apply(self.profile)
            snapshot=jobs.snapshot(task['task_id'])
            self.assertEqual(snapshot['status'],'canceled')
            self.assertIsNone(snapshot['result'])


if __name__=='__main__':unittest.main()
