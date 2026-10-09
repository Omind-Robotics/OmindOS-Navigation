import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from robot_navigation import NavigationSimulation, NavigationJobs, default_config
from robot_planning import PlanningJobs
from workbench_state import WorkbenchState, MotionLocked
from navigation_guard import PlatformGuard, PlatformProfile


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())

    def payload(self,kind='differential',goal=None):
        config=default_config();config['drive_type']=kind
        return {'profile':self.profile,'start':{},'config':config,'goal':goal or {'x_m':.7,'y_m':.4,'yaw_rad':.3}}

    def finish(self,sim):
        seen=[]
        for _ in range(3600):
            sim.step(.05);seen.append(sim.status)
            if sim.status=='parked':break
        self.assertEqual(sim.status,'parked')
        self.assertTrue(sim.parking_confirmed)
        self.assertGreaterEqual(sim.stable,.6)
        self.assertLess(math.hypot(sim.base['x_m']-sim.goal['x_m'],sim.base['y_m']-sim.goal['y_m']),.035)
        self.assertLess(abs(sim.base['yaw_rad']-sim.goal['yaw_rad']),math.radians(2.1))
        self.assertIn('parking',seen)
        return seen

    def test_differential_tracks_and_aligns_with_zero_lateral_command(self):
        sim=NavigationSimulation(self.payload(),lambda:None)
        self.finish(sim)
        self.assertEqual(sim.vy,0)
        self.assertTrue(all(sim.q[n]==sim.robot.start[n] for n in sim.robot.names))

    def test_omni_translates_sideways_without_turning(self):
        sim=NavigationSimulation(self.payload('omnidirectional',{'x_m':0,'y_m':.8,'yaw_rad':0}),lambda:None)
        sideways=False
        for _ in range(3600):
            sim.step(.05);sideways |= abs(sim.vy)>.05
            self.assertAlmostEqual(sim.base['yaw_rad'],0)
            if sim.status=='parked':break
        self.assertTrue(sideways);self.assertTrue(sim.parking_confirmed)

    def test_obstacle_detour_for_both_drive_types(self):
        for kind in ('differential','omnidirectional'):
            data=self.payload(kind,{'x_m':3.2,'y_m':0,'yaw_rad':0})
            data['obstacles']=[{'size_m':[.3,.4,1.2],'position_m':[1.6,0,.6]}]
            sim=NavigationSimulation(data,lambda:None)
            self.assertGreater(max(abs(y) for _,y in sim.path),1.4)
            self.finish(sim)

    def test_blocked_goal_not_cleared_by_inherited_astar(self):
        data=self.payload();data['obstacles']=[{'size_m':[.2,.2,.2],'position_m':[.7,.4,.1]}]
        with self.assertRaises(ValueError):NavigationSimulation(data,lambda:None)

    def test_unknown_map_is_not_traversable(self):
        data=self.payload();data['config']['map']['data']=[-1]*10000
        with self.assertRaises(ValueError):NavigationSimulation(data,lambda:None)

    def test_boundary_respects_whole_robot_envelope(self):
        with self.assertRaises(ValueError):NavigationSimulation(self.payload(goal={'x_m':4.8,'y_m':0,'yaw_rad':0}),lambda:None)

    def test_unreachable_wall_fails_without_partial_route(self):
        data=self.payload(goal={'x_m':3.2,'y_m':0,'yaw_rad':0});data['obstacles']=[{'size_m':[.3,10,1],'position_m':[1.6,0,.5]}]
        with self.assertRaisesRegex(ValueError,'未找到'):NavigationSimulation(data,lambda:None)

    def test_geometry_and_margin_change_footprint(self):
        a=NavigationSimulation(self.payload(),lambda:None)
        data=self.payload();data['config']['safety_margin_m']=.2
        b=NavigationSimulation(data,lambda:None)
        self.assertAlmostEqual(b.radius-a.radius,.12)
        self.assertGreater(a.radius,.8)

    def test_missing_mass_and_motor_data_do_not_block_geometry(self):
        data=self.payload();data['profile']={'kinematics':self.profile['kinematics']}
        sim=NavigationSimulation(data,lambda:None);self.finish(sim)
        self.assertFalse(sim.snapshot()['hardware_executed'])

    def test_precise_goal_required_after_inherited_22cm_tolerance(self):
        sim=NavigationSimulation(self.payload(goal={'x_m':.15,'y_m':0,'yaw_rad':0}),lambda:None)
        self.finish(sim);self.assertGreater(sim.base['x_m'],.11)

    def test_arrival_alone_does_not_confirm_stop(self):
        sim=NavigationSimulation(self.payload(goal={'x_m':0,'y_m':0,'yaw_rad':0}),lambda:None)
        sim.step(.05);self.assertFalse(sim.parking_confirmed)
        for _ in range(11):sim.step(.05)
        self.assertTrue(sim.parking_confirmed)

    def test_collision_during_tracking_stops_before_motion(self):
        sim=NavigationSimulation(self.payload(),lambda:None)
        before=copy.deepcopy(sim.base)
        sim.rect_array[:]=0  # No static rectangles in this map.
        original=sim.edge_clear;sim.edge_clear=lambda a,b:False
        with self.assertRaisesRegex(ValueError,'安全包络'):sim.step(.05)
        self.assertEqual(sim.base,before)

    def test_invalid_map_drive_and_nan_reject(self):
        for change in ({'drive_type':'quadruped'}, {'max_linear_m_s':float('nan')}, {'map':{'size_m':[40,40],'origin_m':[0,0],'resolution_m':.05}}):
            data=self.payload();data['config'].update(change)
            with self.assertRaises(ValueError):NavigationSimulation(data,lambda:None)

    def test_invalid_arm_task_rejects_before_moving(self):
        data=self.payload();data['manipulation']={'translation_m':[1,0,0]}
        with self.assertRaises(ValueError):NavigationSimulation(data,lambda:None)

    def test_original_platform_guard_rejects_stale_feedback(self):
        guard=PlatformGuard(PlatformProfile('test','omnidirectional',.25,.25,.6),enabled=True)
        for source in ('plan','pose','odom'):guard.observe(source,True,1)
        guard.receive_command((.1,.1,0,0,0,0),1)
        self.assertEqual(guard.evaluate(1.1)[0],'READY')
        self.assertEqual(guard.evaluate(2)[1],(0.,)*6)

    def test_cancel_and_estop_never_submit_manipulation(self):
        with tempfile.TemporaryDirectory() as folder:
            state=WorkbenchState(folder);planning=PlanningJobs(state);jobs=NavigationJobs(state,planning)
            data=self.payload();data['manipulation']={'translation_m':[.01,0,0]}
            task=jobs.submit(data)
            with self.assertRaises(MotionLocked):jobs.ensure_idle()
            jobs.cancel(task['task_id']);time.sleep(.1)
            self.assertEqual(jobs.snapshot(task['task_id'])['status'],'canceled');self.assertIsNone(planning.job)
            task=jobs.submit(data);state.set_stop({'engaged':True});time.sleep(.1)
            self.assertEqual(jobs.snapshot(task['task_id'])['status'],'canceled');self.assertIsNone(planning.job)

    def test_real_job_parks_before_actual_runtime2_planning(self):
        with tempfile.TemporaryDirectory() as folder:
            state=WorkbenchState(folder);planning=PlanningJobs(state);jobs=NavigationJobs(state,planning)
            data=self.payload(goal={'x_m':.05,'y_m':0,'yaw_rad':0});data['manipulation']={'translation_m':[.005,0,0]}
            task=jobs.submit(data);deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                result=jobs.snapshot(task['task_id'])
                if planning.job:
                    self.assertTrue(result['navigation']['parking_confirmed'])
                    self.assertGreaterEqual(result['navigation']['stable_stop_s'],.6)
                if result['status'] in {'succeeded','failed','canceled'}:break
                time.sleep(.05)
            self.assertEqual(result['status'],'succeeded',result)
            self.assertTrue(all(f['base_pose']==result['navigation']['base_pose'] for f in result['result']['frames']))
            self.assertFalse(result['result']['hardware_executed'])

    def test_tab_disconnect_aborts_navigation(self):
        with tempfile.TemporaryDirectory() as folder:
            jobs=NavigationJobs(WorkbenchState(folder),PlanningJobs(WorkbenchState(folder)))
            task=jobs.submit(self.payload());time.sleep(3.5)
            result=jobs.snapshot(task['task_id'])
            self.assertEqual(result['status'],'failed');self.assertIn('连接中断',result['error'])


if __name__=='__main__':unittest.main()
