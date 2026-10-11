import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import urllib.request
import urllib.error
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from robot_kinematics import build_model, pose, trial, transform, jog_trial, sine_trial, wheel_trial
from robot_parameter_api import validate_parameters
from robot_workbench import ThreadingHTTPServer, make_handler
from workbench_state import WorkbenchState, MotionLocked
from unittest.mock import patch


def fixture():
    def body(name, mass, com):
        return f'<link name="{name}"><inertial><origin xyz="{com}"/><mass value="{mass}"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>'
    xml = ('<robot name="pendulum">' + body('base', 1, '0 0 0') + body('arm', 2, '.5 0 0') + body('tip', 1, '0 0 0') +
           '<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="50" velocity="2"/></joint>' +
           '<joint name="tip_fixed" type="fixed"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/></joint></robot>')
    return {'robot_id':'pendulum','kinematics':{'urdf_xml':xml},'dynamics':{'joints':{'hinge':{
        'motor_model':'synthetic','reduction_ratio':10,'direction':-1,'zero_offset_rad':.1,'torque_limit_nm':40,'velocity_limit_rad_s':1}}}}


class KinematicsTests(unittest.TestCase):
    def test_analytic_pendulum_pose_and_static_gravity(self):
        model = build_model(fixture()['kinematics']['urdf_xml'])
        flat = pose(model, {'hinge':0})
        self.assertAlmostEqual(flat['link_transforms']['tip'][0][3], 1)
        self.assertAlmostEqual(flat['holding_torque_nm']['hinge'], -19.6133)
        upright = pose(model, {'hinge':math.pi/2})
        self.assertAlmostEqual(upright['link_transforms']['tip'][0][3], 0)
        self.assertAlmostEqual(upright['link_transforms']['tip'][2][3], -1)
        self.assertAlmostEqual(upright['holding_torque_nm']['hinge'], 0)

    def test_changed_joint_origin_and_mass_change_actual_calculation(self):
        data = fixture()
        data['kinematics']['urdf_xml'] = data['kinematics']['urdf_xml'].replace('xyz="1 0 0"','xyz="2 0 0"').replace('mass value="2"','mass value="4"')
        result = pose(build_model(data['kinematics']['urdf_xml']), {})
        self.assertAlmostEqual(result['link_transforms']['tip'][0][3], 2)
        self.assertAlmostEqual(result['holding_torque_nm']['hinge'], -39.2266)

    def test_origin_rpy_rotates_joint_axis_before_joint_motion(self):
        xml = fixture()['kinematics']['urdf_xml'].replace('<axis xyz="0 1 0"/>', f'<origin xyz="1 2 3" rpy="0 0 {math.pi/2}"/><axis xyz="0 0 2"/>')
        result = pose(build_model(xml), {'hinge':math.pi/2})
        point = [result['link_transforms']['tip'][i][3] for i in range(3)]
        for got, expected in zip(point, [0,2,3]):
            self.assertAlmostEqual(got, expected)

    def test_trial_limits_motor_mapping_and_scoped_results(self):
        result = trial({'profile':fixture(),'target':{'hinge':1},'duration_s':2})
        self.assertAlmostEqual(result['peak_joint_velocity_rad_s']['hinge'], .75)
        self.assertAlmostEqual(result['frames'][0]['positions']['hinge'], 0)
        self.assertAlmostEqual(result['frames'][-1]['positions']['hinge'], 1)
        self.assertAlmostEqual(result['frames'][-1]['motor_positions_rad']['hinge'], -9)
        self.assertFalse(result['runtime_applied'])
        with self.assertRaisesRegex(ValueError, 'velocity'):
            trial({'profile':fixture(),'target':{'hinge':1},'duration_s':1})
        with self.assertRaisesRegex(ValueError, 'limits'):
            trial({'profile':fixture(),'target':{'hinge':3}})
        with self.assertRaises(ValueError):
            trial({'profile':fixture(),'target':{'unknown':1}})

    def test_gravity_overload_is_reported(self):
        data = fixture(); data['dynamics']['joints']['hinge']['torque_limit_nm'] = 1
        self.assertEqual(trial({'profile':data})['gravity_limit_exceeded'], ['hinge'])

    def test_malformed_models_rejected_without_external_resource_loading(self):
        original = fixture()['kinematics']['urdf_xml']
        for xml in [original.replace('type="revolute"','type="prismatic"'), original.replace('xyz="1 0 0"','xyz="nan 0 0"'),
                    '<!DOCTYPE robot>'+original, original.replace('name="arm"','name="base"'), original.replace('child link="tip"','child link="base"')]:
            with self.subTest(xml=xml), self.assertRaises(ValueError):
                build_model(xml)
        model = build_model(original.replace('</link>', '<visual><geometry><mesh filename="http://invalid/never-fetch.stl"/></geometry></visual></link>', 1))
        self.assertTrue(model['warnings'])

    def test_demo_configuration_is_valid_and_full_trial_runs(self):
        data = json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())
        validate_parameters(data)
        result = trial({'profile':data,'target':{'left_joint_1':-.5,'left_joint_2':-.7},'duration_s':2})
        self.assertEqual(len(result['model']['links']), 32)
        self.assertEqual(len(result['frames'][-1]['positions']), 23)

    def test_each_rm75_chain_preserves_seven_manufacturer_axes_and_inertias(self):
        source = ET.parse(ROOT/'config/robot_models/realman_rm75/RM75-B.urdf').getroot()
        demo = ET.fromstring(json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())['kinematics']['urdf_xml'])
        for side in ('left', 'right'):
            arm = [j for j in demo.findall('joint') if j.get('name').startswith(side+'_joint_')]
            self.assertEqual(len(arm), 7)
            for i in range(1, 8):
                original = source.find(f"joint[@name='joint_{i}']")
                actual = demo.find(f"joint[@name='{side}_joint_{i}']")
                for tag in ('origin', 'axis', 'limit'):
                    self.assertEqual(actual.find(tag).attrib, original.find(tag).attrib)
                for tag in ('parent', 'child'):
                    self.assertEqual(actual.find(tag).get('link'), side+'_'+original.find(tag).get('link'))
                actual_link = demo.find(f"link[@name='{side}_link_{i}']")
                original_link = source.find(f"link[@name='link_{i}']")
                for tag in ('mass', 'inertia', 'origin'):
                    self.assertEqual(actual_link.find('inertial/'+tag).attrib, original_link.find('inertial/'+tag).attrib)
        body = [j for j in demo.findall('joint') if j.get('type') != 'fixed' and ('_joint_' in j.get('name') or j.get('name') == 'waist_pitch')]
        self.assertEqual(len(body), 15)
        self.assertIsNone(demo.find("joint[@name='waist']"))
        self.assertEqual(demo.find("joint[@name='head_mount']").get('type'), 'fixed')

    def test_planar_base_transforms_entire_tree_without_changing_gravity(self):
        model = build_model(fixture()['kinematics']['urdf_xml'])
        original = pose(model, {})
        moved = pose(model, {}, {'x_m':2, 'y_m':-1, 'yaw_rad':math.pi/2})
        tip = transform(moved['link_transforms']['tip'], [0,0,0])
        for got, expected in zip(tip, [2,0,0]):
            self.assertAlmostEqual(got, expected)
        self.assertAlmostEqual(moved['holding_torque_nm']['hinge'], original['holding_torque_nm']['hinge'])
        result = trial({'profile':fixture(), 'base_start':{'x_m':-.2},
                        'base_target':{'x_m':.4, 'yaw_rad':.4}, 'duration_s':2})
        self.assertEqual(result['frames'][0]['base_pose']['x_m'], -.2)
        self.assertEqual(result['frames'][-1]['base_pose'], {'x_m':.4,'y_m':0,'yaw_rad':.4})
        self.assertAlmostEqual(result['peak_base_velocity']['linear_m_s'], .45)
        self.assertIn('prescribed_planar', result['scope'])
        with self.assertRaisesRegex(ValueError, 'velocity'):
            trial({'profile':fixture(), 'base_target':{'x_m':1}, 'duration_s':1})

    def test_base_pose_rejects_invalid_values(self):
        model = build_model(fixture()['kinematics']['urdf_xml'])
        for value in [[], {'z_m':1}, {'x_m':101}, {'y_m':float('nan')}, {'yaw_rad':7}]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                pose(model, {}, value)
        with self.assertRaisesRegex(ValueError, 'rgba'):
            build_model(fixture()['kinematics']['urdf_xml'].replace('</link>',
                '<visual><geometry><sphere radius=".1"/></geometry><material><color rgba="1 2 0 1"/></material></visual></link>',1))

    def test_grippers_and_pitch_move_downstream_but_leave_chassis_fixed(self):
        data = json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())
        model = build_model(data['kinematics']['urdf_xml'])
        closed = pose(model, {})
        opened = pose(model, {'left_outer_finger_joint':.65, 'left_inner_finger_joint':.65})
        def gap(frame):
            points = [transform(frame['link_transforms']['left_'+name+'_finger'], [0,0,-.1]) for name in ('outer','inner')]
            return math.dist(*points)
        self.assertGreater(gap(opened), gap(closed)+.1)
        self.assertEqual(opened['link_transforms']['right_hand'], closed['link_transforms']['right_hand'])
        pitched = pose(model, {'waist_pitch':.3})
        self.assertEqual(pitched['link_transforms']['base'], closed['link_transforms']['base'])
        self.assertEqual(pitched['link_transforms']['left_front_wheel'], closed['link_transforms']['left_front_wheel'])
        self.assertNotEqual(pitched['link_transforms']['left_hand'], closed['link_transforms']['left_hand'])
        self.assertNotEqual(pitched['link_transforms']['head'], closed['link_transforms']['head'])
        joint = next(j for j in model['joints'] if j['name']=='waist_pitch')
        self.assertEqual(joint['axis'], [0,1,0])


class WorkbenchHTTPTests(unittest.TestCase):
    def test_navigation_heartbeat_http_endpoint_rejects_unknown_task_and_origin(self):
        with tempfile.TemporaryDirectory() as folder:
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(folder))
            runner=threading.Thread(target=server.serve_forever,daemon=True);runner.start()
            try:
                url=f'http://127.0.0.1:{server.server_port}/v1/navigation-heartbeat/unknown'
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    urllib.request.urlopen(url,timeout=5)
                self.assertEqual(caught.exception.code,404)
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    urllib.request.urlopen(urllib.request.Request(url,headers={'Origin':'https://untrusted.example'}),timeout=5)
                self.assertEqual(caught.exception.code,403)
            finally:
                server.shutdown();runner.join(timeout=5);server.server_close()

    def test_real_http_model_trial_version_save_reload_and_origin_controls(self):
        with tempfile.TemporaryDirectory() as store:
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(store)); thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            base=f'http://127.0.0.1:{server.server_port}'
            def request(path,data=None,headers=None):
                h={'Content-Type':'application/json',**(headers or {})}
                return urllib.request.urlopen(urllib.request.Request(base+path,data=None if data is None else json.dumps(data).encode(),headers=h),timeout=5)
            try:
                with request('/') as r:
                    self.assertIn('机器人调参工作台',r.read().decode());self.assertIn("frame-ancestors 'none'",r.headers['Content-Security-Policy'])
                data=fixture()
                with request('/v1/robot-model',{'urdf_xml':data['kinematics']['urdf_xml'], 'base_pose':{'x_m':.2}}) as r:
                    response = json.load(r)
                    self.assertEqual(response['model']['root'],'base')
                    self.assertEqual(response['pose']['link_transforms']['base'][0][3], .2)
                with request('/v1/robot-preview',{'profile':data,'target':{'hinge':.5}}) as r:
                    self.assertEqual(json.load(r)['frames'][-1]['positions']['hinge'],.5)
                with request('/v1/robot-profiles',data) as r:
                    one=json.load(r)['profile_id']
                changed=copy.deepcopy(data);changed['robot_id']='updated'
                with request('/v1/robot-profiles',changed) as r:
                    self.assertNotEqual(json.load(r)['profile_id'],one)
                with request('/v1/robot-profiles/'+one) as r:
                    self.assertEqual(json.load(r),data)
                for headers in [{'Origin':'https://example.com'},{'Host':'malicious.example'},{'Content-Type':'text/plain'}]:
                    with self.subTest(headers=headers),self.assertRaises(urllib.error.HTTPError) as caught:
                        request('/v1/robot-profiles',data,headers)
                    self.assertIn(caught.exception.code,(403,415))
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    request('/v1/robot-preview',{'profile':data,'target':{'hinge':99}})
                self.assertEqual(caught.exception.code,400)
            finally:
                server.shutdown();thread.join(timeout=5);server.server_close()


class MotionControlsTests(unittest.TestCase):
    def test_jog_is_exact_tenth_degree_and_leaves_base_unchanged(self):
        data = fixture()
        result = jog_trial({'profile':data, 'start':{'hinge':.2}, 'joint':'hinge', 'direction':1, 'base_pose':{'x_m':.4}})
        end = result['frames'][-1]
        self.assertAlmostEqual(end['positions']['hinge']-.2, math.pi/1800, places=14)
        self.assertEqual(end['base_pose']['x_m'], .4)
        reverse = jog_trial({'profile':data, 'start':end['positions'], 'joint':'hinge', 'direction':-1})
        self.assertAlmostEqual(reverse['frames'][-1]['positions']['hinge'], .2)
        with self.assertRaisesRegex(ValueError, 'limits'):
            jog_trial({'profile':data, 'start':{'hinge':2}, 'joint':'hinge', 'direction':1})
        for name, direction in [('tip_fixed',1),('hinge',True),('hinge',0)]:
            with self.assertRaises(ValueError):
                jog_trial({'profile':data, 'joint':name, 'direction':direction})

    def test_sine_matches_analytic_extrema_and_returns_to_start(self):
        result = sine_trial({'profile':fixture(),'start':{'hinge':.3},'joints':['hinge'],'amplitude_deg':10,'period_s':2,'cycles':2})
        frames = result['frames']; quarter = (len(frames)-1)//8
        self.assertAlmostEqual(frames[quarter]['positions']['hinge'], .3+math.radians(10))
        self.assertAlmostEqual(frames[3*quarter]['positions']['hinge'], .3-math.radians(10))
        self.assertEqual(frames[0]['positions'], frames[-1]['positions'])
        self.assertAlmostEqual(result['peak_joint_velocity_rad_s']['hinge'], math.pi*math.radians(10))
        self.assertFalse(result['runtime_applied'])

    def test_sine_holds_unselected_joints_and_clamps_amplitude_at_limits(self):
        data = json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())
        result = sine_trial({'profile':data,'start':{'left_joint_2':2.2589,'right_joint_2':.3},'joints':['left_joint_2'],'amplitude_deg':5,'period_s':2,'cycles':1})
        self.assertEqual(result['amplitude_limited_joints'], ['left_joint_2'])
        self.assertAlmostEqual(result['actual_amplitude_deg']['left_joint_2'], math.degrees(.01))
        for frame in result['frames']:
            self.assertLessEqual(frame['positions']['left_joint_2'],2.2689)
            self.assertEqual(frame['positions']['right_joint_2'],.3)
            self.assertEqual(frame['base_pose'], {'x_m':0,'y_m':0,'yaw_rad':0})

    def test_sine_rejects_bad_groups_parameters_and_excess_speed(self):
        base = {'profile':fixture(),'joints':['hinge']}
        for extra in [{'joints':[]},{'joints':['hinge','hinge']},{'joints':['tip_fixed']},
                      {'amplitude_deg':float('nan')},{'period_s':.4},{'cycles':1.5},{'cycles':True},
                      {'cycles':10,'period_s':30},{'start':{'hinge':2}}]:
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                sine_trial({**base, **extra})
        with self.assertRaisesRegex(ValueError,'velocity'):
            sine_trial({**base,'amplitude_deg':45,'period_s':.5})

    def test_parameter_application_survives_restart_and_invalid_apply_preserves_version(self):
        with tempfile.TemporaryDirectory() as folder:
            state=WorkbenchState(folder);data=fixture();saved=state.apply(data)
            self.assertTrue(saved['workbench_applied']);self.assertFalse(saved['runtime_applied'])
            restarted=WorkbenchState(folder)
            self.assertEqual(restarted.active_profile(),data)
            self.assertEqual(restarted.profile_id,saved['profile_id'])
            bad=copy.deepcopy(data);bad['dynamics']['joints']['hinge']['velocity_limit_rad_s']=-1
            with self.assertRaises(ValueError):
                state.apply(bad)
            self.assertEqual(WorkbenchState(folder).active_profile(),data)

    def test_stop_latches_and_stale_reset_or_motion_cannot_resume(self):
        with tempfile.TemporaryDirectory() as folder:
            state=WorkbenchState(folder);epoch=state.check_motion()
            stopped=state.set_stop({'engaged':True})
            with self.assertRaises(MotionLocked):state.check_motion()
            with self.assertRaises(MotionLocked):state.set_stop({'engaged':False,'expected_epoch':epoch})
            state.set_stop({'engaged':False,'expected_epoch':stopped['motion_epoch']})
            with self.assertRaises(MotionLocked):state.check_motion(epoch)
            self.assertFalse(state.snapshot()['emergency_stop'])

    def test_http_stop_interrupts_inflight_calculation_and_blocks_all_motion_routes(self):
        with tempfile.TemporaryDirectory() as folder:
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(folder));runner=threading.Thread(target=server.serve_forever,daemon=True);runner.start()
            base=f'http://127.0.0.1:{server.server_port}'
            def request(path, data=None):
                try:
                    with urllib.request.urlopen(urllib.request.Request(base+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'}),timeout=5) as response:
                        return response.status,json.load(response)
                except urllib.error.HTTPError as e:return e.code,json.load(e)
            entered, release=threading.Event(), threading.Event();response=[]
            def delayed(payload):
                entered.set();release.wait(4);return trial(payload)
            try:
                self.assertEqual(request('/v1/workbench-active-profile',fixture())[0],200)
                with patch('robot_workbench.trial',delayed):
                    pending=threading.Thread(target=lambda:response.append(request('/v1/robot-preview',{'target':{'hinge':.1}})))
                    pending.start();self.assertTrue(entered.wait(2))
                    code,state=request('/v1/workbench-stop',{'engaged':True});self.assertEqual(code,200)
                    release.set();pending.join(5)
                self.assertEqual(response[0][0],423)
                for route in ('/v1/robot-preview','/v1/robot-jog','/v1/robot-sine','/v1/robot-wheel-base'):
                    self.assertEqual(request(route,{})[0],423)
                self.assertTrue(request('/v1/workbench-active-profile')[1]['emergency_stop'])
                self.assertEqual(request('/v1/workbench-stop',{'engaged':False,'expected_epoch':state['motion_epoch']})[0],200)
                self.assertEqual(request('/v1/robot-jog',{'joint':'hinge','direction':1})[0],200)
            finally:
                release.set();server.shutdown();runner.join(5);server.server_close()


class WheelBaseTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())

    def run_drive(self, **kwargs):
        return wheel_trial({'profile': self.data, 'duration_s': 2, **kwargs})

    def test_straight_motion_matches_wheel_circumference_and_starts_stops_at_rest(self):
        for speed in (.2, -.2):
            result = self.run_drive(linear_m_s=speed, start={'left_joint_2': .3})
            end = result['frames'][-1]
            self.assertAlmostEqual(end['base_pose']['x_m'], speed)
            self.assertAlmostEqual(end['base_pose']['y_m'], 0)
            for name in result['model']['wheel_base']['wheels']:
                self.assertAlmostEqual(end['positions'][name], speed/.1)
                self.assertEqual(result['frames'][0]['wheel_velocity_rad_s'][name], 0)
                self.assertEqual(end['wheel_velocity_rad_s'][name], 0)
            for frame in result['frames']:
                self.assertEqual(frame['positions']['left_joint_2'], .3)
            self.assertAlmostEqual(result['peak_base_velocity']['linear_m_s'], abs(speed))

    def test_turning_signs_track_width_and_nonzero_start_pose(self):
        for omega in (.4, -.4):
            result = self.run_drive(angular_rad_s=omega, base_pose={'x_m': .3, 'y_m': -.4, 'yaw_rad': .2})
            end = result['frames'][-1]
            self.assertEqual(end['base_pose']['x_m'], .3)
            self.assertEqual(end['base_pose']['y_m'], -.4)
            self.assertAlmostEqual(end['base_pose']['yaw_rad'], .2+omega)
            self.assertAlmostEqual(end['positions']['left_front_wheel_joint'], -omega*.295/.1)
            self.assertAlmostEqual(end['positions']['right_rear_wheel_joint'], omega*.295/.1)

    def test_arc_odometry_and_heading_are_not_world_axis_translation(self):
        end = self.run_drive(linear_m_s=.2, angular_rad_s=.4)['frames'][-1]
        self.assertAlmostEqual(end['base_pose']['x_m'], .5*math.sin(.4))
        self.assertAlmostEqual(end['base_pose']['y_m'], .5*(1-math.cos(.4)))
        end = self.run_drive(linear_m_s=.2, base_pose={'yaw_rad': math.pi/2})['frames'][-1]
        self.assertAlmostEqual(end['base_pose']['x_m'], 0)
        self.assertAlmostEqual(end['base_pose']['y_m'], .2)

    def test_changed_radius_track_and_axis_sign_change_wheel_rates(self):
        root = ET.fromstring(self.data['kinematics']['urdf_xml'])
        root.find("link[@name='left_front_wheel']/visual/geometry/cylinder").set('radius', '.05')
        root.find("joint[@name='right_front_wheel_joint']/axis").set('xyz', '0 -1 0')
        for j in root.findall('joint'):
            if '_wheel_joint' in j.get('name'):
                xyz = j.find('origin').get('xyz').split();xyz[1] = '.4' if j.get('name').startswith('left') else '-.4'
                j.find('origin').set('xyz', ' '.join(xyz))
        self.data['kinematics']['urdf_xml'] = ET.tostring(root, encoding='unicode')
        end = self.run_drive(linear_m_s=.1, angular_rad_s=.25)['frames'][-1]
        self.assertAlmostEqual(end['positions']['left_front_wheel_joint'], 0)
        self.assertAlmostEqual(end['positions']['right_front_wheel_joint'], -2)
        self.assertAlmostEqual(end['positions']['right_rear_wheel_joint'], 2)
        straight = self.run_drive(linear_m_s=.1)['frames'][-1]
        self.assertAlmostEqual(straight['positions']['left_front_wheel_joint'], 2)

    def test_limits_and_unsupported_base_rejected_before_playback(self):
        for extra in ({'linear_m_s': .6}, {'angular_rad_s': .9}, {'duration_s': .1},
                      {'duration_s': 11}, {'linear_m_s': float('nan')},
                      {'linear_m_s': .5, 'angular_rad_s': .8},
                      {'linear_m_s': .2, 'base_pose': {'x_m': 4.99}}):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                self.run_drive(**extra)
        with self.assertRaisesRegex(ValueError, '未配置'):
            wheel_trial({'profile': fixture()})
        root = ET.fromstring(self.data['kinematics']['urdf_xml'])
        root.find('omindos_base').set('type', 'mecanum')
        with self.assertRaises(ValueError):
            build_model(ET.tostring(root, encoding='unicode'))

    def test_wheel_configuration_is_preserved_on_apply_and_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            state = WorkbenchState(folder);state.apply(self.data)
            restored = WorkbenchState(folder).active_profile()
            self.assertEqual(wheel_trial({'profile': restored, 'linear_m_s': .1})['model']['wheel_base']['track_width_m'], .59)


if __name__=='__main__':
    unittest.main()

