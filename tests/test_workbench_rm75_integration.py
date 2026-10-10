"""Real CAD-derived collision/joint fixtures; not a hardware acceptance test."""
import base64
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import robot_assets
import robot_model_bundle as bundle
import robot_kinematic_preview as preview
import robot_parameter_api as registry
from robot_collision_frames import frame_only_links
from robot_kinematics import build_model,pose
from robot_planning import ProfileRobot,plan,PlanningFailure
from robot_workbench import make_handler
from workbench_state import _register_workbench_profile,_load_workbench_profile,WorkbenchState

PART_HASHES = [
'7b838c185fae61ca8cf6987e3c2ddba634b9c914e3d8c98de801bd57c0dfff3a',
'fa6486350e13c268a41f65994bc7f6438a814225c4c438e11f0917774e37a87b',
'e8fc44dd7ed43037488e62c39e8e4c0cafb311aaeb82ea574ebe1b15f5cee8bd',
'2002048eef32b8ea2c556a6398ce73e52a104fe9e9fc7df6889ccb07939aac6e',
'b0fa18cbf73e0f2a8f91fe4fa1b26ada993864ec8f226cf2f6001606e236a974',
'072b539e04711f5481584874ed2b9e2aa96ef0ffd37c5d74b84d3552ce0befb7',
'0a5a98e1457c2d9d78086c9242e031c79b86a9b3ffccf34a1f8d330f8306dca8',
'5ea7fc19ba638f20afaccb42c3f8684863e4f3ff62225c06b117b189c9e78bd4',
'14a88d7314f43e7d7932d8e57731635336d18ec96f8b252ed00f3f944b6fa171',
'50db852915757e2f8e5bfd0497f1e29bd7c5028497b6edb19b4898c57129c205',
'5a42eeee2a57ea0dca5e18afc7b047860bc799a94bfc86acbdfe6abaabe4c1dc',
'2be152936073bbd8522ee2ee219e4c7f146d0c892e30989396d3b19fcd269749',
'05032afbbad4862b6dd006288b8d14d75d582afeb0477be066140caad192c518',
'c73f9179b8c44a523d54eba90c41db9520fa802b4016e44c7909ac8864610319']
parts=[]
for i,expected in enumerate(PART_HASHES):
    part=(Path(__file__).parent/'rm75_fixture_parts'/f'{i:02d}.part').read_text().strip()
    if hashlib.sha256(part.encode()).hexdigest()!=expected:
        raise ValueError(f'geometry fixture transfer integrity failed for part {i:02d}')
    parts.append(part)
RAW=gzip.decompress(base64.b64decode(''.join(parts),validate=True))
assert hashlib.sha256(RAW).hexdigest()=='ce02d2d439c579030a11711a35b21a445b8d870f6be9fb6421af1ed576b7e232'
FIXTURE=json.loads(RAW)


def profile(variant='b'):
    return {'robot_id':'rm75-integration-'+variant,'profile_mode':'kinematic_preview',
            'kinematics':{'urdf_xml':FIXTURE['profiles'][variant]}}


def robot(p=None,start=None,obstacles=None):
    return ProfileRobot(p or profile(),start or {},None,FIXTURE['mapping'],obstacles or [],lambda:None)


class RM75IntegrationTests(unittest.TestCase):
    def test_three_actual_collision_models_are_within_budget(self):
        for variant in ('b','6fb','6f'):
            r=robot(profile(variant))
            self.assertEqual(len(r.shapes),64)
            self.assertEqual(r.frame_only_links,{'left_tool0','right_tool0'})
            for name,q in FIXTURE['poses'].items():
                with self.subTest(variant=variant,pose=name):
                    self.assertTrue(r.clear([q[n]for n in r.names]),r.last_collision)

    def test_each_of_fifteen_actual_joints_jogs_without_motor_or_torque_values(self):
        for variant in ('b','6fb','6f'):
            p=profile(variant);original=copy.deepcopy(p);q=FIXTURE['poses']['preview']
            for name in q:
                with self.subTest(variant=variant,joint=name):
                    out=preview.jog({'profile':p,'start':q,'joint':name,'direction':1})
                    end=out['frames'][-1]
                    self.assertAlmostEqual(end['positions'][name]-q[name],math.radians(.1),places=12)
                    self.assertTrue(all(end['positions'][n]==q[n]for n in q if n!=name))
                    self.assertEqual(end['base_pose'],{'x_m':0,'y_m':0,'yaw_rad':0})
                    self.assertTrue(all(f['motor_positions_rad']is None and f['holding_torque_nm']is None for f in out['frames']))
                    self.assertFalse(out['collision_validated']);self.assertFalse(out['hardware_validated'])
            self.assertEqual(p,original)

    def test_actual_joint_path_is_checked_by_existing_planner(self):
        for variant in ('b','6fb','6f'):
            start=FIXTURE['poses']['preview'];target={**start,'left_joint_7':start['left_joint_7']+.01}
            out=plan({'profile':profile(variant),'start':start,'target':target,'mapping':FIXTURE['mapping'],'mode':'joint'},lambda:None)
            self.assertGreaterEqual(out['validated_samples'],2)
            self.assertAlmostEqual(out['frames'][-1]['positions']['left_joint_7'],target['left_joint_7'])

    def test_obstacle_intersecting_real_link_blocks_joint_plan(self):
        start=FIXTURE['poses']['preview'];r=robot(start=start)
        name,local,half=next(s for s in r.shapes if s[0]=='left_link_7')
        T=r.world([start[n]for n in r.names])[name]@local
        obstacles=[{'size_m':[.03,.03,.03],'position_m':T[:3,3].tolist()}]
        with self.assertRaises((PlanningFailure,ValueError)):
            plan({'profile':profile(),'start':start,'target':start,'mapping':FIXTURE['mapping'],
                  'mode':'joint','obstacles':obstacles},lambda:None)

    def test_missing_physical_geometry_is_still_rejected(self):
        p=profile();t=ET.fromstring(p['kinematics']['urdf_xml']);link=t.find("link[@name='left_link_3']")
        for c in link.findall('collision'):link.remove(c)
        p['kinematics']['urdf_xml']=ET.tostring(t,encoding='unicode')
        with self.assertRaises(ValueError):robot(p)

    def test_fake_frame_declarations_cannot_skip_body_or_moving_links(self):
        for name in ('base_link','left_link_3','left_link_7'):
            t=ET.fromstring(profile()['kinematics']['urdf_xml']);ET.SubElement(t,'omindos_frame_only',link=name)
            with self.subTest(name=name),self.assertRaises(ValueError):
                frame_only_links(t,['left_tool0','right_tool0',name])
        t=ET.fromstring(profile()['kinematics']['urdf_xml']);leaf=t.find("link[@name='left_tool0']")
        ET.SubElement(leaf,'inertial')
        with self.assertRaises(ValueError):frame_only_links(t,['left_tool0','right_tool0'])

    def test_duplicate_or_undeclared_frame_is_not_silently_accepted(self):
        p=profile();t=ET.fromstring(p['kinematics']['urdf_xml'])
        ET.SubElement(t,'omindos_frame_only',link='left_tool0')
        with self.assertRaises(ValueError):frame_only_links(t,['left_tool0','right_tool0'])
        t=ET.fromstring(p['kinematics']['urdf_xml'])
        for node in t.findall('omindos_frame_only'):t.remove(node)
        p['kinematics']['urdf_xml']=ET.tostring(t,encoding='unicode')
        with self.assertRaises(ValueError):robot(p)

    def test_preview_sine_limits_and_stationary_base(self):
        p=profile();q=FIXTURE['poses']['preview']
        out=preview.sine({'profile':p,'start':q,'joints':['waist_pitch','left_joint_1'],
                          'amplitude_deg':2,'period_s':4,'cycles':1})
        self.assertEqual(out['frames'][0]['positions'],q);self.assertEqual(out['frames'][-1]['positions'],q)
        self.assertIn('amplitude_limited_joints',out)
        for n,speed in out['peak_joint_velocity_rad_s'].items():
            if n=='waist_pitch':self.assertLessEqual(speed,.2)
        with self.assertRaises(ValueError):
            preview.trial({'profile':p,'base_start':{},'base_target':{'x_m':.1}})
        with self.assertRaises(ValueError):
            preview.sine({'profile':p,'start':q,'joints':['waist_pitch'],'amplitude_deg':20,'period_s':.5})

    def test_missing_velocity_not_filled_from_unknown_motor(self):
        p=profile();t=ET.fromstring(p['kinematics']['urdf_xml']);t.find("joint[@name='waist_pitch']/limit").attrib.pop('velocity')
        p['kinematics']['urdf_xml']=ET.tostring(t,encoding='unicode')
        with self.assertRaises(ValueError):preview.jog({'profile':p,'joint':'waist_pitch','direction':1})

    def test_strict_runtime_still_rejects_geometry_only_profile(self):
        with self.assertRaises(ValueError):registry.validate_parameters(profile())
        with tempfile.TemporaryDirectory()as folder:
            saved=_register_workbench_profile(profile(),folder)
            self.assertEqual(_load_workbench_profile(saved['profile_id'],folder),profile())
            with self.assertRaises(ValueError):registry.load_robot_profile(saved['profile_id'],folder)

    def test_real_http_save_history_jog_stop_and_restart(self):
        with tempfile.TemporaryDirectory()as folder:
            stored=None
            for restart in (False,True):
                server=registry.ThreadingHTTPServer(('127.0.0.1',0),make_handler(folder));thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                base=f'http://127.0.0.1:{server.server_port}'
                def call(path,data=None):
                    req=urllib.request.Request(base+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
                    with urllib.request.urlopen(req,timeout=10)as response:return json.load(response)
                try:
                    if not restart:
                        stored=call('/v1/workbench-profiles',profile())['profile_id']
                        self.assertEqual(call('/v1/workbench-profiles/'+stored),profile())
                        call('/v1/workbench-active-profile',profile())
                    self.assertEqual(call('/v1/workbench-active-profile')['profile'],profile())
                    out=call('/v1/robot-jog',{'joint':'waist_pitch','direction':1})
                    self.assertIsNone(out['frames'][-1]['motor_positions_rad'])
                    call('/v1/workbench-stop',{'engaged':True})
                    with self.assertRaises(urllib.error.HTTPError)as caught:
                        call('/v1/robot-jog',{'joint':'waist_pitch','direction':1})
                    self.assertEqual(caught.exception.code,423)
                    self.assertEqual(call('/v1/workbench-profiles/'+stored),profile())
                finally:server.shutdown();thread.join(timeout=5);server.server_close()


class BundleRegistryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.assets=dict(robot_assets.MESH_ASSETS);self.urls=dict(robot_assets.EXTRA_MESH_URLS)
        self.catalog=copy.deepcopy(bundle.CATALOG);self.registered=bundle.REGISTERED_ROOT
        bundle.CATALOG.clear();bundle.REGISTERED_ROOT=None
        # A deliberately tiny synthetic mesh tests transfer integrity, not CAD rendering.
        raw=b'\0'*80+struct.pack('<I',1)+struct.pack('<12fH',0,0,1,0,0,0,1,0,0,0,1,0,0)
        digest=hashlib.sha256(raw).hexdigest();self.rel='meshes/'+digest+'.stl'
        (self.root/'meshes').mkdir();(self.root/self.rel).write_bytes(raw);(self.root/'models').mkdir()
        records={}
        for v in ('b','6fb','6f'):
            p=profile(v);t=ET.fromstring(p['kinematics']['urdf_xml']);vis=ET.SubElement(t.find("link[@name='base_link']"),'visual')
            ET.SubElement(ET.SubElement(vis,'geometry'),'mesh',filename='package://originmind_rm75_workbench/'+self.rel)
            p['kinematics']['urdf_xml']=ET.tostring(t,encoding='unicode');data=json.dumps(p).encode()
            (self.root/'models'/f'{v}.json').write_bytes(data)
            records[v]={'profile':f'models/{v}.json','profile_sha256':hashlib.sha256(data).hexdigest(),'mapping':FIXTURE['mapping']}
        self.manifest={'schema_version':1,'package':'originmind_rm75_workbench','assets':{self.rel:{'sha256':digest,'size_bytes':len(raw)}},'variants':records,'poses':FIXTURE['poses']}
        self.write_manifest()
    def write_manifest(self):(self.root/'manifest.json').write_text(json.dumps(self.manifest))
    def tearDown(self):
        robot_assets.MESH_ASSETS.clear();robot_assets.MESH_ASSETS.update(self.assets)
        robot_assets.EXTRA_MESH_URLS.clear();robot_assets.EXTRA_MESH_URLS.update(self.urls)
        bundle.CATALOG.clear();bundle.CATALOG.update(self.catalog);bundle.REGISTERED_ROOT=self.registered
        self.tmp.cleanup()
    def test_explicit_registration_resolves_only_verified_local_uris(self):
        result=bundle.register_bundle(self.root);self.assertEqual(result['mesh_count'],1)
        self.assertEqual(len(bundle.inventory()['models']),3)
        uri='package://originmind_rm75_workbench/'+self.rel
        self.assertIn(robot_assets.approved_mesh_url(uri),robot_assets.MESH_ASSETS)
        self.assertIsNone(robot_assets.approved_mesh_url('file:///etc/passwd'))
        self.assertIsNone(robot_assets.approved_mesh_url(uri+'/../other'))
        model=bundle.model('b');model['profile']['robot_id']='changed'
        self.assertNotEqual(bundle.model('b')['profile']['robot_id'],'changed')
    def test_tampered_mesh_does_not_publish_partial_catalog(self):
        p=self.root/self.rel;data=p.read_bytes();p.write_bytes(data[:-1]+bytes([data[-1]^1]))
        with self.assertRaises(ValueError):bundle.register_bundle(self.root)
        self.assertEqual(bundle.CATALOG,{});self.assertEqual(robot_assets.EXTRA_MESH_URLS,self.urls)
    def test_unsafe_manifest_path_is_rejected(self):
        record=self.manifest['assets'].pop(self.rel);self.manifest['assets']['../outside.stl']=record;self.write_manifest()
        with self.assertRaises(ValueError):bundle.register_bundle(self.root)
    def test_profile_digest_and_unregistered_mesh_are_rejected(self):
        p=self.root/'models/b.json';data=json.loads(p.read_text());data['kinematics']['urdf_xml']=data['kinematics']['urdf_xml'].replace('package://originmind_rm75_workbench/','https://example.com/')
        raw=json.dumps(data).encode();p.write_bytes(raw)
        with self.assertRaises(ValueError):bundle.register_bundle(self.root)
        self.manifest['variants']['b']['profile_sha256']=hashlib.sha256(raw).hexdigest();self.write_manifest()
        with self.assertRaises(ValueError):bundle.register_bundle(self.root)
        self.assertEqual(bundle.CATALOG,{})


if __name__=='__main__':unittest.main()
