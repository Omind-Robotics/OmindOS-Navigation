"""Synthetic fixtures test CAD conversion, not hardware or collision acceptance."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from prepare_rm75_workbench import prepare_variant, source_path, NAMESPACE


def make_source(folder):
    folder.mkdir(parents=True,exist_ok=True)
    (folder/'meshes').mkdir(); (folder/'urdf').mkdir()
    cube=trimesh.creation.box(extents=(.01,.02,.03))
    (folder/'meshes/box.stl').write_bytes(cube.export(file_type='stl'))
    robot=ET.Element('robot',name='synthetic-converter-test')
    base=ET.SubElement(robot,'link',name='base_link')
    for i in range(20):
        visual=ET.SubElement(base,'visual',name=f'part-{i}')
        ET.SubElement(visual,'origin',xyz=f'{i*.025} .03 .02',rpy='0 .1 .2')
        mesh=ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',
            filename='package://originmind_rm75_description/meshes/box.stl',scale='1 1 1')
        ET.SubElement(ET.SubElement(visual,'material',name=f'm-{i}'),'color',
            rgba='0.5 0.5 0.5 1' if i<10 else '0.9 0.6 0.2 1')
        collision=copy.deepcopy(visual);collision.tag='collision';collision.remove(collision.find('material'));base.append(collision)
    ET.SubElement(robot,'link',name='torso_link')
    joint=ET.SubElement(robot,'joint',name='waist_joint',type='revolute')
    ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link='torso_link')
    ET.SubElement(joint,'axis',xyz='0 1 0');ET.SubElement(joint,'origin',xyz='.1 .00625 .593',rpy='0 0 0')
    ET.SubElement(joint,'limit',lower='-.5',upper='.5',velocity='.2',effort='0')
    for side in ('left','right'):
        previous='torso_link'
        for i in range(1,8):
            child=f'{side}_link_{i}';ET.SubElement(robot,'link',name=child)
            joint=ET.SubElement(robot,'joint',name=f'{side}_joint_{i}',type='revolute')
            ET.SubElement(joint,'parent',link=previous);ET.SubElement(joint,'child',link=child)
            ET.SubElement(joint,'axis',xyz='0 0 1');ET.SubElement(joint,'origin',xyz='0 0 .1',rpy='0 0 0')
            ET.SubElement(joint,'limit',lower='-2',upper='2',velocity='1',effort='0');previous=child
    raw=ET.tostring(robot)
    for variant in ('b','6fb','6f'):(folder/'urdf'/f'dual_rm75_{variant}.urdf').write_bytes(raw)
    return raw


class RM75BundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.source=self.root/'source';self.original=make_source(self.source)
        self.output=self.root/'output';self.output.mkdir()
    def tearDown(self):self.tmp.cleanup()
    def prepare(self):
        assets={};result=prepare_variant(self.source,self.output,'b',assets)
        profile=json.loads((self.output/result['profile']).read_text())
        return result,assets,profile
    def test_visual_budget_and_triangle_preservation(self):
        r,a,p=self.prepare()
        self.assertEqual(r['visuals_before'],20);self.assertEqual(r['visuals_after'],2)
        self.assertLessEqual(r['max_visuals_per_link'],16)
        self.assertEqual(sum(g['triangles'] for g in r['baked_groups']),20*12)
        self.assertLessEqual(max(g['serialized_vertex_error_m'] for g in r['baked_groups']),1e-6)
    def test_joint_name_mapping_preserves_axis_origin_limits(self):
        r,a,p=self.prepare();tree=ET.fromstring(p['kinematics']['urdf_xml'])
        original=ET.fromstring(self.original).find("joint[@name='waist_joint']")
        new=tree.find("joint[@name='waist_pitch']")
        self.assertIsNotNone(new);self.assertIsNone(tree.find("joint[@name='waist_joint']"))
        for tag in ('axis','origin','limit','parent','child'):self.assertEqual(original.find(tag).attrib,new.find(tag).attrib)
        self.assertEqual(r['active_joints'],15);self.assertEqual(r['mapping']['waist'],'waist_pitch')
    def test_collision_count_and_transforms_are_not_removed(self):
        r,a,p=self.prepare();new=ET.fromstring(p['kinematics']['urdf_xml'])
        original=ET.fromstring(self.original)
        self.assertEqual(r['collision_elements_preserved'],20)
        for old,cur in zip(original.findall('.//collision'),new.findall('.//collision')):
            self.assertEqual(old.find('origin').attrib,cur.find('origin').attrib)
            self.assertEqual(old.find('geometry/mesh').get('scale'),cur.find('geometry/mesh').get('scale'))
    def test_asset_digests_and_uris(self):
        r,a,p=self.prepare()
        for rel,record in a.items():
            data=(self.output/rel).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(),record['sha256'])
            self.assertEqual(len(data),record['size_bytes'])
        for mesh in ET.fromstring(p['kinematics']['urdf_xml']).findall('.//mesh'):
            self.assertTrue(mesh.get('filename').startswith('package://'+NAMESPACE+'/meshes/'))
    def test_unknown_physical_values_are_not_fabricated(self):
        r,a,p=self.prepare();self.assertEqual(p['profile_mode'],'kinematic_preview')
        self.assertEqual(len(p['dynamics']['joints']),15)
        for value in p['dynamics']['joints'].values():
            self.assertEqual(value['motor_model'],'')
            self.assertTrue(all(v is None for k,v in value.items() if k!='motor_model'))
        self.assertEqual(len(ET.fromstring(p['kinematics']['urdf_xml']).findall('.//inertial')),0)
    def test_source_is_read_only_and_output_deterministic(self):
        r,a,p=self.prepare();out=self.root/'again';out.mkdir();other={}
        r2=prepare_variant(self.source,out,'b',other)
        self.assertEqual(r,r2);self.assertEqual(a,other)
        self.assertEqual((self.source/'urdf/dual_rm75_b.urdf').read_bytes(),self.original)
    def test_unsafe_mesh_paths_and_schemes_rejected(self):
        for uri in ('https://example.com/model.stl','file:///etc/passwd',
                    'package://originmind_rm75_description/../secret.stl',
                    'package://originmind_rm75_description/meshes/../../secret.stl'):
            with self.subTest(uri=uri),self.assertRaises(ValueError):source_path(self.source,uri)
    def test_symlink_escape_rejected(self):
        secret=self.root/'outside.stl';secret.write_bytes(b'outside')
        (self.source/'meshes/link.stl').symlink_to(secret)
        with self.assertRaises(ValueError):source_path(self.source,'package://originmind_rm75_description/meshes/link.stl')
    def test_invalid_material_rejected(self):
        tree=ET.fromstring(self.original);tree.find('.//visual/material/color').set('rgba','nan 0 0 1')
        (self.source/'urdf/dual_rm75_b.urdf').write_bytes(ET.tostring(tree))
        with self.assertRaises(ValueError):self.prepare()
    def test_current_workbench_parser_and_motion_mapping(self):
        # On GitHub this uses the exact PR source, not a reimplementation.
        if not (ROOT/'scripts/robot_kinematics.py').is_file():
            self.skipTest('current workbench source is tested by PR CI')
        from robot_kinematics import build_model,pose
        r,a,p=self.prepare();model=build_model(p['kinematics']['urdf_xml'])
        q0=pose(model,{})
        for joint in [j for j in model['joints'] if j['type']!='fixed']:
            moved=pose(model,{joint['name']:.01})
            self.assertEqual(moved['link_transforms']['base_link'],q0['link_transforms']['base_link'])
            self.assertFalse(np.allclose(moved['link_transforms'][joint['child']],q0['link_transforms'][joint['child']]))
        self.assertIsNone(q0['holding_torque_nm'])

if __name__=='__main__':unittest.main()
