"""Collision reduction must cover every original box without moving joints."""
import copy
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from prepare_rm75_collision import convert_profile,shape_corners,verified_asset


def fixture():
    t=ET.Element('robot',name='proxy-test')
    for name,count in (('base',8),('arm',1),('tool0',0)):
        link=ET.SubElement(t,'link',name=name)
        for i in range(count):
            c=ET.SubElement(link,'collision',name=f'source-{i}')
            ET.SubElement(c,'origin',xyz=f'{i*.1} 0 .2',rpy=f'0 {i*.2} .1')
            ET.SubElement(ET.SubElement(c,'geometry'),'box',size='.12 .03 .07')
    for name,parent,child,kind in (('hinge','base','arm','revolute'),('tool','arm','tool0','fixed')):
        j=ET.SubElement(t,'joint',name=name,type=kind)
        ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child)
        ET.SubElement(j,'origin',xyz='0 0 .3',rpy='0 .1 0')
        if kind=='revolute':
            ET.SubElement(j,'axis',xyz='0 1 0');ET.SubElement(j,'limit',lower='-1',upper='1',velocity='.2',effort='0')
    return {'profile_mode':'kinematic_preview','robot_id':'test-proxies','kinematics':{'urdf_xml':ET.tostring(t,encoding='unicode')}}


class CollisionProxyTests(unittest.TestCase):
    def test_every_source_corner_is_covered_in_serialized_proxies(self):
        p=fixture();new,audit=convert_profile(p,Path('.'),{'assets':{}},budget=2)
        self.assertEqual(audit['source_collision_count'],9);self.assertEqual(audit['proxy_count'],3)
        old=ET.fromstring(p['kinematics']['urdf_xml']);seen=[]
        for group in audit['groups']:
            link=old.find(f"link[@name='{group['link']}']")
            for i in group['source_indices']:
                pts,_=shape_corners(link.findall('collision')[i],Path('.'),{'assets':{}},{})
                self.assertTrue(np.all(np.abs(pts-np.array(group['center_m']))<=np.array(group['size_m'])/2+1e-12))
                seen.append((group['link'],i))
        self.assertEqual(len(seen),9);self.assertEqual(len(set(seen)),9)
    def test_deterministic_read_only_conversion_preserves_joint_geometry(self):
        p=fixture();before=copy.deepcopy(p);one,a=convert_profile(p,Path('.'),{'assets':{}},budget=2)
        two,b=convert_profile(p,Path('.'),{'assets':{}},budget=2)
        self.assertEqual(p,before);self.assertEqual(one,two);self.assertEqual(a,b)
        old=ET.fromstring(p['kinematics']['urdf_xml']);new=ET.fromstring(one['kinematics']['urdf_xml'])
        for x,y in zip(old.findall('joint'),new.findall('joint')):
            self.assertEqual(x.attrib,y.attrib)
            self.assertEqual([(c.tag,c.attrib)for c in x],[(c.tag,c.attrib)for c in y])
    def test_only_empty_fixed_leaf_is_declared_as_a_frame(self):
        p=fixture();new,a=convert_profile(p,Path('.'),{'assets':{}})
        self.assertEqual(a['frame_only_links'],['tool0'])
        t=ET.fromstring(p['kinematics']['urdf_xml']);arm=t.find("link[@name='arm']")
        for c in arm.findall('collision'):arm.remove(c)
        p['kinematics']['urdf_xml']=ET.tostring(t,encoding='unicode')
        with self.assertRaises(ValueError):convert_profile(p,Path('.'),{'assets':{}})
    def test_invalid_budget_rejected_without_changing_input(self):
        p=fixture();before=copy.deepcopy(p)
        for budget in (0,65,-1,True,2.5):
            with self.subTest(budget=budget),self.assertRaises(ValueError):convert_profile(p,Path('.'),{'assets':{}},budget)
        self.assertEqual(p,before)
    def test_nonfinite_geometry_and_dtd_are_rejected(self):
        for old,new in (('.12 .03 .07','nan .03 .07'),('<robot','<!DOCTYPE robot><robot')):
            p=fixture();p['kinematics']['urdf_xml']=p['kinematics']['urdf_xml'].replace(old,new)
            with self.assertRaises(ValueError):convert_profile(p,Path('.'),{'assets':{}})
    def test_external_and_symlink_assets_are_rejected(self):
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder)/'bundle';(root/'meshes').mkdir(parents=True)
            outside=Path(folder)/'outside';outside.write_bytes(b'not-a-mesh');digest=hashlib.sha256(outside.read_bytes()).hexdigest()
            rel='meshes/'+digest+'.stl';(root/rel).symlink_to(outside)
            manifest={'assets':{rel:{'sha256':digest,'size_bytes':outside.stat().st_size}}}
            for uri in ('https://example.com/mesh.stl','package://originmind_rm75_workbench/'+rel):
                with self.subTest(uri=uri),self.assertRaises(ValueError):verified_asset(root,manifest,uri)


if __name__=='__main__':unittest.main()
