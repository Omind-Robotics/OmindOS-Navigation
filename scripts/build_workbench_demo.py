"""Rebuild the synthetic mobile dual-arm example (not a calibrated robot)."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build_demo():
    robot = ET.Element('robot', name='OmindOS 移动双臂教学模型')
    drives = {}
    colors = {'shell': '.77 .84 .89 1', 'dark': '.055 .085 .12 1',
              'metal': '.32 .42 .5 1', 'accent': '.08 .66 .8 1', 'rubber': '.025 .035 .045 1'}
    for name, rgba in colors.items():
        ET.SubElement(ET.SubElement(robot, 'material', name=name), 'color', rgba=rgba)

    def vector(values):
        return ' '.join(str(x) for x in values)

    def visual(link, kind, size, xyz=(0, 0, 0), color='shell', rpy=(0, 0, 0)):
        v = ET.SubElement(link, 'visual')
        ET.SubElement(v, 'origin', xyz=vector(xyz), rpy=vector(rpy))
        shape = ET.SubElement(ET.SubElement(v, 'geometry'), kind)
        if kind == 'box':
            shape.set('size', vector(size))
        else:
            shape.set('radius', str(size[0]))
            if kind == 'cylinder':
                shape.set('length', str(size[1]))
        ET.SubElement(v, 'material', name=color)

    def body(name, mass, size, com, color='shell', draw=True):
        link = ET.SubElement(robot, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=vector(com))
        ET.SubElement(inertial, 'mass', value=str(mass))
        x, y, z = size
        # Explicit synthetic cuboid approximation; not CAD-derived properties.
        ET.SubElement(inertial, 'inertia', ixx=str(mass*(y*y+z*z)/12),
                      iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                      ixy='0', ixz='0', iyz='0')
        if draw:
            visual(link, 'box', size, com, color)
        return link

    def joint(name, parent, child, xyz, axis=(0, 1, 0), limits=(-2.6, 2.6), kind='revolute', effort=40, speed=1):
        j = ET.SubElement(robot, 'joint', name=name, type=kind)
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=vector(xyz), rpy='0 0 0')
        if kind != 'fixed':
            ET.SubElement(j, 'axis', xyz=vector(axis))
            ET.SubElement(j, 'limit', lower=str(limits[0]), upper=str(limits[1]), effort=str(effort), velocity=str(speed))
            ET.SubElement(j, 'dynamics', damping='.1', friction='.02')
            drives[name] = dict(motor_model='DEMO-NOT-HARDWARE', reduction_ratio=10, direction=1,
                                zero_offset_rad=0, torque_limit_nm=effort*.8, velocity_limit_rad_s=speed)

    base = body('base', 12, (.62, .54, .17), (0, 0, .185), 'dark')
    visual(base, 'box', (.57, .50, .035), (0, 0, .282), 'shell')
    visual(base, 'box', (.012, .30, .014), (.315, 0, .225), 'accent')
    visual(base, 'box', (.014, .45, .035), (-.313, 0, .14), 'rubber')
    for end, x in [('front', .22), ('rear', -.22)]:
        for side, sign in [('left', 1), ('right', -1)]:
            name = f'{side}_{end}_wheel'
            wheel = body(name, .7, (.2, .07, .2), (0, 0, 0), draw=False)
            visual(wheel, 'cylinder', (.1, .07), color='rubber', rpy=(math.pi/2, 0, 0))
            visual(wheel, 'cylinder', (.061, .075), color='metal', rpy=(math.pi/2, 0, 0))
            visual(wheel, 'box', (.10, .078, .015), color='shell')
            joint(name+'_joint', 'base', name, (x, sign*.295, .1), kind='continuous', effort=20, speed=5)

    mount = body('waist_mount', 1.5, (.14, .14, .17), (0, 0, .085), draw=False)
    visual(mount, 'cylinder', (.072, .17), (0, 0, .085), 'metal')
    joint('waist', 'base', 'waist_mount', (0, 0, .3), axis=(0, 0, 1))
    torso = body('torso', 5, (.17, .28, .44), (0, 0, .26))
    visual(torso, 'cylinder', (.078, .26), color='metal', rpy=(math.pi/2, 0, 0))
    visual(torso, 'box', (.14, .48, .09), (0, 0, .465))
    visual(torso, 'box', (.012, .16, .22), (.09, 0, .30), 'dark')
    visual(torso, 'box', (.014, .12, .012), (.09, 0, .43), 'accent')
    joint('waist_pitch', 'waist_mount', 'torso', (0, 0, .18), limits=(-.55, .55), effort=80, speed=.6)
    for side, sign in [('left', 1), ('right', -1)]:
        upper = body(side+'_upper', 1.2, (.07, .07, .36), (0, 0, -.18))
        visual(upper, 'box', (.013, .074, .14), (.03, 0, -.17), 'dark')
        joint(side+'_shoulder', 'torso', side+'_upper', (0, sign*.28, .48))
        forearm = body(side+'_forearm', .8, (.06, .06, .28), (0, 0, -.14))
        visual(forearm, 'box', (.064, .064, .045), (0, 0, -.25), 'dark')
        joint(side+'_elbow', side+'_upper', side+'_forearm', (0, 0, -.36))
        palm = body(side+'_hand', .3, (.08, .12, .06), (0, 0, -.025), 'dark')
        visual(palm, 'box', (.082, .075, .015), (0, 0, -.027), 'accent')
        joint(side+'_wrist', side+'_forearm', side+'_hand', (0, 0, -.28))
        for finger, direction in [('outer', 1), ('inner', -1)]:
            name = f'{side}_{finger}_finger'
            link = body(name, .06, (.035, .018, .1), (0, 0, -.05), 'metal')
            visual(link, 'box', (.036, .035, .023), (0, -direction*.008, -.1), 'rubber')
            joint(name+'_joint', side+'_hand', name, (0, direction*.048, -.055),
                  axis=(direction, 0, 0), limits=(0, .65), effort=4, speed=1)
        body(side+'_tip', .01, (.006, .006, .006), (0, 0, 0), 'accent')
        joint(side+'_tool_fixed', side+'_hand', side+'_tip', (0, 0, -.16), kind='fixed')
    ET.indent(robot)
    return {'robot_id': 'demo-dual-arm', 'kinematics': {'urdf_xml': ET.tostring(robot, encoding='unicode')},
            'dynamics': {'joints': drives}}


if __name__ == '__main__':
    data = build_demo()
    from robot_parameter_api import validate_parameters
    validate_parameters(data)
    (ROOT/'config/robot_profiles/workbench-demo.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
