"""Build a dual RM75-B reference preview; body and drives are uncalibrated."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build_demo():
    robot = ET.Element('robot', name='OmindOS 双七轴移动机器人')
    ET.SubElement(robot, 'omindos_base', type='four_wheel_differential',
                  **{role: role+'_wheel_joint' for role in ('left_front', 'right_front', 'left_rear', 'right_rear')})
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

    def joint(name, parent, child, xyz, axis=(0, 1, 0), limits=(-2.6, 2.6), kind='revolute', effort=40, speed=1, rpy=(0, 0, 0)):
        j = ET.SubElement(robot, 'joint', name=name, type=kind)
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=vector(xyz), rpy=vector(rpy))
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

    mount = body('waist_mount', 1.5, (.14, .14, .36), (0, 0, .18), draw=False)
    visual(mount, 'cylinder', (.072, .36), (0, 0, .18), 'metal')
    joint('waist_mount_fixed', 'base', 'waist_mount', (0, 0, .3), kind='fixed')
    torso = body('torso', 5, (.20, .18, .42), (0, 0, .25))
    visual(torso, 'cylinder', (.078, .26), color='metal', rpy=(math.pi/2, 0, 0))
    visual(torso, 'box', (.14, .48, .09), (0, 0, .465))
    visual(torso, 'box', (.012, .16, .22), (.09, 0, .30), 'dark')
    visual(torso, 'box', (.014, .12, .012), (.09, 0, .43), 'accent')
    joint('waist_pitch', 'waist_mount', 'torso', (.1, 0, .36), limits=(-.55, .55), effort=80, speed=.6)
    # Fixed sensor-style head: neck, rounded housing, visor and two optical windows.
    head = body('head', .8, (.16, .23, .16), (.005, 0, .12))
    visual(head, 'cylinder', (.029, .065), (0, 0, .018), 'metal')
    visual(head, 'cylinder', (.045, .014), (0, 0, -.007), 'dark')
    visual(head, 'box', (.012, .15, .074), (.087, 0, .126), 'dark')
    for sign in (-1, 1):
        visual(head, 'cylinder', (.037, .012), (.087, sign*.075, .126), 'dark', (0, math.pi/2, 0))
        visual(head, 'cylinder', (.020, .008), (.097, sign*.053, .128), 'metal', (0, math.pi/2, 0))
        visual(head, 'cylinder', (.014, .010), (.103, sign*.053, .128), 'accent', (0, math.pi/2, 0))
        visual(head, 'cylinder', (.007, .011), (.106, sign*.053, .128), 'dark', (0, math.pi/2, 0))
        visual(head, 'cylinder', (.031, .012), (.003, sign*.119, .119), 'metal', (math.pi/2, 0, 0))
    visual(head, 'box', (.011, .053, .007), (.087, 0, .069), 'accent')
    joint('head_mount', 'torso', 'head', (0, 0, .535), kind='fixed')
    # Manufacturer kinematic chain and inertial data; only visuals are simplified.
    source = ET.parse(ROOT/'config/robot_models/realman_rm75/RM75-B.urdf').getroot()
    for side, sign in [('left', 1), ('right', -1)]:
        for original in source.findall('link'):
            link = copy.deepcopy(original)
            name = original.get('name')
            link.set('name', side+'_'+name)
            for item in list(link):
                if item.tag in ('visual', 'collision'):
                    link.remove(item)
            robot.append(link)
            outgoing = next((j for j in source.findall('joint') if j.find('parent').get('link') == name), None)
            end = [float(v) for v in outgoing.find('origin').get('xyz').split()] if outgoing is not None else [0, 0, .025]
            length = math.hypot(*end)
            if length > .03:
                orientation = (math.pi/2, 0, 0) if abs(end[1]) > .01 else (0, 0, 0)
                visual(link, 'cylinder', (.041, length), [v/2 for v in end], rpy=orientation)
                visual(link, 'cylinder', (.043, .025), [v*.85 for v in end], 'dark', orientation)
            else:
                visual(link, 'sphere', (.047,), color='metal')
            visual(link, 'cylinder', (.048, .026), color='metal')
            visual(link, 'box', (.012, .06, .026), (.044, 0, 0), 'accent')
        for original in source.findall('joint'):
            j = copy.deepcopy(original)
            name = side+'_'+original.get('name')
            j.set('name', name)
            for tag in ('parent', 'child'):
                node = j.find(tag); node.set('link', side+'_'+node.get('link'))
            robot.append(j)
            limit = j.find('limit')
            drives[name] = dict(motor_model='DEMO-NOT-HARDWARE', reduction_ratio=10, direction=1,
                                zero_offset_rad=0, torque_limit_nm=float(limit.get('effort'))*.8,
                                velocity_limit_rad_s=1)
        joint(side+'_arm_mount', 'torso', side+'_base_link', (0, sign*.1, .4),
              kind='fixed', rpy=(-sign*math.pi/2, 0, 0))
        # Electric gripper palm with a mounting flange, motor cover and jaw cheeks.
        palm = body(side+'_hand', .42, (.080, .116, .072), (0, 0, -.045), 'shell')
        visual(palm, 'cylinder', (.037, .018), (0, 0, -.001), 'metal')
        visual(palm, 'cylinder', (.027, .010), (0, 0, .008), 'dark')
        visual(palm, 'box', (.072, .103, .014), (0, 0, -.084), 'dark')
        visual(palm, 'box', (.007, .037, .007), (.041, 0, -.041), 'accent')
        for cheek in (-1, 1):
            visual(palm, 'box', (.010, .125, .039), (cheek*.039, 0, -.077), 'metal')
            for bolt in (-1, 1):
                visual(palm, 'cylinder', (.0035, .082), (0, bolt*.040, -.067 if cheek == 1 else -.086),
                       'dark', (0, math.pi/2, 0))
        joint(side+'_hand_mount', side+'_link_7', side+'_hand', (0, 0, 0), kind='fixed', rpy=(math.pi, 0, 0))
        for finger, direction in [('outer', 1), ('inner', -1)]:
            name = f'{side}_{finger}_finger'
            link = body(name, .09, (.044, .075, .155), (0, -direction*.024, -.080), draw=False)
            visual(link, 'box', (.034, .026, .056), (0, 0, -.028), 'metal')
            visual(link, 'box', (.032, .026, .084), (0, -direction*.015, -.083), 'metal', (-direction*.38, 0, 0))
            visual(link, 'box', (.036, .024, .048), (0, -direction*.037, -.121), 'shell')
            visual(link, 'box', (.037, .014, .043), (0, -direction*.048, -.123), 'rubber')
            visual(link, 'cylinder', (.020, .042), color='metal', rpy=(0, math.pi/2, 0))
            visual(link, 'cylinder', (.007, .046), color='dark', rpy=(0, math.pi/2, 0))
            for y, z in [(0, -.031), (-direction*.019, -.091)]:
                visual(link, 'cylinder', (.0032, .036), (0, y, z), 'dark', (0, math.pi/2, 0))
            for rib in range(5):
                visual(link, 'box', (.038, .002, .002), (0, -direction*.054, -.106-rib*.008), 'dark')
            joint(name+'_joint', side+'_hand', name, (0, direction*.055, -.09),
                  axis=(direction, 0, 0), limits=(0, .65), effort=4, speed=1)
        body(side+'_tip', .01, (.006, .006, .006), (0, 0, 0), 'accent')
        joint(side+'_tool_fixed', side+'_hand', side+'_tip', (0, 0, -.23), kind='fixed')
    ET.indent(robot)
    return {'robot_id': 'demo-dual-rm75', 'kinematics': {'urdf_xml': ET.tostring(robot, encoding='unicode')},
            'dynamics': {'joints': drives}}


if __name__ == '__main__':
    data = build_demo()
    from robot_parameter_api import validate_parameters
    validate_parameters(data)
    (ROOT/'config/robot_profiles/workbench-demo.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
