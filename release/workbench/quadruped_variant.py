"""Build-time variant data only; shares the existing compiled workbench backend."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def demo_profile():
    """Synthetic 12-DoF teaching geometry, never a calibrated customer model."""
    robot = ET.Element('robot', name='OmindOS 四足教学模型（未标定）')
    drives = {}

    def link(name, size, mass, center):
        node = ET.SubElement(robot, 'link', name=name)
        inertial = ET.SubElement(node, 'inertial')
        xyz = ' '.join(map(str, center))
        ET.SubElement(inertial, 'origin', xyz=xyz)
        ET.SubElement(inertial, 'mass', value=str(mass))
        x, y, z = size
        ET.SubElement(inertial, 'inertia', ixx=str(mass*(y*y+z*z)/12),
                      iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                      ixy='0', ixz='0', iyz='0')
        visual = ET.SubElement(node, 'visual')
        ET.SubElement(visual, 'origin', xyz=xyz)
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size=' '.join(map(str, size)))
        ET.SubElement(ET.SubElement(visual, 'material', name='shell'), 'color', rgba='.12 .55 .65 1')

    link('body', (.5, .28, .12), 5, (0, 0, .5))
    for leg, x, y in [('fl', .2, .16), ('fr', .2, -.16), ('rl', -.2, .16), ('rr', -.2, -.16)]:
        parent = 'body'
        for part, origin, axis, length in [('hip', (x,y,.5), '1 0 0', .04),
                                          ('thigh', (0,0,-.04), '0 1 0', .2),
                                          ('knee', (0,0,-.2), '0 1 0', .2)]:
            name = leg+'_'+part
            child = name+'_link'
            link(child, (.04,.04,length), .3, (0,0,-length/2))
            joint = ET.SubElement(robot, 'joint', name=name, type='revolute')
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=child)
            ET.SubElement(joint, 'origin', xyz=' '.join(map(str, origin)))
            ET.SubElement(joint, 'axis', xyz=axis)
            ET.SubElement(joint, 'limit', lower='-1.5', upper='1.5', effort='20', velocity='1')
            drives[name] = dict(motor_model='DEMO-NOT-HARDWARE', reduction_ratio=10,
                                direction=1, zero_offset_rad=0, torque_limit_nm=15, velocity_limit_rad_s=1)
            parent = child
    ET.indent(robot)
    return dict(robot_id='demo-quadruped', kinematics={'urdf_xml':ET.tostring(robot, encoding='unicode')},
                dynamics={'joints':drives})


def prepare(package: Path, root: Path):
    profile = demo_profile()
    (package/'config/robot_profiles/workbench-demo.json').write_text(
        json.dumps(profile, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    path = package/'web/robot-workbench/index.html'
    html = path.read_text(encoding='utf-8')
    replacements = {
        '加载双七轴移动机器人':'加载四足教学模型',
        '双臂各7轴 + 腰部pitch = 15个主体关节<br>RM75-B关节结构；机身、夹爪与驱动参数待标定。':
            '四条腿 × 每腿3关节；仅用于调参和运动学教学。<br>尺寸、质量和驱动参数为示意值，请导入自己的URDF与参数。',
        'value="demo-dual-rm75"':'value="demo-quadruped"',
        '关节运动学 · 底盘平面位姿 · 静态重力负载':'四足关节运动学 · 静态重力负载',
        '全身仅控制主体关节；夹爪和轮轴单独控制。':'全身正弦控制12个关节，不生成行走步态。',
    }
    for old, new in replacements.items():
        if old not in html:
            raise ValueError('Workbench template changed; review quadruped UI: '+old)
        html = html.replace(old,new)
    path.write_text(html, encoding='utf-8')
    # Keep DOM IDs used by the common renderer; omit manipulation-only controls
    # from this product's visible interface. No different controller is introduced.
    with (package/'web/robot-workbench/workbench.css').open('a',encoding='utf-8') as stream:
        stream.write('\n#connections-panel,.chassis-controls,#pose-details,[data-sine="left"],'
                     '[data-sine="right"],[data-sine="both"]{display:none}\n')
