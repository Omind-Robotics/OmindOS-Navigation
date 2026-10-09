"""Package the pinned, complete MEVIUS2 reference visuals for the quadruped UI."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def demo_profile(root=ROOT):
    """Upstream reference geometry and SI limits; customer calibration is unknown."""
    source = root / 'config/robot_models/mevius2/models/mevius2_stl.urdf'
    robot = ET.fromstring(source.read_text(encoding='utf-8'))
    robot.set('name', 'MEVIUS2 四足参考模型')
    drives = {}
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        limit = joint.find('limit')
        drives[joint.get('name')] = dict(
            motor_model='MEVIUS2-REFERENCE-NOT-CUSTOMER', reduction_ratio=1,
            direction=1, zero_offset_rad=0, torque_limit_nm=float(limit.get('effort')),
            velocity_limit_rad_s=float(limit.get('velocity')))
    return dict(robot_id='mevius2-reference',
                kinematics={'urdf_xml': ET.tostring(robot, encoding='unicode')},
                dynamics={'joints': drives})


def prepare(package: Path, root: Path):
    subprocess.run([sys.executable, str(root / 'scripts/fetch_mevius2_workbench.py')], check=True)
    source = root / 'config/robot_models/mevius2'
    destination = package / 'config/robot_models/mevius2'
    if source.resolve() != destination.resolve():
        shutil.copytree(source, destination, dirs_exist_ok=True)
    (package / 'config/robot_profiles').mkdir(parents=True, exist_ok=True)
    (package / 'config/robot_profiles/workbench-demo.json').write_text(
        json.dumps(demo_profile(root), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    path = package / 'web/robot-workbench/index.html'
    html = path.read_text(encoding='utf-8')
    replacements = {
        '加载双七轴移动机器人': '加载四足参考模型',
        '双臂各7轴 + 腰部pitch = 15个主体关节<br>RM75-B关节结构；可按客户硬件设置模型与参数。':
            '完整 MEVIUS2 外观 · 12 个关节<br>参考参数，客户电机方向与零位待标定。',
        'value="demo-dual-rm75"': 'value="mevius2-reference"',
        '运动学 · 路径规划': '四足模型 · 参数配置',
        '全身仅控制主体关节；夹爪和轮轴单独控制。':
            '12 关节运动学预览；不生成行走步态。',
        '差速 / 全向导航仿真 · 双臂与腰部协同规划<br>导航到位并停车后操作；动力学后续升级':
            '参考模型 · 客户参数待标定 · 真机未连接<br>当前画面为运动学预览；未进行真实板卡验收',
        '参考参数 · 运动学': '完整外观 · 运动学预览',
    }
    for old, new in replacements.items():
        if old not in html:
            raise ValueError('Workbench template changed; review quadruped UI: ' + old)
        html = html.replace(old, new)
    path.write_text(html, encoding='utf-8')
    # Preserve common DOM IDs and backend behavior; expose only quadruped controls.
    with (package / 'web/robot-workbench/workbench.css').open('a', encoding='utf-8') as stream:
        stream.write('\n#connections-panel,.chassis-controls,.navigation-controls,.planning-controls,'
                     '.cooperative-controls,.reference-photo,#pose-details,[data-sine="left"],'
                     '[data-sine="right"],[data-sine="both"],[data-focus],#tool-trails,#follow-base'
                     '{display:none}\n')
