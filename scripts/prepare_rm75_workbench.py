#!/usr/bin/env python3
"""Prepare a traceable RM75 CAD display bundle; do not fabricate physical data.

Source package is read-only. Meshes on the same rigid link and with identical
RGBA are concatenated, never decimated. Collisions are preserved separately.
NumPy and trimesh are build-time dependencies, not runtime requirements.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

SOURCE_PACKAGE = 'originmind_rm75_description'
NAMESPACE = 'originmind_rm75_workbench'
VARIANTS = ('b', '6fb', '6f')
JOINT_RENAMES = {'waist_joint': 'waist_pitch'}
FIELDS = ('motor_model', 'reduction_ratio', 'direction', 'zero_offset_rad',
          'torque_limit_nm', 'velocity_limit_rad_s')
MAX_FILE = 128 * 1024 * 1024


def digest(data):
    return hashlib.sha256(data).hexdigest()


def source_path(root, uri):
    prefix = 'package://' + SOURCE_PACKAGE + '/'
    if not isinstance(uri, str) or not uri.startswith(prefix):
        raise ValueError('Expected a local source-package mesh')
    rel = PurePosixPath(uri[len(prefix):])
    if rel.is_absolute() or '..' in rel.parts or '\\' in str(rel) or not rel.parts:
        raise ValueError('Unsafe mesh path')
    path = (root / str(rel)).resolve(strict=True)
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('Mesh escapes source package')
    if path.stat().st_size > MAX_FILE:
        raise ValueError('Oversized source mesh')
    return path


def origin(element):
    node = element.find('origin')
    xyz = np.array([float(v) for v in (node.get('xyz', '0 0 0') if node is not None else '0 0 0').split()])
    rpy = np.array([float(v) for v in (node.get('rpy', '0 0 0') if node is not None else '0 0 0').split()])
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
        raise ValueError('Invalid origin')
    r, p, y = rpy; cr, cp, cy = np.cos(rpy); sr, sp, sy = np.sin(rpy)
    R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = xyz
    return T


def rgba(visual):
    color = visual.find('material/color')
    if color is None:
        raise ValueError('CAD visual needs explicit RGBA before merging')
    value = tuple(float(v) for v in color.get('rgba', '').split())
    if len(value) != 4 or not np.isfinite(value).all() or any(v < 0 or v > 1 for v in value):
        raise ValueError('Invalid CAD RGBA')
    return value


def write_asset(output, raw, assets):
    sha = digest(raw); relative = 'meshes/' + sha + '.stl'
    path = output / relative
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(raw)
    assets[relative] = {'sha256': sha, 'size_bytes': len(raw)}
    return 'package://' + NAMESPACE + '/' + relative


def prepare_variant(source, output, variant, assets):
    path = source / 'urdf' / ('dual_rm75_' + variant + '.urdf')
    xml = path.read_bytes()
    if len(xml) > 2*1024*1024 or b'<!DOCTYPE' in xml.upper() or b'<!ENTITY' in xml.upper():
        raise ValueError('Invalid source XML')
    tree = ET.fromstring(xml)
    original_joints = copy.deepcopy(tree.findall('joint'))
    collision_before = [ET.tostring(x) for x in tree.findall('.//collision')]
    before_count = len(tree.findall('.//visual')); records = []
    for link in tree.findall('link'):
        visuals = link.findall('visual')
        if len(visuals) <= 16:
            continue
        groups = {}
        for visual in visuals:
            groups.setdefault(rgba(visual), []).append(visual)
        if len(groups) > 16:
            raise ValueError('Too many materials on one link; do not relax viewer limits')
        for group_index, (color, group) in enumerate(groups.items()):
            vertices, faces, sources, vertex_offset, max_error = [], [], [], 0, 0.0
            for visual in group:
                g = visual.find('geometry/mesh')
                if g is None:
                    raise ValueError('Only mesh CAD visuals can be baked')
                p = source_path(source, g.get('filename'))
                mesh = trimesh.load_mesh(p, process=False)
                if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
                    raise ValueError('Expected a nonempty triangular mesh')
                scale = np.array([float(x) for x in g.get('scale', '1 1 1').split()])
                if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                    raise ValueError('Invalid mesh scale')
                T = origin(visual)
                transformed = (np.asarray(mesh.vertices)*scale) @ T[:3, :3].T + T[:3, 3]
                if not np.isfinite(transformed).all():
                    raise ValueError('Nonfinite baked geometry')
                max_error = max(max_error, float(np.max(np.abs(transformed-transformed.astype('<f4').astype(float)))))
                vertices.append(transformed); faces.append(np.asarray(mesh.faces)+vertex_offset)
                sources.append({'visual': visual.get('name'), 'source_sha256': digest(p.read_bytes()),
                                'source_path': str(p.relative_to(source)), 'triangles': len(mesh.faces),
                                'link_from_mesh': T.tolist(), 'scale': scale.tolist()})
                vertex_offset += len(mesh.vertices)
            baked = trimesh.Trimesh(vertices=np.concatenate(vertices), faces=np.concatenate(faces), process=False)
            raw = baked.export(file_type='stl')
            # Verify the actual serialized triangle positions, not just metadata.
            import io
            loaded = trimesh.load_mesh(io.BytesIO(raw), file_type='stl', process=False)
            err = float(np.max(np.abs(np.asarray(loaded.triangles)-np.asarray(baked.triangles))))
            if err > 1e-6 or len(loaded.faces) != sum(s['triangles'] for s in sources):
                raise ValueError('Baked STL changed geometry beyond 1 micrometre')
            uri = write_asset(output, raw, assets)
            new = ET.SubElement(link, 'visual', name=f'baked_{link.get("name")}_{group_index}')
            ET.SubElement(new, 'origin', xyz='0 0 0', rpy='0 0 0')
            ET.SubElement(ET.SubElement(new, 'geometry'), 'mesh', filename=uri, scale='1 1 1')
            mat = ET.SubElement(new, 'material', name=f'baked_{link.get("name")}_{group_index}')
            ET.SubElement(mat, 'color', rgba=' '.join(str(x) for x in color))
            records.append({'link': link.get('name'), 'rgba': color, 'sources': sources,
                            'output_uri': uri, 'triangles': len(loaded.faces),
                            'serialized_vertex_error_m': err})
        for visual in visuals:
            link.remove(visual)
    # Preserve every collision element; remap its resource URI only.
    if collision_before != [ET.tostring(x) for x in tree.findall('.//collision')]:
        raise ValueError('Visual baking changed collision elements')
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://' + NAMESPACE + '/'):
            continue
        p = source_path(source, uri)
        mesh.set('filename', write_asset(output, p.read_bytes(), assets))
    for joint in tree.findall('joint'):
        name = joint.get('name')
        if name in JOINT_RENAMES:
            joint.set('name', JOINT_RENAMES[name])
    for original, prepared in zip(original_joints, tree.findall('joint')):
        original.set('name', JOINT_RENAMES.get(original.get('name'), original.get('name')))
        if ET.tostring(original) != ET.tostring(prepared):
            raise ValueError('Joint geometry or limits changed')
    ET.indent(tree, space='  ')
    urdf = ET.tostring(tree, encoding='unicode')
    active = [j.get('name') for j in tree.findall('joint') if j.get('type') != 'fixed']
    if len(active) != 15 or max(len(l.findall('visual')) for l in tree.findall('link')) > 16:
        raise ValueError('Unexpected model structure')
    profile = {'robot_id': 'originmind-rm75-' + variant, 'profile_mode': 'kinematic_preview',
               'kinematics': {'urdf_xml': urdf}, 'dynamics': {'joints': {
                   n: {k: ('' if k == 'motor_model' else None) for k in FIELDS} for n in active}}}
    model_file = 'models/' + variant + '.json'
    (output / model_file).parent.mkdir(parents=True, exist_ok=True)
    raw = (json.dumps(profile, ensure_ascii=False, sort_keys=True, indent=2)+'\n').encode()
    (output / model_file).write_bytes(raw)
    mapping = {'waist': 'waist_pitch', 'left': [f'left_joint_{i}' for i in range(1,8)],
               'right': [f'right_joint_{i}' for i in range(1,8)], 'tcp_links': ['left_tool0','right_tool0']}
    return {'profile': model_file, 'profile_sha256': digest(raw), 'source_urdf_sha256': digest(xml),
            'mapping': mapping, 'joint_renames': JOINT_RENAMES,
            'links': len(tree.findall('link')), 'active_joints': len(active),
            'visuals_before': before_count, 'visuals_after': len(tree.findall('.//visual')),
            'max_visuals_per_link': max(len(l.findall('visual')) for l in tree.findall('link')),
            'collision_elements_preserved': len(collision_before), 'baked_groups': records}


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists() or output.is_relative_to(source):
        raise ValueError('Output must be a new directory outside the source package')
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.rm75-', dir=output.parent))
    try:
        assets, variants = {}, {}
        for variant in VARIANTS:
            variants[variant] = prepare_variant(source, stage, variant, assets)
        poses = json.loads((source/'config/poses.json').read_text())
        poses = {label:{JOINT_RENAMES.get(n,n):q for n,q in values.items()} for label,values in poses.items()}
        manifest = {'schema_version':1, 'package':NAMESPACE, 'default_variant':'6fb',
                    'scope':'kinematic_display_only', 'variants':variants, 'poses':poses,
                    'assets':assets, 'hardware_validated':False, 'dynamics_validated':False,
                    'collision_planning_validated':False,
                    'limitations':['Sensor subtype is unconfirmed; 6FB is a display default only.',
                     'Waist name is mapped, but CAD axis/zero and software limits are not hardware calibration.',
                     '357 original collision elements are preserved, exceeding the planner budget of 256.',
                     'Fixed tool frames have no physical collision body; planner frame handling remains to be adapted.',
                     'No wheel joints or hardware drivers have been added.']}
        (stage/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
        for file in ('NOTICE','README.md','vendor/RealMan-LICENSE.txt','config/assembly.json','docs/provenance.json'):
            src=source/file
            dest=stage/'source-notices'/file;dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(src,dest)
        (stage/'README.zh-CN.md').write_text('''# OriginMind RM75 工作台模型候选包\n\n这是整机显示资源，不是 Ubuntu 安装程序，也不是已验收的真机模型。\n\n原 341 个机身 CAD 外观实例按刚体和颜色合并为 4 个网格；不删减三角面，保留三种 RM75 变体。腰关节统一命名为 waist_pitch，原坐标、轴向、CAD 保存零姿态和软件预览限位不变。\n\nmodels/*.json 为显式运动学配置；manifest.json 保留网格摘要、来源变换、误差、TCP 映射及预览姿态。未知动力学和电机参数保持为空。\n\n原碰撞几何未删减：总计 357 项，超过当前规划器 256 项预算；该候选不代表碰撞规划已经可用。工具末端空坐标系与规划器也仍需适配。\n\n工作台需要显式注册本包本地资产后才能显示网格；直接向旧版安装包导入 JSON 不能自动安装资源。\n''')
        stage.rename(output)
        return manifest
    except Exception:
        shutil.rmtree(stage,ignore_errors=True)
        raise


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();result=prepare(args.source,args.output)
    print(json.dumps({v:{k:x[k] for k in ('links','active_joints','visuals_before','visuals_after','max_visuals_per_link','collision_elements_preserved')}for v,x in result['variants'].items()},indent=2))
