#!/usr/bin/env python3
"""Create conservative collision proxies without modifying source CAD assets.

Every original collision OBB is enclosed. Grouping is restricted to one rigid
link and is traceable by source collision index. No mass/torque is invented.
"""
import argparse
import copy
import hashlib
import itertools
import json
from pathlib import Path
import re
import shutil
import struct
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from prepare_rm75_workbench import origin

PACKAGE = 'originmind_rm75_workbench'
ASSET = re.compile(r'meshes/[0-9a-f]{64}\.stl\Z')
SIGNS = np.array(list(itertools.product((-1., 1.), repeat=3)))
PADDING_M = 1e-6


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def vector(text):
    v = np.array([float(x) for x in text.split()])
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError('expected finite three-vector')
    return v


def verified_asset(folder, manifest, uri):
    prefix = 'package://' + PACKAGE + '/'
    if not isinstance(uri, str) or not uri.startswith(prefix):
        raise ValueError('external collision mesh is not allowed')
    rel = uri[len(prefix):]
    if not ASSET.fullmatch(rel):
        raise ValueError('invalid content-addressed mesh path')
    record = manifest['assets'].get(rel)
    p = folder / rel
    if p.is_symlink() or not p.resolve(strict=True).is_relative_to(folder.resolve()) or not p.is_file():
        raise ValueError('mesh must be inside the explicit source bundle')
    if not isinstance(record, dict) or p.stat().st_size > 128 * 1024 * 1024 or p.stat().st_size != record.get('size_bytes'):
        raise ValueError('mesh size mismatch')
    raw = p.read_bytes()
    if len(raw) > 128 * 1024 * 1024 or sha256(raw) != record.get('sha256'):
        raise ValueError('mesh checksum mismatch')
    return rel, raw


def shape_corners(node, folder, manifest, cache):
    g = node.find('geometry')
    if g is None or len(g) != 1:
        raise ValueError('one collision geometry is required')
    shape = g[0]
    if shape.tag == 'mesh':
        uri = shape.get('filename')
        if uri not in cache:
            rel, raw = verified_asset(folder, manifest, uri)
            if len(raw) < 84:
                raise ValueError('binary STL required')
            n = struct.unpack_from('<I', raw, 80)[0]
            if n == 0 or n > 2_000_000 or len(raw) != 84 + 50*n:
                raise ValueError('invalid STL triangle count')
            dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
            pts = np.frombuffer(raw,dtype=dtype,offset=84)['vertices'].reshape(-1,3)
            if not np.isfinite(pts).all():
                raise ValueError('nonfinite STL vertices')
            cache[uri] = (pts.min(axis=0).astype(float),pts.max(axis=0).astype(float),rel,sha256(raw))
        lo, hi, rel, digest = cache[uri]
        scale = vector(shape.get('scale','1 1 1'))
        if np.any(scale <= 0):
            raise ValueError('mesh scale must be positive')
        lo, hi = lo*scale, hi*scale
        evidence = {'mesh':rel,'mesh_sha256':digest,'scale':scale.tolist()}
    elif shape.tag == 'box':
        half = vector(shape.get('size',''))/2
        if np.any(half <= 0):
            raise ValueError('positive box size required')
        lo,hi = -half,half; evidence = {'primitive':'box'}
    elif shape.tag in ('sphere','cylinder'):
        radius = float(shape.get('radius'))
        half = np.array([radius,radius,radius if shape.tag=='sphere' else float(shape.get('length'))/2])
        if not np.isfinite(half).all() or np.any(half <= 0):
            raise ValueError('positive primitive size required')
        lo,hi = -half,half; evidence = {'primitive':shape.tag}
    else:
        raise ValueError('unsupported collision geometry')
    T = origin(node)
    corners = ((lo+hi)/2 + SIGNS*(hi-lo)/2) @ T[:3,:3].T + T[:3,3]
    return corners, {**evidence,'link_from_geometry':T.tolist()}


def group_indices(bounds, budget):
    """Deterministic longest-centroid-axis partition; never remove a member."""
    groups = [list(range(len(bounds)))]
    centers = (bounds[:,0]+bounds[:,1])/2
    while len(groups) < min(budget,len(bounds)):
        choices = [(float(np.prod(bounds[g,1].max(0)-bounds[g,0].min(0))),len(g),-i,i)
                   for i,g in enumerate(groups) if len(g)>1]
        if not choices:
            break
        i=max(choices)[-1]; group=groups.pop(i)
        axis=int(np.argmax(np.ptp(centers[group],axis=0)))
        group=sorted(group,key=lambda n:(centers[n,axis],n))
        mid=len(group)//2;groups.extend((group[:mid],group[mid:]))
    return sorted(groups,key=lambda g:min(g))


def convert_profile(profile, folder, manifest, budget=24, *, link_budgets=None):
    if type(budget) is not int or not 1 <= budget <= 64:
        raise ValueError('per-link box budget must be 1..64')
    link_budgets = {} if link_budgets is None else link_budgets
    if not isinstance(link_budgets, dict) or any(
            not isinstance(k, str) or type(v) is not int or not 1 <= v <= 64
            for k, v in link_budgets.items()):
        raise ValueError('link_budgets must map link names to integer budgets 1..64')
    result=copy.deepcopy(profile)
    xml=profile['kinematics']['urdf_xml']
    if not isinstance(xml,str) or len(xml.encode())>2*1024*1024 or '<!DOCTYPE' in xml.upper() or '<!ENTITY' in xml.upper():
        raise ValueError('invalid source URDF')
    tree=ET.fromstring(xml)
    if set(link_budgets)-{link.get('name') for link in tree.findall('link')}:
        raise ValueError('link budget names must exist in the source model')
    if tree.findall('omindos_frame_only'):
        raise ValueError('source already declares frame-only links')
    joints=tree.findall('joint')
    incoming={j.find('child').get('link'):j for j in joints}
    parents={j.find('parent').get('link') for j in joints}
    audit=[];cache={};frames=[]; before=0;after=0;max_excess=-float('inf')
    for link in tree.findall('link'):
        name=link.get('name');collisions=link.findall('collision')
        if not collisions:
            j=incoming.get(name)
            if (len(link.findall('visual')) or link.find('inertial') is not None
                or j is None or j.get('type')!='fixed' or name in parents):
                raise ValueError(name+': missing physical collision geometry')
            ET.SubElement(tree,'omindos_frame_only',link=name);frames.append(name)
            continue
        before+=len(collisions)
        corners=[];sources=[]
        for index,node in enumerate(collisions):
            pts,detail=shape_corners(node,folder,manifest,cache)
            corners.append(pts);sources.append({'collision_index':index,**detail})
        bounds=np.array([[p.min(0),p.max(0)]for p in corners])
        for index,group in enumerate(group_indices(bounds,link_budgets.get(name,budget))):
            pts=np.concatenate([corners[i]for i in group])
            low=pts.min(0)-PADDING_M;high=pts.max(0)+PADDING_M
            center=(low+high)/2;size=high-low
            node=ET.SubElement(link,'collision',name='verified_proxy_'+str(index))
            fmt=lambda a:' '.join(format(float(v),'.17g')for v in a)
            ET.SubElement(node,'origin',xyz=fmt(center),rpy='0 0 0')
            ET.SubElement(ET.SubElement(node,'geometry'),'box',size=fmt(size))
            # Validate the serialized values, not just pre-serialization bounds.
            c=vector(node.find('origin').get('xyz'));s=vector(node.find('geometry/box').get('size'))
            excess=float(np.max(np.abs(pts-c)-s/2));max_excess=max(max_excess,excess)
            if excess>1e-12:
                raise ValueError('serialized proxy does not enclose every source OBB')
            audit.append({'link':name,'proxy':index,'source_indices':group,
                          'sources':[sources[i]for i in group],
                          'center_m':c.tolist(),'size_m':s.tolist(),
                          'max_source_corner_excess_m':excess})
            after+=1
        for node in collisions:link.remove(node)
    if after>256:
        raise ValueError('combined proxy count exceeds existing planner budget')
    if sum(len(a['source_indices'])for a in audit)!=before:
        raise ValueError('source collision coverage is incomplete')
    # Collision conversion cannot modify joint kinematics, inertias or visuals.
    original=ET.fromstring(xml)
    for kind in ('joint','link/visual','link/inertial'):
        if [ET.tostring(n)for n in original.findall(kind)]!=[ET.tostring(n)for n in tree.findall(kind)]:
            raise ValueError('non-collision model content changed')
    ET.indent(tree,space='  ')
    result['kinematics']['urdf_xml']=ET.tostring(tree,encoding='unicode')
    return result,{'source_collision_count':before,'proxy_count':after,'frame_only_links':frames,
                  'padding_m':PADDING_M,'max_source_corner_excess_m':max_excess,
                  'coverage_basis':'every corner of each original collision OBB, in its rigid link frame',
                  'false_positive_collisions_possible':True,'groups':audit}


def prepare(source,output, *, collision_mode='detailed'):
    if collision_mode not in ('lite', 'detailed'):
        raise ValueError('collision_mode must be lite or detailed')
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists() or output.is_relative_to(source):
        raise ValueError('output must be a new folder outside the source bundle')
    raw=(source/'manifest.json').read_bytes();manifest=json.loads(raw)
    if manifest.get('package')!=PACKAGE:
        raise ValueError('wrong source bundle package')
    output.parent.mkdir(parents=True,exist_ok=True)
    stage=Path(tempfile.mkdtemp(prefix='.rm75-collision-',dir=output.parent))
    try:
        (stage/'models').mkdir();(stage/'meshes').mkdir();report={};used=set();new=copy.deepcopy(manifest)
        for variant in ('b','6fb','6f'):
            record=manifest['variants'][variant];path=source/record['profile']
            if not path.resolve().is_relative_to(source):raise ValueError('unsafe profile path')
            data=path.read_bytes()
            if sha256(data)!=record['profile_sha256']:raise ValueError('profile checksum mismatch')
            profile,audit=convert_profile(json.loads(data),source,manifest,
                link_budgets={'base_link':8, 'torso_link':4} if collision_mode=='lite' else None)
            encoded=(json.dumps(profile,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()
            (stage/'models'/f'{variant}.json').write_bytes(encoded)
            new['variants'][variant]['profile']=f'models/{variant}.json'
            new['variants'][variant]['profile_sha256']=sha256(encoded)
            new['variants'][variant]['collision_proxy_count']=audit['proxy_count']
            new['variants'][variant]['frame_only_links']=audit['frame_only_links']
            for mesh in ET.fromstring(profile['kinematics']['urdf_xml']).findall('.//mesh'):
                rel,_=verified_asset(source,manifest,mesh.get('filename'));used.add(rel)
            report[variant]=audit
        for rel in sorted(used):shutil.copyfile(source/rel,stage/rel)
        new['assets']={rel:manifest['assets'][rel]for rel in sorted(used)}
        new['scope']='kinematic_display_with_conservative_collision_proxies'
        new['source_manifest_sha256']=sha256(raw)
        new['collision_representation_verified']=True
        new['collision_mode']=collision_mode
        new['collision_planning_validated']=False
        new['limitations']=['Conservative boxes enclose original collision OBBs and may reject valid narrow-space paths.',
                           'Proxy coverage is not evidence of a feasible collision-free trajectory.',
                           'Waist CAD axis/zero and limits are not hardware calibration.',
                           'Sensor subtype is unconfirmed. No force, dynamics or hardware execution.']
        (stage/'manifest.json').write_text(json.dumps(new,ensure_ascii=False,indent=2)+'\n')
        (stage/'collision-coverage.json').write_text(json.dumps({'source_manifest_sha256':sha256(raw),'variants':report},indent=2)+'\n')
        if (source/'source-notices').is_dir():shutil.copytree(source/'source-notices',stage/'source-notices')
        counts=', '.join(f'{v}: {a["source_collision_count"]} → {a["proxy_count"]}' for v,a in report.items())
        (stage/'README.zh-CN.md').write_text('# RM75 整机运动学模型与保守碰撞表示\n\n仅开发接入资源，非 Ubuntu 安装包。碰撞实体数量：'+counts+'。每个原实体的全部包围盒角点均逐一核验。两个空TCP坐标系显式声明，不能据此跳过任何有实体几何的连杆。\n\n外观网格、关节原点、轴向、限位不变。碰撞包围盒可能保守地拒绝原本可行的姿态；不代表已经找到无碰撞路径。未知动力学及电机参数仍为空。\n')
        stage.rename(output)
        return {'variants':{v:{k:a[k]for k in ('source_collision_count','proxy_count','frame_only_links','max_source_corner_excess_m')}for v,a in report.items()},
                'visual_assets':len(used),'asset_bytes':sum(r['size_bytes']for r in new['assets'].values()),
                'manifest_sha256':sha256((output/'manifest.json').read_bytes())}
    except Exception:
        shutil.rmtree(stage,ignore_errors=True);raise


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--collision-mode', choices=('lite','detailed'), default='detailed',
                   help='lite: base 8 + torso 4 + arm boxes; detailed: up to 24 per link')
    args=p.parse_args();print(json.dumps(prepare(args.source,args.output,collision_mode=args.collision_mode),indent=2))
