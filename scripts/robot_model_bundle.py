"""Explicit local RM75 bundle registration; imported XML cannot open files.

Registration happens only on process startup with a local directory chosen by
the operator/installer. All paths, byte counts and SHA256 hashes are checked
before publishing any URI. It neither applies a profile nor enables hardware.
"""
import copy
import hashlib
import json
from pathlib import Path
import re
import struct

import numpy as np
from robot_assets import MESH_ASSETS, EXTRA_MESH_URLS

PACKAGE='originmind_rm75_workbench'
RELATIVE=re.compile(r'meshes/([0-9a-f]{64})\.stl\Z')
CATALOG={}
REGISTERED_ROOT=None


def checked_bytes(root,relative,maximum):
    p=root/relative
    if p.is_symlink() or not p.resolve(strict=True).is_relative_to(root) or not p.is_file():
        raise ValueError('bundle path must be a regular file inside the selected directory')
    size=p.stat().st_size
    if not 0<size<=maximum:raise ValueError('bundle file size is outside its allowed range')
    raw=p.read_bytes()
    if len(raw)!=size:raise ValueError('bundle changed while being read')
    return p,raw


def register_bundle(directory):
    global REGISTERED_ROOT
    root=Path(directory).resolve(strict=True)
    if not root.is_dir():raise ValueError('model bundle directory required')
    if REGISTERED_ROOT is not None:raise ValueError('model bundle already registered for this process')
    _,raw=checked_bytes(root,'manifest.json',4*1024*1024);manifest=json.loads(raw)
    if not isinstance(manifest,dict) or manifest.get('schema_version')!=1 or manifest.get('package')!=PACKAGE:
        raise ValueError('unsupported model bundle manifest')
    assets=manifest.get('assets')
    if not isinstance(assets,dict) or not 1<=len(assets)<=256:
        raise ValueError('model bundle requires 1..256 explicit mesh records')
    paths={};uris={};total=0
    for rel,record in sorted(assets.items()):
        match=RELATIVE.fullmatch(rel)if isinstance(rel,str)else None
        if not match or not isinstance(record,dict)or record.get('sha256')!=match[1]or type(record.get('size_bytes'))is not int:
            raise ValueError('invalid mesh record')
        p,data=checked_bytes(root,rel,128*1024*1024);total+=len(data)
        if total>256*1024*1024:raise ValueError('model bundle exceeds total mesh budget')
        if len(data)!=record['size_bytes']or hashlib.sha256(data).hexdigest()!=record['sha256']:
            raise ValueError('model mesh integrity check failed')
        if len(data)<84:raise ValueError('binary STL required')
        count=struct.unpack_from('<I',data,80)[0]
        if not 0<count<=2_000_000 or len(data)!=84+50*count:raise ValueError('invalid binary STL size')
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
        triangles=np.frombuffer(data,dtype=dtype,offset=84)
        if not np.isfinite(triangles['vertices']).all()or not np.isfinite(triangles['normal']).all():
            raise ValueError('nonfinite model mesh')
        url='/meshes/rm75-cad/'+match[1]+'.stl'
        paths[url]=p;uris['package://'+PACKAGE+'/'+rel]=url
    variants=manifest.get('variants')
    if not isinstance(variants,dict)or set(variants)!={'b','6fb','6f'}:raise ValueError('expected the three explicit RM75 model variants')
    from workbench_state import _validate_workbench_profile
    import xml.etree.ElementTree as ET
    profiles={}
    for variant,record in variants.items():
        if not isinstance(record,dict)or record.get('profile')!='models/'+variant+'.json':
            raise ValueError('invalid model profile path')
        _,data=checked_bytes(root,record['profile'],2*1024*1024)
        if hashlib.sha256(data).hexdigest()!=record.get('profile_sha256'):raise ValueError('model profile integrity check failed')
        profile=json.loads(data)
        if profile.get('profile_mode')!='kinematic_preview':raise ValueError('bundle profile must be explicitly kinematic')
        _validate_workbench_profile(profile)
        tree=ET.fromstring(profile['kinematics']['urdf_xml'])
        for mesh in tree.findall('.//mesh'):
            if mesh.get('filename')not in uris:raise ValueError('profile references an unregistered mesh')
        mapping=record.get('mapping')
        if not isinstance(mapping,dict)or set(mapping)!={'waist','left','right','tcp_links'}:
            raise ValueError('explicit joint and TCP mapping required')
        poses=manifest.get('poses',{})
        profiles[variant]={'id':variant,'label':'RM75-'+variant.upper()+' · 子型号待实机确认',
                           'profile':profile,'mapping':mapping,'poses':poses,
                           'scope':'local_kinematic_preview','hardware_validated':False,'dynamics_validated':False}
    if any(u in MESH_ASSETS for u in paths)or any(u in EXTRA_MESH_URLS for u in uris):
        raise ValueError('bundle URI namespace is already registered')
    MESH_ASSETS.update(paths);EXTRA_MESH_URLS.update(uris);CATALOG.update(profiles);REGISTERED_ROOT=root
    return {'mesh_count':len(paths),'mesh_bytes':total,'model_ids':sorted(profiles),
            'manifest_sha256':hashlib.sha256(raw).hexdigest()}


def inventory():
    return {'models':[{'id':k,'label':v['label']}for k,v in CATALOG.items()],
            'hardware_connected':False,'scope':'kinematic_preview'}


def model(variant):
    if variant not in CATALOG:raise ValueError('model bundle is not installed or the model id is unknown')
    return copy.deepcopy(CATALOG[variant])
