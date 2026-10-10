#!/usr/bin/env python3
"""Convert the owner's V1.1 STEP to a static, instanced glTF visual asset.

Development only: CAD dependencies are not part of the customer runtime.
No joint, mass, inertia, drive calibration, or collision model is inferred.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import cadquery as cq
import trimesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopLoc import TopLoc_Location
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.XCAFApp import XCAFApp_Application
from OCP.BinXCAFDrivers import BinXCAFDrivers
from OCP.PCDM import PCDM_RS_OK, PCDM_SS_OK


def read_parts(source):
    app=XCAFApp_Application.GetApplication_s();BinXCAFDrivers.DefineFormat_s(app)
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    cache=source.with_suffix('.geometry.xbf')
    if cache.exists():
        doc=app.Retrieve(TCollection_ExtendedString(str(cache.resolve().parent),True),TCollection_ExtendedString(cache.name,True))
        if app.GetRetrieveStatus()!=PCDM_RS_OK:raise ValueError('Cache open failed')
    else:
        reader=STEPCAFControl_Reader()
        reader.SetColorMode(False);reader.SetNameMode(True);reader.SetLayerMode(False);reader.SetSHUOMode(False)
        Interface_Static.SetIVal_s('read.stepcaf.subshapes.name',0)
        if reader.ReadFile(str(source))!=IFSelect_RetDone:
            raise ValueError('STEP read failed')
        print('STEP read complete',flush=True)
        if not reader.Transfer(doc):raise ValueError('STEP transfer failed')
        if app.SaveAs(doc,TCollection_ExtendedString(str(cache),True))!=PCDM_SS_OK:raise ValueError('Cache write failed')
    print('STEP geometry transfer complete',flush=True)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots=TDF_LabelSequence();tool.GetFreeShapes(roots)
    def name(label):
        attr=TDataStd_Name()
        return str(attr.Get().ToExtString()) if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else 'unnamed'
    def walk(label,parent,path):
        location=parent.Multiplied(tool.GetLocation_s(label))
        ref=label
        if tool.IsReference_s(label):
            ref=TDF_Label();tool.GetReferredShape_s(label,ref)
        instance_name=name(label)
        part_name=name(ref)
        path=path+'/'+(part_name+' ['+instance_name+']' if instance_name.startswith('NAUO') else instance_name)
        if tool.IsAssembly_s(ref):
            components=TDF_LabelSequence();tool.GetComponents_s(ref,components)
            for child in components:yield from walk(child,location,path)
        else:
            yield cq.Shape.cast(tool.GetShape_s(ref)),path,cq.Location(location),None
    for label in roots:yield from walk(label,TopLoc_Location(),'')


def display_mesh(mesh):
    mesh.merge_vertices()
    if len(mesh.faces)<=128:return mesh
    # Preserve small parts and thin covers when aggressive simplification changes
    # their envelope. This is a display-fidelity bound, not a collision tolerance.
    for ratio in (.12,.25,.5):
        candidate=mesh.simplify_quadric_decimation(face_count=max(64,int(len(mesh.faces)*ratio)))
        if np.max(np.abs(candidate.bounds-mesh.bounds))<=.5:
            return candidate
    return mesh


def convert(source, output):
    expected = 'e655d503882cb58208b053457ed11bf7cdbc90d1f476141a0e9e8f02b1ac4d34'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != expected:
        raise ValueError('Different STEP revision: review conversion and exclusions before continuing')
    assembly = read_parts(source)
    scene = trimesh.Scene()
    full_scene = trimesh.Scene()
    bounds_errors=[]
    rows, excluded, geometry_cache = [], [], {}
    # CAD X/Y/Z -> display X/Y/Z = -Z/-X/Y; mm -> m, right-handed.
    basis = np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]], dtype=float)
    basis[:3,:3] *= .001
    for index, (shape, name, location, color) in enumerate(assembly):
        located = shape.moved(location)
        bb=Bnd_Box();BRepBndLib.Add_s(located.wrapped,bb,False)
        limits = list(bb.Get())
        # Two camera-internal solids in this specific source are grossly out of scale.
        # Preserve exact names and bounds in the report instead of silently clipping.
        if max(abs(x) for x in limits) > 10000:
            excluded.append({'name':name,'bounds_cad_mm':limits,'reason':'source geometry outside 10 m envelope'})
            continue
        key = shape.wrapped.HashCode(2147483647) if hasattr(shape.wrapped,'HashCode') else hash(shape.wrapped)
        if key not in geometry_cache:
            vertices, triangles = shape.tessellate(.6, .35)
            mesh = trimesh.Trimesh(vertices=np.array([v.toTuple() for v in vertices]),faces=np.array(triangles),process=False)
            if not len(mesh.faces) or not np.isfinite(mesh.vertices).all():
                raise ValueError('Empty or invalid geometry: '+name)
            original_mesh=mesh.copy()
            mesh=display_mesh(mesh)
            bounds_errors.append(float(np.max(np.abs(mesh.bounds-original_mesh.bounds))))
            mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[150,160,177,255],metallicFactor=.35,roughnessFactor=.45))
            geometry_cache[key] = 'geometry_'+str(len(geometry_cache))
            scene.geometry[geometry_cache[key]] = mesh
            full_scene.geometry[geometry_cache[key]] = original_mesh
        transform = np.eye(4)
        occ = location.wrapped.Transformation()
        for i in range(3):
            for j in range(4):
                transform[i,j] = occ.Value(i+1,j+1)
        node = 'part_'+str(index)
        scene.graph.update(frame_to=node,frame_from=scene.graph.base_frame,matrix=basis@transform,geometry=geometry_cache[key])
        full_scene.graph.update(frame_to=node,frame_from=full_scene.graph.base_frame,matrix=basis@transform,geometry=geometry_cache[key])
        rows.append({'node':node,'name':name,'geometry':geometry_cache[key],'bounds_cad_mm':limits})
        if index % 100 == 0:
            print('Converted',index,flush=True)
    output.mkdir(parents=True,exist_ok=True)
    blob = scene.export(file_type='glb')
    (output/'assembly.glb').write_bytes(blob)
    bounds = scene.bounds
    report = {'name':'OriginMind 四足机器狗 V1.1','source_filename':source.name,'source_sha256':digest,
              'status':'static_cad_only','source_leaf_count':len(rows)+len(excluded),'visible_instances':len(rows),
              'unique_meshes':len(scene.geometry),'triangles_instanced':sum(len(scene.geometry[x['geometry']].faces) for x in rows),
              'excluded':excluded,'bounds_m':bounds.tolist(),'size_m':(bounds[1]-bounds[0]).tolist(),
              'units':'m','basis':'display XYZ = (-CAD Z, -CAD X, CAD Y) / 1000',
              'glb_bytes':len(blob),'glb_sha256':hashlib.sha256(blob).hexdigest(),'parts':rows,
              'joint_model_available':False,'collision_model_available':False,'dynamics_validated':False,
              'hardware_connected':False,'display_palette':'preview materials, not manufacturing specification'}
    report['mesh_processing']='CAD tessellation 0.6 / 0.35 rad; welded vertices; 12%, 25%, 50% or original faces, with <=0.5 mm per-part bounds deviation'
    report['source_tessellation_bounds_m']=full_scene.bounds.tolist()
    report['max_part_bounds_deviation_mm']=max(bounds_errors)
    (output/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('parts','excluded')},ensure_ascii=False),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'config/robot_models/originmind_quadruped_v11')
    args=parser.parse_args()
    convert(args.source,args.output)
