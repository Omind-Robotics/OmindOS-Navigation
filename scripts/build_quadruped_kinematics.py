#!/usr/bin/env python3
"""Derive guarded CAD preview axes from this exact STEP; never motor calibration."""
import argparse,hashlib,json,re
from pathlib import Path
import numpy as np
from build_quadruped_cad import read_parts
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
ROOT=Path(__file__).resolve().parents[1]
BASIS=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
# Source instance IDs are revision-pinned by both STEP and GLB digests.
SPECS=[('FL','左前腿',761,903,774,948,953,None),('FR','右前腿',769,817,860,951,954,None),('RL','左后腿',192,275,335,606,614,607),('RR','右后腿',586,425,479,663,654,662)]
MOTORS={48:'body',169:'body',278:'body',279:'body',88:'hip',154:'hip',294:'hip',296:'hip',106:'thigh',166:'thigh',293:'thigh',295:'thigh'}
def point(v):return (BASIS@np.array(v)/1000).round(10).tolist()
def direction(v):
 d=BASIS@np.array(v);d=d/np.linalg.norm(d)
 if d[np.argmax(abs(d))]<0:d=-d
 return d.round(10).tolist()
def line_error(a,b,n):return float(np.linalg.norm(np.cross(np.array(a)-b,n)))
def build(source,out):
 manifest=json.loads((out/'manifest.json').read_text());digest=hashlib.sha256(source.read_bytes()).hexdigest()
 if digest!=manifest['source_sha256']:raise ValueError('STEP revision mismatch')
 if hashlib.sha256((out/'assembly.glb').read_bytes()).hexdigest()!=manifest['glb_sha256']:raise ValueError('GLB revision mismatch')
 byid={p['node']:p for p in manifest['parts']};needed={'part_'+str(v) for s in SPECS for v in s[2:] if v is not None};evidence={}
 for i,(shape,name,loc,_) in enumerate(read_parts(source)):
  node='part_'+str(i)
  if node not in needed:continue
  axes=[]
  for face in shape.moved(loc).Faces():
   surface=BRepAdaptor_Surface(face.wrapped)
   if surface.GetType()!=GeomAbs_Cylinder:continue
   c=surface.Cylinder();p=c.Location();d=c.Axis().Direction();r=c.Radius()
   if not any(abs(r-v)<1e-4 for v in [60,32.5,8.025,13,6]):continue
   item={'radius_mm':round(r,5),'point_cad_mm':[round(p.X(),7),round(p.Y(),7),round(p.Z(),7)],'axis_cad':[round(d.X(),10),round(d.Y(),10),round(d.Z(),10)]}
   if item not in axes:axes.append(item)
  evidence[node]={'name':name,'cylinders':axes}
 def axis(idx,r):return next(a for a in evidence['part_'+str(idx)]['cylinders'] if abs(a['radius_mm']-r)<1e-4)
 legs=[]
 for id,label,roll,pitch,knee,calf,rod,crank in SPECS:
  r,h,k=axis(roll,60),axis(pitch,32.5),axis(knee,32.5);n=direction(h['axis_cad']);o=point(h['point_cad_mm'])
  rodholes=[a for a in evidence['part_'+str(rod)]['cylinders'] if a['radius_mm']==8.025]
  rodholes.sort(key=lambda a:a['point_cad_mm'][1],reverse=True)
  a=point(rodholes[0]['point_cad_mm']);c=point(rodholes[-1]['point_cad_mm']);b=point(axis(calf,8.025)['point_cad_mm'])
  blocked=None;errors={}
  if crank is None:blocked='源 CAD 未找到此腿的 j5 膝驱动摇臂；大腿及膝联动待装配核对'
  else:
   errors={'crank_motor_axis_mm':line_error(point(axis(crank,13)['point_cad_mm']),o,n)*1000,'crank_rod_pin_axis_mm':line_error(point(axis(crank,6)['point_cad_mm']),a,n)*1000}
   if max(errors.values())>.1:blocked='源 CAD 摇臂与电机/传动杆不同轴；大腿及膝联动待修正'
  legs.append({'id':id,'label':label,'roll':{'origin':point(r['point_cad_mm']),'axis':direction(r['axis_cad']),'source_node':'part_'+str(roll)},'pitch':{'origin':o,'axis':n,'source_node':'part_'+str(pitch)},'knee':{'origin':point(k['point_cad_mm']),'axis':direction(k['axis_cad']),'source_node':'part_'+str(knee)},'linkage':{'a':a,'b':b,'c':c,'rod_node':'part_'+str(rod),'calf_node':'part_'+str(calf),'crank_node':'part_'+str(crank) if crank else None},'preview_ranges_deg':[10,20,20],'enabled':[True,not blocked,not blocked],'blocked_reason':blocked,'source_axis_errors_mm':errors,'groups':{g:[] for g in ['hip','thigh','crank','calf','rod']}})
 def legof(p):
  b=p['bounds_cad_mm'];return ('F' if b[2]+b[5]<0 else 'R')+('L' if b[0]+b[3]<0 else 'R')
 def semantic(p):
  pieces=p['name'].split('/');name=pieces[-1]
  if '/腿重装' not in p['name']:return 'body'
  if len(pieces)>4:return MOTORS[int(re.search(r'NAUO(\d+)',pieces[3])[1])]
  if re.search('轴承压板|轴承座|SCHAEFFLER',name):return 'body'
  if re.search('电机座04|j1-t-|j2-',name):return 'hip'
  if re.search('j3-2-|^外-|^内2-|^Mirror外-|^Mirror内2-',name):return 'thigh'
  if re.search('小腿-|脚丫子-',name):return 'calf'
  if re.search('传动杆-',name):return 'rod'
  if re.search('j5-|压盖2-',name):return 'crank'
  return None
 fixed=[];attachments=[]
 for leg in legs:
  parts=[p for p in manifest['parts'] if '/腿重装' in p['name'] and legof(p)==leg['id']]
  anchors=[(p,semantic(p)) for p in parts if semantic(p)]
  for p in parts:
   group=semantic(p)
   if group is None:
    b=np.array(p['bounds_cad_mm']);center=(b[:3]+b[3:])/2
    def score(item):
     bb=np.array(item[0]['bounds_cad_mm']);distance=np.linalg.norm(np.maximum(np.maximum(bb[:3]-center,center-bb[3:]),0))
     return distance,float(np.linalg.norm(center-(bb[:3]+bb[3:])/2))
    anchor,group=min(anchors,key=score)
    attachments.append({'node':p['node'],'group':leg['id']+'/'+group,'nearest_reference':anchor['node'],'method':'nearest CAD bounds; fastening ownership provisional'})
   if group=='body':fixed.append(p['node'])
   else:leg['groups'][group].append(p['node'])
 result={'schema_version':1,'status':'guarded_geometric_preview','source_sha256':digest,'glb_sha256':manifest['glb_sha256'],'units':'m','frame':'X front (dog head), Y left, Z up; display XYZ=(-CAD Z,-CAD X,CAD Y)/1000','zero':'saved CAD pose; all displayed delta angles are zero; encoder zeros unknown','angle_mapping':'right-hand roll delta; thigh delta; crank absolute delta = thigh delta + knee drive delta; calf and rod by circle closure','calibration':{'motor_direction':None,'motor_zero_offset_rad':None,'mechanical_limits_rad':None,'hardware_validated':False,'collision_validated':False},'legs':legs,'fixed_leg_nodes':fixed,'provisional_fastener_attachments':attachments,'notes':['Motor internals are carried with their housing, without simulating gears or rotor internals.','Front crank parts are absent by named-instance review; they are not synthesized.','Right rear source crank is misaligned; blocked axes cannot be enabled from the preview UI.','Fastener ownership is provisional; this is not a complete CAD mate reconstruction.']}
 (out/'kinematics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 (out/'joint_evidence.json').write_text(json.dumps({'source_sha256':digest,'evidence':evidence},ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({'legs':[{'id':l['id'],'enabled':l['enabled'],'errors_mm':l['source_axis_errors_mm'],'groups':{k:len(v) for k,v in l['groups'].items()}} for l in legs],'provisional_fasteners':len(attachments)},ensure_ascii=False))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,default=ROOT/'config/robot_models/originmind_quadruped_v11');a=p.parse_args();build(a.source,a.output)
