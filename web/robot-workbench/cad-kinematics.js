// CAD preview only. Angles are offsets from saved geometry, never motor commands.
import * as THREE from 'three';
const vec=a=>new THREE.Vector3(...a);
function turn(point,axis,angle){
  const r=new THREE.Matrix4().makeRotationAxis(vec(axis),angle);
  return new THREE.Matrix4().makeTranslation(...point).multiply(r).multiply(new THREE.Matrix4().makeTranslation(...point.map(x=>-x)));
}
function moved(p,m){return vec(p).applyMatrix4(m);}
function signedAngle(a,b,n){return Math.atan2(vec(n).dot(a.clone().cross(b)),a.dot(b));}
export function solveLeg(leg,angles){
  if(angles.length!==3||!angles.every(Number.isFinite))throw Error('关节角度无效');
  for(let i=0;i<3;i++)if(!leg.enabled[i]&&Math.abs(angles[i])>1e-10)throw Error(leg.blocked_reason);
  for(let i=0;i<3;i++)if(Math.abs(angles[i])>leg.preview_ranges_deg[i]+1e-8)throw Error('超出预览角度范围');
  const [roll,hip,knee]=angles.map(v=>v*Math.PI/180);
  const {origin:o,axis:n}=leg.pitch, a=vec(leg.linkage.a),b=vec(leg.linkage.b),c=vec(leg.linkage.c);
  const thigh=turn(o,n,hip),crank=turn(o,n,hip+knee),abduction=turn(leg.roll.origin,leg.roll.axis,roll);
  const bp=moved(leg.linkage.b,thigh),ap=moved(leg.linkage.a,crank),normal=vec(n);
  // Circle-circle closure in the plane normal to the pitch axes. Project axial
  // offsets away: bearing midplanes may differ, while pivot axes are coaxial.
  const project=(p)=>p.clone().addScaledVector(normal,-p.clone().sub(vec(o)).dot(normal));
  const aa=project(ap),bb=project(bp),cc=project(c),a0=project(a),b0=project(b);
  const r=cc.distanceTo(b0),l=cc.distanceTo(a0),d=aa.distanceTo(bb);
  if(d<1e-8||d>r+l+1e-9||d<Math.abs(r-l)-1e-9)throw Error('传动杆无闭合解，已保留上一姿态');
  const u=aa.clone().sub(bb).normalize(),v=normal.clone().cross(u);
  const x=(r*r-l*l+d*d)/(2*d),h2=r*r-x*x;
  if(h2< -1e-10)throw Error('传动杆无闭合解');
  const sign=Math.sign(normal.dot(a0.clone().sub(b0).cross(cc.clone().sub(b0))))||1;
  const cp=bb.clone().addScaledVector(u,x).addScaledVector(v,sign*Math.sqrt(Math.max(0,h2)));
  const beta=signedAngle(cc.clone().sub(b0),cp.clone().sub(bb),n);
  const calf=new THREE.Matrix4().makeTranslation(...bp.toArray()).multiply(new THREE.Matrix4().makeRotationAxis(normal,beta)).multiply(new THREE.Matrix4().makeTranslation(...leg.linkage.b.map(x=>-x)));
  const gamma=signedAngle(cc.clone().sub(a0),cp.clone().sub(aa),n);
  const rod=new THREE.Matrix4().makeTranslation(...ap.toArray()).multiply(new THREE.Matrix4().makeRotationAxis(normal,gamma)).multiply(new THREE.Matrix4().makeTranslation(...leg.linkage.a.map(x=>-x)));
  const matrices={hip:abduction,thigh:abduction.clone().multiply(thigh),crank:abduction.clone().multiply(crank),calf:abduction.clone().multiply(calf),rod:abduction.clone().multiply(rod)};
  return {matrices,closure_error_m:Math.max(Math.abs(cp.distanceTo(bb)-r),Math.abs(cp.distanceTo(aa)-l)),pivots:{a:aa.applyMatrix4(abduction).toArray(),b:bb.applyMatrix4(abduction).toArray(),c:cp.applyMatrix4(abduction).toArray()}};
}
export function bindRig(assembly,model){
  const nodes=new Map();assembly.traverse(o=>{if(o.name.startsWith('part_'))nodes.set(o.name,o);});
  const originals=new Map([...nodes].map(([id,obj])=>[id,obj.matrix.clone()]));
  const assigned=new Set();
  for(const leg of model.legs)for(const ids of Object.values(leg.groups))for(const id of ids){
    if(!nodes.has(id)||assigned.has(id))throw Error('关节零件映射无效：'+id);assigned.add(id);
  }
  let angles=Object.fromEntries(model.legs.map(l=>[l.id,[0,0,0]]));
  function pose(next){
    // Solve all legs before touching any node, so invalid requests are atomic.
    const solutions=model.legs.map(leg=>solveLeg(leg,next[leg.id]));
    for(let i=0;i<model.legs.length;i++)for(const [group,ids]of Object.entries(model.legs[i].groups))for(const id of ids){
      const obj=nodes.get(id);obj.matrixAutoUpdate=false;
      obj.matrix.copy(solutions[i].matrices[group]).multiply(originals.get(id));
      obj.matrixWorldNeedsUpdate=true;
    }
    assembly.updateMatrixWorld(true);angles=Object.fromEntries(Object.entries(next).map(([id,a])=>[id,[...a]]));
    return solutions;
  }
  pose(angles);
  return {pose,get angles(){return Object.fromEntries(Object.entries(angles).map(([id,a])=>[id,[...a]]));},nodes,originals};
}
