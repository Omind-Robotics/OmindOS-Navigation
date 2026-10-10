const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..'),mods=path.join(root,'web/robot-workbench/node_modules');
const THREE=require(path.join(mods,'three'));
const temp=fs.mkdtempSync(path.join(os.tmpdir(),'quad-kinematics-'));
require(path.join(mods,'esbuild')).buildSync({entryPoints:[path.join(root,'web/robot-workbench/cad-kinematics.js')],bundle:true,platform:'node',format:'cjs',outfile:path.join(temp,'solver.cjs')});
const {solveLeg,bindRig}=require(path.join(temp,'solver.cjs'));
const model=JSON.parse(fs.readFileSync(path.join(root,'config/robot_models/originmind_quadruped_v11/kinematics.json'))),manifest=JSON.parse(fs.readFileSync(path.join(root,'config/robot_models/originmind_quadruped_v11/manifest.json')));
let samples=0,maxError=0;const vec=a=>new THREE.Vector3(...a);
const checks=[];function check(name,fn){fn();checks.push(name);}
check('revision_and_uncalibrated_state',()=>{assert.equal(model.source_sha256,manifest.source_sha256);assert.equal(model.glb_sha256,manifest.glb_sha256);assert.equal(model.calibration.motor_zero_offset_rad,null);assert.equal(model.calibration.hardware_validated,false);});
check('unique_complete_leg_partition',()=>{const assigned=[...model.fixed_leg_nodes,...model.legs.flatMap(l=>Object.values(l.groups).flat())];assert.equal(new Set(assigned).size,assigned.length);assert.deepEqual(new Set(assigned),new Set(manifest.parts.filter(p=>p.name.includes('/腿重装')).map(p=>p.node)));});
check('saved_zero_preserves_all_group_transforms',()=>{for(const l of model.legs)for(const m of Object.values(solveLeg(l,[0,0,0]).matrices))m.elements.forEach((v,i)=>assert(Math.abs(v-(i%5===0?1:0))<1e-8));});
check('source_mismatch_locks_affected_drives',()=>{assert.deepEqual(model.legs.map(l=>l.enabled),[[true,false,false],[true,false,false],[true,true,true],[true,false,false]]);const rr=model.legs.find(l=>l.id==='RR');assert(rr.source_axis_errors_mm.crank_motor_axis_mm>3);assert(rr.source_axis_errors_mm.crank_rod_pin_axis_mm>43);for(const l of model.legs.filter(l=>l.id!=='RL'))for(const i of [1,2]){const q=[0,0,0];q[i]=1;assert.throws(()=>solveLeg(l,q));}});
check('nonfinite_and_out_of_range_rejected',()=>{for(const l of model.legs){assert.throws(()=>solveLeg(l,[NaN,0,0]));assert.throws(()=>solveLeg(l,[Infinity,0,0]));assert.throws(()=>solveLeg(l,[11,0,0]));}});
check('physical_pivot_axes_remain_coincident_over_preview_sweep',()=>{
 for(const l of model.legs)for(let r=-10;r<=10;r+=10)for(let h=-20;h<=20;h+=2)for(let k=-20;k<=20;k+=2){
  if(!l.enabled[1]&&(h||k))continue;
  const s=solveLeg(l,[r,h,k]),m=s.matrices;
  const rotation=new THREE.Matrix4().extractRotation(m.hip),n=vec(l.pitch.axis).transformDirection(rotation);
  const pairs=[['thigh','calf',l.linkage.b,l.linkage.b],['crank','rod',l.linkage.a,l.linkage.a],['rod','calf',l.linkage.c,l.linkage.c]];
  for(const [g1,g2,p1,p2]of pairs){const delta=vec(p1).applyMatrix4(m[g1]).sub(vec(p2).applyMatrix4(m[g2]));const error=delta.cross(n).length();maxError=Math.max(maxError,error);assert(error<1e-7,`${l.id} ${g1}/${g2} closure ${error}`);}
  samples++;
 }
});
check('rig_returns_to_original_and_keeps_body_fixed',()=>{
 const assembly=new THREE.Group();for(const p of manifest.parts){const obj=new THREE.Object3D();obj.name=p.node;obj.position.set(.11,.22,.33);obj.updateMatrix();assembly.add(obj);}
 const rig=bindRig(assembly,model),q=rig.angles;q.RL=[5,13,-8];rig.pose(q);
 assert(Math.abs(rig.nodes.get('part_606').matrix.elements[12]-.11)>1e-3);
 for(const id of model.fixed_leg_nodes)assert.deepEqual(rig.nodes.get(id).matrix.elements,rig.originals.get(id).elements);
 const old=rig.nodes.get('part_606').matrix.clone();const bad=rig.angles;bad.FR[1]=1;assert.throws(()=>rig.pose(bad));assert.deepEqual(old.elements,rig.nodes.get('part_606').matrix.elements);
 rig.pose(Object.fromEntries(model.legs.map(l=>[l.id,[0,0,0]])));for(const[id,obj]of rig.nodes)obj.matrix.elements.forEach((v,i)=>assert(Math.abs(v-rig.originals.get(id).elements[i])<1e-8));
});
const out={checks,samples,max_pivot_axis_error_m:maxError,enabled_axes:6,identified_axes:12,physical_limits_validated:false,hardware_validated:false};
const dest=process.argv[2];if(dest){fs.mkdirSync(path.dirname(dest),{recursive:true});fs.writeFileSync(dest,JSON.stringify(out,null,2)+'\n');}console.log(JSON.stringify(out));fs.rmSync(temp,{recursive:true});
