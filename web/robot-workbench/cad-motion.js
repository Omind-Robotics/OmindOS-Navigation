import * as THREE from 'three';
import {bindRig} from './cad-kinematics.js';
export async function mountMotion(container,assembly,scene,render,manifest,model){
  if(model.source_sha256!==manifest.source_sha256||model.glb_sha256!==manifest.glb_sha256)throw Error('运动映射与 CAD 版本不一致');
  const rig=bindRig(assembly,model),get=id=>container.querySelector('#'+id);
  const status=get('cad-motion-status'),details=get('cad-axis-details'),canvas=get('cad-scene');
  let current='FL',frame=null,playing=false;
  const names=['侧摆','大腿','膝驱动'];
  const guides=new THREE.Group();scene.add(guides);
  const guideItems=[0,1,2].map((_,i)=>{
    const arrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),.14,[0xe5803c,0x179eac,0x7f68be][i],.025,.014);
    arrow.traverse(o=>{if(o.material){o.material.depthTest=false;o.material.transparent=true;o.material.opacity=.9;o.renderOrder=5;}});
    guides.add(arrow);return arrow;
  });
  const controls=names.map((name,i)=>{
    const row=document.createElement('div');row.className='cad-joint-row';
    const label=document.createElement('label');label.textContent=name;label.htmlFor='cad-q'+i;
    const range=document.createElement('input');range.type='range';range.id='cad-q'+i;range.step='.5';range.setAttribute('aria-label',name+'预览角度');
    const input=document.createElement('input');input.type='number';input.id='cad-q'+i+'-value';input.step='.5';input.setAttribute('aria-label',name+'角度数值');
    const unit=document.createElement('span');unit.textContent='°';row.append(label,range,input,unit);get('cad-joint-controls').append(row);
    function change(value){stop();const next=rig.angles;next[current][i]=value;const ok=apply(next),message=status.textContent;sync();if(!ok)status.textContent=message;}
    range.oninput=()=>change(range.valueAsNumber);input.onchange=()=>change(input.valueAsNumber);
    return {range,input};
  });
  function sync(){
    const leg=model.legs.find(l=>l.id===current),q=rig.angles[current];
    controls.forEach(({range,input},i)=>{for(const control of [range,input]){control.disabled=!leg.enabled[i];control.title=leg.enabled[i]?'':leg.blocked_reason;control.min=-leg.preview_ranges_deg[i];control.max=leg.preview_ranges_deg[i];control.value=q[i].toFixed(1);}});
    const axes=[leg.roll,leg.pitch,leg.knee];
    axes.forEach((a,i)=>{
      let origin=new THREE.Vector3(...a.origin),axis=new THREE.Vector3(...a.axis);
      if(i){const rot=new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...leg.roll.axis),q[0]*Math.PI/180);origin.sub(new THREE.Vector3(...leg.roll.origin)).applyMatrix4(rot).add(new THREE.Vector3(...leg.roll.origin));axis.transformDirection(rot);}
      guideItems[i].position.copy(origin);guideItems[i].setDirection(axis);
    });
    guides.visible=get('cad-show-axes').checked;
    details.textContent=leg.label+'：'+axes.map((a,i)=>names[i]+'轴位置 '+a.origin.map(v=>(v*1000).toFixed(1)).join(', ')+' mm；方向 '+a.axis.map(v=>v.toFixed(4)).join(', ')).join('。')+'。正方向按箭头右手定则；电机方向映射尚未标定。';
    status.textContent=leg.blocked_reason?leg.label+'：仅开放侧摆。'+leg.blocked_reason:leg.label+'：侧摆、大腿与膝驱动已开放几何联动预览';
    canvas.dataset.jointAngles=JSON.stringify(rig.angles);
    container.querySelector('.cad-canvas-label span').textContent=Object.values(rig.angles).flat().some(a=>Math.abs(a)>1e-6)?'CAD / KINEMATICS':'CAD / SAVED POSE';render();
  }
  function apply(next){try{
    const solutions=rig.pose(next);canvas.dataset.closureError=String(Math.max(...solutions.map(s=>s.closure_error_m)));
    status.textContent='几何联动预览 · CAD 相对零位 · 未连接电机';sync();return true;
  }catch(e){status.textContent=e.message;return false;}}
  function stop(){if(frame!==null)cancelAnimationFrame(frame);frame=null;playing=false;get('cad-motion-stop').disabled=true;get('cad-demo').disabled=false;}
  container.querySelectorAll('[data-cad-leg]').forEach(btn=>btn.onclick=()=>{
    stop();current=btn.dataset.cadLeg;
    container.querySelectorAll('[data-cad-leg]').forEach(b=>b.setAttribute('aria-pressed',String(b===btn)));sync();
  });
  get('cad-export-calibration').onclick=()=>{
    const data={schema_version:1,assembly_revision:model.assembly.revision,source_sha256:model.source_sha256,glb_sha256:model.glb_sha256,frame:model.frame,zero:model.zero,angle_mapping:model.angle_mapping,calibration:model.calibration};
    const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));
    const link=document.createElement('a');link.href=url;link.download='OriginMind-V11-Calibration.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  get('cad-reset').disabled=false;get('cad-demo').disabled=false;
  get('cad-reset').onclick=()=>{stop();apply(Object.fromEntries(model.legs.map(l=>[l.id,[0,0,0]])));};
  get('cad-show-axes').onchange=sync;get('cad-motion-stop').onclick=stop;
  get('cad-demo').onclick=()=>{
    stop();const base=rig.angles,start=performance.now();playing=true;get('cad-demo').disabled=true;get('cad-motion-stop').disabled=false;
    function tick(now){
      if(!playing||container.hidden||document.hidden){stop();return;}
      const t=Math.min(1,(now-start)/6000),next=Object.fromEntries(Object.entries(base).map(([id,q])=>[id,[...q]]));
      // Start and end at the current pose; small offsets stay inside preview bounds.
      const leg=model.legs.find(l=>l.id===current);
      next[current]=base[current].map((v,i)=>v+(leg.enabled[i]?Math.min(4,leg.preview_ranges_deg[i]-Math.abs(v))*Math.sin(t*2*Math.PI):0));
      if(!apply(next)||t===1){stop();return;}frame=requestAnimationFrame(tick);
    }
    frame=requestAnimationFrame(tick);
  };
  window.addEventListener('keydown',e=>{if(e.key==='Escape')stop();});
  window.addEventListener('blur',stop);window.addEventListener('workbench:leave-cad',stop);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
  apply(rig.angles);canvas.dataset.joints='12';
  return {stop,rig,model};
}
