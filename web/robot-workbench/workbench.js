'use strict';
const $ = id => document.getElementById(id);
let xml, profile, model, currentPose, targets = {}, mode = 'joint', revision = 0, readyRevision = -1;
let timer, animation, latestTrial, view = {yaw: .75, pitch: .48, zoom: 1, center: [0, 0, .5], span: 1.5};
let projected = [], operation = false;
let sceneEngine = null;
let baseTarget = {x_m:0, y_m:0, yaw_rad:0};
const pendingNodes = new WeakMap(), pendingParents = new WeakMap();
const serializer = new XMLSerializer();
const child = (node, tag) => [...node.children].find(x => x.tagName === tag);
const nodes = tag => [...xml.documentElement.children].filter(x => x.tagName === tag);
const named = (tag, name) => nodes(tag).find(x => x.getAttribute('name') === name);
const numeric = text => text.trim() === '' ? NaN : Number(text);
function status(text, error = false) { $('status').textContent = text; $('status').classList.toggle('error', error); }
function parse(text) {
  if (new TextEncoder().encode(text).length > 2*1024*1024 || /<!DOCTYPE|<!ENTITY/i.test(text)) throw Error('URDF 超过 2 MiB 或包含不支持的 DTD / entity。');
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  if (doc.querySelector('parsererror') || doc.documentElement.tagName !== 'robot') throw Error('请提供有效的 URDF robot XML。');
  return doc;
}
function attach(node) { const parent = pendingParents.get(node); if (parent) { attach(parent); parent.append(node); pendingParents.delete(node); } }
function ensure(node, tag) {
  let n = child(node, tag); if (n) return n;
  if (!pendingNodes.has(node)) pendingNodes.set(node, new Map());
  const cache = pendingNodes.get(node); if (cache.has(tag)) return cache.get(tag);
  n = xml.createElement(tag); cache.set(tag,n); pendingParents.set(n,node);
  const set = n.setAttribute.bind(n); n.setAttribute = (...args) => { attach(n); return set(...args); }; return n;
}
function payload() { if (!profile || !xml) throw Error('请先导入模型。'); if (document.querySelector('#editor [aria-invalid="true"]')) throw Error('请先修正标记为无效的参数。'); profile.robot_id = $('robot-id').value; profile.kinematics.urdf_xml = serializer.serializeToString(xml); return structuredClone(profile); }
async function api(path, data) {
  const response = await fetch(path, data === undefined ? {} : {method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(data)});
  const body = await response.json(); if (!response.ok) throw Error(body.error || `请求失败 ${response.status}`); return body;
}
function stop(message) {
  cancelAnimationFrame(animation); animation = null; $('stop').disabled = true;
  if (message) status(message);
}
function edited({refresh = true, config = true} = {}) {
  stop(); revision++; readyRevision = -1; latestTrial = null; $('results').replaceChildren(); $('progress').value = 0;
  if(config)$('dirty').textContent = '有修改 · 未保存'; $('xml').value = serializer.serializeToString(xml);
  status('参数已修改'); clearTimeout(timer); if (refresh) timer = setTimeout(() => refreshModel(false), 120);
}
async function refreshModel(fit = false) {
  const rev = revision; document.body.dataset.pending = 'true';
  try {
    if (document.querySelector('.base-fields [aria-invalid="true"]')) throw Error('请先修正底盘位姿输入。');
    const result = await api('/v1/robot-model', {urdf_xml: payload().kinematics.urdf_xml, positions: targets, base_pose:baseTarget});
    if (rev !== revision) return;
    model = result.model; currentPose = result.pose; targets = {...result.pose.positions}; readyRevision = rev;
    $('model-name').textContent = model.name;
    $('model-info').textContent = `${model.links.length} 个连杆 · ${Object.keys(targets).length} 个活动关节`;
    $('warnings').textContent = model.warnings.join('；');
    syncQuickControls();
    if (fit) fitView(); draw();
    status('模型已更新 · 使用当前参数');
  } catch (e) { if (rev === revision) { readyRevision = -1; currentPose = null; draw(); status(e.message, true); } }
  finally { if (rev === revision) document.body.dataset.pending = 'false'; }
}
function missingDrives(doc) {
  return Object.fromEntries([...doc.documentElement.children].filter(n => n.tagName === 'joint' && n.getAttribute('type') !== 'fixed').map(n => [n.getAttribute('name'), {motor_model:'', reduction_ratio:null, direction:null, zero_offset_rad:null, torque_limit_nm:null, velocity_limit_rad_s:null}]));
}
async function importProfile(data, fromUrdf = false) {
  const candidate = fromUrdf ? {robot_id:$('robot-id').value || 'robot-a', kinematics:{urdf_xml:data}, dynamics:{joints:{}}} : structuredClone(data);
  if (!candidate || !candidate.kinematics || typeof candidate.kinematics.urdf_xml !== 'string' || !candidate.dynamics || typeof candidate.dynamics.joints !== 'object' || candidate.dynamics.joints === null) throw Error('完整配置需要 robot_id、kinematics.urdf_xml 和 dynamics.joints。');
  const doc = parse(candidate.kinematics.urdf_xml);
  const result = await api('/v1/robot-model', {urdf_xml:candidate.kinematics.urdf_xml});
  if (fromUrdf) candidate.dynamics.joints = missingDrives(doc);
  stop(); clearTimeout(timer); revision++; readyRevision = revision; latestTrial = null;
  profile = candidate; xml = doc; model = result.model; currentPose = result.pose; targets = {...result.pose.positions};
  baseTarget = {...result.pose.base_pose};
  document.querySelectorAll('.base-fields input').forEach(input=>input.removeAttribute('aria-invalid'));
  $('robot-id').value = candidate.robot_id || ''; $('xml').value = candidate.kinematics.urdf_xml;
  $('dirty').textContent = '未保存'; $('model-name').textContent = model.name;
  $('model-info').textContent = `${model.links.length} 个连杆 · ${Object.keys(targets).length} 个活动关节`;
  $('warnings').textContent = model.warnings.join('；'); $('results').replaceChildren(); $('progress').value = 0;
  $('profile-id').value='';$('saved').textContent='参数保存在本机服务目录；旧版本保留。';
  syncQuickControls(); rebuildParts(); fitView(); draw();
  status(fromUrdf ? '模型已导入。请补充各活动关节的电机参数后试运行、保存。' : '模型已载入 · 可调整参数');
}
function textBlock(text, cls = 'note') { const el = document.createElement('div'); el.className = cls; el.textContent = text; $('editor').append(el); return el; }
const fingerNames = side => ['outer','inner'].map(f=>`${side}_${f}_finger_joint`);
function syncQuickControls() {
  const pitch=model.joints.find(j=>j.name==='waist_pitch' && j.type==='revolute');
  $('pitch-control').hidden=!pitch;
  if(pitch){$('pitch').min=Math.ceil(pitch.lower*1800/Math.PI)/10;$('pitch').max=Math.floor(pitch.upper*1800/Math.PI)/10;$('pitch').value=targets.waist_pitch*180/Math.PI;$('pitch-value').textContent=(targets.waist_pitch*180/Math.PI).toFixed(1)+'°';}
  for(const side of ['left','right']){
    const joints=fingerNames(side).map(n=>model.joints.find(j=>j.name===n && j.type==='revolute'));
    $(side+'-gripper-control').hidden=joints.some(j=>!j);
    if(joints.every(Boolean)){
      const values=joints.map(j=>(targets[j.name]-j.lower)/(j.upper-j.lower)*100);
      $(side+'-gripper').value=values[0];
      $(side+'-gripper-value').textContent=Math.abs(values[0]-values[1])>.5?'单指已调':Math.round(values[0])+'%';
    }
  }
  for(const [id,value] of [['base-x',baseTarget.x_m],['base-y',baseTarget.y_m],['base-yaw',Number((baseTarget.yaw_rad*180/Math.PI).toFixed(3))]]){
    const input=$(id);if(document.activeElement!==input && input.getAttribute('aria-invalid')!=='true')input.value=value;
  }
}
function quickJointUpdate(selected) {
  mode='joint';rebuildParts(selected);edited({config:false});syncQuickControls();
}
$('pitch').oninput=()=>{const j=model.joints.find(j=>j.name==='waist_pitch');targets.waist_pitch=Math.max(j.lower,Math.min(j.upper,Number($('pitch').value)*Math.PI/180));quickJointUpdate('waist_pitch');};
for(const side of ['left','right'])$(side+'-gripper').oninput=()=>{
  const fraction=Number($(side+'-gripper').value)/100;
  for(const name of fingerNames(side)){const j=model.joints.find(j=>j.name===name);targets[name]=j.lower+(j.upper-j.lower)*fraction;}
  quickJointUpdate(fingerNames(side)[0]);
};
for(const [id,key,factor] of [['base-x','x_m',1],['base-y','y_m',1],['base-yaw','yaw_rad',Math.PI/180]])$(id).onchange=()=>{
  const input=$(id), value=numeric(input.value);
  edited({refresh:false,config:false});
  if(!Number.isFinite(value)||value<Number(input.min)||value>Number(input.max)){input.setAttribute('aria-invalid','true');status('底盘 X/Y 范围为 ±5 m，转向为 ±360°，请输入有效数字。',true);return;}
  input.removeAttribute('aria-invalid');baseTarget[key]=value*factor;refreshModel(true);
};
$('reset-pose').onclick=()=>{
  if(!model)return;for(const j of model.joints)if(j.type!=='fixed')targets[j.name]=Math.min(j.upper,Math.max(j.lower,0));
  baseTarget={x_m:0,y_m:0,yaw_rad:0};document.querySelectorAll('.base-fields input').forEach(input=>input.removeAttribute('aria-invalid'));
  edited({refresh:false,config:false});syncQuickControls();buildEditor();refreshModel(true);
};
function field(parent, label, value, onChange, attrs = {}) {
  const l = document.createElement('label'); l.textContent = label; const input = document.createElement('input');
  input.type = attrs.type || 'number'; input.step = 'any'; input.value = value ?? ''; for (const [key,v] of Object.entries(attrs)) input.setAttribute(key, v);
  input.addEventListener('change', () => {
    if (input.type !== 'text' && !Number.isFinite(numeric(input.value))) { status('请输入有限数字；空值不能作为参数。', true); input.setAttribute('aria-invalid','true'); return; }
    input.removeAttribute('aria-invalid'); onChange(input.type === 'text' ? input.value : Number(input.value)); edited();
  }); l.append(input); parent.append(l); return input;
}
function vectorFields(parent, label, node, attribute, fallback = '0 0 0') {
  textBlock(label, 'section-label'); const row = document.createElement('div'); row.className = 'fields'; parent.append(row);
  const values = (node.getAttribute(attribute) || fallback).split(/\s+/).map(Number);
  ['X','Y','Z'].forEach((name,i) => field(row, name, values[i], value => { values[i] = value; node.setAttribute(attribute, values.join(' ')); }));
}
function rebuildParts(selected) {
  $('part').replaceChildren();
  for (const node of nodes(mode)) { const opt = document.createElement('option'); opt.value = node.getAttribute('name'); opt.textContent = opt.value; $('part').append(opt); }
  if (selected && [...$('part').options].some(o => o.value === selected)) $('part').value = selected;
  $('joint-tab').setAttribute('aria-pressed', mode === 'joint'); $('link-tab').setAttribute('aria-pressed', mode === 'link'); buildEditor();
}
function buildEditor() {
  $('editor').replaceChildren(); const node = named(mode, $('part').value); if (!node) return;
  if (mode === 'joint') {
    const name = node.getAttribute('name'), type = node.getAttribute('type');
    textBlock(`${child(node,'parent')?.getAttribute('link')} → ${child(node,'child')?.getAttribute('link')} · ${type}`);
    if (type !== 'fixed') {
      const limit = ensure(node, 'limit'); const low = type === 'continuous' ? -Math.PI : Number(limit.getAttribute('lower'));
      const high = type === 'continuous' ? Math.PI : Number(limit.getAttribute('upper'));
      const label = document.createElement('label'); label.textContent = '目标关节角 / rad'; const out = document.createElement('output'); out.textContent = (targets[name] || 0).toFixed(3); label.append(out);
      const slider = document.createElement('input'); slider.type = 'range'; slider.min = low; slider.max = high; slider.step = '0.001'; slider.value = targets[name] ?? 0; slider.setAttribute('aria-label','目标关节角');
      slider.addEventListener('input', () => { targets[name] = Number(slider.value); out.textContent = targets[name].toFixed(3); edited({config:false}); }); label.append(slider); $('editor').append(label);
      vectorFields($('editor'), '关节转轴', ensure(node,'axis'), 'xyz', '1 0 0');
      textBlock('关节限位（关节侧）', 'section-label');
      if (type === 'revolute') for (const [key,title] of [['lower','下限 / rad'],['upper','上限 / rad']]) field($('editor'), title, limit.getAttribute(key), v => {limit.setAttribute(key,v); targets[name] = Math.max(Number(limit.getAttribute('lower')), Math.min(Number(limit.getAttribute('upper')), targets[name] || 0)); buildEditor();});
      for (const [key,title] of [['effort','URDF 力矩 / N·m'],['velocity','URDF 速度 / rad/s']]) field($('editor'), title, limit.getAttribute(key), v => limit.setAttribute(key,v), {min:'0'});
    }
    vectorFields($('editor'), '相对父连杆偏移 / m', ensure(node,'origin'), 'xyz');
    textBlock('调整连接距离会改变机构运动学；几何尺寸、质心和惯量需分别核对。');
    vectorFields($('editor'), '连接坐标旋转 R/P/Y / rad', ensure(node,'origin'), 'rpy');
    if (type !== 'fixed') {
      const dyn = ensure(node,'dynamics'); textBlock('物理阻尼与摩擦', 'section-label');
      for (const [key,title] of [['damping','阻尼 / N·m·s/rad'],['friction','摩擦 / N·m']]) field($('editor'), title, dyn.getAttribute(key) || 0, v => dyn.setAttribute(key,v), {min:'0'});
      textBlock('电机与传动（按实际设备填写）', 'section-label');
      const drive = profile.dynamics.joints[name] || missingDrives(xml)[name];
      const setDrive = (key, value) => {drive[key] = value; profile.dynamics.joints[name] = drive;};
      field($('editor'), '电机型号', drive.motor_model, v => setDrive('motor_model',v), {type:'text'});
      for (const [key,title] of [['reduction_ratio','减速比'],['direction','方向（+1 / −1）'],['zero_offset_rad','关节侧零点 / rad'],['torque_limit_nm','输出侧力矩上限 / N·m'],['velocity_limit_rad_s','输出侧速度上限 / rad/s']]) field($('editor'), title, drive[key], v => setDrive(key,v));
    }
  } else {
    const inertial = ensure(node,'inertial'); textBlock('质量与质心', 'section-label');
    field($('editor'), '质量 / kg', child(inertial,'mass')?.getAttribute('value'), v => ensure(inertial,'mass').setAttribute('value',v), {min:'0'});
    vectorFields($('editor'), '质心位置 / m', ensure(inertial,'origin'), 'xyz');
    vectorFields($('editor'), '惯量坐标旋转 R/P/Y / rad', ensure(inertial,'origin'), 'rpy');
    textBlock('惯性张量 / kg·m²', 'section-label');
    const tensor = ensure(inertial,'inertia'), group = document.createElement('div'); group.className = 'fields'; $('editor').append(group);
    for (const key of ['ixx','iyy','izz','ixy','ixz','iyz']) field(group,key,tensor.getAttribute(key),v=>tensor.setAttribute(key,v));
    textBlock('修改质量或外形后，请填写重新计算的惯量；系统不会猜测质量分布。');
    const visuals = [...node.children].filter(n=>n.tagName==='visual');
    visuals.forEach((visual,index)=>{
      const g = child(visual,'geometry')?.firstElementChild; if (!g) return;
      textBlock(`外形 ${index+1} · ${g.tagName}`, 'section-label');
      if (g.tagName === 'box') vectorFields($('editor'),'外形尺寸 / m',g,'size');
      else if (['sphere','cylinder'].includes(g.tagName)) { field($('editor'),'半径 / m',g.getAttribute('radius'),v=>g.setAttribute('radius',v),{min:'0'}); if(g.tagName==='cylinder') field($('editor'),'长度 / m',g.getAttribute('length'),v=>g.setAttribute('length',v),{min:'0'}); }
      else textBlock('Mesh 当前以关节骨架替代显示。');
      vectorFields($('editor'),'外形相对连杆偏移 / m',ensure(visual,'origin'),'xyz');
    });
    textBlock('外形尺寸只改变显示几何；关节连接距离在“关节”页调整。');
  }
}
function trans(t,p) { return [0,1,2].map(i => t[i][3]+p.reduce((s,x,j)=>s+t[i][j]*x,0)); }
function geometry(g) {
  const lines = []; const add=(a,b)=>lines.push([a,b]);
  if (g.type==='box') {
    const v=Array.from({length:8},(_,i)=>g.size.map((s,j)=>s/2*((i>>j&1)?1:-1)));
    for(let i=0;i<8;i++)for(let j=0;j<3;j++)if(!(i>>j&1))add(v[i],v[i|1<<j]);
  } else if (g.type==='cylinder'||g.type==='sphere') {
    const n=24, r=g.radius;
    if(g.type==='cylinder') for(let i=0;i<n;i++){const a=i*2*Math.PI/n,b=(i+1)*2*Math.PI/n;for(const z of [-g.length/2,g.length/2])add([r*Math.cos(a),r*Math.sin(a),z],[r*Math.cos(b),r*Math.sin(b),z]);if(i%6===0)add([r*Math.cos(a),r*Math.sin(a),-g.length/2],[r*Math.cos(a),r*Math.sin(a),g.length/2]);}
    else for(let axis=0;axis<3;axis++)for(let i=0;i<n;i++){const a=[r*Math.cos(i*2*Math.PI/n),r*Math.sin(i*2*Math.PI/n),0],b=[r*Math.cos((i+1)*2*Math.PI/n),r*Math.sin((i+1)*2*Math.PI/n),0];add(a.slice(axis).concat(a.slice(0,axis)),b.slice(axis).concat(b.slice(0,axis)));}
  } return lines;
}
function worldLines(pose=currentPose) {
  if(!model||!pose)return[]; const lines=[];
  for(const link of model.links)for(const g of link.visuals)for(const [a,b]of geometry(g))lines.push({a:trans(pose.link_transforms[link.name],trans(g.transform,a)),b:trans(pose.link_transforms[link.name],trans(g.transform,b)),link:link.name});
  return lines;
}
function fitView(poses=[currentPose]){
  if(!currentPose)return;const pts=poses.flatMap(pose=>[...worldLines(pose).flatMap(l=>[l.a,l.b]),...Object.values(pose.joint_positions),...Object.values(pose.centers_of_mass)]);
  if(!pts.length)return;const lo=[0,1,2].map(i=>Math.min(...pts.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...pts.map(p=>p[i])));
  view.center=lo.map((x,i)=>(x+hi[i])/2);view.span=Math.max(.15,...lo.map((x,i)=>hi[i]-x))*1.45;view.zoom=1;
}
function draw(){
  $('selected-kind').textContent = mode === 'joint' ? '当前关节' : '当前连杆';
  $('selected-part').textContent = $('part').value || '—';
  if(sceneEngine){sceneEngine.render(model,currentPose,view,mode,$('part').value);return;}
  drawWireframe();
}
function drawWireframe(){
  const canvas=$('scene'),ctx=canvas.getContext('2d'),w=canvas.clientWidth,h=canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);
  if(!ctx)return;
  canvas.width=w*dpr;canvas.height=h*dpr;ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);projected=[];
  if(!currentPose){ctx.fillStyle='#718596';ctx.font='14px system-ui';ctx.textAlign='center';ctx.fillText('导入有效模型后显示三维机构',w/2,h/2);return;}
  const scale=Math.min(w,h)/view.span*view.zoom, cy=Math.cos(view.yaw),sy=Math.sin(view.yaw),cp=Math.cos(view.pitch),sp=Math.sin(view.pitch);
  const project=p=>{const[x,y,z]=p.map((x,i)=>x-view.center[i]),u=cy*x-sy*y,v=sy*x+cy*y;return[w/2+u*scale,h/2-(cp*z-sp*v)*scale,cp*v+sp*z];};
  const line=(a,b,color,width=1)=>{a=project(a);b=project(b);ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();};
  const extent=view.span,step=extent/10;
  for(let i=-10;i<=10;i++){line([i*step,-extent,0],[i*step,extent,0],'#e0e7ed');line([-extent,i*step,0],[extent,i*step,0],'#e0e7ed');}
  for(const j of model.joints){const a=trans(currentPose.link_transforms[j.parent],[0,0,0]),b=currentPose.joint_positions[j.name];line(a,b,'#a6b9c7',5);}
  const list=worldLines().sort((a,b)=>(project(b.a)[2]+project(b.b)[2])-(project(a.a)[2]+project(a.b)[2]));
  for(const l of list)line(l.a,l.b,mode==='link'&&$('part').value===l.link?'#d28731':'#286d93',mode==='link'&&$('part').value===l.link?2.6:1.6);
  for(const [name,pos] of Object.entries(currentPose.centers_of_mass)){const p=project(pos);projected.push({p,name,type:'link'});ctx.beginPath();ctx.arc(p[0],p[1],3,0,Math.PI*2);ctx.fillStyle='#81a4b9';ctx.fill();}
  for(const j of model.joints){const p=project(currentPose.joint_positions[j.name]);projected.push({p,name:j.name,type:'joint'});ctx.beginPath();ctx.arc(p[0],p[1],j.type==='fixed'?3:6,0,Math.PI*2);ctx.fillStyle=mode==='joint'&&$('part').value===j.name?'#d28731':'#1e6387';ctx.fill();ctx.strokeStyle='#f7fbfd';ctx.lineWidth=2;ctx.stroke();}
  const selected=projected.find(p=>p.type===mode&&p.name===$('part').value);if(selected){ctx.font='12px system-ui';ctx.fillStyle='#7f5426';ctx.fillText(selected.name,selected.p[0]+10,selected.p[1]-10);}
  ctx.font='11px system-ui';[['X',[.15,0,0],'#a86458'],['Y',[0,.15,0],'#69865a'],['Z',[0,0,.15],'#5077a0']].forEach(([name,p,color])=>{line([0,0,0],p,color,2);const a=project(p);ctx.fillStyle=color;ctx.fillText(name,a[0]+4,a[1]);});
}
function report(result){
  $('results').replaceChildren();const head=document.createElement('p');head.textContent='运动路径采样点的峰值静态保持力矩';$('results').append(head);
  for(const [name,value] of Object.entries(result.peak_holding_torque_nm)){const row=document.createElement('div');row.className='metric';const label=document.createElement('span');label.textContent=name;const val=document.createElement('strong');val.textContent=`${value.toFixed(2)} N·m`;if(result.gravity_limit_exceeded.includes(name))val.className='error';row.append(label,val);$('results').append(row);}
  if(result.gravity_limit_exceeded.length){const warn=document.createElement('p');warn.className='error';warn.textContent='静态重力负载超限：'+result.gravity_limit_exceeded.join('、');$('results').append(warn);}
}
async function guarded(fn){if(operation)return;operation=true;try{await fn();}catch(e){status(e.message,true);}finally{operation=false;}}
async function loadExample(){await importProfile(await api('/example.json'));targets={...targets,waist_pitch:.12,left_shoulder:-.45,left_elbow:-.65,right_shoulder:-.3,right_elbow:-.75};for(const side of ['left','right'])for(const name of fingerNames(side))targets[name]=side==='left'?.4:.23;mode='joint';rebuildParts('waist_pitch');await refreshModel(true);status('教学示例已载入 · 双指夹爪、腰部俯仰、四轮底盘');}
$('run').onclick=()=>guarded(async()=>{
  clearTimeout(timer);if(readyRevision!==revision)await refreshModel(false);if(readyRevision!==revision)throw Error('模型参数有误，请先修正。');
  stop();const rev=revision;status('正在计算运动轨迹和重力负载…');const result=await api('/v1/robot-preview',{profile:payload(),target:targets,base_target:baseTarget,duration_s:numeric($('duration').value)});
  if(rev!==revision)return;latestTrial=result;report(result);model=result.model;fitView([result.frames[0],result.frames.at(-1)]);const start=performance.now();$('stop').disabled=false;
  const tick=now=>{const elapsed=(now-start)/1000,index=Math.min(result.frames.length-1,Math.floor(elapsed/result.duration_s*(result.frames.length-1)));currentPose=result.frames[index];$('progress').value=Math.min(1,elapsed/result.duration_s);draw();status(`运动学试运行 ${Math.min(elapsed,result.duration_s).toFixed(1)} / ${result.duration_s.toFixed(1)} 秒`);if(elapsed<result.duration_s)animation=requestAnimationFrame(tick);else{stop();status(result.gravity_limit_exceeded.length?'试运行结束 · 发现静态重力负载超限':'运动学试运行完成 · 未进行碰撞或动力学验收');}};animation=requestAnimationFrame(tick);
});
$('stop').onclick=()=>stop('已停止预览 · 未发送真机指令');
$('save').onclick=()=>guarded(async()=>{const rev=revision;const result=await api('/v1/robot-profiles',payload());$('profile-id').value=result.profile_id;$('saved').textContent='已保存配置：'+result.profile_id;if(rev===revision)$('dirty').textContent='参数已保存';status(rev===revision?'配置已保存，旧版本保留；未应用到控制器。':'修改前的参数已保存，当前编辑内容尚未保存。');});
$('load').onclick=()=>guarded(async()=>{const id=$('profile-id').value.trim();if(!/^[0-9a-f]{64}$/.test(id))throw Error('请输入完整的 64 位配置编号。');await importProfile(await api('/v1/robot-profiles/'+id));$('profile-id').value=id;$('saved').textContent='当前读取版本：'+id;$('dirty').textContent='已读取保存版本';status('已读取保存版本 · 尚未应用到控制器');});
$('export').onclick=()=>{try{const data=payload(),url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='robot-parameters.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);status('已导出编辑内容；保存版本时会进行完整参数校验。');}catch(e){status(e.message,true);}};
$('example').onclick=()=>guarded(loadExample);
$('import').onchange=()=>guarded(async()=>{const file=$('import').files[0];if(!file)return;if(file.size>2*1024*1024)throw Error('文件上限 2 MiB。');const content=await file.text();await importProfile(file.name.toLowerCase().endsWith('.json')?JSON.parse(content):content,!file.name.toLowerCase().endsWith('.json'));$('import').value='';});
$('apply-xml').onclick=()=>guarded(async()=>{const candidate=payload();candidate.kinematics.urdf_xml=$('xml').value;const doc=parse(candidate.kinematics.urdf_xml),missing=missingDrives(doc);candidate.dynamics.joints=Object.fromEntries(Object.keys(missing).map(n=>[n,candidate.dynamics.joints[n]||missing[n]]));await importProfile(candidate);});
$('robot-id').onchange=()=>edited();$('part').onchange=()=>{buildEditor();draw();};
$('joint-tab').onclick=()=>{mode='joint';rebuildParts();draw();};$('link-tab').onclick=()=>{mode='link';rebuildParts();draw();};
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{view.yaw=b.dataset.view==='front'?Math.PI/2:b.dataset.view==='side'?0:.75;view.pitch=b.dataset.view==='iso'?.48:0;draw();});$('fit').onclick=()=>{fitView();draw();};
$('solid-view').onclick=()=>{if(sceneEngine)sceneEngine.wireframe=false;$('solid-view').setAttribute('aria-pressed','true');$('wireframe-view').setAttribute('aria-pressed','false');draw();};
$('wireframe-view').onclick=()=>{if(sceneEngine)sceneEngine.wireframe=true;$('solid-view').setAttribute('aria-pressed','false');$('wireframe-view').setAttribute('aria-pressed','true');draw();};
let drag;const canvas=$('scene');canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,sx:e.clientX,sy:e.clientY};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag)return;view.yaw+=(e.clientX-drag.x)*.008;view.pitch=Math.max(-1.3,Math.min(1.3,view.pitch+(e.clientY-drag.y)*.006));drag.x=e.clientX;drag.y=e.clientY;draw();});
canvas.addEventListener('pointerup',e=>{if(drag&&Math.hypot(e.clientX-drag.sx,e.clientY-drag.sy)<5){const r=canvas.getBoundingClientRect(),p=[e.clientX-r.left,e.clientY-r.top];const hit=sceneEngine?.pick(...p);const nearest=hit||projected.map(item=>({...item,d:Math.hypot(item.p[0]-p[0],item.p[1]-p[1])})).sort((a,b)=>a.d-b.d)[0];if(nearest&&(hit||nearest.d<18)){mode=nearest.type;rebuildParts(nearest.name);draw();}}drag=null;});canvas.addEventListener('pointercancel',()=>drag=null);canvas.addEventListener('wheel',e=>{e.preventDefault();view.zoom=Math.max(.3,Math.min(5,view.zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
guarded(async()=>{
  try{const {RobotScene}=await import('/robot-scene.js');sceneEngine=new RobotScene(canvas);}
  catch(error){console.warn('实体渲染不可用，使用兼容线框视图。',error);canvas.dataset.renderer='canvas2d';$('solid-view').disabled=true;$('solid-view').setAttribute('aria-pressed','false');$('wireframe-view').setAttribute('aria-pressed','true');}
  new ResizeObserver(draw).observe(canvas);
  await loadExample();
});
