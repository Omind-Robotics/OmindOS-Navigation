'use strict';
const $ = id => document.getElementById(id);
let xml, profile, model, currentPose, targets = {}, mode = 'joint', revision = 0, readyRevision = -1;
let timer, animation, latestTrial, view = {yaw: .75, pitch: .48, zoom: 1, center: [0, 0, .5], span: 1.5};
let projected = [], operation = false;
let sceneEngine = null;
let planningJob = null, planningStart = null;
let baseTarget = {x_m:0, y_m:0, yaw_rad:0};
let emergencyStop = false, safetyPending = false, safetyRequest = 0, motionEpoch = -1, motionTicket = 0;
let connectionOptions, connectionDraft = {schema_version:1,devices:{}};
const connectionInvalid = new Set();
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
function payload() { if (!profile || !xml) throw Error('请先导入模型。'); if (connectionInvalid.size||document.querySelector('#editor [aria-invalid="true"]')) throw Error('请先修正标记为无效的参数。'); profile.robot_id = $('robot-id').value; profile.kinematics.urdf_xml = serializer.serializeToString(xml); return structuredClone(profile); }
async function api(path, data) {
  const response = await fetch(path, data === undefined ? {} : {method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(data)});
  const body = await response.json(); if (!response.ok) throw Error(body.error || `请求失败 ${response.status}`); return body;
}
function stop(message) {
  motionTicket++;
  if(planningJob){const id=planningJob;planningJob=null;api('/v1/planning-cancel',{task_id:id}).catch(()=>{});$('planning-cancel').disabled=true;}
  cancelAnimationFrame(animation); animation = null; $('stop').disabled = true;
  updateWheelSpeeds();
  if (message) status(message);
}
function edited({refresh = true, config = true} = {}) {
  if(config)planningStart=null;
  stop(); revision++; readyRevision = -1; latestTrial = null; $('results').replaceChildren(); $('progress').value = 0;
  if(config){$('dirty').textContent = '有修改 · 未保存';$('apply-note').textContent='参数有修改，点击写入后作为程序当前配置。';} $('xml').value = serializer.serializeToString(xml);
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
    updateModelInfo();
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
  const result = await api('/v1/robot-model', {urdf_xml:candidate.kinematics.urdf_xml,connections:candidate.connections});
  if (fromUrdf) candidate.dynamics.joints = missingDrives(doc);
  stop(); clearTimeout(timer); revision++; readyRevision = revision; latestTrial = null;
  profile = candidate; xml = doc; model = result.model; currentPose = result.pose; targets = {...result.pose.positions};
  planningStart=null;
  connectionInvalid.clear();connectionDraft=structuredClone(candidate.connections||{schema_version:1,devices:{}});renderConnections();
  baseTarget = {...result.pose.base_pose};
  document.querySelectorAll('.base-fields input').forEach(input=>input.removeAttribute('aria-invalid'));
  $('robot-id').value = candidate.robot_id || ''; $('xml').value = candidate.kinematics.urdf_xml;
  $('dirty').textContent = '未保存'; $('model-name').textContent = model.name;
  updateModelInfo();
  $('warnings').textContent = model.warnings.join('；'); $('results').replaceChildren(); $('progress').value = 0;
  $('profile-id').value='';$('saved').textContent='参数保存在本机服务目录；旧版本保留。';
  $('apply-note').textContent='当前编辑配置尚未写入程序，点击写入后生效。';
  syncQuickControls(); rebuildParts(); fitView(); draw();
  status(fromUrdf ? '模型已导入。请补充各活动关节的电机参数后试运行、保存。' : '模型已载入 · 可调整参数');
}
function textBlock(text, cls = 'note') { const el = document.createElement('div'); el.className = cls; el.textContent = text; $('editor').append(el); return el; }
function connectionChanged(){
  const wasReady=readyRevision===revision;stop();freezePose();clearTimeout(timer);
  profile.connections=structuredClone(connectionDraft);edited({refresh:false});
  if(wasReady)readyRevision=revision;
  $('connection-state').textContent='通信配置已修改 · 尚未写入程序';
}
function connectionRecord(device){
  return connectionDraft.devices[device] ||= {selected:'unselected',profiles:{}};
}
function renderConnections(){
  if(!connectionOptions)return;
  const device=$('connection-device').value,spec=connectionOptions.devices[device];
  const saved=connectionDraft.devices[device]||{selected:'unselected',profiles:{}};
  const select=$('connection-protocol');select.replaceChildren();
  for(const [key,label] of [['unselected','待选择'],...Object.entries(spec.protocols).map(([k,v])=>[k,v.label])]){
    const option=document.createElement('option');option.value=key;option.textContent=label;select.append(option);
  }
  select.value=saved.selected;$('connection-fields').replaceChildren();
  const protocol=spec.protocols[saved.selected];
  $('connection-note').textContent=protocol?.note||'先保存设备配置，连接时再选择通信方式。';
  $('connection-state').textContent='仅保存配置 · 驱动未接入';
  if(!protocol)return;
  for(const field of protocol.fields){
    const label=document.createElement('label');label.textContent=field.label;
    const input=document.createElement(field.type==='choice'?'select':'input');input.id=`connection-${device}-${field.key}`;
    if(field.type==='choice'){
      const translations={none:'无校验',even:'偶校验',odd:'奇校验',tcp:'TCP',tls:'TLS',rtu:'RTU',standard_rs232:'标准 RS-232',ttl:'TTL'};
      for(const value of ['',...field.choices]){const option=document.createElement('option');option.value=value;option.textContent=value?(translations[value]||value):'待填写';input.append(option);}
    }else{input.type=field.type==='integer'?'number':'text';input.autocomplete='off';input.placeholder=field.placeholder||'待填写';
      if(field.type==='integer'){input.min=field.min;input.max=field.max;input.step='1';}else input.maxLength=256;}
    input.value=saved.profiles[saved.selected]?.[field.key]??'';
    const invalidKey=`${device}/${saved.selected}/${field.key}`;
    if(connectionInvalid.has(invalidKey))input.setAttribute('aria-invalid','true');
    input.oninput=()=>{
      const record=connectionRecord(device),values=record.profiles[record.selected]||={};
      const raw=input.value;let value=raw;
      if(field.type==='integer')value=raw===''?null:Number(raw);
      const valid=input.validity.valid && (field.type!=='integer'||value===null||(Number.isInteger(value)&&value>=field.min&&value<=field.max));
      if(valid){input.removeAttribute('aria-invalid');connectionInvalid.delete(invalidKey);}
      else{input.setAttribute('aria-invalid','true');connectionInvalid.add(invalidKey);}
      values[field.key]=value;connectionChanged();
    };
    label.append(input);$('connection-fields').append(label);
  }
}
$('connection-device').onchange=renderConnections;
$('connection-protocol').onchange=()=>{
  const record=connectionRecord($('connection-device').value);record.selected=$('connection-protocol').value;
  renderConnections();connectionChanged();
};
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
  syncJogControls(); updateMotionControls();
}
function armJoints(side){return (model?.joints||[]).filter(j=>j.type!=='fixed' && j.name.startsWith(side+'_') && !/wheel|finger|gripper|hand|waist|neck|head/i.test(j.name)).map(j=>j.name);}
function jointGroups(){
  const names=(model?.joints||[]).filter(j=>j.type!=='fixed').map(j=>j.name);
  return {body:names.filter(n=>!/wheel|finger|gripper/i.test(n)),grippers:names.filter(n=>/finger|gripper/i.test(n)),wheels:names.filter(n=>/wheel/i.test(n))};
}
function updateModelInfo(){
  const groups=jointGroups(),left=armJoints('left').length,right=armJoints('right').length;
  $('model-info').textContent=`${groups.body.length} 个主体关节`+(left||right?` · 左臂 ${left} / 右臂 ${right}`:'')+(groups.grippers.length||groups.wheels.length?' · 夹爪与轮轴另列':'');
  const base=model?.wheel_base;
  $('chassis-info').textContent=base?`四轮差速参考模型 · 轮距 ${base.track_width_m.toFixed(3)} m · 实机轮型待确认`:'当前模型未配置四轮底盘，试跑不可用。';
  updateWheelSpeeds();
}
function jointLabel(name){const wheel=name.match(/^(left|right)_(front|rear)_wheel_joint$/);if(wheel)return `${wheel[1]==='left'?'左':'右'}${wheel[2]==='front'?'前':'后'}轮`;const m=name.match(/^(left|right)_joint_(\d+)$/);return m?`${m[1]==='left'?'左臂':'右臂'} J${m[2]}`:name==='waist_pitch'?'腰部 pitch':name;}
function updateWheelSpeeds(rates={}){
  $('wheel-speeds').replaceChildren(...Object.keys(model?.wheel_base?.wheels||{}).map(name=>{
    const row=document.createElement('div');row.className='metric';const label=document.createElement('span'),value=document.createElement('output');
    label.textContent=jointLabel(name);value.dataset.wheel=name;value.textContent=((rates[name]||0)*30/Math.PI).toFixed(2)+' rpm';row.append(label,value);return row;
  }));
}
function syncJogControls(){
  const select=$('jog-joint'), groups=jointGroups(),names=Object.values(groups).flat();
  if([...select.options].map(o=>o.value).join('|')!==names.join('|')){
    const previous=select.value;select.replaceChildren(...Object.entries(groups).filter(([,items])=>items.length).map(([key,items])=>{
      const group=document.createElement('optgroup');group.label={body:'主体关节',grippers:'夹爪',wheels:'底盘轮轴'}[key];
      for(const name of items){const option=document.createElement('option');option.value=name;option.textContent=jointLabel(name);group.append(option);}return group;
    }));
    if(names.includes(previous))select.value=previous;
  }
  updateJogAngle();
}
function updateJogAngle(){const q=currentPose?.positions[$('jog-joint').value];$('jog-angle').textContent=Number.isFinite(q)?(q*180/Math.PI).toFixed(3)+'°':'—';}
function updateMotionControls(){
  const locked=emergencyStop||safetyPending||!model;
  document.body.dataset.estop=String(emergencyStop);
  document.querySelectorAll('[data-motion],#pitch,#left-gripper,#right-gripper,#base-x,#base-y,#base-yaw,#reset-pose,#editor input[type=range]').forEach(el=>el.disabled=locked);
  for(const button of document.querySelectorAll('[data-sine]')){
    const group=button.dataset.sine;
    if(group==='left'||group==='right')button.disabled=locked||!armJoints(group).length;
    if(group==='both')button.disabled=locked||!armJoints('left').length||!armJoints('right').length;
  }
  document.querySelectorAll('[data-drive]').forEach(button=>button.disabled=locked||!model?.wheel_base);
  $('cooperative-demo').disabled=locked||armJoints('left').length!==7||armJoints('right').length!==7||!model?.wheel_base;
  $('reset-stop').disabled=!emergencyStop||safetyPending;
  $('emergency-state').textContent=emergencyStop?'已急停 · 运动锁定，需手动复位':'电脑端软件急停 · 真机未连接';
}
function freezePose(){
  if(!currentPose)return;targets={...currentPose.positions};baseTarget={...currentPose.base_pose};syncQuickControls();
  const slider=$('editor').querySelector('input[type=range]');
  if(slider && mode==='joint' && targets[$('part').value]!==undefined){slider.value=targets[$('part').value];slider.parentElement.querySelector('output').textContent=targets[$('part').value].toFixed(3);}
}
function latchStop(message='已急停 · 运动已锁定'){
  const wasReady=readyRevision===revision;stop();clearTimeout(timer);revision++;readyRevision=wasReady?revision:-1;
  document.body.dataset.pending='false';emergencyStop=true;freezePose();updateMotionControls();status(message,true);
}
async function engageStop(){
  const request=++safetyRequest;safetyPending=true;latchStop();
  try{const state=await api('/v1/workbench-stop',{engaged:true});if(request===safetyRequest)motionEpoch=Math.max(motionEpoch,state.motion_epoch);}
  catch(e){if(request===safetyRequest)status('本页已停止；服务急停确认失败：'+e.message,true);}
  finally{if(request===safetyRequest){safetyPending=false;updateMotionControls();}}
}
$('emergency-stop').onclick=engageStop;
document.addEventListener('keydown',e=>{if(e.key==='Escape'){e.preventDefault();engageStop();}});
$('reset-stop').onclick=async()=>{
  if(safetyPending)return;const request=++safetyRequest;safetyPending=true;updateMotionControls();
  try{const state=await api('/v1/workbench-stop',{engaged:false,expected_epoch:motionEpoch});
    if(request!==safetyRequest)return;motionEpoch=state.motion_epoch;emergencyStop=false;status('已复位 · 急停已解除，请重新选择并启动运动');
  }catch(e){if(request===safetyRequest)status(e.message,true);}
  finally{if(request===safetyRequest){safetyPending=false;updateMotionControls();}}
};
async function pollSafety(){
  try{const state=await api('/v1/workbench-state');
    if(!safetyPending && state.motion_epoch>=motionEpoch){
      const changed=motionEpoch>=0 && state.motion_epoch!==motionEpoch;motionEpoch=state.motion_epoch;
      if(state.emergency_stop&&!emergencyStop)latchStop('服务已急停 · 运动已锁定');
      else if(changed && !emergencyStop){stop();freezePose();status('程序状态已改变 · 本次运动已停止');}
    }
  }catch(e){if(!emergencyStop)latchStop('服务连接中断 · 运动已停止，恢复后请手动复位');}
  finally{setTimeout(pollSafety,750);}
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
      const slider = document.createElement('input'); slider.type = 'range'; slider.min = low; slider.max = high; slider.step = 'any'; slider.value = targets[name] ?? 0; slider.setAttribute('aria-label','目标关节角');
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
      const dyn = child(node,'dynamics'); textBlock('物理阻尼与摩擦', 'section-label');
      for (const [key,title] of [['damping','阻尼 / N·m·s/rad'],['friction','摩擦 / N·m']]) field($('editor'), title, dyn?.getAttribute(key) || 0, v => ensure(node,'dynamics').setAttribute(key,v), {min:'0'});
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
      else textBlock('厂家网格保持原连杆坐标；未知资源以骨架显示。');
      vectorFields($('editor'),'外形相对连杆偏移 / m',ensure(visual,'origin'),'xyz');
    });
    textBlock('外形尺寸只改变显示几何；关节连接距离在“关节”页调整。');
  }
  if(mode==='joint' && currentPose?.positions[$('part').value]!==undefined)$('jog-joint').value=$('part').value;
  updateJogAngle();updateMotionControls();
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
  $('scene-phase').textContent=currentPose?.phase||'交互预览';
  if(currentPose)$('scene-base').textContent=`底盘 X ${currentPose.base_pose.x_m.toFixed(2)} m · Y ${currentPose.base_pose.y_m.toFixed(2)} m · 转向 ${(currentPose.base_pose.yaw_rad*180/Math.PI).toFixed(1)}° · 参考运动学`;
  $('selected-kind').textContent = mode === 'joint' ? '当前关节' : '当前连杆';
  $('selected-part').textContent = $('part').value || '—';
  if(view.follow && currentPose){view.center[0]=view.followOffset[0]+currentPose.base_pose.x_m;view.center[1]=view.followOffset[1]+currentPose.base_pose.y_m;}
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
  $('results').replaceChildren();const head=document.createElement('p');head.textContent=result.motion_type==='planned'?`规划通过 · ${result.validated_samples} 个离散碰撞样本`:'运动路径采样点的峰值静态保持力矩';$('results').append(head);
  if(result.motion_type==='planned'){const note=document.createElement('p');note.textContent='底盘固定 · 几何包围盒保守检查 · 不含动力学';$('results').append(note);if(result.max_tcp_position_error_m!==undefined){const error=document.createElement('p');error.textContent=`两手最大位置误差 ${(result.max_tcp_position_error_m*1000).toFixed(3)} mm；相对误差 ${(result.max_relative_position_error_m*1000).toFixed(3)} mm（模型数值）`;$('results').append(error);}return;}
  if(result.actual_amplitude_deg){const note=document.createElement('p');note.textContent='实际正弦振幅：'+Object.entries(result.actual_amplitude_deg).map(([n,v])=>`${n} ${v.toFixed(2)}°`).join('；');$('results').append(note);}
  for(const [name,value] of Object.entries(result.peak_holding_torque_nm)){const row=document.createElement('div');row.className='metric';const label=document.createElement('span');label.textContent=name;const val=document.createElement('strong');val.textContent=`${value.toFixed(2)} N·m`;if(result.gravity_limit_exceeded.includes(name))val.className='error';row.append(label,val);$('results').append(row);}
  if(result.gravity_limit_exceeded.length){const warn=document.createElement('p');warn.className='error';warn.textContent='静态重力负载超限：'+result.gravity_limit_exceeded.join('、');$('results').append(warn);}
}
async function guarded(fn){if(operation)return;operation=true;try{await fn();}catch(e){status(e.message,true);}finally{operation=false;}}
async function loadExample(){
  await importProfile(await api('/example.json'));
  if(!model.joints.some(j=>j.name==='waist_pitch')){
    mode='joint';rebuildParts();await refreshModel(true);status('四足教学示例已载入 · 12关节；示意参数，非实机标定');return;
  }
  targets.waist_pitch=0;
  for(const [side,q] of [['left',[1.57,1.30,-1.57,1.10,.10,.50,0]],['right',[-1.57,1.30,1.57,1.10,-.10,.50,0]]]){
    q.forEach((v,i)=>targets[`${side}_joint_${i+1}`]=v);
    for(const name of fingerNames(side))targets[name]=side==='left'?.4:.23;
  }
  mode='joint';rebuildParts('waist_pitch');await refreshModel(true);status('教学示例已载入 · 双臂各7轴 + 腰部pitch，夹爪与底盘独立控制');
}
async function motionReady(){
  if(emergencyStop||safetyPending)throw Error('急停锁定中，请先手动复位。');
  if(animation){stop();freezePose();}
  const ticket=motionTicket;
  clearTimeout(timer);if(readyRevision!==revision)await refreshModel(false);if(readyRevision!==revision)throw Error('模型参数有误，请先修正。');
  if(ticket!==motionTicket)throw Error('运动已取消，请重新选择并启动。');
  if(emergencyStop||safetyPending)throw Error('急停锁定中，请先手动复位。');
}
async function executeMotion(path,data,label,validatedResult=null){
  stop();$('progress').value=0;const rev=revision,ticket=motionTicket;status('正在计算运动轨迹和重力负载…');
  const result=validatedResult||await api(path,data);
  if(rev!==revision||ticket!==motionTicket||emergencyStop||safetyPending||result.motion_epoch<motionEpoch)return;
  motionEpoch=result.motion_epoch;latestTrial=result;report(result);model=result.model;
  if(result.motion_type==='sine'){
    const step=(result.frames.length-1)/result.cycles;
    fitView([result.frames[0],result.frames[Math.round(step/4)],result.frames[Math.round(3*step/4)]]);
    $('sine-info').textContent=result.amplitude_limited_joints.length?'限位已缩小振幅：'+result.amplitude_limited_joints.join('、'):'围绕当前姿态往复，完成后回到起点。';
  }else if(result.motion_type!=='jog')fitView([result.frames[0],result.frames.at(-1)]);
  sceneEngine?.resetToolPaths();
  if(view.follow)view.followOffset=[view.center[0]-currentPose.base_pose.x_m,view.center[1]-currentPose.base_pose.y_m];
  const start=performance.now();$('stop').disabled=false;
  const tick=now=>{
    if(ticket!==motionTicket||emergencyStop)return;
    // A frame timestamp can precede performance.now() when scheduled in the same frame.
    const elapsed=Math.max(0,(now-start)/1000);
    let low=0,high=result.frames.length-1;
    while(low<high){const mid=Math.ceil((low+high)/2);if(result.frames[mid].time_s<=elapsed)low=mid;else high=mid-1;}
    const index=low;
    const frame=result.frames[index],next=result.frames[Math.min(index+1,result.frames.length-1)];
    const fraction=next.time_s>frame.time_s?Math.max(0,Math.min(1,(elapsed-frame.time_s)/(next.time_s-frame.time_s))):0;
    currentPose=sceneEngine?sceneEngine.interpolatePose(model,frame,next,fraction):frame;
    if(result.phases)$('sequence-phase').textContent=frame.phase;
    $('progress').value=Math.min(1,elapsed/result.duration_s);updateJogAngle();updateWheelSpeeds(currentPose.wheel_velocity_rad_s);draw();
    status(`${label} ${Math.min(elapsed,result.duration_s).toFixed(1)} / ${result.duration_s.toFixed(1)} 秒`);
    if(elapsed<result.duration_s)animation=requestAnimationFrame(tick);
    else{stop();freezePose();status(result.gravity_limit_exceeded.length?`${label}结束 · 发现静态重力负载超限`:`${label}完成 · 仅运动学预览`);}
  };animation=requestAnimationFrame(tick);
}
$('cooperative-demo').onclick=()=>guarded(async()=>{
  await motionReady(); const demo=await api('/cooperative-demo.json');
  await executeMotion('/v1/robot-sequence',{profile:payload(),start:currentPose.positions,base_pose:currentPose.base_pose,
    segments:demo.segments,reference_signature:demo.reference_signature},'双臂协同演示');
});
async function runPlanning(mode){
  await motionReady();stop();const ticket=motionTicket,rev=revision;
  const start=mode==='joint'?planningStart:currentPose;
  if(!start)throw Error('请先记下规划起点，再调整主体关节到目标姿态。');
  if(mode==='joint'&&Object.keys(baseTarget).some(k=>Math.abs(baseTarget[k]-start.base_pose[k])>1e-9))throw Error('规划时底盘必须固定，请保持起点底盘位姿。');
  const mapping=JSON.parse($('planning-mapping').value),obstacles=JSON.parse($('planning-obstacles').value);
  const data={profile:payload(),start:start.positions,base_pose:start.base_pose,mapping,obstacles,mode};
  if(mode==='joint')data.target=targets;
  else{const ids=['planning-x','planning-y','planning-z'];if(ids.some(id=>!$(id).checkValidity()||!$(id).value.trim()))throw Error('请填写有效的共同位移。');data.translation_m=ids.map(id=>numeric($(id).value));}
  localStorage.setItem('omindos-planning-config',JSON.stringify({mapping,obstacles}));
  $('planning-status').textContent='正在计算 · 最多 60 秒，可取消';
  const task=await api('/v1/planning-start',data);
  if(ticket!==motionTicket||rev!==revision){await api('/v1/planning-cancel',{task_id:task.task_id});return;}
  planningJob=task.task_id;$('planning-cancel').disabled=false;
  try{
    while(planningJob===task.task_id&&ticket===motionTicket&&rev===revision){
      const state=await api('/v1/planning-task/'+task.task_id);
      if(state.status==='running'){await new Promise(resolve=>setTimeout(resolve,300));continue;}
      if(ticket!==motionTicket||rev!==revision||planningJob!==task.task_id)return;
      planningJob=null;$('planning-cancel').disabled=true;
      if(state.status!=='succeeded')throw Error(state.error||'规划未通过');
      if(emergencyStop||state.motion_epoch<motionEpoch)throw Error('规划状态已改变，请重新规划。');
      $('planning-status').textContent=`规划通过 · ${state.result.validated_samples} 个离散样本 · 播放六秒`;
      await executeMotion(null,null,mode==='joint'?'关节路径规划':'双臂与腰部协同规划',state.result);
      return;
    }
  }catch(e){$('planning-status').textContent=e.message;throw e;}
  finally{if(planningJob===task.task_id){stop();}$('planning-cancel').disabled=true;}
}
$('planning-start-pose').onclick=()=>guarded(async()=>{await motionReady();stop();planningStart=structuredClone(currentPose);$('planning-status').textContent='已记录起点；调整主体关节后点击规划到目标姿态。';});
$('planning-joint').onclick=()=>guarded(()=>runPlanning('joint'));
$('planning-object').onclick=()=>guarded(()=>runPlanning('object'));
$('planning-cancel').onclick=()=>{stop('规划已取消');$('planning-status').textContent='规划已取消，保留当前姿态。';};
for(const id of ['planning-mapping','planning-obstacles','planning-x','planning-y','planning-z'])$(id).oninput=()=>{stop();$('planning-status').textContent='规划输入已修改，重新计算后生效。';};
$('planning-export').onclick=()=>guarded(async()=>{const config={mapping:JSON.parse($('planning-mapping').value),obstacles:JSON.parse($('planning-obstacles').value)};const url=URL.createObjectURL(new Blob([JSON.stringify(config,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='OmindOS-Planning-Config.json';a.click();URL.revokeObjectURL(url);});
$('planning-import').onchange=()=>guarded(async()=>{const file=$('planning-import').files[0];if(!file)return;if(file.size>65536)throw Error('规划配置不能超过 64 KiB。');const config=JSON.parse(await file.text());if(!config.mapping||!Array.isArray(config.obstacles))throw Error('需要 mapping 和 obstacles。');stop();planningStart=null;$('planning-mapping').value=JSON.stringify(config.mapping,null,2);$('planning-obstacles').value=JSON.stringify(config.obstacles,null,2);localStorage.setItem('omindos-planning-config',JSON.stringify(config));$('planning-status').textContent='客户规划配置已导入，将使用当前模型校验。';});
(async()=>{try{const saved=localStorage.getItem('omindos-planning-config');const config=saved?JSON.parse(saved):await api('/v1/planning-config');$('planning-mapping').value=JSON.stringify(config.mapping,null,2);$('planning-obstacles').value=JSON.stringify(config.obstacles||[],null,2);}catch(e){$('planning-status').textContent=e.message;}})();
$('run').onclick=()=>guarded(async()=>{await motionReady();await executeMotion('/v1/robot-preview',{profile:payload(),target:targets,base_target:baseTarget,duration_s:numeric($('duration').value)},'运动学试运行');});
for(const button of document.querySelectorAll('[data-drive]'))button.onclick=()=>guarded(async()=>{
  await motionReady();
  for(const id of ['drive-linear','drive-angular','drive-duration'])if(!$(id).checkValidity()||!$(id).value.trim())throw Error('请填写有效的底盘速度和试跑时长。');
  const action=button.dataset.drive,linear=numeric($('drive-linear').value),angular=numeric($('drive-angular').value)*Math.PI/180;
  await executeMotion('/v1/robot-wheel-base',{profile:payload(),start:currentPose.positions,base_pose:currentPose.base_pose,
    linear_m_s:action==='forward'?linear:action==='backward'?-linear:0,
    angular_rad_s:action==='left'?angular:action==='right'?-angular:0,duration_s:numeric($('drive-duration').value)},'底盘'+button.textContent);
});
$('chassis-stop').onclick=()=>{stop('已停车 · 保持当前位置和轮轴角度');freezePose();};
$('jog-joint').onchange=()=>{mode='joint';rebuildParts($('jog-joint').value);draw();};
for(const [id,direction] of [['jog-minus',-1],['jog-plus',1]])$(id).onclick=()=>guarded(async()=>{
  await motionReady();await executeMotion('/v1/robot-jog',{profile:payload(),start:currentPose.positions,joint:$('jog-joint').value,direction,base_pose:currentPose.base_pose},'单关节点动');
});
for(const button of document.querySelectorAll('[data-sine]'))button.onclick=()=>guarded(async()=>{
  await motionReady();const group=button.dataset.sine;
  const joints=group==='all'?jointGroups().body:group==='both'?[...armJoints('left'),...armJoints('right')]:armJoints(group);
  await executeMotion('/v1/robot-sine',{profile:payload(),start:currentPose.positions,joints,amplitude_deg:numeric($('sine-amplitude').value),period_s:numeric($('sine-period').value),cycles:numeric($('sine-cycles').value),base_pose:currentPose.base_pose},button.textContent);
});
$('stop').onclick=()=>{stop('已停止预览 · 未发送真机指令');freezePose();};
$('apply-program').onclick=()=>guarded(async()=>{
  stop();const rev=revision;const result=await api('/v1/workbench-active-profile',payload());motionEpoch=Math.max(motionEpoch,result.motion_epoch);
  $('profile-id').value=result.profile_id;$('saved').textContent='程序当前配置：'+result.profile_id;
  $('apply-note').textContent=rev===revision?'已写入程序 · 重启后自动加载此配置':'写入完成，当前编辑内容另有修改。';
  if(rev===revision)$('connection-state').textContent='通信配置已写入 · 驱动未接入';
  if(rev===revision)$('dirty').textContent='参数已写入程序';status('参数已写入工作台程序 · 未接入真机控制器');
});
$('save').onclick=()=>guarded(async()=>{const rev=revision;const result=await api('/v1/robot-profiles',payload());$('profile-id').value=result.profile_id;$('saved').textContent='已保存配置：'+result.profile_id;if(rev===revision)$('dirty').textContent='参数已保存';status(rev===revision?'配置已保存，旧版本保留；未应用到控制器。':'修改前的参数已保存，当前编辑内容尚未保存。');});
$('load').onclick=()=>guarded(async()=>{const id=$('profile-id').value.trim();if(!/^[0-9a-f]{64}$/.test(id))throw Error('请输入完整的 64 位配置编号。');await importProfile(await api('/v1/robot-profiles/'+id));$('profile-id').value=id;$('saved').textContent='当前读取版本：'+id;$('dirty').textContent='已读取保存版本';status('已读取保存版本 · 尚未应用到控制器');});
$('export').onclick=()=>{try{const data=payload(),url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='robot-parameters.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);status('已导出编辑内容；保存版本时会进行完整参数校验。');}catch(e){status(e.message,true);}};
$('example').onclick=()=>guarded(loadExample);
$('import').onchange=()=>guarded(async()=>{const file=$('import').files[0];if(!file)return;if(file.size>2*1024*1024)throw Error('文件上限 2 MiB。');const content=await file.text();await importProfile(file.name.toLowerCase().endsWith('.json')?JSON.parse(content):content,!file.name.toLowerCase().endsWith('.json'));$('import').value='';});
$('apply-xml').onclick=()=>guarded(async()=>{const candidate=payload();candidate.kinematics.urdf_xml=$('xml').value;const doc=parse(candidate.kinematics.urdf_xml),missing=missingDrives(doc);candidate.dynamics.joints=Object.fromEntries(Object.keys(missing).map(n=>[n,candidate.dynamics.joints[n]||missing[n]]));await importProfile(candidate);});
$('robot-id').onchange=()=>edited();$('part').onchange=()=>{buildEditor();draw();};
$('joint-tab').onclick=()=>{mode='joint';rebuildParts();draw();};$('link-tab').onclick=()=>{mode='link';rebuildParts();draw();};
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{view.yaw=b.dataset.view==='front'?Math.PI/2:b.dataset.view==='side'?0:.75;view.pitch=b.dataset.view==='iso'?.48:0;draw();});$('fit').onclick=()=>{view.follow=false;$('follow-base').setAttribute('aria-pressed','false');fitView();draw();};
$('expand-view').onclick=()=>{const active=document.body.classList.toggle('expanded-view');$('expand-view').textContent=active?'恢复面板':'扩大视图';$('expand-view').setAttribute('aria-pressed',String(active));draw();};
$('stage-stop').onclick=engageStop;
$('follow-base').onclick=()=>{view.follow=!view.follow;view.followOffset=[view.center[0]-currentPose.base_pose.x_m,view.center[1]-currentPose.base_pose.y_m];$('follow-base').setAttribute('aria-pressed',String(view.follow));draw();};
$('tool-trails').onclick=()=>{if(sceneEngine){sceneEngine.showTrails=!sceneEngine.showTrails;$('tool-trails').setAttribute('aria-pressed',String(sceneEngine.showTrails));draw();}};
document.querySelectorAll('[data-focus]').forEach(button=>button.onclick=()=>{
  const t=currentPose?.link_transforms[button.dataset.focus+'_hand'];if(!t)return;
  view.follow=false;$('follow-base').setAttribute('aria-pressed','false');view.center=t.slice(0,3).map(r=>r[3]);view.span=.65;view.zoom=1;draw();
});
$('solid-view').onclick=()=>{if(sceneEngine)sceneEngine.wireframe=false;$('solid-view').setAttribute('aria-pressed','true');$('wireframe-view').setAttribute('aria-pressed','false');draw();};
$('wireframe-view').onclick=()=>{if(sceneEngine)sceneEngine.wireframe=true;$('solid-view').setAttribute('aria-pressed','false');$('wireframe-view').setAttribute('aria-pressed','true');draw();};
let drag;const canvas=$('scene');canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,sx:e.clientX,sy:e.clientY};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag)return;view.yaw+=(e.clientX-drag.x)*.008;view.pitch=Math.max(-1.3,Math.min(1.3,view.pitch+(e.clientY-drag.y)*.006));drag.x=e.clientX;drag.y=e.clientY;draw();});
canvas.addEventListener('pointerup',e=>{if(drag&&Math.hypot(e.clientX-drag.sx,e.clientY-drag.sy)<5){const r=canvas.getBoundingClientRect(),p=[e.clientX-r.left,e.clientY-r.top];const hit=sceneEngine?.pick(...p);const nearest=hit||projected.map(item=>({...item,d:Math.hypot(item.p[0]-p[0],item.p[1]-p[1])})).sort((a,b)=>a.d-b.d)[0];if(nearest&&(hit||nearest.d<18)){mode=nearest.type;rebuildParts(nearest.name);draw();}}drag=null;});canvas.addEventListener('pointercancel',()=>drag=null);canvas.addEventListener('wheel',e=>{e.preventDefault();view.zoom=Math.max(.3,Math.min(5,view.zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
updateMotionControls();
guarded(async()=>{
  connectionOptions=await api('/v1/connection-options');
  for(const [key,spec] of Object.entries(connectionOptions.devices)){
    const option=document.createElement('option');option.value=key;option.textContent=spec.label+' · '+spec.model;$('connection-device').append(option);
  }
  $('connection-device').value='chassis';renderConnections();
  try{const {RobotScene}=await import('/robot-scene.js');sceneEngine=new RobotScene(canvas);
    sceneEngine.onAssetsChanged=stats=>{$('mesh-status').textContent=stats.requested?`厂家网格 ${stats.loaded} / ${stats.requested}`+(stats.failed?' · 部分失败，显示骨架':' · 本地加载'):'三维实体预览';canvas.dataset.meshLoaded=String(stats.loaded);canvas.dataset.meshFailed=String(stats.failed);};}
  catch(error){console.warn('实体渲染不可用，使用兼容线框视图。',error);canvas.dataset.renderer='canvas2d';$('solid-view').disabled=true;$('solid-view').setAttribute('aria-pressed','false');$('wireframe-view').setAttribute('aria-pressed','true');}
  new ResizeObserver(draw).observe(canvas);
  const saved=await api('/v1/workbench-active-profile');motionEpoch=saved.motion_epoch;
  if(saved.profile){await importProfile(saved.profile);$('profile-id').value=saved.active_profile_id;$('dirty').textContent='程序当前配置';$('apply-note').textContent='已加载上次写入程序的配置';status('已加载程序当前配置 · 可开始调试');}
  else await loadExample();
  if(saved.emergency_stop)latchStop('服务已急停 · 请手动复位后再运动');
  updateMotionControls();pollSafety();
});


