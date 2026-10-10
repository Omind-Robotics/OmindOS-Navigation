import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';

// The assembly is deliberately independent of the active controller profile.
// It contains saved CAD placements only, with no inferred movable joints.
export async function mountCAD(container) {
  const canvas=container.querySelector('canvas'), status=container.querySelector('#cad-status');
  const response=await fetch('/cad/v11/manifest.json');
  if(!response.ok)throw Error('CAD 信息加载失败');
  const manifest=await response.json();
  const renderer=new THREE.WebGLRenderer({canvas,antialias:true,powerPreference:'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
  renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;
  const scene=new THREE.Scene();scene.background=new THREE.Color('#eef2f6');
  const camera=new THREE.PerspectiveCamera(34,1,.01,30);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,canvas);controls.enableDamping=false;
  controls.minDistance=.3;controls.maxDistance=7;controls.maxPolarAngle=Math.PI*.92;
  scene.add(new THREE.HemisphereLight(0xffffff,0x5d6f86,2.3));
  for(const [position,intensity] of [[[2,-3,4],3],[[-3,1,2],1.3]]){
    const light=new THREE.DirectionalLight(0xffffff,intensity);light.position.set(...position);scene.add(light);
  }
  const {scene:assembly}=await new GLTFLoader().loadAsync('/cad/v11/assembly.glb');
  scene.add(assembly);assembly.updateMatrixWorld(true);
  const parts=new Map(manifest.parts.map(p=>[p.node,p]));
  const meshes=[];
  const selectedMaterial=new THREE.MeshStandardMaterial({color:0x20b5ce,metalness:.25,roughness:.4});
  assembly.traverse(obj=>{if(obj.isMesh){
    if(!obj.geometry.hasAttribute('normal'))obj.geometry.computeVertexNormals();
    let parent=obj;while(parent&&!parts.has(parent.name))parent=parent.parent;
    obj.userData.part=parts.get(parent?.name);meshes.push(obj);
  }});
  const bounds=new THREE.Box3().setFromObject(assembly,true),center=bounds.getCenter(new THREE.Vector3());
  const size=bounds.getSize(new THREE.Vector3());
  const grid=new THREE.GridHelper(3,30,0xc0cbd5,0xdce3e9);grid.rotation.x=Math.PI/2;
  grid.position.set(center.x,center.y,bounds.min.z-.001);scene.add(grid);
  const plane=new THREE.Mesh(new THREE.PlaneGeometry(20,20),new THREE.MeshBasicMaterial({color:0xeef2f6}));
  plane.position.z=bounds.min.z-.002;scene.add(plane);
  let selected=null;
  function render(){if(!container.hidden)renderer.render(scene,camera);}
  function resize(){const rect=canvas.getBoundingClientRect();if(rect.width<1||rect.height<1)return;
    renderer.setSize(rect.width,rect.height,false);camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();render();}
  new ResizeObserver(resize).observe(canvas);
  function fit(view='iso'){
    controls.target.copy(center);
    const direction={iso:[1.35,-1.5,.95],front:[1,0,.12],side:[0,-1,.12],top:[.001,0,1]}[view];
    const aspect=Math.max(.3,canvas.clientWidth/canvas.clientHeight);
    const distance=Math.max(size.x,size.y,size.z)/Math.tan(THREE.MathUtils.degToRad(camera.fov/2))* .54/Math.min(1,aspect);
    camera.position.copy(center).add(new THREE.Vector3(...direction).normalize().multiplyScalar(distance));
    controls.update();render();
  }
  let paletteMaterials=[];
  function palette(value){
    for(const material of paletteMaterials)material.dispose();
    const materials=new Map();
    for(const mesh of meshes){
      const name=mesh.userData.part?.name||'';
      let color=value==='blue'?0x7182ad:0x8997a7;
      if(/球|足垫|橡胶|脚垫/.test(name))color=0x303b46;
      else if(/电机|减速|轴承/.test(name))color=value==='blue'?0x98a8c5:0xaab6c0;
      else if(/盖板|仓|壳|罩/.test(name))color=value==='blue'?0x5b719b:0x465b70;
      if(value==='cad')color=0x888eb3;
      if(!materials.has(color))materials.set(color,new THREE.MeshStandardMaterial({color,roughness:.44,metalness:.32}));
      mesh.material=materials.get(color);
      mesh.userData.baseMaterial=mesh.material;
    }
    paletteMaterials=[...materials.values()];
    if(selected)selected.material=selectedMaterial;
    render();
  }
  controls.addEventListener('change',render);
  const raycaster=new THREE.Raycaster();let pointer;
  canvas.addEventListener('pointerdown',e=>pointer=[e.clientX,e.clientY]);
  canvas.addEventListener('pointerup',e=>{
    if(!pointer||Math.hypot(e.clientX-pointer[0],e.clientY-pointer[1])>5)return;
    const rect=canvas.getBoundingClientRect();
    raycaster.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,1-(e.clientY-rect.top)/rect.height*2),camera);
    const hit=raycaster.intersectObjects(meshes,false)[0];
    if(selected)selected.material=selected.userData.baseMaterial;
    selected=hit?.object||null;
    if(selected)selected.material=selectedMaterial;
    const record=selected?.userData.part;
    container.querySelector('#cad-part-name').textContent=record?.name.split('/').pop()||'点击模型查看零件';
    container.querySelector('#cad-part-path').textContent=record?.name||'保留原始装配层级与零件名称';
    render();
  });
  container.querySelectorAll('[data-cad-view]').forEach(button=>button.onclick=()=>fit(button.dataset.cadView));
  container.querySelector('#cad-palette').onchange=e=>palette(e.target.value);
  container.querySelector('#cad-fit').onclick=()=>fit();
  container.querySelector('#cad-size').textContent=manifest.size_m.map(n=>Math.round(n*1000)).join(' × ')+' mm';
  container.querySelector('#cad-count').textContent=manifest.visible_instances+' 个零件实例';
  container.querySelector('#cad-exclusions').textContent=manifest.excluded.length
    ?'源文件中 '+manifest.excluded.length+' 个异常几何实例未显示，详见模型来源。':'全部零件实例已加载';
  container.querySelector('#cad-source-details').textContent=
    '来源：'+manifest.source_filename+'。显示资源 '+(manifest.glb_bytes/1048576).toFixed(1)+' MiB。'+
    '配色仅用于显示。此处外廓为当前装配姿态的网格测量。'+
    (manifest.excluded.length?' 排除项：'+manifest.excluded.map(x=>x.name).join('；'):'');
  palette('graphite');resize();fit();
  status.textContent='装配加载完成 · 可旋转、缩放、选择零件';
  canvas.dataset.loaded='true';canvas.dataset.parts=String(meshes.length);
  canvas.dataset.bounds=JSON.stringify([bounds.min.toArray(),bounds.max.toArray()]);
  return {resize,render,fit};
}
