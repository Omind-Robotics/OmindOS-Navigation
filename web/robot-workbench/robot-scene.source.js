// Bundled locally with three@0.180.0. See THIRD_PARTY.md for reproduction.
import * as THREE from 'three';
import {RoundedBoxGeometry} from 'three/addons/geometries/RoundedBoxGeometry.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';

const zAxis = new THREE.Vector3(0, 0, 1);
const matrix = rows => new THREE.Matrix4().set(...rows.flat());

export class RobotScene {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({canvas, antialias:true, alpha:true, powerPreference:'low-power'});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75));
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.02;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.VSMShadowMap;
    this.scene = new THREE.Scene();
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, .01, 1000);
    this.camera.up.set(0, 0, 1);
    this.robot = new THREE.Group();
    this.scene.add(this.robot);
    this.raycaster = new THREE.Raycaster();
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    const room = new RoomEnvironment();
    this.environment = pmrem.fromScene(room, .06);
    this.scene.environment = this.environment.texture;
    room.dispose(); pmrem.dispose();
    const ambient = new THREE.HemisphereLight(0xe9f4ff, 0x475269, 1.3);
    ambient.position.set(0, 0, 3); this.scene.add(ambient);
    this.key = new THREE.DirectionalLight(0xfff5e9, 2.1);
    this.key.castShadow = true;
    this.key.shadow.mapSize.set(2048, 2048);
    this.key.shadow.bias = -.0002;
    this.key.shadow.normalBias = .008;
    this.key.shadow.radius = 5;
    this.key.shadow.blurSamples = 8;
    this.scene.add(this.key, this.key.target);
    const fill = new THREE.DirectionalLight(0xbddcff, .8);
    fill.position.set(3, 2, 4); this.scene.add(fill);
    this.floor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.ShadowMaterial({opacity:.16}));
    this.floor.receiveShadow = true; this.scene.add(this.floor);
    this.grid = new THREE.GridHelper(1, 12, 0xc4cfd8, 0xdde4e9);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.material.transparent = true; this.grid.material.opacity = .25;
    this.scene.add(this.grid);
    this.links = new Map(); this.joints = new Map();
    this.meshCache = new Map(); this.meshLoader = new STLLoader(); this.generation = 0;
    this.meshStats = {requested:0, loaded:0, failed:0}; this.onAssetsChanged = null;
    this.toolPaths = new Map(); this.showTrails = true;
    for (const [side, color] of [['left',0x178da9],['right',0xec8a32]]) {
      const path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color, transparent:true, opacity:.7}));
      path.userData.points = []; this.scene.add(path); this.toolPaths.set(side,path);
    }
    this.modelKey = ''; this.lastModel = null; this.resources = [];
    this.wireframe = false;
    canvas.dataset.renderer = 'webgl';
    canvas.addEventListener('webglcontextlost', e => {e.preventDefault(); canvas.dataset.renderer = 'lost';});
    canvas.addEventListener('webglcontextrestored', () => {canvas.dataset.renderer = 'webgl'; this.lastArgs && this.render(...this.lastArgs);});
  }

  own(resource) {this.resources.push(resource); return resource;}

  rebuild(model) {
    this.generation++; this.meshStats = {requested:0, loaded:0, failed:0};
    this.robot.clear(); this.resources.forEach(resource => resource.dispose()); this.resources = [];
    this.links.clear(); this.joints.clear();
    this.shell = this.own(new THREE.MeshStandardMaterial({color:0xdde5ea, roughness:.3, metalness:.35, envMapIntensity:1.05}));
    this.dark = this.own(new THREE.MeshStandardMaterial({color:0x263748, roughness:.32, metalness:.6, envMapIntensity:1.3}));
    this.selected = this.own(new THREE.MeshStandardMaterial({color:0x54c9e5, roughness:.26, metalness:.4, envMapIntensity:1.1}));
    this.jointHighlight = this.own(this.selected.clone());
    this.jointHighlight.depthTest = false; this.jointHighlight.transparent = true; this.jointHighlight.opacity = .9;
    this.jointMaterial = this.own(new THREE.MeshStandardMaterial({color:0x617587, roughness:.25, metalness:.85}));
    this.ringMaterial = this.own(new THREE.MeshStandardMaterial({color:0x3cc6e2, roughness:.27, metalness:.35, emissive:0x16718a, emissiveIntensity:.18}));
    this.bounds = new THREE.Box3();
    const dimensions = model.links.flatMap(link => link.visuals.flatMap(g => g.size || (g.radius ? [g.radius*2] : []))).filter(n => n > 0);
    this.markerSize = Math.max(.002, Math.min(.045, (dimensions.length ? Math.max(...dimensions) : .5)*.065));
    const cylinder = this.own(new THREE.CylinderGeometry(1, 1, 1, 32)); cylinder.rotateX(Math.PI/2);
    const ring = this.own(new THREE.TorusGeometry(1, .10, 10, 40));
    for (const link of model.links) {
      const group = new THREE.Group(); group.matrixAutoUpdate = false;
      const meshes = [];
      for (const g of link.visuals) {
        let geometry;
        if (g.type === 'box') geometry = new RoundedBoxGeometry(...g.size, 3, Math.min(...g.size)*(/head/.test(link.name) ? .42 : /base|torso/.test(link.name) ? .24 : .12));
        else if (g.type === 'sphere') geometry = new THREE.SphereGeometry(g.radius, 32, 20);
        else if (g.type === 'cylinder') {geometry = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32); geometry.rotateX(Math.PI/2);}
        else if (g.type === 'mesh' && g.url) {this.loadMesh(g, link.name, group, meshes); continue;}
        else continue;
        const dark = link.name === model.root || /hand|tip|tool|wheel|finger/i.test(link.name);
        const material = g.color ? this.own(new THREE.MeshStandardMaterial({
          color:new THREE.Color().setRGB(...g.color.slice(0,3)), roughness:/wheel|finger/.test(link.name) ? .55 : .32,
          metalness:/wheel/.test(link.name) ? .15 : .4, opacity:g.color[3], transparent:g.color[3]<1
        })) : dark ? this.dark : this.shell;
        const mesh = new THREE.Mesh(this.own(geometry), material);
        mesh.userData = {name:link.name, type:'link', material};
        mesh.matrixAutoUpdate = false; mesh.matrix.copy(matrix(g.transform));
        mesh.castShadow = true; mesh.receiveShadow = true;
        group.add(mesh); meshes.push(mesh);
      }
      this.links.set(link.name, {group, meshes}); this.robot.add(group);
    }
    for (const joint of model.joints) {
      if (joint.type === 'fixed') continue;
      const group = new THREE.Group();
      const size = this.markerSize * (/finger/.test(joint.name) ? .28 : /wheel/.test(joint.name) ? .55 : 1);
      // These are compact visual joint handles, not additional URDF bodies.
      const cap = new THREE.Mesh(cylinder, this.jointMaterial);
      cap.scale.set(size, size, size*2.2); cap.castShadow = true;
      cap.userData = {name:joint.name, type:'joint'};
      const rim = new THREE.Mesh(ring, this.dark);
      rim.scale.setScalar(size*1.04); rim.position.z = size*1.12;
      rim.userData = {name:joint.name, type:'joint'};
      group.add(cap, rim); this.robot.add(group); this.joints.set(joint.name, {group, cap, rim, joint, manufacturer: /^(left|right)_joint_[1-7]$/.test(joint.name) && model.links.find(l=>l.name===joint.child)?.visuals.some(g=>g.type==='mesh' && g.url)});
    }
    // Retain a structural guide for links whose mesh resources are unavailable.
    const edges = model.joints.filter(joint => !this.links.get(joint.parent).meshes.length || !this.links.get(joint.child).meshes.length);
    const vertices = new Float32Array(edges.length*6);
    const geometry = this.own(new THREE.BufferGeometry()); geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    this.skeleton = new THREE.LineSegments(geometry, this.own(new THREE.LineBasicMaterial({color:0x6ba3b6})));
    this.skeleton.userData.edges = edges; this.robot.add(this.skeleton);
    this.stageNeedsFit = true; this.onAssetsChanged?.({...this.meshStats});
  }

  render(model, pose, view, mode, selectedName) {
    this.lastArgs = [model, pose, view, mode, selectedName];
    const width = this.canvas.clientWidth, height = this.canvas.clientHeight;
    if (!width || !height) return;
    this.renderer.setSize(width, height, false);
    if (!model || !pose) {this.robot.visible = false; this.renderer.render(this.scene, this.camera); return;}
    this.robot.visible = true;
    if (this.lastModel !== model) {
      const key = JSON.stringify(model);
      if (key !== this.modelKey) {this.rebuild(model); this.modelKey = key;}
      this.lastModel = model;
    }
    this.shell.wireframe = this.dark.wireframe = this.selected.wireframe = this.wireframe;
    for (const [name, link] of this.links) {
      link.group.matrix.copy(matrix(pose.link_transforms[name]));
      link.group.matrixWorldNeedsUpdate = true;
      for (const mesh of link.meshes) {
        mesh.userData.material.wireframe = this.wireframe;
        mesh.material = mode === 'link' && selectedName === name ? this.selected : mesh.userData.material;
      }
    }
    for (const [name, {group, cap, rim, joint, manufacturer}] of this.joints) {
      group.position.fromArray(pose.joint_positions[name]);
      const world = matrix(pose.link_transforms[joint.child]);
      const direction = new THREE.Vector3(...joint.axis).transformDirection(world).normalize();
      group.quaternion.setFromUnitVectors(zAxis, direction);
      const selected = mode === 'joint' && name === selectedName;
      group.visible = !manufacturer || selected;
      cap.material = selected ? this.jointHighlight : this.jointMaterial;
      cap.renderOrder = selected ? 2 : 0;
      rim.material = selected ? this.ringMaterial : this.dark;
    }
    const edges = model.joints.filter(j => !this.links.get(j.parent).meshes.length || !this.links.get(j.child).meshes.length);
    if(edges.length !== this.skeleton.userData.edges.length) {
      this.skeleton.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(edges.length*6),3));
      this.skeleton.userData.edges = edges;
    }
    const positions = this.skeleton.geometry.attributes.position;
    this.skeleton.userData.edges.forEach((joint, i) => {
      positions.setXYZ(i*2, ...pose.link_transforms[joint.parent].slice(0,3).map(row => row[3]));
      positions.setXYZ(i*2+1, ...pose.joint_positions[joint.name]);
    }); positions.needsUpdate = true; this.skeleton.geometry.computeBoundingSphere();
    this.updateToolPaths(pose);
    this.robot.updateMatrixWorld(true);
    if (this.stageNeedsFit) {
      this.bounds.setFromObject(this.robot);
      this.floor.position.set(view.center[0], view.center[1], Math.min(0, this.bounds.min.z)-.003);
      this.grid.position.copy(this.floor.position); this.grid.position.z -= .001;
      this.stageNeedsFit = false;
    }
    const span = Math.max(.05, view.span), half = span / (2*view.zoom), aspect = width/height;
    this.camera.left = -half*Math.max(1, aspect); this.camera.right = -this.camera.left;
    this.camera.top = half*Math.max(1, 1/aspect); this.camera.bottom = -this.camera.top;
    const center = new THREE.Vector3(...view.center), distance = span*4;
    const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
    this.camera.position.copy(center).add(new THREE.Vector3(Math.sin(view.yaw)*cp, Math.cos(view.yaw)*cp, sp).multiplyScalar(distance));
    this.camera.near = span*.001; this.camera.far = span*30;
    this.camera.lookAt(center); this.camera.updateProjectionMatrix();
    this.floor.scale.setScalar(span*12); this.grid.scale.setScalar(span*2.3);
    this.key.position.copy(center).add(new THREE.Vector3(-2, -3, 5).multiplyScalar(span));
    this.key.target.position.copy(center);
    Object.assign(this.key.shadow.camera, {left:-span*2, right:span*2, top:span*2, bottom:-span*2, near:span*.05, far:span*15});
    this.key.shadow.camera.updateProjectionMatrix();
    this.key.shadow.normalBias = span*.002;
    this.renderer.render(this.scene, this.camera);
  }

  loadMesh(g, name, group, meshes) {
    // Server URL is an allowlisted same-origin asset, never a filename from an imported file.
    if (!/^\/meshes\/rm75\/(base_link|link_[1-7])\.STL$/.test(g.url)) return;
    const generation = this.generation; this.meshStats.requested++;
    if (!this.meshCache.has(g.url)) {
      this.meshCache.set(g.url, this.meshLoader.loadAsync(g.url).then(geometry => {
        geometry.computeVertexNormals(); geometry.computeBoundingBox(); return geometry;
      }).catch(error => {this.meshCache.delete(g.url); throw error;}));
    }
    this.meshCache.get(g.url).then(geometry => {
      if(generation !== this.generation) return;
      const rgba = g.color || [1,1,1,1];
      const material = this.own(new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(...rgba.slice(0,3)),
        roughness:.32, metalness:.16, envMapIntensity:.85, opacity:rgba[3], transparent:rgba[3]<1}));
      const mesh = new THREE.Mesh(geometry,material);
      mesh.matrixAutoUpdate = false; mesh.matrix.copy(matrix(g.transform).multiply(new THREE.Matrix4().makeScale(...g.scale)));
      mesh.userData = {name, type:'link', material}; mesh.castShadow = mesh.receiveShadow = true;
      group.add(mesh); meshes.push(mesh); this.meshStats.loaded++; this.stageNeedsFit = true;
      this.onAssetsChanged?.({...this.meshStats}); this.lastArgs && this.render(...this.lastArgs);
    }).catch(() => {
      if(generation !== this.generation) return;
      this.meshStats.failed++; this.onAssetsChanged?.({...this.meshStats});
    });
  }

  interpolatePose(model, a, b, u) {
    // Recompute exact URDF FK at interpolated joint angles, rather than blending link endpoints.
    const positions = Object.fromEntries(Object.keys(a.positions).map(n=>[n,a.positions[n]+(b.positions[n]-a.positions[n])*u]));
    const base = Object.fromEntries(Object.keys(a.base_pose).map(n=>[n,a.base_pose[n]+(b.base_pose[n]-a.base_pose[n])*u]));
    // wheel_trial wraps yaw; use the shortest adjacent difference across +/-pi.
    base.yaw_rad = a.base_pose.yaw_rad + Math.atan2(Math.sin(b.base_pose.yaw_rad-a.base_pose.yaw_rad),Math.cos(b.base_pose.yaw_rad-a.base_pose.yaw_rad))*u;
    const world = {[model.root]:new THREE.Matrix4().makeRotationZ(base.yaw_rad).setPosition(base.x_m,base.y_m,0)};
    const joint_positions = {}, centers_of_mass = {};
    for(const j of model.joints) {
      const pivot = world[j.parent].clone().multiply(matrix(j.origin));
      joint_positions[j.name] = new THREE.Vector3().setFromMatrixPosition(pivot).toArray();
      world[j.child] = pivot.multiply(new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...j.axis).normalize(),positions[j.name]||0));
    }
    const link_transforms = Object.fromEntries(Object.entries(world).map(([n,t])=>[n,[0,1,2,3].map(i=>[0,1,2,3].map(k=>t.elements[k*4+i]))]));
    for(const l of model.links) centers_of_mass[l.name] = new THREE.Vector3(...l.com).applyMatrix4(world[l.name]).toArray();
    return {...a, positions, base_pose:base, joint_positions, centers_of_mass, link_transforms,
      time_s:a.time_s+(b.time_s-a.time_s)*u,
      wheel_velocity_rad_s:Object.fromEntries(Object.keys(a.wheel_velocity_rad_s||{}).map(n=>[n,a.wheel_velocity_rad_s[n]+((b.wheel_velocity_rad_s||{})[n]-a.wheel_velocity_rad_s[n])*u]))};
  }

  resetToolPaths() {
    for(const path of this.toolPaths.values()) {path.userData.points=[]; path.geometry.setFromPoints([]);}
  }
  updateToolPaths(pose) {
    for(const [side,path] of this.toolPaths) {
      const t=pose.link_transforms[side+'_tip']; path.visible=this.showTrails && !!t;
      if(!t || !this.showTrails) continue;
      const point=new THREE.Vector3(t[0][3],t[1][3],t[2][3]),points=path.userData.points;
      if(!points.length || points.at(-1).distanceTo(point)>.004) {
        points.push(point); if(points.length>160)points.shift(); path.geometry.setFromPoints(points);
      }
    }
  }

  pick(x, y) {
    this.raycaster.setFromCamera(new THREE.Vector2(x/this.canvas.clientWidth*2-1, 1-y/this.canvas.clientHeight*2), this.camera);
    return this.raycaster.intersectObject(this.robot, true).find(hit => hit.object.userData.type)?.object.userData || null;
  }
}

