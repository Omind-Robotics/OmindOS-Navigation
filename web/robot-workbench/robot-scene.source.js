// Bundled locally with three@0.180.0. See THIRD_PARTY.md for reproduction.
import * as THREE from 'three';
import {RoundedBoxGeometry} from 'three/addons/geometries/RoundedBoxGeometry.js';
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
    this.renderer.toneMappingExposure = 1.12;
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
    const ambient = new THREE.HemisphereLight(0xe9f4ff, 0x475269, 2.2);
    ambient.position.set(0, 0, 3); this.scene.add(ambient);
    this.key = new THREE.DirectionalLight(0xfff5e9, 3.2);
    this.key.castShadow = true;
    this.key.shadow.mapSize.set(1024, 1024);
    this.key.shadow.bias = -.0002;
    this.key.shadow.normalBias = .008;
    this.key.shadow.radius = 5;
    this.key.shadow.blurSamples = 8;
    this.scene.add(this.key, this.key.target);
    const fill = new THREE.DirectionalLight(0xbddcff, 1.3);
    fill.position.set(3, 2, 4); this.scene.add(fill);
    this.floor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.ShadowMaterial({opacity:.24}));
    this.floor.receiveShadow = true; this.scene.add(this.floor);
    this.grid = new THREE.GridHelper(1, 12, 0xc4cfd8, 0xdde4e9);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.material.transparent = true; this.grid.material.opacity = .25;
    this.scene.add(this.grid);
    this.links = new Map(); this.joints = new Map();
    this.modelKey = ''; this.lastModel = null; this.resources = [];
    this.wireframe = false;
    canvas.dataset.renderer = 'webgl';
    canvas.addEventListener('webglcontextlost', e => {e.preventDefault(); canvas.dataset.renderer = 'lost';});
    canvas.addEventListener('webglcontextrestored', () => {canvas.dataset.renderer = 'webgl'; this.lastArgs && this.render(...this.lastArgs);});
  }

  own(resource) {this.resources.push(resource); return resource;}

  rebuild(model) {
    this.robot.clear(); this.resources.forEach(resource => resource.dispose()); this.resources = [];
    this.links.clear(); this.joints.clear();
    this.shell = this.own(new THREE.MeshStandardMaterial({color:0xdde5ea, roughness:.3, metalness:.35, envMapIntensity:1.05}));
    this.dark = this.own(new THREE.MeshStandardMaterial({color:0x263748, roughness:.32, metalness:.6, envMapIntensity:1.3}));
    this.selected = this.own(new THREE.MeshStandardMaterial({color:0x54c9e5, roughness:.26, metalness:.4, envMapIntensity:1.1}));
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
        if (g.type === 'box') geometry = new RoundedBoxGeometry(...g.size, 3, Math.min(...g.size)*.12);
        else if (g.type === 'sphere') geometry = new THREE.SphereGeometry(g.radius, 32, 20);
        else if (g.type === 'cylinder') {geometry = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32); geometry.rotateX(Math.PI/2);}
        else continue;
        const dark = link.name === model.root || /hand|tip|tool|wheel/i.test(link.name);
        const mesh = new THREE.Mesh(this.own(geometry), dark ? this.dark : this.shell);
        mesh.userData = {name:link.name, type:'link', dark};
        mesh.matrixAutoUpdate = false; mesh.matrix.copy(matrix(g.transform));
        mesh.castShadow = true; mesh.receiveShadow = true;
        group.add(mesh); meshes.push(mesh);
      }
      this.links.set(link.name, {group, meshes}); this.robot.add(group);
    }
    for (const joint of model.joints) {
      if (joint.type === 'fixed') continue;
      const group = new THREE.Group();
      const size = this.markerSize;
      // These are compact visual joint handles, not additional URDF bodies.
      const cap = new THREE.Mesh(cylinder, this.jointMaterial);
      cap.scale.set(size, size, size*2.2); cap.castShadow = true;
      cap.userData = {name:joint.name, type:'joint'};
      const rim = new THREE.Mesh(ring, this.dark);
      rim.scale.setScalar(size*1.04); rim.position.z = size*1.12;
      rim.userData = {name:joint.name, type:'joint'};
      group.add(cap, rim); this.robot.add(group); this.joints.set(joint.name, {group, cap, rim, joint});
    }
    // Retain a structural guide for links whose mesh resources are unavailable.
    const edges = model.joints.filter(joint => !this.links.get(joint.parent).meshes.length || !this.links.get(joint.child).meshes.length);
    const vertices = new Float32Array(edges.length*6);
    const geometry = this.own(new THREE.BufferGeometry()); geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    this.skeleton = new THREE.LineSegments(geometry, this.own(new THREE.LineBasicMaterial({color:0x6ba3b6})));
    this.skeleton.userData.edges = edges; this.robot.add(this.skeleton);
    this.stageNeedsFit = true;
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
      for (const mesh of link.meshes) mesh.material = mode === 'link' && selectedName === name ? this.selected : mesh.userData.dark ? this.dark : this.shell;
    }
    for (const [name, {group, cap, rim, joint}] of this.joints) {
      group.position.fromArray(pose.joint_positions[name]);
      const world = matrix(pose.link_transforms[joint.child]);
      const direction = new THREE.Vector3(...joint.axis).transformDirection(world).normalize();
      group.quaternion.setFromUnitVectors(zAxis, direction);
      const selected = mode === 'joint' && name === selectedName;
      cap.material = selected ? this.selected : this.jointMaterial;
      rim.material = selected ? this.ringMaterial : this.dark;
    }
    const positions = this.skeleton.geometry.attributes.position;
    this.skeleton.userData.edges.forEach((joint, i) => {
      positions.setXYZ(i*2, ...pose.link_transforms[joint.parent].slice(0,3).map(row => row[3]));
      positions.setXYZ(i*2+1, ...pose.joint_positions[joint.name]);
    }); positions.needsUpdate = true; this.skeleton.geometry.computeBoundingSphere();
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

  pick(x, y) {
    this.raycaster.setFromCamera(new THREE.Vector2(x/this.canvas.clientWidth*2-1, 1-y/this.canvas.clientHeight*2), this.camera);
    return this.raycaster.intersectObject(this.robot, true).find(hit => hit.object.userData.type)?.object.userData || null;
  }
}
