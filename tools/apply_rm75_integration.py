#!/usr/bin/env python3
"""One-time, hash-locked application of reviewed RM75 integration edits.

Run only against the verified PR/main integration baseline. All replacements
are validated in memory before any file is written. Never changes a branch,
starts a driver, replaces configuration records, or publishes a release.
"""
import hashlib
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
BASE={
 'scripts/robot_kinematic_preview.py':'1798b580526418e66b4175967080437d280c3105',
 'scripts/robot_model_bundle.py':'3979a421849ea458df16ef2ae452d5e425d0ee99',
 'scripts/robot_assets.py':'08ee65980a99c555943644f75d76ba5fdd6210ba',
 'scripts/robot_kinematics.py':'5de98e1ca578e07591deaccf2ba1ebd171949266',
 'scripts/robot_planning.py':'b2b577498c8000da71818748a6c0873c50af06ab',
 'scripts/robot_workbench.py':'d635108d6345dbaa3c5303722b5dd460840ec08d',
 'scripts/workbench_desktop.py':'c33e1a3e1f07bc810833a1517ed2f55b795591a1',
 'web/robot-workbench/index.html':'b690e8515110f2e91221e5dc7a2b53bf9f6f3265',
 'web/robot-workbench/workbench.js':'73b8e811ec7b176f00d1569b0111edc567470c95',
 'release/workbench/package_binary.py':'9d26223441dfd95d75e5afb73619730c19df2457',
}
files={}
for name,expected in BASE.items():
 raw=(ROOT/name).read_bytes();actual=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
 if actual!=expected:raise SystemExit('Source changed; inspect before applying: '+name)
 files[name]=raw.decode('utf-8')

def replace(name,old,new,count=1):
 if files[name].count(old)!=count:raise SystemExit('Expected exact unique edit anchor: '+name+' / '+old[:70])
 files[name]=files[name].replace(old,new)

replace('scripts/robot_kinematic_preview.py','out=pose(model,q,base)','out=pose(model,q,base,calculate_gravity=False)')
replace('scripts/robot_model_bundle.py',"if profile.get('profile_mode')!='kinematic_preview':","if not isinstance(profile,dict) or profile.get('profile_mode')!='kinematic_preview':")
p='scripts/robot_assets.py'
replace(p,'from pathlib import Path\n','from pathlib import Path\n\n# Populated only after explicit local bundle integrity verification.\nEXTRA_MESH_URLS = {}\n')
replace(p,'def approved_mesh_url(filename):\n','def approved_mesh_url(filename):\n    if isinstance(filename, str) and filename in EXTRA_MESH_URLS:\n        return EXTRA_MESH_URLS[filename]\n')
p='scripts/robot_kinematics.py'
replace(p,'def pose(model, values, base_pose=None):','def pose(model, values, base_pose=None, *, calculate_gravity=True):')
replace(p,'    complete_mass = all(link["mass"] is not None for link in model["links"])','    complete_mass = calculate_gravity and all(link["mass"] is not None for link in model["links"])')
replace(p,'        if link["mass"] is None:\n','        if not calculate_gravity or link["mass"] is None:\n')
p='scripts/robot_planning.py'
replace(p,'from robot_parameter_api import number\n','from robot_parameter_api import number\nfrom robot_collision_frames import frame_only_links\n')
replace(p,"        root = ET.fromstring(profile['kinematics']['urdf_xml'])\n        self.shapes = []", "        root = ET.fromstring(profile['kinematics']['urdf_xml'])\n        self.frame_only_links = frame_only_links(root, self.tcp_links)\n        self.model['frame_only_links'] = sorted(self.frame_only_links)\n        self.shapes = []")
replace(p,"            name = link.get('name')\n            geometries =", "            name = link.get('name')\n            if name in self.frame_only_links:\n                continue\n            geometries =")
p='scripts/robot_workbench.py'
replace(p,'from workbench_state import WorkbenchState, MotionLocked\n','from workbench_state import (WorkbenchState, MotionLocked, _register_workbench_profile, _load_workbench_profile)\nfrom robot_kinematic_preview import motion as kinematic_motion\nfrom robot_model_bundle import register_bundle, inventory as model_inventory, model as bundled_model\n')
replace(p,'          "/workbench.js":', '          "/rm75-workbench.js": ("web/robot-workbench/rm75-workbench.js", "text/javascript; charset=utf-8"),\n          "/workbench.js":')
replace(p,"            if self.path == '/v1/workbench-state':", """            if self.path == '/v1/workbench-models':
                return self.respond(200, model_inventory())
            if self.path.startswith('/v1/workbench-model/'):
                try:
                    return self.respond(200, bundled_model(self.path[len('/v1/workbench-model/'):]))
                except ValueError as e:
                    return self.respond(404, {'error': str(e)})
            if self.path.startswith('/v1/workbench-profiles/'):
                try:
                    return self.respond(200, _load_workbench_profile(self.path[len('/v1/workbench-profiles/'):], store_dir))
                except FileNotFoundError:
                    return self.respond(404, {'error': 'profile not found'})
                except ValueError as e:
                    return self.respond(400, {'error': str(e)})
                except OSError:
                    return self.respond(500, {'error': 'profile storage unavailable'})
            if self.path == '/v1/workbench-state':""")
replace(p,'            if self.path not in ("/v1/robot-model",', '            if self.path not in ("/v1/workbench-profiles", "/v1/robot-model",')
replace(p,"                if runtime_route:\n", "                if self.path == '/v1/workbench-profiles':\n                    return self.respond(201, _register_workbench_profile(payload, store_dir))\n                if runtime_route:\n")
replace(p,"                    result = function(payload)\n", "                    result = (kinematic_motion(self.path, payload) if isinstance(payload, dict) and isinstance(payload.get('profile'), dict) and payload['profile'].get('profile_mode') == 'kinematic_preview' else function(payload))\n")
replace(p,'    args = parser.parse_args()\n', '    parser.add_argument("--model-bundle", type=Path, help="Explicit local RM75 model resource directory")\n    args = parser.parse_args()\n    bundle = args.model_bundle or ROOT / "config/robot_models/rm75-cad"\n    if args.model_bundle is not None or bundle.is_dir():\n        register_bundle(bundle)\n')
p='scripts/workbench_desktop.py'
replace(p,'import robot_planning\n','import robot_planning\nfrom robot_model_bundle import register_bundle\n')
replace(p,"    parser.add_argument('--no-browser'", "    parser.add_argument('--model-bundle', type=Path, help='Explicit local RM75 resource directory; bundled model is used when installed')\n    parser.add_argument('--no-browser'")
replace(p,"    try:\n        server = robot_workbench.ThreadingHTTPServer", "    try:\n        bundle = args.model_bundle or ROOT / 'config/robot_models/rm75-cad'\n        if args.model_bundle is not None or bundle.is_dir():\n            register_bundle(bundle)\n        server = robot_workbench.ThreadingHTTPServer")
p='web/robot-workbench/index.html'
replace(p,'<script defer src="/workbench.js"></script>', '<script defer src="/workbench.js"></script><script defer src="/rm75-workbench.js"></script>')
replace(p,'<label>机器人编号', '''<section id="rm75-model-panel" aria-label="整机模型与配置模式">
<label>配置模式<select id="profile-mode"><option value="full">完整参数配置</option><option value="kinematic_preview">运动学预览 · 不要求动力学参数</option></select></label>
<p id="profile-mode-note" class="note">运动学模式不计算电机目标、力矩或动力学；未知参数可留空。</p>
<label>整机 CAD<select id="cad-model" aria-label="RM75模型子型号"></select></label>
<button id="load-cad-model" disabled>加载整机 CAD 模型</button>
<p id="cad-bundle-note" class="note" role="status">检查本机整机资源…</p>
</section>
<label>机器人编号''')
p='web/robot-workbench/workbench.js'
replace(p,"const candidate = fromUrdf ? {robot_id:","const candidate = fromUrdf ? {profile_mode:'kinematic_preview', robot_id:")
old="  if (!candidate || !candidate.kinematics || typeof candidate.kinematics.urdf_xml !== 'string' || !candidate.dynamics || typeof candidate.dynamics.joints !== 'object' || candidate.dynamics.joints === null) throw Error('完整配置需要 robot_id、kinematics.urdf_xml 和 dynamics.joints。');\n  const doc = parse(candidate.kinematics.urdf_xml);"
new="  if (!candidate || !candidate.kinematics || typeof candidate.kinematics.urdf_xml !== 'string') throw Error('需要 robot_id 与 kinematics.urdf_xml。');\n  const doc = parse(candidate.kinematics.urdf_xml);\n  if (candidate.profile_mode === 'kinematic_preview' && candidate.dynamics === undefined) candidate.dynamics = {joints:missingDrives(doc)};\n  if (!candidate.dynamics || typeof candidate.dynamics.joints !== 'object' || candidate.dynamics.joints === null) throw Error('完整配置需要 dynamics.joints；仅运动学可显式选择 kinematic_preview。');"
replace(p,old,new)
replace(p,'  profile = candidate; xml = doc;','  profile = candidate; $(\'profile-mode\').value = candidate.profile_mode || \'full\'; xml = doc;')
replace(p,"api('/v1/robot-profiles',payload())", "api(profile.profile_mode==='kinematic_preview'?'/v1/workbench-profiles':'/v1/robot-profiles',payload())")
replace(p,"api('/v1/robot-profiles/'+id)","api('/v1/workbench-profiles/'+id)")
replace(p,":'运动路径采样点的峰值静态保持力矩';", ":result.gravity_evaluated===false?'仅运动学预览 · 未计算力矩或电机目标':'运动路径采样点的峰值静态保持力矩';")
replace(p,"status('正在计算运动轨迹和重力负载…');", "status(profile?.profile_mode==='kinematic_preview'?'正在计算关节运动学轨迹…':'正在计算运动轨迹和重力负载…');")
replace(p,"{profile:payload(),target:targets,base_target:baseTarget,duration_s:numeric($('duration').value)}", "{profile:payload(),...(profile.profile_mode==='kinematic_preview'?{start:currentPose.positions,base_start:currentPose.base_pose}:{}),target:targets,base_target:baseTarget,duration_s:numeric($('duration').value)}")
p='release/workbench/package_binary.py'
replace(p,"    parser.add_argument('--jobs'", "    parser.add_argument('--model-bundle', type=Path, help='Verified local RM75 model bundle to include in this single installer')\n    parser.add_argument('--jobs'")
replace(p,"    output = args.output.resolve()", "    bundle_info = None\n    if args.model_bundle is not None:\n        if args.variant != 'manipulation':\n            parser.error('RM75 model bundle is manipulation-only')\n        sys.path.insert(0, str(ROOT / 'scripts'))\n        from robot_model_bundle import register_bundle\n        bundle_info = register_bundle(args.model_bundle)\n    output = args.output.resolve()")
replace(p,"               '--include-module=robot_assets',", "               '--include-module=robot_model_bundle', '--include-module=robot_collision_frames',\n               '--include-module=robot_kinematic_preview',\n               '--include-module=robot_assets',")
replace(p,"    paths = ['web/robot-workbench/index.html',", "    paths = ['web/robot-workbench/rm75-workbench.js', 'web/robot-workbench/index.html',")
replace(p,"    if args.variant == 'quadruped':\n        from quadruped_variant", """    if args.model_bundle is not None:
        source = args.model_bundle.resolve()
        bundle_manifest = json.loads((source/'manifest.json').read_text())
        names = ['manifest.json', *bundle_manifest['assets'],
                 *[r['profile'] for r in bundle_manifest['variants'].values()]]
        if (source/'source-notices').is_dir():
            names += [str(p.relative_to(source)) for p in (source/'source-notices').rglob('*') if p.is_file()]
        for name in names:
            path = (source/name).resolve(strict=True)
            if not path.is_relative_to(source) or path.suffix.lower() in ('.py','.pyc','.cpp','.h','.map'):
                raise ValueError('unsafe bundle delivery file')
            destination = package/'config/robot_models/rm75-cad'/name
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(path,destination)
    if args.variant == 'quadruped':
        from quadruped_variant""")
replace(p,"    manifest = {'version': args.version,", "    manifest = {'customer_cad_bundle': bundle_info, 'version': args.version,")
for name,text in files.items():
 (ROOT/name).write_text(text,encoding='utf-8')
 print(name, hashlib.sha256(text.encode()).hexdigest())
