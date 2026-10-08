"""Private build utility; never include the build tree in public artifacts."""
from pathlib import Path
import os
import shutil
import subprocess
from setuptools import Extension, setup
from Cython.Build import cythonize

root=Path('/build/source')
out=Path('/build/native-app')
out.mkdir(exist_ok=True)
names=['robot_parameter_api','robot_workbench','robot_kinematics','workbench_state',
       'robot_connections','robot_runtime_client','robot_profile_runtime',
       'robot_profile_runtime_core','joint_interface','quadruped_controller',
       'quadruped_backends','quadruped_worker','quadruped_ros2','robstride04_driver',
       'rs04_simulated_bus','check_customer_profile','check_rs04_hardware',
       'validate_quadruped_dynamics','native_entry']
files=[root/'scripts'/f'{n}.py' if n!='native_entry' else root/'release/control-preview/native_entry.py' for n in names]
setup(name='omindos-control-native',ext_modules=cythonize(
    [Extension(n,[str(p)],extra_compile_args=['-O2','-g0']) for n,p in zip(names,files)],
    build_dir='/build/native-c',compiler_directives={'language_level':3,'annotation_typing':False,'binding':True,'embedsignature':False}),
    script_args=['build_ext','--build-lib',str(out/'scripts'),'--build-temp','/build/native-objects','-j','2'])
for folder in ['config','LICENSES']:
    shutil.copytree(root/folder,out/folder,dirs_exist_ok=True)
web=out/'web/robot-workbench';web.mkdir(parents=True,exist_ok=True)
for name in ['index.html','workbench.css','robot-scene.js','workbench.js','runtime.js','THIRD_PARTY.md']:
    shutil.copy2(root/'web/robot-workbench'/name,web/name)
reference=out/'reference';reference.mkdir(exist_ok=True)
shutil.copytree('/build/reference-models',reference/'models',dirs_exist_ok=True)
shutil.copy2('/build/reference-manifest.json',reference/'REFERENCE.json')
for f in (out/'scripts').glob('*.so'): subprocess.run(['strip','--strip-unneeded',str(f)],check=True)
bad=[str(f) for f in out.rglob('*') if f.is_file() and f.suffix in ('.py','.pyc','.pyo','.c','.h','.cpp','.map')]
if bad: raise RuntimeError('Source leaked into native application: '+str(bad))
print('NATIVE_BUILD_PASS',len(names),flush=True)
