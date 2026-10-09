#!/usr/bin/env python3
"""Private build tool. Publish only the validated archive produced by this script."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
VERSION = '0.3.0'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', choices=('manipulation', 'quadruped'), default='manipulation')
    parser.add_argument('--version', default=VERSION)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    output = args.output.resolve()
    if args.jobs < 1 or not __import__('re').fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]*', args.version):
        parser.error('invalid jobs or version')
    output.mkdir(parents=True, exist_ok=True)
    product = 'omindos-quadruped-workbench' if args.variant == 'quadruped' else 'omindos-workbench'
    windows = sys.platform == 'win32'
    target = 'windows-x64' if windows else 'linux-x64'
    if platform.machine().lower() not in ('amd64', 'x86_64'):
        raise SystemExit('This release supports x64 only.')
    executable = 'OmindOS-Workbench.exe' if windows else 'OmindOS-Workbench'
    build = output / 'private-build'
    env = {**os.environ, 'PYTHONPATH': str(ROOT / 'scripts')}
    command = [sys.executable, '-m', 'nuitka', '--mode=standalone', '--assume-yes-for-downloads',
               '--python-flag=isolated', '--python-flag=no_docstrings', '--jobs=' + str(args.jobs),
               '--output-dir=' + str(build), '--output-filename=' + executable,
               '--report=' + str(output / 'private-compilation-report.xml'),
               '--include-module=robot_workbench', '--include-module=robot_parameter_api',
               '--include-module=robot_kinematics', '--include-module=workbench_state',
               '--include-module=robot_assets', '--include-module=robot_sequence',
               '--include-module=robot_connections', '--include-module=robot_runtime_client',
               '--include-module=robot_planning', '--include-module=dual_task_planner',
               '--include-module=robot_navigation', '--include-module=navigation_kernel', '--include-module=navigation_guard']
    if windows:
        command += ['--msvc=latest', '--windows-console-mode=force',
                    '--company-name=OriginMind', '--product-name=OmindOS Workbench',
                    '--file-version=0.2.0.8', '--product-version=0.2.0.8']
    command.append(str(ROOT / 'scripts/workbench_desktop.py'))
    subprocess.run(command, cwd=ROOT, env=env, check=True)
    package = output / (product + '-' + args.version + '-' + target)
    shutil.copytree(build / 'workbench_desktop.dist', package)
    paths = ['web/robot-workbench/index.html', 'web/robot-workbench/workbench.css',
             'web/robot-workbench/robot-scene.js', 'web/robot-workbench/runtime.js', 'web/robot-workbench/cooperative-demo.json',
             'web/robot-workbench/mda1-reference.webp', 'config/robot_profiles/workbench-demo.json',
             'LICENSES/Three-MIT.txt', 'LICENSES/OpenSSL-3-Apache-2.0.txt',
             'LICENSES/Workbench-Runtime-NOTICE.txt', 'LICENSES/bzip2-LICENSE.txt',
             'LICENSES/XZ-COPYING.txt', 'config/robot_models/realman_rm75/LICENSE',
             'LICENSES/nav3d-native-Apache-2.0.txt', 'LICENSES/nav3d-native-NOTICE.txt',
             'config/robot_models/realman_rm75/UPSTREAM_LICENSE', 'config/robot_models/realman_rm75/NOTICE.md']
    paths += ['config/robot_models/realman_rm75/meshes/' + p.name
              for p in sorted((ROOT/'config/robot_models/realman_rm75/meshes').iterdir()) if p.is_file()]
    for name in paths:
        destination = package / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, destination)
    if args.variant == 'quadruped':
        from quadruped_variant import prepare
        prepare(package, ROOT)
    subprocess.run(['node', str(ROOT / 'release/workbench/minify.cjs'),
                    str(ROOT / 'web/robot-workbench/workbench.js'),
                    str(package / 'web/robot-workbench/workbench.js')], cwd=ROOT, check=True)
    shutil.copyfile(ROOT / 'release/workbench' / ('QUADRUPED.zh-CN.md' if args.variant == 'quadruped' else 'README.zh-CN.md'), package / 'README.zh-CN.md')
    shutil.copyfile(ROOT / 'release/workbench/ROS2.zh-CN.md', package / 'ROS2.zh-CN.md')
    if args.variant == 'manipulation':
        shutil.copyfile(ROOT/'docs/UNIFIED_MANIPULATION_PLANNING.zh-CN.md',package/'PLANNING.zh-CN.md')
        shutil.copyfile(ROOT/'docs/WORKBENCH_NAVIGATION.zh-CN.md',package/'NAVIGATION.zh-CN.md')
    python_version = platform.python_version()
    with urllib.request.urlopen('https://raw.githubusercontent.com/python/cpython/v' + python_version + '/LICENSE', timeout=60) as response:
        (package / 'LICENSES/Python.txt').write_bytes(response.read())
    distribution = importlib.metadata.distribution('Nuitka')
    for item in distribution.files or []:
        if item.name.lower() in ('license.txt', 'license', 'copying'):
            source = Path(distribution.locate_file(item))
            if source.is_file():
                shutil.copyfile(source, package / 'LICENSES' / ('Nuitka-' + item.name))
    for dependency in ('numpy','scipy'):
        distribution = importlib.metadata.distribution(dependency)
        for index,item in enumerate(distribution.files or []):
            if item.name.lower() in ('license.txt','license','copying'):
                source = Path(distribution.locate_file(item))
                if source.is_file():
                    shutil.copyfile(source, package/'LICENSES'/f'{dependency}-{index}-{item.name}')
    manifest = {'version': args.version, 'variant': args.variant, 'platform': target, 'python_bundled': python_version,
                'compiler': 'Nuitka ' + importlib.metadata.version('Nuitka'),
                'source_commit': os.environ.get('GITHUB_SHA', subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()),
                'release_stage': 'geometric-workbench', 'backend_source_included': False,
                'planning_kernel_source': 'Omind-Robotics/OmindOS-Manipulation@cbe8caf13921002e463355c6a2982e900640c7e0',
                'planning_kernel_sha256': hashlib.sha256((ROOT/'scripts/dual_task_planner.py').read_bytes()).hexdigest(),
                'coupled_waist_dual_arm_planning': True,
                'navigation_integrated':True, 'navigation_drive_types':['differential','omnidirectional'],
                'navigation_kernel_source':'Omind-Robotics/OmindOS-Navigation@3293dfc83e32b8e56f1f185b80e19e56a2fe600d:src/nav3d_native/nav3d_native/algorithms.py',
                'navigation_kernel_sha256':hashlib.sha256((ROOT/'scripts/navigation_kernel.py').read_bytes()).hexdigest(),
                'navigation_guard_sha256':hashlib.sha256((ROOT/'scripts/navigation_guard.py').read_bytes()).hexdigest(),
                'navigation_localization_source':'geometric_simulator', 'sensor_localization_bundled':False,
                'parking_interlock':{'position_tolerance_m':.035,'yaw_tolerance_deg':2,'observed_zero_motion_s':.6},
                'planning_collision_geometry': 'conservative_oriented_boxes',
                'continuous_collision_validated': False, 'trajectory_timing_validated': False,
                'ros_required': False, 'ros_runtime_bundled': False, 'ros_bridge_included': True,
                'runtime_applied': False, 'hardware_validated': False,
                'dynamics_simulation_bundled': False, 'torch_bundled': False, 'mujoco_bundled': False,
                'manufacturer_meshes_bundled': 8, 'dual_arm_mesh_instances': 16,
                'cooperative_demo_duration_s': 35, 'contact_dynamics_validated': False,
                'files': {str(p.relative_to(package)).replace(os.sep, '/'): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(package.rglob('*')) if p.is_file()}}
    # Refuse business source, bytecode, development reports and source maps.
    for name in manifest['files']:
        if Path(name).suffix.lower() in ('.py', '.pyc', '.pyo', '.pyz', '.c', '.cpp', '.h', '.map') or name.endswith('.source.js'):
            raise SystemExit('Forbidden source artifact: ' + name)
    (package / 'MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    subprocess.run([sys.executable, str(ROOT / 'release/workbench/smoke_binary.py'), str(package / executable), '--variant', args.variant], check=True)
    if args.variant == 'manipulation':
        subprocess.run([sys.executable,str(ROOT/'release/workbench/smoke_planning.py'),str(package/executable),
                        '--report',str(output/'planning-binary-validation.json')],check=True)
        subprocess.run([sys.executable,str(ROOT/'release/workbench/smoke_navigation.py'),str(package/executable),
                        '--report',str(output/'navigation-binary-validation.json')],check=True)
    archive = output / (package.name + ('.zip' if windows else '.tar.gz'))
    if windows:
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as handle:
            for path in sorted(package.rglob('*')):
                if path.is_file():
                    handle.write(path, path.relative_to(output))
    else:
        with tarfile.open(archive, 'w:gz', compresslevel=9) as handle:
            handle.add(package, arcname=package.name)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    (output / (archive.name + '.sha256')).write_text(digest+'  '+archive.name+'\n', encoding='utf-8')
    # The downloadable archive, independently extracted, must also run.
    extracted = output / 'fresh-unpack'
    extracted.mkdir()
    if windows:
        with zipfile.ZipFile(archive) as handle:
            handle.extractall(extracted)
    else:
        with tarfile.open(archive) as handle:
            handle.extractall(extracted, filter='data')
    subprocess.run([sys.executable, str(ROOT / 'release/workbench/smoke_binary.py'),
                    str(extracted / package.name / executable), '--variant', args.variant], check=True)
    if args.variant == 'manipulation':
        subprocess.run([sys.executable,str(ROOT/'release/workbench/smoke_planning.py'),
                        str(extracted/package.name/executable),'--report',str(output/'planning-fresh-unpack-validation.json')],check=True)
        subprocess.run([sys.executable,str(ROOT/'release/workbench/smoke_navigation.py'),
                        str(extracted/package.name/executable),'--report',str(output/'navigation-fresh-unpack-validation.json')],check=True)
    print(json.dumps({'archive': str(archive), 'sha256': digest, 'bytes': archive.stat().st_size}), flush=True)


if __name__ == '__main__':
    main()




