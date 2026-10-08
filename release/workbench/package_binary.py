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
VERSION = '0.2.0-preview.6'


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
               '--include-module=robot_connections', '--include-module=robot_runtime_client']
    if windows:
        command += ['--msvc=latest', '--windows-console-mode=force',
                    '--company-name=OriginMind', '--product-name=OmindOS Workbench',
                    '--file-version=0.2.0.6', '--product-version=0.2.0.6']
    command.append(str(ROOT / 'scripts/workbench_desktop.py'))
    subprocess.run(command, cwd=ROOT, env=env, check=True)
    package = output / (product + '-' + args.version + '-' + target)
    shutil.copytree(build / 'workbench_desktop.dist', package)
    paths = ['web/robot-workbench/index.html', 'web/robot-workbench/workbench.css',
             'web/robot-workbench/robot-scene.js', 'web/robot-workbench/runtime.js', 'config/robot_profiles/workbench-demo.json',
             'LICENSES/Three-MIT.txt', 'LICENSES/OpenSSL-3-Apache-2.0.txt',
             'LICENSES/Workbench-Runtime-NOTICE.txt', 'LICENSES/bzip2-LICENSE.txt',
             'LICENSES/XZ-COPYING.txt', 'config/robot_models/realman_rm75/LICENSE',
             'config/robot_models/realman_rm75/UPSTREAM_LICENSE', 'config/robot_models/realman_rm75/NOTICE.md']
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
    python_version = platform.python_version()
    with urllib.request.urlopen('https://raw.githubusercontent.com/python/cpython/v' + python_version + '/LICENSE', timeout=60) as response:
        (package / 'LICENSES/Python.txt').write_bytes(response.read())
    distribution = importlib.metadata.distribution('Nuitka')
    for item in distribution.files or []:
        if item.name.lower() in ('license.txt', 'license', 'copying'):
            source = Path(distribution.locate_file(item))
            if source.is_file():
                shutil.copyfile(source, package / 'LICENSES' / ('Nuitka-' + item.name))
    manifest = {'version': args.version, 'variant': args.variant, 'platform': target, 'python_bundled': python_version,
                'compiler': 'Nuitka ' + importlib.metadata.version('Nuitka'),
                'source_commit': os.environ.get('GITHUB_SHA', subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()),
                'release_stage': 'desktop-preview', 'backend_source_included': False,
                'ros_required': False, 'ros_runtime_bundled': False, 'ros_bridge_included': True,
                'runtime_applied': False, 'hardware_validated': False,
                'dynamics_simulation_bundled': False, 'torch_bundled': False, 'mujoco_bundled': False,
                'files': {str(p.relative_to(package)).replace(os.sep, '/'): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(package.rglob('*')) if p.is_file()}}
    # Refuse business source, bytecode, development reports and source maps.
    for name in manifest['files']:
        if Path(name).suffix.lower() in ('.py', '.pyc', '.pyo', '.pyz', '.c', '.cpp', '.h', '.map') or name.endswith('.source.js'):
            raise SystemExit('Forbidden source artifact: ' + name)
    (package / 'MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    subprocess.run([sys.executable, str(ROOT / 'release/workbench/smoke_binary.py'), str(package / executable), '--variant', args.variant], check=True)
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
    print(json.dumps({'archive': str(archive), 'sha256': digest, 'bytes': archive.stat().st_size}), flush=True)


if __name__ == '__main__':
    main()



