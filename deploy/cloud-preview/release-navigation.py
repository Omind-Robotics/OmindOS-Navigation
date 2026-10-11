#!/usr/bin/env python3
"""Promote only verified navigation files; preserve the newer public Cloud adapter.

Run --prepare, review manifest, then --apply. --rollback refuses changed files.
Authentication and customer data are never copied or changed by this script.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import urllib.error
import urllib.request

SOURCE = Path('/home/ubuntu/omindos-build/rm75-final-20261010-1826/source')
APP = Path('/opt/omindos-cloud-preview/app')
CONTAINER = 'omindos-cloud-preview'
FILES = {
    'scripts/robot_navigation.py': ('0d42a61650f754e97b4169a3526b59333b72d5d44f198580be2d0c0fcee04c2a', '9aab61183d503c7293995718f5a32120fc9d1449f84db349ed71cf0a1b7e2b47'),
    'scripts/robot_workbench.py': ('7ef8dc22387ce127f1c287c31a6624f74771a8a5b158b4b4cbd7599ee4635f80', 'f4d6d388a097a8cd5c28d80a338898682dbec88c882a5f84d16db6a1dcfcf48e'),
    'web/robot-workbench/workbench.js': ('fe61b41e58315009edd7095767a7971e1bf2d33de053b8d0ba069d9766de4df2', '404aca0c71235ab9feee38f7a9664098db1a3adb502504bd7439d74423fb68d7'),
    'app/navigation-ready.js': ('56364baa048b9dddaf1c268d547eb83ad711cf7d2aefc7288618c12833e93eb4', '6bfd052704ba4916a68d7e357fcbe1f61e3c1c547e01bef0fccd47c66fd3811b'),
}
ADAPTER_SHA = '5c89e1890877c59982223fbd98954504486f5630a218d644d40ea681bb428671'
ANCHOR = "or route.startswith('/v1/navigation-task/') or route.startswith('/v1/planning-task/')):"
HEARTBEAT = "or route.startswith('/v1/navigation-task/') or route.startswith('/v1/navigation-heartbeat/') or route.startswith('/v1/planning-task/')):"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def target(name):
    return APP / name[4:] if name.startswith('app/') else SOURCE / name


def atomic_copy(source, destination):
    old = destination.stat()
    temporary = destination.with_name(destination.name + '.navigation-release')
    shutil.copyfile(source, temporary)
    os.chmod(temporary, old.st_mode)
    os.chown(temporary, old.st_uid, old.st_gid)
    os.replace(temporary, destination)


def health(expected_root, heartbeat_status):
    deadline = time.monotonic() + 20
    while True:
        try:
            with urllib.request.urlopen('https://cloud.omindos.cn/', timeout=3) as response:
                assert response.status == 200
                assert hashlib.sha256(response.read()).hexdigest() == expected_root
            try:
                urllib.request.urlopen('https://cloud.omindos.cn/v1/navigation-heartbeat/release-health', timeout=3)
            except urllib.error.HTTPError as error:
                assert error.code == heartbeat_status, error.code
                return
            raise AssertionError('unknown task unexpectedly exists')
        except (OSError, AssertionError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    for name in ('prepare', 'apply', 'rollback'):
        mode.add_argument('--' + name, action='store_true')
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--backup', type=Path, required=True)
    parser.add_argument('--commit', required=True)
    args = parser.parse_args()
    assert re.fullmatch('[0-9a-f]{40}', args.commit), 'commit SHA required'
    manifest_path = args.backup / 'manifest.json'
    if args.prepare:
        assert not args.backup.exists(), 'never overwrite a release backup'
        expected = dict(FILES, **{'app/cloud_preview.py': (ADAPTER_SHA, None)})
        for name, (before, after) in expected.items():
            assert digest(target(name)) == before, 'production drift: ' + name
            if after:
                candidate = args.candidate / ('app' if name.startswith('app/') else 'source') / (name[4:] if name.startswith('app/') else name)
                assert digest(candidate) == after, 'candidate mismatch: ' + name
        source = (APP / 'cloud_preview.py').read_text()
        assert source.count(ANCHOR) == 1, 'guest permission anchor changed'
        modified = source.replace(ANCHOR, HEARTBEAT, 1)
        compile(modified, str(APP / 'cloud_preview.py'), 'exec')
        root = urllib.request.urlopen('https://cloud.omindos.cn/', timeout=10).read()
        manifest = {'commit': args.commit, 'root_sha256': hashlib.sha256(root).hexdigest(),
                    'app_before': {p.name: digest(p) for p in APP.iterdir() if p.is_file()},
                    'files': {}, 'status': 'prepared'}
        for name, (before, after) in expected.items():
            old, new = args.backup / 'before' / name, args.backup / 'after' / name
            old.parent.mkdir(parents=True, exist_ok=True)
            new.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target(name), old)
            if name == 'app/cloud_preview.py':
                new.write_text(modified)
            else:
                shutil.copyfile(candidate := args.candidate / ('app' if name.startswith('app/') else 'source') / (name[4:] if name.startswith('app/') else name), new)
            manifest['files'][name] = {'before': digest(old), 'after': digest(new)}
        script = (args.backup / 'after/web/robot-workbench/workbench.js').read_text()
        script = script.replace('async function runNavigation(thenArm=false){\n', (args.backup / 'after/app/navigation-ready.js').read_text(), 1)
        checked = args.backup / 'prepared-workbench.js'
        checked.write_text(script)
        subprocess.run(['node', '--check', str(checked)], check=True)
        for name in ('scripts/robot_navigation.py', 'scripts/robot_workbench.py'):
            compile((args.backup / 'after' / name).read_text(), name, 'exec')
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    else:
        manifest = json.loads(manifest_path.read_text())
        assert manifest['commit'] == args.commit, 'release commit changed'
        before, after = ('after', 'before') if args.rollback else ('before', 'after')
        for name, hashes in manifest['files'].items():
            assert digest(target(name)) == hashes[before], 'production drift: ' + name
            assert digest(args.backup / after / name) == hashes[after], 'backup mismatch: ' + name
        for name, sha in manifest['app_before'].items():
            if 'app/' + name not in manifest['files']:
                assert digest(APP / name) == sha, 'unrelated app drift: ' + name
        subprocess.run(['docker', 'stop', '-t', '5', CONTAINER], check=True)
        try:
            for name in manifest['files']:
                atomic_copy(args.backup / after / name, target(name))
            subprocess.run(['docker', 'start', CONTAINER], check=True)
            health(manifest['root_sha256'], 401 if args.rollback else 404)
        except Exception:
            subprocess.run(['docker', 'stop', '-t', '5', CONTAINER], check=True)
            for name in manifest['files']:
                atomic_copy(args.backup / before / name, target(name))
            subprocess.run(['docker', 'start', CONTAINER], check=True)
            manifest['status'] = 'reverted_after_error'
            manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
            raise
        manifest['status'] = 'rolled_back' if args.rollback else 'released'
        manifest['updated_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
