"""Assemble an internal candidate after the standalone image passes its own tests."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tarfile


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


parser = argparse.ArgumentParser()
parser.add_argument('directory')
parser.add_argument('--final', action='store_true')
args = parser.parse_args()
root = Path(args.directory).resolve()
package = root / 'package'
log = (root / 'standalone-validation.log').read_text()
match = re.search(r'(\d+) passed in ', log)
if not match or int(match.group(1)) != 20:
    raise SystemExit('Expected all 20 tests to pass')
results = {}
for name in ('wheeled_diff', 'wheeled_omni', 'quadruped'):
    import ast
    line = next((line for line in log.splitlines() if line.startswith(name + ' {')), None)
    if line is None:
        raise SystemExit('Missing platform result: ' + name)
    payload = ast.literal_eval(line.split(' ', 1)[1])
    if payload.get('status') != 'PASS' or payload.get('stopped_sec', 0) < 1:
        raise SystemExit('Platform did not pass: ' + name)
    results[name] = payload
image_name = 'omindos/navigation:0.2.0-preview.1'
image = json.loads(subprocess.check_output(['docker', 'image', 'inspect', image_name]))[0]
if image['Architecture'] != 'amd64' or image['Os'] != 'linux':
    raise SystemExit('Unexpected image architecture')
runtime = package / 'omindos-navigation-0.2.0-preview.1-linux-amd64.tar.gz'
if not runtime.is_file() or runtime.stat().st_size < 1024 * 1024:
    raise SystemExit('Missing Docker save archive')
if json.loads((package / 'LAYER_SCAN.json').read_text()).get('status') != 'PASS':
    raise SystemExit('All-layer excluded-component check did not pass')
provenance = json.loads((root / 'context/PROVENANCE.json').read_text())
if args.final and provenance.get('publication_status') != 'approved_preview':
    raise SystemExit('Final package requires recorded distribution confirmation')
validation = {'test_count': 20, 'test_result': 'PASS', 'platforms': results,
              'hardware_validated': False, 'scope': 'planar_kinematic_preview',
              'network': 'none', 'image_id': image['Id']}
(package / 'VALIDATION.json').write_text(json.dumps(validation, indent=2) + '\n')
(package / 'IMAGE_ID').write_text(image['Id'] + '\n')
manifest = {**provenance, 'image': image_name, 'image_id': image['Id'],
            'base_image': 'ros@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19',
            'platform': 'linux/amd64', 'hardware_validated': False,
            'excluded_components': ['3D_NAV', 'FAST_LIO_ROS2', 'livox_ros_driver2',
                                    'planning_3d_ros2', 'nmpc_planner_ros2',
                                    'nav3d_hardware_bringup', 'unitree_sdk2_python'],
            'runtime_sha256': digest(runtime), 'runtime_bytes': runtime.stat().st_size}
(package / 'MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
with (package / 'OS_PACKAGES.tsv').open('wb') as out:
    subprocess.run(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', '/bin/cat',
                    image_name, '/opt/omindos_preview/distribution-evidence/os-packages.tsv'],
                   check=True, stdout=out)
files = sorted(path for path in package.iterdir() if path.is_file() and path.name != 'SHA256SUMS')
(package / 'SHA256SUMS').write_text(''.join(digest(p) + '  ' + p.name + '\n' for p in files))
subprocess.run(['bash', './omindos', 'verify'], cwd=package, check=True)
archive = root / ('omindos-navigation-0.2.0-preview.1-linux-amd64.tar' if args.final else
                  'omindos-navigation-0.2.0-preview.1-candidate-linux-amd64.tar')
with tarfile.open(archive, 'w') as output:
    for path in sorted(package.iterdir()):
        if path.is_file():
            output.add(path, arcname='omindos-navigation-0.2.0-preview.1/' + path.name)
summary = {'archive': archive.name, 'bytes': archive.stat().st_size,
           'sha256': digest(archive), 'image_id': image['Id'],
           'publication_status': manifest['publication_status'], 'validation': validation}
(root / ('RELEASE_SUMMARY.json' if args.final else 'CANDIDATE_SUMMARY.json')).write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
