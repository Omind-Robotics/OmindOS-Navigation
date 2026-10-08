"""Inspect all Docker save layers for excluded application trees, including deleted files."""
import argparse
import json
from pathlib import PurePosixPath
import tarfile


parser = argparse.ArgumentParser()
parser.add_argument('archive')
parser.add_argument('output')
args = parser.parse_args()
manifest = None
with tarfile.open(args.archive, 'r|gz') as archive:
    for member in archive:
        if member.name == 'manifest.json':
            manifest = json.load(archive.extractfile(member))
            break
if not manifest or len(manifest) != 1:
    raise SystemExit('Expected a single-image Docker save archive')
layers = set(manifest[0]['Layers'])
checked = []
violations = []
file_count = 0
forbidden = ('opt/nav3d_ws/', 'workspace/Nav_3d/',
             'opt/omindos_preview/src/3D_NAV/', 'opt/omindos_preview/src/FAST_LIO_ROS2/',
             'opt/omindos_preview/src/livox_ros_driver2/',
             'opt/omindos_preview/src/planning_3d_ros2/',
             'opt/omindos_preview/src/nmpc_planner_ros2/',
             'opt/omindos_preview/src/nav3d_hardware_bringup/')
with tarfile.open(args.archive, 'r|gz') as archive:
    for member in archive:
        if member.name not in layers:
            continue
        checked.append(member.name)
        with tarfile.open(fileobj=archive.extractfile(member), mode='r|*') as layer:
            for item in layer:
                file_count += 1
                name = str(PurePosixPath(item.name.lstrip('./')))
                in_application = name.startswith('opt/omindos_preview/')
                if name.startswith(forbidden) or (in_application and '.git' in PurePosixPath(name).parts):
                    violations.append({'layer': member.name, 'path': name})
result = {'status': 'PASS' if set(checked) == layers and not violations else 'FAIL',
          'layer_count': len(checked), 'layer_entries_inspected': file_count,
          'excluded_application_path_hits': violations,
          'scope': 'All layer paths for excluded historical application trees and application .git; not a complete license or secrets audit'}
with open(args.output, 'w') as output:
    json.dump(result, output, indent=2)
    output.write('\n')
print(json.dumps(result))
if result['status'] != 'PASS':
    raise SystemExit(1)
