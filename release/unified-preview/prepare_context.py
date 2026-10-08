"""Create a small clean candidate context from committed files only."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


parser = argparse.ArgumentParser()
parser.add_argument('destination')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
destination = Path(args.destination).resolve()
if destination.exists():
    raise SystemExit('Use a new, empty destination')
destination.mkdir(parents=True)
commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
paths = subprocess.check_output(
    ['git', 'ls-tree', '-r', '--name-only', commit, 'src/nav3d_native'], cwd=root, text=True).splitlines()
paths += ['scripts/validate_platform_preview.sh']
hashes = []
for name in paths:
    data = subprocess.check_output(['git', 'show', commit + ':' + name], cwd=root)
    target = destination / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    hashes.append(hashlib.sha256(data).hexdigest() + '  ' + name)
for name in ('Dockerfile', 'entrypoint.sh'):
    data = subprocess.check_output(
        ['git', 'show', commit + ':release/unified-preview/' + name], cwd=root)
    (destination / name).write_bytes(data)
(destination / 'SOURCE_FILES.sha256').write_text('\n'.join(hashes) + '\n')
(destination / 'PROVENANCE.json').write_text(json.dumps({
    'development_commit': commit,
    'version': '0.2.0-preview.1',
    'publication_status': 'approved_preview',
    'distribution_rights_confirmation': 'project_owner_confirmed_2026-10-08',
    'allowlisted_application_paths': paths,
}, indent=2) + '\n')
print(json.dumps({'context': str(destination), 'commit': commit, 'application_files': len(paths)}))
