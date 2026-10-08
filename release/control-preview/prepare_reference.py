"""Verify pinned MEVIUS2 models before copying into the compiled image."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

COMMIT = '4f09680bb575574377b903bbc971bdb2695d507b'
EXPECTED = '4f222335e987683f8fb46fc8e1966c1d7b6b85e1d7ebc98ecfe1c29e7617fd3e'
checkout = Path('/build/mevius2')
subprocess.run(['git', 'init', str(checkout)], check=True)
subprocess.run(['git', '-C', str(checkout), 'fetch', '--depth=1',
                'https://github.com/haraduka/mevius2.git', COMMIT], check=True)
subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', 'FETCH_HEAD'], check=True)
actual = subprocess.check_output(['git', '-C', str(checkout), 'rev-parse', 'HEAD'], text=True).strip()
assert actual == COMMIT
models = checkout/'models'
files = {str(f.relative_to(checkout)): hashlib.sha256(f.read_bytes()).hexdigest()
         for f in sorted(models.rglob('*')) if f.is_file()}
raw = json.dumps({'commit': COMMIT, 'files': files}, sort_keys=True, separators=(',', ':')).encode()
assert hashlib.sha256(raw).hexdigest() == EXPECTED, 'reference model manifest mismatch'
Path('/build/reference-manifest.json').write_bytes(raw)
shutil.copytree(models, '/build/reference-models')
