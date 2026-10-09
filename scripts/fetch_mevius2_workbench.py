#!/usr/bin/env python3
"""Fetch the immutable MIT MEVIUS2 reference visuals; verify every Git blob."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = ROOT / 'config/robot_models/mevius2'


def verified(data, entry):
    digest = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
    return len(data) == entry['size'] and digest == entry['git_blob_sha1']


def fetch_assets():
    manifest = json.loads((ASSET_ROOT / 'UPSTREAM.json').read_text())
    # The source URL cannot be supplied by an imported URDF or API request.
    if manifest['repository'] != 'haraduka/mevius2' or manifest['commit'] != '4f09680bb575574377b903bbc971bdb2695d507b':
        raise ValueError('Unexpected upstream reference; review the pinned asset manifest')

    def obtain(item):
        name, entry = item
        if name != 'LICENSE' and name != 'models/mevius2_stl.urdf' and not (name.startswith('models/meshes/') and Path(name).name.endswith('.STL') and len(Path(name).parts) == 3):
            raise ValueError('Unexpected reference asset path')
        destination = ASSET_ROOT / name
        if destination.is_file() and verified(destination.read_bytes(), entry):
            return
        url = f"https://raw.githubusercontent.com/{manifest['repository']}/{manifest['commit']}/{name}"
        with urlopen(url, timeout=60) as response:
            data = response.read(entry['size'] + 1)
        if not verified(data, entry):
            raise ValueError('Pinned asset content mismatch: ' + name)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + '.tmp')
        temporary.write_bytes(data)
        temporary.replace(destination)

    with ThreadPoolExecutor(max_workers=6) as pool:
        list(pool.map(obtain, manifest['files'].items()))
    return ASSET_ROOT


if __name__ == '__main__':
    fetch_assets()
    print('Verified 13 MEVIUS2 STL meshes, URDF and MIT license at the pinned commit')
