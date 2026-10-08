#!/usr/bin/env python3
"""Desktop executable entry point; runs without ROS or an external Python install."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import threading
import webbrowser

import robot_workbench

VERSION = '0.2.0-preview.4'
ROOT = Path(__file__).resolve().parent if '__compiled__' in globals() else Path(__file__).resolve().parents[1]


def verify_package():
    manifest = json.loads((ROOT / 'MANIFEST.json').read_text(encoding='utf-8'))
    for name, expected in manifest['files'].items():
        path = ROOT / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Package file missing or changed: ' + name)


def main():
    parser = argparse.ArgumentParser(description='OmindOS desktop workbench ' + VERSION)
    parser.add_argument('--port', type=int, default=8085)
    parser.add_argument('--store', type=Path, default=Path.home() / 'OmindOS' / 'workbench-configurations')
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('--port must be between 0 and 65535')
    if '__compiled__' in globals() or args.verify:
        try:
            verify_package()
        except (OSError, ValueError, KeyError) as error:
            raise SystemExit(str(error)) from error
    if args.verify:
        print('Package verification passed.', flush=True)
        return
    robot_workbench.ROOT = ROOT
    try:
        server = robot_workbench.ThreadingHTTPServer(('127.0.0.1', args.port), robot_workbench.make_handler(args.store))
    except (OSError, ValueError) as error:
        raise SystemExit('Cannot start workbench. Check the port and store directory: ' + str(error)) from error
    url = f'http://127.0.0.1:{server.server_port}/'
    print('OmindOS workbench ' + VERSION + ': ' + url, flush=True)
    print('Configurations: ' + str(args.store.resolve()), flush=True)
    print('Keep this window open. Press Ctrl+C to stop.', flush=True)
    if not args.no_browser:
        timer = threading.Timer(0.5, lambda: webbrowser.open(url))
        timer.daemon = True
        timer.start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()

