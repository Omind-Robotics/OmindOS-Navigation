"""Compiled entry point. All modes start without hardware access."""
import argparse
import importlib
import os
import platform
from pathlib import Path
import signal
import subprocess
import sys
import time

VERSION = '0.2.0-preview.5'
MODES = {'workbench':'robot_workbench', 'parameter-api':'robot_parameter_api',
         'profile-runtime':'robot_profile_runtime', 'quadruped':'quadruped_worker',
         'quadruped-ros':'quadruped_ros2', 'validate-dynamics':'validate_quadruped_dynamics',
         'check-profile':'check_customer_profile'}


def main():
    if sys.argv[1:] == ['--version']:
        print(VERSION)
        return 0
    if len(sys.argv)<2 or sys.argv[1] in ('-h','--help'):
        print('OmindOS control preview '+VERSION+' (Linux '+platform.machine()+', simulation only)')
        print('Modes: '+', '.join(MODES)+', ros-preview')
        return 0
    mode=sys.argv.pop(1)
    if mode=='ros-preview':
        parser=argparse.ArgumentParser(description='Optional ROS 2 workbench preview; no motor transport')
        parser.add_argument('--store',default='/data/profiles')
        parser.add_argument('--port',type=int,default=8085)
        parser.add_argument('--runtime-port',type=int,default=18098)
        args=parser.parse_args()
        children=[]
        def stop(*unused):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,stop)
        try:
            exe='/usr/local/bin/omindos-runtime'
            children.append(subprocess.Popen([exe,'profile-runtime','--store',args.store,'--port',str(args.runtime_port)]))
            children.append(subprocess.Popen([exe,'workbench','--store',args.store,'--port',str(args.port),
                '--runtime-url',f'http://127.0.0.1:{args.runtime_port}']))
            while all(p.poll() is None for p in children): time.sleep(.2)
            return 1
        except KeyboardInterrupt:
            return 0
        finally:
            for p in children:
                if p.poll() is None: p.terminate()
            for p in children:
                try: p.wait(5)
                except subprocess.TimeoutExpired: p.kill();p.wait()
    if mode not in MODES:
        raise SystemExit('Unknown mode: '+mode)
    if mode=='quadruped':
        if '--allow-hardware' in sys.argv or any(x.startswith('--backend=') and x!='--backend=mujoco' for x in sys.argv):
            raise SystemExit('This public preview is simulation only')
        if '--backend' in sys.argv and sys.argv[sys.argv.index('--backend')+1]!='mujoco':
            raise SystemExit('This public preview is simulation only')
        sys.argv.extend(['--backend','mujoco'])
        if '--source' not in sys.argv and not any(x.startswith('--source=') for x in sys.argv):
            sys.argv.extend(['--source','/opt/omindos/reference'])
    return importlib.import_module(MODES[mode]).main()


if __name__=='__main__':
    raise SystemExit(main())

