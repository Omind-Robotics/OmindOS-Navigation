#!/usr/bin/env python3
"""Measure a local image without network or hardware access. Never estimates bytes."""
import argparse
import json
from pathlib import Path
import subprocess

PROBE = r'''
import json,os,subprocess,sys
from pathlib import Path
def size(path):
    total=allocated=count=0
    for root,dirs,files in os.walk(path,followlinks=False):
        for name in files:
            p=Path(root)/name
            if p.is_symlink(): continue
            s=p.stat();total+=s.st_size;allocated+=s.st_blocks*512;count+=1
    return {'logical_file_bytes':total,'allocated_file_bytes':allocated,'files':count}
site='/opt/omindos-runtime/lib/python3.10/site-packages'
paths=['/opt/ros','/opt/omindos','/opt/omindos/scripts','/opt/omindos/reference',
       '/opt/omindos-runtime','/usr/lib/python3.10','/usr/lib/python3/dist-packages',
       '/usr/lib/gcc','/root/.cache/pip','/var/lib/apt/lists']
parts={p:size(p) for p in paths}
packages={p.name:size(p) for p in Path(site).iterdir() if p.is_dir()}
dpkg=subprocess.check_output(['dpkg-query','-W','-f=${Package}\t${Version}\t${Installed-Size}\n'],text=True)
ros=[l.split('\t') for l in dpkg.splitlines() if l.startswith('ros-humble-')]
import torch,numpy,mujoco
print(json.dumps({'python':sys.version.split()[0],
 'versions':{'torch':torch.__version__,'numpy':numpy.__version__,'mujoco':mujoco.__version__},
 'paths':parts,'python_packages':packages,'ros_deb_installed_kib':sum(int(r[2]) for r in ros),
 'ros_deb_count':len(ros),'dpkg_inventory':dpkg,
 'measurement_note':'Directory totals overlap; logical files and allocated blocks differ. ROS directory excludes system dependencies.'}))
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    image=json.loads(subprocess.check_output(['docker','image','inspect',args.image]))[0]
    result=json.loads(subprocess.check_output(['docker','run','--rm','--network','none',
        '--cap-drop','ALL','--security-opt','no-new-privileges','--entrypoint',
        '/opt/omindos-runtime/bin/python',image['Id'],'-c',PROBE],text=True))
    result.update(image_id=image['Id'],docker_inspect_size_bytes=image['Size'],
                  architecture=image['Architecture'],image=args.image)
    result['history']=subprocess.check_output(['docker','history','--no-trunc','--format',
        '{{json .}}',image['Id']],text=True).splitlines()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('image_id','docker_inspect_size_bytes','versions','paths')}))


if __name__=='__main__':
    main()
