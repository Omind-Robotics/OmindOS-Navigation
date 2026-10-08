#!/usr/bin/env python3
"""Export an independently named candidate; never writes published paths or tags."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''): h.update(block)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',required=True)
    parser.add_argument('--base-package',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--validation',type=Path,required=True)
    args=parser.parse_args()
    report=json.loads(args.validation.read_text())
    image=json.loads(subprocess.check_output(['docker','image','inspect',args.image]))[0]
    if report.get('status')!='PASS' or report.get('image_id')!=image['Id']:
        raise SystemExit('Require PASS validation for the exact image ID')
    if 'size-candidate' not in args.image:
        raise SystemExit('Candidate tag must contain size-candidate')
    version='0.2.0-preview.5-size-candidate.1'
    package=args.output/('omindos-control-preview-'+version+'-linux-amd64')
    package.mkdir(parents=True,exist_ok=False)
    old=json.loads((args.base_package/'MANIFEST.json').read_text())
    for name in ['config','LICENSES']:
        shutil.copytree(args.base_package/name,package/name)
    for name in ['THIRD_PARTY.md','THIRD_PARTY_PACKAGES.txt']:
        shutil.copy2(args.base_package/name,package/name)
    with (package/'THIRD_PARTY_PYTHON.json').open('w') as stream:
        subprocess.run(['docker','run','--rm','--network','none','--entrypoint',
            '/opt/omindos-runtime/bin/python',image['Id'],'-m','pip','list','--format=json'],check=True,stdout=stream)
    filename='runtime.image.tar.gz'
    launcher=(args.base_package/'omindos').read_text()
    launcher=launcher.replace('omindos/control-preview:0.2.0-preview.5',args.image)
    launcher=launcher.replace('omindos-control-preview-0.2.0-preview.5-linux-amd64.image.tar.gz',filename)
    (package/'omindos').write_text(launcher);(package/'omindos').chmod(0o755)
    with (package/filename).open('wb') as stream:
        save=subprocess.Popen(['docker','save',args.image],stdout=subprocess.PIPE)
        zipper=subprocess.run(['gzip','-1'],stdin=save.stdout,stdout=stream)
        save.stdout.close()
        if save.wait() or zipper.returncode: raise RuntimeError('Image export failed')
    (package/'SHA256SUMS-image.txt').write_text(sha(package/filename)+'  '+filename+'\n')
    (package/'IMAGE_ID.txt').write_text(image['Id']+'\n')
    (package/'VALIDATION.json').write_text(json.dumps(report,indent=2)+'\n')
    note='# 完整运行包体积优化候选（未发布）\n\n本包保留 preview.5 的编译业务程序、ROS 2 Humble、PyTorch CPU、NumPy、MuJoCo 与模型。\n只清理构建依赖和缓存，重新生成镜像层；业务版本仍显示 0.2.0-preview.5。\n完整解压后执行 `./omindos workbench`；`./omindos validate-dynamics` 运行仿真。\n需要 Linux x86_64 和 Docker Engine。当前不连接真实电机。\n\n'
    (package/'README.zh-CN.md').write_text(note+(args.base_package/'README.zh-CN.md').read_text())
    manifest={'package_version':version,'application_version':old['version'],
        'source_commit':old['source_commit'],'image_id':image['Id'],
        'baseline_image_id':report['baseline_image_id'],'publication_status':'private_candidate',
        'backend_source_included':False,'hardware_validated':False,
        'files':{str(p.relative_to(package)):sha(p) for p in sorted(package.rglob('*')) if p.is_file()}}
    (package/'MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
    archive=args.output/(package.name+'.tar')
    with tarfile.open(archive,'w') as handle: handle.add(package,arcname=package.name)
    result={'filename':archive.name,'bytes':archive.stat().st_size,'sha256':sha(archive),
        'image_bytes':(package/filename).stat().st_size,'image_id':image['Id'],'publication_status':'private_candidate'}
    (args.output/'SIZE_CANDIDATE.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.output/'SHA256SUMS').write_text(result['sha256']+'  '+archive.name+'\n')
    print(json.dumps(result))


if __name__=='__main__':
    main()
