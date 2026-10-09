#!/usr/bin/env python3
"""Private builder: export only compiled workbench updates and public model assets."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/ubuntu-reference-update'
VERSION = '0.2.0-preview.7'
BASE_NAME = 'omindos-control-preview-0.2.0-preview.5-size-candidate.1-linux-amd64.tar'
BASE_SHA = 'af2cc32308c0ffc0a14b26cd71f454b8b9566096573ee09fb09f191ec7eedbd8'
BASE_URL = 'https://github.com/maganrobotics-boop/OmindOS-Navigation-Release/releases/download/v0.2.0-preview.6/' + BASE_NAME
MODULES = ['robot_workbench','robot_assets','robot_kinematics','robot_parameter_api','workbench_state',
           'robot_connections','robot_runtime_client','robot_sequence','robot_planning','robot_navigation',
           'dual_task_planner','navigation_kernel','navigation_guard','robot_profile_runtime_core',
           'robot_profile_runtime','native_entry']


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def run(*args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    archive = OUT / BASE_NAME
    if not archive.exists() or sha(archive) != BASE_SHA:
        run('curl','--fail','--location','--retry','3','--output',str(archive),BASE_URL)
    assert archive.stat().st_size == 540416000 and sha(archive) == BASE_SHA
    base = OUT / 'base'
    base.mkdir(exist_ok=True)
    with tarfile.open(archive) as t:
        t.extractall(base, filter='data')
    package = next(base.glob('omindos-control-preview-*'))
    image = re.search(r"^image='([^']+)'", (package / 'omindos').read_text(), re.M).group(1)
    image_archive = package / 'runtime.image.tar.gz'
    run('sha256sum','-c','SHA256SUMS-image.txt',cwd=package)
    run('docker','load','-i',str(image_archive))
    image_id = subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
    assert image_id in (package / 'IMAGE_ID.txt').read_text().splitlines()
    sys.path.insert(0,str(ROOT / 'release/workbench'))
    from quadruped_variant import prepare
    payload = OUT / 'payload'
    (payload / 'web').mkdir(parents=True,exist_ok=True)
    web = payload / 'web/robot-workbench'
    web.mkdir(exist_ok=True)
    for name in ['index.html','workbench.css','workbench.js','robot-scene.js','runtime.js',
                 'cooperative-demo.json','mda1-reference.webp','THIRD_PARTY.md']:
        shutil.copy2(ROOT / 'web/robot-workbench' / name, web / name)
    prepare(payload, ROOT)
    context = OUT / 'context'
    source = context / 'source'
    context.mkdir(exist_ok=True)
    for folder in ['scripts','tests','config','web','release']:
        shutil.copytree(ROOT / folder, source / folder,dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('node_modules','__pycache__','*.pyc','build'))
    # native_entry is compiled with the public release's version, preserving simulation-only modes.
    entry = source / 'release/control-preview/native_entry.py'
    entry.write_text(entry.read_text().replace("VERSION = '0.2.0-preview.5'", "VERSION = '" + VERSION + "'"))
    compiler = '''from pathlib import Path
import shutil,subprocess
from setuptools import Extension,setup
from Cython.Build import cythonize
root=Path('/build/source');out=Path('/build/native-app');(out/'scripts').mkdir(parents=True)
names=MODULES
files=[root/'scripts'/(n+'.py') if n!='native_entry' else root/'release/control-preview/native_entry.py' for n in names]
setup(name='omindos-workbench-update',ext_modules=cythonize(
 [Extension(n,[str(p)],extra_compile_args=['-O2','-g0']) for n,p in zip(names,files)],
 build_dir='/build/native-c',compiler_directives={'language_level':3,'annotation_typing':False,'binding':True,'embedsignature':False}),
 script_args=['build_ext','--build-lib',str(out/'scripts'),'--build-temp','/build/objects','-j','2'])
for f in (out/'scripts').glob('*.so'):subprocess.run(['strip','--strip-unneeded',str(f)],check=True)
for folder in ['config','web','tests','release']:shutil.copytree(root/folder,out/folder)
assert len(list((out/'scripts').glob('*.so')))==len(names)
'''.replace('names=MODULES','names='+repr(MODULES))
    (context / 'compile.py').write_text(compiler)
    dockerfile = f'''FROM {image}
RUN /opt/omindos-runtime/bin/python -m pip install --no-cache-dir Cython==3.0.12 setuptools==75.8.2 wheel==0.45.1 numpy==1.26.4 scipy==1.14.1
COPY source /build/source
COPY compile.py /build/compile.py
RUN /opt/omindos-runtime/bin/python /build/compile.py
RUN /opt/omindos-runtime/bin/python -m unittest discover -s /build/native-app/tests -p 'test_robot*.py' -v
RUN /opt/omindos-runtime/bin/python -m unittest discover -s /build/native-app/tests -p 'test_quadruped_reference*.py' -v
'''
    (context / 'Dockerfile').write_text(dockerfile)
    builder = 'omindos/reference-workbench-build:' + VERSION
    with (OUT / 'compiled-validation.log').open('w') as log:
        subprocess.run(['docker','build','--progress','plain','-t',builder,str(context)],check=True,stdout=log,stderr=subprocess.STDOUT)
    logtext = (OUT / 'compiled-validation.log').read_text()
    assert re.search(r'Ran 79 tests.*?OK',logtext,re.S) and re.search(r'Ran 3 tests.*?OK',logtext,re.S)
    container = subprocess.check_output(['docker','create',builder],text=True).strip()
    try:
        (payload / 'scripts').mkdir(exist_ok=True)
        run('docker','cp',container+':/build/native-app/scripts/.',str(payload / 'scripts'))
        licenses = payload / 'LICENSES'
        licenses.mkdir(exist_ok=True)
        run('docker','cp',container+':/opt/omindos-runtime/lib/python3.10/site-packages/scipy-1.14.1.dist-info/LICENSE.txt',str(licenses / 'Scipy-1.14.1-LICENSE.txt'))
        run('docker','cp',container+':/opt/omindos-runtime/lib/python3.10/site-packages/numpy-1.26.4.dist-info/LICENSE.txt',str(licenses / 'NumPy-1.26.4-LICENSE.txt'))
    finally:
        run('docker','rm',container)
    assert len(list((payload / 'scripts').glob('*.so'))) == len(MODULES)
    for p in payload.rglob('*'):
        if p.is_file() and (p.suffix in {'.py','.pyc','.pyo','.c','.h','.cpp','.map'} or p.name.endswith('.source.js') or 'customer-record' in p.name):
            raise ValueError('Private source or customer data in update: ' + str(p))
    report = {'status':'PASS','version':VERSION,'source_commit':os.environ.get('GITHUB_SHA'),
        'base_archive_sha256':BASE_SHA,'baseline_image_id':image_id,'compiled_modules':MODULES,
        'compiled_regression_tests':79,'reference_asset_tests':3,'python_abi':'cp310',
        'numpy':'1.26.4','scipy':'1.14.1','backend_source_included':False,'hardware_validated':False,
        'files':{str(p.relative_to(payload)):sha(p) for p in sorted(payload.rglob('*')) if p.is_file()}}
    (payload / 'WORKBENCH_UPDATE.json').write_text(json.dumps(report,indent=2)+'\n')
    result = OUT / 'omindos-workbench-reference-update.tar.gz'
    with tarfile.open(result,'w:gz',compresslevel=9) as t:
        t.add(payload,arcname='update')
    summary = {'filename':result.name,'bytes':result.stat().st_size,'sha256':sha(result),
               'status':'PASS','compiled_modules':len(MODULES),'compiled_regression_tests':79,
               'reference_asset_tests':3,'source_commit':os.environ.get('GITHUB_SHA')}
    (OUT / 'UPDATE_SUMMARY.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)


if __name__ == '__main__':
    main()
