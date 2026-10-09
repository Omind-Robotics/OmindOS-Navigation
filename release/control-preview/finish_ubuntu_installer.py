#!/usr/bin/env python3
"""Build and verify the complete Ubuntu installer from pinned compiled bytes."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/ubuntu-full-installer'
VERSION = '0.2.0-preview.7'
BASE_NAME = 'omindos-control-preview-0.2.0-preview.5-size-candidate.1-linux-amd64.tar'
BASE_SHA = 'af2cc32308c0ffc0a14b26cd71f454b8b9566096573ee09fb09f191ec7eedbd8'
UPDATE_SHA = '4a4f32b4e595fae89b87f3f6140e1f17ee7d44783ce15108df5993b8b09fdf00'
IMAGE = 'omindos/navigation:' + VERSION
PYTHON = '/opt/omindos-runtime/bin/python'
IDENTITY_FORMAT = '{{json .RootFS.Layers}}|{{.Architecture}}|{{.Os}}|{{json .Config.Env}}|{{json .Config.Entrypoint}}|{{json .Config.Cmd}}|{{.Config.WorkingDir}}|{{.Config.User}}'


def run(*args, **kwargs):
    return subprocess.run([str(a) for a in args], check=True, **kwargs)


def output(*args):
    return subprocess.check_output([str(a) for a in args], text=True).strip()


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def request(path, payload=None):
    from urllib.request import Request
    data = None if payload is None else json.dumps(payload).encode()
    req = Request('http://127.0.0.1:8085' + path, data=data,
                  headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=10) as response:
        raw = response.read()
        return json.loads(raw) if 'application/json' in response.headers.get('Content-Type', '') else raw


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    archive = OUT / BASE_NAME
    if not archive.exists() or sha(archive) != BASE_SHA:
        run('curl', '-fL', '--retry', '3', '-o', archive,
            'https://github.com/maganrobotics-boop/OmindOS-Navigation-Release/releases/download/v0.2.0-preview.6/' + BASE_NAME)
    assert archive.stat().st_size == 540416000 and sha(archive) == BASE_SHA
    base = OUT / 'base'
    with tarfile.open(archive) as tar:
        tar.extractall(base, filter='data')
    base = next(base.glob('omindos-control-preview-*'))
    base_image = re.search(r"^image='([^']+)'", (base / 'omindos').read_text(), re.M).group(1)
    run('sha256sum', '-c', 'SHA256SUMS-image.txt', cwd=base)
    run('docker', 'load', '-i', base / 'runtime.image.tar.gz')
    update = ROOT / 'release/compiled-update/omindos-workbench-reference-update.tar.gz'
    assert sha(update) == UPDATE_SHA
    context = OUT / 'context'
    context.mkdir(exist_ok=True)
    with tarfile.open(update) as tar:
        tar.extractall(context, filter='data')
    metadata = json.loads((context / 'update/WORKBENCH_UPDATE.json').read_text())
    assert metadata['compiled_regression_tests'] == 79 and metadata['reference_asset_tests'] == 3
    for name, digest in metadata['files'].items():
        assert sha(context / 'update' / name) == digest, name
    # Reproduce whether the old runtime can load the new compiled update.
    probe = subprocess.run(['docker','run','--rm','--network','none','--entrypoint',PYTHON,
        '-v',str(context / 'update/scripts') + ':/update:ro', base_image,
        '-c',"import sys;sys.path.insert(0,'/update');import robot_workbench"],
        text=True, capture_output=True)
    diagnosis = {'old_runtime_update_import_exit': probe.returncode,
                 'old_runtime_update_import_stderr': probe.stderr[-6000:]}
    dump(OUT / 'STARTUP_DIAGNOSIS.json', diagnosis)
    print(json.dumps(diagnosis), flush=True)
    (context / 'Dockerfile').write_text(f'''FROM {base_image}
RUN {PYTHON} -m pip install --no-cache-dir --only-binary=:all: --index-url https://pypi.org/simple numpy==1.26.4 scipy==1.14.1
COPY update/ /opt/omindos/
RUN PYTHONPATH=/opt/omindos/scripts {PYTHON} -c "import robot_workbench,robot_navigation,robot_planning,numpy,scipy; assert numpy.__version__=='1.26.4'; assert scipy.__version__=='1.14.1'"
''')
    run('docker','build','--progress','plain','-t',IMAGE,context)
    assert output('docker','run','--rm','--network','none',IMAGE,'--version') == VERSION
    # Audit only application-owned files; third-party Python/ROS libraries retain their licenses and source.
    audit = "from pathlib import Path;p=Path('/opt/omindos');bad=[str(f) for f in p.rglob('*') if f.is_file() and (f.suffix in {'.py','.pyc','.pyo','.c','.h','.cpp','.map'} or f.name.endswith('.source.js') or 'customer-record' in f.name)];assert not bad,bad;print('APPLICATION_AUDIT_PASS')"
    run('docker','run','--rm','--network','none','--entrypoint',PYTHON,IMAGE,'-c',audit)
    distribution = OUT / 'distribution'
    package = distribution / ('omindos-navigation-' + VERSION + '-ubuntu-amd64')
    package.mkdir(parents=True, exist_ok=False)
    for name in ['LICENSES','config']:
        shutil.copytree(base / name, package / name)
    shutil.copytree(context / 'update/LICENSES', package / 'LICENSES', dirs_exist_ok=True)
    for name in ['THIRD_PARTY.md','THIRD_PARTY_PACKAGES.txt']:
        shutil.copy2(base / name, package / name)
    with (package / 'THIRD_PARTY_PYTHON.json').open('w') as f:
        run('docker','run','--rm','--network','none','--entrypoint',PYTHON,IMAGE,'-m','pip','list','--format=json',stdout=f)
    with (package / 'runtime.image.tar.gz').open('wb') as f:
        save = subprocess.Popen(['docker','save',IMAGE], stdout=subprocess.PIPE)
        gzip = subprocess.run(['gzip','-1'], stdin=save.stdout, stdout=f)
        save.stdout.close()
        assert save.wait() == gzip.returncode == 0
    (package / 'SHA256SUMS-image.txt').write_text(sha(package / 'runtime.image.tar.gz') + '  runtime.image.tar.gz\n')
    identity = output('docker','image','inspect','--format',IDENTITY_FORMAT,IMAGE)
    (package / 'IMAGE_IDENTITY.sha256').write_text(hashlib.sha256((identity+'\n').encode()).hexdigest()+'\n')
    launcher = '''#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
[ "$(uname -s)" = Linux ] && [ "$(uname -m)" = x86_64 ] || { echo '需要 Ubuntu x86_64。' >&2; exit 2; }
command -v docker >/dev/null || { echo '请先安装 Docker Engine。' >&2; exit 2; }
image='IMAGE'
identity_format='FORMAT'
image_known() {
  actual=$(docker image inspect --format "$identity_format" "$image" 2>/dev/null | sha256sum | cut -d ' ' -f 1)
  [ "$actual" = "$(cat IMAGE_IDENTITY.sha256)" ]
}
if [ "$#" -eq 0 ]; then set -- workbench; fi
case "$1" in
  verify) sha256sum -c SHA256SUMS-image.txt; exit ;;
esac
if ! image_known; then
  sha256sum -c SHA256SUMS-image.txt
  docker load -i runtime.image.tar.gz
fi
image_known || { echo '运行镜像完整性验证失败。' >&2; exit 2; }
if [ "$1" = install ]; then echo '运行环境安装完成。'; exit; fi
data_dir=${OMINDOS_DATA_DIR:-"$HOME/.local/share/omindos-navigation/data"}
mkdir -p "$data_dir"
data_dir=$(CDPATH= cd -- "$data_dir" && pwd)
case "$1" in
  workbench|parameter-api|ros-preview) set -- "$@" --store /data/profiles ;;
  quadruped|quadruped-ros) set -- "$@" --token-file /data/quadruped.token ;;
  validate-dynamics) set -- "$@" --source /opt/omindos/reference --output /data/dynamics.json ;;
esac
exec docker run --rm --init --network host --user "$(id -u):$(id -g)" \\
  --cap-drop ALL --security-opt no-new-privileges \\
  -e HOME=/tmp -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \\
  -v "$data_dir:/data" "$image" "$@"
'''.replace("image='IMAGE'", "image='"+IMAGE+"'").replace("identity_format='FORMAT'", "identity_format='"+IDENTITY_FORMAT+"'")
    (package / 'omindos').write_text(launcher)
    (package / 'omindos').chmod(0o755)
    installer = '''#!/bin/sh
set -eu
source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
prefix=${HOME}/.local/opt/omindos-navigation/VERSION
if [ "$#" -eq 2 ] && [ "$1" = --prefix ]; then prefix=$2
elif [ "$#" -ne 0 ]; then echo '用法：./install.sh [--prefix 安装目录]' >&2; exit 2; fi
[ ! -e "$prefix" ] || { echo '安装目录已存在；请使用其中的 omindos 启动，或指定新目录。' >&2; exit 2; }
"$source_dir/omindos" verify
mkdir -p "$prefix"
cp -a "$source_dir/." "$prefix/"
"$prefix/omindos" install
printf '安装完成。启动命令："%s/omindos" workbench\\n浏览器地址：http://127.0.0.1:8085/\\n' "$prefix"
'''.replace('VERSION',VERSION)
    (package / 'install.sh').write_text(installer)
    (package / 'install.sh').chmod(0o755)
    (package / 'README.zh-CN.md').write_text('''# OmindOS Navigation Ubuntu 完整安装包

适用 Ubuntu 22.04 / 24.04、Intel/AMD 64 位电脑。需要预先安装 Docker Engine，并使当前用户能运行 docker；不需要另装 Python 算法环境、ROS 2 或单独工作台。

解压后，在该目录执行 `./install.sh`，随后按输出的启动命令运行工作台。在本机浏览器打开 http://127.0.0.1:8085/ ，无需登录。也可直接运行 `./omindos workbench`。

参数保存于 `~/.local/share/omindos-navigation/data`，升级保留。可通过环境变量 `OMINDOS_DATA_DIR` 指定已有数据目录。新目录默认加载完整 MEVIUS2 四足参考模型；旧配置不会被覆盖。

`./omindos validate-dynamics` 执行四组四足动力学仿真；`./omindos ros-preview` 启动 ROS 2 预览；`./omindos verify` 校验运行镜像。

工作台支持导入客户 URDF、参数编辑、保存和本地 HTTP API。三维调参与四足策略仿真为独立功能。轮式导航独立环境见下载主页专业指南。本版为预览发行，未进行客户真实机器人或电机验收。
''')
    for name in ['omindos','install.sh']:
        run('sh','-n',package / name)
    archive = distribution / (package.name + '.tar')
    with tarfile.open(archive, 'w') as tar:
        tar.add(package, arcname=package.name)
    # Re-extract the actual delivery, remove the build image, then install afresh.
    fresh = OUT / 'fresh extraction'
    with tarfile.open(archive) as tar:
        tar.extractall(fresh, filter='data')
    run('docker','image','rm',IMAGE)
    installed = OUT / 'fresh installation'
    run(fresh / package.name / 'install.sh','--prefix',installed)
    env = {**os.environ, 'OMINDOS_DATA_DIR':str(OUT / 'clean-user-data')}
    log = (OUT / 'workbench.log').open('w')
    server = subprocess.Popen([str(installed / 'omindos'),'workbench'],env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        for _ in range(90):
            if server.poll() is not None:
                raise RuntimeError('Workbench exited: ' + (OUT / 'workbench.log').read_text())
            try:
                html = request('/')
                break
            except Exception:
                time.sleep(1)
        else:
            raise RuntimeError('Workbench HTTP timeout: ' + (OUT / 'workbench.log').read_text())
        assert b'<html' in html.lower()
        checks = ['fresh_archive_extraction','clean_image_load','install_script_path_with_spaces','http_root_200']
        for path in ['/workbench.js','/robot-scene.js','/workbench.css','/example.json','/v1/workbench-state']:
            assert request(path)
            checks.append(path)
        run('node',ROOT / 'scripts/capture_installed_quadruped.cjs',OUT / 'browser',env=env)
        checks.append('browser_13_meshes_12_joints_apply_jog_stop')
    finally:
        server.terminate()
        try: server.wait(20)
        except subprocess.TimeoutExpired: server.kill();server.wait()
        log.close()
    run(installed / 'omindos','validate-dynamics',env=env)
    dynamics = json.loads((OUT / 'clean-user-data/dynamics.json').read_text())
    assert dynamics['status'] == 'PASS' and len(dynamics['scenarios']) == 4
    checks.append('four_quadruped_contact_dynamics_scenarios')
    report = {'status':'PASS','version':VERSION,'checks':checks,'diagnosis':diagnosis,
        'compiled_update_sha256':UPDATE_SHA,'compiled_source_commit':metadata['source_commit'],
        'compiled_regression_tests':79,'reference_asset_tests':3,'hardware_validated':False,
        'numpy':'1.26.4','scipy':'1.14.1','backend_source_included':False}
    dump(distribution / 'VALIDATION.json',report)
    dump(distribution / 'DYNAMICS_VALIDATION.json',dynamics)
    shutil.copy2(OUT / 'browser/CAPTURE.json',distribution / 'BROWSER_VALIDATION.json')
    shutil.copy2(OUT / 'browser/workbench.png',distribution / 'workbench.png')
    summary = {'filename':archive.name,'bytes':archive.stat().st_size,'sha256':sha(archive),
        'version':VERSION,'source_commit':os.environ.get('GITHUB_SHA'),
        'compiled_update_sha256':UPDATE_SHA,'status':'PASS','hardware_validated':False}
    dump(distribution / 'RELEASE_SUMMARY.json',summary)
    (distribution / 'SHA256SUMS-release.txt').write_text(summary['sha256']+'  '+archive.name+'\n')
    print(json.dumps(summary),flush=True)


if __name__ == '__main__':
    main()
