"""Create a private compiled candidate with launcher and immutable checksums."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile

root = Path('arm64-artifact')
validation = json.loads((root/'VALIDATION.json').read_text())
assert validation['platform'] == 'linux/arm64'
for gate in ('native_module_imports', 'ros2_import', 'torch_cpu', 'mujoco_step'):
    assert validation.get(gate) is True, 'Missing validation gate: '+gate
assert validation.get('backend_source_included') is False
assert validation.get('customer_records_included') is False
assert validation.get('motor_output_enabled') is False
package = root/'omindos-control-arm64-candidate'
package.mkdir()
shutil.copy2('release/control-preview/omindos-arm64', package/'omindos')
(package/'omindos').chmod(0o755)
for name in ('VALIDATION.json', 'IMAGE_ID.txt', 'SHA256SUMS-image.txt',
             'omindos-control-linux-arm64.image.tar.gz'):
    shutil.move(root/name, package/name)
container = subprocess.check_output(['docker', 'create', 'omindos/control-arm64:candidate'], text=True).strip()
try:
    for folder in ('config', 'LICENSES'):
        subprocess.run(['docker', 'cp', container+':/opt/omindos/'+folder, str(package/folder)], check=True)
finally:
    subprocess.run(['docker', 'rm', container], check=True, stdout=subprocess.DEVNULL)
for name in ('THIRD_PARTY.md',):
    shutil.copy2('release/control-preview/'+name, package/name)
(package/'README.zh-CN.md').write_text('''# ARM64 编译候选包

Linux ARM64、Docker Engine；当前仅支持软件仿真与调参，不访问 USB/CAN 电机。
此包未完成 Jetson 实测，不含 CUDA、TensorRT 或已验收的 DM-MC02 固件。

首次运行：`./omindos workbench`，浏览器打开 http://localhost:8085。
ROS 工作台：`./omindos ros-preview`。其他模式运行 `./omindos --help` 查看。
四足参考仿真：`./omindos quadruped`，另一个终端执行 `./omindos quadruped-ros`。
这只是原版 MEVIUS2 仿真，不代表客户 RS04 的整机动力学或步态验收。
候选版本按 MANIFEST.json 的 source_commit 和 IMAGE_ID.txt 的镜像 ID 复现。
不要作为机器人现场控制发行版；下载后须先核对外层 SHA256 和内部镜像校验。
''')
def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()
manifest = {'platform': 'linux/arm64', 'stage': 'compiled_candidate',
            'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
            'jetson_hardware_tested': False, 'hardware_adapter_ready': False,
            'files': {str(f.relative_to(package)): sha(f) for f in sorted(package.rglob('*')) if f.is_file()}}
(package/'MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n')
archive = root/(package.name+'.tar')
with tarfile.open(archive, 'w') as stream:
    stream.add(package, arcname=package.name)
(root/'SHA256SUMS-release.txt').write_text(sha(archive)+'  '+archive.name+'\n')
shutil.rmtree(package)
