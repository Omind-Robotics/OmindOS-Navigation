"""Checks run inside the final ARM64 image. No source or device access."""
import importlib
import json
from pathlib import Path
import platform
import struct

assert platform.machine() == 'aarch64'
modules = Path('/opt/omindos/scripts')
for binary in modules.glob('*.so'):
    header = binary.read_bytes()[:20]
    assert header[:4] == b'\x7fELF' and header[4] == 2
    assert struct.unpack('<H', header[18:20])[0] == 183, str(binary)
    importlib.import_module(binary.name.split('.')[0])
assert len(list(modules.glob('*.so'))) >= 19
assert not [f for f in Path('/opt/omindos').rglob('*')
            if f.is_file() and f.suffix in ('.py', '.pyc', '.c', '.h', '.cpp')]
assert not list(Path('/opt/omindos').rglob('*.customer-record.json'))
import rclpy
import torch
import mujoco
from dm_mc02_protocol import Packet, PROBE
assert Packet.decode(Packet(PROBE).encode()).kind == PROBE
assert torch.ones(2).sum().item() == 2
model = mujoco.MjModel.from_xml_path('/opt/omindos/reference/models/scene.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
print(json.dumps({'platform': 'linux/arm64', 'native_module_imports': True,
                  'ros2_import': True, 'torch_cpu': True, 'mujoco_step': True,
                  'backend_source_included': False, 'customer_records_included': False,
                  'jetson_hardware_tested': False, 'motor_output_enabled': False}))
