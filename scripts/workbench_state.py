"""Local workbench configuration and software stop latch; no hardware transport."""
import json
import os
from pathlib import Path
import tempfile
import threading

from robot_parameter_api import register_robot_parameters, load_robot_profile, validate_parameters
from robot_kinematics import build_model


class MotionLocked(ValueError):
    pass


class WorkbenchState:
    def __init__(self, store_dir):
        self.folder = Path(store_dir)
        self.lock = threading.RLock()
        self.emergency_stop = False
        self.epoch = 0
        self.profile_id = None
        pointer = self.folder / 'workbench-active.json'
        if pointer.exists():
            self.profile_id = json.loads(pointer.read_text())['profile_id']
            self.active_profile()  # Validate the version before using it on startup.

    def snapshot(self):
        with self.lock:
            return {'active_profile_id': self.profile_id, 'emergency_stop': self.emergency_stop,
                    'motion_epoch': self.epoch, 'hardware_connected': False}

    def active_profile(self):
        with self.lock:
            return load_robot_profile(self.profile_id, self.folder) if self.profile_id else None

    def apply(self, profile):
        validate_parameters(profile)
        build_model(profile['kinematics']['urdf_xml'])
        with self.lock:
            saved = register_robot_parameters(profile, self.folder)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile('w', dir=self.folder, prefix='.workbench-', delete=False) as handle:
                    temporary = Path(handle.name)
                    json.dump({'profile_id': saved['profile_id']}, handle)
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temporary, self.folder / 'workbench-active.json')
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            self.profile_id = saved['profile_id']
            self.epoch += 1
            return {**saved, **self.snapshot(), 'workbench_applied': True}

    def set_stop(self, payload):
        if not isinstance(payload, dict) or set(payload) - {'engaged', 'expected_epoch'} or type(payload.get('engaged')) is not bool:
            raise ValueError('expected engaged boolean and optional expected_epoch')
        with self.lock:
            if not payload['engaged'] and (type(payload.get('expected_epoch')) is not int or payload['expected_epoch'] != self.epoch):
                raise MotionLocked('状态已改变，请重新确认后解除急停。')
            self.emergency_stop = payload['engaged']
            self.epoch += 1
            return self.snapshot()

    def check_motion(self, expected_epoch=None):
        with self.lock:
            if self.emergency_stop or (expected_epoch is not None and expected_epoch != self.epoch):
                raise MotionLocked('急停或参数状态已改变，本次运动已取消。')
            return self.epoch
