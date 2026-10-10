"""Local workbench configuration and software stop latch; no hardware transport."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading

from robot_parameter_api import (register_robot_parameters, load_robot_profile,
                                 validate_parameters, DRIVE_FIELDS, ROBOT_ID, PROFILE_ID, number)
from robot_kinematics import build_model


KINEMATIC_MODE = 'kinematic_preview'


def _validate_workbench_profile(profile):
    """Explicit preview opt-in; never relax the controller parameter registry."""
    if not isinstance(profile, dict) or profile.get('profile_mode') != KINEMATIC_MODE:
        return validate_parameters(profile)
    required = {'profile_mode', 'robot_id', 'kinematics'}
    if not required <= set(profile) or set(profile) - required - {'dynamics', 'connections'}:
        raise ValueError('expected explicit kinematic preview profile')
    rid = profile['robot_id']
    if not isinstance(rid, str) or not ROBOT_ID.fullmatch(rid):
        raise ValueError('invalid robot_id')
    kin = profile['kinematics']
    if not isinstance(kin, dict) or set(kin) != {'urdf_xml'}:
        raise ValueError('kinematics must contain urdf_xml')
    if 'connections' in profile:
        from robot_connections import validate_connections
        validate_connections(profile['connections'])
    model = build_model(kin['urdf_xml'])
    names = [joint['name'] for joint in model['joints'] if joint['type'] != 'fixed']
    if not names:
        raise ValueError('kinematic preview requires an active joint')
    # Preserve optional motor drafts, including unknown values, without filling
    # masses, inertias, encoder zeros, motor directions or effort limits.
    if 'dynamics' in profile:
        dyn = profile['dynamics']
        if (not isinstance(dyn, dict) or set(dyn) != {'joints'}
                or not isinstance(dyn['joints'], dict) or set(dyn['joints']) != set(names)):
            raise ValueError('dynamics.joints drafts must match active URDF joints')
        for name, drive in dyn['joints'].items():
            if not isinstance(drive, dict) or set(drive) != DRIVE_FIELDS:
                raise ValueError(name + ': expected all six drive draft fields')
            motor = drive['motor_model']
            if motor is not None and not isinstance(motor, str):
                raise ValueError(name + ': motor_model draft must be text or null')
            direction = drive['direction']
            if direction is not None and (type(direction) is not int or direction not in (-1, 1)):
                raise ValueError(name + ': direction draft must be 1, -1 or null')
            for key in ('reduction_ratio', 'zero_offset_rad', 'torque_limit_nm', 'velocity_limit_rad_s'):
                value = drive[key]
                if value is None:
                    continue
                if type(value) not in (float, int):
                    raise ValueError(name + '.' + key + ' draft must be a JSON number or null')
                number(value, name + '.' + key, positive=(key != 'zero_offset_rad'))
    return {'robot_id': rid, 'joint_names': names, 'profile_mode': KINEMATIC_MODE,
            'validation_scope': 'kinematic_configuration_only',
            'runtime_applied': False, 'hardware_validated': False, 'dynamics_validated': False}


def _register_workbench_profile(profile, store_dir):
    metadata = _validate_workbench_profile(profile)
    if profile.get('profile_mode') != KINEMATIC_MODE:
        return register_robot_parameters(profile, store_dir)
    canonical = json.dumps(profile, sort_keys=True, ensure_ascii=False,
                           allow_nan=False, separators=(',', ':'))
    profile_id = hashlib.sha256(canonical.encode('utf-8')).hexdigest()
    folder = Path(store_dir)
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / (profile_id + '.json')
    # Publish only a fully written version. A simultaneous identical save must
    # not observe a partially written JSON file or overwrite an older version.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=folder,
                                         prefix='.kinematic-', delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(canonical + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        try:
            os.link(temporary, target)
        except FileExistsError:
            if target.read_text(encoding='utf-8').strip() != canonical:
                raise ValueError('stored profile content mismatch')
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {'profile_id': profile_id, **metadata}


def _load_workbench_profile(profile_id, store_dir):
    if not isinstance(profile_id, str) or not PROFILE_ID.fullmatch(profile_id):
        raise ValueError('invalid profile_id')
    payload = json.loads((Path(store_dir) / (profile_id + '.json')).read_text(encoding='utf-8'))
    if not isinstance(payload, dict) or payload.get('profile_mode') != KINEMATIC_MODE:
        return load_robot_profile(profile_id, store_dir)
    canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False,
                           allow_nan=False, separators=(',', ':'))
    if hashlib.sha256(canonical.encode('utf-8')).hexdigest() != profile_id:
        raise ValueError('stored profile checksum mismatch')
    _validate_workbench_profile(payload)
    return payload


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
            return _load_workbench_profile(self.profile_id, self.folder) if self.profile_id else None

    def apply(self, profile):
        _validate_workbench_profile(profile)
        build_model(profile['kinematics']['urdf_xml'])
        with self.lock:
            saved = _register_workbench_profile(profile, self.folder)
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
