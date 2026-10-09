"""Coordinated reference choreography. Kinematics only; no contact or actuators."""
import hashlib
import json
import math

from robot_kinematics import build_model, positions, base_position, pose, wheel_trial
from robot_parameter_api import number, validate_parameters


def structural_signature(model):
    tree = [{k: j[k] for k in ('name', 'type', 'parent', 'child', 'origin', 'axis')}
            for j in model['joints']]
    return hashlib.sha256(json.dumps(tree, sort_keys=True).encode()).hexdigest()


def sequence_trial(payload):
    allowed = {'profile', 'start', 'base_pose', 'segments', 'reference_signature'}
    if not isinstance(payload, dict) or set(payload) - allowed:
        raise ValueError('expected profile, start, base_pose and segments')
    profile = payload.get('profile')
    validate_parameters(profile)
    model = build_model(profile['kinematics']['urdf_xml'])
    if 'reference_signature' in payload and payload['reference_signature'] != structural_signature(model):
        raise ValueError('协同示例仅适用于原参考关节结构；请重新加载双七轴模型。参数编辑与普通试跑仍可使用。')
    segments = payload.get('segments')
    if not isinstance(segments, list) or not 1 <= len(segments) <= 16:
        raise ValueError('sequence requires 1–16 segments')
    q = positions(model, payload.get('start', {}))
    base = base_position(payload.get('base_pose'))
    drives = profile['dynamics']['joints']
    wheels = set((model['wheel_base'] or {}).get('wheels', {}))
    frames, phases, total = [], [], 0.0
    peaks = {n: 0.0 for n in q}
    peak_speed = {n: 0.0 for n in q}
    for segment in segments:
        if not isinstance(segment, dict) or set(segment) - {'target', 'duration_s', 'linear_m_s', 'angular_rad_s', 'label'}:
            raise ValueError('invalid sequence segment')
        duration = number(segment.get('duration_s'), 'duration_s', positive=True)
        if not .5 <= duration <= 10 or total + duration > 60:
            raise ValueError('segments require 0.5–10 seconds; total at most 60')
        label = segment.get('label', '协同动作')
        if not isinstance(label, str) or len(label) > 80:
            raise ValueError('invalid sequence label')
        target_values = segment.get('target', {})
        if not isinstance(target_values, dict) or set(target_values) & wheels:
            raise ValueError('sequence wheel angles come only from chassis odometry')
        target = positions(model, {**q, **target_values})
        for n in q:
            # Quintic transition has zero endpoint velocity/acceleration; exact peak 1.875.
            speed = 1.875*abs(target[n]-q[n])/duration
            if speed > drives[n]['velocity_limit_rad_s'] + 1e-10:
                raise ValueError(f'{n}: sequence velocity exceeds drive limit; extend segment duration')
            peak_speed[n] = max(peak_speed[n], speed)
        linear = number(segment.get('linear_m_s', 0), 'linear_m_s')
        angular = number(segment.get('angular_rad_s', 0), 'angular_rad_s')
        if linear or angular:
            wheel_motion = wheel_trial({'profile': profile, 'start': q, 'base_pose': base,
                                        'linear_m_s': linear, 'angular_rad_s': angular, 'duration_s': duration})
            samples = wheel_motion['frames']
            for n in wheels:
                peak_speed[n] = max(peak_speed[n], wheel_motion['peak_joint_velocity_rad_s'][n])
        else:
            count = math.ceil(duration*30/2)*2+1
            samples = [{'time_s': duration*i/(count-1), 'positions': q, 'base_pose': base,
                        'wheel_velocity_rad_s': {n: 0.0 for n in wheels}}
                       for i in range(count)]
        phases.append({'label': label, 'start_s': total, 'end_s': total+duration})
        for i, sample in enumerate(samples):
            if frames and i == 0:
                continue
            u = sample['time_s']/duration
            smooth = u*u*u*(10+u*(-15+6*u))
            angles = {n: (sample['positions'][n] if n in wheels else q[n]+(target[n]-q[n])*smooth) for n in q}
            frame = pose(model, angles, sample['base_pose'])
            frame.update(time_s=total+sample['time_s'], phase=label,
                         wheel_velocity_rad_s=sample['wheel_velocity_rad_s'],
                         motor_positions_rad={n: (angles[n]-drives[n]['zero_offset_rad'])*drives[n]['reduction_ratio']/drives[n]['direction'] for n in q})
            for n, value in frame['holding_torque_nm'].items():
                peaks[n] = max(peaks[n], abs(value))
            frames.append(frame)
        q, base = frames[-1]['positions'], frames[-1]['base_pose']
        total += duration
    return {'model': model, 'frames': frames, 'duration_s': total, 'phases': phases,
            'motion_type': 'cooperative_sequence', 'peak_holding_torque_nm': peaks,
            'gravity_limit_exceeded': [n for n in q if peaks[n] > drives[n]['torque_limit_nm']],
            'peak_joint_velocity_rad_s': peak_speed, 'scope': 'coordinated_kinematics_and_static_gravity',
            'runtime_applied': False, 'hardware_validated': False, 'contact_dynamics_validated': False,
            'limitations': ['双臂为预先求解的协同参考轨迹；未计算抓取力或物体接触。',
                           '四轮差速参考底盘采用轮速里程计；没有打滑、轮地接触或惯性动力学。']}
