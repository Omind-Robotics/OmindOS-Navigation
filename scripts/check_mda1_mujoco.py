#!/usr/bin/env python3
"""Offline fixed-base import/gravity check. Requires MuJoCo; no contact validation."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robot_kinematics import build_model, pose

ROOT = Path(__file__).resolve().parents[1]


def check():
    profile = json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())
    root = ET.fromstring(profile['kinematics']['urdf_xml'])
    eigenvalues = []
    for link in root.findall('link'):
        inertia = link.find('inertial/inertia')
        xx, yy, zz, xy, xz, yz = [float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')]
        values = np.linalg.eigvalsh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        assert values.min() > 0 and values[-1] <= sum(values[:-1])+1e-10, link.get('name')
        eigenvalues.extend(values.tolist())
    ET.SubElement(ET.SubElement(root, 'mujoco'), 'compiler',
                  meshdir=str(ROOT/'config/robot_models/realman_rm75/meshes'),
                  discardvisual='false', fusestatic='false')
    for mesh in root.findall('.//mesh'):
        mesh.set('filename', mesh.get('filename').split('/')[-1])
    physics = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    physics.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    physics.opt.timestep = .002
    physics.opt.gravity[:] = [0, 0, -9.80665]
    kinematics = build_model(profile['kinematics']['urdf_xml'])
    recipe = json.loads((ROOT/'web/robot-workbench/cooperative-demo.json').read_text())
    q, max_error, max_drift = {}, 0.0, 0.0
    for segment in recipe['segments']:
        q.update(segment['target'])
        state = mujoco.MjData(physics)
        for name, angle in q.items():
            joint = mujoco.mj_name2id(physics, mujoco.mjtObj.mjOBJ_JOINT, name)
            state.qpos[physics.jnt_qposadr[joint]] = angle
        mujoco.mj_forward(physics, state)
        torque = pose(kinematics, q)['holding_torque_nm']
        for name, value in torque.items():
            joint = mujoco.mj_name2id(physics, mujoco.mjtObj.mjOBJ_JOINT, name)
            max_error = max(max_error, abs(state.qfrc_bias[physics.jnt_dofadr[joint]]-value))
        start = state.qpos.copy()
        for _ in range(1000):
            mujoco.mj_forward(physics, state)
            state.qfrc_applied[:] = state.qfrc_bias
            mujoco.mj_step(physics, state)
            assert np.isfinite(state.qpos).all() and np.isfinite(state.qvel).all()
        max_drift = max(max_drift, float(np.max(np.abs(state.qpos-start))))
    assert max_error < 1e-5 and max_drift < 1e-7, (max_error, max_drift)
    return {'status': 'PASS', 'mujoco_version': mujoco.__version__,
            'imported_dof': int(physics.nv), 'bodies_including_world': int(physics.nbody),
            'mesh_assets': int(physics.nmesh), 'inertia_min_eigenvalue': min(eigenvalues),
            'poses_checked': len(recipe['segments']), 'integration_s_per_pose': 2,
            'gravity_torque_max_error_nm': max_error, 'ideal_gravity_hold_max_drift_rad': max_drift,
            'base_fixed': True, 'contacts_enabled': False,
            'contact_dynamics_validated': False, 'hardware_commands': False,
            'reference_parameters': ['body/mount/head/gripper/wheel mass and inertia',
                                     'mount dimensions', 'all drive ratios and limits'],
            'missing_for_real_contacts': ['actual base model (DT-01 Pro vs current four-wheel reference)',
                                         'whole-body CAD and installation calibration',
                                         'real gripper model, stroke, effort and collision shapes',
                                         'wheel friction/compliance and payload properties',
                                         'measured actuator/damping/friction/controller parameters'],
            'scope': 'Fixed-base URDF import, inertia checks, static gravity comparison and ideal compensated integration only'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = check()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False))
