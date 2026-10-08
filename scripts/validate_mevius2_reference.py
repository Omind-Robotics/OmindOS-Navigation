#!/usr/bin/env python3
"""Validate the pinned MEVIUS2 policy in headless MuJoCo. No ROS, CAN or hardware I/O.

Observation/scaling follows haraduka/mevius2, Copyright (c) 2025
Kento Kawaharazuka, MIT. See docs/MEVIUS2_INTEGRATION.zh-CN.md.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess

PINNED_COMMIT = "4f09680bb575574377b903bbc971bdb2695d507b"
POLICY_GIT_BLOB = "2d0395c86d32b82f5fe1567049f5961bc8e186db"


def parameters(path):
    values = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            values[node.targets[0].id] = ast.literal_eval(node.value)
    return values


def policy_observation(np, quaternion_xyzw, angular_velocity, command, q, dq, default):
    """34 values in upstream deployment joint order; gyro is already body-frame."""
    x, y, z, w = quaternion_xyzw
    gravity = np.array([2 * (w*y - x*z), -2 * (y*z + w*x), 2*(x*x+y*y) - 1])
    symmetry = np.array([1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1])
    return np.concatenate([
        angular_velocity * 0.25, gravity, command * np.array([2.0, 2.0, 0.25]),
        (q - default) * symmetry, dq * 0.05 * symmetry,
        [float(np.linalg.norm(command) < 0.03)],
    ]).astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    actual_commit = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if actual_commit != PINNED_COMMIT:
        raise SystemExit("Upstream commit mismatch")
    subprocess.run(["git", "-C", str(source), "diff", "--exit-code", "HEAD", "--"],
                   check=True, stdout=subprocess.DEVNULL)
    policy_path = source / "models/policy.pt"
    policy_bytes = policy_path.read_bytes()
    git_digest = hashlib.sha1(b"blob " + str(len(policy_bytes)).encode() + b"\0" + policy_bytes).hexdigest()
    if git_digest != POLICY_GIT_BLOB:
        raise SystemExit("Policy artifact mismatch")

    import numpy as np
    import torch
    import mujoco
    torch.set_num_threads(1)
    p = parameters(source / "scripts/parameters.py")
    policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
    model = mujoco.MjModel.from_xml_path(str(source / "models/scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    names = p["JOINT_NAME"]
    qids = [model.jnt_qposadr[model.joint(name).id] for name in names]
    vids = [model.jnt_dofadr[model.joint(name).id] for name in names]
    aids = [model.actuator(name).id for name in names]
    default = np.array(p["DEFAULT_ANGLE"])
    standby = np.array(p["STANDBY_ANGLE"])
    kp, kd = np.array(p["KP_GAIN"]), np.array(p["KD_GAIN"])
    symmetry = np.array([1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1])
    dt = float(model.opt.timestep)
    decimation = round(1.0 / p["CONTROL_HZ"] / dt)
    if len(names) != 12 or decimation != 4 or abs(dt - 0.005) > 1e-12:
        raise SystemExit("Unexpected reference model/control dimensions")
    stages = [
        ("standby", 1.0, None), ("standup", 3.0, None),
        ("policy_stand", 3.0, [0.0, 0.0, 0.0]),
        ("policy_forward", 8.0, [0.2, 0.0, 0.0]),
        ("policy_stop", 5.0, [0.0, 0.0, 0.0]),
    ]
    summaries, snapshots = [], []
    finite = True
    inference_count = 0
    target = standby.copy()
    for label, seconds, body_command in stages:
        start = data.qpos[:3].copy()
        min_height, min_upright = math.inf, math.inf
        max_abs_torque = 0.0
        speeds = []
        initial = data.qpos[qids].copy()
        for step in range(round(seconds / dt)):
            if body_command is None:
                ratio = min((step+1) * dt / seconds, 1.0)
                target = standby if label == "standby" else initial + ratio * (default - initial)
            elif step % decimation == 0:
                quat_wxyz = data.sensor("body_quat_sensor").data.copy()
                obs = policy_observation(np, quat_wxyz[[1, 2, 3, 0]],
                    data.sensor("body_gyro_sensor").data.copy(), np.array(body_command),
                    data.qpos[qids], data.qvel[vids], default)
                with torch.inference_mode():
                    action = policy(torch.from_numpy(np.clip(obs, -100, 100)).reshape(1, 34))
                action = action.detach().cpu().numpy()
                if action.shape != (1, 12) or not np.isfinite(action).all():
                    raise RuntimeError("Invalid policy output")
                target = default + p["ACTION_SCALE"] * np.clip(action[0] * symmetry, -100, 100)
                inference_count += 1
            torque = kp * (target - data.qpos[qids]) - kd * data.qvel[vids]
            data.ctrl[aids] = torque
            mujoco.mj_step(model, data)
            finite = finite and bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
            if not finite:
                raise RuntimeError("Non-finite simulation state")
            body = data.body("base_link")
            height, upright = float(body.xpos[2]), float(body.xmat[8])
            min_height, min_upright = min(min_height, height), min(min_upright, upright)
            max_abs_torque = max(max_abs_torque, float(np.max(np.abs(data.actuator_force))))
            speed = float(np.linalg.norm(data.sensor("body_vel_sensor").data[:2]))
            speeds.append(speed)
            if step % round(0.1 / dt) == 0:
                snapshots.append({"time": round(float(data.time), 4), "stage": label,
                    "base_position": data.qpos[:3].tolist(), "upright": upright,
                    "body_planar_speed": speed})
        delta = data.qpos[:3] - start
        summaries.append({"stage": label, "duration_seconds": seconds,
            "displacement_xyz_m": delta.tolist(), "minimum_base_height_m": min_height,
            "minimum_upright_cosine": min_upright, "maximum_actuator_torque_nm": max_abs_torque,
            "final_one_second_mean_planar_speed_m_s": float(np.mean(speeds[-round(1/dt):]))})
    policy_stages = summaries[2:]
    checks = {
        "finite_simulation": finite,
        "policy_io_34_to_12": inference_count > 0,
        "upright_during_policy_stages": all(s["minimum_upright_cosine"] > 0.8 for s in policy_stages),
        "base_clearance_during_policy_stages": all(s["minimum_base_height_m"] > 0.2 for s in policy_stages),
        "forward_displacement_above_0_3m": summaries[3]["displacement_xyz_m"][0] > 0.3,
        "zero_command_final_speed_below_0_15m_s": summaries[4]["final_one_second_mean_planar_speed_m_s"] < 0.15,
    }
    report = {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
        "scope": "pinned_upstream_headless_mujoco_reference_only", "hardware_tested": False,
        "ros2_integration_tested": False, "customer_robot_model_tested": False,
        "upstream": "https://github.com/haraduka/mevius2", "commit": actual_commit,
        "policy_sha256": hashlib.sha256(policy_bytes).hexdigest(),
        "versions": {"torch": torch.__version__, "mujoco": mujoco.__version__, "numpy": np.__version__},
        "physics_hz": 1/dt, "policy_hz": p["CONTROL_HZ"], "joint_order": names,
        "policy_inferences": inference_count, "stages": summaries, "trace": snapshots}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k:v for k,v in report.items() if k != "trace"}, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
