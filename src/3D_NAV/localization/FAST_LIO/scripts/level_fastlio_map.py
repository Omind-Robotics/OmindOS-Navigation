#!/usr/bin/env python3
"""Level FAST-LIO MID360 map outputs by aligning the dominant ground plane to +Z."""

import argparse
import math
import os
from pathlib import Path

import numpy as np


def load_binary_pcd(path):
    header_lines = []
    header = {}
    with open(path, "rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError(f"{path}: missing DATA line")
            header_lines.append(line)
            text = line.decode("latin1").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            header[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                if parts[1] != "binary":
                    raise RuntimeError(f"{path}: only binary PCD is supported, got DATA {parts[1]}")
                break

        fields = header.get("FIELDS", [])
        sizes = [int(x) for x in header.get("SIZE", [])]
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
        types = header.get("TYPE", [])
        points = int(header["POINTS"][0])
        if fields[:3] != ["x", "y", "z"] or sizes[:3] != [4, 4, 4] or types[:3] != ["F", "F", "F"]:
            raise RuntimeError(f"{path}: expected x/y/z float32 as the first three fields")

        point_step = sum(size * count for size, count in zip(sizes, counts))
        raw = handle.read(point_step * points)
        if len(raw) != point_step * points:
            raise RuntimeError(f"{path}: truncated PCD data")

    if point_step % 4 != 0:
        raise RuntimeError(f"{path}: point step {point_step} is not float32-aligned")

    data = np.frombuffer(raw, dtype=np.float32).reshape(points, point_step // 4).copy()
    return header_lines, data


def write_binary_pcd(path, header_lines, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as handle:
        for line in header_lines:
            handle.write(line)
        handle.write(data.astype(np.float32, copy=False).tobytes())


def fit_plane_svd(points):
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    return normal, centroid


def ransac_ground_plane(points, sample_size, iterations, threshold, min_up_dot, seed):
    rng = np.random.default_rng(seed)
    sample_count = min(sample_size, len(points))
    sample_indices = rng.choice(len(points), size=sample_count, replace=False)
    sample = points[sample_indices]

    best_inliers = None
    best_count = 0
    for _ in range(iterations):
        ids = rng.choice(sample_count, size=3, replace=False)
        p0, p1, p2 = sample[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < min_up_dot:
            continue

        distances = np.abs((sample - p0) @ normal)
        inliers = distances < threshold
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 100:
        raise RuntimeError("failed to find a dominant upward ground plane")

    return fit_plane_svd(sample[best_inliers])


def rotation_align_vectors(src, dst):
    src = src / np.linalg.norm(src)
    dst = dst / np.linalg.norm(dst)
    cross = np.cross(src, dst)
    dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))
    norm = np.linalg.norm(cross)

    if norm < 1e-9:
        return np.eye(3) if dot > 0 else np.diag([1.0, -1.0, -1.0])

    axis = cross / norm
    kx = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    angle = math.acos(dot)
    return np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx)


def matrix_to_quaternion(matrix):
    m = matrix
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    else:
        axis = int(np.argmax(np.diag(m)))
        if axis == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif axis == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def matrix_to_rpy_deg(matrix):
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return np.degrees([roll, pitch, yaw])


def plane_tilt_deg(normal):
    return math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))


def transform_trajectory(input_path, output_path, rotation, rotation_quat):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    changed = 0
    with open(input_path, "r", encoding="utf-8", errors="ignore") as src, open(
        output_path, "w", encoding="utf-8"
    ) as dst:
        for line in src:
            parts = line.split()
            if len(parts) < 8:
                dst.write(line)
                continue

            values = [float(x) for x in parts]
            position = np.array(values[1:4], dtype=np.float64)
            quat = np.array(values[4:8], dtype=np.float64)
            quat = quat / np.linalg.norm(quat)
            leveled_position = rotation @ position
            leveled_quat = quaternion_multiply(rotation_quat, quat)
            leveled_quat = leveled_quat / np.linalg.norm(leveled_quat)

            values[1:4] = leveled_position.tolist()
            values[4:8] = leveled_quat.tolist()
            dst.write(" ".join(f"{value:.9f}" for value in values) + "\n")
            changed += 1
    return changed


def write_info(path, normal, centroid, rotation, quat, before_tilt, after_tilt, pcd_out, traj_out):
    rpy = matrix_to_rpy_deg(rotation)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# FAST-LIO map leveling result. Use this as map -> camera_init.\n")
        handle.write(f"ground_normal_camera_init: [{normal[0]:.9f}, {normal[1]:.9f}, {normal[2]:.9f}]\n")
        handle.write(f"ground_centroid_camera_init: [{centroid[0]:.9f}, {centroid[1]:.9f}, {centroid[2]:.9f}]\n")
        handle.write(f"ground_tilt_before_deg: {before_tilt:.6f}\n")
        handle.write(f"ground_tilt_after_deg: {after_tilt:.6f}\n")
        handle.write(f"map_to_camera_init_rpy_deg: [{rpy[0]:.9f}, {rpy[1]:.9f}, {rpy[2]:.9f}]\n")
        handle.write(
            "map_to_camera_init_quat_xyzw: "
            f"[{quat[0]:.9f}, {quat[1]:.9f}, {quat[2]:.9f}, {quat[3]:.9f}]\n"
        )
        handle.write("roslaunch_args: >\n")
        handle.write(
            "  map_to_camera_init_x:=0 map_to_camera_init_y:=0 map_to_camera_init_z:=0 "
            f"map_to_camera_init_qx:={quat[0]:.9f} "
            f"map_to_camera_init_qy:={quat[1]:.9f} "
            f"map_to_camera_init_qz:={quat[2]:.9f} "
            f"map_to_camera_init_qw:={quat[3]:.9f}\n"
        )
        handle.write(f"leveled_pcd: {pcd_out}\n")
        handle.write(f"leveled_trajectory: {traj_out}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    data_root = Path(os.environ.get("NAV3D_DATA_ROOT", "/tmp/nav3d_data"))
    leveled_cloud_dir = data_root / "leveled_cloud"
    parser.add_argument("--pcd-in", default=str(data_root / "point_cloud" / "scans.pcd"))
    parser.add_argument("--traj-in", default=str(data_root / "trace_data" / "mapping_trajectory.txt"))
    parser.add_argument("--pcd-out", default=str(leveled_cloud_dir / "scans_leveled.pcd"))
    parser.add_argument(
        "--traj-out", default=str(data_root / "trace_data" / "mapping_trajectory_leveled.txt")
    )
    parser.add_argument("--info-out", default=str(leveled_cloud_dir / "leveling_result.yaml"))
    parser.add_argument("--sample-size", type=int, default=120000)
    parser.add_argument("--ransac-iterations", type=int, default=500)
    parser.add_argument("--distance-threshold", type=float, default=0.12)
    parser.add_argument("--min-up-dot", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    header, data = load_binary_pcd(args.pcd_in)
    xyz = data[:, :3].astype(np.float64, copy=False)
    normal, centroid = ransac_ground_plane(
        xyz,
        sample_size=args.sample_size,
        iterations=args.ransac_iterations,
        threshold=args.distance_threshold,
        min_up_dot=args.min_up_dot,
        seed=args.seed,
    )
    rotation = rotation_align_vectors(normal, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    rotation_quat = matrix_to_quaternion(rotation)

    before_tilt = plane_tilt_deg(normal)
    data[:, :3] = (rotation @ xyz.T).T.astype(np.float32)
    after_tilt = plane_tilt_deg(rotation @ normal)

    write_binary_pcd(args.pcd_out, header, data)
    trajectory_count = transform_trajectory(args.traj_in, args.traj_out, rotation, rotation_quat)
    write_info(args.info_out, normal, centroid, rotation, rotation_quat, before_tilt, after_tilt, args.pcd_out, args.traj_out)

    print(f"ground tilt before: {before_tilt:.3f} deg")
    print(f"ground tilt after:  {after_tilt:.3f} deg")
    print(f"leveled pcd:        {args.pcd_out}")
    print(f"leveled trajectory: {args.traj_out} ({trajectory_count} poses)")
    print(f"leveling info:      {args.info_out}")
    print(
        "map_to_camera_init quaternion xyzw: "
        f"{rotation_quat[0]:.9f} {rotation_quat[1]:.9f} {rotation_quat[2]:.9f} {rotation_quat[3]:.9f}"
    )


if __name__ == "__main__":
    main()
