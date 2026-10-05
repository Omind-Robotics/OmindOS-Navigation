#!/usr/bin/env python3
"""
Utility tools for map_process experiments.

This file intentionally does not modify the existing ROS nodes.  It provides:

1. level
   Estimate the dominant floor plane from a PCD map, rotate the map so that the
   plane normal aligns with +Z, and apply the same transform to the trajectory.

2. obstacle-filter
   Remove traversable points that have nearby points in the original/leveled map
   above them.  This compensates for cases where the ground extractor outputs a
   floor patch under/near low obstacles.

3. add-start-plane
   Append a manually approved horizontal patch around a known navigation start
   pose while preserving the original traversable PCD.

Supported PCD DATA encodings: binary and ascii.  The current project data is
binary PCD, which is the primary path.
"""

from __future__ import annotations

import argparse
import io
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np


PCD_TYPE_MAP = {
    ("F", 4): np.float32,
    ("F", 8): np.float64,
    ("I", 1): np.int8,
    ("I", 2): np.int16,
    ("I", 4): np.int32,
    ("I", 8): np.int64,
    ("U", 1): np.uint8,
    ("U", 2): np.uint16,
    ("U", 4): np.uint32,
    ("U", 8): np.uint64,
}


def _parse_pcd_header(path: Path) -> Tuple[List[str], Dict[str, List[str]], int]:
    header: List[str] = []
    meta: Dict[str, List[str]] = {}

    with path.open("rb") as fh:
        while True:
            line_bytes = fh.readline()
            if not line_bytes:
                raise ValueError(f"{path}: missing DATA line in PCD header")
            line = line_bytes.decode("utf-8", errors="replace").rstrip("\n")
            header.append(line)
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                parts = stripped.split()
                meta[parts[0].upper()] = parts[1:]
                if parts[0].upper() == "DATA":
                    return header, meta, fh.tell()


def _pcd_dtype(meta: Dict[str, List[str]]) -> np.dtype:
    fields = meta["FIELDS"]
    sizes = [int(x) for x in meta["SIZE"]]
    types = meta["TYPE"]
    counts = [int(x) for x in meta.get("COUNT", ["1"] * len(fields))]

    dtype_fields = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        np_type = PCD_TYPE_MAP.get((typ.upper(), size))
        if np_type is None:
            raise ValueError(f"Unsupported PCD field type: {name} TYPE={typ} SIZE={size}")
        if count == 1:
            dtype_fields.append((name, np_type))
        else:
            dtype_fields.append((name, np_type, (count,)))
    return np.dtype(dtype_fields)


def load_pcd(path: Path) -> Tuple[List[str], Dict[str, List[str]], np.ndarray]:
    header, meta, offset = _parse_pcd_header(path)
    dtype = _pcd_dtype(meta)
    data_kind = meta["DATA"][0].lower()
    points = int(meta["POINTS"][0])

    if data_kind == "binary":
        raw = path.read_bytes()[offset:]
        cloud = np.frombuffer(raw, dtype=dtype, count=points).copy()
    elif data_kind == "ascii":
        raw = path.read_bytes()[offset:]
        arr = np.loadtxt(io.BytesIO(raw), dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        fields = meta["FIELDS"]
        if arr.shape[1] != len(fields):
            raise ValueError(f"{path}: ascii column count mismatch")
        cloud = np.empty(arr.shape[0], dtype=dtype)
        for i, name in enumerate(fields):
            cloud[name] = arr[:, i].astype(cloud.dtype[name])
    else:
        raise ValueError(f"{path}: unsupported PCD DATA encoding: {data_kind}")

    for required in ("x", "y", "z"):
        if required not in cloud.dtype.names:
            raise ValueError(f"{path}: missing required PCD field '{required}'")
    return header, meta, cloud


def _updated_header(header: List[str], meta: Dict[str, List[str]], npoints: int) -> bytes:
    out: List[str] = []
    for line in header:
        key = line.strip().split(maxsplit=1)[0].upper() if line.strip() else ""
        if key == "WIDTH":
            out.append(f"WIDTH {npoints}")
        elif key == "HEIGHT":
            out.append("HEIGHT 1")
        elif key == "POINTS":
            out.append(f"POINTS {npoints}")
        else:
            out.append(line)
    return ("\n".join(out) + "\n").encode("utf-8")


def write_pcd(path: Path, header: List[str], meta: Dict[str, List[str]], cloud: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data_kind = meta["DATA"][0].lower()
    with path.open("wb") as fh:
        fh.write(_updated_header(header, meta, len(cloud)))
        if data_kind == "binary":
            cloud.tofile(fh)
        elif data_kind == "ascii":
            fields = meta["FIELDS"]
            arr = np.column_stack([cloud[name] for name in fields])
            np.savetxt(fh, arr, fmt="%.9g")
        else:
            raise ValueError(f"Unsupported PCD DATA encoding: {data_kind}")


def xyz_view(cloud: np.ndarray) -> np.ndarray:
    return np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64, copy=False)


def transform_xyz_in_place(cloud: np.ndarray, rotation: np.ndarray, pivot: np.ndarray) -> None:
    pts = xyz_view(cloud)
    transformed = (pts - pivot) @ rotation.T + pivot
    cloud["x"] = transformed[:, 0].astype(cloud["x"].dtype, copy=False)
    cloud["y"] = transformed[:, 1].astype(cloud["y"].dtype, copy=False)
    cloud["z"] = transformed[:, 2].astype(cloud["z"].dtype, copy=False)

    names = cloud.dtype.names or ()
    if {"normal_x", "normal_y", "normal_z"}.issubset(names):
        normals = np.column_stack((cloud["normal_x"], cloud["normal_y"], cloud["normal_z"])).astype(
            np.float64, copy=False
        )
        rotated_normals = normals @ rotation.T
        cloud["normal_x"] = rotated_normals[:, 0].astype(cloud["normal_x"].dtype, copy=False)
        cloud["normal_y"] = rotated_normals[:, 1].astype(cloud["normal_y"].dtype, copy=False)
        cloud["normal_z"] = rotated_normals[:, 2].astype(cloud["normal_z"].dtype, copy=False)


def rotation_from_vectors(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    src = src / np.linalg.norm(src)
    dst = dst / np.linalg.norm(dst)
    cross = np.cross(src, dst)
    dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))
    norm_cross = np.linalg.norm(cross)
    if norm_cross < 1e-12:
        if dot > 0.0:
            return np.eye(3)
        axis = np.array([1.0, 0.0, 0.0])
        if abs(src[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = axis - src * np.dot(axis, src)
        axis /= np.linalg.norm(axis)
        return Rotation.from_rotvec(axis * math.pi).as_matrix()
    skew = np.array(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ]
    )
    return np.eye(3) + skew + skew @ skew * ((1.0 - dot) / (norm_cross**2))


def lower_surface_candidates(
    points: np.ndarray,
    grid_size: float,
    percentile: float,
    min_bin_points: int,
) -> np.ndarray:
    xy = points[:, :2]
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    xy = xy[finite]
    min_xy = xy.min(axis=0)
    ij = np.floor((xy - min_xy) / grid_size).astype(np.int64)
    keys = ij[:, 0] * 4_294_967_291 + ij[:, 1]
    order = np.argsort(keys)
    keys_sorted = keys[order]
    change = np.r_[True, keys_sorted[1:] != keys_sorted[:-1], True]
    bounds = np.flatnonzero(change)

    selected: List[int] = []
    for start, end in zip(bounds[:-1], bounds[1:]):
        if end - start < min_bin_points:
            continue
        idxs = order[start:end]
        z_values = points[idxs, 2]
        z_ref = np.percentile(z_values, percentile)
        selected.append(int(idxs[np.argmin(np.abs(z_values - z_ref))]))

    if not selected:
        raise ValueError("No lower-surface candidates found; reduce --min-bin-points or increase sample size")
    return points[np.asarray(selected, dtype=np.int64)]


def fit_plane_ransac(
    points: np.ndarray,
    iterations: int,
    threshold: float,
    rng: np.random.Generator,
) -> Tuple[np.ndarray, np.ndarray, int]:
    if len(points) < 3:
        raise ValueError("Need at least 3 points to fit a plane")

    best_inliers: np.ndarray | None = None
    best_count = -1

    for _ in range(iterations):
        ids = rng.choice(len(points), size=3, replace=False)
        p0, p1, p2 = points[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        distances = np.abs((points - p0) @ normal)
        inliers = distances < threshold
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 3:
        raise ValueError("RANSAC failed to find a valid plane")

    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
    return normal, centroid, best_count


def transform_trajectory_format1(parts: List[str], rotation: np.ndarray, pivot: np.ndarray) -> List[str]:
    from scipy.spatial.transform import Rotation

    pos = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
    new_pos = rotation @ (pos - pivot) + pivot
    quat = np.array([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])
    new_quat = (Rotation.from_matrix(rotation) * Rotation.from_quat(quat)).as_quat()
    return [
        parts[0],
        f"{new_pos[0]:.9f}",
        f"{new_pos[1]:.9f}",
        f"{new_pos[2]:.9f}",
        f"{new_quat[0]:.9f}",
        f"{new_quat[1]:.9f}",
        f"{new_quat[2]:.9f}",
        f"{new_quat[3]:.9f}",
        *parts[8:],
    ]


def transform_trajectory_line(line: str, fmt: int, rotation: np.ndarray, pivot: np.ndarray) -> str:
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return line.rstrip("\n")
    parts = stripped.split()
    try:
        if fmt == 1:
            if len(parts) < 8:
                return line.rstrip("\n")
            return " ".join(transform_trajectory_format1(parts, rotation, pivot))
        if fmt == 2:
            if len(parts) < 3:
                return line.rstrip("\n")
            idxs = (0, 1, 2)
        elif fmt == 3:
            if len(parts) < 7:
                return line.rstrip("\n")
            idxs = (4, 5, 6)
        else:
            raise ValueError(f"Unsupported trajectory format: {fmt}")

        pos = np.array([float(parts[idxs[0]]), float(parts[idxs[1]]), float(parts[idxs[2]])])
        new_pos = rotation @ (pos - pivot) + pivot
        for idx, value in zip(idxs, new_pos):
            parts[idx] = f"{value:.9f}"
        return " ".join(parts)
    except ValueError:
        return line.rstrip("\n")


def transform_trajectory(
    input_path: Path,
    output_path: Path,
    fmt: int,
    rotation: np.ndarray,
    pivot: np.ndarray,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with input_path.open("r", encoding="utf-8", errors="replace") as src, output_path.open(
        "w", encoding="utf-8"
    ) as dst:
        for line in src:
            dst.write(transform_trajectory_line(line, fmt, rotation, pivot) + "\n")


def command_level(args: argparse.Namespace) -> None:
    header, meta, cloud = load_pcd(args.input_pcd)
    points = xyz_view(cloud)

    rng = np.random.default_rng(args.seed)
    if len(points) > args.max_sample_points:
        sample_idx = rng.choice(len(points), size=args.max_sample_points, replace=False)
        sample = points[sample_idx]
    else:
        sample = points

    candidates = lower_surface_candidates(
        sample,
        grid_size=args.grid_size,
        percentile=args.ground_percentile,
        min_bin_points=args.min_bin_points,
    )
    normal, pivot, inlier_count = fit_plane_ransac(
        candidates,
        iterations=args.ransac_iterations,
        threshold=args.ransac_threshold,
        rng=rng,
    )
    tilt_deg = math.degrees(math.acos(float(np.clip(normal @ np.array([0.0, 0.0, 1.0]), -1.0, 1.0))))
    if tilt_deg > args.max_correction_deg:
        raise RuntimeError(
            f"Estimated tilt is {tilt_deg:.2f} deg, above --max-correction-deg={args.max_correction_deg}. "
            "This usually means the plane fit locked onto a wall or ramp. Increase sample quality or limit input."
        )

    rotation = rotation_from_vectors(normal, np.array([0.0, 0.0, 1.0]))
    transform_xyz_in_place(cloud, rotation, pivot)
    write_pcd(args.output_pcd, header, meta, cloud)
    transform_trajectory(args.input_trajectory, args.output_trajectory, args.trajectory_format, rotation, pivot)

    print(f"Input PCD:        {args.input_pcd}")
    print(f"Output PCD:       {args.output_pcd}")
    print(f"Input trajectory: {args.input_trajectory}")
    print(f"Output trajectory:{args.output_trajectory}")
    print(f"Estimated ground normal: [{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}]")
    print(f"Correction tilt: {tilt_deg:.3f} deg")
    print(f"Plane support: {inlier_count}/{len(candidates)} lower-surface candidates")
    print(f"Rotation matrix:\n{rotation}")
    print(f"Pivot: [{pivot[0]:.6f}, {pivot[1]:.6f}, {pivot[2]:.6f}]")


def command_obstacle_filter(args: argparse.Namespace) -> None:
    from scipy.spatial import cKDTree

    ref_header, ref_meta, ref_cloud = load_pcd(args.reference_pcd)
    trav_header, trav_meta, trav_cloud = load_pcd(args.input_traversable_pcd)
    del ref_header, ref_meta

    ref_xyz = xyz_view(ref_cloud)
    trav_xyz = xyz_view(trav_cloud)
    finite_ref = np.isfinite(ref_xyz).all(axis=1)
    ref_xyz = ref_xyz[finite_ref]

    tree = cKDTree(ref_xyz[:, :2])
    keep = np.ones(len(trav_xyz), dtype=bool)

    for start in range(0, len(trav_xyz), args.chunk_size):
        end = min(start + args.chunk_size, len(trav_xyz))
        neighbors = tree.query_ball_point(trav_xyz[start:end, :2], r=args.obstacle_radius)
        for local_i, ids in enumerate(neighbors):
            if not ids:
                continue
            global_i = start + local_i
            dz = ref_xyz[np.asarray(ids, dtype=np.int64), 2] - trav_xyz[global_i, 2]
            if np.any((dz >= args.min_obstacle_height) & (dz <= args.max_obstacle_height)):
                keep[global_i] = False

    filtered = trav_cloud[keep].copy()
    write_pcd(args.output_pcd, trav_header, trav_meta, filtered)
    removed = len(trav_cloud) - len(filtered)
    pct = 100.0 * removed / max(1, len(trav_cloud))
    print(f"Reference PCD:    {args.reference_pcd}")
    print(f"Input traversable:{args.input_traversable_pcd}")
    print(f"Output PCD:       {args.output_pcd}")
    print(f"Removed {removed}/{len(trav_cloud)} traversable points ({pct:.2f}%)")
    print(
        "Obstacle rule: remove traversable point if reference map has a nearby point "
        f"within {args.obstacle_radius:.2f} m XY and "
        f"{args.min_obstacle_height:.2f}..{args.max_obstacle_height:.2f} m above it"
    )


def command_add_start_plane(args: argparse.Namespace) -> None:
    header, meta, cloud = load_pcd(args.input_pcd)
    if args.spacing <= 0.0 or args.size_x <= 0.0 or args.size_y <= 0.0:
        raise ValueError("spacing and plane sizes must be positive")

    x_values = np.arange(
        args.center_x - args.size_x / 2.0,
        args.center_x + args.size_x / 2.0 + args.spacing * 0.5,
        args.spacing,
    )
    y_values = np.arange(
        args.center_y - args.size_y / 2.0,
        args.center_y + args.size_y / 2.0 + args.spacing * 0.5,
        args.spacing,
    )
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="xy")
    patch = np.zeros(grid_x.size, dtype=cloud.dtype)
    patch["x"] = grid_x.ravel().astype(patch.dtype["x"], copy=False)
    patch["y"] = grid_y.ravel().astype(patch.dtype["y"], copy=False)
    patch["z"] = np.asarray(args.z, dtype=patch.dtype["z"])
    if "scalar_intensity" in (patch.dtype.names or ()):
        patch["scalar_intensity"] = np.asarray(
            args.intensity, dtype=patch.dtype["scalar_intensity"])
    elif "intensity" in (patch.dtype.names or ()):
        patch["intensity"] = np.asarray(args.intensity, dtype=patch.dtype["intensity"])

    augmented = np.concatenate((cloud, patch))
    write_pcd(args.output_pcd, header, meta, augmented)
    print(f"Input PCD:        {args.input_pcd}")
    print(f"Output PCD:       {args.output_pcd}")
    print(f"Original points:  {len(cloud)}")
    print(f"Added points:     {len(patch)}")
    print(f"Output points:    {len(augmented)}")
    print(
        "Start plane:      "
        f"x=[{x_values[0]:.3f}, {x_values[-1]:.3f}], "
        f"y=[{y_values[0]:.3f}, {y_values[-1]:.3f}], "
        f"z={args.z:.3f}, spacing={args.spacing:.3f}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    level = subparsers.add_parser("level", help="Level a PCD map and matching trajectory")
    level.add_argument("--input-pcd", type=Path, required=True)
    level.add_argument("--input-trajectory", type=Path, required=True)
    level.add_argument("--output-pcd", type=Path, required=True)
    level.add_argument("--output-trajectory", type=Path, required=True)
    level.add_argument("--trajectory-format", type=int, default=1, choices=(1, 2, 3))
    level.add_argument("--max-sample-points", type=int, default=350_000)
    level.add_argument("--grid-size", type=float, default=0.45)
    level.add_argument("--ground-percentile", type=float, default=18.0)
    level.add_argument("--min-bin-points", type=int, default=5)
    level.add_argument("--ransac-iterations", type=int, default=450)
    level.add_argument("--ransac-threshold", type=float, default=0.08)
    level.add_argument("--max-correction-deg", type=float, default=20.0)
    level.add_argument("--seed", type=int, default=7)
    level.set_defaults(func=command_level)

    obstacle = subparsers.add_parser("obstacle-filter", help="Remove traversable points near raised obstacles")
    obstacle.add_argument("--reference-pcd", type=Path, required=True)
    obstacle.add_argument("--input-traversable-pcd", type=Path, required=True)
    obstacle.add_argument("--output-pcd", type=Path, required=True)
    obstacle.add_argument("--obstacle-radius", type=float, default=0.35)
    obstacle.add_argument("--min-obstacle-height", type=float, default=0.12)
    obstacle.add_argument("--max-obstacle-height", type=float, default=1.40)
    obstacle.add_argument("--chunk-size", type=int, default=1500)
    obstacle.set_defaults(func=command_obstacle_filter)

    start_plane = subparsers.add_parser(
        "add-start-plane", help="Append an approved horizontal start patch")
    start_plane.add_argument("--input-pcd", type=Path, required=True)
    start_plane.add_argument("--output-pcd", type=Path, required=True)
    start_plane.add_argument("--center-x", type=float, default=0.2)
    start_plane.add_argument("--center-y", type=float, default=0.3)
    start_plane.add_argument("--z", type=float, default=-0.25)
    start_plane.add_argument("--size-x", type=float, default=2.4)
    start_plane.add_argument("--size-y", type=float, default=2.2)
    start_plane.add_argument("--spacing", type=float, default=0.05)
    start_plane.add_argument("--intensity", type=float, default=1.0)
    start_plane.set_defaults(func=command_add_start_plane)

    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.func(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
