#!/usr/bin/env bash
set -euo pipefail

DURATION="${1:-120}"
OUTPUT_DIR="${2:-${NAV3D_DATA_ROOT:-/tmp/nav3d_data}/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${OUTPUT_DIR}/m20_static_${DURATION}s_${STAMP}"

if ! [[ "${DURATION}" =~ ^[0-9]+$ ]] || (( DURATION < 10 )); then
  echo "Duration must be an integer of at least 10 seconds." >&2
  exit 2
fi

mkdir -p "${OUTPUT_DIR}"

# Two point clouds consume about 42 MiB/s before compression. Require 25% headroom.
REQUIRED_BYTES=$((DURATION * 42 * 1024 * 1024 * 5 / 4))
AVAILABLE_BYTES="$(df --output=avail -B1 "${OUTPUT_DIR}" | tail -n 1 | tr -d ' ')"
if (( AVAILABLE_BYTES < REQUIRED_BYTES )); then
  echo "Insufficient disk space: need about $((REQUIRED_BYTES / 1024 / 1024)) MiB." >&2
  exit 3
fi

source /opt/ros/noetic/setup.bash

for topic in /m20/lidar/front /m20/lidar/rear /IMU; do
  if ! rostopic info "${topic}" >/dev/null 2>&1; then
    echo "Required topic is unavailable: ${topic}" >&2
    exit 4
  fi
done

echo "Keep M20 completely stationary for ${DURATION} seconds."
echo "Recording to ${OUTPUT}.bag"
exec rosbag record --duration="${DURATION}" --lz4 -O "${OUTPUT}" \
  /m20/lidar/front \
  /m20/lidar/rear \
  /IMU
