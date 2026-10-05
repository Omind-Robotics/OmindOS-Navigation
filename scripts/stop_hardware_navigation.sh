#!/usr/bin/env bash
set -euo pipefail

container_name="${NAV3D_CONTAINER:-Nav_3d_ros2}"
launch_pattern='hardware_navigation_known_start.launch.py'

if ! docker inspect "${container_name}" >/dev/null 2>&1; then
  echo "Container ${container_name} does not exist." >&2
  exit 1
fi

launch_pid="$(docker exec "${container_name}" \
  pgrep -f "${launch_pattern}" | head -n 1 || true)"
if [ -z "${launch_pid}" ]; then
  echo "Hardware navigation is not running."
  exit 0
fi

docker exec "${container_name}" kill -INT "${launch_pid}"
echo "Stop requested; the Go2 bridge is sending its shutdown zero sequence."

for _ in 1 2 3 4 5; do
  if ! docker exec "${container_name}" \
    kill -0 "${launch_pid}" >/dev/null 2>&1
  then
    exit 0
  fi
  sleep 1
done

echo "Navigation did not exit after 5 seconds; terminating the launch group."
docker exec "${container_name}" kill -TERM "-${launch_pid}" 2>/dev/null || \
  docker exec "${container_name}" kill -TERM "${launch_pid}" 2>/dev/null || true
