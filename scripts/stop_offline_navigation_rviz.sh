#!/usr/bin/env bash
set -euo pipefail

container_name="${NAV3D_CONTAINER:-Nav_3d_ros2}"
launch_pattern='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch nav3d_hardware_bringup offline_navigation_sim.launch.py'

if ! docker inspect "${container_name}" >/dev/null 2>&1; then
  echo "Container ${container_name} does not exist." >&2
  exit 1
fi

launch_pid="$(docker exec "${container_name}" \
  pgrep -f "${launch_pattern}" | head -n 1 || true)"
if [ -z "${launch_pid}" ]; then
  echo "Offline navigation RViz is not running."
  exit 0
fi

docker exec "${container_name}" kill -INT "${launch_pid}"
echo "Stopped offline navigation RViz in ${container_name}."
