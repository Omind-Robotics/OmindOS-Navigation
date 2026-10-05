#!/usr/bin/env bash
set -euo pipefail

container_name="${NAV3D_CONTAINER:-Nav_3d_ros2}"
display_name="${DISPLAY:-:1}"
xauthority_path="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
motion_mode="${NAV3D_MOTION_MODE:-path}"

if ! docker inspect "${container_name}" >/dev/null 2>&1; then
  echo "Container ${container_name} does not exist." >&2
  exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' "${container_name}")" != "true" ]; then
  docker start "${container_name}" >/dev/null
fi

launch_pattern='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch nav3d_hardware_bringup offline_navigation_sim.launch.py'
if docker exec "${container_name}" pgrep -f "${launch_pattern}" >/dev/null; then
  echo "Offline navigation RViz is already running in ${container_name}."
  echo "Use ./scripts/stop_offline_navigation_rviz.sh before restarting it."
  exit 0
fi

if [ ! -r "${xauthority_path}" ]; then
  echo "Cannot read Xauthority file: ${xauthority_path}" >&2
  exit 1
fi

docker cp "${xauthority_path}" \
  "${container_name}:/tmp/nav3d-rviz.Xauthority" >/dev/null
docker exec "${container_name}" \
  bash -lc 'chmod 600 /tmp/nav3d-rviz.Xauthority && \
    mkdir -p /tmp/nav3d-rviz-runtime && \
    chmod 700 /tmp/nav3d-rviz-runtime'

exec docker exec -it \
  -e DISPLAY="${display_name}" \
  -e XAUTHORITY=/tmp/nav3d-rviz.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XDG_RUNTIME_DIR=/tmp/nav3d-rviz-runtime \
  -e ROS_DOMAIN_ID=179 \
  -e ROS_LOCALHOST_ONLY=1 \
  -e NAV3D_MOTION_MODE="${motion_mode}" \
  -e LD_LIBRARY_PATH=/opt/casadi/lib \
  "${container_name}" bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/Nav_3d/install/setup.bash
    source /workspace/Nav_3d/install_lio/setup.bash
    source /workspace/Nav_3d/install_sim/setup.bash
    exec ros2 launch nav3d_hardware_bringup \
      offline_navigation_sim.launch.py \
      start_rviz:=true motion_mode:="${NAV3D_MOTION_MODE}"
  '
