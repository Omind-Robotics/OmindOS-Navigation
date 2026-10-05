#!/usr/bin/env bash
set -euo pipefail

container_name="${NAV3D_CONTAINER:-Nav_3d_ros2}"
display_name="${DISPLAY:-:1}"
xauthority_path="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
unitree_interface="${NAV3D_UNITREE_INTERFACE:-enp86s0}"
unitree_peer="${NAV3D_UNITREE_PEER:-192.168.123.161}"
max_linear_velocity='0.40'
max_angular_velocity='0.40'
confirmation='START_REAL_NAVIGATION'
hardware_launch_pattern='hardware_navigation_known_start.launch.py'
offline_launch_pattern='offline_navigation_sim.launch.py'

if ! docker inspect "${container_name}" >/dev/null 2>&1; then
  echo "Container ${container_name} does not exist." >&2
  exit 1
fi
if [ "$(docker inspect -f '{{.State.Running}}' "${container_name}")" != "true" ]; then
  docker start "${container_name}" >/dev/null
fi

if docker exec "${container_name}" pgrep -f "${hardware_launch_pattern}" >/dev/null; then
  echo "Hardware navigation is already running in ${container_name}."
  exit 0
fi
if docker exec "${container_name}" pgrep -f "${offline_launch_pattern}" >/dev/null; then
  echo "Offline navigation is still running." >&2
  echo "Run ./scripts/stop_offline_navigation_rviz.sh first." >&2
  exit 1
fi

if ! ip link show "${unitree_interface}" >/dev/null 2>&1; then
  echo "Unitree interface ${unitree_interface} does not exist on the host." >&2
  exit 1
fi
if [ "$(cat "/sys/class/net/${unitree_interface}/operstate")" != "up" ]; then
  echo "Unitree interface ${unitree_interface} is not up." >&2
  exit 1
fi
if ! ping -c 1 -W 1 "${unitree_peer}" >/dev/null 2>&1; then
  echo "Go2 peer ${unitree_peer} is not reachable." >&2
  exit 1
fi
if [ ! -r "${xauthority_path}" ]; then
  echo "Cannot read Xauthority file: ${xauthority_path}" >&2
  exit 1
fi

if ! docker exec -e ROS_DOMAIN_ID=142 -e ROS_LOCALHOST_ONLY=0 \
  "${container_name}" bash -lc '
    source /opt/ros/humble/setup.bash
    timeout 5 ros2 topic echo --once /lio/odom >/dev/null
  '
then
  echo "No fresh /lio/odom sample; hardware navigation will not start." >&2
  exit 1
fi

echo "Real navigation preflight passed."
echo "Required: robot at the old map start pose and heading; area clear."
echo "Active speeds: linear=${max_linear_velocity} m/s "\
"angular=${max_angular_velocity} rad/s."
printf "Type %s to arm navigation: " "${confirmation}"
read -r response
if [ "${response}" != "${confirmation}" ]; then
  echo "Confirmation mismatch; nothing was started."
  exit 1
fi

docker cp "${xauthority_path}" \
  "${container_name}:/tmp/nav3d-rviz.Xauthority" >/dev/null
docker exec "${container_name}" bash -lc '
  chmod 600 /tmp/nav3d-rviz.Xauthority
  mkdir -p /tmp/nav3d-rviz-runtime
  chmod 700 /tmp/nav3d-rviz-runtime
'

echo "Starting hardware navigation. Ctrl+C is the emergency terminal stop."
exec docker exec -it \
  -e DISPLAY="${display_name}" \
  -e XAUTHORITY=/tmp/nav3d-rviz.Xauthority \
  -e XDG_RUNTIME_DIR=/tmp/nav3d-rviz-runtime \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e ROS_DOMAIN_ID=142 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/opt/casadi/lib \
  -e NAV3D_UNITREE_INTERFACE="${unitree_interface}" \
  -e NAV3D_UNITREE_PEER="${unitree_peer}" \
  -e NAV3D_MAX_LINEAR_VELOCITY="${max_linear_velocity}" \
  -e NAV3D_MAX_ANGULAR_VELOCITY="${max_angular_velocity}" \
  "${container_name}" bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/Nav_3d/install/setup.bash
    source /workspace/Nav_3d/install_lio/setup.bash
    source /workspace/Nav_3d/install_sim/setup.bash
    exec ros2 launch nav3d_hardware_bringup \
      hardware_navigation_known_start.launch.py \
      start_lio:=false \
      start_controller:=true \
      start_in_auto:=true \
      start_go2_bridge:=true \
      start_rviz:=true \
      unitree_interface:="${NAV3D_UNITREE_INTERFACE}" \
      unitree_peer:="${NAV3D_UNITREE_PEER}" \
      max_linear_velocity:="${NAV3D_MAX_LINEAR_VELOCITY}" \
      max_angular_velocity:="${NAV3D_MAX_ANGULAR_VELOCITY}" \
      min_effective_linear_velocity:="${NAV3D_MAX_LINEAR_VELOCITY}" \
      min_effective_angular_velocity:="${NAV3D_MAX_ANGULAR_VELOCITY}"
  '
