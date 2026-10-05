#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${ART3DNAV_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
M20_ROOT="${M20_EXTERNAL_ROOT:-}"
DATA_ROOT="${NAV3D_DATA_ROOT:-${WORKSPACE_ROOT}/src/data}"

RAW_MAP="${M20_RAW_MAP:-${DATA_ROOT}/point_cloud/scans.pcd}"
TRAJECTORY="${M20_TRAJECTORY:-${DATA_ROOT}/trace_data/mapping_trajectory.txt}"
TRAVERSABLE_MAP="${M20_TRAVERSABLE_MAP:-${DATA_ROOT}/traversable/traversable_areas.pcd}"
MAP_CONFIG="${M20_MAP_CONFIG:-${WORKSPACE_ROOT}/src/3D_NAV/map_process_test/config/default_params.yaml}"
BAG_DIR="${M20_BAG_DIR:-${DATA_ROOT}/bags}"
ROS_IP="${ROS_IP:-10.21.31.50}"
IMAGE_NAME="${IMAGE_NAME:-m20pro-sensor-pipeline:foxy-noetic}"
M20_SSH_PASSWORD="${M20_SSH_PASSWORD:-}"
DRY_RUN="${M20_DRY_RUN:-0}"

SENSOR_SCRIPT="${M20_ROOT}/scripts/m20_sensor_transport.sh"
BRIDGE_SCRIPT="${M20_ROOT}/scripts/run_bridge_container.sh"
STAIR_ADAPTER_SCRIPT="${M20_ROOT}/scripts/start_m20_stair_motion_adapter.sh"
FEEDBACK_CONTAINER="m20_stair_feedback_probe"
DDS_SHM_SIZE="${M20_DDS_SHM_SIZE:-512m}"

bridge_pid=''
adapter_pid=''
bridge_owned=0
adapter_owned=0

die() {
  echo "ERROR: $*" >&2
  exit 1
}

print_command() {
  printf '  +'
  printf ' %q' "$@"
  printf '\n'
}

require_file() {
  [[ -f "$1" ]] || die "Required file not found: $1"
}

require_nonempty_file() {
  [[ -s "$1" ]] || die "Required output is missing or empty: $1"
}

source_ros() {
  require_file /opt/ros/noetic/setup.bash
  require_file "${WORKSPACE_ROOT}/devel/setup.bash"
  set +u
  # ROS setup scripts are not guaranteed to be nounset-safe.
  source /opt/ros/noetic/setup.bash
  source "${WORKSPACE_ROOT}/devel/setup.bash"
  set -u
}

sensor_ready() {
  rosnode list >/dev/null 2>&1 &&
    timeout 5 rostopic echo -n 1 /m20/lidar/front >/dev/null 2>&1
}

cleanup_container() {
  local name=$1
  if docker ps -a --format '{{.Names}}' | grep -Fxq "${name}"; then
    if [[ "${DRY_RUN}" == "1" ]]; then
      print_command docker stop -t 3 "${name}"
      return 0
    fi
    echo "Cleaning stale container: ${name}"
    docker stop -t 3 "${name}" >/dev/null 2>&1 || true
    docker rm -f "${name}" >/dev/null 2>&1 || true
  fi
}

cleanup_control_runtime() {
  command -v docker >/dev/null || die "docker is not installed or not in PATH."
  cleanup_container m20_motion_adapter
  cleanup_container m20_cmd_vel_bridge
  cleanup_container "${FEEDBACK_CONTAINER}"
}

sensor_uses_private_ipc() {
  [[ "$(docker inspect m20_sensor_pipeline_safe --format '{{.HostConfig.IpcMode}}' 2>/dev/null || true)" == "private" ]]
}

ensure_sensor_transport() {
  [[ -n "${M20_ROOT}" ]] || die "Set M20_EXTERNAL_ROOT to the M20 integration repository."
  require_file "${SENSOR_SCRIPT}"

  if [[ "${DRY_RUN}" == "1" ]]; then
    print_command env "M20_SSH_PASSWORD=${M20_SSH_PASSWORD}" "${SENSOR_SCRIPT}" start
    return 0
  fi

  if sensor_ready; then
    if sensor_uses_private_ipc; then
      echo "M20 front lidar transport is already ready."
      return 0
    fi
    echo "Restarting sensor transport with isolated DDS shared memory..."
  fi

  echo "Cleaning incomplete M20 sensor transport state..."
  M20_SSH_PASSWORD="${M20_SSH_PASSWORD}" "${SENSOR_SCRIPT}" stop ||
    die "Could not clean the existing M20 sensor transport."
  echo "Starting M20 sensor transport..."
  M20_SSH_PASSWORD="${M20_SSH_PASSWORD}" "${SENSOR_SCRIPT}" start
  sensor_ready || die "Sensor transport started, but /m20/lidar/front is not readable."
}

run_mapping() {
  source_ros
  cleanup_control_runtime
  ensure_sensor_transport

  if [[ "${DRY_RUN}" == "1" ]]; then
    print_command roslaunch fast_lio mapping_m20.launch \
      start_lidar_fusion:=false lidar_topic:=/m20/lidar/front
    print_command roslaunch map_process_test map_process_custom.launch \
      "point_cloud_file:=${RAW_MAP}" \
      "trajectory_file:=${TRAJECTORY}" \
      "output_file:=${TRAVERSABLE_MAP}" \
      "config_file:=${MAP_CONFIG}" \
      trajectory_format:=1
    return 0
  fi

  mkdir -p "$(dirname "${RAW_MAP}")" "$(dirname "${TRAJECTORY}")" "$(dirname "${TRAVERSABLE_MAP}")"
  require_file "${MAP_CONFIG}"

  local marker mapping_rc=0
  marker="$(mktemp)"
  echo "Mapping is running. Move the robot through the target area, then press Ctrl+C once."

  # Keep the wrapper alive after Ctrl+C so the just-recorded map can be processed.
  trap ':' INT
  set +e
  roslaunch fast_lio mapping_m20.launch \
    start_lidar_fusion:=false \
    lidar_topic:=/m20/lidar/front
  mapping_rc=$?
  set -e
  trap - INT

  if [[ "${mapping_rc}" != "0" && "${mapping_rc}" != "130" ]]; then
    rm -f "${marker}"
    die "FAST-LIO mapping exited with status ${mapping_rc}."
  fi

  require_nonempty_file "${RAW_MAP}"
  require_nonempty_file "${TRAJECTORY}"
  if [[ ! "${RAW_MAP}" -nt "${marker}" || ! "${TRAJECTORY}" -nt "${marker}" ]]; then
    rm -f "${marker}"
    die "The map and trajectory were not both updated by this mapping run."
  fi
  rm -f "${marker}"

  echo "Mapping outputs are ready. Extracting the traversable area..."
  roslaunch map_process_test map_process_custom.launch \
    "point_cloud_file:=${RAW_MAP}" \
    "trajectory_file:=${TRAJECTORY}" \
    "output_file:=${TRAVERSABLE_MAP}" \
    "config_file:=${MAP_CONFIG}" \
    trajectory_format:=1

  require_nonempty_file "${TRAVERSABLE_MAP}"
  echo "Traversable map ready: ${TRAVERSABLE_MAP}"
}

nav_cleanup() {
  local rc=$?
  trap - EXIT
  set +e
  if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -Eq '^(m20_cmd_vel_bridge|m20_motion_adapter)$'; then
    M20_SSH_PASSWORD="${M20_SSH_PASSWORD}" "${SENSOR_SCRIPT}" stop
  else
    echo "M20 base containers are still running; sensor transport was left active." >&2
  fi
  set -e
  return "${rc}"
}

run_navigation() {
  source_ros
  cleanup_control_runtime
  ensure_sensor_transport

  if [[ "${DRY_RUN}" == "1" ]]; then
    print_command roslaunch nav_bringup bringup_m20_nav.launch \
      m20_start_lidar_fusion:=false \
      m20_lidar_topic:=/m20/lidar/front \
      "map_path:=${RAW_MAP}" \
      "pcd_path:=${TRAVERSABLE_MAP}" \
      use_map_registration:=true \
      start_in_auto:=false \
      start_rviz:=true
    return 0
  fi

  require_nonempty_file "${RAW_MAP}"
  require_nonempty_file "${TRAVERSABLE_MAP}"
  trap nav_cleanup EXIT

  roslaunch nav_bringup bringup_m20_nav.launch \
    m20_start_lidar_fusion:=false \
    m20_lidar_topic:=/m20/lidar/front \
    "map_path:=${RAW_MAP}" \
    "pcd_path:=${TRAVERSABLE_MAP}" \
    use_map_registration:=true \
    start_in_auto:=false \
    start_rviz:=true
}

record_cleanup() {
  local rc=$?
  trap - EXIT
  set +e
  if [[ "${record_sensor_owned:-0}" == "1" ]]; then
    M20_SSH_PASSWORD="${M20_SSH_PASSWORD}" "${SENSOR_SCRIPT}" stop
  fi
  set -e
  return "${rc}"
}

run_recording() {
  source_ros

  local record_sensor_owned=0
  sensor_ready || record_sensor_owned=1
  ensure_sensor_transport

  local output="${BAG_DIR}/m20_stair_$(date +%Y%m%d_%H%M%S)"
  if [[ "${DRY_RUN}" == "1" ]]; then
    print_command mkdir -p "${BAG_DIR}"
    print_command rosbag record --lz4 --split --size=4096 --min-space=10G \
      --buffsize=1024 --repeat-latched -O "${output}" \
      /m20/lidar/front /IMU /tf /tf_static /Odometry_loc \
      /planning_3d_PRM_node/planned_path /path_smooth /local_plan /local_path \
      /curr_state /navigation_state /navigation_state_debug /cmd_vel /rosout_agg
    return 0
  fi

  mkdir -p "${BAG_DIR}"
  trap record_cleanup EXIT
  echo "Recording M20 stair data to ${output}*.bag"
  echo "Press Ctrl+C to stop and wait for the rosbag index to finish writing."
  rosbag record --lz4 --split --size=4096 --min-space=10G \
    --buffsize=1024 --repeat-latched -O "${output}" \
    /m20/lidar/front /IMU /tf /tf_static /Odometry_loc \
    /planning_3d_PRM_node/planned_path /path_smooth /local_plan /local_path \
    /curr_state /navigation_state /navigation_state_debug /cmd_vel /rosout_agg
}

read_stair_feedback() {
  docker run --rm --name "${FEEDBACK_CONTAINER}" --net=host \
    --ipc=private --shm-size="${DDS_SHM_SIZE}" \
    -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    "${IMAGE_NAME}" bash -lc \
    'source /opt/ros/foxy/setup.bash; source /opt/m20_real_ws/install/setup.bash; timeout 4 ros2 topic echo --qos-reliability reliable /MOTION_INFO' \
    2>&1 || true
}

verify_stair_feedback() {
  local feedback state gait
  feedback="$(read_stair_feedback)"
  printf '%s\n' "${feedback}"

  state="$(sed -nE 's/.*state[=:][[:space:]]*(-?[0-9]+).*/\1/p' <<<"${feedback}" | tail -n 1)"
  gait="$(sed -nE 's/.*gait[=:][[:space:]]*(-?[0-9]+).*/\1/p' <<<"${feedback}" | tail -n 1)"
  [[ "${state}" == "17" ]] ||
    die "Latest M20 state is '${state:-unknown}', expected navigation-ready state 17."
  [[ "${gait}" == "4099" ]] ||
    die "Latest M20 gait is '${gait:-unknown}', expected standard stair gait 4099."
}

base_cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  set +e
  if [[ "${adapter_owned:-0}" == "1" ]]; then
    docker stop -t 3 m20_motion_adapter >/dev/null 2>&1
  fi
  if [[ "${bridge_owned:-0}" == "1" ]]; then
    docker stop -t 3 m20_cmd_vel_bridge >/dev/null 2>&1
  fi
  [[ -z "${adapter_pid:-}" ]] || wait "${adapter_pid}" 2>/dev/null
  [[ -z "${bridge_pid:-}" ]] || wait "${bridge_pid}" 2>/dev/null
  set -e
  return "${rc}"
}

wait_for_bridge() {
  for _ in {1..100}; do
    kill -0 "${bridge_pid}" 2>/dev/null || die "The ROS1/ROS2 cmd_vel bridge exited during startup."
    if rostopic info /cmd_vel 2>/dev/null | grep -Fq '/m20_cmd/ros_bridge'; then
      return 0
    fi
    sleep 0.1
  done
  die "Timed out waiting for /m20_cmd/ros_bridge to subscribe to /cmd_vel."
}

wait_for_stair_adapter() {
  for _ in {1..100}; do
    kill -0 "${adapter_pid}" 2>/dev/null || die "The M20 stair motion adapter exited during startup."
    if docker logs m20_motion_adapter 2>&1 | grep -Fq 'M20 feedback connected: state=17, gait=4099'; then
      return 0
    fi
    sleep 0.1
  done
  die "Timed out waiting for the stair adapter to confirm state=17, gait=4099."
}

run_base() {
  source_ros
  require_file "${BRIDGE_SCRIPT}"
  require_file "${STAIR_ADAPTER_SCRIPT}"

  if [[ "${DRY_RUN}" == "1" ]]; then
    print_command docker run --rm --name "${FEEDBACK_CONTAINER}" --net=host \
      --ipc=private "--shm-size=${DDS_SHM_SIZE}" "${IMAGE_NAME}" bash -lc \
      'source /opt/ros/foxy/setup.bash; source /opt/m20_real_ws/install/setup.bash; timeout 4 ros2 topic echo --qos-reliability reliable /MOTION_INFO'
    print_command rosparam set /m20_cmd_vel_bridge_topics \
      '[{topic: /cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10}]'
    print_command env "ROS_IP=${ROS_IP}" "IMAGE_NAME=${IMAGE_NAME}" \
      CONTAINER_NAME=m20_cmd_vel_bridge "${BRIDGE_SCRIPT}" bash -lc \
      'source /opt/ros1_bridge_ws/install/setup.bash && exec ros2 run ros1_bridge parameter_bridge /m20_cmd_vel_bridge_topics /m20_cmd_vel_bridge_services_1_to_2 /m20_cmd_vel_bridge_services_2_to_1 __ns:=/m20_cmd'
    print_command env "ROS_IP=${ROS_IP}" "IMAGE_NAME=${IMAGE_NAME}" \
      CONTAINER_NAME=m20_motion_adapter "${STAIR_ADAPTER_SCRIPT}"
    return 0
  fi

  command -v docker >/dev/null || die "docker is not installed or not in PATH."
  rosnode list >/dev/null 2>&1 || die "ROS1 master is not reachable. Start the nav command first."
  cleanup_control_runtime
  sensor_ready || die "M20 sensor transport is not ready. Start the nav command first."

  verify_stair_feedback

  rosparam set /m20_cmd_vel_bridge_topics \
    '[{topic: /cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10}]'
  rosparam set /m20_cmd_vel_bridge_services_1_to_2 '[]'
  rosparam set /m20_cmd_vel_bridge_services_2_to_1 '[]'

  bridge_pid=''
  adapter_pid=''
  bridge_owned=0
  adapter_owned=0
  trap base_cleanup EXIT
  trap 'exit 130' INT TERM

  ROS_IP="${ROS_IP}" IMAGE_NAME="${IMAGE_NAME}" CONTAINER_NAME=m20_cmd_vel_bridge \
    IPC_MODE=private SHM_SIZE="${DDS_SHM_SIZE}" \
    "${BRIDGE_SCRIPT}" bash -lc \
    'source /opt/ros1_bridge_ws/install/setup.bash && exec ros2 run ros1_bridge parameter_bridge /m20_cmd_vel_bridge_topics /m20_cmd_vel_bridge_services_1_to_2 /m20_cmd_vel_bridge_services_2_to_1 __ns:=/m20_cmd' &
  bridge_pid=$!
  bridge_owned=1
  wait_for_bridge

  ROS_IP="${ROS_IP}" IMAGE_NAME="${IMAGE_NAME}" CONTAINER_NAME=m20_motion_adapter \
    IPC_MODE=private SHM_SIZE="${DDS_SHM_SIZE}" \
    "${STAIR_ADAPTER_SCRIPT}" &
  adapter_pid=$!
  adapter_owned=1
  wait_for_stair_adapter

  echo "M20 stair base is ready: /cmd_vel -> bridge -> gait 4099 adapter."
  echo "Keep this terminal open. Press Ctrl+C here before stopping navigation."
  wait "${adapter_pid}"
}

usage() {
  cat <<'EOF'
Usage: m20_stair_workflow.sh {map|nav|base|record}

  map   Start front-lidar mapping, then extract the traversable map after Ctrl+C.
  nav   Start M20 localization, global planning, Bezier smoothing, NMPC, and RViz.
  base  Verify state 17 / gait 4099, then start the cmd_vel bridge and stair adapter.
  record  Start/reuse front-lidar transport and record stair-navigation data to rosbag.
EOF
}

case "${1:-}" in
  map) run_mapping ;;
  nav) run_navigation ;;
  base) run_base ;;
  record) run_recording ;;
  -h|--help|help) usage ;;
  *) usage >&2; exit 2 ;;
esac
