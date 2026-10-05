#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

cd "${workspace}"
colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --packages-select livox_ros_driver2 \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble \
    -DBUILD_TESTING=OFF
