#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
cd "${workspace}"

# Keep the baseline native package and the vendor driver in the established
# install tree. Livox's CMake logic requires DISTRO_ROS to select the Humble
# rosidl target path.
./scripts/build_workspace.sh
colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release

set +u
source "${workspace}/install/setup.bash"
set -u

# Isolate the imported estimator build products from the baseline build.
colcon --log-base log_lio build \
  --build-base build_lio \
  --install-base install_lio \
  --symlink-install \
  --packages-select \
    fast_lio \
    planning_3d_ros2 \
    nmpc_planner_ros2 \
    rviz_3d_nav_goal_tool_ros2 \
    nav3d_hardware_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
