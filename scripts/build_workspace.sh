#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
cd "${workspace}"
colcon build --symlink-install --event-handlers console_direct+ --packages-select nav3d_native
