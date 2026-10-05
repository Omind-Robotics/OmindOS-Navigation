#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
set +u
source "${workspace}/install/setup.bash"
set -u
exec ros2 launch nav3d_native demo.launch.py "$@"
