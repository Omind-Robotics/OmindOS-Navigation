#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f /opt/nav3d_ws/install/setup.bash ]]; then
  source /opt/nav3d_ws/install/setup.bash
fi
exec "$@"
