#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/.." && pwd)"
profile="${1:-core}"

# shellcheck disable=SC1091
source "${script_dir}/nav3d_env.sh"

if [[ "${NAV3D_INSTALL_DEPS:-0}" == "1" ]]; then
  rosdep install --from-paths "${workspace}/src" --ignore-src -r -y
fi

core_packages=(
  map_process
  planning_3d
  bezier_path_optimizer
  nmpc_planner
  obstacle_processor
  map_publisher
  nav_bringup
  rviz-3d-nav-goal-tool
)

join_packages() {
  local IFS=';'
  echo "$*"
}

cd "${workspace}"
case "${profile}" in
  core)
    catkin_make -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_WHITELIST_PACKAGES="$(join_packages "${core_packages[@]}")"
    ;;
  mid360)
    mid360_packages=(livox_ros_driver2 fast_lio "${core_packages[@]}")
    catkin_make -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_WHITELIST_PACKAGES="$(join_packages "${mid360_packages[@]}")"
    ;;
  all)
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES=""
    ;;
  *)
    echo "Usage: $0 {core|mid360|all}" >&2
    exit 2
    ;;
esac

echo "Build complete. Run: source ${workspace}/devel/setup.bash"
