#!/usr/bin/env bash

# Source this file from the workspace root or any other directory.
_nav3d_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAV3D_WS="${NAV3D_WS:-$(cd "${_nav3d_script_dir}/.." && pwd)}"
export NAV3D_DATA_ROOT="${NAV3D_DATA_ROOT:-${NAV3D_WS}/src/data}"
if [[ -d "${NAV3D_WS}/.local/casadi" ]]; then
  _nav3d_default_casadi_root="${NAV3D_WS}/.local/casadi"
else
  _nav3d_default_casadi_root="/opt/casadi"
fi
export CASADI_ROOT="${CASADI_ROOT:-${_nav3d_default_casadi_root}}"
export CASADI_LIB_PATH="${CASADI_LIB_PATH:-${CASADI_ROOT}/lib}"

_nav3d_ros_distro="${ROS_DISTRO:-noetic}"
_nav3d_ros_setup="/opt/ros/${_nav3d_ros_distro}/setup.bash"
if [[ -r "${_nav3d_ros_setup}" ]]; then
  # shellcheck disable=SC1090
  source "${_nav3d_ros_setup}"
else
  echo "Nav_3d: ROS setup not found: ${_nav3d_ros_setup}" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -r "${NAV3D_WS}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${NAV3D_WS}/devel/setup.bash"
fi

if [[ -d "${CASADI_LIB_PATH}" ]]; then
  export LD_LIBRARY_PATH="${CASADI_LIB_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

unset _nav3d_script_dir _nav3d_ros_distro _nav3d_ros_setup _nav3d_default_casadi_root
