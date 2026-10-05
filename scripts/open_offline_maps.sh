#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

RAW_MAP="${REPO_ROOT}/src/data/point_cloud/scans.pcd"
TRAVERSABLE_MAP="${REPO_ROOT}/src/data/traversable_cloud/traversable_areas.pcd"
DEBUG_MODE=0

CC_BIN="${CC_BIN:-$(command -v CloudCompare || command -v cloudcompare || true)}"
if [[ -z "${CC_BIN}" ]]; then
  echo "CloudCompare is not installed." >&2
  exit 1
fi

MAP_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)
      DEBUG_MODE=1
      shift
      ;;
    *)
      MAP_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#MAP_ARGS[@]} -eq 0 ]]; then
  MAP_ARGS=()
  [[ -f "${RAW_MAP}" ]] && MAP_ARGS+=("${RAW_MAP}")
  [[ -f "${TRAVERSABLE_MAP}" ]] && MAP_ARGS+=("${TRAVERSABLE_MAP}")
fi

if [[ ${#MAP_ARGS[@]} -eq 0 ]]; then
  echo "No map files found to open." >&2
  exit 1
fi

USER_ID="$(id -u)"
DEFAULT_RUNTIME_DIR="/run/user/${USER_ID}"

discover_gui_env() {
  local shell_pid
  shell_pid="$(pgrep -u "${USER_ID}" -n gnome-shell || true)"
  if [[ -z "${shell_pid}" || ! -r "/proc/${shell_pid}/environ" ]]; then
    return 0
  fi

  while IFS='=' read -r key value; do
    case "${key}" in
      DISPLAY)
        [[ -z "${DISPLAY:-}" ]] && export DISPLAY="${value}"
        ;;
      XAUTHORITY)
        [[ -z "${XAUTHORITY:-}" ]] && export XAUTHORITY="${value}"
        ;;
      XDG_RUNTIME_DIR)
        [[ -z "${XDG_RUNTIME_DIR:-}" ]] && export XDG_RUNTIME_DIR="${value}"
        ;;
    esac
  done < <(tr '\0' '\n' < "/proc/${shell_pid}/environ")
}

discover_gui_env

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-${DEFAULT_RUNTIME_DIR}}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=${XDG_RUNTIME_DIR}/bus}"

if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -r "${DEFAULT_RUNTIME_DIR}/gdm/Xauthority" ]]; then
    export XAUTHORITY="${DEFAULT_RUNTIME_DIR}/gdm/Xauthority"
  elif [[ -r "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
fi

for map_file in "${MAP_ARGS[@]}"; do
  if [[ ! -f "${map_file}" ]]; then
    echo "Map file not found: ${map_file}" >&2
    exit 1
  fi
done

if [[ "${DEBUG_MODE}" -eq 1 ]]; then
  echo "Launching CloudCompare in debug mode"
  echo "DISPLAY=${DISPLAY}"
  echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}"
  echo "XAUTHORITY=${XAUTHORITY:-<unset>}"
  exec "${CC_BIN}" "${MAP_ARGS[@]}"
fi

nohup "${CC_BIN}" "${MAP_ARGS[@]}" >/tmp/cloudcompare.out 2>/tmp/cloudcompare.err &

echo "CloudCompare launched with ${#MAP_ARGS[@]} file(s)."
echo "Log: /tmp/cloudcompare.err"
