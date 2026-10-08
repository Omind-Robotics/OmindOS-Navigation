#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_dir="${1:-${workspace}/artifacts-platform-preview}"
mkdir -p "${artifact_dir}"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${workspace}/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-171}"
launch_pid=''
cleanup() {
  if [[ -n "${launch_pid}" ]]; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    sleep 0.2
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=''
  fi
}
trap cleanup EXIT
for platform in wheeled_diff wheeled_omni quadruped; do
  result="${artifact_dir}/${platform}.json"
  log="${artifact_dir}/${platform}.log"
  rm -f "${result}"
  setsid ros2 launch nav3d_native platform_preview.launch.py \
    platform:="${platform}" result_file:="${result}" \
    map_output:="${artifact_dir}/${platform}-map" >"${log}" 2>&1 &
  launch_pid=$!
  passed=false
  for _ in $(seq 1 230); do
    if [[ -f "${result}" ]]; then
      status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result}")"
      if [[ "${status}" == PASS ]]; then
        test -s "${artifact_dir}/${platform}-map.pgm"
        test -s "${artifact_dir}/${platform}-map.yaml"
        python3 -c 'import json,sys; print(sys.argv[1], json.load(open(sys.argv[2])))' "${platform}" "${result}"
        passed=true
        break
      fi
      if [[ "${status}" == FAIL ]]; then break; fi
    fi
    kill -0 "${launch_pid}" 2>/dev/null || break
    sleep 0.5
  done
  cleanup
  if [[ "${passed}" != true ]]; then
    tail -100 "${log}" >&2
    exit 1
  fi
done
