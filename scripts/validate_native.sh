#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_dir="${1:-${workspace}/artifacts}"
mkdir -p "${artifact_dir}"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
set +u
source "${workspace}/install/setup.bash"
set -u
cd "${workspace}"
ament_flake8 \
  src/nav3d_native/nav3d_native \
  src/nav3d_native/test \
  src/nav3d_native/launch
colcon test --packages-select nav3d_native --event-handlers console_direct+
colcon test-result --verbose
result="${artifact_dir}/demo_result.json"
map_base="${artifact_dir}/demo_map"
log="${artifact_dir}/demo.log"
rm -f "${result}" "${map_base}.pgm" "${map_base}.yaml" "${log}"
setsid ros2 launch nav3d_native demo.launch.py result_file:="${result}" map_output:="${map_base}" >"${log}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${launch_pid}" 2>/dev/null || { wait "${launch_pid}" 2>/dev/null || true; return; }
    sleep 0.1
  done
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  sleep 0.5
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT
for _ in $(seq 1 100); do
  if [[ -f "${result}" ]]; then
    status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${result}")"
    if [[ "${status}" == PASS ]]; then
      test -s "${map_base}.pgm"
      test -s "${map_base}.yaml"
      cat "${result}"
      exit 0
    fi
    if [[ "${status}" == FAIL ]]; then
      cat "${result}" >&2
      tail -200 "${log}" >&2
      exit 1
    fi
  fi
  sleep 0.5
done
echo 'Timed out waiting for native ROS 2 demo' >&2
tail -200 "${log}" >&2
exit 1
