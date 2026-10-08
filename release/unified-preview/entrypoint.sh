#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/omindos_preview/install/setup.bash
set -u
cd /opt/omindos_preview
case "${1:-profiles}" in
  profiles)
    python3 -c 'from nav3d_native.platform_guard import load_profile; [print(n, load_profile(n)) for n in ("wheeled_diff", "wheeled_omni", "quadruped")]'
    ;;
  test)
    python3 -m pytest src/nav3d_native/test -q
    bash scripts/validate_platform_preview.sh /tmp/omindos-results
    ;;
  preview)
    platform="${2:-wheeled_diff}"
    case "${platform}" in wheeled_diff|wheeled_omni|quadruped) ;; *) echo 'Unknown platform' >&2; exit 2 ;; esac
    exec ros2 launch nav3d_native platform_preview.launch.py \
      platform:="${platform}" result_file:=/tmp/omindos-result.json \
      map_output:=/tmp/omindos-map
    ;;
  *) echo 'Usage: profiles | test | preview {wheeled_diff|wheeled_omni|quadruped}' >&2; exit 2 ;;
esac
