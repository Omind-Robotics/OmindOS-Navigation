#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/.." && pwd)"

runtime_roots=(
  "${workspace}/src/3D_NAV/nav_bringup"
  "${workspace}/src/3D_NAV/map_process"
  "${workspace}/src/3D_NAV/global_planner/planning_3d"
  "${workspace}/src/3D_NAV/local_planner/nmpc_planner"
  "${workspace}/src/3D_NAV/localization/FAST_LIO/launch"
  "${workspace}/src/3D_NAV/localization/map_publisher"
)

if rg -n --glob '*.launch' --glob '*.yaml' --glob '*.cpp' --glob '*.py' \
  --glob '*.sh' '/home/[^/]+/' "${runtime_roots[@]}"; then
  echo "Portability check failed: maintained runtime files contain home-directory paths." >&2
  exit 1
fi

if rg -n --glob '*.launch' --glob '*.yaml' --glob '*.sh' \
  '(PASSWORD|TOKEN|API_KEY)="[^"$][^"]*"' "${runtime_roots[@]}"; then
  echo "Portability check failed: possible credential in runtime configuration." >&2
  exit 1
fi

echo "Portability check passed."
