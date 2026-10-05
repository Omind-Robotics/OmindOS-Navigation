#!/usr/bin/env bash
set -euo pipefail

GO2_PACKAGE="$(rospack find go2_base_controller)"
CONTROL_SCRIPT="${GO2_PACKAGE}/loco/go2_base_control.sh"

cleanup() {
  "$CONTROL_SCRIPT" stop || true
}

trap cleanup EXIT INT TERM

"$CONTROL_SCRIPT" start

while true; do
  sleep 5
done
