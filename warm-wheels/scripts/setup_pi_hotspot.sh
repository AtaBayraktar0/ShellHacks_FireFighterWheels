#!/usr/bin/env bash
# Default is a read-only plan; append --apply to change the Pi's Wi-Fi profile.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
WW_ACTION=plan
if [[ "${1:-}" == "--apply" ]]; then
  WW_ACTION=apply
  shift
fi
exec python3 -m scripts.pi_hotspot "$WW_ACTION" "$@"
