#!/usr/bin/env bash
# Explicit opt-in rollback; removes only the recorded WARM wheels AP profile.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "${1:-}" != "--apply" || "$#" != 1 ]]; then
  echo "Preview: removes only the managed hotspot and tries the previously active Wi-Fi profile."
  echo "Run on the Pi: sudo bash scripts/rollback_pi_hotspot.sh --apply"
  exit 0
fi
exec python3 -m scripts.pi_hotspot rollback
