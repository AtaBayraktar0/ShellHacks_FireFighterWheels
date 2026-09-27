#!/usr/bin/env bash
# Explicitly enabled, supervised manual motion after separate hardware bench checks.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
WW_SERIAL=""
WW_ENABLE_MOTORS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) [[ $# -ge 2 ]] || { echo "--port requires a serial device path"; exit 1; }; WW_SERIAL="$2"; shift 2 ;;
    --enable-motors) WW_ENABLE_MOTORS=1; shift ;;
    *) echo "Usage: bash scripts/start_pi_rover.sh --port /dev/serial/by-id/YOUR_UNO --enable-motors"; exit 1 ;;
  esac
done
if [[ -z "$WW_SERIAL" || "$WW_ENABLE_MOTORS" != 1 ]]; then
  echo "Explicit --port and --enable-motors are required after hardware bench validation."
  echo "For camera only, run bash scripts/start_pi_camera.sh."
  exit 1
fi
if [[ ! -x .venv-pi/bin/python ]]; then
  echo "Run bash scripts/prepare_pi.sh first."
  exit 1
fi
export ROVER_TOKEN="$(.venv-pi/bin/python -c 'import secrets; print(secrets.token_urlsafe(24))')"
echo "W.A.R.M wheels - supervised manual rover over Wi-Fi"
echo "Pi addresses: $(hostname -I)"
echo "On laptop: START_PRESENTATION.cmd > Connect Raspberry Pi."
echo "Enter http://PI_ADDRESS:8000 and the session token: ${ROVER_TOKEN}"
echo "Motor output requested on ${WW_SERIAL}; verified firmware and dashboard arming are still required."
echo "No hardware geofence or autonomous search is provided. Ctrl+C stops the server."
exec .venv-pi/bin/python -m rover.app --mode hardware --host 0.0.0.0 --http-port 8000 --port "$WW_SERIAL" --enable-motors
