#!/usr/bin/env bash
# Camera-only server for a laptop on the same private Wi-Fi/LAN.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ ! -x .venv-pi/bin/python ]]; then
  echo "Run bash scripts/prepare_pi.sh on the Raspberry Pi first."
  exit 1
fi
export ROVER_TOKEN="$(.venv-pi/bin/python -c 'import secrets; print(secrets.token_urlsafe(24))')"
echo "W.A.R.M wheels - Raspberry Pi camera-only Wi-Fi server"
echo "Pi addresses: $(hostname -I)"
echo "On laptop: START_PRESENTATION.cmd > Connect Raspberry Pi."
echo "Enter http://PI_ADDRESS:8000 and the token below."
echo "Optional direct browser view: http://PI_ADDRESS:8000/"
echo "Session access token: ${ROVER_TOKEN}"
echo "Token is printed for this session only; it is not written to a report."
echo "Motor output is disabled (camera-only). For supervised driving after bench checks:"
echo "  bash scripts/start_pi_rover.sh --port /dev/serial/by-id/YOUR_UNO --enable-motors"
echo "Ctrl+C stops the server."
exec .venv-pi/bin/python -m rover.app --mode hardware --host 0.0.0.0 --http-port 8000
