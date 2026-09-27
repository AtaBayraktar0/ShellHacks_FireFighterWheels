#!/usr/bin/env bash
# Run on the Pi after copying the project; never connects remotely or drives motors.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Run this helper on the Raspberry Pi's Linux installation."
  exit 1
fi
python3 -m venv .venv-pi
.venv-pi/bin/python -m pip install -r requirements.txt
if ! .venv-pi/bin/python -m pip install --only-binary=:all: pyrealsense2; then
  echo "No compatible RealSense wheel for this Pi/Python; building the ARM64 binding."
  bash scripts/install_realsense_pi.sh
fi
.venv-pi/bin/python -m scripts.hardware_preflight --output reports/pi-preflight.json
echo "Python dependencies prepared. Review docs/raspberry-pi.md for USB permissions."
echo "Camera-only check: .venv-pi/bin/python -m bench.check_hardware --mode hardware --duration 10"
echo "Camera-only Wi-Fi dashboard: bash scripts/start_pi_camera.sh"
echo "No firmware uploaded, serial ports opened, or motor commands sent."
