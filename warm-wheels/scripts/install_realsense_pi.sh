#!/usr/bin/env bash
# Build the RealSense Python binding on ARM64 when PyPI has no wheel.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv-pi"
VERSION="v2.58.1"
SOURCE="$HOME/.cache/warm-wheels/librealsense-${VERSION}"
BUILD="$SOURCE/build-warm"

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "aarch64" ]]; then
  echo "This helper is intended for a 64-bit Raspberry Pi Linux installation."
  exit 1
fi
if [[ ! -x "$VENV/bin/python" ]]; then
  echo "Missing $VENV. Run bash scripts/prepare_pi.sh once first."
  exit 1
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config libusb-1.0-0-dev libudev-dev \
  libssl-dev python3-dev

mkdir -p "$(dirname "$SOURCE")"
if [[ ! -d "$SOURCE/.git" ]]; then
  git clone --depth 1 --branch "$VERSION" \
    https://github.com/realsenseai/librealsense.git "$SOURCE"
fi

SITE_PACKAGES="$($VENV/bin/python -c 'import site; print(site.getsitepackages()[0])')"
cmake -S "$SOURCE" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-D_GNU_SOURCE" \
  -DCMAKE_CXX_FLAGS="-D_GNU_SOURCE" \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="$VENV/bin/python" \
  -DPYTHON_INSTALL_DIR="$SITE_PACKAGES" \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_TOOLS=ON

# Two jobs stays within the practical memory envelope of a 4 GB Pi 4.
cmake --build "$BUILD" --parallel 2
sudo cmake --install "$BUILD"

sudo install -m 0644 "$SOURCE/config/99-realsense-libusb.rules" \
  /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Some CMake versions install the module outside the requested venv path.
# Link any installed Python package back into the project environment.
if ! "$VENV/bin/python" -c 'import pyrealsense2' 2>/dev/null; then
  MODULE="$(find "$BUILD/wrappers/python" /usr/local/lib -type f \
    \( -name 'pyrealsense2*.so' -o -name 'pyrealsense2*.so.*' \) 2>/dev/null \
    | head -n 1)"
  if [[ -z "$MODULE" ]]; then
    echo "Build completed but the pyrealsense2 module was not found."
    exit 1
  fi
  ln -sf "$MODULE" "$SITE_PACKAGES/$(basename "$MODULE")"
fi

"$VENV/bin/python" -c 'import pyrealsense2 as rs; print("pyrealsense2 import OK", rs.__file__)'
echo "RealSense binding installed. Unplug/replug the D435i, then run:"
echo "  cd $ROOT && bash scripts/start_pi_camera.sh"
