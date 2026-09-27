# Raspberry Pi 4 / 4 GB bring-up

Target **64-bit Raspberry Pi OS**. The actual OS installed on your Pi is still unknown. Check before choosing packages:

```sh
uname -m
cat /etc/os-release
python3 --version
```

`aarch64` identifies a 64-bit kernel; also check Python is 64-bit with `python3 -c 'import struct; print(struct.calcsize("P")*8)'`. Back up any existing team code before changing the OS or flashing the Uno. This project does not require reinstalling the OS, Isaac Sim, or ROS just to stream camera data and control motors.

## Python service

Copy the `warm-wheels` directory to the Pi, then:

```sh
sudo apt update
sudo apt install python3-venv python3-dev build-essential cmake git libusb-1.0-0-dev pkg-config
cd warm-wheels
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
```

If OpenCV or NumPy has no wheel for your Python/OS combination, use compatible OS/Python packages rather than assuming an x86 wheel will work on ARM. Dependency bounds in `requirements.txt` are not an ARM-tested lockfile.

## RealSense SDK

The [official wrapper](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md) supports `pip install pyrealsense2`, but a compatible wheel is not guaranteed for every Pi Python/ARM combination. First try inside the active environment:

```sh
python -m pip install pyrealsense2
python -c 'import pyrealsense2 as rs; print(rs.context().query_devices())'
```

If no matching wheel exists, build the SDK Python bindings from source. Use an SDK release compatible with your OS, Python, and camera firmware; record the tag/commit you use. This is an adapted headless RSUSB build recipe based on [official SDK build instructions](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md) and [Python binding flags](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md), not a tested image for your Pi:

```sh
cd ..
git clone https://github.com/realsenseai/librealsense.git
cd librealsense
# Optionally git checkout your selected stable SDK release tag here.
python -m pip install setuptools
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="$(command -v python)" \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF
cmake --build build -j2
sudo cmake --install build
sudo ./scripts/setup_udev_rules.sh
sudo ldconfig
```

On 4 GB RAM use `-j2` initially; a source build may take significant time. If CMake selects a different Python than the virtual environment, fix that before building. If the import still fails, locate the generated `pyrealsense2*.so` and `librealsense2.so*` under `build/` and the install prefix; ensure the matching binding directory is on `PYTHONPATH` and the library directory on the loader path. Do not copy a Python binding built for a different Python version/architecture. The wrapper documentation describes these paths and install alternatives.

Reconnect the D435i after udev rules are installed. Use a USB 3 data cable and the Pi's blue USB 3 port; check `lsusb -t`. If the SDK includes `rs-enumerate-devices`, use it to inspect supported streams. The project requests depth Z16 and color BGR8 at 640×480 / 15 fps. If that exact profile is unsupported, choose supported matching dimensions/rates in `rover/camera.py`; the code should fail clearly rather than silently switch to synthetic frames.

The D435i IMU is not consumed in this version. An IMU reading is not a drift-free house position. No thermal measurement is available from the color/depth streams.

## Mount calibration

The initial projection assumes a level camera **0.22 m above the floor**, robot half-width **0.20 m**, and collision height **0.45 m**. Measure the assembled rover including the camera, cables, and Pi. Set `--camera-height`, `--robot-half-width`, and `--obstacle-height` to those values when launching. Keep the camera rigid and level; this version does not compensate for pitch/roll. The 3.5 cm floor cutoff can miss low objects, and neither this filter nor the camera establishes drop-off safety.

Example after measuring your rover (numbers here are defaults, not measurements):

```sh
python -m rover.app --mode hardware --host 0.0.0.0 \
  --camera-height 0.22 --robot-half-width 0.20 --obstacle-height 0.45
```

Missing depth is retained. A hole in the forward safety window makes clearance unknown and inhibits motion. Expect frequent stops until capture, mounting, and lighting are stable; do not paper over holes by converting them to free space.

## Serial permission and remote viewing

```sh
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$USER"
```

Log out and in after changing group membership. Upload firmware from Arduino IDE and close Serial Monitor first. If the kit camera/ESP controller shares the Uno's RX/TX pins, disconnect that UART source according to the kit instructions so it cannot interfere with the Pi link.

Set the same long `ROVER_TOKEN` on Pi and PC. `--host 0.0.0.0` makes the server listen on the local network; browse to the Pi's actual address (`hostname -I` helps identify it), not `0.0.0.0`. Do not bind or forward the service to the public internet. Wi-Fi loss stops driving; it does not trigger autonomous return, because that requires validated localization and navigation.
