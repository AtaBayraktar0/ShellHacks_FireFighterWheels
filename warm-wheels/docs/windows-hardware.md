# Laptop hardware setup

The project has its own prepared Windows Python environment in `.venv`. It includes
the dashboard dependencies, tests, USB serial support, and the official D435i
`pyrealsense2` binding. This computer has Arduino IDE's CLI 1.5.0 and Arduino AVR
Boards 1.8.8. The recorded check found **no RealSense camera and no Uno USB device**;
COM5 and COM6 were Bluetooth ports. Check current results with the launcher below.

## First check when plugging equipment into this laptop

Double-click **hardware_preflight.cmd**. It changes to its own project directory
and uses the project's Python, so there is no activation step. The window remains
open and reports dependencies, RealSense devices, USB serial candidates, and the
Uno compiler. It writes `reports/laptop-preflight.json`.

This check enumerates devices without opening a COM port, starting a camera
stream, uploading firmware, or moving motors. A recognized CH340 USB adapter is
a candidate, not proof of an Uno: read the actual ELEGOO board label. Bluetooth
COM ports cannot be used by the firmware uploader.

If a camera is absent, connect its USB-C port to a laptop USB 3 port using a USB 3
data cable. If the Uno is absent, connect the Uno USB-B port using a data cable.
Try another data cable/USB port before changing drivers. Device Manager can show
whether Windows reports an error. No driver replacement, admin changes, or reboot
was performed by this setup.

After the camera appears, close other camera viewers and run this from a terminal
opened in the project directory:

```bat
.venv\Scripts\python.exe -m bench.check_hardware --mode hardware --duration 10
```

This captures depth and RGB with no motor output. A ten-second pass is a capture
test, not a full physical acceptance test. See
[the hardware test environment](hardware-test-environment.md) for evidence limits.

## Compile and upload the Uno bridge

Double-click **firmware_compile.cmd** to compile the default V4/unverified build.
It uses the existing Arduino toolchain and saves binaries, compiler output, and a
source/checksum manifest in `reports/firmware/unverified-v4/`. This build does not
enable any motor pins. A verified build cannot be selected without both explicit
pin-check and raised-wheel flags.

To upload the default build later, identify the Uno's COM number in preflight and
inspect the board. With motor power disconnected, replace **COM7** in both places:

```bat
firmware_upload.cmd --port COM7 --confirm-port COM7 --confirm-uno
```

The helper requires that exact port to be present as a recognized USB Uno/CH340
candidate, compiles the sketch, then checks the port again before uploading.
It rejects Bluetooth and unspecified/mismatched ports. Upload overwrites the Uno's
current sketch; no firmware was uploaded while preparing this laptop.

Only after matching the [V4 pin map](hardware.md) and raising all wheels on a
stable stand, select the motor-enabled firmware explicitly:

```bat
firmware_upload.cmd --port COM7 --confirm-port COM7 --confirm-uno --verified --confirm-pin-check --wheels-raised
```

The flags record your checks; software cannot inspect the stand or wiring.
Upload does not send drive commands. Continue with the separate, bounded
raised-wheel pulse checks in [hardware-test-environment.md](hardware-test-environment.md).
Keep a spotter at the motor-power disconnect.

## Laptop over Wi-Fi; cables remain on the Raspberry Pi

The presentation laptop can receive RGB, depth, map data, and controls over Wi-Fi.
The **D435i connects by USB to the Raspberry Pi 4**, and the **Uno connects by USB
to the Pi**. The Pi needs suitable USB-C power; it does not need a USB tether to
the laptop. The motors use their separate kit supply.

Put the Pi and laptop on the same private Wi-Fi/LAN or hotspot. Do not configure
router port forwarding. On the Pi, `hostname -I` shows local addresses. The
camera-only starter prints its address guidance and a fresh session token for
the laptop's Connect Raspberry Pi page; it does not write the token to a report.

Copy the project source to the Pi, excluding Windows `.venv`, `.venv-ai`, and model
weights. The laptop performs AI inference. A copy-ready source ZIP can be made
from a terminal in the project directory using Windows' built-in tar:

```bat
tar -a -c -f warm-wheels-pi.zip rover dashboard bench scripts docs firmware requirements.txt
```

Transfer/extract that ZIP into a project folder on the Pi, then run:

```sh
bash scripts/prepare_pi.sh
bash scripts/start_pi_camera.sh
```

The setup helper creates `.venv-pi`, installs Python requirements and a compatible
prebuilt RealSense binding, and performs enumeration. It does not install kernel
drivers, upload firmware, reboot, or contact a remote Pi. If no compatible wheel
exists for the Pi's Python/architecture, it stops with guidance; see
[Raspberry Pi setup](raspberry-pi.md). USB access permissions may still need the
documented Pi-specific setup.

The starter binds the authenticated dashboard to `0.0.0.0:8000` with **motors
disabled**. On the laptop open `START_PRESENTATION.cmd`, choose **Connect
Raspberry Pi**, and enter `http://PI_ADDRESS:8000` plus the printed token. A direct
browser connection to `http://PI_ADDRESS:8000/` is also available. After joining,
`START_AI.cmd` can select **Joined Raspberry Pi** and reuse the laptop session
token without asking you to find it. Hardware motion must be enabled separately
after commissioning, using an explicit Pi serial path:

```sh
bash scripts/start_pi_rover.sh --port /dev/serial/by-id/YOUR_UNO_DEVICE --enable-motors
```

That helper starts supervised manual control; it does not drive on launch.
A physical meter-based boundary needs reliable rover localization; the current
camera-only scan does not implement a physical geofence or autonomous search.

## Recreate the Windows environment if needed

The prepared environment works from this folder. If the project is moved, recreate
it rather than copying an old Windows virtual environment:

```bat
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt pyrealsense2
hardware_preflight.cmd
```

Use the [official RealSense Python wrapper instructions](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md)
and [official wheel listing](https://pypi.org/project/pyrealsense2/) when checking
supported Python/OS versions. The installed Windows wheel is 2.58.4.10922 for
CPython 3.12 x64. The NVIDIA AI environment is separate in `.venv-ai`.
