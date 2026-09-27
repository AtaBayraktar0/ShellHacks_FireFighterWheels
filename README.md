# Fire-avoidance robot — Python prototype

**Updated Isaac launcher:** root `run_isaac.py` now generates our small-room
scene using the bundled CODEX integration. No external USD files are required.
Follow [ISAAC_ROOM.md](ISAAC_ROOM.md); it supersedes the external-USD Isaac
setup below. The real-robot tools remain separate.

Local software for a Raspberry Pi 4, Intel RealSense D435, and Elegoo V4.0 Uno
motor controller. There are no cloud services. A separate algorithm-only demo
uses a pretend room so A* and return behavior can be tested without hardware.

The project includes D435 capture, point-cloud export, floor mapping, local
flame-prop color detection, deterministic A*, guarded motor commands, and an Uno
motor-only listener. Comments in the code are short and beginner friendly.

This is an integration prototype. Real localization and full camera coverage are
still required before the car can drive autonomously. The controller refuses to
drive when its map or position is missing, old, or unsafe.

## Install and test the software

Use Python 3.10 or newer. Open a terminal in this folder:

```sh
python -m venv .venv
# Raspberry Pi / Linux:
source .venv/bin/activate
# Windows PowerShell instead:
# .venv\Scripts\Activate.ps1
python -m pip install -e .
python -m unittest discover -s tests -v
```

The automated tests use small made-up arrays to check the math safely.

## Algorithm-only demo

Run this before connecting hardware:

```sh
python -m firebot simulate --output demo
```

Open `demo/simulation.html` in a browser to play the animation, or open
`demo/route.svg` for the still route. The pretend world has fixed objects so the
test is repeatable, but A* cannot read that hidden world directly. A simulated
sensor scan discovers free and blocked cells and updates the robot's map first.
The blocked-return test adds an object to the hidden world; the next scan detects
it and the return check stops. The demo does not test the real D435, motors,
localization, wheel slip, or cardboard display.

Test the safety stop for a newly blocked return route:

```sh
python -m firebot simulate --block-return --output demo/blocked-return
```

## Files

| File | Purpose |
| --- | --- |
| `firebot/planning.py` | 5 cm grid, safety clearance, and A* |
| `firebot/vision.py` | Real D435 frames, floor mapping, flame color, and PLY export |
| `firebot/motors.py` | Real USB serial commands and short motor pulses |
| `firebot/mission.py` | Real sensor checks, planning, measured waypoints, and return |
| `firebot/isaac_bridge.py` | Isaac Sim 6.1 camera, pose, and wheel adapters |
| `run_isaac.py` | Standalone Isaac Sim 6.1 mission launcher |
| `isaac_config.json` | Isaac USD paths, joints, camera, and wheel measurements |
| `firebot/simulation.py` | Clearly separated pretend-room algorithm demo |
| `firebot/cli.py` | Hardware test commands and configuration checks |
| `firmware/motor_listener/motor_listener.ino` | Uno motor-only executor |
| `config.json` | Camera, grid, car, and motor measurements |
| `tests/test_robot.py` | Small checks for planning, mapping, pose, and motor safety |

## Hardware bring-up

### 1. Test the D435 on the Pi

Connect the D435 to a blue USB 3 port and keep the car still.

```sh
python -m pip install -e '.[hardware]'
python -m pip install --only-binary=:all: pyrealsense2
python -m firebot camera-test --output runs/camera
```

The command must open the real camera and report valid depth. It saves
`frame.npz` and `cloud.ply`. Installation success alone does not prove that USB
streaming works. A compatible `pyrealsense2` wheel may not exist for every Pi OS
and Python version.

### 2. Upload the Uno listener

Open `firmware/motor_listener/motor_listener.ino` in Arduino IDE. Select Arduino
Uno and its USB port, then verify and upload. Uploading replaces the stock
avoidance and line-tracking logic.

The defaults follow Elegoo's V4.0 pin map: right PWM D5, left PWM D6, right
direction D7, left direction D8, and standby D3. Confirm your shield revision and
motor direction before floor testing.

### 3. Test the serial link

Connect Pi to Uno by USB. Test STOP first:

```sh
python -m firebot serial-test --port /dev/ttyACM0
```

Lift the wheels before requesting one short forward pulse:

```sh
python -m firebot serial-test --port /dev/ttyACM0 --enable-motion
```

On Windows, use a port such as `COM3`. The Uno stops each pulse itself, rejects
bad commands, and uses a watchdog. Confirm forward, left, right, and stopping
before placing the car on the floor.

### 4. Prepare the flame prop

Use a stable red, orange, and yellow prop that produces usable D435 depth. Other
objects with these colors may also be marked as hazards. This is prop detection,
not a real-fire safety system.

### 5. Test real mapping and flame detection

Measure the mounted camera height and downward angle, then edit `config.json`.
Place the car at the configured start position. Yaw zero points along world +X.

```sh
python -m firebot map-scan --yaw-deg 0 --output runs/mapping
python -m firebot flame-test --output runs/flame
```

`map-scan` saves a map built from the real D435 frame. `flame-test` saves a real
camera picture with color candidates marked in magenta. A single scan only sees
the camera's current view; it is not a complete scan of the whole display.

Only observed floor becomes free. Objects above the floor, flame candidates,
unknown areas, missing depth, and old readings remain unsafe. Obstacles stay
blocked until a new mission starts.

### 6. Integrate after localization and scan coverage exist

A real localization program must repeatedly write this JSON structure:

```json
{"x": 0.525, "y": 0.525, "yaw": 0.0, "timestamp": 1780000000.0}
```

The timestamp must be the real measurement time. ArUco markers or tested visual
odometry can provide the pose. They are not implemented yet. The D435 depth image
alone does not tell the car its location in the room.

Before driving, measure camera placement, robot size, PWM, pulse travel, and
coasting distance. Confirm the camera can see the full safe area around each
move. Then set `calibrated` to `true` in `config.json`.

```sh
python -m firebot run-live --pose-file pose.json --port /dev/ttyACM0 --enable-motion
```

The live loop stops, reads the real camera and pose, updates the map, plans one
step, and sends one short motor pulse. It repeats those checks before every move.
At the goal, it follows measured waypoints home only when fresh D435 readings say
they are safe. Missing data, a blocked route, lost serial messages, or a timeout
stops the motors.

The forward-facing D435 cannot see behind or directly under the car. Multi-view
scanning and reliable localization must be built and tested before calling this
a fully autonomous robot.

## Isaac Sim 6.1

The shared mission code can also use simulated devices. `isaac_bridge.py`
changes Isaac RGB, depth, pose, and wheel controls into the same simple objects
used by the real robot. A* and the safety checks therefore stay unchanged.

Build an Elegoo car USD with two wheel joints, correct wheel size, car size,
mass, friction, and a camera mounted where the D435 will sit. Build the cardboard
display as a second USD scene. In an Isaac 6.1 standalone script:

1. Start `SimulationApp` before importing other Isaac modules.
2. Read RGB and depth from the simulated camera.
3. Pass those arrays and its camera intrinsics to `IsaacCamera`.
4. Pass the robot world pose to `IsaacPoseReader`.
5. Use Isaac's experimental `DifferentialController` inside the two callbacks
   given to `IsaacMotors`.
6. Call `run_mission` with those three adapters.

Run the script with Isaac Sim's `python.bat` on Windows. Isaac Sim 6.1 uses its
own Python environment, so do not run this part from the project's normal venv.
The bridge math has automated tests here. The complete simulation still needs
the car USD, scene USD, wheel joint names, camera path, and measured wheel sizes.

After creating those two USD files, edit `isaac_config.json`. Replace both file
paths and confirm the prim paths, wheel joint names, wheel measurements, and
camera intrinsics. Set `ready` to `true` only after checking them. From Isaac
Sim's installation folder on Windows, run:

```powershell
python.bat C:\path\to\ShellHacks_FireFighterWheels\run_isaac.py
```

Add `--headless` when a visible Isaac window is not needed. `run_isaac.py`
loads both USD files, starts the RTX color/depth camera, reads the simulated
pose, controls the wheels, and calls the shared `run_mission` loop.

References: [Isaac Sim 6.1 mobile robot controllers](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_simulation/mobile_robot_controllers.html)
and [standalone Python workflow](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/manual_standalone_python.html).

## Validation

- Planning, map safety, flame filtering, pose age, steering, and motor failure
  behavior have automated tests.
- D435 streaming, real motor motion, Pi speed, and Uno upload require the physical
  hardware and have not been tested in this Windows workspace.

## References

- [Official RealSense Python wrapper](https://github.com/realsenseai/librealsense/tree/master/wrappers/python)
- [RealSense projection conventions](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)
- [Elegoo V4.0 firmware and pin map](https://github.com/elegooofficial/ELEGOO-Smart-Robot-Car-Kit-V4.0-New)

This project contains work created outside any future hackathon window. Check the
event rules before submitting it as newly created hackathon work.
