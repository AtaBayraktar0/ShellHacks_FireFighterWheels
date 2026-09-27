# W.A.R.M wheels

**Indoor reconnaissance prototype for ELEGOO Smart Robot Car V4.0 + Raspberry Pi 4 (4 GB) + RealSense D435i.**

This branch contains the application source; run the commands below from the
`warm-wheels` directory. Virtual environments, model weights, camera recordings,
session tokens and local hardware inventory are not included. Install dependencies
and download models using the included setup tools. The connected camera in the
latest Pi session identified itself as a **D435**, so IMU availability is not assumed.

The transportation problem: discover obstructions and people, make that information visible to an operator, and eventually support evacuation routing. The mission is **search, map, report, return** even if no person or fire is found.

This repository runs a useful hardware prototype and a separate autonomous mission simulator. It does **not** yet autonomously scan or escort people through a real house. Physical autonomy needs localization, persistent mapping, exit verification, and wider obstacle coverage; the D435i alone does not supply those. Use normal, supervised indoor test spaces and simulated hazards.

## What works in this version

| Component | Implemented behavior |
|---|---|
| Pi + D435i | Aligned color/depth frames, approximate object depths, a fresh local occupancy scan and a bounded colored surface mesh |
| Pi + Uno | USB serial motor commands, exact acknowledgements, bounded forward movement, stale-command/depth stops |
| V4 Arduino firmware | Documented V4 pins, disabled-by-default motor outputs, emergency latch, 350 ms watchdog |
| PC AI worker | Installed person/pose and trained fire/smoke checkpoints; automatic CUDA selection, cautious pose cues and frame-matched candidate observations |
| Visual fire hints | Warm-color blobs labeled unverified candidates; orange objects can trigger them |
| Operator dashboard | Desktop/mobile browser, color/depth, local map, observations, manual assistance notes and supervised stop controls |
| 3D scan viewer | Orbitable triangle surface, RGB/height/wireframe styles, frame-matched observation pins, source selector and snapshot export |
| 3D mission theater | Procedural floorplans and encounters, accumulated ray-cast depth mesh, evolving fire, observed actor responses, 1–50 m boundaries and following checks |
| Route planning | Unknown-blocking, footprint-aware A* preview; never sends motor commands |

**Not implemented:** real house SLAM, real autonomous exploration/return/escort, human identity tracking, reliable age classification, consciousness diagnosis, verified smoke/fire detection, thermal imaging, battery telemetry, autonomous battery-based return, audio instructions, cloud/mobile push alerts. The included mobile app is a responsive web dashboard, not a native phone app.

## Fastest demo on this PC

**Double-click `START_PRESENTATION.cmd`.** Press **New mission**, then **Play mission**. The seed generates the layout, entry point, people, responses, fires and possible interruptions together; there are no scenario buttons. Orbit the accumulated scan and switch **Surface / Wireframe / Points**. **Replay** repeats the same mission, or use **Load seed** to reproduce a saved one. Width and height sliders set the simulated planning boundary. See the [presentation and wireless connection guide](docs/presentation-guide.md). `START_HARDWARE.cmd` opens the camera-to-laptop depth viewer; `START_AI.cmd` starts the installed person/fire worker after a live stream is connected.

For a manual application setup, open a terminal in this folder. Python 3.11 or 3.12 is a reasonable starting point; verification here used Python 3.12. The prepared laptop already has its `.venv`.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m rover.app --mode demo
```

Open `http://127.0.0.1:8000/presentation` and enter the session token through **Session**, or open the operator console at the root URL. No car, camera, AI models, or physics simulator are needed for these synthetic demos.

The generator builds branching rooms, corridor apartments, open interiors, and irregular suites with variable doors, angled dividers, rounded furniture and launch sides. People and fire origins are sampled across near, middle and far regions. Their locations remain unknown to the rover until observed. A seed reproduces geometry and environmental evolution; it does not select one of eight fixed stories.

The viewport displays accumulated triangle patches from first-hit rays against continuous physical surfaces. Occluded surfaces remain absent. The simulated camera has a forward 87° horizontal field of view and retains its previously acquired patches as the rover moves; turning or revisiting exposes new surfaces. This is **synthetic depth**, not a D435i recording, and it uses known simulated poses for accumulation. The navigation planner uses a separate conservative 2D occupancy map. Fire intensity and bounded neighbor-to-neighbor spread can change that map; this is a scenario hazard model, not a combustion solver. Five visible actor response samples support a simulated assistance/escort decision, not a medical diagnosis.

The separate `/scan` camera viewer offers a single-frame view and experimental
retained RGB-D scans using estimated camera poses. Select **New retained scan**
in camera-only mode and move the camera slowly by hand. See
[retained-room-scan.md](docs/retained-room-scan.md) for tracking-loss handling,
export, storage limits and drift limitations. Physical autonomous driving and
whole-house reconstruction remain unimplemented. Coverage is a simulation
diagnostic using initial reachable cells; partial returns and blocked outcomes
never imply an all-clear.

## Hardware setup

Follow [hardware.md](docs/hardware.md) for the exact V4 pin table, USB connections, Uno upload, disabled-by-default firmware, and raised-wheel checks. The kit's original small camera is not used; the D435i plugs directly into the Pi over USB 3. Power the Pi independently through its USB-C power input. The micro-HDMI ports are optional display outputs.

Follow [raspberry-pi.md](docs/raspberry-pi.md) to install the service and RealSense binding. Start with camera-only operation:

```sh
export ROVER_TOKEN='replace-with-a-long-random-local-session-token'
python -m rover.app --mode hardware --host 0.0.0.0
```

Camera-only mode opens no Arduino port. After the firmware and bench checks pass:

```sh
python -m rover.app --mode hardware --host 0.0.0.0 \
  --port /dev/ttyACM0 --enable-motors
```

Use the actual `/dev/serial/by-id/...` path when available. Motor output additionally requires a verified firmware profile, explicit **Arm rover**, a fresh valid forward depth view, and holding a drive button. Power-up never arms or resumes a mission.

### Enabling real motion (checklist)

If the camera and scan work but the car never moves, you are almost certainly
running camera-only. The startup log prints `Motors: OFF (camera-only)` and the
dashboard's drive panel shows `Motor output disabled at launch`. Physical motion
needs every step below, in order:

1. **Firmware opt-in.** The shipped sketch has `HARDWARE_VERIFIED 0`, so the Uno
   reports `PROFILE V4 UNVERIFIED` and refuses drive commands; the server then
   fails to start with motors enabled. After the raised-wheel checks in
   [hardware.md](docs/hardware.md), upload a verified build
   (`firmware_upload.cmd --verified --confirm-pin-check --wheels-raised --port COMx --confirm-port COMx --confirm-uno`
   on Windows, or set `HARDWARE_VERIFIED 1` and upload with the Arduino IDE).
2. **Launch opt-in.** Start in hardware mode with motor output enabled, using one of:
   - `python -m rover.app --mode hardware --host 0.0.0.0 --enable-motors --port /dev/serial/by-id/YOUR_UNO`
   - `WARM_WHEELS_ENABLE_MOTORS=1 WARM_WHEELS_SERIAL_PORT=/dev/serial/by-id/YOUR_UNO python -m rover.app --mode hardware --host 0.0.0.0`
   - On the Pi: `bash scripts/start_pi_rover.sh --port /dev/serial/by-id/YOUR_UNO --enable-motors`
     (`start_pi_camera.sh` is camera-only by design).
   - On a Windows laptop with the car on USB: `START_HARDWARE.cmd --enable-motors --port COMx`.

   The startup log must print `Motors: ENABLED on <port>`. `--enable-motors` is
   rejected in demo mode.
3. **Use the operator console.** Drive controls live on `/` (or `/pi` when the
   laptop is connected to the Pi through **Connect Raspberry Pi**). The `/scan`
   viewer has no drive controls, and the retained room scan is refused while
   motors are enabled.
4. **Arm.** Press **Arm rover**. It stays disabled, with the reason shown, until
   the motor link is connected, the depth frame is fresh, at least 85% of the
   depth image is valid, and the forward window is clear for at least 0.65 m.
5. **Hold to drive.** Hold Forward / Left arc / Right arc. Releasing, a stale
   frame, an obstacle, a lost link, or **Emergency stop** stops the motors.

Still active when motors are enabled: emergency stop latch (UI, API and
firmware), the 25% PWM cap, forward-only gentle arcs, the 0.30 s hold-to-drive
command lease, the 0.45 s camera freshness check, the depth-validity and 0.65 m
obstacle stop, the 350 ms firmware watchdog, and acknowledged serial commands
that latch a stop on any failure. There is no physical autonomous driving:
route plans are previews and hardware missions are rejected.

On your phone or PC, open `http://PI_IP:8000` on the same trusted network. Use one operator console. Network examples use unencrypted HTTP for the local demo; do not publish/port-forward this control server. Use an SSH tunnel or a TLS proxy for networks you do not trust. The access token protects images and all API/control endpoints.

## Person and fire detection on the PC

On this prepared laptop, use **START_AI.cmd** after opening a live local camera or joining the Pi. It uses `.venv-ai`, both installed checkpoints, and CUDA GPU 0 when available; otherwise it selects CPU. To recreate that environment elsewhere, follow [model provenance and setup](docs/model-provenance.md). For a direct Pi connection from the prepared environment:

```powershell
$env:ROVER_TOKEN = 'same-token-as-the-pi'
.\.venv-ai\Scripts\python.exe -m pc.detector --rover http://PI_IP:8000
```

The defaults are the local files `models/yolo11n-pose.pt` and `models/fire.pt`, `--device auto`, and `--imgsz 640`. Person confidence defaults to 0.45 and fire/smoke confidence to 0.35. Missing local checkpoints produce a setup error rather than an implicit download by the worker. `--person-model` and `--fire-model` accept compatible local alternatives; check their provenance and licensing. The worker refuses synthetic input unless `--allow-demo` is explicitly supplied for a transport test. Expired results are discarded, and observations use their original aligned depth frame.

This laptop's RTX 4060 ran both models on public reference images: median local inference was 10.397 ms for person/pose and 8.894 ms for fire/smoke over 20 measured runs after warmup. These measurements exclude camera, network, Pi processing and display, and do not establish held-out accuracy. See the [recorded model test](docs/model-provenance.md). Real Pi camera throughput, Wi-Fi latency and candle sensitivity remain unmeasured. The 4 GB Pi service keeps inference on the laptop by default.

COCO person weights do not provide fire classes; the separate installed fire/smoke checkpoint supplies those candidates. To select an alternative evaluated checkpoint:

```powershell
.\.venv-ai\Scripts\python.exe -m pc.detector --rover http://PI_IP:8000 --fire-model models/your-fire-model.pt --fire-labels fire,flame,smoke
```

The prepared laptop now includes `models/yolo11n-pose.pt` and `models/fire.pt`, with pinned sources and measured sample inference in [model-provenance.md](docs/model-provenance.md). The fire model is a third-party D-Fire fine-tune; candle sensitivity has not been tested. Validate on held-out images from your camera and venue, including orange clothing/screens/lights and partial occlusions. Bounding-box median depth can reflect background; it is an approximate observation, not a verified victim position. A horizontal pose hint is not proof of fainting. Child/assistance/mobility notes are human reports tied to a particular observation and expire from the live view; the session log keeps the latest 100 notes in memory.

## The search-and-return behavior

1. Record a verified launch/exit position in a persistent coordinate frame.
2. Observe and update the map; explore reachable frontiers even if detections remain empty.
3. Report people/hazard candidates separately from exploration progress.
4. Replan when observed obstacles change; never treat unseen cells as free.
5. When no reachable exploration target remains, return along observed free space. An operator request or exploration budget can trigger an earlier return.
6. If the return path is blocked, stop and report **blocked**. Do not invent an exit route.

The simulator implements this mission loop. For actual rover integration, see [architecture.md](docs/architecture.md). Hardware mission start is rejected until the missing localization/navigation integration is built and tested.

## Verification

```sh
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

See [validation.md](docs/validation.md) for the final checks performed and physical checks still required. No hardware was attached to the development computer; no firmware was uploaded to your car.

## Engineering guidelines and test environments

Start with [engineering guidelines](docs/engineering-guidelines.md) and the [test plan](docs/test-plan.md), then use these layers:

1. **Repeatable software scenarios:** [simulation environment](docs/simulation-test-environment.md). Run `python -m validation.run_scenarios --output reports/simulation`; it saves JSON/CSV evidence and exits nonzero on a failed scenario. The example report is included.
2. **Workstation 3D physics:** [professional test environments](docs/professional-test-environments.md). Four self-contained Gazebo Harmonic worlds are bundled under `sim/gazebo/worlds/`. They have been generated and structurally checked, not executed in Gazebo here; deployment adapters are still required.
3. **Real camera/serial bench:** [hardware test environment](docs/hardware-test-environment.md). Start with `python -m bench.check_hardware --mode hardware --duration 10 --output captures/hardware-camera.json`. This does not drive the motors. A separate, explicitly enabled raised-wheel pulse tool is provided for motor direction checks.

For limited Pi hardware, see [Pi resource budget](docs/pi-resource-budget.md). The target is lightweight 2D navigation with optional PC assistance; the simulator and heavy development tools belong on the workstation.

## Source layout

```text
rover/app.py          authenticated dashboard/API and command-line entry point
rover/runtime.py      separate camera, mission, and motor loops
rover/camera.py       RealSense acquisition and explicit synthetic camera
rover/spatial.py      local projection, occupancy, A* route previews
rover/perception.py   weak visual fire hints and matched-frame depth
rover/safety.py       command lease, freshness, clearance and motion limits
rover/serial_link.py  acknowledged Uno link and simulated motor
rover/mission.py      synthetic frontier exploration and return-home mission
rover/presentation.py procedural world evolution and observed actor decisions
rover/world_geometry.py diverse floorplans and accumulated synthetic depth rays
rover/mesh.py         bounded aligned-depth triangulation and scan API
pc/detector.py       workstation person/pose and fire/smoke worker
firmware/rover_bridge/rover_bridge.ino
dashboard/           operator console, Three.js theater and depth-mesh viewer; local assets
tests/               geometry, mission, API and motor failure cases
```

## Primary references

- [ELEGOO V4 product](https://us.elegoo.com/products/elegoo-smart-robot-car-kit-v-4-0) and [official V4 firmware/pin map](https://github.com/elegooofficial/ELEGOO-Smart-Robot-Car-Kit-V4.0-New)
- [RealSense Python wrapper](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md) and [SDK source/build guidance](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md)
- [Ultralytics pose prediction API](https://docs.ultralytics.com/tasks/pose/)
