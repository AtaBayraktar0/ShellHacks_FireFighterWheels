# CODEX edition — Isaac Sim 6.1

This integration was created by **OpenAI Codex** in a separate project copy. Target: **NVIDIA Isaac Sim 6.1**, on a supported NVIDIA RTX Windows or Linux desktop. The original car project is user-provided code; Codex authorship applies to the documented modifications and integration.

**Status: implemented and CPU-tested; not yet executed in Isaac Sim.** This development machine cannot run Isaac Sim. Run the desktop smoke test below before treating the integration as validated. The earlier 2D simulation results do not establish 3D physics or RTX camera performance.

## What runs

`run_isaac.py` creates a bounded table, obstacles, an orange flame prop, and a procedural four-wheel skid-steer robot. It commands revolute wheel joints through the 6.1 experimental Articulation API; it does not teleport the robot along prerecorded waypoints. The existing `Mission` mapping/planning logic is reused through `IsaacBase` and camera adapters.

The default RGB-D mode uses 6.1 `RtxCamera` / `CameraSensor`, rendered RGB, and image-plane depth. Existing HSV flame detection and camera-to-robot geometry process those frames. The camera is panned virtually; servo mechanics are not modeled. The flame prop is a colored solid object, not a fire/fluid simulation.

`--sensor raycast` is a diagnostic option using PhysX scene rays with idealized floor points and flame labels. It is useful for isolating motor/planner problems, but is not a realistic camera validation.

The chassis dimensions, tire friction, masses and motor gains are approximations. This is not calibrated Elegoo CAD. Heading comes from physics yaw as an ideal gyro; timed distance estimation remains in use, so slip and position drift remain possible. Ground-truth XY is used for evaluation/debug sensing, not mission localization.

## Upload and clone

Upload the **contents of this CODEX folder** to your GitHub repository. Keep `run_isaac.py`, `isaac/`, `pi/`, and the other folders together at the repository root. Include `.github/workflows/tests.yml` and `.gitignore`. Do not upload virtual environments, outputs or the distribution ZIP as source files.

On the desktop, clone your repository and open a terminal in its root. Install Isaac Sim **6.1** according to NVIDIA's workstation instructions, then run with its bundled Python launcher. Do not use the original `pi/run_mission.py` for Isaac: it selects either the 2D simulator or physical serial hardware.

## Windows (PowerShell)

Replace the installation path below with the actual one:

```powershell
$env:ISAAC_SIM_PATH = 'C:\isaacsim'
& "$env:ISAAC_SIM_PATH\python.bat" -c "import numpy, cv2; print('Dependencies OK')"
.\scripts\run_isaac.bat --smoke-test --output outputs/smoke
.\scripts\run_isaac.bat --hold --save-stage --output outputs/mission
```

## Linux (bash)

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
"$ISAAC_SIM_PATH/python.sh" -c "import numpy, cv2; print('Dependencies OK')"
bash scripts/run_isaac.sh --smoke-test --output outputs/smoke
bash scripts/run_isaac.sh --hold --save-stage --output outputs/mission
```

The environment variable points to the directory containing `python.sh` / `python.bat`. If using an Isaac Sim pip installation instead, use that installation's activated Python environment: `python run_isaac.py --smoke-test`.

If `cv2` is missing, install it with **that same Isaac interpreter**:

```bash
# Linux example; on Windows substitute the python.bat launcher.
"$ISAAC_SIM_PATH/python.sh" -m pip install --no-deps opencv-python-headless==4.11.0.86
```

Isaac provides NumPy and its own runtime dependencies. Do not install `pi/requirements.txt` into Isaac: that file includes physical RealSense dependencies that this integration does not use.

## First desktop checks

1. Run `--smoke-test`. It requires floor and obstacle observations, approximately 12 cm of forward motion in the correct direction, and a left turn near 45 degrees. It exits nonzero on failure and stops wheel commands.
2. If the camera fails, repeat with `--sensor raycast --smoke-test --output outputs/raycast-smoke`. A pass here checks the idealized sensor/control path, not RTX RGB-D.
3. Run the default mission. Review `result.json`; a finished mission can still fail the true-position tolerance check.
4. Run the alternative flame layout: `--layout flame --output outputs/flame`.

Do not manually pause the timeline during a mission: the runner will abort rather than run its motor loops with a stopped clock. `--hold` pauses physics only after success, keeping the window open for inspection. Headless mode still needs compatible NVIDIA rendering hardware; it is not a CPU-only mode.

## Other commands

```bash
bash scripts/run_isaac.sh --headless --output outputs/headless
bash scripts/run_isaac.sh --sensor raycast --layout flame --output outputs/debug
bash scripts/run_isaac.sh --goal 2.0 0.0 --speed 0.15 --max-seconds 300
```

Use the `.bat` wrapper with the same flags on Windows. `--config path.json` accepts the existing configuration format. The Isaac runner requires `use_imu=true`, `edge_is_wall=true`, and a planning radius at least 0.145 m. Command-line speed defaults to 0.15 m/s and overrides the config speed; the model is slower than the hardware runner's defaults.

## Outputs and interpretation

Each output directory contains:

- `result.json`: creator, configuration, completion, true arrival errors, replans, clearance checks and pass/fail status.
- `true_trajectory.json`: `[simulation_seconds, x, y, heading_radians]` samples.
- `mission.log`: mission messages; `[CODEX]` also identifies terminal output.
- `camera_rgb.png` / `camera_depth.npy`: last captured RGB-D observation, when available.
- `error.txt`: traceback if an exception occurred.
- `codex_scene.usda`: generated scene when requested with `--save-stage`.

Exit codes: **0** passed, **1** runtime/mission error, **2** mission completed but true arrival tolerance failed, **130** interrupted. True goal and home errors must each be within `goal_tolerance_m` for a mission pass. The smoke test has separate motion/sensor criteria.

Clearances use a conservative 14.5 cm circular footprint against scene geometry at control/render updates. Negative clearance aborts the run. These are not PhysX contact counts and not a proof of continuous collision detection. The controller uses 120 Hz configured physics; actual application updates may contain multiple physics steps.

The demo table is 3.0 m by 2.5 m. Increasing these bounds is a change to the virtual arena, not a fix for a smaller physical table. Default fallback margins are retained.

## Tests without Isaac Sim

```bash
python -m pip install -r requirements-test.txt
python -m pytest -q pi/tests tests
```

GitHub Actions runs these CPU tests. They cover existing planning/perception/serial simulations and new wheel-command, depth-unprojection, clearance and control-failure checks. They cannot validate USD schemas, articulation dynamics, RTX rendering or Isaac runtime behavior. Runtime validation must happen on the desktop.

## API references

- [6.1 mobile robot controllers and lifecycle](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_simulation/mobile_robot_controllers.html)
- [6.1 articulation API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html)
- [6.1 RTX camera API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/index.html)
- [6.1 workstation installation](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html)
