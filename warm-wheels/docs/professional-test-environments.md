# Professional test environments

Use three levels: deterministic software regressions, workstation physics/sensor simulation, then measured hardware trials. Visual realism is not a pass criterion; repeatability, logged evidence, failure injection and measured behavior are.

| Environment | Use | Provided status |
|---|---|---|
| Python scenario runner | Mission coverage/return, blocked paths, depth faults, command gates | Runnable; local reports included |
| Gazebo Harmonic lab | 3D geometry, narrow doors, drivetrain/sensor integration, visual distractors | Four generated SDF fixtures and instructions; engine not installed/run here |
| Pi/Uno/D435i bench | Actual capture cadence, depth validity, serial acknowledgement, bounded raised-wheel pulse | Runnable tools; hardware not attached here |

## Workstation physics lab: Gazebo Harmonic

Run simulation on the workstation; the operational Pi should spend its resources on sensing and control. [Gazebo Harmonic supports RGB-D cameras and physics](https://gazebosim.org/docs/harmonic/comparison/), and its [sensor examples](https://gazebosim.org/docs/harmonic/sensors/) provide a basis for ROS integration. Use the [official installation procedure](https://gazebosim.org/docs/harmonic/install_ubuntu/) on a compatible Ubuntu workstation/VM; hardware-accelerated graphics and sensor rendering must work there. This repository does not install Gazebo, change system package repositories, or claim it ran on this Windows host.

Generate the worlds from the project directory:

```sh
python -m sim.gazebo.generate_worlds
gz sim -r sim/gazebo/worlds/empty.sdf
```

The included scenes are:

- `empty.sdf`: a bounded 6×6 m, four-room floorplan with no person/fire targets.
- `blocked-door.sdf`: one doorway filled by a solid door; remaining routes may still exist.
- `obstacle-course.sdf`: boxes added to the otherwise empty layout.
- `visual-distractor.sdf`: an orange non-fire object for exposing warm-color false positives.

Each contains a generic four-wheel skid-steer rover, front RGB-D sensor, drivetrain plugin, floor, walls, broad doorways, light and collision shapes. Assets are local primitives with no downloads. The source uses the [official differential-drive plugin conventions](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive.sdf).

**These are engineering fixtures, not a calibrated ELEGOO/D435i digital twin.** Wheel radius, track, mass, friction, camera FOV and payload dimensions are declared assumptions in `generate_worlds.py`. Replace them with measurements. The ideal camera does not reproduce smoke, IR interference, depth holes or thermal imaging.

Check world loading and actual published topics before connecting any controller:

```sh
gz topic -l
gz topic -e -t /warm_wheels/odometry
```

The drive topic is `/warm_wheels/cmd_vel`; the RGB-D sensor prefix is `/warm_wheels/rgbd`. Inspect `gz topic -l` for version-specific image/depth/camera-info suffixes. The vehicle starts stationary. These worlds are not connected to the dashboard or mission controller yet. A Gazebo-to-`Frame` camera adapter and map/pose navigation adapter remain integration work; **loading a scene is not an autonomous mission pass**. Gazebo velocity commands also do not automatically inherit the Uno watchdog; explicitly zero simulation velocities when testing a controller.

For each physics run, retain the world file/hash, engine/plugin versions, initial pose, camera settings, controller commit, sensor/log timestamps and the measured result. Test physics and real hardware at matched speeds and geometry before transferring a controller. Ground-truth pose can score simulation error but must not be used as the deployment localization source.

## Other professional tools

- **Webots** is a useful alternative when a desktop GUI workflow is preferable; the vendor supports Windows, Linux and macOS. No Webots world/controller is bundled here. [Official Webots site](https://www.cyberbotics.com/)
- **Isaac Sim** can remain useful for visual datasets if a teammate already has a suitable workstation. Its resources belong on that workstation, not the 4 GB Pi. Check the requirements for the installed release; the referenced 5.0 requirements are version-specific. No Isaac assets are bundled. [NVIDIA requirements](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html)

Choose one physics simulator and a small reproducible scene first. Do not spend the hackathon rebuilding the same integration across multiple engines.

## Roomba-style operational target on limited hardware

Use a small 2D occupancy map for navigation, a frontier/search state machine, a bounded local obstacle controller, and a verified home location. For example, a 10×10 m map at 10 cm resolution has only 10,000 cells; grid storage is modest, while robust localization and perception are the harder workloads. Treat 3D visualization as optional telemetry and reduce its frequency before compromising motor/sensor deadlines.

Keep the Uno responsible for motor outputs and watchdogs. The Pi should own freshness checks, obstacle stopping, navigation state and return decisions; a PC may help with optional AI or visualization. This is the intended architecture, **not a claim that physical autonomous navigation is already delivered**. The current motor code supports supervised driving; real localization, persistent mapping and path following are still required for independent room exploration and returning.
