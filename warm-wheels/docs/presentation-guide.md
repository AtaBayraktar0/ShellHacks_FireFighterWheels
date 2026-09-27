# Present W.A.R.M wheels on this laptop

## Start here

Double-click **START_PRESENTATION.cmd** in the project folder. It starts the service on an available local port (normally 8010), opens the mission theater, and signs the browser in. It works from any starting directory. Keep the terminal open; Ctrl+C stops it. Opening the launcher again reuses its running session. It leaves the older 8000/8001 demonstrations alone.

The presentation assets, scenario logic, vendored Three.js libraries and installed AI weights are local. No cloud account or Internet connection is needed for the already-installed presentation. Wi-Fi is needed when joining the Pi wirelessly. The theater requires a WebGL-capable browser; it reports unavailable 3D graphics rather than substituting a flat scene.

The existing laptop has an Intel i7-13620H, about 16 GB system RAM, and an NVIDIA RTX 4060 Laptop GPU with 8 GB VRAM. The application environment is `.venv`; GPU inference is isolated in `.venv-ai`. No GPU driver upgrade or reboot was performed.

## A concise live demonstration

1. Press **New mission**, then **Play mission**. No incident type is selected in advance. New interiors, entrances, obstacles, people, response states, fires and occasional sensing interruptions are generated together.
2. Drag to orbit, right-drag to pan, and scroll to zoom. Watch **Surface** accumulate from depth returns. **Wireframe** exposes its triangle structure; **Points** shows sampled geometry. Unobserved and occluded surfaces remain absent.
3. Watch the observation cards and mission timeline. An observed fire can intensify and expand. A newly blocked route changes the plan or produces a stopped/blocked outcome. The world keeps evolving during response checks and follower holds.
4. When a person is observed, the rover holds for repeated simulated interaction samples. A mobile, willing actor may be escorted with distance/visibility checks. An explicit request for assistance or repeated lack of response triggers a return with the last observed location. These are simulated actor responses, not medical diagnoses.
5. **Replay** repeats the seed. A saved seed entered through **Load seed** reproduces the full mission. **New mission** generates a fresh one. Keyboard shortcuts: Space play/pause, R replay, N new mission when focus is outside form controls.
6. Set width and height using the **1–50 m sliders**, click **Apply area**, and play again. This resets the mission at the new boundary while keeping its seed. Small or narrow spaces naturally have fewer rooms and encounters. The rectangle includes the rover's modeled footprint.
7. Download the mission report. It records decisions, measured observations, actor checks, the route, and the outcome. Completed-run ground-truth diagnostics are explicitly separated from the rover's observations.
8. For the real-camera workflow, open **3D scan**. Select Local service or Joined Raspberry Pi. That viewer is a separate camera-relative depth source, not the moving simulation.

The header has a fullscreen control. Use 1× to explain decisions, or 2×/4× to shorten a run. A blocked mission is a valid reported failure; a random mission is not guaranteed to end in successful return.

## What is actually being scanned

The world has continuous floor, wall, box and cylinder surfaces. A synthetic forward camera sweep casts 1,536 rays (96 horizontal samples × 16 vertical samples) from the rover's camera position. Its horizontal field of view is 87° and its vertical field of view is 80°. First-hit intersections enforce occlusion. Neighboring returns on the same physical surface create triangles; missing returns and discontinuities are not bridged. A bounded cache retains observed patches as the rover moves, so earlier surfaces remain visible; a new heading or revisit acquires a new forward view. This replaces the previous visualization of inflated occupancy cells as cubes.

The sensor is a deliberately simplified forward camera and uses exact simulated poses. The surface cache is bounded, so it is not a lossless or complete house reconstruction. The separate 2D map drives frontier exploration and route planning. Rendering a measured synthetic surface does not validate physical camera accuracy, localization, vehicle dynamics, heat, smoke or combustion. The optional [physics test environments](professional-test-environments.md) remain separate integration fixtures.

The live hardware depth viewer still triangulates one aligned depth frame. It needs trustworthy camera poses before frames can be accumulated into a physical house map. See [behavior details](presentation-behaviors.md) for simulation assumptions and decision transitions.

## Wireless Pi stream and supervised control

For copy-and-install commands using the small Pi ZIP, start with [Pi quickstart](pi-quickstart.md).

The physical connections are **D435i → Pi USB 3**, **Uno → Pi USB**, and **independent power → Pi/rover**. The laptop connects by Wi-Fi. There is no laptop-to-rover USB cable in this arrangement.

1. Copy the project directory to the Pi. Follow [the Pi setup guide](raspberry-pi.md) or run `bash scripts/prepare_pi.sh`; check its output for RealSense binding availability.
2. Put the Pi and laptop on the same Wi-Fi/router or phone hotspot. Guest-network client isolation can prevent them communicating even if both have Internet access.
3. On the Pi, run `bash scripts/start_pi_camera.sh`. It starts camera-only operation and prints its addresses and a fresh access token. `hostname -I` also lists Pi addresses.
4. On the laptop, open **START_PRESENTATION.cmd → Connect Raspberry Pi**. Enter an address such as `http://192.168.1.42:8000` and the token printed by the Pi.
5. Click **Join live stream**, then **Open 3D firefighter view**. The Operator console link provides the separate supervised controls.
6. Start **START_AI.cmd** and choose the joined Pi. The laptop's GPU pulls RGB frames and sends person/fire detections back with the original frame IDs. The Pi associates them with matching depth; expired detections are rejected.

The join flow stores the Pi token in laptop-server memory only. It never falls back to a synthetic feed on disconnection. Disconnecting asks the Pi to disarm; if the network is broken, the Pi/Uno command timeouts remain the stopping mechanism. The laptop gateway forwards only the specified rover API endpoints. Keep this prototype on a trusted local network; it uses HTTP and is not an Internet service.

Camera-only mode cannot move motors. Wireless driving requires the inspected V4 wiring, verified firmware and passed raised-wheel checks described in [Windows hardware preparation](windows-hardware.md). Once enabled, the live dashboard provides supervised hold-to-drive controls. Loss of an optional AI worker does not supply a movement command.

## Real person and candle presentation

The default local checkpoints are `models/yolo11n-pose.pt` and `models/fire.pt`. **START_AI.cmd** runs both and selects CUDA GPU 0 automatically when available, otherwise CPU. Defaults are image size 640, person confidence 0.45, and fire/smoke confidence 0.35. The worker requires existing local model files and normally refuses synthetic sources; a deliberate `--allow-demo` transport check is separate from a live-camera demonstration.

Both models ran on this laptop's RTX 4060 using public reference images. Over 20 runs after warmup, median local inference was 10.397 ms for person/pose and 8.894 ms for fire/smoke. See [model provenance and measured GPU results](model-provenance.md) for exact settings and p95 values. These timings exclude camera capture, network, Pi and browser work; the sample images are not a held-out accuracy evaluation. Live D435i, candle and Wi-Fi performance remain to be measured.

Start stationary: verify a no-person/no-flame view, stand in the camera view, and check that the person box/depth observation appears. For an approved candle demonstration, use a stable nonflammable holder, keep the rover stationary with motors disarmed, and keep heat away from the camera, battery and electronics. Compare lit and unlit views at the actual planned distance and lighting. Record misses and false positives, including lamps and orange objects. Do not assume a tiny flame will be detected because a larger fire was recognized in a dataset image.

Model scores are not calibrated probabilities. A horizontal pose or lack of visible movement does not establish unconsciousness. Assistance and follower decisions require additional evidence; scripted scenario inputs must remain labeled as such.

## What is operational now, and what requires commissioning

| Capability | Current status |
| --- | --- |
| Laptop 3D mission theater | Procedural missions, observed ray-cast mesh, growing hazards, actor response checks, timeline and report; no motors used |
| Colored 3D surface mesh | Current camera-relative depth frame; missing depth remains holes |
| Person/fire model inference on laptop GPU | Installed and exercised on public sample images |
| Wi-Fi join, frame forwarding, supervised controls | Implemented; actual Pi network/hardware trial still needed |
| Uno firmware | Compiled; no device flashed during laptop preparation |
| Physical independent search, metered geofence, return and escort | Not implemented for hardware; localization, persistent mapping, path following and follower observation remain deployment work |
| Full house 3D reconstruction | Not implemented; camera-relative meshes cannot be merged without trustworthy poses |
| Alerting outside responders | Scenario/dashboard event and downloadable report; no automated emergency-service calls or messages |

The D435i faces forward. A person following behind the rover is outside that camera's view during forward driving. Reliable continuous following checks need a rear/rotating sensor arrangement or a validated stop-and-look-back behavior. Neither is supplied by the present mechanical kit. This limitation must be addressed before promising autonomous escort.

## Troubleshooting during the presentation

- **Token confusion:** use START_PRESENTATION.cmd again; it opens a signed-in page for its own running session.
- **Pi unreachable:** verify its IP, same Wi-Fi and `--host 0.0.0.0`. Try a private hotspot if the venue isolates clients.
- **Missing camera/Uno:** run hardware_preflight.cmd. It lists devices without driving or flashing them; Bluetooth COM ports are not an Uno.
- **AI not started:** use START_AI.cmd after connecting a hardware stream. Simulation does not need the models.
- **Stale depth:** the mesh is marked stale, and the rover gates inhibit motion. Restore the stream before proceeding.
- **Boundary slider:** it configures the simulated mission only. It cannot establish a physical metre fence without localization.
- **Unexpected layout:** the selected seed controls randomization. Use **Load seed**, then **Replay**, with the same boundary to reproduce a mission.
- **3D unavailable:** use a WebGL-enabled browser with hardware acceleration. The theater does not replace a failed 3D renderer with a misleading flat animation.

Keep the report for a failed run. Repeatability includes explaining why a mission stopped, not only showing successful runs.
