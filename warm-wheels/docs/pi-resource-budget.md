# Pi 4 / 4 GB operational target

Think of the deployment as a small floor robot: a compact 2D map, an estimate of its position, a search/return state machine, and continuous obstacle stopping. Detailed 3D rendering is a dashboard feature, not a prerequisite for navigation. The current implementation has the camera/motor/reporting parts and a simulated mission; real persistent mapping, localization and waypoint following are not yet connected.

## Keep on the Pi

- Camera acquisition, aligned depth and timestamp checks.
- Local obstacle checks and every motor permission decision.
- Serial acknowledgements and the operator-command lease.
- Eventually: a compact persistent map, localization health, exploration targets and return-home logic. This remains integration work.

The Uno independently handles PWM and its own watchdog. A stalled Pi must not continue driving. The phone/PC may display telemetry and supply optional detections; loss of optional AI must not become invented free space or an excuse to bypass motor health checks. The current manual-drive mode stops on network command loss, rather than attempting an unlocalized return.

## Existing bounded defaults

| Workload | Current setting | Implication |
|---|---|---|
| RGB/depth capture | 640×480, 15 fps requested | Actual negotiated rate and processing speed must be measured on Pi |
| Retained RGB/depth frames | At most 24 | About 49 MiB for raw RGB uint8 + depth float32 arrays, plus Python/processing overhead |
| Local map | 60×60 at 0.10 m/cell | Small camera-relative scan; not a persistent house map |
| Point cloud | At most 1,200 points per snapshot | Bounded visualization payload |
| Surface mesh | Stride 8 over the requested 640×480 depth frame: at most 80×60 = 4,800 sampled vertices | Invalid samples are removed; triangles cannot bridge invalid depth or large depth discontinuities |
| Mesh generation cache | 200 ms minimum recomputation interval | Bounds repeated mesh work; does not guarantee Pi processing can sustain 5 meshes/s |
| 3D scan viewer | 250 ms mesh poll interval, nominally 4 Hz, one request at a time | Browser renders/orbits locally; polling pauses while hidden or frozen; actual delivered rate depends on the Pi/network |
| Operator dashboard polling | About 2 Hz per visible client | Separate from the scan viewer; use one active control console |
| Motor loop | 75 ms wait plus ACK/lock work | Host timing is not a real-time guarantee |
| Person/pose and fire/smoke models | Both on the laptop worker by default; automatic CUDA if available | No PyTorch model required in the Pi service |

A 10×10 m persistent grid at 10 cm resolution needs 10,000 cells. Storing one signed byte per cell would use roughly 10 KB, before map metadata and implementation overhead. Localization reliability, perception and camera throughput—not the cell count—are the main unresolved deployment questions.

The mesh is one camera-local surface, not accumulated SLAM. A fully valid 80×60 sample grid could form at most 9,322 triangles before discontinuity checks; actual geometry often contains fewer faces and deliberate holes. Mesh generation also inspects full-resolution depth neighborhoods, so 4,800 output samples do not mean only 4,800 source pixels are processed. JSON encoding, transfer and retained-frame memory add overhead.

Three.js rendering runs in the laptop/phone browser, not on the headless Pi. The scan viewer retains a visibly stale surface when the source fails and supports freezing for inspection. Freezing a view only stops that browser's mesh requests; it is not a rover stop command. **Pi mesh throughput and combined camera/mesh/network load have not been measured on the user's hardware.**

## Measure before optimizing

Run the hardware bench report with motors disconnected, then run a separate service trial with the 3D scan viewer open. The bench and service must not compete for the same camera. Record actual capture and mesh delivery rates, maximum capture gap, mesh request/encoding time, valid depth fraction, CPU, memory, temperature/undervoltage indicators where available, and network delay. The included bench tool measures frame/depth/serial properties; use the Pi's normal OS monitoring tools and timestamped service/browser logs for the additional measurements. Do not interpret laptop GPU inference or desktop mesh timings as Pi results.

If frame freshness misses the 450 ms gate, reduce visualization or unnecessary image copies first. Keep the stop gate intact. Confirm every resolution/FPS change is supported by the camera and that intrinsics remain correct. Reducing model image size or detection rate can help optional AI, but it changes accuracy and must be evaluated on held-out examples.

For an all-Pi model, first validate the exact ARM/Python inference runtime and measure memory/latency alongside camera capture. Do not install an entire simulator or large training stack on the Pi to run the rover. Model export/quantization is a separate optimization step; no all-Pi person/fire inference benchmark is claimed in this deliverable.
