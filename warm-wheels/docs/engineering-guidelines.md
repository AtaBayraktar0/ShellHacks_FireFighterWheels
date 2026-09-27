# W.A.R.M wheels engineering guidelines

This guide defines the working practices and evidence needed for the ELEGOO V4 / Pi 4 / D435i prototype. It is an engineering verification plan, not certification, a fire-service operating procedure, or authorization for use in an active fire. The current physical system supports supervised forward movement and local observation. Autonomous physical search, persistent house mapping, return-home, and escort are not implemented.

## 1. Preserve the distinction between observation and knowledge

| Requirement | Engineering rule | Verification reference |
| --- | --- | --- |
| REQ-01 | Boot, reconnect, fault recovery, and emergency reset must not resume a previous drive. Reset and arm are separate operator actions. | SIL-02, HIL-03, BENCH-03 |
| REQ-02 | Motion needs fresh depth, a fresh operator command, a healthy motor link, an armed gate, and bounded forward commands. Unknown space cannot be treated as free. | SIL-02, SIL-03, HIL-04 |
| REQ-03 | A lost command source must stop refreshing motor drive. The Uno independently expires a drive command; physical stopping distance must be measured. | SIL-04, BENCH-04, FIELD-02 |
| REQ-04 | Detection boxes and depth must refer to the same source frame. An old observation must not be projected onto a newer image or local map. | SIL-05, DATA-03 |
| REQ-05 | Person, posture, and fire-candidate outputs are observations. Age, consciousness, injury, and ability to evacuate are not model conclusions. Assistance notes are operator reports tied to an observation. | SIL-05, DATA-01, DATA-02 |
| REQ-06 | Search-mission simulation and camera demonstrations remain explicitly synthetic. No simulation route may command the physical car. | SIL-01, SIL-06, FIELD-04 |
| REQ-07 | No detection, a blank room image, or exhausted exploration frontiers must never produce “whole place clear,” “no trapped people,” or “safe exit.” Report observed areas, unknown areas, and limitations. | SIL-06, DATA-01, FIELD-03 |
| REQ-08 | Images, observations, and motor APIs require authentication. Use one operator and one serial-port owner. | SIL-07, HIL-03 |

Case definitions and evidence requirements are in [test-plan.md](test-plan.md). Existing development results remain in [validation.md](validation.md); a planned case is not a passed case.

## 2. Establish a reproducible baseline

Before a run, record the source commit and dirty diff, or a SHA-256 manifest when the delivery has no Git metadata. Record exact Python/package versions, OS/kernel/architecture, Pi RAM, librealsense and camera firmware versions, camera serial number, Uno board/core version, compiled sketch hash, selected motor profile, verification/inversion settings, and attached payload. Record battery state, surface, lighting, ambient conditions, operator, spotter, and test fixture dimensions.

Keep evidence under a new run directory such as `captures/2026-09-27T140000Z_bench04/`. Save unedited command output, configuration, timestamps, fixture photographs, and relevant recordings. Do not include access tokens. Preserve failed runs; reruns get a new ID and reference the original failure. For deterministic fixtures record the fixture/version and `seed: not applicable`; for randomized scenarios record the exact seed, generator version, and resulting fixture hash.

Use the pinned versions recorded for the tested environment as evidence of that environment, not as a claim that the same wheels install on Pi/ARM. Keep model weights and their SHA-256 hashes separate from application versions. Model names alone are insufficient because a file can change under the same name.

## 3. Measure the actual platform before enabling movement

Follow [hardware.md](hardware.md) for the V4 pin map, independent power, USB wiring, and disabled firmware. The shipped `HARDWARE_VERIFIED 0` is deliberate. The `VERIFIED` handshake describes an operator-selected build profile; it does not discover wiring or certify the assembly.

Measure camera height and level mounting, loaded rover width, loaded height, forward camera-to-bumper offset, and the swept envelope of a gentle arc. The runtime accepts `--camera-height`, `--robot-half-width`, and `--obstacle-height`; its defaults are 0.22 m, 0.20 m, and 0.45 m. These are starting values, not measurements of your car. There is no implemented tilt compensation or odometry-based localization.

The local planner uses its own footprint default of 0.18 m radius in `rover/spatial.py`. That value is separate from the forward-depth corridor half-width. Review and document both against the loaded robot; changing one does not calibrate the other. The planner is currently a preview, not a physical navigation controller.

Establish motor direction with all wheels supported clear of the table and floor. Keep a reachable motor-power disconnect and a second person available for powered floor tests. A serial-port open commonly resets an Uno. Stop the rover service and close Serial Monitor before a bench tool owns the port.

`bench.check_hardware` defaults to a synthetic probe with no serial access. Explicit hardware mode starts the real camera. Supplying `--serial` opens the Uno, verifies the handshake, and issues acknowledged stop commands; that option is active communication and may reset the board. It never requests nonzero motor drive. `bench.motor_pulse` requires the raised-wheel and pin-check flags, permits a single bounded forward pulse, and does not clear a latch. See the test plan for the staged procedure.

## 4. Treat timing limits as separate contracts

Current source defaults are shown below. These values are not measurements of end-to-end stopping performance.

| Mechanism | Configured value | What it means |
| --- | --- | --- |
| Dashboard held command | About 120 ms refresh | Browser sends only while a direction is held; scheduling is not real time. |
| Operator command lease | 300 ms | A motor-gate evaluation rejects an expired operator command. |
| Motor-loop wait | 75 ms after work | Actual period includes serial I/O, locks, processing, and OS scheduling. |
| Serial ACK deadline | 150 ms | Missing or incorrect acknowledgements fault the link; this is not a wheel-stop measurement. |
| Uno drive watchdog | 350 ms | Time since the last valid `M` command before firmware removes drive and latches a fault. |
| Partial serial line | 100 ms | An incomplete command faults and stops the firmware. |
| Camera freshness gate | 450 ms | Host acquisition age beyond this value blocks motion. It is not sensor exposure age. |
| Remote inference expiry | 1,200 ms | Older source-frame results are rejected; inference has no permission to sustain motor drive. |

The **350 ms watchdog budget begins at the Uno's last accepted drive**, not at browser release, Wi-Fi loss, or an obstacle appearing. A running Pi may refresh the Uno until the 300 ms operator lease expires. Camera loss has a different path through the 450 ms freshness gate. Do not advertise any of these as a universal 350 ms stopping guarantee.

Measure four events on a common time base: last complete valid command, PWM/enable removal, last visible wheel rotation, and chassis standstill. A logic analyzer or oscilloscope resolves electrical cutoff; timestamped high-frame-rate video or an external motion sensor resolves wheel/chassis motion. An ACK or cached `left/right=0` is not physical feedback.

For watchdog acceptance, electrical cutoff must occur at the configured deadline within the declared instrument resolution and measured firmware-loop delay. Record those tolerances before the run. For ordinary operation, record maximum and percentile inter-command gaps under combined camera, inference, and network load. They must stay below 350 ms with an explicitly documented reserve; a missed refresh should result in a latched stop, never automatic reset. Tail latency matters more than the average.

## 5. Calibrate clearance using measured travel

`SafetyGate.stop_distance_m` is currently a **static 0.65 m threshold** in `rover/safety.py`; the application does not expose a stop-distance CLI flag. It blocks clearance strictly below that value. The default 25% command limit is PWM, not a known speed. A software stop removes drive and does not provide a mechanical brake.

For each allowed command, battery condition, payload, and approved test surface, measure worst observed travel after release, emergency stop, camera failure, and command loss. Include sensing/control delay, drivetrain inertia, wheel slip, range error, and the camera-to-front-bumper offset. A useful engineering check in camera-depth coordinates is:

`required clearance > bumper offset + speed × measured reaction delay + measured coast distance + measurement uncertainty + chosen reserve`

Avoid double-counting delay if the measured stopping distance already starts at the fault stimulus. Record how every term was measured. If the result does not fit inside the current 0.65 m buffer, do not proceed to floor demonstrations: reduce command limits, increase the threshold through a reviewed source change, improve sensing, and repeat the relevant tests. Do not lower depth-quality gates merely to make movement possible.

The current front-only camera cannot verify the full arc envelope, rear space, stair edges, or unseen floor. Use level, bounded test areas and modest forward arcs. Bumper offset and turning envelopes are not automatically compensated by the current gate.

## 6. Evaluate perception on held-out scenes

Maintain separate training, tuning/validation, and frozen test sets. Split by room/location and recording session; neighboring frames of one video belong to the same split. Deduplicate near-identical images across sets. Record capture device, lighting, distance, occlusion, motion blur, annotations, and model/threshold hashes. Keep test labels away from model selection.

The test set must include empty rooms and negative examples: orange/red clothing, furnishings, signs, screens displaying flames, warm lamps, reflections, and sunlight. Include standing, seated, partly occluded, and safely staged horizontal human figures or mannequins. Use consenting adults or synthetic material for human examples; no age classifier is present or required. No person should need to enter a hazardous scene or perform a fall for testing.

Report person precision/recall and false negatives by scene condition; report hazard-candidate false positives per image and per minute on negative video. Report matched-frame age, depth availability, and position error separately from class accuracy. State counts and denominators, and report failures rather than hiding them in a mean. The color heuristic's fixed score is not a calibrated fire probability. No trained fire/smoke model or dataset is bundled.

Before evaluating a custom model, freeze the intended-use targets, matching rule, thresholds, and allowable latency. Unspecified accuracy requirements are a release blocker for detection claims; do not retrospectively select favorable targets. Printed images and screen playback test visual behavior only, not response to real combustion, smoke, temperature, or a complete emergency scene.

## 7. Advance through evidence gates

Run software-in-the-loop (SIL), optional physics simulation, camera/serial hardware-in-the-loop (HIL), raised-wheel checks, then a supervised mock-room field trial. Each stage has an independent status. The [professional test environments guide](professional-test-environments.md) describes the supplied Gazebo Harmonic empty, blocked-door, obstacle-course, and visual-distractor SDF worlds. They have received structural checks only; the physics engine has not been run here and no dashboard/camera/mission adapter is supplied. Physics simulation is neither mandatory for the existing demo nor a substitute for HIL. Record the simulator version, world, controller, physics step, sensor model, and seed; do not quietly reuse the ideal mission sensor as a D435i model.

Stop progression on unexpected movement, direction mismatch, missing stop/latch, stale data represented as fresh, or loss of the physical disconnect. Keep the fault record, correct the cause, repeat the affected case, and rerun related regression cases. Changes to wiring, firmware, geometry, payload, model, power, or timing limits invalidate the relevant prior results.

The final test report should state exactly what was exercised and what remains untested. A passed mock-room trial supports that configuration and test envelope. It does not demonstrate autonomous evacuation, whole-house clearance, performance in smoke, or fire-service certification.

## 8. Keep the eventual autonomous system small and measurable

The intended Roomba-like architecture puts a persistent 2D occupancy map, exploration/return state, verified home pose, and bounded local navigation on the Pi; the Uno retains motor output and its watchdog. Keep optional PC perception separate from the ability to stop. Reduce dashboard point-cloud traffic before sacrificing control or camera deadlines.

This is a development target, not the current physical capability. The implemented Pi path builds fresh camera-local geometry and supervises manual commands. It does not yet estimate a persistent global pose, combine scans into a navigable house map, follow paths autonomously, or verify a physical return to launch. Add and validate those functions in small stages, with explicit pose-loss and blocked-return stops, before calling the rover operationally autonomous. A demo completing an empty synthetic floorplan does not close those gaps.
