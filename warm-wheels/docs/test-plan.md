# W.A.R.M wheels verification and test plan

This plan covers the implemented prototype and its separate synthetic search mission. All **HIL, powered bench, dataset, and field cases below are NOT RUN on the user's hardware**. Software evidence already recorded in [validation.md](validation.md) applies only to its stated development environment. Do not convert a planned case into a pass without the case's evidence.

Use [engineering-guidelines.md](engineering-guidelines.md) for requirements, baseline recording, timing interpretation, and stop-distance calibration. Use [hardware.md](hardware.md) for electrical bring-up. The current code does not implement autonomous physical mapping, search, return, or escort.

## Test environments and entry gates

| Layer | Environment | Entry condition | Exit evidence |
| --- | --- | --- | --- |
| SIL | Development PC, `SimulationMotor`, synthetic camera/mission, automated tests | Exact source/dependency baseline saved; no hardware serial port supplied | Test output, scenario artifact, configuration and fixture/seed identity |
| Optional physics | Supplied Gazebo SDF worlds on PC; no serial bridge to the car | Install/verify engine; add and review controller/sensor adapters before integrated tests | World/controller hashes, sensor/dynamics settings, event trace and replay |
| HIL | Pi 4 4 GB + D435i; optional Uno USB with motor power disconnected | Wiring inspected; no competing serial owner | Probe JSON, device versions, alignment/range observations and fault logs |
| Raised-wheel bench | Secured V4 car with every wheel clear, correct firmware, accessible motor-power disconnect | HIL passed for the exact assembly; operator and spotter prepared | Serial/electrical trace, video, direction and stop results |
| Mock-room field | Level bounded floor, foam/cardboard obstacles, marked stand-off distances, no flame/smoke | Raised-wheel faults passed; stopping and swept-envelope checks planned with conservative space | Layout, videos, distances/times, operator log, findings |

The historical empty-fixture mission uses ideal 360° occupancy sensing and discrete cell movement. The current procedural theater's 3D scanner uses a forward camera field of view and persists acquired mesh patches. Its floorplan and camera scene remain separate examples. The [professional test environments guide](professional-test-environments.md) describes the supplied Gazebo Harmonic worlds and missing adapters. Those worlds have structural XML checks only; no engine run is claimed. A physics test must state its D435i field of view, noise/dropout assumptions, rover dimensions, friction, wheel response, and localization assumptions. Passing SIL does not validate those parameters.

## Commands and evidence collection

Run these from the project root in the appropriate virtual environment:

```sh
python -m pip install -r requirements-dev.txt
python -m pytest -q
python -m validation.run_scenarios --output reports/simulation --seed 0
python -m bench.check_hardware --mode demo --duration 10 --output captures/probe-demo.json
```

Use a fresh, timestamped output directory for each campaign. The scenario runner emits `results.json` and `results.csv`; JSON includes the baseline ID, source hashes, environment, seed, per-check actual/expected values, and summary. Exit 0 means all selected software scenarios and any requested baseline comparison passed; exit 1 means a failed check/exception/comparison and exit 2 means invalid arguments. Version 1 records the seed but does not use randomness: its fixtures and 0.1 s logical clock are deterministic. A repeatable `--scenario ID` selects a subset. The eight IDs are:

`empty_search_return`, `operator_early_return`, `blocked_launch`, `exploration_step_budget`, `depth_data_holes`, `dropped_data`, `changed_obstacle_replan`, and `cluttered_static_fixture`.

Add `--baseline PATH/results.json` for exact regression comparison of source hashes, baseline/schema IDs, seed, and selected scenario results. Environment and generated time are recorded but excluded from exact equality. A changed source hash intentionally fails this comparison even if all scenario checks pass; review the change before adopting a new baseline. Inspect `summary.all_passed` and `baseline_comparison`, not only the scenario pass count.

Save the exact command and output; never infer a pass from the existence of a file. `captures/` contains local test artifacts, not production telemetry retention. Bench tools require a new JSON path under `captures/` and refuse to overwrite an existing report. Their exit codes are 0 for the tool's limited checks passing, 1 for failure, and 2 for blocked/invalid input. Tool success does not substitute for the visual/electrical evidence required by a hardware case.

For real camera HIL, keep motor power disconnected and omit serial initially:

```sh
python -m bench.check_hardware --mode hardware --duration 10 --output captures/probe-camera.json
```

Only after identifying the device and closing the rover service/Serial Monitor, include the actual serial path:

```sh
python -m bench.check_hardware --mode hardware --duration 10 \
  --serial /dev/serial/by-id/ACTUAL_DEVICE --output captures/probe-camera-uno.json
```

Opening USB can reset the Uno. This probe verifies the profile and sends acknowledged stop commands; it does not send a nonzero motor command or reset an emergency latch. An unverified profile is an expected commissioning block, not evidence of working motor control.

After HIL and wiring checks, the following is a **powered, raised-wheel-only** left-side direction pulse, not a floor-driving command:

```sh
python -m bench.motor_pulse --port /dev/serial/by-id/ACTUAL_DEVICE \
  --wheels-raised --confirm-pin-check --left 15 --right 0 --duration 0.15 \
  --output captures/pulse-left.json
```

Repeat with left zero/right 15 only after reviewing the first result. The tool permits 0–20 PWM and at most 0.25 seconds, sends one drive request, and stops/closes afterward; it does not perpetually refresh or automatically reset. A nonrotating wheel may reflect deadband rather than success. Do not increase power on the floor to diagnose it. CLI confirmations document the operator's statement; they cannot physically verify the stand or wiring.

## Software and API matrix

Status **BASELINE EXISTS / RERUN** means related automated coverage exists in the repository, not that this particular new release has been executed under this plan.

| Case | Setup | Stimulus | Measurement | Pass criterion | Required evidence | Initial status |
| --- | --- | --- | --- | --- | --- | --- |
| SIL-01 | Demo runtime, simulated motor | Start/finish empty mission; request local route | Nonzero motor-call count; mission phase/home | Simulation explores independently of detections and returns to its home; preview/mission issues zero nonzero motor requests | Test log; mission trace and fixture identity | BASELINE EXISTS / RERUN |
| SIL-02 | `SafetyGate` and API fixtures | Expire frame/command; missing depth; low clearance; invalid/reverse/tight-turn command | Output tuple, arm/estop transitions | Faults produce zero; environmental faults disarm; invalid commands rejected; reset never arms | Parametrized test output and input values | BASELINE EXISTS / RERUN |
| SIL-03 | Known grids and depth arrays | Insert unknown barrier, narrow gap, near obstacle, invalid values or depth hole | Occupancy cells and planned path | No unknown traversal/corner cutting; footprint cannot fit through insufficient space; no invented clearance | Array fixture and assertions | BASELINE EXISTS / RERUN |
| SIL-04 | Fake serial device and runtime | Bad/missing/partial ACK, wrong sequence, disconnect, firmware reset | Port state, errors, stop attempts, persistent fault | No false ACK success; disarm/error persists; cleanup runs; no automatic reset/replay | Serial transcript and test output | BASELINE EXISTS / RERUN |
| SIL-05 | Frame cache and observation API | Submit mismatched/expired source frame and manual assessment | Accepted/rejected response; source IDs; note contents | >1.2 s old result rejected; depth stays with original frame; manual notes remain reports, not identity/medical inference | API test output and sample sanitized state | BASELINE EXISTS / RERUN |
| SIL-06 | Empty mission; blocked home fixture | No detections; early return; exhausted budget; block known return | Position/phase/coverage and output wording | Exploration does not require a detection; blocked home gives `blocked`; `complete` means home, never “whole place clear” | Scenario trace, screenshots and fixture hash | BASELINE EXISTS / RERUN |
| SIL-07 | Authenticated API | Missing/wrong token; valid token; hardware mission start | HTTP statuses and motor-call count | Unauthorized image/control access rejected; hardware mission start rejected; no motion from rejected call | API test output with tokens removed | BASELINE EXISTS / RERUN |
| UI-01 | Demo dashboard, desktop and phone width | Hold/release, cancel pointer, hide tab, lose connection, estop/reset | Command trace and visible controls | Refresh stops on release/cancel/hidden/fault; no stale rearm; explicit mode/source; no overflow | Browser/network trace and screenshots | BASELINE EXISTS / RERUN |
| UI-02 | Delayed remote observations and newer color frames | Cause source-frame mismatch | Rendered boxes, local dots, card ages | Boxes only on matching JPEG frame; local dots only on matching scan; old observations retain age/source labels | Frame IDs and screenshot pair | PLANNED / NOT RUN |

## Optional physics matrix

The supplied worlds are `empty.sdf`, `blocked-door.sdf`, `obstacle-course.sdf`, and `visual-distractor.sdf`, generated by `sim/gazebo/generate_worlds.py`. These cases still require a running engine and an integrated controller/sensor adapter. A simulated scenario must never open the physical serial port. Their initial status does not imply that a simulator is installed or an adapter exists.

| Case | Setup | Stimulus | Measurement | Pass criterion | Required evidence | Initial status |
| --- | --- | --- | --- | --- | --- | --- |
| PHY-01 | Versioned world with measured rover dimensions and bounded camera view | Replay identical controls/seed | Trajectory, sensor outputs, collisions | Replay is reproducible within a declared tolerance; all sensor/dynamics simplifications listed | World/controller/config hashes and two traces | OPTIONAL / NOT RUN |
| PHY-02 | Modeled delay, friction, occlusion and depth dropout | Stop commands; obstacles at view edge; camera loss | Actuation cutoff, swept envelope, unknown cells | Commands stop on fault; unseen space stays unknown; modeled collisions/failures are retained | Seeded fault trace and replay | OPTIONAL / NOT RUN |
| PHY-03 | Localization/navigation integration, if later implemented | Block route home; inject pose drift | Estimated/true pose and mission decisions | Stops on lost localization or blocked return; never declares a verified exit without evidence | Integration version and failure trace | FUTURE / NOT IMPLEMENTED |

## Camera and serial HIL matrix — motor power disconnected

| Case | Setup | Stimulus | Measurement | Pass criterion | Required evidence | Initial status |
| --- | --- | --- | --- | --- | --- | --- |
| HIL-01 | Actual Pi OS/Python, D435i on USB 3, camera-only launch and controlled matte target scene | Acquire for 10 s, then a 15 min runtime stability log or repeated probe campaign (each probe is limited to 60 s) | Frame count/rate/age, valid depth, USB/power errors | Hardware frames only; at least 2 frames, at least 10 FPS, maximum gap/delivery age at most 0.45 s, mean valid depth at least 85%; unavailable capture is FAIL/BLOCKED; all errors recorded | Probe JSON, versions, USB tree and extended log | NOT RUN — HARDWARE |
| HIL-02 | Level camera, measured targets at multiple known ranges | Place matte target, reflective/dark target, narrow obstacle; partially cover camera | Alignment, range residuals, validity, local map | RGB/depth association verified; invalid depth remains unknown; measured range uncertainty recorded; any false-clear result blocks floor testing | Target distances, sample frames, residual table | NOT RUN — HARDWARE |
| HIL-03 | Uno USB, correct board inspected, motors unpowered | Unverified then verified profile; open/reconnect; competing-owner check | Handshake, acknowledged stop, firmware reset behavior | Unverified rejected; verified connection stops before use; no automatic arm; one port owner | Sketch hash, profile/pins, probe JSON and serial log | NOT RUN — HARDWARE |
| HIL-04 | Real camera and motor service, motors unpowered | Disconnect/occlude camera; halt capture; interrupt USB or detector | Gate state, fault persistence, frame age and output | Unknown/stale camera blocks motion and disarms; fresh frames do not erase motor faults; detector messages never refresh the operator motor lease | Timestamped status/serial logs | NOT RUN — HARDWARE |
| HIL-05 | Pi camera, dashboard and PC detector under combined load | Sustained inference, network delay/loss and CPU load | Inter-`M` gaps, ACK latency, frame age, restart behavior | Recorded tail timing fits the watchdog reserve; missed deadlines cause visible stops, no automatic recovery/replay | Common-clock timing trace and load parameters | NOT RUN — HARDWARE |

Camera acquisition timestamps are currently host monotonic times after frame retrieval. Measure exposure-to-display delay separately with a visible timer/LED stimulus; frame age alone does not establish full sensing latency. USB/power stability and D435i profile availability are platform tests, not assumptions.

## Raised-wheel and stopping matrix

Stop after any unexpected direction, uncontrolled movement, missing latch, or inability to disconnect motor power. Correct the cause before proceeding. An acknowledged pulse proves communication, not wheel direction or stop timing.

| Case | Setup | Stimulus | Measurement | Pass criterion | Required evidence | Initial status |
| --- | --- | --- | --- | --- | --- | --- |
| BENCH-01 | All wheels raised; verified wiring; guarded stand | Boot, arm without drive, reset/reconnect | Wheel movement and PWM/enable | Zero drive at every startup/reset; no recalled movement | Continuous video plus serial/electrical trace | NOT RUN — HARDWARE |
| BENCH-02 | Same, low bounded pulse | Single left pulse; inspect; single right pulse | Side, direction, pulse/output duration | Only selected side moves forward; stop follows; no auto reset; deadband documented | Pulse JSON, wiring/stand photo and video | NOT RUN — HARDWARE |
| BENCH-03 | Same, explicit manual fault/reset sequence | Emergency stop; `S`; then explicit `R`; malformed/partial command | Electrical output and latch replies | `E` and malformed input remove drive/latch; `S` cannot clear latch; `R` leaves zero and application disarmed | Serial and electrical trace | NOT RUN — HARDWARE |
| BENCH-04 | Same, time-synchronized serial/PWM/video capture | One valid drive; stop refreshing; separately terminate Pi/unplug serial | Last accepted `M` to PWM-off; coast to wheel stop | PWM removed at 350 ms deadline within preregistered instrument/loop tolerance; watchdog latches; wheel-coast time measured separately | Analyzer capture and synchronized video; repeated trials | NOT RUN — HARDWARE |
| BENCH-05 | Same, real service with camera and browser | Release/blur/hide controls; expire camera; delay ACK | Stimulus-to-output-zero and wheel standstill | Every fault produces zero and required latch/disarm; measured bounds recorded with no claim of universal 350 ms end-to-end stop | At least 10 repetitions per fault, timing table | NOT RUN — HARDWARE |

For BENCH-04, do not use the ordinary pulse tool as the only watchdog test: it intentionally sends a stop. A separately reviewed single-command fault-injection procedure is needed to measure command expiration. Never send a stream that conceals the watchdog condition. Changing the browser token or losing Wi-Fi is not equivalent to loss of serial refresh at the Uno.

## Perception evaluation matrix

Freeze scene/session-separated train, validation, and held-out test manifests before evaluating a model. Record model hash, inference parameters, labels, split counts, frame rate, and confidence threshold. Choose operational accuracy targets before revealing test labels; the project currently has no validated detection-performance target or custom fire weights.

| Case | Setup | Stimulus | Measurement | Pass criterion | Required evidence | Initial status |
| --- | --- | --- | --- | --- | --- | --- |
| DATA-01 | Held-out empty/no-person rooms and clutter | Negative video; orange clothes, lamps, screens, reflections | Person/fire-cue false positives per image/minute; false-clear statements | Metrics reported against preregistered targets; zero unsupported “room clear”/verified-fire statements | Split manifest, annotations, per-scene results and examples | NOT RUN — DATASET |
| DATA-02 | Consenting adult or mannequin scenes across sessions | Upright/seated/occluded/horizontal figures at varied ranges/light | Person precision/recall; misses by condition; posture outputs | Frozen matching/accuracy targets met; occluded/unknown preserved; no automatic age/consciousness/evacuation-ability claim | Held-out report with failure montage and counts | NOT RUN — DATASET |
| DATA-03 | Matched recorded RGB/depth and delayed inference | Delay, duplicate/out-of-order source frame, partial depth | Source-ID matching, rejection age, position error/availability | Results older than 1.2 s rejected; no depth from wrong frame; invalid depth unlocated; error distribution reported | Input frame IDs, worker log and annotated range results | NOT RUN — DATASET |
| DATA-04 | Team-supplied fire/smoke model, safe recorded material | Held-out positive and hard-negative scenes | Class precision/recall, negative-video false alarms, latency | Preregistered targets met for stated scene envelope; labeled candidate/unvalidated until evidence reviewed | Weights hash, license/source, split and evaluation report | NOT RUN — MODEL NOT BUNDLED |

Recorded/synthetic fire imagery evaluates appearance only. Do not use flame, smoke generators, combustion, or dangerous human staging for the mock-room tests. A pose cue is not a diagnosis; manual assistance/child reports remain explicit operator notes.

## Controlled mock-room field matrix

This stage is supervised manual driving in a normal indoor space. Start below the maximum allowed command, with a clear escape space for the rover and a spotter at the power disconnect. Mark the camera-to-bumper offset and target stand-off on the floor. No person is used as a collision target.

| Case | Setup | Stimulus | Measurement | Pass criterion | Required evidence | Initial status |
| --- | --- | --- | --- | --- | --- | --- |
| FIELD-01 | Marked level course, foam obstacles, measured geometry | Straight movement then gentle arcs | Swept envelope, depth blind spots, contacts | No contact; operator remains in sight; measured envelope fits the cleared area; any unobserved side risk stops progression | Overhead layout/video and dimensions | NOT RUN — HARDWARE |
| FIELD-02 | Large clear stand-off area; varied approved battery/load/surface | Release, estop, command loss at staged PWM up to allowed max | Reaction delay, speed, travel/coast, uncertainty | Measured required clearance plus reserve fits inside the calibrated threshold; default 0.65 m not assumed adequate | At least 10 repetitions per condition; maximum travel and uncertainty | NOT RUN — HARDWARE |
| FIELD-03 | Mock room with no person/fire and separate staged observations | Manual scan; partial occlusion; camera view blocked | Observed/unknown regions, observation age and operator report | Reports remain local/incomplete; empty view never implies whole-place clearance; no automatic medical/age claim | Screen/video capture and signed observation sheet | NOT RUN — HARDWARE |
| FIELD-04 | Hardware mode, stationary rover | Attempt Search mission and route preview | API response and physical motion | Autonomous mission rejected; preview commands no movement; operator can manually return only through supervised clear space | API trace, video and state log | NOT RUN — HARDWARE |

The 0.65 m threshold is in `rover/safety.py`. If calibration fails, stop floor trials and revise limits through a reviewed change; do not treat a failed test as permission to bypass the gate. Recheck after changing payload, tires, battery, surface, geometry, or firmware.

## Test report and failure record template

Copy this template for each case. Attach evidence rather than replacing measurements with “worked.” Use `PASS`, `FAIL`, `BLOCKED`, or `NOT RUN`.

```text
Run ID / UTC date:
Case ID / requirement IDs:
Status / operator / spotter / reviewer:
Source commit + dirty diff OR source SHA-256 manifest:
Exact OS, Python/dependency versions, Pi/PC hardware:
Camera serial/firmware/librealsense/profile:
Uno/core version, sketch hash, motor profile and verification flags:
Model hashes / inference parameters / dataset split manifest:
Fixture/world/controller hashes / exact seed (or not applicable):
Battery/load/surface/lighting; measured dimensions and calibration:
Preconditions and exact command line (token removed):
Stimulus and step-by-step reproduction:
Expected result / acceptance limit / declared instrument tolerance:
Actual measurements (units, count, max, percentiles, uncertainty):
Fault timestamp and event ordering:
Evidence paths and SHA-256 hashes (JSON, logs, images, video, analyzer trace):
Failure impact / smallest reproduction / issue ID:
Correction and affected regression cases:
Rerun ID (original evidence preserved):
Unresolved limitations / decision for next stage:
```

A final campaign summary lists every case and its evidence path, including blocked and unrun cases. Do not state “hardware tested,” “autonomous rescue ready,” or “certified” on the strength of compilation, a successful synthetic mission, or this written plan.
