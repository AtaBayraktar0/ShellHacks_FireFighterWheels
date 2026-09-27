# Validation record

Development host: Windows, Python 3.12.10. Final dependency versions are recorded in `tested-environment.txt`; that file describes this host, not a Pi/ARM lockfile.

## Completed

- Final automated suite: **331 passed and 61 subtests passed**. One dependency deprecation warning is retained in [test-results.txt](test-results.txt). Pytest collects the project `tests/` directory explicitly, so a separately extracted nested copy cannot create duplicate test imports.
- The current procedural campaign covers **12/12** seeded 6 × 6 m missions. It checks cardinal movement, post-step observed/truth clearance, actor separation, finite indexed accumulated mesh, bounded mesh memory, replay determinism and return reporting. Boundary runs remain bounded software trials across 1 × 1, 1 × 50, 50 × 1 and 50 × 50 m areas.
- All eight presentation scenarios passed across five seeded 6 x 6 m layouts: **40/40**. Boundary trials at 1 x 1, 1 x 50, 50 x 1 and 50 x 50 m produced **31 passed, 1 not applicable, 0 failed**. The unavailable case was the blocked-exit event in one narrow layout; it is not counted as a demonstrated blockage. See [randomized campaign](../reports/presentation-campaign/summary.json) and [boundary campaign](../reports/boundary-campaign/summary.json). Seeds, source hashes, events and trajectories are included. These are bounded software trials, not proof for every possible seed.
- Trained person/pose and fire/smoke models ran on the laptop RTX 4060 with CUDA. The public sample inference medians were about 10.4 ms and 8.9 ms respectively. [Model evidence](../reports/ai-smoke-test.json) is a functional sample check, not held-out accuracy or end-to-end camera latency.
- The workstation worker submitted **3/3** matching synthetic frames through the laptop Pi gateway, with no errors/discards. Switching the gateway rejected the previous generation with HTTP 409. [Transport evidence](../reports/ai-transport-smoke.json) uses loopback, not physical Wi-Fi. Regression tests also cover disconnect during connection, missing source generation, mesh observation refresh, stale camera frames and token redirect prevention.
- Eight deterministic scenarios passed, then passed an exact repeat against the frozen baseline. JSON/CSV evidence is in [the scenario report](../reports/simulation/results.json) and [repeat report](../reports/simulation-repeat/results.json). These exercise software behavior, not physical sensors or driving.
- The synthetic camera bench passed its capture/cleanup checks; [bench-demo.json](../reports/bench-demo.json) labels its source as demo. All 23 bench unit tests use injected devices and perform no physical movement.
- Four Gazebo world files were generated and checked for XML structure, joints, sensor configuration and distinct fixtures. The Gazebo engine was not run; these are starting fixtures awaiting controller/sensor integration.
- Python automated tests cover depth holes/invalid values/near obstacles, conservative occupancy and A*, the serial handshake and missing/wrong acknowledgements, emergency latches, stale camera/command gates, authenticated API access, frame-associated inference, manual observation notes, and pose-hint/depth serialization.
- Runtime regressions check that firmware emergency latches block arming, camera faults immediately request a serialized motor stop, fresh frames cannot erase a motor fault, and camera cleanup still runs after a motor failure.
- Mission tests cover empty-room exploration, no moves into unknown/occupied cells, return to launch, operator early return, time/step budgets, blocked return paths, and detached snapshots.
- Default empty four-room mission completed both in tests and through the running dashboard: **202 simulation moves, 100% of reachable synthetic cells observed, returned to `[4,25]`**. Coverage is a ground-truth simulator diagnostic, not a real-world guarantee.
- Arduino Uno firmware compiled with AVR core 1.8.8 in three configurations: default V4/unverified (3,696 bytes flash / 256 bytes RAM), verified V4 (4,732 / 260), and test-only custom L298N (4,790 / 260). No upload/flash was performed.
- Dashboard JavaScript syntax checked with `node --check`. Browser validation covered authenticated color/depth rendering, mission completion, arm/estop/reset state, desktop layout at 1440 px and phone layout at 390 px. No horizontal overflow at either size. No browser console errors during the final check.
- The new Three.js theater completed the people-and-fire scenario at seed 42 in the browser: 282 moves, two observations, 100% reachable synthetic coverage and return home. The New layout control generated a different seed; a 1 x 1 m boundary applied successfully. The 3D scan displayed 4,800 sampled vertices and 7,332 triangles, matched observation pins, RGB/height styles, freeze/resume and a valid downloaded JSON snapshot. These browser observations use synthetic data.

## Not validated here

- No Pi, Uno, D435i or physical motors were connected. Wiring, directions, camera profiles, Pi OS/ARM dependencies, USB power/bandwidth, floor calibration, braking and real watchdog timing still require the bench checks in `hardware.md`.
- No live D435i person or candle trials were performed. The two installed trained models need held-out evaluation at the venue's actual distance, lighting and occlusion conditions. Public fire examples may overlap model training data.
- The warm-color heuristic remains an unverified candidate source. It is distinct from the installed trained fire/smoke model; neither is a validated safety detector for this rover.
- The procedural 3D scanner uses a bounded synthetic forward field of view and deterministic cell motion. The navigation fixture remains an idealized occupancy model. Neither models the D435i noise profile, wheel slip, localization drift, Wi-Fi loss, smoke or real sensor noise.
- Physical autonomous exploration, global mapping, return-home and escort are not implemented. The hardware mission API rejects starts instead of sending simulated routes to the car.

Run `python -m pytest -q` from the project root. See `test-results.txt` for the final recorded result.
