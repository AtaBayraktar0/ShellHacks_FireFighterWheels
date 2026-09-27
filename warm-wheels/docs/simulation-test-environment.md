# Repeatable software scenario environment

This environment checks specific software behaviors in an empty synthetic building. It does not run the Elegoo vehicle, Arduino firmware, RealSense camera, a person detector, or a fire detector. Passing results are evidence for the checks below; they are not evidence of safe operation in smoke, fire, or an occupied building.

## Run the suite

From the `warm-wheels` project directory, using the project's Python environment:

```sh
python -m pip install -r requirements.txt
python -m validation.run_scenarios --output reports/simulation --seed 0
python -m unittest discover -s tests -p test_validation_scenarios.py -v
```

The runner writes `results.json` and `results.csv` in the output directory. These names are replaced on a subsequent run to the same directory. Use a distinct output directory when preserving an earlier run. No hardware needs to be attached. The runner makes no network calls or model downloads.

The default suite contains eight scenarios. A subset can be selected with repeated arguments:

```sh
python -m validation.run_scenarios --output reports/selected --scenario depth_data_holes --scenario dropped_data
```

Exit status is `0` when all selected scenario checks pass, `1` for a failed check, scenario exception, or failed baseline comparison, and `2` for an invalid command-line argument. A failed scenario is retained in the evidence; it is not silently omitted. A filesystem failure that prevents report writing raises an error and exits unsuccessfully.

## Synthetic assumptions

- The mission fixture is 30 by 30 cells at 0.20 m per cell, with four connected rooms, broad doorways, furniture, and a fixed launch cell `[4,25]`.
- Walls and objects are expanded by one cell before planning to reserve clearance. The mission then moves a point robot through this synthetic clearance grid, at most one cardinal cell per tick.
- The historical empty-fixture checks use ideal 360-degree occupancy rays with a six-cell range. The current procedural theater's 3D depth mesh uses a forward field of view and a persistent acquisition cache. Neither model includes sensor noise, wheel slip, latency, localization error, smoke, heat, or battery behavior, and neither is a D435i simulation.
- The navigation planner sees only accumulated observations: `-1` unknown, `0` free, `1` occupied. It does not use the hidden floorplan to choose a destination or a route. Shortest paths are found with breadth-first search on an unweighted grid.
- People and fire objects are absent. Exploration and return are independent of detections. The additional-clutter scenario contains stationary solid objects, not a crowd of moving people.
- A logical clock advances 0.1 seconds per tick without wall-clock sleeps. This supports repeatability; logical duration is not physical vehicle travel time or measured real-time performance.
- Version 1 contains no random behavior. `--seed` defaults to `0` and is recorded for experiment bookkeeping; changing it does not randomize the scene. Do not describe different seeds as independent trials.

The coverage diagnostic counts observed cells in the hidden fixture's reachable free-space component. It is available only because the simulator knows the fixture. A real robot does not have this ground-truth denominator. This value is not whole-house scan completeness, survivor-detection recall, or proof that an environment is safe.

## Scenario acceptance criteria

All mission scenarios must terminate within 700 ticks and have zero observed violations of the movement invariant: a move is exactly one cardinal cell and its destination was previously observed free. The additional criteria are specific to each scenario:

| Scenario ID | Setup | Required software result |
| --- | --- | --- |
| `empty_search_return` | Default empty four-room fixture; no detections supplied | `complete` at launch; all four room quadrants visited; zero unobserved reachable fixture cells |
| `operator_early_return` | Operator requests return after 12 ticks | No teleport at request; enters `returning`; reaches launch with some reachable cells still unobserved |
| `blocked_launch` | After 12 ticks, a synthetic observed obstacle covers launch | Reports `blocked` while away from launch; stays blocked on the next tick; never reports returned |
| `exploration_step_budget` | Exploration limited to 10 moves | Returns to launch; reason identifies step budget; remaining unknown reachable cells are reported |
| `depth_data_holes` | Synthetic aligned image/depth pair with one front zero, NaN, or infinite return | Every case yields unknown clearance, zero software motor command, and disarmed gate |
| `dropped_data` | One-second-old camera frame, then separately an expired operator command lease | Both cases yield zero software command; stale-frame fault disarms the gate |
| `changed_obstacle_replan` | A synthetic observed obstacle covers part of a known return route | Next blocked cell is not entered; return route changes; rover still reaches launch |
| `cluttered_static_fixture` | Eight additional solid objects, initially hidden from the sensor | Returns to launch and observes every reachable fixture cell without violating movement invariant |

The frame-age limit exercised by `dropped_data` is 0.45 seconds. The command-lease limit is 0.30 seconds. These are software configuration values. The scenario does not measure physical stopping distance, braking time, network outage recovery, serial watchdog operation, or electrical emergency-stop performance.

There are no minimum person/fire accuracy thresholds in this suite because it performs no detector evaluation. There are no claimed thermal, smoke, or real-world obstacle-avoidance thresholds.

## Fault injection

`MissionSimulator.set_obstacle(col, row, observed=False, clearance_cells=1)` is a test-only simulation interface. It adds an occupied square with a clearance buffer, rejects invalid coordinates and robot overlap, and never sends a motor command. It is not exposed by the robot's HTTP API.

By default the change remains unknown until sensed. `observed=True` supplies an explicit synthetic observation for a reproducible fault test; it does not represent a measurement from hardware. The observed-obstacle test therefore verifies replanning after an occupancy update, not whether a real camera would detect that obstacle.

Injection recomputes the reachable ground-truth component used for the coverage diagnostic. Coverage before and after a topology change may use different denominators. In `blocked_launch`, launch itself becomes occupied, so the reachable diagnostic denominator is zero and coverage is reported as `0.0`; this value does not assess exploration success. Acceptance for that case depends on the blocked state and absence of a false return.

## Freeze and repeat a baseline

Choose a reviewed revision and save its first complete run separately:

```sh
python -m validation.run_scenarios --output reports/baseline-v1 --seed 0
python -m validation.run_scenarios --output reports/repeat-v1 --seed 0 --baseline reports/baseline-v1/results.json
```

`--baseline` compares schema version, baseline ID, source hashes, seed, and exact selected scenario evidence. Scenario evidence includes actual/expected checks, metrics, and mission trajectories. The environment and generation timestamp are recorded but excluded from exact comparison. Use the same scenario selection and seed for both runs.

An intentional algorithm change can improve behavior and still fail an exact comparison. Review the source difference and changed trajectories before approving a new baseline; do not overwrite the old baseline to conceal a mismatch. Source hashes cover the mission, spatial, safety, and camera modules plus this runner. Preserve the dependency lock/environment and Git revision with a released baseline as well: hashes alone do not freeze the whole Python environment.

`summary.passed` and `summary.failed` count scenarios only. A failed baseline comparison sets `summary.all_passed` to `false` and returns exit status `1`, even when all eight current scenario checks pass. The separate `baseline_comparison` object explains which comparison failed.

## Evidence schema

`results.json` contains:

- `schema_version`, `baseline_id`, generation time, seed handling, environment versions, and SHA-256 hashes of the tested source files.
- `scope`, explicitly limiting the evidence to software simulation.
- `summary`: total, passed, failed, and overall pass status.
- `results`: scenario ID and description; each check's ID, boolean outcome, actual value, and expected value; scenario metrics; mission trajectories where applicable.
- `baseline_comparison`: `null` unless a frozen report was supplied, otherwise comparison outcomes and any read/parse error.

`results.csv` has one row per scenario with columns:

```text
scenario_id,passed,phase,steps,coverage_percent,checks_passed,checks_total,failure_details,trajectory_sha256
```

The CSV is a summary. Consult JSON for thresholds, fault parameters, trajectories, and baseline comparison. Empty mission fields in depth and dropped-data rows mean those scenarios did not run a mission; they do not represent zero mission performance.

The supplied example evidence is in `reports/simulation/`. In the current deterministic baseline, the empty scene returns after 202 grid moves and the static-clutter fixture after 228. These are fixture-specific regression observations, not speed or reliability claims for hardware. The executable acceptance criteria above remain the authority for pass/fail.

See `test-plan.md` and `hardware.md` for the separate proposed physical test process. Physical test results must be recorded from actual runs and must not be inferred from this suite.
