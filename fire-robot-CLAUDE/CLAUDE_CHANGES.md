# 🤖 CLAUDE VERSION — made by Claude (Anthropic, Claude Code)

**This copy was made by Claude, not Codex.** Date: 2026-09-26.
Original project: `~/Desktop/fire-robot` (left untouched by this copy).

Every changed spot in the code is marked with a comment starting with
`# [CLAUDE EDIT]`, and each changed file starts with a "MODIFIED BY CLAUDE" banner.
Find them all with:

    grep -rn "CLAUDE EDIT" pi

## What was changed: map-edge fix

Problem: the planner only kept the car's *center* inside the map, so the body
could hang over the arena edge, and the simulator didn't count that as a collision.

| File | Change |
|---|---|
| `pi/rescue/config.py` | Arena y bounds widened to ±1.25 m (were ±1.0 m). New `edge_is_wall = True` option. |
| `pi/rescue/grid.py` | New `block_edges()` helper: blocks a border band of width robot radius + active clearance, so the whole body stays inside. Used by `blocked_mask()`. Guards against a zero-width slice bug. |
| `pi/rescue/mission.py` | **Bug fix Claude added beyond the original proposal:** `blocked()` re-applies the border after `clear_disk()`, which otherwise erased border cells under the car when it was near an edge. |
| `pi/rescue/sim.py` | `collides()` counts the body crossing the arena edge as a collision. |
| `pi/tests/test_planning.py` | 3 new regression tests: border on all 4 sides (incl. zero-clearance fallback), footprint clearing keeps the border, sim detects edge crossings. |

Credit: the border inflation, sim edge check and wider bounds came from the
edge-fix proposal Claude was asked to review and apply. The `mission.py` re-apply,
the shared `block_edges()` helper with its zero-width guard, and the tests are Claude's additions.

## Isaac Sim 6.1 support (added by Claude)

| File | Change |
|---|---|
| `pi/rescue/isaac.py` | **New.** Builds the arena, boxes, flame and car on the USD stage, and renders a D435-like camera (848x480, 87° FOV, same mount/pitch/pan as `config.py`). `IsaacCamera` runs the images through the real `D435Camera` perception code. `IsaacBase` reuses the `SimBase` error model and moves the car in Isaac. |
| `pi/run_isaac.py` | **New.** Launcher, same flags as `run_mission.py` plus `--headless`, `--save-frames`, `--render-frames`, `--keep-open`. |
| `pi/tests/test_isaac_geometry.py` | **New.** Checks the Isaac camera mount matches `camera_to_robot()`, and runs a full mission through the Isaac classes with a fake renderer. Needs no Isaac install. |
| `README.md`, `.gitignore` | How to run it in Isaac; ignore files for the GitHub upload. |

Limits: the car is moved kinematically (no wheel physics). Sonar and collisions use the
2D truth map, not PhysX. Isaac only uses `SimulationApp`, plain USD (`pxr`) and Replicator.
**Not yet run inside Isaac Sim itself**: written and unit-tested on a Mac without Isaac.

## Not included here
- The "edge escape" idea (letting a drifted car plan its way back out of the
  border band) was only tried in a sandbox and was never approved, so it's not in this copy.
- Simulation results, plots and the video are not included, only the car code.

## Caveats
- Set the map bounds to the **measured** real arena. Bigger numbers don't make a small table safe.
- The border limits the *estimated* position. Localization drift can still take the real car over an edge.
