# FIRE_ROBOT_CODEX

**This separate code copy was prepared and annotated by OpenAI Codex.**
The underlying fire-robot project is existing user-provided code; Codex does not claim original authorship of it.

## Changes made by OpenAI Codex

| File | Modification |
|---|---|
| `pi/rescue/config.py` | Added `edge_is_wall=True`; changed demo y bounds from ±1.0 m to ±1.25 m. |
| `pi/rescue/grid.py` | Added a blocked border band equal to robot radius plus active clearance. Shared helper handles fallback clearance and zero-width bands. |
| `pi/rescue/mission.py` | Restores the border after clearing the current robot footprint, so boundary constraints cannot be erased by that operation. |
| `pi/rescue/sim.py` | Counts a full-body crossing of configured map bounds as a collision when edge walls are enabled. |
| `pi/tests/test_planning.py` | Added border, fallback-clearance, footprint-clearing, collision, and disabled-edge regression tests. |

Each changed Python file has a `CODEX COPY` comment at its top explaining its modifications. The README identifies this copy as well.

## Scope

- Software only; no additional hardware.
- Existing fallback margins remain enabled and unchanged.
- Firmware and remaining application modules are copied without behavioral changes.
- No D435 localization, new gyro-failure handling, arrival-status changes, camera degradation modes, or uncertainty-based planning were implemented.
- Temporary simulation instrumentation and video files are excluded from this code copy.
- A new Isaac Sim 6.1 integration is included; see the section below.
- The demo's wider bounds are not suitable for a smaller real arena: configure measured bounds before operation.
- Border planning constrains estimated position; it does not eliminate physical position drift.

## Validation

The behavioral version previously passed 28 tests. A fresh test run is saved in `CODEX_VALIDATION.txt` for this annotated copy.
Thirty default gyro-enabled simulations using seeds 0–29 completed with zero recorded collisions, six runs using fallback margins, and worst true home error of 7.47 cm. These figures apply to that demo/noise setup, not hardware certification.

## Isolation

This copy resides in the sandbox under `FIRE_ROBOT_CODEX`. Creating and annotating it does not modify the user's original project. Earlier edge changes already present in the original project are neither reverted nor altered by this copy operation.

## Isaac Sim 6.1 adaptation — created by OpenAI Codex

Added `run_isaac.py` and `isaac/` with a procedural four-wheel articulated robot, bounded arena, joint-velocity motor adapter, RTX RGB-D adapter using the 6.1 APIs, idealized PhysX raycast debug sensor, and truth-based evaluation reports. Added Linux/Windows launch wrappers, CPU adapter tests, GitHub Actions, `.gitignore`, and `ISAAC_SIM.md`. These are new adapters; the physical hardware runner and firmware were not changed.

The new control adapter has turn timeout/error handling scoped to Isaac. This does not change real-car gyro-failure handling. No camera localization is implemented. The procedural robot is an approximation; full Isaac/RTX/PhysX execution has not been tested on this development machine.
