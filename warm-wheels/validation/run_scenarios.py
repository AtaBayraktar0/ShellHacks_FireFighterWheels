"""Deterministic synthetic scenarios with JSON/CSV evidence and pass/fail exit.

Run from the project root:
    python -m validation.run_scenarios --output reports/simulation

No Arduino, RealSense, network endpoint, model download, or motor is accessed.
The seed is recorded for reproducibility conventions; v1 uses no randomness.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys

import cv2
import numpy as np

from rover.camera import Frame
from rover.mission import MissionSimulator
from rover.safety import SafetyGate
from rover.spatial import analyze


BASELINE_ID = "warm-wheels-synthetic-v1"
SCHEMA_VERSION = "1.0"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
CSV_FIELDS = ("scenario_id", "passed", "phase", "steps", "coverage_percent",
              "checks_passed", "checks_total", "failure_details", "trajectory_sha256")


class LogicalClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self):
        self.now = round(self.now + 0.1, 6)


def _check(identifier, actual, expected, passed=None):
    return {"id": identifier, "passed": bool(actual == expected if passed is None else passed),
            "actual": actual, "expected": expected}


def _mission(**kwargs):
    clock = LogicalClock()
    sim = MissionSimulator(clock=clock, **kwargs)
    initial = sim.start()
    evidence = {"trajectory": [initial["position"]], "move_violations": [],
                "rooms": {tuple(v > 15 for v in initial["position"])}, "ticks": 0}
    return sim, clock, evidence


def _step(sim, clock, evidence):
    before = sim.snapshot()
    clock.advance()
    state = sim.tick()
    col, row = state["position"]
    old_col, old_row = before["position"]
    if (col, row) != (old_col, old_row):
        if abs(col - old_col) + abs(row - old_row) != 1:
            evidence["move_violations"].append("movement exceeded one cardinal grid cell")
        if before["grid"]["cells"][row * before["grid"]["width"] + col] != 0:
            evidence["move_violations"].append("moved into a cell not previously observed free")
    evidence["trajectory"].append(state["position"])
    evidence["rooms"].add(tuple(v > 15 for v in state["position"]))
    evidence["ticks"] += 1
    return state


def _finish(sim, clock, evidence, limit=700):
    state = sim.snapshot()
    while state["phase"] not in ("complete", "blocked") and evidence["ticks"] < limit:
        state = _step(sim, clock, evidence)
    return state


def _mission_result(state, evidence, extra_checks=(), extra_metrics=None):
    trajectory_json = json.dumps(evidence["trajectory"], separators=(",", ":"))
    metrics = {name: state[name] for name in (
        "phase", "reason", "steps", "position", "home", "coverage_percent", "unobserved_cells", "reachable_cells")}
    metrics.update({"ticks": evidence["ticks"], "logical_duration_s": round(evidence["ticks"] * .1, 1),
                    "visited_room_quadrants": len(evidence["rooms"]),
                    "motion_invariant_violations": len(evidence["move_violations"]),
                    "trajectory_sha256": hashlib.sha256(trajectory_json.encode()).hexdigest()})
    metrics.update(extra_metrics or {})
    return {"checks": [_check("terminates_within_700_ticks", state["phase"] in ("complete", "blocked"), True),
                       _check("no_unknown_or_noncardinal_moves", evidence["move_violations"], []),
                       *extra_checks],
            "metrics": metrics, "trajectory": evidence["trajectory"]}


def _empty_search():
    sim, clock, evidence = _mission()
    state = _finish(sim, clock, evidence)
    return _mission_result(state, evidence, [
        _check("mission_complete", state["phase"], "complete"),
        _check("returned_to_launch", state["position"], state["home"]),
        _check("all_reachable_fixture_cells_observed", state["unobserved_cells"], 0),
        _check("all_four_rooms_visited", len(evidence["rooms"]), 4),
        _check("person_or_fire_not_required", state["scenario"], "Empty four-room fixture: no fire or person objects"),
    ], {"detections_supplied": 0})


def _operator_return():
    sim, clock, evidence = _mission()
    for _ in range(12):
        _step(sim, clock, evidence)
    before = sim.snapshot()
    requested = sim.abort()
    state = _finish(sim, clock, evidence)
    return _mission_result(state, evidence, [
        _check("abort_does_not_teleport", requested["position"], before["position"]),
        _check("abort_enters_returning", requested["phase"], "returning"),
        _check("mission_complete", state["phase"], "complete"),
        _check("returned_to_launch", state["position"], state["home"]),
        _check("partial_scan_reported", state["unobserved_cells"] > 0, True),
    ], {"abort_at_tick": 12})


def _blocked_home():
    sim, clock, evidence = _mission()
    for _ in range(12):
        _step(sim, clock, evidence)
    sim.set_obstacle(*sim.home, observed=True)
    state = sim.abort()
    terminal = _step(sim, clock, evidence)
    return _mission_result(terminal, evidence, [
        _check("blocked_return_reported", state["phase"], "blocked"),
        _check("no_false_return", state["position"] != state["home"], True),
        _check("blocked_is_stable", terminal["phase"], "blocked"),
    ], {"fault": "synthetic observed obstacle covering launch cell"})


def _step_budget():
    sim, clock, evidence = _mission(max_exploration_steps=10)
    state = _finish(sim, clock, evidence)
    return _mission_result(state, evidence, [
        _check("mission_complete", state["phase"], "complete"),
        _check("returned_to_launch", state["position"], state["home"]),
        _check("step_budget_reason", "step budget" in state["reason"], True),
        _check("partial_scan_reported", state["unobserved_cells"] > 0, True),
    ], {"exploration_step_budget": 10})


def _depth_hole():
    checks, cases = [], []
    for label, bad_depth in (("zero", 0.0), ("nan", float("nan")), ("infinite", float("inf"))):
        depth = np.full((120, 160), 2.0, np.float32)
        depth[60, 80] = bad_depth
        frame = Frame(np.zeros((120, 160, 3), np.uint8), depth, 144., 144., 80., 60., 1.)
        scan = analyze(frame)
        gate = SafetyGate(motion_enabled=True, armed=True)
        command, reason = gate.output(1.05, 1.0, scan["clearance_m"], scan["depth_valid_fraction"], True, 1.04, (20, 20))
        checks.extend((_check(f"{label}_clearance_unknown", scan["clearance_m"], None),
                       _check(f"{label}_zero_command", list(command), [0, 0]),
                       _check(f"{label}_disarmed", gate.armed, False)))
        cases.append({"input": label, "clearance_m": scan["clearance_m"], "command": list(command), "reason": reason})
    return {"checks": checks, "metrics": {"cases": cases, "hardware_motor_calls": 0}}


def _dropped_data():
    cases, checks = [], []
    for label, frame_at, command_at in (("stale_frame", 1.0, 2.0), ("expired_command_lease", 1.99, 1.0)):
        gate = SafetyGate(motion_enabled=True, armed=True)
        command, reason = gate.output(2.0, frame_at, 2.0, 1.0, True, command_at, (20, 20))
        checks.append(_check(f"{label}_zero_command", list(command), [0, 0]))
        cases.append({"fault": label, "command": list(command), "reason": reason, "armed_after": gate.armed})
    checks.append(_check("frame_fault_disarms", cases[0]["armed_after"], False))
    return {"checks": checks, "metrics": {"cases": cases, "max_frame_age_s": .45, "command_lease_s": .30,
                                          "hardware_motor_calls": 0}}


def _changed_obstacle():
    sim, clock, evidence = _mission()
    for _ in range(12):
        _step(sim, clock, evidence)
    before = sim.abort()
    old_route = before["route"]
    # Two cells ahead: a one-cell clearance buffer does not overlap the rover.
    obstacle = old_route[2]
    sim.set_obstacle(*obstacle, observed=True)
    after = _step(sim, clock, evidence)
    rerouted = after["route"]
    state = _finish(sim, clock, evidence)
    return _mission_result(state, evidence, [
        _check("return_route_changed", rerouted != old_route[1:], True),
        _check("blocked_next_cell_not_entered", after["position"] != old_route[1], True),
        _check("mission_complete", state["phase"], "complete"),
        _check("returned_to_launch", state["position"], state["home"]),
    ], {"injected_obstacle": obstacle, "old_route": old_route, "replanned_route": rerouted,
        "synthetic_observation_supplied": True})


def _cluttered_fixture():
    sim, clock, evidence = _mission(max_exploration_steps=400)
    obstacles = [(11, 5), (5, 10), (20, 5), (24, 12), (5, 20), (11, 27), (19, 26), (26, 19)]
    for col, row in obstacles:
        sim.set_obstacle(col, row, observed=False)
    state = _finish(sim, clock, evidence)
    return _mission_result(state, evidence, [
        _check("mission_complete", state["phase"], "complete"),
        _check("returned_to_launch", state["position"], state["home"]),
        _check("all_reachable_fixture_cells_observed", state["unobserved_cells"], 0),
    ], {"injected_solid_fixture_objects": [list(item) for item in obstacles], "objects_are_people": False,
        "synthetic_observation_supplied": False})


SCENARIOS = {
    "empty_search_return": ("Empty whole-place search completes and returns without people/fire", _empty_search),
    "operator_early_return": ("Operator abort returns home with explicitly incomplete coverage", _operator_return),
    "blocked_launch": ("Observed blocked launch is reported; no pretend return", _blocked_home),
    "exploration_step_budget": ("Step-budget exhaustion returns before exploration is complete", _step_budget),
    "depth_data_holes": ("Zero/NaN/infinite front depth produces unknown clearance and zero software command", _depth_hole),
    "dropped_data": ("Stale camera data and expired operator leases stop software output", _dropped_data),
    "changed_obstacle_replan": ("New observed obstacle changes the return route", _changed_obstacle),
    "cluttered_static_fixture": ("Additional stationary solid objects are explored around; no people modeled", _cluttered_fixture),
}


def run_scenarios(output, *, seed=0, scenario_names=None, baseline=None):
    """Write evidence and return it; caller decides how to handle failed checks."""
    names = list(SCENARIOS) if scenario_names is None else list(scenario_names)
    if not names or any(name not in SCENARIOS for name in names):
        raise ValueError("Select one or more known scenarios.")
    results = []
    for name in names:
        description, run = SCENARIOS[name]
        try:
            result = run()
        except Exception as exc:
            result = {"checks": [_check("scenario_execution", f"{type(exc).__name__}: {exc}", "no exception", False)], "metrics": {}}
        result = {"scenario_id": name, "description": description,
                  "passed": bool(result["checks"]) and all(item["passed"] for item in result["checks"]), **result}
        results.append(result)
    hashes = {}
    for relative in ("rover/mission.py", "rover/spatial.py", "rover/safety.py", "rover/camera.py", "validation/run_scenarios.py"):
        hashes[relative] = hashlib.sha256((PROJECT_ROOT / relative).read_bytes()).hexdigest()
    passed = sum(result["passed"] for result in results)
    report = {
        "schema_version": SCHEMA_VERSION, "baseline_id": BASELINE_ID,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "seed": int(seed), "randomization": "none; deterministic fixtures and logical clock; seed is recorded but unused",
        "environment": {"python": platform.python_version(), "numpy": np.__version__, "opencv": cv2.__version__,
                        "platform": platform.platform()},
        "source_sha256": hashes,
        "scope": "Software simulation only. No hardware run, smoke/fire test, human trial, or safety certification.",
        "summary": {"total": len(results), "passed": passed, "failed": len(results) - passed, "all_passed": passed == len(results)},
        "results": results,
    }
    report["baseline_comparison"] = None
    if baseline is not None:
        comparison = {"path": str(Path(baseline)), "passed": False}
        try:
            frozen = json.loads(Path(baseline).read_text(encoding="utf-8"))
            comparisons = {
                "schema_matches": frozen.get("schema_version") == SCHEMA_VERSION,
                "baseline_id_matches": frozen.get("baseline_id") == BASELINE_ID,
                "source_hashes_match": frozen.get("source_sha256") == hashes,
                "seed_matches": frozen.get("seed") == int(seed),
                "scenario_results_match": frozen.get("results") == results,
            }
            comparison.update(comparisons)
            comparison["passed"] = all(comparisons.values())
        except (OSError, ValueError, AttributeError) as exc:
            comparison["error"] = f"{type(exc).__name__}: {exc}"
        report["baseline_comparison"] = comparison
        report["summary"]["all_passed"] = report["summary"]["all_passed"] and comparison["passed"]
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "results.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (destination / "results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for result in results:
            metrics = result["metrics"]
            writer.writerow({"scenario_id": result["scenario_id"], "passed": result["passed"],
                "phase": metrics.get("phase", ""), "steps": metrics.get("steps", ""),
                "coverage_percent": metrics.get("coverage_percent", ""),
                "checks_passed": sum(check["passed"] for check in result["checks"]),
                "checks_total": len(result["checks"]),
                "failure_details": "; ".join(check["id"] for check in result["checks"] if not check["passed"]),
                "trajectory_sha256": metrics.get("trajectory_sha256", "")})
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("reports/simulation"))
    parser.add_argument("--seed", type=int, default=0, help="Recorded only; baseline v1 has no randomized behavior")
    parser.add_argument("--scenario", action="append", choices=tuple(SCENARIOS), help="Repeat to select a subset; default runs all eight")
    parser.add_argument("--baseline", type=Path, help="Frozen results.json to compare source hashes, seed, and scenario evidence exactly")
    args = parser.parse_args(argv)
    report = run_scenarios(args.output, seed=args.seed, scenario_names=args.scenario, baseline=args.baseline)
    for result in report["results"]:
        print(f"{'PASS' if result['passed'] else 'FAIL'} {result['scenario_id']}")
    if report["baseline_comparison"] is not None:
        print(f"{'PASS' if report['baseline_comparison']['passed'] else 'FAIL'} frozen_baseline_comparison")
    summary = report["summary"]
    print(f"{summary['passed']}/{summary['total']} software scenarios passed; evidence: {args.output.resolve()}")
    return 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
