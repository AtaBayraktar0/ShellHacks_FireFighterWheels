"""Seeded presentation acceptance campaign; no hardware or trained model is used."""
from __future__ import annotations
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from rover.presentation import PresentationSimulator, SCENARIO_IDS


def score(report, finished):
    scenario = report["scenario"]["id"]
    metric, outcome = report["metrics"], report["outcome"]
    boundary = report["boundary"]
    checks = {"terminated": finished, "explicit_simulation": report["simulation_only"] is True,
              "no_hardware_test_claim": report["hardware_test_performed"] is False}
    footprint_ok = True
    cardinal_ok = True
    previous = None
    radius, resolution = boundary["robot_radius_m"], boundary["resolution_m"]
    for col, row in report["trajectory"]:
        x, y = (col + .5) * resolution, (row + .5) * resolution
        footprint_ok &= radius - 1e-8 <= x <= boundary["width_m"] - radius + 1e-8
        footprint_ok &= radius - 1e-8 <= y <= boundary["height_m"] - radius + 1e-8
        if previous is not None:
            cardinal_ok &= abs(col - previous[0]) + abs(row - previous[1]) <= 1
        previous = col, row
    checks.update({"footprint_within_boundary": bool(footprint_ok), "consecutive_cell_motion": bool(cardinal_ok)})
    if outcome["scenario_applicability"] == "not_applicable":
        return checks, "not_applicable" if all(checks.values()) else "fail"
    checks["expected_terminal_state"] = outcome["status"] == ("blocked" if scenario == "blocked_exit" else "returned")
    if scenario in {"empty_search", "person_found"}:
        checks["reachable_search_complete"] = metric["unobserved_cells"] == 0
    if scenario in {"person_found", "people_and_fire", "needs_assistance", "escort_following"}:
        checks["person_observed"] = metric["people_found"] >= 1
    if scenario in {"hazard_reroute", "people_and_fire"}:
        checks["hazard_observed"] = metric["hazards_found"] >= 1
        checks["replan_observed"] = metric["replans"] >= 1
    if scenario == "sensor_loss":
        checks["sensor_hold_observed"] = metric["sensor_holds"] >= 1
    if scenario == "needs_assistance":
        checks["alert_delivered_after_return"] = bool(report["alerts"]) and all(a["delivered_at_home"] for a in report["alerts"])
    if scenario == "escort_following":
        checks["follower_arrived"] = metric["escort_verified"] is True
        checks["following_measured"] = metric["following_checks"] > 0
        follower = report.get("follower") or {}
        trail = follower.get("trail", [])
        checks["actor_never_teleports"] = all(abs(a[0]-b[0])+abs(a[1]-b[1]) <= 1 for a,b in zip(trail,trail[1:]))
    return checks, "pass" if all(checks.values()) else "fail"


def run_case(scenario, seed, width, height, output):
    began = time.perf_counter()
    sim = PresentationSimulator(scenario)
    sim.control("boundary", width_m=width, height_m=height)
    sim.control("randomize", seed=seed)
    sim.control("speed", speed=4)
    sim.control("play")
    state = sim.snapshot()
    for ticks in range(1500):
        state = sim.tick(1.0)
        if state["playback_status"] == "finished":
            break
    report = sim.report()
    checks, verdict = score(report, state["playback_status"] == "finished")
    case_id = f"{scenario}-seed{seed}-{width:g}x{height:g}"
    report["acceptance"] = {"verdict": verdict, "checks": checks}
    target = output / "runs" / (case_id + ".json")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return {"case": case_id, "scenario": scenario, "seed": seed, "width_m": width, "height_m": height,
            "verdict": verdict, "status": report["outcome"]["status"], "moves": report["metrics"]["steps"],
            "coverage_percent": report["metrics"]["coverage_percent"], "failed_checks": [k for k,v in checks.items() if not v],
            "runtime_s": round(time.perf_counter()-began, 3), "report": f"runs/{case_id}.json"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("reports/presentation-campaign"))
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(5)))
    parser.add_argument("--sizes", nargs="+", default=["6x6"], help="Metre rectangles, e.g. 1x1 6x6 50x50")
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIO_IDS, default=list(SCENARIO_IDS))
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for size in args.sizes:
        width, height = map(float, size.lower().split("x"))
        for seed in args.seeds:
            for scenario in args.scenarios:
                try:
                    result = run_case(scenario, seed, width, height, args.output)
                except Exception as exc:
                    result = {"case": f"{scenario}-seed{seed}-{size}", "scenario": scenario, "seed": seed,
                              "width_m": width, "height_m": height, "verdict": "fail", "error": f"{type(exc).__name__}: {exc}"}
                results.append(result)
                print(f"{result['verdict'].upper()} {result['case']} {result.get('failed_checks', [])}", flush=True)
    root = Path(__file__).resolve().parents[1]
    summary = {"total":len(results), "passed":sum(r["verdict"]=="pass" for r in results),
               "failed":sum(r["verdict"]=="fail" for r in results), "not_applicable":sum(r["verdict"]=="not_applicable" for r in results)}
    report = {"generated_utc":datetime.now(timezone.utc).isoformat(), "simulation_only":True,
              "source_sha256":{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/"rover/presentation.py",root/"rover/mission.py",Path(__file__).resolve()]},
              "summary":summary,"runs":results}
    (args.output/"summary.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    columns=["case","scenario","seed","width_m","height_m","verdict","status","moves","coverage_percent","runtime_s"]
    with (args.output/"summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer=csv.DictWriter(handle,fieldnames=columns,extrasaction="ignore");writer.writeheader();writer.writerows(results)
    print(json.dumps(summary),flush=True)
    return 1 if summary["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
