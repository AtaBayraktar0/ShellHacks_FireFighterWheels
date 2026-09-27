"""Bounded acceptance trials for generated missions; no hardware is accessed."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from rover.presentation import PresentationSimulator


def canonical_outcome(report):
    keys = ("seed", "layout_hash", "metrics", "outcome", "events", "entities", "trajectory", "alerts", "following_log")
    value = {key: report.get(key) for key in keys}
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def exercise(seed, width, height, max_ticks=6000):
    began = time.perf_counter()
    sim = PresentationSimulator()
    sim.control("boundary", width_m=width, height_m=height)
    sim.control("randomize", seed=seed)
    state = sim.control("play")
    checks = {"simulation_only": state["simulation_only"] is True,
              "physical_boundary_not_claimed": state["boundary"]["hardware_supported"] is False,
              "no_teleport": True, "movement_uses_observed_free_space": True,
              "physical_destination_free": True, "person_clearance": True,
              "footprint_stays_inside": True, "alerts_only_delivered_at_home": True}
    scans, largest_mesh, largest_fire, assessments = 0, 0, 0, set()
    for tick in range(max_ticks):
        previous = state
        state = sim.tick(.15)
        col, row = state["position"]
        if previous["position"] != state["position"]:
            old_col, old_row = previous["position"]
            checks["no_teleport"] &= abs(col-old_col)+abs(row-old_row) == 1
            # A moving actor may clear a cell between snapshots. Check the
            # refreshed observation independently from the movement flag.
            cells = state["grid"]["cells"]
            value = cells[row][col] if isinstance(cells[0], list) else cells[row*state["grid"]["width"]+col]
            checks["movement_uses_observed_free_space"] &= value == 0
            checks["physical_destination_free"] &= bool(sim._mission._truth[row, col] == 0)
            checks["person_clearance"] &= all(math.dist(state["position"], person["position"]) * sim._resolution >= .38 - 1e-9 for person in sim._people)
        boundary = state["boundary"]
        radius, resolution = boundary["robot_radius_m"], boundary["resolution_m"]
        x, z = (col+.5)*resolution, (row+.5)*resolution
        checks["footprint_stays_inside"] &= radius-1e-8 <= x <= width-radius+1e-8 and radius-1e-8 <= z <= height-radius+1e-8
        for alert in state.get("alerts", []):
            if alert.get("delivered_at_home"):
                checks["alerts_only_delivered_at_home"] &= state["position"] == state["home"]
        mesh = state.get("scan_mesh") or {}
        scans = max(scans, mesh.get("revision", 0))
        largest_mesh = max(largest_mesh, len(mesh.get("triangles", [])))
        for entity in state.get("entities", []):
            if entity["kind"] == "person":
                assessment = entity.get("assessment")
                if isinstance(assessment, dict):
                    assessments.add(str(assessment.get("status", "unassessed")))
                elif assessment:
                    assessments.add(str(assessment))
            elif entity["kind"] in ("hazard", "fire"):
                largest_fire = max(largest_fire, len(entity.get("spread_cells", [])))
        if state["playback_status"] == "finished":
            break
    report = sim.report()
    mesh = state.get("scan_mesh") or {}
    vertices, triangles = mesh.get("vertices", []), mesh.get("triangles", [])
    checks["bounded_termination"] = state["playback_status"] == "finished"
    checks["depth_surface_acquired"] = bool(vertices and triangles and scans)
    checks["mesh_indices_valid"] = all(len(face) == 3 and all(type(i) is int and 0 <= i < len(vertices) for i in face) for face in triangles)
    checks["finite_surface"] = all(len(vertex) == 3 and all(math.isfinite(v) for v in vertex) for vertex in vertices)
    checks["mesh_memory_bounded"] = len(vertices) <= 30000 and len(triangles) <= 30000
    if report["outcome"].get("returned_home"):
        checks["return_reached_home"] = state["position"] == state["home"]
    if report["metrics"].get("escort_verified"):
        follower = report.get("follower") or {}
        checks["escort_arrival_verified"] = follower.get("at_launch_zone") is True and state["position"] == state["home"]
    # Keep geometry counts in the report; the live API supplies the full mesh.
    report.pop("scan_mesh", None)
    report["acceptance"] = {"checks": checks, "verdict": "pass" if all(checks.values()) else "fail",
                            "ticks": tick+1, "mesh_revisions": scans, "largest_mesh_triangles": largest_mesh,
                            "largest_observed_fire_cells": largest_fire, "observed_assessments": sorted(assessments)}
    return report, round(time.perf_counter()-began, 3)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(12)))
    parser.add_argument("--sizes", nargs="+", default=["6x6"])
    parser.add_argument("--replay-seeds", type=int, nargs="*", default=[0, 1])
    parser.add_argument("--output", type=Path, default=Path("reports/procedural-campaign"))
    args = parser.parse_args(argv)
    (args.output/"runs").mkdir(parents=True, exist_ok=True)
    rows = []
    for size in args.sizes:
        width, height = map(float, size.lower().split("x"))
        for seed in args.seeds:
            case = f"mission-seed{seed}-{size}"
            try:
                report, elapsed = exercise(seed, width, height)
                if seed in args.replay_seeds:
                    replay, _ = exercise(seed, width, height)
                    same = canonical_outcome(report) == canonical_outcome(replay)
                    report["acceptance"]["checks"]["deterministic_replay"] = same
                    if not same:
                        report["acceptance"]["verdict"] = "fail"
                filename = "runs/"+case+".json"
                (args.output/filename).write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
                row = {"case": case, "seed": seed, "size": size, "layout_hash": report["layout_hash"],
                       "verdict": report["acceptance"]["verdict"], "status": report["outcome"]["status"],
                       "metrics": report["metrics"], "runtime_s": elapsed,
                       "failed_checks": [key for key, value in report["acceptance"]["checks"].items() if not value],
                       "report": filename}
            except Exception as exc:
                row = {"case": case, "verdict": "fail", "error": f"{type(exc).__name__}: {exc}"}
            rows.append(row)
            print(f"{row['verdict'].upper()} {case}: {row.get('status', '')} {row.get('failed_checks', row.get('error', ''))}", flush=True)
    root = Path(__file__).resolve().parents[1]
    source_files = [root/name for name in ("rover/presentation.py", "rover/procedural_mission.py", "rover/world_geometry.py", "rover/mission.py", "validation/procedural_campaign.py") if (root/name).exists()]
    summary = {"total": len(rows), "passed": sum(r["verdict"] == "pass" for r in rows), "failed": sum(r["verdict"] != "pass" for r in rows)}
    result = {"generated_utc": datetime.now(timezone.utc).isoformat(), "simulation_only": True,
              "source_sha256": {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},
              "summary": summary, "runs": rows}
    (args.output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(summary), flush=True)
    return int(bool(summary["failed"]))


if __name__ == "__main__":
    raise SystemExit(main())
