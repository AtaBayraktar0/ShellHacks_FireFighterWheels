"""Behavior tests use real seeded worlds; disable only expensive mesh rendering."""
import copy
import json
import math

import numpy as np
import pytest

from rover.presentation import PresentationSimulator, ProceduralPresentationSimulator
from rover.mission import _ray
from rover.world_geometry import SyntheticDepthScan


@pytest.fixture
def fast_scan(monkeypatch):
    # Mesh ray acquisition is independently tested by test_world_geometry.py.
    monkeypatch.setattr(SyntheticDepthScan, "update", lambda *args, **kwargs: False)


def run(sim, limit=2500):
    sim.control("play")
    for _ in range(limit):
        prior = sim.snapshot()
        state = sim.tick(.15)
        if state["position"] != prior["position"]:
            col, row = state["position"]
            assert abs(col - prior["position"][0]) + abs(row - prior["position"][1]) == 1
            assert prior["grid"]["cells"][row * prior["grid"]["width"] + col] == 0
            assert sim._mission._truth[row, col] == 0
            assert all(math.dist(state["position"], person["position"]) * sim._resolution >= .38 - 1e-9 for person in sim._people)
        for entity in state["entities"]:
            col, row = entity["position"]
            assert state["grid"]["cells"][row * state["grid"]["width"] + col] != -1
            if entity["kind"] == "hazard":
                assert all(state["grid"]["cells"][r * state["grid"]["width"] + c] == 1
                           for c, r in entity["spread_cells"])
        if state["playback_status"] == "finished":
            assert not state["outcome"]["all_clear"]
            assert all(alert["delivered_at_home"] == state["outcome"]["returned_home"] for alert in state["alerts"])
            return state
    pytest.fail("Procedural mission failed to terminate within bounded test horizon")


def person_fixture(sim, response):
    candidates = [cell for cell in sim._initial_reachable if 2 <= math.dist(cell, sim._mission.home) <= 4
                  and all(sim._base_fixture[y, x] == 0 for x, y in _ray(sim._mission.home, cell))]
    assert candidates
    cell = sorted(candidates)[0]
    person = {"id": "controlled-person", "position": cell, "response": response,
              "posture": "seated", "samples": 0, "assessment": "unobserved", "observed": None,
              "trail": [list(cell)], "following": False, "arrived": False,
              "pause_at": 999, "moves": 0, "pause_ticks": 0, "pause_done": False}
    sim._people = [person]
    sim._fires = []
    sim._fault_tick = None
    return person


def test_default_is_procedural_and_random_seed_can_be_replayed(fast_scan, monkeypatch):
    monkeypatch.setattr("rover.presentation.secrets.randbelow", lambda _limit: 8123)
    sim = PresentationSimulator()
    assert isinstance(sim, ProceduralPresentationSimulator)
    initial = sim.snapshot()
    assert initial["seed"] == 8123
    assert initial["scenario"]["id"] == "randomized_mission" and initial["scenarios"] == []
    assert initial["entities"] == [] and initial["metrics"]["detections"] == 0
    assert "simulation_diagnostics" not in sim.report()
    sim.control("play")
    for _ in range(25):
        sim.tick(.15)
    first = sim.snapshot()
    sim.control("reset")
    sim.control("play")
    for _ in range(25):
        sim.tick(.15)
    assert sim.snapshot() == first
    sim.control("pause")
    held = sim.snapshot()
    assert sim.tick(1) == held
    with pytest.raises(ValueError):
        sim.control("select", scenario="empty_search")


def test_seed_changes_composition_spawns_and_multiple_world_families(fast_scan):
    layouts, compositions, home_sides, bands = set(), set(), set(), set()
    for seed in range(30):
        sim = PresentationSimulator(seed=seed)
        layouts.add(sim.snapshot()["layout_hash"])
        compositions.add((len(sim._people), len(sim._fires), tuple(p["response"] for p in sim._people), sim._fault_tick is not None))
        c, r = sim._mission.home
        home_sides.add(min(range(4), key=lambda side: (c, sim._mission.width - c - 1, r, sim._mission.height - r - 1)[side]))
        distances = sorted(math.dist(cell, sim._mission.home) for cell in sim._initial_reachable)
        for entity in sim._people + sim._fires:
            position = entity.get("position", entity.get("origin"))
            distance = math.dist(position, sim._mission.home)
            bands.add(sum(distance > distances[len(distances) * i // 3] for i in (1, 2)))
    assert len(layouts) == 30
    assert len(compositions) >= 15
    assert home_sides == {0, 1, 2, 3}
    assert bands == {0, 1, 2}


@pytest.mark.parametrize("seed", range(10))
def test_generated_missions_terminate_with_honest_observation_and_motion(seed, fast_scan):
    sim = PresentationSimulator(seed=seed)
    result = run(sim)
    assert result["outcome"]["status"] in ("returned", "blocked")
    if result["outcome"]["returned_home"]:
        assert result["position"] == result["home"]
    report = sim.report()
    assert report["simulation_diagnostics"]["people_total"] >= result["metrics"]["people_found"]
    for person in report["simulation_diagnostics"]["people"]:
        assert all(abs(a[0]-b[0]) + abs(a[1]-b[1]) == 1
                   for a, b in zip(person["actual_trail"], person["actual_trail"][1:]))
    assert not report["hardware_test_performed"]
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize("response", ["mobile", "assistance_needed", "no_response"])
def test_actor_response_requires_repeated_samples_and_drives_a_real_decision(response, fast_scan):
    sim = PresentationSimulator(seed=1)
    person = person_fixture(sim, response)
    sim.control("play")
    assert person["samples"] == 1
    for _ in range(3):
        state = sim.tick(.15)
        assert state["phase"] == "assessing"
        assert not sim._pending_alerts and sim._follower is None
        assert person["assessment"] == "observing"
    sim.tick(.15)
    assert person["samples"] == 5
    assert person["assessment"] == response
    assert person["observed"]["posture"] == "seated"  # never inferred from response
    result = run_started(sim)
    if response == "mobile":
        assert result["metrics"]["escort_verified"]
        assert result["follower"]["visible"] and result["follower"]["at_launch_zone"]
    else:
        assert result["alerts"][0]["response"] == response
        assert result["alerts"][0]["delivered_at_home"]
        assert not result["metrics"]["escort_verified"]


def run_started(sim, limit=1200):
    for _ in range(limit):
        state = sim.tick(.15)
        if state["playback_status"] == "finished":
            return state
    pytest.fail("Active mission did not finish")


def test_fire_evolves_at_bounded_rate_without_crossing_walls_or_leaking_hidden_origins(fast_scan):
    sim = PresentationSimulator(seed=0)
    sim._people = []
    sim._fault_tick = None
    fire = sim._fires[0]
    fire["origin"] = max(sim._initial_reachable, key=lambda cell: math.dist(cell, sim._mission.home))
    fire["cells"] = {fire["origin"]}
    fire["start_tick"] = 0
    sim.control("play")
    assert not sim.snapshot()["entities"]
    initial_intensity = fire["intensity"]
    previous = set(fire["cells"])
    for tick in range(1, 90):
        sim._ticks = tick
        sim._evolve_environment()
        assert len(fire["cells"] - previous) <= 1
        for cell in fire["cells"] - previous:
            assert any(abs(cell[0] - old[0]) + abs(cell[1] - old[1]) == 1 for old in previous)
        assert all(sim._world.raw_cells[row, col] == 0 for col, row in fire["cells"])
        previous = set(fire["cells"])
    assert fire["intensity"] > initial_intensity
    assert len(fire["cells"]) > 1
    # Simulator truth changed, but no new sensor call means no operator event.
    assert sim.snapshot()["entities"] == []
    assert not any(event["type"] in ("discovery", "hazard") for event in sim.snapshot()["events"])


def test_sensor_gap_holds_position_while_fire_keeps_evolving(fast_scan):
    sim = PresentationSimulator(seed=0)
    sim._people = []
    sim._fault_tick = 1
    sim._fires[0]["start_tick"] = 0
    sim.control("play")
    before = sim.snapshot()["position"]
    intensity = sim._fires[0]["intensity"]
    for _ in range(10):
        state = sim.tick(.15)
        assert state["position"] == before
        assert state["phase"] == "sensor_hold"
    assert sim._fires[0]["intensity"] > intensity


def test_newly_observed_fire_changes_planned_route_and_preserves_initial_coverage_basis(fast_scan):
    sim = PresentationSimulator(seed=1)  # independently generated empty mission
    sim.control("play")
    for _ in range(50):
        sim.tick(.15)
        if len(sim._mission._route) >= 4:
            break
    old_route = list(sim._mission._route)
    assert len(old_route) >= 4
    initial_denominator = len(sim._initial_reachable)
    origin = old_route[2]
    sim._fires = [{"id": "measured-dynamic-fire", "origin": origin, "cells": {origin},
                   "start_tick": 0, "intensity": .4, "growth_per_tick": .003,
                   "spread_period": 100, "max_cells": 8, "active": False,
                   "observed": None, "exclusion": set()}]
    result = sim.tick(.15)
    assert result["metrics"]["hazards_found"] == 1
    assert result["metrics"]["replans"] >= 1
    assert sim._mission.position not in sim._fires[0]["exclusion"]
    assert sim._mission._route != old_route[1:]
    assert len(sim._mission._reachable_truth) == initial_denominator


@pytest.mark.parametrize("width,height", [(1,1), (1,50), (50,1), (50,50), (3,7)])
def test_generated_boundary_enforces_full_footprint(width, height, fast_scan):
    sim = PresentationSimulator(seed=22)
    state = sim.control("boundary", width_m=width, height_m=height)
    rows, cols = np.where(sim._base_fixture == 0)
    resolution = state["boundary"]["resolution_m"]
    assert np.all((cols + .5) * resolution - .2 >= -1e-9)
    assert np.all((rows + .5) * resolution - .2 >= -1e-9)
    assert np.all((cols + .5) * resolution + .2 <= width + 1e-9)
    assert np.all((rows + .5) * resolution + .2 <= height + 1e-9)
    assert state["grid"]["width"] <= 80 and state["grid"]["height"] <= 80
    assert not state["playing"] and state["phase"] == "idle"


def test_initial_ray_mesh_is_genuinely_acquired_and_reset_deterministic():
    sim = PresentationSimulator(seed=1)
    before = sim.snapshot()["scan_mesh"]
    assert not before["vertices"]
    first = sim.control("play")["scan_mesh"]
    assert first["vertices"] and first["triangles"]
    assert first["source"] == "synthetic raycast depth"
    sim.control("reset")
    second = sim.control("play")["scan_mesh"]
    assert first == second


def test_pause_resume_at_launch_keeps_pending_follower_and_does_not_reset(fast_scan):
    sim = PresentationSimulator(seed=1)
    person = person_fixture(sim, "mobile")
    sim.control("play")
    # Reached launch while actor still needs measured movement into the zone.
    for _ in range(4):
        sim.tick(.15)
    assert sim._follower is person
    sim._mission.phase = "complete"
    sim._mission.position = sim._mission.home
    sim._escort_wait = True
    sim._outcome_recorded = False
    ticks = sim._ticks
    sim.control("pause")
    sim.control("play")
    assert sim._follower is person and sim._ticks == ticks
    assert sim.playing


def test_actor_footprint_blocks_measured_cells_and_physics_guard_never_routes_with_truth(fast_scan):
    sim = PresentationSimulator(seed=1)
    person = person_fixture(sim, "mobile")
    sim.control("play")
    exclusions = sim._observed_person_exclusions()
    assert tuple(person["position"]) in exclusions
    assert sim._mission.home not in exclusions
    # Test the independent simulator interlock with a known free attempted move.
    prior = sim._mission.position
    sim._mission.tick(motion_guard=lambda _cell: False)
    assert sim._mission.position == prior
    assert sim._mission.phase == "blocked"
    assert "physical clearance interlock" in sim._mission.reason
