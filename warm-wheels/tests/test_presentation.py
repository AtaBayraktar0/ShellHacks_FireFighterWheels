import copy
import json
import math
from unittest.mock import Mock

import numpy as np
import pytest
from fastapi.testclient import TestClient

from rover.app import create_app
from rover.presentation import PresentationSimulator, SCENARIO_IDS
from rover.runtime import RoverRuntime


TOKEN = "presentation-test-token-only"
AUTH = {"Authorization": f"Bearer {TOKEN}"}


def finish(sim, limit=1200):
    history = []
    state = sim.snapshot()
    for _ in range(limit):
        before = state
        state = sim.tick(.15)
        history.append(state)
        col, row = state["position"]
        old_col, old_row = before["position"]
        if state["position"] != before["position"]:
            assert abs(col - old_col) + abs(row - old_row) == 1
            assert before["grid"]["cells"][row * before["grid"]["width"] + col] == 0
        for entity in state["entities"]:
            ec, er = entity["position"]
            assert state["grid"]["cells"][er * state["grid"]["width"] + ec] != -1
            assert entity["source"] == "scripted synthetic entity"
            assert "age" not in entity and "consciousness" not in entity
        if state["playback_status"] == "finished":
            return state, history
    pytest.fail("Presentation did not terminate within bounded ticks")


@pytest.mark.parametrize("scenario", SCENARIO_IDS)
def test_presentation_scenarios_produce_honest_outcomes(scenario):
    sim = PresentationSimulator(scenario)
    sim.control("play")
    result, history = finish(sim)
    assert result["simulation_only"]
    assert result["outcome"]["scenario_applicability"] == "demonstrated"
    if scenario == "blocked_exit":
        assert result["phase"] == "blocked"
        assert not result["outcome"]["returned_home"]
        assert result["position"] != result["home"]
        assert "before exit obstruction" in result["metrics"]["coverage_basis"]
    else:
        assert result["phase"] == "complete"
        assert result["outcome"]["returned_home"]
        assert result["position"] == result["home"]
    if scenario == "empty_search":
        assert result["metrics"]["detections"] == 0
        assert result["metrics"]["coverage_percent"] == 100
    elif scenario == "person_found":
        assert result["metrics"]["people_found"] == 1
    elif scenario == "hazard_reroute":
        assert result["metrics"]["hazards_found"] == 1
        assert result["metrics"]["replans"] >= 1
    elif scenario == "sensor_loss":
        holds = [item for item in history if item["phase"] == "sensor_hold"]
        assert len(holds) >= 20
        assert len({tuple(item["position"]) for item in holds}) == 1
        assert 0 < result["metrics"]["coverage_percent"] < 100
    elif scenario == "needs_assistance":
        assert result["metrics"]["assistance_alerts"] == 1
        assert result["alerts"][0]["delivered_at_home"]
        assert "scripted actor" in result["alerts"][0]["source"]
        assert result["metrics"]["coverage_percent"] < 100
    elif scenario == "escort_following":
        assert result["metrics"]["escort_verified"]
        assert result["metrics"]["follower_holds"] >= 1
        assert result["follower"]["at_launch_zone"]
        assert result["follower"]["visible"]
        assert result["metrics"]["assistance_alerts"] == 0
        assert any(item["phase"] == "escort_wait" for item in history)
        trail = result["follower"]["trail"]
        assert len(trail) > 5
        assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(trail, trail[1:]))
        assert len(sim.report()["following_log"]) >= result["metrics"]["following_checks"]
    elif scenario == "people_and_fire":
        assert result["metrics"]["people_found"] == 1
        assert result["metrics"]["hazards_found"] == 1
        assert result["metrics"]["replans"] >= 1
    report = sim.report()
    assert not report["hardware_test_performed"]
    assert report["trajectory"] == result["trail"]
    json.dumps(report, allow_nan=False)


def test_pause_reset_and_speed_controls_are_deterministic():
    sim = PresentationSimulator("empty_search")
    sim.control("play")
    for _ in range(12):
        sim.tick(.15)
    first = sim.snapshot()
    sim.control("pause")
    paused = sim.snapshot()
    assert sim.tick(1.0) == paused
    sim.control("reset")
    assert sim.snapshot()["phase"] == "idle"
    assert sim.snapshot()["metrics"]["steps"] == 0
    sim.control("play")
    for _ in range(12):
        sim.tick(.15)
    assert sim.snapshot() == first
    sim.control("speed", speed=4)
    before = sim.snapshot()["metrics"]["steps"]
    sim.tick(.15)
    assert sim.snapshot()["metrics"]["steps"] == before + 4
    for speed in (0, 4.1, float("nan"), True):
        with pytest.raises(ValueError):
            sim.control("speed", speed=speed)


def test_seed_changes_real_geometry_and_reset_preserves_exact_fixture():
    sim = PresentationSimulator("people_and_fire")
    first = sim.control("randomize", seed=17)
    first_fixture = sim._mission._initial_fixture.copy()
    assert first["seed"] == 17 and not first["playing"]
    sim.control("play")
    for _ in range(12):
        sim.tick(.15)
    prefix = copy.deepcopy(sim.snapshot()["trail"])
    sim.control("reset")
    assert sim.snapshot()["seed"] == 17
    assert sim.snapshot()["layout_hash"] == first["layout_hash"]
    assert np.array_equal(sim._mission._initial_fixture, first_fixture)
    sim.control("play")
    for _ in range(12):
        sim.tick(.15)
    assert sim.snapshot()["trail"] == prefix
    second = sim.control("randomize", seed=18)
    assert second["layout_hash"] != first["layout_hash"]
    assert not np.array_equal(sim._mission._initial_fixture, first_fixture)
    assert second["metrics"]["steps"] == 0
    selected = sim.control("select", scenario="empty_search")
    assert selected["seed"] == 18
    assert selected["layout_hash"] == second["layout_hash"]
    assert sim.report()["seed"] == 18


def test_randomize_rejects_invalid_seeds_without_changing_current_layout():
    sim = PresentationSimulator()
    prior = sim.snapshot()["layout_hash"]
    for seed in (-1, 2147483648, True, 1.2, "1"):
        with pytest.raises(ValueError):
            sim.control("randomize", seed=seed)
        assert sim.snapshot()["layout_hash"] == prior
    state = sim.control("randomize")
    assert 0 <= state["seed"] <= 2147483647


@pytest.mark.parametrize("width,height", [(1, 1), (50, 50), (1, 50), (50, 1), (3, 7)])
@pytest.mark.parametrize("scenario", SCENARIO_IDS)
def test_metric_boundary_contains_every_possible_robot_footprint(width, height, scenario):
    sim = PresentationSimulator(scenario)
    boundary_state = sim.control("boundary", width_m=width, height_m=height)
    assert not boundary_state["playing"] and boundary_state["phase"] == "idle"
    assert boundary_state["boundary"]["enforced"]
    assert not boundary_state["boundary"]["hardware_supported"]
    grid = boundary_state["grid"]
    assert grid["width"] <= 80 and grid["height"] <= 80
    resolution = grid["resolution_m"]
    # Check the entire traversable configuration space, not only sampled steps.
    rows, cols = np.where(sim._mission._initial_fixture == 0)
    centers_x, centers_y = (cols + .5) * resolution, (rows + .5) * resolution
    assert np.all(centers_x - .2 >= -1e-9)
    assert np.all(centers_x + .2 <= width + 1e-9)
    assert np.all(centers_y - .2 >= -1e-9)
    assert np.all(centers_y + .2 <= height + 1e-9)
    sim.control("play")
    for _ in range(15):
        state = sim.tick(.15)
        col, row = state["position"]
        assert .2 <= (col + .5) * resolution <= width - .2 + 1e-9
        assert .2 <= (row + .5) * resolution <= height - .2 + 1e-9


def test_untriggered_fault_before_completion_is_not_claimed_as_success():
    sim = PresentationSimulator("sensor_loss")
    sim.control("boundary", width_m=1, height_m=1)
    # A fixture whose event is scheduled beyond mission completion cannot
    # truthfully claim that the fault response was demonstrated.
    sim._trigger_step = 1000
    sim.control("play")
    result, _ = finish(sim)
    assert result["outcome"]["scenario_applicability"] == "not_applicable"
    assert result["metrics"]["replans"] == 0
    assert "not applicable" in result["outcome"]["summary"]


def test_boundary_change_resets_active_simulation_and_rejects_bad_dimensions():
    sim = PresentationSimulator("person_found")
    sim.control("play")
    sim.tick(.3)
    changed = sim.control("boundary", width_m=8, height_m=4)
    assert changed["scenario"]["id"] == "person_found"
    assert changed["phase"] == "idle" and not changed["playing"]
    assert changed["metrics"]["steps"] == 0 and changed["entities"] == []
    for width, height in ((0, 5), (51, 5), (1, float("inf")), (None, 4), (True, 4)):
        with pytest.raises(ValueError):
            sim.control("boundary", width_m=width, height_m=height)


def test_failed_follower_reacquisition_returns_and_alerts_without_claiming_escort():
    sim = PresentationSimulator("escort_following")
    sim.control("play")
    for _ in range(200):
        state = sim.tick(.15)
        if state["follower"] is not None:
            break
    assert state["follower"] is not None
    # Extend the scripted actor's occlusion beyond the bounded reacquire limit.
    sim._follower_pause_triggered = True
    sim._follower_pause_ticks = 100
    result, _ = finish(sim)
    assert result["outcome"]["returned_home"]
    assert not result["metrics"]["escort_verified"]
    assert result["follower"]["state"] == "lost"
    assert result["metrics"]["assistance_alerts"] == 1
    assert "follower not reacquired" in result["outcome"]["summary"]


def test_presentation_endpoints_require_auth_and_never_access_motor_interface():
    runtime = RoverRuntime()
    runtime.motor = Mock()
    with TestClient(create_app(runtime, TOKEN, manage_runtime=False)) as client:
        assert client.get("/api/health").json() == {"product": "warm-wheels", "mode": "demo", "version": "procedural-v2"}
        assert client.get("/api/presentation").status_code == 401
        assert client.post("/api/presentation/control", json={"action": "play"}).status_code == 401
        assert client.get("/api/presentation/report").status_code == 401
        assert client.post("/api/presentation/control", headers=AUTH, json={"action": "play"}).status_code == 200
        assert client.get("/api/presentation", headers=AUTH).json()["playing"]
        assert client.post("/api/presentation/control", headers=AUTH, json={"action": "boundary", "width_m": 4, "height_m": 8}).json()["phase"] == "idle"
        report = client.get("/api/presentation/report", headers=AUTH)
        assert report.status_code == 200 and "attachment" in report.headers["content-disposition"]
        assert report.json()["simulation_only"]
    assert runtime.motor.mock_calls == []
    assert not runtime.gate.armed


def test_hardware_mode_rejects_every_presentation_endpoint():
    runtime = RoverRuntime(mode="hardware")
    runtime.motor = Mock()
    with TestClient(create_app(runtime, TOKEN, manage_runtime=False)) as client:
        assert client.get("/api/presentation", headers=AUTH).status_code == 409
        assert client.post("/api/presentation/control", headers=AUTH, json={"action": "play"}).status_code == 409
        assert client.get("/api/presentation/report", headers=AUTH).status_code == 409
    assert runtime.motor.mock_calls == []


@pytest.mark.parametrize("body", [
    {"action": "speed", "speed": 5}, {"action": "boundary", "width_m": 0, "height_m": 5},
    {"action": "select", "scenario": "invented"}, {"action": "play", "extra": 1},
    {"action": "boundary", "width_m": True, "height_m": 5},
    {"action": "randomize", "seed": True}, {"action": "randomize", "seed": -1},
])
def test_presentation_payload_validation(body):
    with TestClient(create_app(RoverRuntime(), TOKEN, manage_runtime=False)) as client:
        assert client.post("/api/presentation/control", headers=AUTH, json=body).status_code == 422
