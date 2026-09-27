"""Procedural laptop worlds, isolated from every physical motor interface.

Default missions combine seeded geometry, evolving fire, simulated people and
faults. Robot decisions use observations and the live frontier/return planner.
Explicit legacy scenario IDs remain available for offline regression only.
"""
from __future__ import annotations

import copy
from collections import deque
import hashlib
import math
import random
import secrets

import numpy as np

from .mission import MissionSimulator, _ray


SCENARIOS = (
    {"id": "empty_search", "title": "Search an empty home",
     "description": "Explore all reachable rooms and return without needing a person or fire detection.",
     "expected_outcome": "Returns to launch after reachable exploration."},
    {"id": "person_found", "title": "Find a person marker",
     "description": "Reveal a scripted person marker when its cell is observed; continue mapping and return.",
     "expected_outcome": "Reports the synthetic person location and returns; assistance remains unassessed."},
    {"id": "hazard_reroute", "title": "Reroute around a hazard",
     "description": "A scripted hazard blocks a known route; the planner must choose another path.",
     "expected_outcome": "Avoids the new occupied cells, continues reachable search, and returns."},
    {"id": "blocked_exit", "title": "Report a blocked exit",
     "description": "The known launch area becomes obstructed during the return leg.",
     "expected_outcome": "Stops with a blocked outcome and never claims a successful return."},
    {"id": "sensor_loss", "title": "Hold when sensing drops",
     "description": "A scripted sensor gap freezes simulated motion; recovery triggers an early return.",
     "expected_outcome": "Holds position during the gap and returns with an explicitly partial scan."},
    {"id": "needs_assistance", "title": "Return with an assistance alert",
     "description": "A scripted actor signals that assistance is needed. The rover returns with the observed location.",
     "expected_outcome": "Returns to launch and delivers an assistance alert; no medical condition is inferred."},
    {"id": "escort_following", "title": "Escort and verify following",
     "description": "A scripted mobile actor follows the rover. A temporary follower pause makes the rover wait and reacquire.",
     "expected_outcome": "Returns with the actor verified in the launch zone, logging every following check."},
    {"id": "people_and_fire", "title": "People and fire in one mission",
     "description": "A scripted person is discovered while a fire exclusion zone changes the observed route.",
     "expected_outcome": "Reports both synthetic entities, replans around the fire zone, and returns."},
)
SCENARIO_IDS = tuple(item["id"] for item in SCENARIOS)
ASSUMPTIONS = [
    "Laptop simulation only; no Arduino, motors, or physical camera are controlled.",
    "Robot routes are calculated by frontier exploration on observed free cells.",
    "Legacy offline fixtures use an ideal 360-degree grid model; the procedural 3D theater uses a forward camera mesh.",
    "Person/hazard markers and fault events are scripted synthetic fixtures, not model predictions.",
    "The scenario author checks hidden-fixture connectivity when placing scripted obstacles; robot route planning still uses observations only.",
    "Coverage is a hidden-fixture diagnostic, not proof that a real building was fully scanned.",
    "No age, consciousness, mobility, or medical condition is inferred.",
]


def _bounded_fixture(width_m, height_m, seed=0):
    """Configuration-space rectangle: each free center includes 0.20 m radius.

    Cell extents may round past the requested boundary; those centers are not
    traversable unless their complete robot footprint fits inside the exact
    metric rectangle. Thin 1x50 m regions therefore work without a fake scale.
    """
    resolution = max(.1, min(.2, width_m / 10, height_m / 10), width_m / 80, height_m / 80)
    width, height = math.ceil(width_m / resolution), math.ceil(height_m / resolution)
    radius = .20
    rng = random.Random(seed)
    x = (np.arange(width) + .5) * resolution
    y = (np.arange(height) + .5) * resolution
    inside = ((x[None, :] >= radius) & (x[None, :] <= width_m - radius)
              & (y[:, None] >= radius) & (y[:, None] <= height_m - radius))
    cells = np.where(inside, 0, 1).astype(np.int8)
    padding = max(1, math.ceil(radius / resolution))
    if width_m >= 4 and height_m >= 4:
        raw = np.zeros_like(cells)
        partition_row = int(height * rng.uniform(.44, .56))
        partition_col = int(width * rng.uniform(.44, .56))
        raw[partition_row, :] = 1
        raw[:, partition_col] = 1
        door_half = max(padding + 1, math.ceil(.6 / resolution))
        for fraction in (.25, .75):
            center_col = int(width * (fraction + rng.uniform(-.06, .06)))
            center_row = int(height * (fraction + rng.uniform(-.06, .06)))
            raw[partition_row, max(0, center_col - door_half):min(width, center_col + door_half + 1)] = 0
            raw[max(0, center_row - door_half):min(height, center_row + door_half + 1), partition_col] = 0
        for row, col in np.argwhere(raw == 1):
            cells[max(0, row - padding):min(height, row + padding + 1),
                  max(0, col - padding):min(width, col + padding + 1)] = 1
    free = np.argwhere(cells == 0)
    if not len(free):
        raise ValueError("Selected area cannot fit the simulated rover footprint")
    target_col, target_row = width_m * .15 / resolution, height_m * .85 / resolution
    row, col = min(free, key=lambda cell: (cell[1] + .5 - target_col) ** 2 + (cell[0] + .5 - target_row) ** 2)
    home = (int(col), int(row))
    # Add seeded solid furniture only when it preserves a connected remaining
    # free region and does not overlap launch. These are geometry changes, not
    # decorative colors; every accepted object enters occupancy and sensing.
    desired = min(12, int(width_m * height_m / 4))
    accepted = 0
    for _ in range(desired * 12):
        if accepted >= desired:
            break
        row, col = free[rng.randrange(len(free))]
        row, col = int(row), int(col)
        if max(abs(col - home[0]), abs(row - home[1])) <= 2 * padding + 2:
            continue
        x1, x2 = max(0, col - padding), min(width, col + padding + 1)
        y1, y2 = max(0, row - padding), min(height, row + padding + 1)
        candidate = cells.copy()
        candidate[y1:y2, x1:x2] = 1
        if np.array_equal(candidate, cells):
            continue
        remaining = int(np.count_nonzero(candidate == 0))
        if remaining < 8 or len(MissionSimulator._reachable(candidate, home)) != remaining:
            continue
        cells = candidate
        accepted += 1
    return cells, home, resolution


class LegacyPresentationSimulator:
    STEP_PERIOD_S = 0.15  # Playback pacing; logical simulation time advances 0.1 s/move tick.

    def __init__(self, scenario="empty_search"):
        self.speed = 1.0
        self.seed = 0
        self.width_m = self.height_m = 6.0
        self.reset(scenario)

    def reset(self, scenario=None):
        chosen = scenario or getattr(self, "scenario_id", "empty_search")
        if chosen not in SCENARIO_IDS:
            raise ValueError("Unknown presentation scenario")
        self.scenario_id = chosen
        self.playing = False
        self._logical_time = 0.0
        self._accumulator = 0.0
        fixture, home, resolution = _bounded_fixture(self.width_m, self.height_m, self.seed)
        self._layout_hash = hashlib.sha256(fixture.tobytes() + repr((fixture.shape, self.width_m, self.height_m, resolution)).encode()).hexdigest()
        self._resolution = resolution
        self._clearance_cells = max(1, math.ceil(.2 / resolution))
        self._mission = MissionSimulator(clock=lambda: self._logical_time, fixture=fixture,
                                         home=home, resolution_m=resolution,
                                         max_exploration_steps=15000, max_duration_s=100000)
        free = [(int(col), int(row)) for row, col in np.argwhere(fixture == 0)]
        ordered = sorted(free, key=lambda cell: abs(cell[0] - home[0]) + abs(cell[1] - home[1]), reverse=True)
        candidates = ordered[:max(1, len(ordered) // 4)]
        random.Random(self.seed + 731).shuffle(candidates)
        self._person_position = ordered[0]
        for col, row in candidates + ordered:
            if max(abs(col - home[0]), abs(row - home[1])) <= self._clearance_cells:
                continue
            if chosen == "escort_following":
                self._person_position = (col, row)
                break
            prospective = fixture.copy()
            prospective[max(0, row - self._clearance_cells):min(fixture.shape[0], row + self._clearance_cells + 1),
                        max(0, col - self._clearance_cells):min(fixture.shape[1], col + self._clearance_cells + 1)] = 1
            if len(MissionSimulator._reachable(prospective, home)) == int(np.count_nonzero(prospective == 0)):
                self._person_position = (col, row)
                break
        self._trigger_step = min(35, max(2, int(len(free) * .07)))
        self._trail = [list(self._mission.home)]
        self._heading = -math.pi / 2
        self._events = []
        self._entities = []
        self._revealed_ids = set()
        self._triggered = False
        self._sensor_hold = 0.0
        self._sensor_holds = 0
        self._replans = 0
        self._pending_route_comparison = None
        self._coverage_before_exit_block = None
        self._outcome_recorded = False
        self._assistance_location = None
        self._alerts = []
        self._behavior = "SEARCH"
        self._follower_position = None
        self._follower_trail = []
        self._following_log = []
        self._escort_active = False
        self._escort_wait = False
        self._escort_wait_ticks = 0
        self._escort_moves = 0
        self._follower_holds = 0
        self._reacquisitions = 0
        self._follower_pause_ticks = 0
        self._follower_pause_triggered = False
        self._follower_visible = False
        self._following_verified = False
        self._escort_failed = False
        self._escort_verified = False
        return self.snapshot()

    def _event(self, category, title, message, position=None):
        event = {"id": len(self._events) + 1, "step": self._mission.steps,
                 "sim_time_s": round(self._logical_time, 1), "type": category,
                 "title": title, "message": message}
        if position is not None:
            event["position"] = list(position)
        self._events.append(event)

    def _start(self):
        self._mission.start()
        if self.scenario_id in ("person_found", "needs_assistance", "escort_following", "people_and_fire"):
            if self.scenario_id != "escort_following":
                self._mission.set_obstacle(*self._person_position, observed=False, clearance_cells=self._clearance_cells)
            col, row = self._person_position
            visible_cells = [[x, y] for y in range(max(0, row - self._clearance_cells), min(self._mission.height, row + self._clearance_cells + 1))
                             for x in range(max(0, col - self._clearance_cells), min(self._mission.width, col + self._clearance_cells + 1))]
            if self.scenario_id == "escort_following":
                visible_cells = [list(self._person_position)]
            self._entities.append({"id": "synthetic-person-1", "kind": "person", "position": list(self._person_position),
                                   "label": "Person marker · assistance unassessed",
                                   "source": "scripted synthetic entity", "_visibility_cells": visible_cells})
        self._event("mission", "Search started", "Frontier exploration is choosing the route from observed free space.", self._mission.home)

    def control(self, action, *, scenario=None, speed=None, width_m=None, height_m=None, seed=None):
        if action == "randomize":
            if any(value is not None for value in (scenario, speed, width_m, height_m)):
                raise ValueError("Randomize accepts only an optional seed")
            if seed is not None and (type(seed) is not int or not 0 <= seed <= 2147483647):
                raise ValueError("Seed must be an integer from 0 to 2147483647")
            self.seed = secrets.randbelow(2147483648) if seed is None else seed
            return self.reset()
        if seed is not None:
            raise ValueError("Seed is only accepted for the randomize action")
        if action == "boundary":
            if scenario is not None or speed is not None:
                raise ValueError("Set scenario and speed separately from the boundary")
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 1 <= value <= 50 for value in (width_m, height_m)):
                raise ValueError("Boundary width and height must both be finite metres from 1 to 50")
            self.width_m, self.height_m = float(width_m), float(height_m)
            return self.reset()
        if width_m is not None or height_m is not None:
            raise ValueError("Width and height are only accepted for the boundary action")
        if action == "select":
            if scenario is None:
                raise ValueError("Select requires a scenario ID")
            if speed is not None:
                raise ValueError("Set playback speed with the speed action")
            return self.reset(scenario)
        if scenario is not None:
            raise ValueError("Scenario is only accepted for the select action")
        if action == "speed":
            if isinstance(speed, bool) or not isinstance(speed, (float, int)) or not math.isfinite(speed) or not .25 <= speed <= 4:
                raise ValueError("Playback speed must be between 0.25 and 4")
            self.speed = float(speed)
        elif speed is not None:
            raise ValueError("Speed is only accepted for the speed action")
        elif action in ("reset", "restart"):
            return self.reset()
        elif action == "play":
            if self._mission.phase in ("complete", "blocked"):
                self.reset()
            if self._mission.phase == "idle":
                self._start()
            self.playing = True
        elif action == "pause":
            self.playing = False
            self._accumulator = 0.0
        else:
            raise ValueError("Unknown presentation action")
        return self.snapshot()

    def _reveal_entities(self, state):
        width = state["grid"]["width"]
        for entity in self._entities:
            candidates = entity.get("_visibility_cells", [entity["position"]])
            visible = [cell for cell in candidates if state["grid"]["cells"][cell[1] * width + cell[0]] != -1]
            if entity["id"] not in self._revealed_ids and visible:
                # Marker reports an observed edge of its scripted exclusion
                # footprint, never an unseen object center behind that edge.
                entity["position"] = min(visible, key=lambda cell: abs(cell[0] - state["position"][0]) + abs(cell[1] - state["position"][1]))
                self._revealed_ids.add(entity["id"])
                entity["first_seen_step"] = state["steps"]
                title = {"person": "Person marker observed", "hazard": "Hazard zone observed",
                         "obstruction": "Exit obstruction reported"}[entity["kind"]]
                self._event("discovery", title, entity["label"] + ". Scripted synthetic entity; not a detector result.", entity["position"])
                if entity["kind"] == "person" and self.scenario_id == "needs_assistance":
                    self._assistance_location = list(entity["position"])
                    self._behavior = "REPORTING_ASSISTANCE"
                    self._event("decision", "Assistance response received — returning to report",
                                "The scenario supplies an assistance-needed actor response. This is not an age, mobility, or medical diagnosis.", entity["position"])
                    self._mission.abort("scripted actor requests assistance; returning with observed location")
                elif entity["kind"] == "person" and self.scenario_id == "escort_following":
                    self._escort_active = True
                    self._follower_position = list(entity["position"])
                    self._follower_trail = [list(entity["position"])]
                    self._behavior = "ESCORT_RETURN"
                    self._event("decision", "Mobile actor response received — escort initiated",
                                "The scenario supplies a willing mobile actor. Following distance and visibility must remain verified.", entity["position"])
                    self._mission.abort("scripted mobile actor following; escort to launch")

    def _actor_path(self, grid, target):
        start = tuple(self._follower_position)
        target = tuple(target)
        queue, parents = deque([start]), {start: None}
        while queue:
            col, row = queue.popleft()
            if (col, row) == target:
                path = [(col, row)]
                while parents[path[-1]] is not None:
                    path.append(parents[path[-1]])
                return path[::-1]
            for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
                node = (col + dx, row + dy)
                if (0 <= node[0] < grid["width"] and 0 <= node[1] < grid["height"]
                        and grid["cells"][node[1] * grid["width"] + node[0]] == 0 and node not in parents):
                    parents[node] = (col, row)
                    queue.append(node)
        return []

    def _follower_measurement(self, state):
        distance = math.dist(self._follower_position, state["position"]) * self._resolution
        grid = state["grid"]
        visible = (self._follower_pause_ticks == 0 and distance <= 6 * self._resolution
                   and all(grid["cells"][row * grid["width"] + col] == 0
                           for col, row in _ray(tuple(self._follower_position), tuple(state["position"]))))
        verified = visible and distance <= max(.6, 3 * self._resolution) + 1e-9
        self._follower_visible, self._following_verified = visible, verified
        entry = {"step": state["steps"], "sim_time_s": round(self._logical_time, 1),
                 "rover": list(state["position"]), "person": list(self._follower_position),
                 "distance_m": round(distance, 3), "visible": visible, "verified": verified}
        self._following_log.append(entry)
        return entry

    def _at_launch_zone(self):
        return (self._follower_position is not None
                and math.dist(self._follower_position, self._mission.home) * self._resolution <= max(.4, self._resolution) + 1e-9
                and self._follower_visible)

    def _update_follower(self, state):
        if not self._escort_active:
            return True
        if not self._follower_pause_triggered and self._escort_moves >= 8:
            self._follower_pause_triggered = True
            self._follower_pause_ticks = 10
            self._event("follower", "Scripted follower pauses and becomes occluded",
                        "The actor stops for one logical second. Following verification must stop rover progress.", self._follower_position)
        if self._follower_pause_ticks > 0:
            self._follower_pause_ticks -= 1
        else:
            path = self._actor_path(state["grid"], state["position"])
            # A short path can still turn around a wall corner. Move closer
            # when line of sight is lost rather than waiting forever at a
            # nominal trailing distance on the far side of that corner.
            stop_cells = 1 if state["position"] == state["home"] or not self._follower_visible else 2
            if len(path) > stop_cells + 1:
                self._follower_position = list(path[1])
                self._follower_trail.append(list(path[1]))
        for entity in self._entities:
            if entity["kind"] == "person":
                entity["position"] = list(self._follower_position)
        measurement = self._follower_measurement(state)
        if not measurement["verified"]:
            if not self._escort_wait:
                self._follower_holds += 1
                self._event("follower", "Waiting for verified following", "Rover holds position until the scripted actor is visible and within the following limit.", state["position"])
            self._escort_wait = True
            self._behavior = "ESCORT_WAIT"
            self._escort_wait_ticks += 1
            if self._escort_wait_ticks > 50:
                self._escort_active = False
                self._escort_failed = True
                self._assistance_location = list(self._follower_position)
                self._behavior = "REPORTING_ASSISTANCE"
                self._event("decision", "Follower not reacquired — returning to alert",
                            "Following was not verified within the simulation timeout. Last observed actor location retained.", self._follower_position)
                self._mission.abort("follower not reacquired; return with last observed location")
                self._escort_wait = False
                return True
            return False
        if self._escort_wait:
            self._reacquisitions += 1
            self._event("follower", "Following reacquired", "Visibility and distance are verified; escort may continue.", self._follower_position)
        self._escort_wait = False
        self._escort_wait_ticks = 0
        self._behavior = "ESCORT_RETURN"
        return True

    def _inject_scenario_event(self, state):
        if self._triggered:
            return
        if self.scenario_id in ("hazard_reroute", "people_and_fire") and state["steps"] >= self._trigger_step:
            route = state["route"]
            # Pick an actually planned, already observed future cell. Keep the
            # inflated fixture outside the rover's present occupied footprint.
            selected = None
            for cell in route[2:]:
                if max(abs(cell[0] - state["position"][0]), abs(cell[1] - state["position"][1])) <= self._clearance_cells:
                    continue
                observed = np.array(state["grid"]["cells"], dtype=np.int8).reshape(state["grid"]["height"], state["grid"]["width"])
                col, row = cell
                observed[max(0, row - self._clearance_cells):min(observed.shape[0], row + self._clearance_cells + 1),
                         max(0, col - self._clearance_cells):min(observed.shape[1], col + self._clearance_cells + 1)] = 1
                # Scenario construction may inspect its hidden fixture to keep
                # this designated reroute case feasible. The rover's actual
                # path decision still receives only the observed grid above.
                fixture = self._mission._truth.copy()
                fixture[max(0, row - self._clearance_cells):min(fixture.shape[0], row + self._clearance_cells + 1),
                        max(0, col - self._clearance_cells):min(fixture.shape[1], col + self._clearance_cells + 1)] = 1
                stays_connected = len(MissionSimulator._reachable(fixture, tuple(state["home"]))) == int(np.count_nonzero(fixture == 0))
                if stays_connected and tuple(state["home"]) in MissionSimulator._reachable(observed, tuple(state["position"])):
                    selected = cell
                    break
            if selected is None:
                return
            self._pending_route_comparison = copy.deepcopy(route)
            self._mission.set_obstacle(*selected, observed=True, clearance_cells=self._clearance_cells)
            self._entities.append({"id": "synthetic-hazard-1", "kind": "hazard", "position": list(selected),
                                   "label": "Scripted fire exclusion zone" if self.scenario_id == "people_and_fire" else "Scripted hazard exclusion zone",
                                   "source": "scripted synthetic entity"})
            self._triggered = True
            self._event("replan", "Route changed by a new obstacle", "An observed route cell became occupied; the next step must be replanned.", selected)
        elif (self.scenario_id == "blocked_exit" and state["phase"] == "returning"
              and max(abs(state["position"][0] - self._mission.home[0]), abs(state["position"][1] - self._mission.home[1])) > self._clearance_cells):
            self._coverage_before_exit_block = state["coverage_percent"]
            self._mission.set_obstacle(*self._mission.home, observed=True, clearance_cells=self._clearance_cells)
            self._entities.append({"id": "synthetic-exit-block", "kind": "obstruction", "position": list(self._mission.home),
                                   "label": "Scripted obstruction at the known launch area", "source": "scripted synthetic entity"})
            self._triggered = True
            self._event("replan", "Launch area obstructed", "A synthetic occupancy update blocks the known exit. Return must fail explicitly.", self._mission.home)
        elif self.scenario_id == "sensor_loss" and state["steps"] >= max(2, self._trigger_step + 10):
            self._triggered = True
            self._sensor_hold = 2.0
            self._sensor_holds += 1
            self._event("sensor", "Sensor data missing — holding position", "Scripted two-second data gap. No simulated motion is allowed during this hold.", state["position"])

    def tick(self, dt_s=.1):
        """Advance playback without sleeping. Runtime supplies elapsed wall time."""
        if not math.isfinite(dt_s) or not 0 <= dt_s <= 1:
            raise ValueError("Presentation tick interval must be finite and in 0..1 seconds")
        if not self.playing:
            return self.snapshot()
        self._accumulator += dt_s * self.speed
        while self._accumulator + 1e-9 >= self.STEP_PERIOD_S and self.playing:
            self._accumulator = max(0.0, self._accumulator - self.STEP_PERIOD_S)
            self._logical_time = round(self._logical_time + .1, 6)
            self._advance_one()
        return self.snapshot()

    def _advance_one(self):
        if self._sensor_hold > 0:
            self._sensor_hold = round(max(0.0, self._sensor_hold - .1), 6)
            if self._sensor_hold == 0:
                self._event("sensor", "Sensor stream restored", "Synthetic sensing recovered. Returning via known free space; reported coverage shows any remaining unobserved region.")
                self._mission.abort("synthetic sensor recovered; early return after data gap")
            return
        before = self._mission.snapshot()
        if not self._update_follower(before):
            return
        self._inject_scenario_event(before)
        self._reveal_entities(self._mission.snapshot())
        if self._sensor_hold > 0:
            return
        state = self._mission.tick()
        if state["position"] != before["position"]:
            dx = state["position"][0] - before["position"][0]
            dy = state["position"][1] - before["position"][1]
            self._heading = math.atan2(dy, dx)
            self._trail.append(state["position"])
            if self._escort_active:
                self._escort_moves += 1
        if self._pending_route_comparison is not None:
            if state["route"] != self._pending_route_comparison[1:]:
                self._replans += 1
                self._event("replan", "Alternative route selected", "The live planner selected a route that avoids the new occupied cells.", state["position"])
            self._pending_route_comparison = None
        self._reveal_entities(state)
        state = self._mission.snapshot()
        if self._escort_active:
            self._follower_measurement(state)
        if state["phase"] == "returning" and before["phase"] != "returning":
            self._event("mission", "Returning to launch", state["reason"], state["position"])
        if state["phase"] in ("complete", "blocked"):
            if state["phase"] == "complete" and self._escort_active:
                self._follower_measurement(state)
                if not self._at_launch_zone():
                    self._escort_wait = True
                    self._behavior = "ESCORT_WAIT"
                    return
                self._escort_verified = True
            self.playing = False
            if self.scenario_id in ("hazard_reroute", "people_and_fire", "blocked_exit", "sensor_loss") and not self._triggered:
                self._event("scenario", "Fault scenario not applicable to this area", "The selected boundary finished before a separate, non-overlapping fault location or trigger was available. Increase the area; no fault was fabricated.")
            if not self._outcome_recorded:
                self._outcome_recorded = True
                if state["phase"] == "complete" and self._assistance_location is not None:
                    self._alerts.append({"kind": "assistance_needed", "position": list(self._assistance_location),
                                         "delivered_at_home": True, "step": state["steps"],
                                         "source": "scripted actor response in simulation"})
                    self._event("alert", "Assistance alert delivered at launch", "Observed actor location is ready to report to the outside team. No medical state was inferred.", self._assistance_location)
                if self._escort_verified:
                    self._event("follower", "Follower verified in the launch zone", "The actor arrived by consecutive grid moves and is visible within the launch-zone distance.", self._follower_position)
                self._behavior = "COMPLETE" if state["phase"] == "complete" else "BLOCKED"
                self._event("outcome", "Returned to launch" if state["phase"] == "complete" else "Return blocked",
                            state["reason"], state["position"])

    def _frontiers(self, grid):
        cells, width, height = grid["cells"], grid["width"], grid["height"]
        result = []
        for row in range(height):
            for col in range(width):
                if cells[row * width + col] == 0 and any(
                        0 <= col + dx < width and 0 <= row + dy < height and
                        cells[(row + dy) * width + col + dx] == -1
                        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0))):
                    result.append([col, row])
        return result

    def snapshot(self):
        state = self._mission.snapshot()
        grid = {**state["grid"], "frame": "synthetic-presentation"}
        phase = "sensor_hold" if self._sensor_hold > 0 else "escort_wait" if self._escort_wait and self.playing else state["phase"]
        entities = [{key: copy.deepcopy(value) for key, value in entity.items() if not key.startswith("_")}
                    for entity in self._entities if entity["id"] in self._revealed_ids]
        coverage = self._coverage_before_exit_block if self._coverage_before_exit_block is not None else state["coverage_percent"]
        coverage_basis = ("Coverage recorded immediately before exit obstruction changed connectivity"
                          if self._coverage_before_exit_block is not None else state["coverage_basis"])
        completed = (state["phase"] == "complete" and state["position"] == state["home"]
                     and (not self._escort_active or self._escort_verified))
        blocked = state["phase"] == "blocked"
        not_applicable = (completed or blocked) and self.scenario_id in ("hazard_reroute", "people_and_fire", "blocked_exit", "sensor_loss") and not self._triggered
        metrics = {"steps": state["steps"], "coverage_percent": coverage,
                   "unobserved_cells": state["unobserved_cells"], "detections": len(entities),
                   "people_found": sum(entity["kind"] == "person" for entity in entities),
                   "hazards_found": sum(entity["kind"] == "hazard" for entity in entities),
                   "replans": self._replans, "sensor_holds": self._sensor_holds,
                   "elapsed_sim_s": round(self._logical_time, 1), "route_length_cells": max(0, len(state["route"]) - 1),
                   "coverage_basis": coverage_basis}
        metrics.update({"follower_holds": self._follower_holds, "following_checks": len(self._following_log),
                        "follower_reacquisitions": self._reacquisitions, "escort_verified": self._escort_verified,
                        "assistance_alerts": len(self._alerts)})
        outcome = {"status": "returned" if completed else "blocked" if blocked else "pending",
                   "title": "Returned to launch" if completed else "Exit blocked — rover has not returned" if blocked else "Mission in progress" if state["phase"] != "idle" else "Ready to play",
                   "summary": state["reason"] + (" Fault scenario was not applicable to this small/short mission; enlarge the area to demonstrate it." if not_applicable else ""),
                   "returned_home": completed, "coverage_percent": coverage,
                   "scenario_applicability": "not_applicable" if not_applicable else "demonstrated" if self._triggered or self.scenario_id in ("empty_search", "person_found", "needs_assistance", "escort_following") else "pending"}
        if completed and self._alerts:
            outcome["title"] = "Returned with an assistance alert"
        elif completed and self._escort_verified:
            outcome["title"] = "Escort complete — follower verified at launch"
        follower = None
        if self._follower_position is not None:
            follower = {"position": list(self._follower_position), "visible": self._follower_visible,
                        "distance_m": round(math.dist(self._follower_position, state["position"]) * self._resolution, 3),
                        "following_verified": self._following_verified, "at_launch_zone": self._at_launch_zone(),
                        "state": "lost" if self._escort_failed else "arrived" if self._escort_verified else "waiting" if self._escort_wait else "following",
                        "source": "scripted synthetic actor", "trail": copy.deepcopy(self._follower_trail)}
        return {"simulation_only": True, "label": "SCRIPTED SYNTHETIC SCENARIO · ACTUAL FRONTIER PLANNER",
                "seed": self.seed, "layout_hash": self._layout_hash,
                "scenarios": copy.deepcopy(list(SCENARIOS)),
                "scenario": copy.deepcopy(next(item for item in SCENARIOS if item["id"] == self.scenario_id)),
                "boundary": {"width_m": self.width_m, "height_m": self.height_m, "resolution_m": self._resolution,
                             "robot_radius_m": .2, "sensor_range_m": round(6 * self._resolution, 3),
                             "enforced": True, "hardware_supported": False},
                "playing": self.playing, "speed": self.speed, "phase": phase,
                "playback_status": "finished" if completed or blocked else "playing" if self.playing else "ready" if state["phase"] == "idle" else "paused",
                "grid": grid, "position": state["position"], "home": state["home"], "heading_rad": self._heading,
                "route": state["route"], "trail": copy.deepcopy(self._trail), "frontiers": self._frontiers(grid),
                "entities": entities, "events": copy.deepcopy(self._events), "metrics": metrics, "outcome": outcome,
                "follower": follower, "alerts": copy.deepcopy(self._alerts),
                "behavior": {"state": self._behavior, "source": "scripted actor response in simulation"},
                "sensor_hold_remaining_s": self._sensor_hold, "assumptions": list(ASSUMPTIONS)}

    def report(self):
        state = self.snapshot()
        return {"schema_version": "1.0", "export_type": "warm-wheels-presentation-report",
                "seed": self.seed, "layout_hash": self._layout_hash,
                "simulation_only": True, "scenario": state["scenario"], "metrics": state["metrics"],
                "boundary": state["boundary"],
                "outcome": state["outcome"], "events": state["events"], "entities": state["entities"],
                "follower": state["follower"], "following_log": copy.deepcopy(self._following_log), "alerts": state["alerts"],
                "trajectory": state["trail"], "final_observed_grid": state["grid"],
                "assumptions": list(ASSUMPTIONS), "hardware_test_performed": False}


PROCEDURAL_SCENARIO = {
    "id": "randomized_mission", "title": "Autonomous randomized mission",
    "description": "Explore an unknown generated place, assess observed simulated actor responses, and choose a return or escort action as conditions change.",
    "expected_outcome": "Discover what can be observed; report the actual outcome and any unresolved areas or people.",
}
PROCEDURAL_ASSUMPTIONS = [
    "Laptop simulation only; no physical camera, Arduino, or motors are controlled.",
    "Geometry, people, actor responses, fire origins and faults are seeded synthetic world state.",
    "The navigation fixture uses a separate ideal occupancy observation; the 3D camera mesh is forward-facing and retains only acquired surfaces.",
    "Person responses require five visible samples; no age, consciousness or medical diagnosis is inferred.",
    "Fire spreads by bounded cardinal grid growth on physical free cells. This is a synthetic hazard model, not fire physics.",
    "The 3D scan is accumulated simulated raycast depth, not a physical D435i reconstruction.",
    "Coverage is an initial-fixture diagnostic. Return does not imply all-clear, successful rescue or a complete scan.",
]


class ProceduralPresentationSimulator(LegacyPresentationSimulator):
    """Seeded environment evolution and observation-driven decisions.

    World generation and actor/fire evolution may use hidden truth because they
    are the simulator. Decision methods only inspect visible samples and the
    observed occupancy grid. A saved seed and controls reproduce the run.
    """

    GENERATION_VERSION = "procedural-v2"
    ASSESSMENT_SAMPLES = 5
    MAX_LOGICAL_TICKS = 18000

    def __init__(self, seed=None):
        self.speed = 1.0
        self.seed = secrets.randbelow(2147483648) if seed is None else seed
        self.width_m = self.height_m = 6.0
        self.reset()

    def reset(self, scenario=None):
        if scenario not in (None, "randomized_mission"):
            raise ValueError("The interactive presentation uses randomized missions; fixed scenarios are CLI regression fixtures only")
        from .world_geometry import build_world, SyntheticDepthScan
        self.scenario_id = "randomized_mission"
        self.playing = False
        self._logical_time = self._accumulator = 0.0
        self._ticks = 0
        self._world = build_world(self.width_m, self.height_m, self.seed)
        fixture, home, resolution = self._world.cells, self._world.home, self._world.resolution_m
        self._layout_hash = hashlib.sha256(fixture.tobytes() + repr((fixture.shape, home, resolution)).encode()).hexdigest()
        self._resolution = resolution
        self._clearance_cells = max(1, math.ceil(.2 / resolution))
        self._base_fixture = fixture.copy()
        self._mission = MissionSimulator(clock=lambda: self._logical_time, fixture=fixture,
                                         home=home, resolution_m=resolution,
                                         max_exploration_steps=12000, max_duration_s=1500)
        self._mission.reason = "Ready: unknown procedural world; mission contents are not yet observed"
        self._initial_reachable = set(self._mission._reachable_truth)
        self._scanner = SyntheticDepthScan(self._world)
        self._scan_mesh = self._scanner.snapshot()
        self._scan_step = -2
        self._rng = random.Random(self.seed ^ 0x57A2E19)
        self._people = []
        self._fires = []
        self._make_population()
        self._events = []
        self._trail = [list(home)]
        self._heading = 0.0
        self._alerts = []
        self._pending_alerts = {}
        self._behavior = "SEARCH"
        self._assessing = False
        self._visible = set()
        self._replans = 0
        self._sensor_holds = 0
        self._sensor_hold = 0.0
        self._fault_tick = self._rng.randint(45, 180) if self._rng.random() < .22 else None
        self._fault_triggered = False
        self._follower = None
        self._following_log = []
        self._follower_holds = 0
        self._follower_wait_ticks = 0
        self._escort_wait = False
        self._escort_verified = False
        self._outcome_recorded = False
        self._last_move = None
        return self.snapshot()

    def _make_population(self):
        """Independent counts + shuffled distance strata, not a scenario choice."""
        free = [cell for cell in sorted(self._initial_reachable)
                if math.dist(cell, self._mission.home) * self._resolution >= .6]
        if not free:
            return
        ordered = sorted(free, key=lambda cell: (math.dist(cell, self._mission.home), cell))
        strata = [ordered[index * len(ordered) // 3:(index + 1) * len(ordered) // 3] for index in range(3)]
        self._rng.shuffle(strata)
        used = set()

        def choose(index):
            pool = [cell for cell in strata[index % 3] if cell not in used]
            if not pool:
                pool = [cell for cell in ordered if cell not in used]
            if not pool:
                return None
            cell = self._rng.choice(pool)
            used.add(cell)
            return cell

        # Density is bounded on huge worlds, and tiny worlds can be empty.
        person_count = self._rng.choices((0, 1, 2, 3), weights=(2, 4, 3, 1))[0]
        fire_count = self._rng.choices((0, 1, 2, 3), weights=(3, 4, 2, 1))[0]
        max_entities = max(0, len(free) // 12)
        for index in range(min(person_count, max_entities)):
            position = choose(index)
            if position is None:
                break
            response = self._rng.choice(("mobile", "assistance_needed", "no_response"))
            posture = self._rng.choice(("standing", "seated", "crouched", "prone"))
            self._people.append({"id": f"sim-person-{index + 1}", "position": position,
                                 "response": response, "posture": posture, "samples": 0,
                                 "assessment": "unobserved", "observed": None,
                                 "trail": [list(position)], "following": False, "arrived": False,
                                 "pause_at": self._rng.randint(10, 28), "moves": 0,
                                 "pause_ticks": 0, "pause_done": False})
        for index in range(min(fire_count, max_entities)):
            position = choose(index + 1)
            if position is None:
                break
            self._fires.append({"id": f"sim-fire-{index + 1}", "origin": position,
                                "cells": {position}, "start_tick": self._rng.randint(0, 35),
                                "intensity": self._rng.uniform(.15, .35),
                                "growth_per_tick": self._rng.uniform(.002, .006),
                                "spread_period": self._rng.randint(8, 16),
                                "max_cells": min(40, max(4, round(2.5 / self._resolution ** 2))),
                                "active": False, "observed": None, "exclusion": set()})

    def control(self, action, **kwargs):
        if action == "select":
            raise ValueError("Scenario selection is not available; randomize creates a complete new mission")
        if action == "play" and self._mission.phase == "complete" and not self._outcome_recorded:
            if any(value is not None for value in kwargs.values()):
                raise ValueError("Play does not accept additional parameters")
            self.playing = True
            return self.snapshot()
        return super().control(action, **kwargs)

    def _start(self):
        self._mission.start()
        self._event("mission", "Autonomous search started", "Exploring observed free space. World contents remain unknown until sensed.", self._mission.home)
        self._evolve_environment()
        self._sense_and_observe()
        self._update_scan(force=True)

    def _evolve_environment(self):
        """Environmental physics, deliberately separate from robot decisions."""
        truth = self._base_fixture.copy()
        height, width = truth.shape
        raw = self._world.raw_cells
        for fire in self._fires:
            if self._ticks < fire["start_tick"]:
                continue
            fire["active"] = True
            fire["intensity"] = min(1.0, fire["intensity"] + fire["growth_per_tick"])
            if (self._ticks > fire["start_tick"] and
                    (self._ticks - fire["start_tick"]) % fire["spread_period"] == 0
                    and len(fire["cells"]) < fire["max_cells"]):
                candidates = sorted({(col + dx, row + dy) for col, row in fire["cells"]
                                     for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0))
                                     if 0 <= col + dx < width and 0 <= row + dy < height
                                     and raw[row + dy, col + dx] == 0
                                     and (col + dx, row + dy) not in fire["cells"]})
                if candidates:
                    # One new cardinal neighbor per fire per spread interval.
                    fire["cells"].add(self._rng.choice(candidates))
            exclusion = set()
            for col, row in fire["cells"]:
                for y in range(max(0, row - self._clearance_cells), min(height, row + self._clearance_cells + 1)):
                    for x in range(max(0, col - self._clearance_cells), min(width, col + self._clearance_cells + 1)):
                        # Clearance is conservative but never creates rendered
                        # fire through a physical wall. Growth uses raw above.
                        if raw[y, x] == 0 and all(raw[ry, rx] == 0 for rx, ry in _ray((col, row), (x, y))):
                            exclusion.add((x, y))
                            truth[y, x] = 1
            fire["exclusion"] = exclusion
        # No observation flag: the ideal sensor must observe each change. Keep
        # coverage denominator fixed to initial reachability as routes close.
        self._mission._truth = truth
        self._mission._reachable_truth = self._initial_reachable

    def _sense_and_observe(self):
        self._mission._sense()
        origin = self._mission.position
        radius = self._mission.sensor_radius_cells
        visible = set()
        for row in range(max(0, origin[1] - radius), min(self._mission.height, origin[1] + radius + 1)):
            for col in range(max(0, origin[0] - radius), min(self._mission.width, origin[0] + radius + 1)):
                if math.dist(origin, (col, row)) > radius:
                    continue
                for cell in _ray(origin, (col, row)):
                    visible.add(cell)
                    if self._mission._truth[cell[1], cell[0]] != 0:
                        break
        self._visible = visible
        for fire in self._fires:
            seen = sorted(fire["exclusion"] & visible) if fire["active"] else []
            if not seen:
                if fire["observed"]:
                    fire["observed"]["visible"] = False
                continue
            first = fire["observed"] is None
            old_cells = set(map(tuple, fire["observed"]["spread_cells"])) if not first else set()
            measured = sorted(old_cells | set(seen))
            fire["observed"] = {"id": fire["id"], "kind": "hazard", "position": list(min(seen, key=lambda p: math.dist(origin, p))),
                                "label": "Observed synthetic fire", "source": "simulated sensor observation",
                                "intensity": round(fire["intensity"], 3), "spread_cells": [list(cell) for cell in measured],
                                "radius_m": round(max(.2, math.sqrt(len(measured) / math.pi) * self._resolution), 3),
                                "visible": True, "last_seen_tick": self._ticks,
                                "spread_model": "observed exclusion footprint; bounded synthetic growth"}
            if first:
                self._event("discovery", "Fire observed", "Measured synthetic fire perimeter added to the exclusion map; route planning uses observed occupancy.", fire["observed"]["position"])
            elif set(seen) - old_cells:
                self._event("hazard", "Observed fire perimeter expanded", "Newly measured exclusion cells may change the safe route.", fire["observed"]["position"])
        for person in self._people:
            cell = tuple(person["position"])
            can_see = cell in visible and self._mission._truth[cell[1], cell[0]] == 0 and person["pause_ticks"] == 0
            if not can_see:
                if person["observed"]:
                    person["observed"]["visible"] = False
                continue
            first = person["observed"] is None
            if person["assessment"] in ("unobserved", "observing"):
                person["samples"] += 1
                person["assessment"] = "observing"
                if person["samples"] >= self.ASSESSMENT_SAMPLES:
                    person["assessment"] = person["response"]
            assessed = person["assessment"] != "observing"
            evidence = ("Repeated explicit simulated help requests" if person["assessment"] == "assistance_needed" else
                        "No simulated response across five visible samples; medical status unknown" if person["assessment"] == "no_response" else
                        "Repeated simulated willingness and movement response" if person["assessment"] == "mobile" else
                        "Collecting visible simulated interaction samples")
            person["observed"] = {"id": person["id"], "kind": "person", "position": list(cell),
                                  "label": "Simulated person · " + person["assessment"].replace("_", " "),
                                  "source": "simulated sensor observation", "visible": True, "last_seen_tick": self._ticks,
                                  "posture": person["posture"],
                                  "assessment": {"state": person["assessment"], "status": person["assessment"],
                                                 "samples": person["samples"], "required_samples": self.ASSESSMENT_SAMPLES,
                                                 "evidence": evidence, "source": "simulated actor observations"},
                                  "response": person["assessment"] if assessed else "unassessed",
                                  "disposition": "arrived" if person["arrived"] else "following" if person["following"] else "observed"}
            if first:
                self._event("discovery", "Person observed — assessing response", "Collecting repeated visible actor responses; posture alone does not establish a medical state.", cell)

    def _decide(self):
        """No access to unobserved response/origin truth in this decision layer."""
        self._assessing = False
        for person in self._people:
            observed = person["observed"]
            if not observed:
                continue
            assessment = observed["assessment"]["state"]
            if assessment == "observing" and observed["visible"]:
                self._assessing = True
            if assessment in ("assistance_needed", "no_response") and person["id"] not in self._pending_alerts:
                self._pending_alerts[person["id"]] = {"kind": "assistance_needed", "person_id": person["id"],
                    "position": list(observed["position"]), "response": assessment,
                    "source": "simulated actor observations", "delivered_at_home": False}
                self._event("decision", "Assistance response — return and alert", observed["assessment"]["evidence"] + ". Returning with the last observed location.", observed["position"])
                self._mission.abort("observed actor requires assistance; return to outside team with location")
            elif assessment == "mobile" and self._follower is None and not self._pending_alerts:
                self._follower = person
                person["following"] = True
                self._event("decision", "Verified mobile response — escort started", "The simulated actor agreed to follow. Each move requires current distance and visibility verification.", observed["position"])
                self._mission.abort("observed mobile actor; escort to launch with following checks")
        if self._pending_alerts:
            self._behavior = "REPORTING_ASSISTANCE"
        elif self._follower and not self._follower["arrived"]:
            self._behavior = "ESCORT_RETURN"
        else:
            self._behavior = "RETURN" if self._mission.phase == "returning" else "SEARCH"
        if self._assessing:
            self._behavior = "ASSESS_RESPONSE"

    def _move_follower(self):
        person = self._follower
        if person is None or not person["following"] or person["arrived"]:
            return True
        if not person["pause_done"] and person["moves"] >= person["pause_at"]:
            person["pause_done"] = True
            person["pause_ticks"] = self._rng.randint(4, 12)
        if person["pause_ticks"] > 0:
            person["pause_ticks"] -= 1
        else:
            parents = MissionSimulator._reachable(self._mission._observed, tuple(person["position"]))
            path = MissionSimulator._path(parents, self._mission.position)
            distance = math.dist(person["position"], self._mission.position) * self._resolution
            seen = tuple(person["position"]) in self._visible
            threshold = max(.4, 2 * self._resolution)
            if len(path) >= 2 and (distance > threshold or not seen):
                candidate = path[1]
                # Physics cannot move an actor into a newly burning cell even
                # if its last observation was free; stale path gives a hold.
                if (self._mission._truth[candidate[1], candidate[0]] == 0
                        and math.dist(candidate, self._mission.position) * self._resolution >= .38 - 1e-9):
                    person["position"] = candidate
                    person["trail"].append(list(candidate))
                    person["moves"] += 1
        cell = tuple(person["position"])
        distance = math.dist(cell, self._mission.position) * self._resolution
        visible = (person["pause_ticks"] == 0 and distance <= 6 * self._resolution and
                   all(self._mission._truth[y, x] == 0 for x, y in _ray(self._mission.position, cell)))
        verified = visible and distance <= max(.6, 3 * self._resolution) + 1e-9
        if visible:
            # Publish only this fresh measurement, never the actor's hidden
            # trajectory while occluded or paused.
            person["observed"]["position"] = list(cell)
            person["observed"]["last_seen_tick"] = self._ticks
        person["observed"]["visible"] = visible
        self._following_log.append({"step": self._mission.steps, "tick": self._ticks,
                                   "rover": list(self._mission.position),
                                   "person": list(cell) if visible else None,
                                   "last_observed_person": list(person["observed"]["position"]),
                                   "distance_m": round(distance, 3) if visible else None,
                                   "visible": visible, "verified": verified})
        if not verified:
            if not self._escort_wait:
                self._follower_holds += 1
                self._event("follower", "Following unverified — rover holds", "Waiting for a current visible measurement, not trusting the last actor position.", self._mission.position)
            self._escort_wait = True
            self._follower_wait_ticks += 1
            self._behavior = "ESCORT_WAIT"
            if self._follower_wait_ticks >= 50:
                person["following"] = False
                self._escort_wait = False
                self._pending_alerts[person["id"]] = {"kind": "following_lost", "person_id": person["id"],
                    "position": list(person["observed"]["position"]), "source": "last simulated sensor observation",
                    "delivered_at_home": False}
                self._event("decision", "Follower not reacquired — return with last observation", "No escort success is claimed. The last observed location is retained for an assistance report.", person["observed"]["position"])
                self._mission.abort("following lost; return with last observed actor location")
                return True
            return False
        if self._escort_wait:
            self._event("follower", "Following reacquired", "Current simulated measurement verifies visibility and following distance.", cell)
        self._escort_wait = False
        self._follower_wait_ticks = 0
        if self._mission.position == self._mission.home and distance <= max(.6, 2 * self._resolution) + 1e-9:
            person["arrived"] = True
            self._escort_verified = True
            person["observed"]["disposition"] = "arrived"
        return True

    def _observed_person_exclusions(self):
        """Robot radius .20 m plus actor body radius .18 m, measured cells only."""
        blocked = set()
        padding = math.ceil(.38 / self._resolution)
        for person in self._people:
            observed = person["observed"]
            if observed is None:
                continue
            col, row = observed["position"]
            for y in range(max(0, row - padding), min(self._mission.height, row + padding + 1)):
                for x in range(max(0, col - padding), min(self._mission.width, col + padding + 1)):
                    if math.dist((col, row), (x, y)) * self._resolution < .38 - 1e-9:
                        blocked.add((x, y))
        return blocked

    def _update_scan(self, force=False):
        if force or self._mission.steps - self._scan_step >= 2:
            self._scanner.update(self._mission.position, self._heading)
            # Scanner snapshots are immutable for this revision. Do not copy
            # thousands of vertices on every unchanged state request.
            self._scan_mesh = self._scanner.snapshot()
            self._scan_step = self._mission.steps

    def _advance_one(self):
        self._ticks += 1
        self._evolve_environment()  # keeps evolving during assessment/follower holds
        if self._sensor_hold > 0:
            self._sensor_hold = round(max(0.0, self._sensor_hold - .1), 6)
            if self._sensor_hold == 0:
                self._sense_and_observe()
                self._event("sensor", "Sensor recovered — partial survey return", "A generated data interruption ended. Return is replanned from refreshed observations.")
                self._mission.abort("sensor interruption; partial survey return")
            return
        if self._fault_tick is not None and self._ticks >= self._fault_tick and not self._fault_triggered:
            self._fault_triggered = True
            self._sensor_hold = 1.5
            self._sensor_holds += 1
            self._event("sensor", "Sensor data unavailable — hold", "Generated sensing interruption: the rover does not move until observations recover.", self._mission.position)
            return
        before_position = self._mission.position
        before_route = list(self._mission._route)
        before_observed = self._mission._observed.copy()
        self._sense_and_observe()
        if self._mission._observed[before_position[1], before_position[0]] != 0:
            self._mission._block("Observed fire exclusion reached rover; stopped in place")
            self._finish_procedural()
            return
        self._decide()
        follower_ready = self._move_follower()
        if self._ticks >= self.MAX_LOGICAL_TICKS:
            self._mission._block("Logical time limit reached before verified mission completion")
            self._finish_procedural()
            return
        if self._assessing or not follower_ready:
            self._update_scan()
            return
        newly_blocked = ((before_observed == 0) & (self._mission._observed == 1))
        affected_route = any(newly_blocked[row, col] for col, row in before_route[1:])
        person_exclusions = self._observed_person_exclusions()
        if self._mission.position in person_exclusions:
            self._mission._block("Observed actor overlaps required clearance; rover holds without a safe move")
            self._finish_procedural()
            return
        state = self._mission.tick(observed_obstacles=person_exclusions,
            motion_guard=lambda cell: all(math.dist(cell, person["position"]) * self._resolution >= .38 - 1e-9
                                          for person in self._people))
        if affected_route:
            self._replans += 1
            self._event("replan", "Observed hazard changed the route", "The planner recomputed its next move from refreshed occupancy; a blocked return is reported explicitly.", self._mission.position)
        if self._mission.position != before_position:
            dx, dy = self._mission.position[0] - before_position[0], self._mission.position[1] - before_position[1]
            self._heading = math.atan2(dy, dx)
            self._trail.append(list(self._mission.position))
            self._last_move = {"from": list(before_position), "to": list(self._mission.position),
                               "observed_free": True, "clearance_checked": True, "tick": self._ticks}
        # Refresh geometric visibility after movement without sampling an actor
        # twice in one logical tick. Actor sampling resumes on the next tick.
        self._update_scan()
        if state["phase"] in ("complete", "blocked"):
            if state["phase"] == "complete" and self._follower and self._follower["following"] and not self._follower["arrived"]:
                self._escort_wait = True
                self._behavior = "ESCORT_WAIT"
                return
            self._finish_procedural()

    def _finish_procedural(self):
        if self._outcome_recorded:
            return
        self._outcome_recorded = True
        self.playing = False
        returned = self._mission.phase == "complete" and self._mission.position == self._mission.home
        # Every observed person who has not arrived remains unresolved. This
        # includes people seen too briefly to finish the response assessment.
        for person in self._people:
            observed = person["observed"]
            if observed and not person["arrived"] and person["id"] not in self._pending_alerts:
                self._pending_alerts[person["id"]] = {"kind": "person_location", "person_id": person["id"],
                    "position": list(observed["position"]), "response": observed["assessment"]["state"],
                    "source": "last simulated sensor observation", "delivered_at_home": False}
        self._alerts = [{**copy.deepcopy(alert), "delivered_at_home": returned,
                         "step": self._mission.steps} for alert in self._pending_alerts.values()]
        self._behavior = "COMPLETE" if returned else "BLOCKED"
        self._event("outcome", "Returned with observations" if returned else "Stopped — return not verified",
                    self._mission.reason + ". No all-clear is implied.", self._mission.position)
        if returned and self._alerts:
            self._event("alert", "Observed person locations reported at launch", "Unresolved people are listed with their last observed positions; physical rescue is not claimed.")

    def snapshot(self):
        state = self._mission.snapshot()
        grid = {**state["grid"], "frame": "synthetic-presentation",
                "note": "Observed configuration space; unobserved cells stay unknown. Synthetic sensor only."}
        entities = [copy.deepcopy(item["observed"]) for item in self._people + self._fires if item["observed"]]
        complete = (state["phase"] == "complete" and state["position"] == state["home"] and
                    (not self._follower or not self._follower["following"] or self._follower["arrived"]))
        blocked = state["phase"] == "blocked"
        phase = ("sensor_hold" if self._sensor_hold > 0 else "assessing" if self._assessing and self.playing else
                 "escort_wait" if self._escort_wait and self.playing else state["phase"])
        people_found = sum(person["observed"] is not None for person in self._people)
        unresolved = sum(person["observed"] is not None and not person["arrived"] for person in self._people)
        metrics = {"steps": state["steps"], "logical_ticks": self._ticks,
                   "coverage_percent": state["coverage_percent"], "unobserved_cells": state["unobserved_cells"],
                   "coverage_basis": "Simulation diagnostic: observed initial reachable configuration cells; denominator stays fixed as fire grows",
                   "detections": len(entities), "people_found": people_found,
                   "hazards_found": sum(fire["observed"] is not None for fire in self._fires),
                   "replans": self._replans, "sensor_holds": self._sensor_holds,
                   "elapsed_sim_s": round(self._logical_time, 1), "route_length_cells": max(0, len(state["route"]) - 1),
                   "follower_holds": self._follower_holds, "following_checks": len(self._following_log),
                   "escort_verified": self._escort_verified,
                   "assistance_alerts": len(self._alerts), "unresolved_observed_people": unresolved}
        follower = None
        if self._follower:
            observed = self._follower["observed"]
            last = self._following_log[-1] if self._following_log else {}
            follower = {"person_id": self._follower["id"], "position": list(observed["position"]),
                        "visible": last.get("visible", False), "distance_m": last.get("distance_m"),
                        "following_verified": last.get("verified", False),
                        "at_launch_zone": self._follower["arrived"],
                        "state": "arrived" if self._follower["arrived"] else "lost" if not self._follower["following"] else "waiting" if self._escort_wait else "following",
                        "source": "last simulated sensor observation",
                        "trail": [entry["person"] for entry in self._following_log if entry["visible"]]}
        summary = state["reason"] + ("; unresolved observed people are included in the report" if unresolved else "")
        return {"simulation_only": True, "label": "PROCEDURAL SYNTHETIC WORLD · OBSERVATION-DRIVEN PLANNER",
                "seed": self.seed, "layout_hash": self._layout_hash, "scenarios": [], "scenario": dict(PROCEDURAL_SCENARIO),
                "mission": {"mode": "procedural", "generation_version": self.GENERATION_VERSION,
                            "layout_family": self._world.family, "observation_gated": True,
                            "environment_evolves": True, "hidden_contents_disclosed": False},
                "boundary": {"width_m": self.width_m, "height_m": self.height_m, "resolution_m": self._resolution,
                             "robot_radius_m": .2, "sensor_range_m": round(6 * self._resolution, 3),
                             "enforced": True, "hardware_supported": False},
                "playing": self.playing, "speed": self.speed, "phase": phase,
                "playback_status": "finished" if self._outcome_recorded else "playing" if self.playing else "ready" if state["phase"] == "idle" else "paused",
                "grid": grid, "position": state["position"], "home": state["home"], "heading_rad": self._heading,
                "route": state["route"], "trail": copy.deepcopy(self._trail), "frontiers": self._frontiers(grid),
                "last_move": copy.deepcopy(self._last_move),
                "entities": entities, "events": copy.deepcopy(self._events), "metrics": metrics,
                "outcome": {"status": "returned" if complete else "blocked" if blocked else "pending",
                            "title": "Returned with observations" if complete else "Return not verified — stopped" if blocked else "Ready to explore" if state["phase"] == "idle" else "Mission in progress",
                            "summary": summary, "returned_home": complete, "all_clear": False,
                            "coverage_percent": state["coverage_percent"], "scenario_applicability": "procedural"},
                "follower": follower, "alerts": copy.deepcopy(self._alerts),
                "behavior": {"state": self._behavior, "source": "observed simulated actor responses and occupancy"},
                "sensor_hold_remaining_s": self._sensor_hold,
                "scan_mesh": self._scan_mesh, "assumptions": list(PROCEDURAL_ASSUMPTIONS)}

    def report(self):
        state = self.snapshot()
        report = {"schema_version": "2.0", "export_type": "warm-wheels-presentation-report",
                  "seed": self.seed, "layout_hash": self._layout_hash, "mission": state["mission"],
                  "simulation_only": True, "scenario": state["scenario"], "boundary": state["boundary"],
                  "metrics": state["metrics"], "outcome": state["outcome"], "events": state["events"],
                  "entities": state["entities"], "follower": state["follower"], "alerts": state["alerts"],
                  "following_log": copy.deepcopy(self._following_log), "trajectory": state["trail"],
                  "final_observed_grid": state["grid"], "assumptions": list(PROCEDURAL_ASSUMPTIONS),
                  "hardware_test_performed": False}
        if self._outcome_recorded:
            report["simulation_diagnostics"] = {"label": "HIDDEN GROUND TRUTH — POST-RUN SOFTWARE DIAGNOSTIC ONLY",
                "people_total": len(self._people), "fire_origins_total": len(self._fires),
                "people_unobserved": sum(person["observed"] is None for person in self._people),
                "people": [{"id": person["id"], "response_fixture": person["response"],
                            "initial_position": person["trail"][0], "actual_trail": person["trail"],
                            "arrived": person["arrived"]} for person in self._people],
                "fires": [{"id": fire["id"], "origin": list(fire["origin"]),
                           "physical_cells": [list(cell) for cell in sorted(fire["cells"])],
                           "active": fire["active"], "intensity": round(fire["intensity"], 3)} for fire in self._fires]}
        return report


class PresentationSimulator:
    """Default interactive world; explicit old IDs remain offline test fixtures."""

    def __new__(cls, scenario=None, *, seed=None):
        if scenario in (None, "randomized_mission"):
            return ProceduralPresentationSimulator(seed=seed)
        if seed is not None:
            raise ValueError("Pass a legacy fixture seed with its randomize control")
        return LegacyPresentationSimulator(scenario)
