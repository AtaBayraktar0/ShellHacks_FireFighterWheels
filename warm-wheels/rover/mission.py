"""EMPTY-room SEARCH_AND_RETURN simulation; never a physical motor controller.

The robot uses an ideal, simulated 360-degree range sensor. This is explicitly
not the front-facing D435i. The four-room fixture contains walls/furniture and
no people or fire. Wall cells are expanded by one cell before simulation to
represent clearance for a small robot; the planner then moves a point through
that configuration-space grid. Hidden fixture data is used by the sensor and
simulation diagnostics, never to choose exploration destinations or routes.
"""
from __future__ import annotations

from collections import deque
import math
import time

import numpy as np


_NEIGHBORS = ((0, -1), (1, 0), (0, 1), (-1, 0))


def _fixture() -> np.ndarray:
    """30x30 cells, four connected rooms, broad doors, ordinary solid objects."""
    walls = np.zeros((30, 30), dtype=np.int8)
    walls[[0, -1], :] = 1
    walls[:, [0, -1]] = 1
    walls[15, :] = 1
    walls[:, 15] = 1
    walls[15, 6:11] = 0
    walls[15, 20:25] = 0
    walls[6:11, 15] = 0
    walls[20:25, 15] = 0
    walls[5:7, 5:7] = 1
    walls[21:24, 8:10] = 1
    walls[6:9, 23:25] = 1
    walls[22:24, 22:24] = 1
    # Pre-inflate all walls/furniture by one 0.20 m cell. Sensor and display
    # operate on this synthetic clearance grid, not a raw physical-depth map.
    expanded = walls.copy()
    for row, col in np.argwhere(walls == 1):
        expanded[max(0, row - 1):min(30, row + 2),
                 max(0, col - 1):min(30, col + 2)] = 1
    return expanded


def _ray(start: tuple[int, int], target: tuple[int, int]):
    """Bresenham cells, with conservative checks for diagonal corner crossings."""
    x, y = start
    tx, ty = target
    dx, dy = abs(tx - x), -abs(ty - y)
    sx, sy = (1 if x < tx else -1), (1 if y < ty else -1)
    error = dx + dy
    yield x, y
    while (x, y) != (tx, ty):
        twice = 2 * error
        previous_x, previous_y = x, y
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy
        if x != previous_x and y != previous_y:
            # These side cells conservatively stop rays from seeing through a
            # corner made by two touching walls. Slight under-observation is OK.
            yield x, previous_y
            yield previous_x, y
        yield x, y


class MissionSimulator:
    """One cell per tick; runtime decides pacing (e.g. one tick every 0.1 s).

    start() resets the synthetic fixture observation state. abort() requests a
    route home; it does not teleport the rover. Completion requires actually
    reaching home. Losing every known return path produces phase='blocked'.
    """

    def __init__(self, sensor_radius_cells: int = 6,
                 max_exploration_steps: int = 300, max_duration_s: float = 90.0,
                 clock=None, fixture=None, home=None, resolution_m: float = 0.2,
                 sensor_fov_deg: float = 360.0):
        if sensor_radius_cells < 1 or max_exploration_steps < 0:
            raise ValueError("Sensor radius must be positive and step budget nonnegative.")
        if not math.isfinite(max_duration_s) or max_duration_s <= 0:
            raise ValueError("Mission duration must be finite and positive.")
        self.sensor_radius_cells = int(sensor_radius_cells)
        if not math.isfinite(float(sensor_fov_deg)) or not 0 < float(sensor_fov_deg) <= 360:
            raise ValueError("Sensor horizontal field of view must be in (0, 360].")
        self.sensor_fov_deg = float(sensor_fov_deg)
        self.sensor_heading_rad = 0.0
        self.max_exploration_steps = int(max_exploration_steps)
        self.max_duration_s = float(max_duration_s)
        self._clock = clock or time.monotonic
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("Grid resolution must be finite and positive.")
        self.resolution_m = float(resolution_m)
        if fixture is None:
            self._initial_fixture = _fixture()
        else:
            supplied = np.asarray(fixture)
            if supplied.ndim != 2 or min(supplied.shape) < 1 or max(supplied.shape) > 80 or not np.all((supplied == 0) | (supplied == 1)):
                raise ValueError("Custom synthetic fixture must be a 1..80 by 1..80 binary grid.")
            self._initial_fixture = supplied.astype(np.int8).copy()
        self._truth = self._initial_fixture.copy()
        self.height, self.width = self._truth.shape
        selected_home = (4, 25) if home is None else tuple(home)
        if (len(selected_home) != 2 or any(type(value) is not int for value in selected_home)
                or not 0 <= selected_home[0] < self.width or not 0 <= selected_home[1] < self.height
                or self._truth[selected_home[1], selected_home[0]] != 0):
            raise ValueError("Launch cell must be a free cell inside the fixture.")
        self.home = selected_home
        self.position = self.home
        self.phase = "idle"
        self.reason = "Ready: empty synthetic four-room search and return"
        self.steps = 0
        self._started_at = None
        self._ended_at = None
        self._return_reason = ""
        self._route = []
        self._observed = np.full_like(self._truth, -1)
        # Diagnostic denominator only; frontier selection uses _observed alone.
        self._reachable_truth = set(self._reachable(self._truth, self.home))

    @staticmethod
    def _reachable(cells, start):
        height, width = cells.shape
        if cells[start[1], start[0]] != 0:
            return {}
        parents = {start: None}
        queue = deque([start])
        while queue:
            col, row = queue.popleft()
            for dx, dy in _NEIGHBORS:
                node = (col + dx, row + dy)
                if (0 <= node[0] < width and 0 <= node[1] < height
                        and cells[node[1], node[0]] == 0 and node not in parents):
                    parents[node] = (col, row)
                    queue.append(node)
        return parents

    @staticmethod
    def _path(parents, target):
        if target not in parents:
            return []
        result = [target]
        while parents[result[-1]] is not None:
            result.append(parents[result[-1]])
        return result[::-1]

    def _sense(self):
        col, row = self.position
        radius = self.sensor_radius_cells
        for target_row in range(max(0, row - radius), min(self.height, row + radius + 1)):
            for target_col in range(max(0, col - radius), min(self.width, col + radius + 1)):
                if (target_col - col) ** 2 + (target_row - row) ** 2 > radius ** 2:
                    continue
                if self.sensor_fov_deg < 360.0:
                    angle = math.atan2(target_row - row, target_col - col)
                    delta = (angle - self.sensor_heading_rad + math.pi) % math.tau - math.pi
                    if abs(math.degrees(delta)) > self.sensor_fov_deg / 2:
                        continue
                for ray_col, ray_row in _ray(self.position, (target_col, target_row)):
                    if not (0 <= ray_col < self.width and 0 <= ray_row < self.height):
                        break
                    self._observed[ray_row, ray_col] = self._truth[ray_row, ray_col]
                    if self._truth[ray_row, ray_col] == 1:
                        break

    def _frontier_path(self):
        # BFS gives shortest paths through observed free space, equivalent to
        # A* with h=0. Dictionary insertion order is BFS order and deterministic.
        parents = self._reachable(self._observed, self.position)
        for col, row in parents:
            if (col, row) == self.position:
                continue
            if any(0 <= col + dx < self.width and 0 <= row + dy < self.height
                   and self._observed[row + dy, col + dx] == -1
                   for dx, dy in _NEIGHBORS):
                return self._path(parents, (col, row))
        return []

    def start(self) -> dict:
        if self.phase in ("exploring", "returning"):
            raise ValueError("Mission already active; abort it before restarting.")
        self._truth = self._initial_fixture.copy()
        self._reachable_truth = set(self._reachable(self._truth, self.home))
        self.position = self.home
        self.sensor_heading_rad = 0.0
        self.phase = "exploring"
        self.reason = "Exploring observed frontiers; no fire/person discovery is required"
        self.steps = 0
        self._started_at = self._clock()
        self._ended_at = None
        self._return_reason = ""
        self._route = []
        self._observed[:] = -1
        self._sense()
        return self.snapshot()

    def _begin_return(self, reason):
        self.phase = "returning"
        self._return_reason = reason
        self.reason = f"Returning to launch: {reason}"
        self._route = self._path(self._reachable(self._observed, self.position), self.home)
        if self.position == self.home:
            self._finish()
        elif not self._route:
            self._block("Return path unavailable through observed free space")

    def abort(self, reason="operator requested early return") -> dict:
        if self.phase in ("exploring", "returning"):
            self._sense()
            self._begin_return(str(reason)[:200])
        return self.snapshot()

    def set_obstacle(self, col: int, row: int, *, observed: bool = False,
                     clearance_cells: int = 1) -> dict:
        """Inject a solid obstacle into the SIMULATION fixture for software tests.

        This is not exposed by the robot API and never sends a motor command.
        A square clearance buffer is included, matching the synthetic fixture.
        By default the changed cells remain hidden until the ideal sensor sees
        them. observed=True explicitly supplies a synthetic observation for a
        deterministic fault test; it is not a real sensor measurement.

        The new obstacle cannot overlap the simulated robot. Coverage uses the
        updated fixture's reachable cells, so its denominator can change after
        injection. Navigation continues to use only the observed grid.
        """
        if type(col) is not int or type(row) is not int or type(clearance_cells) is not int:
            raise ValueError("Obstacle coordinates and clearance must be integers.")
        if not (0 <= col < self.width and 0 <= row < self.height) or not 0 <= clearance_cells <= 3:
            raise ValueError("Obstacle must be inside the fixture with clearance 0..3 cells.")
        if self.phase not in ("exploring", "returning"):
            raise ValueError("Start a simulation before injecting an obstacle.")
        x1, x2 = max(0, col - clearance_cells), min(self.width, col + clearance_cells + 1)
        y1, y2 = max(0, row - clearance_cells), min(self.height, row + clearance_cells + 1)
        if x1 <= self.position[0] < x2 and y1 <= self.position[1] < y2:
            raise ValueError("Injected obstacle cannot overlap the simulated robot.")
        self._truth[y1:y2, x1:x2] = 1
        if observed:
            self._observed[y1:y2, x1:x2] = 1
        self._reachable_truth = set(self._reachable(self._truth, self.home))
        return self.snapshot()

    def _finish(self):
        self.phase = "complete"
        self.reason = f"Returned to launch: {self._return_reason}"
        self._route = []
        self._ended_at = self._clock()

    def _block(self, reason):
        self.phase = "blocked"
        self.reason = reason + "; rover has not returned to launch"
        self._route = []
        self._ended_at = self._clock()

    def tick(self, *, observed_obstacles=(), motion_guard=None) -> dict:
        """Step using fresh sensing and optional measured dynamic exclusions.

        The procedural simulator supplies observed actor footprint cells.
        Unknown cells remain unknown; this interface cannot reveal hidden space.
        """
        if self.phase not in ("exploring", "returning"):
            return self.snapshot()
        self._sense()
        for col, row in observed_obstacles:
            if (0 <= col < self.width and 0 <= row < self.height
                    and self._observed[row, col] != -1):
                self._observed[row, col] = 1
        if self.phase == "exploring":
            if self.steps >= self.max_exploration_steps:
                self._begin_return("exploration step budget reached; scan may be incomplete")
            elif self._clock() - self._started_at >= self.max_duration_s:
                self._begin_return("exploration time budget reached; scan may be incomplete")
            else:
                self._route = self._frontier_path()
                if not self._route:
                    self._begin_return("no reachable observed frontier remains")
        if self.phase == "returning":
            # Recompute every tick against current sensing, including a door
            # becoming blocked. Never trust an old return route blindly.
            self._route = self._path(self._reachable(self._observed, self.position), self.home)
            if not self._route:
                self._block("Return path unavailable through observed free space")
        if self.phase in ("exploring", "returning") and len(self._route) >= 2:
            next_position = self._route[1]
            if (self._observed[next_position[1], next_position[0]] != 0
                    or self._truth[next_position[1], next_position[0]] != 0):
                self._block("Simulation refused a move into an unknown or occupied cell")
                return self.snapshot()
            if motion_guard is not None and not motion_guard(next_position):
                # Physics/contact interlock, not a route oracle. It can refuse
                # an attempted move but never selects an alternative using truth.
                self._block("Simulation physical clearance interlock refused the attempted move")
                return self.snapshot()
            previous_position = self.position
            self.position = next_position
            self.sensor_heading_rad = math.atan2(next_position[1] - previous_position[1],
                                                 next_position[0] - previous_position[0])
            self.steps += 1
            self._route = self._route[1:]
            self._sense()
        if self.phase == "returning" and self.position == self.home:
            self._finish()
        return self.snapshot()

    def snapshot(self) -> dict:
        observed_reachable = sum(int(self._observed[row, col] != -1)
                                 for col, row in self._reachable_truth)
        reachable_count = len(self._reachable_truth)
        elapsed = 0.0 if self._started_at is None else (
            (self._ended_at if self._ended_at is not None else self._clock()) - self._started_at)
        return {
            "phase": self.phase, "reason": self.reason,
            "position": list(self.position), "home": list(self.home),
            "route": [list(cell) for cell in self._route], "steps": self.steps,
            "elapsed_s": round(max(0.0, elapsed), 1),
            "coverage_percent": round(100 * observed_reachable / max(1, reachable_count), 1),
            "unobserved_cells": int(reachable_count - observed_reachable),
            "reachable_cells": reachable_count,
            "unknown_grid_cells": int(np.sum(self._observed == -1)),
            "coverage_basis": "Simulation diagnostic: observed reachable cells of the hidden clearance fixture",
            "sensor_model": ("IDEAL SIMULATED 360-degree rays; not a D435i sensor model"
                             if self.sensor_fov_deg >= 360.0 else
                             f"IDEAL SIMULATED forward {self.sensor_fov_deg:g}-degree rays; not a D435i sensor model"),
            "sensor_fov_deg": self.sensor_fov_deg,
            "sensor_heading_rad": self.sensor_heading_rad,
            "scenario": "Empty four-room fixture: no fire or person objects",
            "grid": {
                "width": self.width, "height": self.height, "resolution_m": self.resolution_m,
                "origin_col": self.position[0], "origin_row": self.position[1],
                "cells": self._observed.reshape(-1).astype(int).tolist(),
                "frame": "synthetic-mission",
                "note": ("Simulation only: 360-degree sensor; walls include one-cell clearance buffer"
                         if self.sensor_fov_deg >= 360.0 else
                         "Simulation only: forward camera cone; walls include one-cell clearance buffer"),
            },
        }
