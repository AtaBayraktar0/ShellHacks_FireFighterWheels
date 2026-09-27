"""Plan a real trip and verify every step with fresh sensor data."""
import json
import math
import time
from pathlib import Path
from .planning import astar
from .vision import Pose


class Mission:
    """Remember the measured trip to the goal and back home."""

    def __init__(self, start, goal):
        self.goal = tuple(goal)
        self.visited = [tuple(start)]
        self.returning = False
        self.return_targets = []
        self.done = False
        self.target = None

    def next_target(self, cell, safe):
        """Choose the robot's next safe grid square."""
        if self.done:
            return None
        if self.target is not None and cell != self.target:
            raise RuntimeError("Previous waypoint has not been reached")
        if self.target is not None:
            if self.returning:
                self.return_targets.pop(0)
            elif cell != self.visited[-1]:
                self.visited.append(cell)
            self.target = None
        if not self.returning and cell == self.goal:
            # Reverse the measured trip after reaching the goal.
            self.returning = True
            self.return_targets = self.visited[-2::-1]
        if self.returning:
            if not self.return_targets:
                self.done = True
                return None
            target = self.return_targets[0]
            # Stop when a fresh scan says the return route is unsafe.
            if not safe[cell[1], cell[0]] or not safe[target[1], target[0]]:
                raise RuntimeError("Return waypoint is blocked, unknown, or stale")
        else:
            # Plan again so a newly seen obstacle can change the route.
            route = astar(safe, cell, self.goal)
            if len(route) < 2:
                raise RuntimeError("No observed safe path to goal")
            target = route[1]
        self.target = target
        return target


def read_pose(path, max_age):
    """Read a recent robot position from a JSON file."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    values = [float(data[key]) for key in ("x", "y", "yaw", "timestamp")]
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Pose contains invalid numbers")
    age = time.time() - values[3]
    if age < -0.05 or age > max_age:
        raise RuntimeError("Position update is stale or its clock is incorrect")
    return Pose(*values[:3])


def steering(pose, target_xy):
    """Choose whether to turn or drive forward."""
    dx, dy = target_xy[0] - pose.x, target_xy[1] - pose.y
    error = (math.atan2(dy, dx) - pose.yaw + math.pi) % (2 * math.pi) - math.pi
    if abs(error) > math.radians(10):
        return "LEFT" if error > 0 else "RIGHT"
    return "FWD"


def save_log(path, mission, samples, status):
    """Save the measured positions from a real mission."""
    data = {"status": status, "outbound_cells": mission.visited, "samples": samples}
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def run_live(camera, motors, grid, config, pose_file, output):
    """Run the guarded loop for the real camera and motors."""
    from .vision import flame_mask, update_map

    if not config["calibrated"]:
        raise RuntimeError("Calibrate the mount, footprint, and pulse distance first")
    mission = Mission(config["start_cell"], config["goal_cell"])
    samples, status, target = [], "stopped", None
    last_progress = time.monotonic()
    try:
        for _ in range(10000):
            # Stop before every real camera scan.
            motors.stop()
            before = read_pose(pose_file, config["pose_max_age_s"])
            frame = camera.read()
            pose = read_pose(pose_file, config["pose_max_age_s"])
            yaw_delta = (pose.yaw - before.yaw + math.pi) % (2 * math.pi) - math.pi
            if math.hypot(pose.x-before.x, pose.y-before.y) > 0.01 or abs(yaw_delta) > 0.03:
                raise RuntimeError("Robot moved during stopped scan")
            update_map(grid, frame, pose, config, flame_mask(frame.bgr))
            now = time.monotonic()
            if now - frame.captured > config["map_max_age_s"]:
                raise RuntimeError("Scan processing took too long")
            safe = grid.safe(now, config["map_max_age_s"],
                             config["robot_radius_m"] + config["max_pulse_travel_m"])
            cell = grid.cell(pose.x, pose.y)
            if not grid.inside(cell) or not safe[cell[1], cell[0]]:
                raise RuntimeError("Current footprint is not freshly verified free")
            samples.append({"x": pose.x, "y": pose.y, "yaw": pose.yaw,
                            "returning": mission.returning, "timestamp": time.time()})
            if target is None:
                # The first real pose must match the chosen start square.
                start_xy = grid.center(mission.visited[0])
                if math.hypot(pose.x-start_xy[0], pose.y-start_xy[1]) > 0.015:
                    raise RuntimeError("Place robot at configured start center")
                target = mission.next_target(cell, safe)
            elif math.dist((pose.x, pose.y), grid.center(target)) <= 0.015:
                target = mission.next_target(target, safe)
                last_progress = now
            if target is None:
                status = "complete"
                break
            if not safe[target[1], target[0]]:
                raise RuntimeError("Next footprint is no longer safe")
            if abs(cell[0]-target[0]) + abs(cell[1]-target[1]) > 1:
                raise RuntimeError("Robot left the planned segment")
            if now - last_progress > 20:
                raise RuntimeError("Waypoint timed out; check localization and motors")
            fresh = read_pose(pose_file, config["pose_max_age_s"])
            if math.dist((fresh.x, fresh.y), (pose.x, pose.y)) > 0.01:
                raise RuntimeError("Position changed before command")
            # Move one short pulse before checking the sensors again.
            motors.pulse(steering(pose, grid.center(target)),
                         config["pwm"], config["pulse_ms"])
        else:
            raise RuntimeError("Mission step limit reached")
    except BaseException as exc:
        status = str(exc) or type(exc).__name__
        raise
    finally:
        try:
            motors.stop()
        finally:
            save_log(output / "mission.json", mission, samples, status)
    return status
