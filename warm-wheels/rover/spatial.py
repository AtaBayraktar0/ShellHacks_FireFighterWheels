"""Camera-local geometry for a supervised tabletop prototype, not SLAM.

Every frame replaces the previous scan. There is no global pose, house map,
localization, rear coverage, or proof that an exit exists. The camera must be
level, rigidly mounted, and its height measured before floor filtering works.
"""
from __future__ import annotations

import heapq
import math

import cv2
import numpy as np

from .camera import Frame

GRID_SIZE = 60
RESOLUTION_M = 0.1
MIN_DEPTH_M = 0.1
MAX_DEPTH_M = 6.0


def _line(x0: int, y0: int, x1: int, y1: int):
    """Integer Bresenham ray, including both endpoints."""
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * err
        if twice >= dy:
            err += dy
            x0 += sx
        if twice <= dx:
            err += dx
            y0 += sy


def analyze(frame: Frame, camera_height_m: float = 0.22,
            robot_half_width_m: float = 0.20, obstacle_top_m: float = 0.45) -> dict:
    """Produce a local scan and a strict front depth advisory.

    Positive x is camera-right, y camera-up, z camera-forward. Grid rows
    decrease with increasing z. Floor returns are filtered at heights below
    0.035 m, which also means objects below that height can be missed. This
    scan cannot certify a traversable surface (stairs/dropoffs/glass/smoke).

    A single invalid depth sample in the projected front safety window makes
    clearance unknown. This intentionally causes frequent stops on hardware.
    It is preferable to turning a missing return into imaginary free space.
    """
    depth = np.asarray(frame.depth, dtype=np.float32)
    if depth.ndim != 2 or frame.rgb.shape != (*depth.shape, 3):
        raise ValueError("RGB and depth must have matching aligned HxW dimensions.")
    if not all(math.isfinite(v) for v in (frame.fx, frame.fy, frame.cx, frame.cy)):
        raise ValueError("Camera intrinsics must be finite.")
    if frame.fx <= 0 or frame.fy <= 0 or camera_height_m <= 0:
        raise ValueError("Focal lengths and camera height must be positive.")
    height, width = depth.shape
    valid = np.isfinite(depth) & (depth >= MIN_DEPTH_M) & (depth <= MAX_DEPTH_M)
    # Use zero at unknown samples to avoid propagating NaN/inf into projections.
    z = np.where(valid, depth, 0.0)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    x = (u - frame.cx) * z / frame.fx
    y = (frame.cy - v) * z / frame.fy
    above_floor = y + camera_height_m
    obstacles = valid & (above_floor >= 0.035) & (above_floor <= obstacle_top_m)
    corridor = obstacles & (np.abs(x) <= robot_half_width_m + 0.03)

    # Safety window encloses the robot's width/height at 0.70 m ahead. All its
    # depth must be observed; this is only a forward advisory, not full coverage.
    horizon = 0.70
    xa = max(0, int(math.floor(frame.cx - frame.fx * robot_half_width_m / horizon)))
    xb = min(width, int(math.ceil(frame.cx + frame.fx * robot_half_width_m / horizon)) + 1)
    ya = max(0, int(math.floor(frame.cy - frame.fy * (obstacle_top_m - camera_height_m) / horizon)))
    yb = min(height, int(math.ceil(frame.cy + frame.fy * (camera_height_m - 0.035) / horizon)) + 1)
    front = valid[ya:yb, xa:xb]
    front_fraction = float(np.mean(front)) if front.size else 0.0
    clearance = None
    reason = "missing depth in front safety window"
    if front.size and bool(np.all(front)):
        # Minimum over per-column observed horizons avoids claiming range that
        # only one part of the image happens to see.
        observed_horizon = float(np.min(np.max(z[ya:yb, xa:xb], axis=0)))
        obstacle_distance = float(np.min(z[corridor])) if np.any(corridor) else MAX_DEPTH_M
        clearance = round(min(observed_horizon, obstacle_distance, MAX_DEPTH_M), 3)
        reason = "front view only; camera height and level mounting must be calibrated"

    cells = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
    origin_col, origin_row = GRID_SIZE // 2, GRID_SIZE - 1
    obstacle_cells = []
    # Combine adjacent image columns conservatively: the nearest obstacle
    # truncates every free-space ray in that column group. Endpoints are applied
    # after carving, so later rays cannot erase an observed obstacle.
    for col in range(0, width, 4):
        stop = min(width, col + 4)
        block_obstacles = obstacles[:, col:stop]
        if np.any(block_obstacles):
            block_depth = np.where(block_obstacles, z[:, col:stop], np.inf)
            row_index, relative_col = np.unravel_index(np.argmin(block_depth), block_depth.shape)
            ray_col = col + int(relative_col)
            distance = float(z[row_index, ray_col])
            occupied = True
        else:
            observed = valid[:, col:stop] & (above_floor[:, col:stop] >= -0.03) & (above_floor[:, col:stop] <= obstacle_top_m)
            if not np.any(observed):
                continue
            block_depth = np.where(observed, z[:, col:stop], 0.0)
            row_index, relative_col = np.unravel_index(np.argmax(block_depth), block_depth.shape)
            ray_col = col + int(relative_col)
            distance = float(z[row_index, ray_col])
            occupied = False
        endpoint_x = (ray_col - frame.cx) * distance / frame.fx
        endpoint = (origin_col + round(endpoint_x / RESOLUTION_M),
                    origin_row - round(distance / RESOLUTION_M))
        ray = list(_line(origin_col, origin_row, *endpoint))
        for cell_col, cell_row in (ray[:-1] if occupied else ray):
            if 0 <= cell_col < GRID_SIZE and 0 <= cell_row < GRID_SIZE:
                cells[cell_row, cell_col] = 0
        if occupied and 0 <= endpoint[0] < GRID_SIZE and 0 <= endpoint[1] < GRID_SIZE:
            obstacle_cells.append(endpoint)
    for col, row in obstacle_cells:
        cells[row, col] = 1
    # The current camera origin is known, but unseen adjacent floor stays unknown.
    cells[origin_row, origin_col] = 0

    # At most 1200 points; include BGR->RGB conversion for browser display.
    stride = max(1, int(math.ceil(math.sqrt(depth.size / 1200))))
    sampled = valid[::stride, ::stride]
    xyz = np.stack((x[::stride, ::stride], y[::stride, ::stride], z[::stride, ::stride]), axis=-1)[sampled]
    rgb = frame.rgb[::stride, ::stride, ::-1][sampled]
    points = [[round(float(a), 3), round(float(b), 3), round(float(c), 3),
               int(r), int(g), int(bl)]
              for (a, b, c), (r, g, bl) in zip(xyz[:1200], rgb[:1200])]
    return {
        "clearance_m": clearance,
        "clearance_reason": reason,
        "front_depth_valid_fraction": round(front_fraction, 4),
        "depth_valid_fraction": round(float(np.mean(valid)), 4),
        "grid": {
            "width": GRID_SIZE, "height": GRID_SIZE,
            "resolution_m": RESOLUTION_M,
            "origin_col": origin_col, "origin_row": origin_row,
            "cells": cells.reshape(-1).astype(int).tolist(),
            "frame": "camera-local",
        },
        "points": points,
    }


def demo_planning_grid(local_grid: dict) -> dict:
    """Add an explicitly fictional start fixture for the synthetic demo ONLY.

    The fixed demo assumes its rover starts in a measured, clear disk of radius
    0.50 m. That prior is not a sensor observation and must never be applied to
    hardware scans. Moving the origin six cells upward makes room to display
    the rear half of the fixture. Existing occupied cells always win.

    This returns a new grid and never edits the camera-local source grid.
    """
    if local_grid.get("frame") != "camera-local":
        raise ValueError("Demo fixture requires an original camera-local grid.")
    width, height = int(local_grid["width"]), int(local_grid["height"])
    resolution = float(local_grid["resolution_m"])
    if not math.isfinite(resolution) or resolution <= 0 or height <= 6 or width <= 0:
        raise ValueError("Invalid dimensions or resolution for demo grid.")
    origin_col = int(local_grid["origin_col"])
    origin_row = int(local_grid["origin_row"]) - 6
    if not (0 <= origin_col < width and 0 <= origin_row < height):
        raise ValueError("Demo origin would fall outside grid.")
    original = np.asarray(local_grid["cells"], dtype=np.int8).reshape(height, width)
    cells = np.full((height, width), -1, dtype=np.int8)
    cells[:-6, :] = original[6:, :]
    rows, cols = np.ogrid[:height, :width]
    fixture = (((cols - origin_col) * resolution) ** 2 +
               ((rows - origin_row) * resolution) ** 2) <= (0.5 ** 2 + 1e-10)
    cells[fixture & (cells != 1)] = 0
    return {**local_grid, "origin_row": origin_row,
            "cells": cells.reshape(-1).astype(int).tolist(),
            "frame": "synthetic-demo",
            "note": "Synthetic start clearance supplied by demo fixture; not sensed"}


def plan_path(grid: dict, goal: list[int], robot_radius_m: float = 0.18) -> list[list[int]]:
    """A* on observed free cells after inflating both obstacles AND unknown.

    No diagonal corner cutting. A front-only scan has insufficient known
    floor around the start for a complete footprint and correctly returns [].
    This path is an informational local overlay, never autonomous motor control.
    Grid exterior is unknown too. A nonzero footprint cannot start at the edge
    of a front-only grid; a wider observed map is needed for a usable route.
    """
    try:
        width, height = int(grid["width"]), int(grid["height"])
        resolution = float(grid["resolution_m"])
        if width <= 0 or height <= 0 or width * height > 1000000:
            return []
        if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(robot_radius_m) or robot_radius_m < 0:
            return []
        if len(goal) != 2 or any(float(value) != int(value) for value in goal):
            return []
        target = (int(goal[0]), int(goal[1]))
        start = (int(grid["origin_col"]), int(grid["origin_row"]))
        raw = np.asarray(grid["cells"], dtype=np.int16).reshape(height, width)
    except (KeyError, ValueError, TypeError, OverflowError):
        return []
    if any(not (0 <= p[0] < width and 0 <= p[1] < height) for p in (start, target)):
        return []
    radius = int(math.ceil(robot_radius_m / resolution))
    if radius > max(width, height):
        return []
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    footprint = ((xx * xx + yy * yy) <= radius * radius).astype(np.uint8)
    blocked = cv2.dilate((raw != 0).astype(np.uint8), footprint,
                         borderType=cv2.BORDER_CONSTANT, borderValue=1).astype(bool)
    if blocked[start[1], start[0]] or blocked[target[1], target[0]]:
        return []

    def heuristic(node):
        dx, dy = abs(node[0] - target[0]), abs(node[1] - target[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    queue = [(heuristic(start), 0.0, start)]
    costs = {start: 0.0}
    came_from = {}
    neighbors = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs.get(current, math.inf):
            continue
        if current == target:
            path = [list(current)]
            while current in came_from:
                current = came_from[current]
                path.append(list(current))
            return path[::-1]
        for dx, dy in neighbors:
            col, row = current[0] + dx, current[1] + dy
            if not (0 <= col < width and 0 <= row < height) or blocked[row, col]:
                continue
            if dx and dy and (blocked[current[1], col] or blocked[row, current[0]]):
                continue
            node = (col, row)
            candidate_cost = cost + (math.sqrt(2) if dx and dy else 1.0)
            if candidate_cost < costs.get(node, math.inf):
                costs[node] = candidate_cost
                came_from[node] = current
                heapq.heappush(queue, (candidate_cost + heuristic(node), candidate_cost, node))
    return []
