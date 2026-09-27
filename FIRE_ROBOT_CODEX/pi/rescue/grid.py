# CODEX COPY — modifications by OpenAI Codex.
# Original robot-project authorship is not claimed.
# Changes included in this file:
# - Added block_edges() to reserve robot-radius-plus-clearance bands along all four map edges.
# - blocked_mask() now applies those bands using the active clearance, including fallback values.
# - Guarded zero-width bands to avoid Python negative-zero slicing blocking the entire map.
# See CODEX_CHANGES.md at the project root for scope and validation.

"""2D occupancy grid built from camera observations.

Each cell is UNKNOWN, FREE, OCC (obstacle) or FIRE. Planning never uses the raw
states directly: blocked_mask() grows obstacles by the robot's radius so A* can
treat the car as a point.
"""

import math
from dataclasses import dataclass, field

import numpy as np

# Cell states stored in OccupancyGrid.state.
UNKNOWN, FREE, OCC, FIRE = 0, 1, 2, 3


@dataclass
class Observation:
    """Points seen by the camera, in the robot frame (x forward, y left), meters."""
    floor: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    obstacles: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    flame: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))


def dilate(mask, r):
    """Grow True cells by a disk of radius r cells."""
    out = mask.copy()
    if r <= 0:
        return out
    h, w = mask.shape
    # OR in a shifted copy of the mask for every offset inside the disk.
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            # Skip offsets outside the disk, or larger than the array (empty slice).
            if dx * dx + dy * dy > r * r or abs(dy) >= h or abs(dx) >= w:
                continue
            # The slices shift the source by (dx, dy) while clipping at the edges.
            out[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)] |= \
                mask[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)]
    return out


class OccupancyGrid:
    def __init__(self, cfg):
        self.cfg = cfg
        self.res = cfg.cell_m
        self.x0, self.y0 = cfg.x_min, cfg.y_min
        # Grid size in cells; state is indexed [iy, ix] (row = y).
        self.nx = int(round((cfg.x_max - cfg.x_min) / self.res))
        self.ny = int(round((cfg.y_max - cfg.y_min) / self.res))
        self.state = np.zeros((self.ny, self.nx), dtype=np.uint8)

    # --- coordinates ---

    def cell(self, xy):
        """World point -> (ix, iy), or None if outside the map."""
        ix = math.floor((xy[0] - self.x0) / self.res)
        iy = math.floor((xy[1] - self.y0) / self.res)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return ix, iy
        return None

    def center(self, c):
        """Cell (ix, iy) -> world point at the middle of that cell."""
        return (self.x0 + (c[0] + 0.5) * self.res, self.y0 + (c[1] + 0.5) * self.res)

    def _flat_indices(self, pts):
        """World (N,2) points -> flat indices into state, dropping points off the map."""
        if len(pts) == 0:
            return np.empty(0, dtype=np.int64)
        ix = np.floor((pts[:, 0] - self.x0) / self.res).astype(np.int64)
        iy = np.floor((pts[:, 1] - self.y0) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return iy[ok] * self.nx + ix[ok]

    # --- updates ---

    def update(self, obs, pose):
        """Fold one observation (robot frame) taken at `pose` into the map.

        Floor seen in a cell with no obstacle points clears it, so obstacles
        that moved away disappear on the next look.
        """
        n = self.nx * self.ny
        # Per-cell hit counts for each kind of point, after moving them to the world frame.
        count = lambda pts: np.bincount(self._flat_indices(pose.to_world(pts)), minlength=n)
        floor = count(obs.floor) > 0
        # Require several hits before trusting an obstacle or flame, to filter depth noise.
        occ = count(obs.obstacles) >= self.cfg.min_obstacle_hits
        fire = count(obs.flame) >= self.cfg.min_obstacle_hits

        # Write through a flat view: fire wins over obstacle, obstacle over floor.
        s = self.state.reshape(-1)
        s[floor & ~occ & ~fire] = FREE
        s[occ] = OCC
        s[fire] = FIRE

    def mark(self, xy, value=OCC):
        """Set the cell containing world point xy (ignored if off the map)."""
        c = self.cell(xy)
        if c:
            self.state[c[1], c[0]] = value

    # --- planning views ---

    def blocked_mask(self, clearance=None, fire_margin=None):
        """Cells the car's center must not enter.

        Obstacles are grown by robot radius + clearance, fire by an extra
        fire_margin on top. Pass None to use the config's values.
        """
        cfg = self.cfg
        clearance = cfg.clearance_m if clearance is None else clearance
        fire_margin = cfg.fire_margin_m if fire_margin is None else fire_margin
        # Inflation radii in cells; the epsilon stops float error from rounding up a whole cell.
        r = math.ceil((cfg.robot_radius_m + clearance) / self.res - 1e-9)
        rf = math.ceil((cfg.robot_radius_m + clearance + fire_margin) / self.res - 1e-9)

        # The camera only sees the near face of a box. Assume it has some depth.
        shadow = math.ceil(cfg.assumed_depth_m / self.res - 1e-9)
        unknown = self.state == UNKNOWN
        occ, fire = self.state == OCC, self.state == FIRE
        # Only extend into unknown cells: floor seen behind the face stays free.
        occ |= dilate(occ, shadow) & unknown
        fire |= dilate(fire, shadow) & unknown

        blocked = dilate(occ, r) | dilate(fire, rf)
        # Optionally treat unexplored space as a wall (more cautious).
        if not cfg.unknown_is_free:
            blocked |= self.state == UNKNOWN
        self.block_edges(blocked, clearance)
        return blocked

    def block_edges(self, mask, clearance=None):
        """Keep the full body and active clearance inside the map bounds."""
        if not self.cfg.edge_is_wall:
            return
        clearance = self.cfg.clearance_m if clearance is None else clearance
        r = math.ceil((self.cfg.robot_radius_m + clearance) / self.res - 1e-9)
        if r > 0:
            mask[:r, :] = mask[-r:, :] = True
            mask[:, :r] = mask[:, -r:] = True

    def clear_disk(self, mask, xy, radius):
        """Unblock the cells under the car: it's physically there, so they're free."""
        r = math.ceil(radius / self.res)
        c = self.cell(xy)
        if not c:
            return
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                x, y = c[0] + dx, c[1] + dy
                if dx * dx + dy * dy <= r * r and 0 <= x < self.nx and 0 <= y < self.ny:
                    mask[y, x] = False

    def segment_clear(self, mask, a, b):
        """True if the straight line a->b stays inside the map on unblocked cells."""
        # Sample every half cell so the line can't skip over a blocked cell.
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res / 2))))
        for i in range(n + 1):
            t = i / n
            c = self.cell((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            if c is None or mask[c[1], c[0]]:
                return False
        return True
