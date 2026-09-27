"""Seeded physical interiors and a bounded, ray-acquired synthetic surface mesh.

The navigation grid is conservative configuration space for a 0.20 m radius
rover. It is not the geometry rendered by the depth scanner. The scanner uses
continuous floor, wall and furniture surfaces, with a deliberately synthetic
forward camera sweep; it is not a model of the front-facing D435i.
"""
from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
import math
import random

import numpy as np


FAMILIES = ("branching_rooms", "corridor_apartments", "open_plan", "irregular_suites")


@dataclass(frozen=True)
class SurfaceObject:
    """A vertical oriented box, or a cylinder with radius in half_x."""

    kind: str
    x: float
    z: float
    half_x: float
    half_z: float
    height: float
    angle: float = 0.0
    color: tuple = (.53, .62, .67)
    role: str = "furniture"

    def distance(self, x, z):
        dx, dz = np.asarray(x) - self.x, np.asarray(z) - self.z
        if self.kind == "cylinder":
            return np.maximum(0, np.hypot(dx, dz) - self.half_x)
        co, si = math.cos(self.angle), math.sin(self.angle)
        lx, lz = co * dx + si * dz, -si * dx + co * dz
        return np.hypot(np.maximum(0, np.abs(lx) - self.half_x),
                        np.maximum(0, np.abs(lz) - self.half_z))


@dataclass
class SyntheticWorld:
    cells: np.ndarray
    raw_cells: np.ndarray
    home: tuple[int, int]
    resolution_m: float
    width_m: float
    height_m: float
    family: str
    objects: list[SurfaceObject]
    seed: int
    rover_radius_m: float = .20


def _component(cells, start=None):
    free = np.argwhere(cells == 0)
    if len(free) == 0:
        return set()
    if start is None:
        start = (int(free[0, 1]), int(free[0, 0]))
    if cells[start[1], start[0]]:
        return set()
    seen, queue = {start}, deque([start])
    height, width = cells.shape
    while queue:
        col, row = queue.popleft()
        for x, z in ((col - 1, row), (col + 1, row), (col, row - 1), (col, row + 1)):
            if 0 <= x < width and 0 <= z < height and not cells[z, x] and (x, z) not in seen:
                seen.add((x, z))
                queue.append((x, z))
    return seen


def build_world(width_m, height_m, seed):
    """Create replayable, connected interiors in an exact 1..50 metre boundary.

    Grid cells include a half-diagonal guard around internal obstacles so that
    adjacent free cell centres have continuous footprint clearance as well.
    Boundary clearance uses the actual rover radius, retaining thin corridors.
    """
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or not 1 <= v <= 50 for v in (width_m, height_m)):
        raise ValueError("World width and height must be finite metres in 1..50")
    if type(seed) is not int or not 0 <= seed <= 2147483647:
        raise ValueError("World seed must be an integer in 0..2147483647")
    width_m, height_m = float(width_m), float(height_m)
    rng = random.Random(seed)
    resolution = max(.10, min(.2, width_m / 10, height_m / 10), width_m / 80, height_m / 80)
    cols, rows = math.ceil(width_m / resolution), math.ceil(height_m / resolution)
    gx, gz = np.meshgrid((np.arange(cols) + .5) * resolution,
                         (np.arange(rows) + .5) * resolution)
    radius = .20
    inside = (gx >= radius) & (gz >= radius) & (gx <= width_m - radius) & (gz <= height_m - radius)
    cells = np.where(inside, 0, 1).astype(np.int8)
    family = FAMILIES[rng.randrange(len(FAMILIES))]
    objects = []
    # Perimeter walls lie outside the advertised footprint. This keeps exact
    # bounds even where ceil(grid size) makes the final raster cell partial.
    wall_color = (.55 + rng.random() * .12, .63 + rng.random() * .10, .67 + rng.random() * .10)
    for x, z, hx, hz in ((width_m / 2, -.05, width_m / 2 + .1, .05),
                          (width_m / 2, height_m + .05, width_m / 2 + .1, .05),
                          (-.05, height_m / 2, .05, height_m / 2),
                          (width_m + .05, height_m / 2, .05, height_m / 2)):
        objects.append(SurfaceObject("box", x, z, hx, hz, 2.45, color=wall_color, role="wall"))
    guard = radius + resolution / math.sqrt(2)

    def accept(group, minimum=6):
        nonlocal cells
        candidate = cells.copy()
        for obj in group:
            candidate[obj.distance(gx, gz) <= guard] = 1
        remaining = int(np.count_nonzero(candidate == 0))
        if remaining < minimum or remaining < np.count_nonzero(cells == 0) * .52:
            return False
        if len(_component(candidate)) != remaining:
            return False
        cells = candidate
        objects.extend(group)
        return True

    def wall(x1, z1, x2, z2, door=False):
        dx, dz = x2 - x1, z2 - z1
        length = math.hypot(dx, dz)
        if length < .4:
            return []
        angle = math.atan2(dz, dx)
        intervals = [(0., 1.)]
        if door:
            gap = max(.95, 2 * guard + 1.4 * resolution)
            if length < gap + .60:
                return []
            center = rng.uniform(.30, .70)
            half = gap / length / 2
            center = max(half + .08, min(1 - half - .08, center))
            intervals = [(0., center - half), (center + half, 1.)]
        return [SurfaceObject("box", x1 + dx * (a + b) / 2, z1 + dz * (a + b) / 2,
                              length * (b - a) / 2, .045, rng.uniform(2.25, 2.65),
                              angle, wall_color, "wall") for a, b in intervals if b - a > .01]

    # Geometry families have different graph structures, rather than perturbing
    # the doors of a single cross-shaped four-room floor plan.
    if min(width_m, height_m) >= 3:
        if family == "branching_rooms":
            rooms = [(0., 0., width_m, height_m, 0)]
            while rooms:
                x1, z1, x2, z2, depth = rooms.pop(rng.randrange(len(rooms)))
                rw, rh = x2 - x1, z2 - z1
                if depth >= rng.randint(2, 4) or min(rw, rh) < 2.1:
                    continue
                vertical = rw > rh * 1.25 or (rw >= rh / 1.25 and rng.random() < .5)
                if vertical:
                    cut = x1 + rw * rng.uniform(.32, .68)
                    group = wall(cut, z1, cut, z2, True)
                    children = [(x1, z1, cut, z2, depth + 1), (cut, z1, x2, z2, depth + 1)]
                else:
                    cut = z1 + rh * rng.uniform(.32, .68)
                    group = wall(x1, cut, x2, cut, True)
                    children = [(x1, z1, x2, cut, depth + 1), (x1, cut, x2, z2, depth + 1)]
                if group and accept(group):
                    rooms.extend(children)
        elif family == "corridor_apartments":
            horizontal = rng.choice((True, False))
            long_side, short_side = (width_m, height_m) if horizontal else (height_m, width_m)
            center = short_side * rng.uniform(.38, .62)
            half_width = max(.55, resolution * 1.9)
            count = rng.randint(2, max(2, min(5, int(long_side / 2))))
            cuts = [0.] + sorted(long_side * (i / count + rng.uniform(-.06, .06)) for i in range(1, count)) + [long_side]
            for side in (-1, 1):
                edge = max(.8, min(short_side - .8, center + side * half_width))
                for a, b in zip(cuts, cuts[1:]):
                    group = wall(a, edge, b, edge, True) if horizontal else wall(edge, a, edge, b, True)
                    if group:
                        accept(group)
                for cut in cuts[1:]:
                    end = 0 if side < 0 else short_side
                    group = wall(cut, end, cut, edge) if horizontal else wall(end, cut, edge, cut)
                    accept(group)
        elif family == "open_plan":
            for _ in range(rng.randint(2, 6)):
                x, z = rng.uniform(.8, width_m - .8), rng.uniform(.8, height_m - .8)
                angle = rng.choice((0, math.pi / 2, math.pi / 4, -math.pi / 4)) + rng.uniform(-.12, .12)
                length = rng.uniform(.7, min(width_m, height_m) * .48)
                dx, dz = math.cos(angle) * length / 2, math.sin(angle) * length / 2
                if .3 < x - abs(dx) and x + abs(dx) < width_m - .3 and .3 < z - abs(dz) and z + abs(dz) < height_m - .3:
                    accept(wall(x - dx, z - dz, x + dx, z + dz))
        else:
            # Offset L-shaped suites and short angled dividers; no universal
            # pair of crossing room-spanning walls.
            for _ in range(rng.randint(2, 5)):
                x, z = rng.uniform(width_m * .2, width_m * .8), rng.uniform(height_m * .2, height_m * .8)
                sx, sz = rng.choice((-1, 1)), rng.choice((-1, 1))
                end_x = min(width_m, max(0, x + sx * rng.uniform(.7, width_m * .42)))
                end_z = min(height_m, max(0, z + sz * rng.uniform(.7, height_m * .42)))
                accept(wall(end_x, z, x, z, abs(end_x - x) > 2) + wall(x, z, x, end_z, abs(end_z - z) > 2))

    # Home is sampled along a random perimeter side, not fixed southwest.
    free = np.argwhere(cells == 0)
    side = rng.randrange(4)
    target_x, target_z = ((rng.uniform(.12, .88) * width_m, 0),
                           (width_m, rng.uniform(.12, .88) * height_m),
                           (rng.uniform(.12, .88) * width_m, height_m),
                           (0, rng.uniform(.12, .88) * height_m))[side]
    row, col = min(free, key=lambda p: ((p[1] + .5) * resolution - target_x) ** 2 + ((p[0] + .5) * resolution - target_z) ** 2)
    home = (int(col), int(row))
    home_x, home_z = (col + .5) * resolution, (row + .5) * resolution
    desired = min(28, max(0, int(width_m * height_m / rng.uniform(5, 10))))
    furniture_colors = ((.35, .49, .48), (.48, .42, .36), (.38, .43, .57), (.56, .49, .39))
    for _ in range(desired * 8):
        if sum(o.role == "furniture" for o in objects) >= desired:
            break
        x, z = rng.uniform(.45, width_m - .45), rng.uniform(.45, height_m - .45)
        if math.hypot(x - home_x, z - home_z) < 1.0:
            continue
        cylinder = rng.random() < .40
        hx = rng.uniform(.17, min(.58, min(width_m, height_m) / 5))
        hz = hx if cylinder else rng.uniform(.20, min(.75, min(width_m, height_m) / 4))
        obj = SurfaceObject("cylinder" if cylinder else "box", x, z, hx, hz,
                            rng.uniform(.38, 1.30), rng.uniform(0, math.pi), rng.choice(furniture_colors))
        if obj.distance(home_x, home_z) <= guard + .20:
            continue
        # Avoid decorative objects visibly intersecting walls or each other.
        extent = hx if cylinder else math.hypot(hx, hz)
        if any(existing.distance(x, z) < extent + .06 for existing in objects):
            continue
        accept([obj])
    raw = np.zeros_like(cells)
    raw[(gx < 0) | (gz < 0) | (gx > width_m) | (gz > height_m)] = 1
    # Physical occupancy represents cell overlap, not only cell centres: a
    # thin wall between two centres must stop cell-to-cell fire spread. This
    # guard is raster uncertainty only and does not include rover radius.
    # Perimeter walls are handled by the exact boundary mask above, since a
    # coarse edge cell can be partially outside yet have usable interior.
    for obj in objects[4:]:
        raw[obj.distance(gx, gz) <= resolution / math.sqrt(2)] = 1
    return SyntheticWorld(cells, raw, home, resolution, width_m, height_m, family, objects, seed)


class SyntheticDepthScan:
    """Raycast a forward RGB-D camera and retain its observed surface patches.

    The rover camera is fixed to the front of the chassis.  The synthetic
    model uses an 87 degree horizontal field of view and an 80 degree vertical
    field of view, with the current rover heading as the optical axis.  It
    does not see behind the rover until the rover turns or revisits that area.
    Patches are fused in a bounded cache, so surfaces acquired earlier remain
    in the mesh while the rover moves.  This is an observable camera mesh, not
    a hidden global-world reconstruction.
    """

    MAX_VERTICES = 12000
    MAX_TRIANGLES = 4000

    def __init__(self, world: SyntheticWorld):
        self.world = world
        self.revision = 0
        self._patches = OrderedDict()
        self._snapshot = None
        self._last_pose = None
        self._origin = None
        self._latest = []
        self._seen_surfaces = set()
        self.max_range_m = 8.0
        self._voxel = max(.115, math.sqrt(world.width_m * world.height_m) / 65)
        self.horizontal_fov_deg = 87.0
        self.vertical_fov_deg = 80.0
        self._azimuths = 96
        self._elevations = 16
        elevation = np.radians(np.linspace(-40.0, 40.0, self._elevations))
        azimuth = np.radians(np.linspace(-self.horizontal_fov_deg / 2,
                                         self.horizontal_fov_deg / 2,
                                         self._azimuths))
        az, el = np.meshgrid(azimuth, elevation)
        self._shape = az.shape
        self._directions = np.column_stack((np.cos(el).ravel() * np.cos(az).ravel(),
                                            np.sin(el).ravel(), np.cos(el).ravel() * np.sin(az).ravel()))
        self._palette = [(0.25, .34, .38)]
        for obj in world.objects:
            self._palette.extend([obj.color] * 8)
        self._palette = np.asarray(self._palette)

    def _cast(self, origin, directions):
        """Return closest positive range and physical surface id for each ray."""
        count = len(directions)
        best, surface = np.full(count, np.inf), np.full(count, -1, dtype=np.int32)
        down = directions[:, 1] < -1e-9
        floor_t = np.divide(-origin[1], directions[:, 1], out=np.full(count, np.inf), where=down)
        floor_x = origin[0] + np.where(down, floor_t, 0) * directions[:, 0]
        floor_z = origin[2] + np.where(down, floor_t, 0) * directions[:, 2]
        valid_floor = down & (floor_x >= 0) & (floor_z >= 0) & (floor_x <= self.world.width_m) & (floor_z <= self.world.height_m)
        best[valid_floor], surface[valid_floor] = floor_t[valid_floor], 0
        for index, obj in enumerate(self.world.objects):
            base_id = 1 + index * 8
            co, si = math.cos(obj.angle), math.sin(obj.angle)
            dx, dz = origin[0] - obj.x, origin[2] - obj.z
            local_o = np.array([co * dx + si * dz, origin[1], -si * dx + co * dz])
            local_d = np.column_stack((co * directions[:, 0] + si * directions[:, 2], directions[:, 1],
                                       -si * directions[:, 0] + co * directions[:, 2]))
            if obj.kind == "box":
                bounds_lo = np.array([-obj.half_x, 0., -obj.half_z])
                bounds_hi = np.array([obj.half_x, obj.height, obj.half_z])
                parallel = np.abs(local_d) < 1e-10
                inv = np.divide(1., local_d, out=np.zeros_like(local_d), where=~parallel)
                t1, t2 = (bounds_lo - local_o) * inv, (bounds_hi - local_o) * inv
                near, far = np.minimum(t1, t2), np.maximum(t1, t2)
                near[parallel], far[parallel] = -np.inf, np.inf
                parallel_outside = np.any(parallel & ((local_o < bounds_lo) | (local_o > bounds_hi)), axis=1)
                entry, leave = np.max(near, axis=1), np.min(far, axis=1)
                hit = (entry > 1e-5) & (leave >= entry) & ~parallel_outside
                axis = np.argmax(near, axis=1)
                face = axis * 2 + (local_d[np.arange(count), axis] > 0).astype(int)
                take = hit & (entry < best)
                best[take], surface[take] = entry[take], base_id + face[take]
            else:
                a = local_d[:, 0] ** 2 + local_d[:, 2] ** 2
                b = 2 * (local_o[0] * local_d[:, 0] + local_o[2] * local_d[:, 2])
                c = local_o[0] ** 2 + local_o[2] ** 2 - obj.half_x ** 2
                disc = b * b - 4 * a * c
                t = np.divide(-b - np.sqrt(np.maximum(disc, 0)), 2 * a,
                              out=np.full(count, np.inf), where=a > 1e-10)
                y = local_o[1] + t * local_d[:, 1]
                hit = (disc >= 0) & (t > 1e-5) & (y >= 0) & (y <= obj.height)
                take = hit & (t < best)
                best[take], surface[take] = t[take], base_id
                top_t = np.divide(obj.height - local_o[1], local_d[:, 1],
                                  out=np.full(count, np.inf), where=np.abs(local_d[:, 1]) > 1e-10)
                finite_top = np.isfinite(top_t)
                safe_top = np.where(finite_top, top_t, 0.)
                top_r2 = (local_o[0] + safe_top * local_d[:, 0]) ** 2 + (local_o[2] + safe_top * local_d[:, 2]) ** 2
                take = finite_top & (top_t > 1e-5) & (top_r2 <= obj.half_x ** 2) & (top_t < best)
                best[take], surface[take] = top_t[take], base_id + 1
        surface[best > self.max_range_m] = -1
        best[surface < 0] = np.nan
        return best, surface

    def update(self, position_cell, heading_rad=0):
        col, row = map(int, position_cell)
        if not (0 <= row < self.world.cells.shape[0] and 0 <= col < self.world.cells.shape[1]):
            raise ValueError("Scan position is outside world grid")
        if self.world.cells[row, col]:
            raise ValueError("Scan position must have rover footprint clearance")
        # Do not generate duplicate patches every dashboard refresh while the
        # rover remains stationary. A heading change is a real new camera view.
        heading_rad = float(heading_rad)
        if not math.isfinite(heading_rad):
            raise ValueError("Scan heading must be finite")
        pose = (col, row, round(heading_rad, 5))
        if pose == self._last_pose:
            return False
        self._last_pose = pose
        resolution = self.world.resolution_m
        origin = np.array([(col + .5) * resolution, .22, (row + .5) * resolution])
        # Rotate the camera-local rays around the vertical axis. heading=0
        # points along +x; positive headings turn toward increasing grid rows.
        cosine, sine = math.cos(heading_rad), math.sin(heading_rad)
        directions = self._directions.copy()
        local_x, local_z = directions[:, 0].copy(), directions[:, 2].copy()
        directions[:, 0] = cosine * local_x - sine * local_z
        directions[:, 2] = sine * local_x + cosine * local_z
        depth, surface = self._cast(origin, directions)
        points = origin + depth[:, None] * directions
        self._origin = origin.tolist()
        valid_points = points[surface >= 0]
        stride = max(1, math.ceil(len(valid_points) / 384))
        self._latest = np.round(valid_points[::stride], 3).tolist()
        self._seen_surfaces.update(int(value) for value in np.unique(surface) if value >= 0)
        h, w = self._shape
        indices = np.arange(h * w).reshape(h, w)
        top_left, top_right = indices[:-1].ravel(), np.roll(indices[:-1], -1, axis=1).ravel()
        bottom_left, bottom_right = indices[1:].ravel(), np.roll(indices[1:], -1, axis=1).ravel()
        candidates = np.concatenate((np.column_stack((top_left, bottom_left, top_right)),
                                     np.column_stack((top_right, bottom_left, bottom_right))))
        ids = surface[candidates]
        valid = (ids[:, 0] >= 0) & np.all(ids == ids[:, :1], axis=1)
        candidates, ids = candidates[valid], ids[valid, 0]
        vertices = points[candidates]
        # Same physical face plus bounded edge lengths rejects holes, depth
        # discontinuities and long skinny triangles along grazing rays.
        edges = np.linalg.norm(vertices - np.roll(vertices, 1, axis=1), axis=2)
        area = np.linalg.norm(np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0]), axis=1)
        good = (edges.max(axis=1) < 1.05) & (area > 1e-6)
        for verts, sid in zip(vertices[good], ids[good]):
            center = np.mean(verts, axis=0)
            key = (int(sid), *np.floor(center / self._voxel).astype(int).tolist())
            self._patches[key] = (verts.astype(np.float32), int(sid))
            self._patches.move_to_end(key)
        while len(self._patches) > self.MAX_TRIANGLES:
            self._patches.popitem(last=False)
        self.revision += 1
        self._snapshot = None
        return True

    def snapshot(self):
        if self._snapshot is None:
            vertices, triangles, colors, lookup = [], [], [], {}
            for points, sid in self._patches.values():
                tri = []
                for point in points:
                    rounded = tuple(round(float(v), 3) for v in point)
                    key = (sid, *rounded)
                    if key not in lookup:
                        lookup[key] = len(vertices)
                        vertices.append(list(rounded))
                        colors.append(self._palette[sid].round(3).tolist())
                    tri.append(lookup[key])
                if len(set(tri)) == 3:
                    triangles.append(tri)
            self._snapshot = {"vertices": vertices, "triangles": triangles, "colors": colors,
                              "revision": self.revision, "source": "synthetic raycast depth",
                              "coordinate_frame": "world_m_x_right_y_up_z_down", "units": "metres",
                              "scan_origin": self._origin, "heading_rad": self._last_pose[2] if self._last_pose else 0.0,
                              "latest_points": self._latest,
                              "ray_count": len(self._directions), "observed_surface_count": len(self._seen_surfaces),
                              "vertex_count": len(vertices), "triangle_count": len(triangles),
                              "max_range_m": self.max_range_m, "camera_height_m": .22,
                              "accumulation": "persistent bounded observed surface patches; no hidden truth mesh",
                              "sensor_model": "synthetic forward RGB-D camera, 87 degree horizontal FOV, not D435i",
                              "horizontal_fov_deg": self.horizontal_fov_deg,
                              "vertical_fov_deg": self.vertical_fov_deg,
                              "family": self.world.family}
        return self._snapshot
