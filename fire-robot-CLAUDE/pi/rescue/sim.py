# ======================================================================
# MODIFIED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# Search for "[CLAUDE EDIT]" to find each change. Full list: CLAUDE_CHANGES.md
# ======================================================================
# Change here: simulator counts crossing the map edge as a collision.

"""A 2D stand-in for the car and the D435, for testing the mission on a laptop.

SimWorld holds the true obstacle layout and the true car pose. SimCamera
ray-casts it to produce the same Observation the real camera produces, and
SimBase moves the true pose with some error, like timed moves on the real car.
The mission only ever sees its own estimated pose, as on the real robot.
"""

import math
from dataclasses import dataclass

import numpy as np

from .geometry import Pose
from .grid import Observation

RES = 0.01  # truth raster resolution, meters


@dataclass
class Box:
    x0: float
    y0: float
    x1: float
    y1: float


class SimWorld:
    def __init__(self, cfg, boxes=(), flame=None, flame_r=0.05, seed=0,
                 turn_noise=0.03, move_noise=0.03):
        self.cfg = cfg
        self.x0, self.y0 = cfg.x_min, cfg.y_min
        self.w = int(round((cfg.x_max - cfg.x_min) / RES))
        self.h = int(round((cfg.y_max - cfg.y_min) / RES))
        self.raster = np.zeros((self.h, self.w), dtype=np.uint8)  # 0 free, 1 obstacle, 2 fire
        self.boxes, self.flame, self.flame_r = [], flame, flame_r
        for b in boxes:
            self.add_box(b)
        if flame:
            # Rasterize the flame as a disk (label 2).
            yy, xx = np.mgrid[0:self.h, 0:self.w]
            cx, cy = self._px(flame)
            self.raster[(xx - cx) ** 2 + (yy - cy) ** 2 <= (flame_r / RES) ** 2] = 2
        self.pose = Pose()  # true pose
        self.rng = np.random.default_rng(seed)
        self.turn_noise, self.move_noise = turn_noise, move_noise
        self.collisions = 0
        self.trail = [self.pose.xy]

    @classmethod
    def demo(cls, cfg, seed=0, **kw):
        """A tabletop-sized layout with a flame near the straight-line route."""
        boxes = [Box(0.55, -0.30, 0.75, 0.15),
                 Box(0.95, 0.40, 1.10, 0.60),
                 Box(1.20, -0.65, 1.40, -0.30),
                 Box(1.65, 0.05, 1.80, 0.25)]
        return cls(cfg, boxes, flame=(1.30, 0.20), seed=seed, **kw)

    def _px(self, xy):
        """World point -> (column, row) in the truth raster."""
        return int((xy[0] - self.x0) / RES), int((xy[1] - self.y0) / RES)

    def add_box(self, b):
        self.boxes.append(b)
        (i0, j0), (i1, j1) = self._px((b.x0, b.y0)), self._px((b.x1, b.y1))
        # Clip negative indices to 0 so boxes partly off the map still work.
        self.raster[max(0, j0):max(0, j1), max(0, i0):max(0, i1)] = 1

    def label(self, x, y):
        """Truth label at a world point: 0 free, 1 obstacle, 2 fire. Off the map is free."""
        i, j = self._px((x, y))
        if 0 <= i < self.w and 0 <= j < self.h:
            return int(self.raster[j, i])
        return 0

    def ray(self, x, y, angle, max_d):
        """Distance to the first obstacle along a ray and its label, or (max_d, 0)."""
        c, s = math.cos(angle), math.sin(angle)
        d = 0.0
        # March along the ray one raster cell at a time.
        while d < max_d:
            lab = self.label(x + d * c, y + d * s)
            if lab:
                return d, lab
            d += RES
        return max_d, 0

    def collides(self, x, y, radius):
        """True if the body crosses a wall boundary or touches obstacle/fire."""
        # [CLAUDE EDIT] New: body crossing the arena border counts as a collision in the sim,
        #   so edge crossings show up in test/collision counts instead of being ignored.
        if self.cfg.edge_is_wall and (
                x - radius < self.cfg.x_min or x + radius > self.cfg.x_max or
                y - radius < self.cfg.y_min or y + radius > self.cfg.y_max):
            return True
        i, j = self._px((x, y))
        r = int(math.ceil(radius / RES))
        # Check the bounding square first, then the disk inside it.
        sub = self.raster[max(0, j - r):j + r + 1, max(0, i - r):i + r + 1]
        if not sub.any():
            return False
        yy, xx = np.nonzero(sub)
        # Offsets from the center pixel, accounting for clipping at the map edge.
        yy, xx = yy + max(0, j - r) - j, xx + max(0, i - r) - i
        return bool(((xx ** 2 + yy ** 2) <= r * r).any())


class SimCamera:
    def __init__(self, world, n_rays=120, noise_m=0.01):
        self.world, self.cfg = world, world.cfg
        self.n_rays, self.noise_m = n_rays, noise_m

    def observe(self, yaw=0.0):
        w, cfg, pose = self.world, self.cfg, self.world.pose
        # The camera sits cam_forward_m ahead of the car's center.
        cx = pose.x + cfg.cam_forward_m * math.cos(pose.theta)
        cy = pose.y + cfg.cam_forward_m * math.sin(pose.theta)
        half = math.radians(cfg.hfov_deg) / 2
        floor, obst, flame = [], [], []
        # Fan of rays across the horizontal field of view.
        for a in np.linspace(-half, half, self.n_rays):
            ang = pose.theta + yaw + a
            c, s = math.cos(ang), math.sin(ang)
            d, lab = w.ray(cx, cy, ang, cfg.max_range_m)
            # Floor points along the ray, up to just short of the hit.
            for r in np.arange(cfg.min_range_m, d - 0.02, cfg.cell_m):
                floor.append((cx + r * c, cy + r * s))
            if lab and d >= cfg.min_range_m:
                # several points per hit, like a vertical strip of depth pixels
                for k in range(cfg.min_obstacle_hits + 1):
                    p = (cx + (d + 0.005 * k) * c, cy + (d + 0.005 * k) * s)
                    obst.append(p)
                    if lab == 2:
                        flame.append(p)

        def to_robot(pts):
            # Add depth noise, then express in the robot frame like the real camera.
            if not pts:
                return np.empty((0, 2))
            pts = np.asarray(pts) + w.rng.normal(0, self.noise_m, (len(pts), 2))
            return pose.to_robot(pts)

        return Observation(to_robot(floor), to_robot(obst), to_robot(flame))


class SimBase:
    """Timed moves with realistic error.

    Without the IMU: each turn is off by ~turn_noise (3%) and straight moves veer.
    With the IMU (cfg.use_imu): turns stop within about a degree, straight moves
    are steered to hold heading, and heading() reports the true heading plus a
    little gyro noise and drift.
    """
    BODY_RADIUS = 0.12  # the real car's footprint, a bit smaller than the planning radius

    def __init__(self, world, veer_deg_per_m=4.0, gyro_noise_deg=0.3, gyro_drift_deg=0.2):
        self.world, self.cfg = world, world.cfg
        self.imu = world.cfg.use_imu
        self.veer = math.radians(veer_deg_per_m)
        self.gyro_noise = math.radians(gyro_noise_deg)
        self.gyro_drift = math.radians(gyro_drift_deg)  # random walk per move
        self.gyro_bias = 0.0

    def heading(self):
        if not self.imu:
            return None
        return self.world.pose.theta + self.gyro_bias + self.world.rng.normal(0, self.gyro_noise)

    def pan(self, yaw):
        pass  # SimCamera takes the yaw directly

    def turn(self, angle):
        w = self.world
        # Gyro-stopped turn: nearly exact, but the gyro slowly drifts.
        if self.imu:
            w.pose.theta += angle + w.rng.normal(0, math.radians(1.0))
            self.gyro_bias += w.rng.normal(0, self.gyro_drift)
            return angle  # the mission re-reads heading() anyway
        # Timed turn: error proportional to the angle.
        w.pose.theta += angle * (1 + w.rng.normal(0, w.turn_noise))
        return angle

    def move(self, dist):
        """Advance in small steps, stopping like the sonar would.

        Returns (estimated distance, stopped_by_sonar), like UnoBase.move.
        """
        w, cfg, pose = self.world, self.cfg, self.world.pose
        scale = 1 + w.rng.normal(0, w.move_noise)  # true distance / estimated distance
        veer = w.rng.normal(0, self.veer) * (0.1 if self.imu else 1.0)  # rad per meter
        step, est, hit, stopped = 0.005, 0.0, False, False
        while est < dist - 1e-9:
            # Simulated sonar: anything within sonar_stop_m of the front bumper stops the car.
            fx = pose.x + cfg.robot_front_m * math.cos(pose.theta)
            fy = pose.y + cfg.robot_front_m * math.sin(pose.theta)
            if w.ray(fx, fy, pose.theta, cfg.sonar_stop_m)[1]:
                stopped = True
                break
            ds = min(step, dist - est)
            pose.advance(ds * scale)
            pose.theta += veer * ds * scale
            est += ds
            # Count each move's collision once; the tests assert there are none.
            if not hit and w.collides(pose.x, pose.y, self.BODY_RADIUS):
                w.collisions += 1
                hit = True
        if self.imu:
            self.gyro_bias += w.rng.normal(0, self.gyro_drift)
        w.trail.append(pose.xy)
        return est, stopped

    def stop(self):
        pass
