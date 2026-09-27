"""Created by OpenAI Codex: sensor/control math, testable without Isaac Sim."""
import math
import numpy as np


def yaw_from_quaternion(q):
    w, x, y, z = q  # Isaac experimental API uses wxyz.
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def wheel_speeds(linear, angular, radius, track):
    if radius <= 0 or track <= 0:
        raise ValueError('Wheel radius and track must be positive')
    left = (linear - angular * track / 2) / radius
    right = (linear + angular * track / 2) / radius
    return np.array([left, left, right, right])


def intrinsics(width, height, hfov_deg):
    f = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return np.array([[f, 0, width/2], [0, f, height/2], [0, 0, 1.]])


def optical_points(depth, K, cfg, mask=None):
    """Image-plane depth -> RealSense-style x-right/y-down/z-forward points."""
    s = cfg.depth_stride
    z = depth[::s, ::s]
    v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
    valid = np.isfinite(z) & (z > cfg.min_range_m) & (z < cfg.max_range_m)
    if mask is not None:
        valid &= mask[::s, ::s]
    return np.column_stack(((u[valid]-K[0,2])*z[valid]/K[0,0],
                            (v[valid]-K[1,2])*z[valid]/K[1,1], z[valid]))


def body_gaps(x, y, radius, cfg, boxes, flame, flame_radius=.05):
    """Conservative circular footprint clearances; not PhysX contact counts."""
    edge = min(x-cfg.x_min, cfg.x_max-x, y-cfg.y_min, cfg.y_max-y)-radius
    box_gap = math.inf
    for x0, y0, x1, y1 in boxes:
        dx, dy = max(x0-x, 0., x-x1), max(y0-y, 0., y-y1)
        d = math.hypot(dx, dy)
        if x0 <= x <= x1 and y0 <= y <= y1:
            d = -min(x-x0, x1-x, y-y0, y1-y)
        box_gap = min(box_gap, d-radius)
    fire_gap = math.inf if flame is None else math.hypot(x-flame[0], y-flame[1])-flame_radius-radius
    return dict(edge=edge, box=box_gap, fire=fire_gap)
