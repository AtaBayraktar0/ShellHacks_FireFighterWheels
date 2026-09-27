"""Camera geometry and flame detection, tested without a D435."""

import math

import numpy as np
import pytest

from rescue.config import Config
from rescue.perception import camera_to_robot, classify, fit_floor, flame_mask


def optical_ray_to_floor(cfg, forward, left):
    """Optical-frame point where the floor is at robot-frame (forward, left), yaw 0."""
    p = math.radians(cfg.cam_pitch_deg)
    h = cfg.cam_height_m
    fx = forward - cfg.cam_forward_m
    # invert camera_to_robot for z = 0
    f = fx * math.cos(p) + h * math.sin(p)
    u = fx * math.sin(p) - h * math.cos(p)
    return np.array([[-left, -u, f]])


def test_floor_point_lands_on_floor():
    cfg = Config()
    pt = camera_to_robot(optical_ray_to_floor(cfg, 1.0, 0.2), cfg, 0.0)[0]
    assert pt == pytest.approx([1.0, 0.2, 0.0], abs=1e-9)


def test_pan_rotates_points_left():
    cfg = Config()
    pt = np.array([[0.0, 0.0, 1.0]])  # straight out of the lens
    ahead = camera_to_robot(pt, cfg, 0.0)[0]
    left = camera_to_robot(pt, cfg, math.pi / 2)[0]
    assert ahead[1] == pytest.approx(0)
    assert left[0] == pytest.approx(cfg.cam_forward_m)
    assert left[1] > 0.5


def test_classify():
    cfg = Config()
    pts = np.array([[1.0, 0.0, 0.0],     # floor
                    [1.0, 0.0, 0.10],    # obstacle
                    [1.0, 0.0, 1.50],    # too tall, ignored
                    [0.05, 0.0, 0.10]])  # the car itself, ignored
    floor, obst = classify(pts, cfg)
    assert len(floor) == 1 and len(obst) == 1


def test_fit_floor_recovers_mount():
    cfg = Config(cam_height_m=0.18, cam_pitch_deg=20.0)
    rng = np.random.default_rng(0)
    # Synthesize floor points for a known mount and check the fit recovers it.
    pts = np.vstack([optical_ray_to_floor(cfg, f, l)
                     for f, l in zip(rng.uniform(0.5, 2, 200), rng.uniform(-0.5, 0.5, 200))])
    h, pitch = fit_floor(pts)
    assert h == pytest.approx(0.18, abs=1e-6)
    assert pitch == pytest.approx(20.0, abs=1e-4)


def test_flame_mask_finds_orange_blob():
    cv2 = pytest.importorskip("cv2")
    cfg = Config()
    img = np.full((120, 160, 3), (90, 90, 90), np.uint8)       # grey background
    cv2.circle(img, (80, 60), 20, (0, 110, 255), -1)            # orange (BGR)
    cv2.circle(img, (10, 10), 2, (0, 110, 255), -1)             # speck: too small
    m = flame_mask(img, cfg)
    assert m[60, 80] and not m[10, 10]
    assert 900 < m.sum() < 1500
