# ======================================================================
# CREATED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# [CLAUDE EDIT] New file: checks the Isaac camera mount without Isaac installed.
# ======================================================================

"""The Isaac camera must sit exactly where camera_to_robot() thinks the D435 is,
or rendered depth would land in the wrong map cells."""

import math

import numpy as np
import pytest

from rescue.config import Config
from rescue.isaac import HEIGHT, WIDTH, camera_mount_matrix, focal_px, pan_matrix, pose_matrix
from rescue.perception import camera_to_robot


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.0])
@pytest.mark.parametrize("pitch", [0.0, 15.0, 30.0])
def test_isaac_mount_matches_camera_to_robot(yaw, pitch):
    cfg = Config(cam_pitch_deg=pitch)
    rng = np.random.default_rng(0)
    # Points in the USD camera frame (x right, y up, looking down -z).
    usd = rng.uniform(-1, 1, (20, 3))
    usd[:, 2] = -rng.uniform(0.3, 3, 20)
    homo = np.column_stack([usd, np.ones(20)])
    robot = (homo @ camera_mount_matrix(cfg) @ pan_matrix(cfg, yaw))[:, :3]
    # Same points in the RealSense optical frame (x right, y down, z forward).
    optical = np.column_stack([usd[:, 0], -usd[:, 1], -usd[:, 2]])
    assert np.allclose(robot, camera_to_robot(optical, cfg, yaw))


def test_pose_matrix_places_car():
    m = pose_matrix(1.0, 2.0, math.pi / 2)
    assert np.allclose(np.array([1, 0, 0, 1]) @ m, [1.0, 3.0, 0, 1])


def test_focal_matches_field_of_view():
    cfg = Config()
    f = focal_px(cfg)
    assert math.degrees(2 * math.atan((WIDTH / 2) / f)) == pytest.approx(cfg.hfov_deg)
    # 848x480 gives roughly the D435's 58 degree vertical field of view
    assert math.degrees(2 * math.atan((HEIGHT / 2) / f)) == pytest.approx(56.5, abs=1)


class FakeScene:
    """Stands in for IsaacScene: renders an empty floor analytically."""

    def __init__(self, world):
        self.world, self.cfg, self.render_frames, self.yaw, self.shown = world, world.cfg, 1, 0.0, 0

    def set_pan(self, yaw):
        self.yaw = yaw

    def render(self, n):
        pass

    def show_true_pose(self):
        self.shown += 1

    def images(self):
        f = focal_px(self.cfg)
        v, u = np.mgrid[0:HEIGHT, 0:WIDTH]
        rays = np.column_stack([((u - WIDTH / 2) / f).ravel(), ((v - HEIGHT / 2) / f).ravel(),
                                np.ones(u.size)])
        lens = camera_to_robot(np.zeros((1, 3)), self.cfg, self.yaw)[0]
        dz = camera_to_robot(rays, self.cfg, self.yaw)[:, 2] - lens[2]
        with np.errstate(divide="ignore"):
            depth = np.where(dz < 0, -lens[2] / dz, 0.0)  # where each ray meets z = 0
        return depth.reshape(HEIGHT, WIDTH), np.full((HEIGHT, WIDTH, 3), 128, np.uint8)


def test_isaac_classes_run_a_mission_on_an_empty_floor():
    pytest.importorskip("cv2")
    from rescue.isaac import IsaacBase, IsaacCamera
    from rescue.mission import Mission
    from rescue.sim import SimWorld

    cfg = Config()
    world = SimWorld(cfg)
    scene = FakeScene(world)
    camera = IsaacCamera(scene)
    obs = camera.observe(0.0)
    assert len(obs.floor) > 100 and len(obs.obstacles) == 0 and len(obs.flame) == 0
    base = IsaacBase(world, scene)
    Mission(cfg, base, camera, log=lambda *_: None).run()
    assert scene.shown > 0 and world.collisions == 0
    assert math.hypot(world.pose.x, world.pose.y) < 0.15  # back home
