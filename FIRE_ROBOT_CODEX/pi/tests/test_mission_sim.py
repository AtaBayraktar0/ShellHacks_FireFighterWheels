"""End-to-end missions in the simulator: no collisions, flame avoided, back at start."""

import math

import pytest

from rescue.config import Config
from rescue.geometry import dist
from rescue.mission import Mission
from rescue.sim import Box, SimBase, SimCamera, SimWorld


def make(seed, **cfg_kw):
    """Demo world plus a quiet mission, with the given seed and config overrides."""
    cfg = Config(**cfg_kw)
    world = SimWorld.demo(cfg, seed=seed)
    mission = Mission(cfg, SimBase(world), SimCamera(world), log=lambda *_: None)
    return cfg, world, mission


# Different seeds give different motion and sensor noise.
@pytest.mark.parametrize("seed", range(5))
def test_round_trip_avoids_obstacles_and_fire(seed):
    cfg, world, m = make(seed)
    m.run()
    assert world.collisions == 0
    assert dist(world.pose.xy, (0, 0)) < 0.15          # true position back at start
    # the true trail never came near the flame
    fx, fy = world.flame
    closest = min(math.hypot(x - fx, y - fy) for x, y in world.trail)
    assert closest > cfg.robot_radius_m


def test_flame_is_mapped():
    _, world, m = make(0)
    m.run()
    fx, fy = world.flame
    c = m.grid.cell((fx, fy))
    near = m.grid.state[c[1] - 2:c[1] + 3, c[0] - 2:c[0] + 3]
    assert (near == 3).any()


def test_return_trip_replans_around_moved_obstacle():
    cfg, world, m = make(1)
    m.scan_and_plan()
    m.outbound()
    # someone drops a box on the route home while the car is at the goal
    mid = m.driven_out[len(m.driven_out) // 2]
    world.add_box(Box(mid[0] - 0.06, mid[1] - 0.06, mid[0] + 0.06, mid[1] + 0.06))
    replans_before = m.replans
    m.return_trip()
    assert m.replans > replans_before
    assert world.collisions == 0
    assert dist(world.pose.xy, (0, 0)) < 0.15
