# ======================================================================
# MODIFIED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# Search for "[CLAUDE EDIT]" to find each change. Full list: CLAUDE_CHANGES.md
# ======================================================================
# Change here: regression tests for the map-edge fix (bottom of file).

"""Occupancy grid updates, obstacle inflation, A* and path cleanup."""

import math

import numpy as np
import pytest

from rescue.astar import astar, smooth, split_long
from rescue.config import Config
from rescue.geometry import Pose
from rescue.grid import FIRE, FREE, OCC, Observation, OccupancyGrid, dilate
from rescue.mission import Mission
from rescue.sim import SimBase, SimCamera, SimWorld


def test_dilate_disk():
    m = np.zeros((9, 9), bool)
    m[4, 4] = True
    d = dilate(m, 2)
    assert d[4, 6] and d[6, 4] and d[5, 5]
    assert not d[6, 6]  # corner of the square, outside the disk


def test_astar_goes_around_wall():
    b = np.zeros((10, 10), bool)
    b[0:8, 5] = True  # wall with a gap at the top
    path = astar(b, (0, 0), (9, 0))
    assert path[0] == (0, 0) and path[-1] == (9, 0)
    assert all(not b[y, x] for x, y in path)
    assert max(y for _, y in path) >= 8


def test_astar_no_corner_cutting():
    b = np.zeros((3, 3), bool)
    b[0, 1] = b[1, 0] = True
    assert astar(b, (0, 0), (1, 1)) is None


def test_astar_no_path():
    b = np.zeros((5, 5), bool)
    b[:, 2] = True
    assert astar(b, (0, 0), (4, 4)) is None


def test_smooth_and_split():
    pts = [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert smooth(pts, lambda a, b: True) == [(0, 0), (3, 0)]
    out = split_long([(0, 0), (1, 0)], 0.3)
    assert len(out) == 5 and out[-1] == (1, 0)


def test_grid_update_marks_and_clears():
    cfg = Config()
    g = OccupancyGrid(cfg)
    pose = Pose(0, 0, math.pi / 2)  # facing +y
    # obstacle 0.8 m ahead in the robot frame -> world (0, 0.8)
    obs = Observation(floor=np.array([[0.5, 0.0]]),
                      obstacles=np.array([[0.8, 0.0]] * 3))
    g.update(obs, pose)
    ix, iy = g.cell((0.0, 0.8))
    assert g.state[iy, ix] == OCC
    fx, fy = g.cell((0.0, 0.5))
    assert g.state[fy, fx] == FREE
    # later the floor there is visible and empty: the obstacle moved away
    g.update(Observation(floor=np.array([[0.8, 0.0]])), pose)
    assert g.state[iy, ix] == FREE


def test_fire_gets_wider_margin():
    cfg = Config(assumed_depth_m=0.0)
    g = OccupancyGrid(cfg)
    c_occ, c_fire = g.cell((0.5, -0.5)), g.cell((0.5, 0.5))
    g.state[c_occ[1], c_occ[0]] = OCC
    g.state[c_fire[1], c_fire[0]] = FIRE
    m = g.blocked_mask()
    probe = 0.25  # between robot+clearance (0.18) and robot+clearance+fire (0.28)
    a = g.cell((0.5 + probe, -0.5))
    b = g.cell((0.5 + probe, 0.5))
    assert not m[a[1], a[0]]
    assert m[b[1], b[0]]


def test_segment_clear_rejects_out_of_map():
    cfg = Config()
    g = OccupancyGrid(cfg)
    m = g.blocked_mask()
    assert g.segment_clear(m, (0, 0), (1, 0))
    assert not g.segment_clear(m, (0, 0), (10, 0))


@pytest.mark.parametrize("clearance, cells", [(None, 4), (0.0, 3)])
# [CLAUDE EDIT] New test: border band on all four sides, incl. zero-clearance fallback.
def test_map_border_keeps_body_inside(clearance, cells):
    g = OccupancyGrid(Config())
    mask = g.blocked_mask(clearance=clearance)
    assert mask[:cells, :].all() and mask[-cells:, :].all()
    assert mask[:, :cells].all() and mask[:, -cells:].all()
    assert not mask[cells:-cells, cells:-cells].any()
    assert not g.segment_clear(mask, (0, 0), (0, g.cfg.y_max - 0.01))


@pytest.mark.parametrize("margins, cells", [(None, 4), ((0.0, 0.1), 3)])
# [CLAUDE EDIT] New test: car parked near the edge must not reopen the border.
def test_clearing_robot_footprint_preserves_border(margins, cells):
    cfg = Config()
    world = SimWorld(cfg)
    mission = Mission(cfg, SimBase(world), SimCamera(world))
    mission.pose.y = cfg.y_max - 0.05
    mask = mission.blocked(margins)
    assert mask[-cells:, :].all()


@pytest.mark.parametrize("edge_is_wall", [False, True])
# [CLAUDE EDIT] New test: sim flags crossing each edge; edge_is_wall=False disables it.
def test_sim_detects_body_crossing_each_edge(edge_is_wall):
    cfg = Config(edge_is_wall=edge_is_wall)
    world = SimWorld(cfg)
    radius = SimBase.BODY_RADIUS
    for x, y in [(cfg.x_min + 0.01, 0), (cfg.x_max - 0.01, 0),
                 (0, cfg.y_min + 0.01), (0, cfg.y_max - 0.01)]:
        assert world.collides(x, y, radius) == edge_is_wall
    assert not world.collides(0, 0, radius)
    if not edge_is_wall:
        assert not OccupancyGrid(cfg).blocked_mask().any()
