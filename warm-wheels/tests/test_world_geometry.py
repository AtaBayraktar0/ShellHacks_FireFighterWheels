"""Physical geometry and observation-only depth meshing invariants."""
import hashlib
import math

import numpy as np
import pytest

from rover.world_geometry import (FAMILIES, SurfaceObject, SyntheticDepthScan,
                                  SyntheticWorld, _component, build_world)


def test_seed_replays_geometry_and_home_exactly():
    first, second = build_world(7, 5, 142857), build_world(7, 5, 142857)
    np.testing.assert_array_equal(first.cells, second.cells)
    assert first.objects == second.objects
    assert first.home == second.home
    assert first.family == second.family


def test_seed_changes_topology_and_entry_side_not_only_door_offsets():
    worlds = [build_world(8, 8, seed) for seed in range(40)]
    assert {world.family for world in worlds} == set(FAMILIES)
    assert len({hashlib.sha256(world.cells.tobytes()).digest() for world in worlds}) >= 38
    assert len({len([obj for obj in world.objects if obj.role == "wall"]) for world in worlds}) >= 6
    sides = set()
    for world in worlds:
        x, z = ((world.home[0] + .5) * world.resolution_m, (world.home[1] + .5) * world.resolution_m)
        sides.add(int(np.argmin((z, world.width_m - x, world.height_m - z, x))))
    assert sides == {0, 1, 2, 3}
    # Angled partitions and curved furniture must be real collision geometry.
    assert any(any(obj.role == "wall" and abs(math.sin(2 * obj.angle)) > .2
                   for obj in world.objects) for world in worlds)
    assert any(any(obj.kind == "cylinder" for obj in world.objects) for world in worlds)
    # A fixed cross cannot occur in every layout: open plan includes no
    # room-spanning middle wall in either axis.
    assert any(world.family == "open_plan" and all(
        obj.role != "wall" or obj in world.objects[:4] or obj.half_x * 2 < 4.0
        for obj in world.objects) for world in worlds)


@pytest.mark.parametrize("width,height", [(1, 1), (1, 50), (50, 1), (50, 50), (2.3, 9.7), (6, 6)])
@pytest.mark.parametrize("seed", [0, 1, 7, 21])
def test_every_free_center_and_cardinal_path_has_footprint_clearance(width, height, seed):
    world = build_world(width, height, seed)
    assert max(world.cells.shape) <= 80
    assert world.cells.dtype == np.int8
    assert not world.cells[world.home[1], world.home[0]]
    free = np.argwhere(world.cells == 0)
    assert len(_component(world.cells, world.home)) == len(free) > 0
    x, z = (free[:, 1] + .5) * world.resolution_m, (free[:, 0] + .5) * world.resolution_m
    assert np.min(x) >= .20 and np.max(x) <= width - .20
    assert np.min(z) >= .20 and np.max(z) <= height - .20
    for obj in world.objects:
        assert np.all(obj.distance(x, z) >= .20 - 1e-9)
    # Include edge midpoints so coarse 50m rasters cannot step through a thin wall.
    for dr, dc in ((0, 1), (1, 0)):
        mask = (free[:, 0] + dr < world.cells.shape[0]) & (free[:, 1] + dc < world.cells.shape[1])
        nearby = free[mask]
        nearby = nearby[world.cells[nearby[:, 0] + dr, nearby[:, 1] + dc] == 0]
        x = (nearby[:, 1] + .5 + dc / 2) * world.resolution_m
        z = (nearby[:, 0] + .5 + dr / 2) * world.resolution_m
        for obj in world.objects:
            assert np.all(obj.distance(x, z) >= .20 - 1e-9)


def _occlusion_world():
    # A complete x=3 partition is intentionally disconnected to test sensing,
    # independent of the connected-world generator.
    objects = [SurfaceObject("box", 3.05, 3, .05, 3, 2.45, role="wall"),
               SurfaceObject("cylinder", 4.5, 3, .4, .4, 1.2)]
    cells = np.zeros((30, 30), dtype=np.int8)
    return SyntheticWorld(cells, cells.copy(), (5, 15), .2, 6., 6., "occlusion_test", objects, 0)


def test_rays_stop_at_nearest_real_surface_and_hide_behind_wall():
    world = _occlusion_world()
    scanner = SyntheticDepthScan(world)
    distance, sid = scanner._cast(np.array([1., .22, 3.]), np.array([[1., 0., 0.]]))
    assert distance[0] == pytest.approx(2.)
    assert sid[0] > 0
    scanner.update(world.home)
    mesh = scanner.snapshot()
    points = np.asarray(mesh["vertices"])
    assert len(points) > 100 and mesh["triangle_count"] > 100
    assert points[:, 0].max() <= 3.001
    assert np.any(points[:, 1] == 0), "floor must be acquired as depth surfaces"
    assert np.any(points[:, 1] > 1.5), "scan must include full room height walls"
    assert mesh["observed_surface_count"] < 5


def test_cylinder_depth_hits_curved_surface():
    cells = np.zeros((20, 20), dtype=np.int8)
    world = SyntheticWorld(cells, cells, (3, 10), .2, 4, 4, "curve_test",
                           [SurfaceObject("cylinder", 2., 2., .5, .5, 1.0)], 0)
    scan = SyntheticDepthScan(world)
    origins = np.array([.5, .22, 2.])
    distance, ids = scan._cast(origins, np.array([[1., 0, 0.], [math.cos(.15), 0, math.sin(.15)]]))
    assert ids[0] == ids[1] > 0
    assert distance[0] == pytest.approx(1.)
    assert distance[1] > distance[0]


@pytest.mark.parametrize("width,height,seed", [(6, 6, 0), (8, 8, 3), (50, 50, 17), (1, 50, 7)])
def test_raw_free_cardinal_edges_cannot_cross_thin_physical_walls(width, height, seed):
    world = build_world(width, height, seed)
    assert world.raw_cells[world.home[1], world.home[0]] == 0
    assert np.all(world.raw_cells[world.cells == 0] == 0)
    free = np.argwhere(world.raw_cells == 0)
    samples = np.linspace(0, 1, 9)
    for dr, dc in ((0, 1), (1, 0)):
        mask = (free[:, 0] + dr < world.raw_cells.shape[0]) & (free[:, 1] + dc < world.raw_cells.shape[1])
        nearby = free[mask]
        nearby = nearby[world.raw_cells[nearby[:, 0] + dr, nearby[:, 1] + dc] == 0]
        x = (nearby[:, 1, None] + .5 + dc * samples[None, :]) * world.resolution_m
        z = (nearby[:, 0, None] + .5 + dr * samples[None, :]) * world.resolution_m
        for obj in world.objects[4:]:
            assert np.all(obj.distance(x, z) > 1e-9), "raw-free neighbor transition crosses a physical obstacle"


def test_mesh_only_changes_with_acquisition_and_stays_bounded():
    world = build_world(10, 8, 184)
    scan = SyntheticDepthScan(world)
    assert scan.snapshot()["vertices"] == []
    assert scan.update(world.home)
    first = scan.snapshot()
    assert scan.update(world.home, heading_rad=1.)
    assert scan.snapshot()["revision"] > first["revision"]
    free = np.argwhere(world.cells == 0)
    for row, col in free[::max(1, len(free) // 32)]:
        scan.update((int(col), int(row)))
    mesh = scan.snapshot()
    assert mesh["vertex_count"] <= 12000
    assert mesh["triangle_count"] <= 18000
    assert len(mesh["colors"]) == mesh["vertex_count"]
    assert len(mesh["latest_points"]) <= 384
    points = np.asarray(mesh["vertices"])
    assert np.isfinite(points).all()
    assert points[:, 0].min() >= -.001 and points[:, 0].max() <= world.width_m + .001
    assert points[:, 2].min() >= -.001 and points[:, 2].max() <= world.height_m + .001
    for triangle in mesh["triangles"]:
        assert len(set(triangle)) == 3
        assert all(0 <= i < len(points) for i in triangle)
    assert "synthetic" in mesh["source"]
    assert "not D435i" in mesh["sensor_model"]
    assert mesh["horizontal_fov_deg"] == pytest.approx(87.0)
    assert mesh["vertical_fov_deg"] == pytest.approx(80.0)


def test_front_camera_does_not_acquire_rear_until_heading_changes():
    cells = np.zeros((20, 20), dtype=np.int8)
    world = SyntheticWorld(cells, cells, (5, 5), .2, 4, 4, "front_camera",
                           [], 0)
    scan = SyntheticDepthScan(world)
    assert scan.update(world.home, heading_rad=0.0)
    first = scan.snapshot()
    origin_x = (world.home[0] + .5) * world.resolution_m
    points = np.asarray(first["vertices"])
    assert len(points) and np.all(points[:, 0] >= origin_x - 1e-6)
    assert first["ray_count"] == 96 * 16
    assert scan.update(world.home, heading_rad=math.pi)
    turned = scan.snapshot()
    points = np.asarray(turned["vertices"])
    assert np.any(points[:, 0] < origin_x - .05)
    assert np.any(points[:, 0] > origin_x + .05)
    assert turned["triangle_count"] >= first["triangle_count"]


def test_acquired_mesh_persists_when_rover_moves_to_a_new_view():
    cells = np.zeros((24, 24), dtype=np.int8)
    world = SyntheticWorld(cells, cells, (5, 12), .2, 4.8, 4.8, "persistent_scan",
                           [], 0)
    scan = SyntheticDepthScan(world)
    assert scan.update(world.home, heading_rad=0.0)
    first = scan.snapshot()
    first_vertices = {tuple(vertex) for vertex in first["vertices"]}
    assert first_vertices
    assert scan.update((8, 12), heading_rad=0.0)
    second = scan.snapshot()
    assert first_vertices.intersection({tuple(vertex) for vertex in second["vertices"]})
    assert second["revision"] > first["revision"]


@pytest.mark.parametrize("width,height,seed", [(0, 6, 0), (51, 6, 0), (6, float("nan"), 0), (6, 6, True), (6, 6, -1)])
def test_invalid_world_dimensions_or_seed_rejected(width, height, seed):
    with pytest.raises(ValueError):
        build_world(width, height, seed)
