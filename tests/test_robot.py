import json
import math
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from firebot.planning import Grid, astar
from firebot.vision import Pose, Frame, world_points, update_map, flame_mask
from firebot.mission import Mission, read_pose, steering
from firebot.simulation import build_demo_world, scan_demo_sensor, simulate
from firebot.motors import Motors
from firebot.cli import load_config

ROOT = Path(__file__).resolve().parents[1]


class PlanningTests(unittest.TestCase):
    """Check the map, pathfinder, and verified return trip."""

    def test_astar_detour_and_no_corner_cut(self):
        safe = np.ones((7, 7), dtype=bool)
        safe[0:6, 3] = False
        route = astar(safe, (1, 1), (5, 1))
        self.assertEqual(route[0], (1, 1))
        self.assertEqual(route[-1], (5, 1))
        for a, b in zip(route, route[1:]):
            self.assertEqual(abs(a[0]-b[0])+abs(a[1]-b[1]), 1)
            self.assertTrue(safe[b[1], b[0]])
        self.assertEqual(astar(np.eye(2, dtype=bool), (0, 0), (1, 1)), [])

    def test_demo_map_is_filled_by_sensor_scan(self):
        grid = Grid()
        truth, _ = build_demo_world(grid)
        self.assertFalse(grid.blocked.any())
        scan_demo_sensor(grid, truth, 1)
        np.testing.assert_array_equal(grid.blocked, truth)

    def test_unknown_stale_boundary_and_inflation(self):
        grid = Grid(9, 9, 0.05)
        self.assertFalse(grid.safe(0).any())
        grid.seen[:] = 0
        grid.blocked[4, 4] = True
        safe = grid.safe(1, 2, .05)
        self.assertFalse(safe[4, 5])
        self.assertFalse(safe[0, 0])
        self.assertTrue(safe[2, 2])
        self.assertFalse(grid.safe(3, 2, .05).any())
        self.assertEqual(grid.cell(-.01, 0), (-1, 0))

    def test_obstacles_are_latched(self):
        grid = Grid(5, 5)
        grid.observe([], [(2, 2)], 0)
        grid.observe([(2, 2)], [], 1)
        self.assertTrue(grid.blocked[2, 2])

    def test_return_requires_new_safe_observations(self):
        mission = Mission((1, 1), (2, 1))
        safe = np.ones((4, 4), dtype=bool)
        self.assertEqual(mission.next_target((1, 1), safe), (2, 1))
        safe[1, 1] = False
        with self.assertRaises(RuntimeError):
            mission.next_target((2, 1), safe)

    def test_demo_round_trip_and_blocked_return(self):
        cfg = load_config(ROOT / "config.json")
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            self.assertEqual(simulate(Grid(), cfg, output), "complete")
            self.assertTrue((output / "simulation.html").exists())
            self.assertIn("Return waypoint", simulate(Grid(), cfg, output, True))
            animation = (output / "simulation.html").read_text(encoding="utf-8")
            self.assertIn("Stopped: ", animation)
            self.assertIn("Return waypoint is blocked", animation)
            self.assertIn("NEW BLOCK", animation)


class SensorTests(unittest.TestCase):
    """Check camera math, flame colors, position, and motor safety."""

    def test_camera_to_world_axes(self):
        xyz = np.array([[[0, .2, 1.0]]])
        p = world_points(xyz, Pose(1, 2, math.pi/2), .2, 0)[0, 0]
        np.testing.assert_allclose(p, [1, 3, 0], atol=1e-8)

    def test_floor_obstacle_flame_and_missing_depth(self):
        cfg = load_config(ROOT / "config.json")
        cfg.update(camera_height_m=.2, camera_pitch_deg=0)
        xyz = np.tile([0., .2, 1.], (20, 20, 1))
        frame = Frame(xyz, np.zeros((20, 20, 3), dtype=np.uint8), 10)
        grid = Grid()
        pose = Pose(.5, .5, 0)
        update_map(grid, frame, pose, cfg)
        x, y = grid.cell(1.5, .5)
        self.assertEqual(grid.seen[y, x], 10)
        xyz[:10, :, 1] = 0
        update_map(grid, frame, pose, cfg)
        self.assertTrue(grid.blocked[y, x])
        xyz[0, 0] = 0
        mask = np.zeros((20, 20), dtype=bool)
        mask[0, 0] = True
        with self.assertRaises(RuntimeError):
            update_map(grid, frame, pose, cfg, mask)

    def test_color_filter(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV not installed")
        bgr = np.zeros((50, 50, 3), dtype=np.uint8)
        bgr[10:30, 10:30] = [0, 100, 255]
        bgr[0, 0] = [0, 0, 255]
        mask = flame_mask(bgr)
        self.assertTrue(mask[20, 20])
        self.assertFalse(mask[0, 0])

    def test_pose_freshness(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp) / "pose.json"
            data = dict(x=1, y=2, yaw=0, timestamp=time.time())
            file.write_text(json.dumps(data))
            self.assertEqual(read_pose(file, 1), Pose(1, 2, 0))
            data["timestamp"] -= 10
            file.write_text(json.dumps(data))
            with self.assertRaises(RuntimeError):
                read_pose(file, 1)

    def test_steering(self):
        self.assertEqual(steering(Pose(0, 0, 0), (1, 0)), "FWD")
        self.assertEqual(steering(Pose(0, 0, 0), (0, 1)), "LEFT")
        self.assertEqual(steering(Pose(0, 0, 0), (0, -1)), "RIGHT")

    def test_motor_pulse_stops_after_failure(self):
        motors = Motors("fake")
        commands = []
        def send(text):
            commands.append(text)
            if text != "STOP":
                raise RuntimeError("Link failed")
        motors.send = send
        with self.assertRaises(RuntimeError):
            motors.pulse("FWD")
        self.assertEqual(commands, ["FWD 70 100", "STOP"])


if __name__ == "__main__":
    unittest.main()
    def test_demo_room_route_stays_safe(self):
        grid = Grid()
        build_demo_room(grid)
        grid.seen[:] = 0
        safe = grid.safe(0, 2, .20)
        route = astar(safe, (10, 10), (48, 48))
        self.assertTrue(route)
        self.assertTrue(all(safe[y, x] for x, y in route))
