import json
import math
import unittest

import numpy as np

from rover.camera import DemoCamera, Frame, RealSenseCamera
from rover.perception import attach_depth, flame_candidates
from rover.spatial import analyze, demo_planning_grid, plan_path


def frame_from_depth(depth):
    height, width = depth.shape
    return Frame(np.zeros((height, width, 3), dtype=np.uint8), depth,
                 width * .90, width * .90, width / 2, height / 2, 1.0)


def grid_from_cells(cells, start):
    return {"width": cells.shape[1], "height": cells.shape[0],
            "origin_col": start[0], "origin_row": start[1],
            "resolution_m": 0.1, "cells": cells.reshape(-1).tolist(),
            "frame": "camera-local"}


class DepthAnalysisTests(unittest.TestCase):
    def test_clear_depth_wall_has_finite_range(self):
        result = analyze(frame_from_depth(np.full((120, 160), 2.0, np.float32)))
        self.assertEqual(result["clearance_m"], 2.0)
        self.assertEqual(result["depth_valid_fraction"], 1.0)
        self.assertLessEqual(len(result["points"]), 1200)
        self.assertEqual(len(result["grid"]["cells"]), 3600)
        json.dumps(result, allow_nan=False)

    def test_single_front_hole_is_unknown_not_clear(self):
        for value in (0.0, float("nan"), float("inf"), -1.0, 99.0):
            with self.subTest(value=value):
                depth = np.full((120, 160), 2.0, np.float32)
                depth[60, 80] = value
                result = analyze(frame_from_depth(depth))
                self.assertIsNone(result["clearance_m"])
                json.dumps(result, allow_nan=False)

    def test_near_obstacle_overrides_far_wall(self):
        depth = np.full((120, 160), 3.0, np.float32)
        depth[45:78, 77:84] = 0.28
        result = analyze(frame_from_depth(depth))
        self.assertAlmostEqual(result["clearance_m"], 0.28, places=2)
        grid = np.array(result["grid"]["cells"]).reshape(60, 60)
        self.assertTrue(np.any(grid[55:59, 28:33] == 1))

    def test_invalid_frame_does_not_invent_observed_floor(self):
        result = analyze(frame_from_depth(np.zeros((120, 160), np.float32)))
        self.assertIsNone(result["clearance_m"])
        self.assertEqual(result["depth_valid_fraction"], 0.0)
        self.assertEqual(result["points"], [])
        self.assertEqual(result["grid"]["cells"].count(0), 1)
        self.assertEqual(plan_path(result["grid"], [30, 40]), [])

    def test_floor_is_not_reported_as_near_wall(self):
        height, width, fy = 120, 160, 144
        depth = np.full((height, width), 4.0, np.float32)
        for row in range(height // 2 + 1, height):
            floor = 0.22 * fy / (row - height / 2)
            depth[row, :] = min(4.0, floor)
        result = analyze(frame_from_depth(depth))
        self.assertEqual(result["clearance_m"], 4.0)

    def test_misaligned_shapes_rejected(self):
        frame = frame_from_depth(np.ones((120, 160), np.float32))
        frame.rgb = frame.rgb[:100]
        with self.assertRaises(ValueError):
            analyze(frame)


class PlanningTests(unittest.TestCase):
    def test_synthetic_fixture_enables_demo_route_without_changing_sensor_grid(self):
        camera = DemoCamera().start()
        local_grid = analyze(camera.read())["grid"]
        before = json.dumps(local_grid)
        demo_grid = demo_planning_grid(local_grid)
        self.assertEqual(json.dumps(local_grid), before)
        self.assertEqual(demo_grid["origin_row"], 53)
        self.assertEqual(demo_grid["frame"], "synthetic-demo")
        self.assertIn("not sensed", demo_grid["note"])
        path = plan_path(demo_grid, [30, 30])
        self.assertTrue(path)
        self.assertEqual(path[0], [30, 53])
        self.assertEqual(path[-1], [30, 30])
        self.assertEqual(plan_path(local_grid, [30, 36]), [])
        camera.close()

    def test_synthetic_fixture_preserves_obstacles_and_distant_unknown(self):
        cells = np.full((60, 60), -1, dtype=np.int8)
        cells[59, 30] = 0
        cells[57, 30] = 1
        result = demo_planning_grid(grid_from_cells(cells, (30, 59)))
        output = np.array(result["cells"]).reshape(60, 60)
        self.assertEqual(output[51, 30], 1)
        self.assertEqual(output[53, 33], 0)
        self.assertEqual(output[30, 30], -1)
        self.assertEqual(output[59, 30], -1)
        self.assertEqual(output[59, 45], -1)
        with self.assertRaises(ValueError):
            demo_planning_grid(result)

    def test_route_around_obstacle_with_footprint_clearance(self):
        cells = np.zeros((30, 30), dtype=np.int8)
        cells[9:21, 14:16] = 1
        path = plan_path(grid_from_cells(cells, (5, 15)), [24, 15])
        self.assertTrue(path)
        self.assertEqual(path[0], [5, 15])
        self.assertEqual(path[-1], [24, 15])
        for col, row in path:
            self.assertEqual(cells[row, col], 0)
            for obstacle_row, obstacle_col in np.argwhere(cells == 1):
                self.assertGreater(math.hypot(col - obstacle_col, row - obstacle_row), 2)

    def test_unknown_barrier_cannot_be_routed_through(self):
        cells = np.zeros((20, 20), dtype=np.int8)
        cells[:, 10] = -1
        self.assertEqual(plan_path(grid_from_cells(cells, (4, 10)), [16, 10]), [])

    def test_footprint_cannot_squeeze_through_narrow_gap(self):
        cells = np.zeros((20, 20), dtype=np.int8)
        cells[9, :] = 1
        cells[9, 9:11] = 0
        self.assertEqual(plan_path(grid_from_cells(cells, (10, 16)), [10, 3]), [])
        self.assertTrue(plan_path(grid_from_cells(cells, (10, 16)), [10, 3], robot_radius_m=0))

    def test_unknown_is_also_inflated(self):
        cells = np.zeros((20, 20), dtype=np.int8)
        cells[9, :] = -1
        cells[9, 9:11] = 0
        self.assertEqual(plan_path(grid_from_cells(cells, (10, 16)), [10, 3]), [])

    def test_no_diagonal_corner_cutting(self):
        cells = np.array([[0, 1], [1, 0]], dtype=np.int8)
        self.assertEqual(plan_path(grid_from_cells(cells, (0, 0)), [1, 1], robot_radius_m=0), [])

    def test_goal_outside_map_rejected(self):
        grid = grid_from_cells(np.zeros((10, 10), dtype=np.int8), (3, 3))
        self.assertEqual(plan_path(grid, [-1, 3]), [])
        self.assertEqual(plan_path(grid, [4.5, 3]), [])

    def test_footprint_cannot_extend_into_unobserved_map_exterior(self):
        cells = np.zeros((20, 20), dtype=np.int8)
        self.assertEqual(plan_path(grid_from_cells(cells, (10, 19)), [10, 4]), [])


class PerceptionTests(unittest.TestCase):
    def test_demo_is_explicit_and_heuristic_is_unverified(self):
        camera = DemoCamera().start()
        frame = camera.read()
        candidates = flame_candidates(frame)
        self.assertTrue(candidates)
        for detection in candidates:
            self.assertEqual(detection["kind"], "fire_candidate")
            self.assertIn("unverified", detection["source"])
            self.assertNotIn("age", detection)
        camera.close()
        with self.assertRaises(RuntimeError):
            camera.read()

    def test_depth_attachment_clips_boxes_rejects_nan_and_preserves_inputs(self):
        frame = frame_from_depth(np.full((120, 160), 2.5, np.float32))
        original = {"kind": "person", "confidence": 0.8, "box": [-5, -2, 35, 60], "source": "test"}
        result = attach_depth([original, {"box": [0, 0, float("nan"), 10]}], frame)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["distance_m"], 2.5)
        self.assertEqual(result[0]["box"], [0, 0, 35, 60])
        self.assertEqual(original["box"][0], -5)
        frame.depth[:] = 0
        self.assertIsNone(attach_depth([original], frame)[0]["distance_m"])

    def test_realsense_read_never_falls_back_to_demo(self):
        with self.assertRaises(RuntimeError):
            RealSenseCamera().read()


if __name__ == "__main__":
    unittest.main()
