import json
import unittest

import numpy as np

from rover.mission import MissionSimulator


class MissionTests(unittest.TestCase):
    def finish(self, sim, limit=1000):
        history = []
        for _ in range(limit):
            before = sim.snapshot()
            state = sim.tick()
            history.append(state)
            if state["position"] != before["position"]:
                col, row = state["position"]
                old_col, old_row = before["position"]
                self.assertEqual(abs(col - old_col) + abs(row - old_row), 1)
                self.assertEqual(before["grid"]["cells"][row * before["grid"]["width"] + col], 0)
                self.assertEqual(sim._truth[row, col], 0)
            if state["phase"] in ("complete", "blocked"):
                return state, history
        self.fail("Mission did not terminate within bounded ticks")

    def test_empty_fixture_explores_all_rooms_and_returns_home(self):
        sim = MissionSimulator()
        initial = sim.start()
        self.assertEqual(initial["phase"], "exploring")
        self.assertLess(initial["coverage_percent"], 100)
        state, history = self.finish(sim)
        self.assertEqual(state["phase"], "complete")
        self.assertEqual(state["position"], state["home"])
        self.assertEqual(state["coverage_percent"], 100.0)
        self.assertEqual(state["unobserved_cells"], 0)
        self.assertIn("no reachable observed frontier", state["reason"])
        visited_rooms = {(item["position"][0] > 15, item["position"][1] > 15) for item in history}
        self.assertEqual(len(visited_rooms), 4)
        self.assertIn("not a D435i", state["sensor_model"])
        self.assertIn("no fire or person", state["scenario"])
        self.assertEqual(state["grid"]["frame"], "synthetic-mission")
        json.dumps(state, allow_nan=False)

    def test_abort_requests_motion_home_and_does_not_teleport(self):
        sim = MissionSimulator()
        sim.start()
        for _ in range(12):
            sim.tick()
        before = sim.snapshot()
        self.assertNotEqual(before["position"], before["home"])
        aborted = sim.abort()
        self.assertEqual(aborted["phase"], "returning")
        self.assertEqual(aborted["position"], before["position"])
        state, _ = self.finish(sim)
        self.assertEqual(state["phase"], "complete")
        self.assertEqual(state["position"], state["home"])
        self.assertIn("operator requested", state["reason"])
        self.assertGreater(state["unobserved_cells"], 0)

    def test_step_budget_returns_early(self):
        sim = MissionSimulator(max_exploration_steps=10)
        sim.start()
        state, _ = self.finish(sim)
        self.assertEqual(state["phase"], "complete")
        self.assertEqual(state["position"], state["home"])
        self.assertIn("step budget", state["reason"])
        self.assertLess(state["coverage_percent"], 100)

    def test_time_budget_returns_early(self):
        now = [0.0]
        sim = MissionSimulator(max_duration_s=5, clock=lambda: now[0])
        sim.start()
        for _ in range(6):
            sim.tick()
        now[0] = 6.0
        state, _ = self.finish(sim)
        self.assertEqual(state["phase"], "complete")
        self.assertIn("time budget", state["reason"])
        self.assertEqual(state["position"], state["home"])

    def test_blocked_home_never_claims_return(self):
        sim = MissionSimulator()
        sim.start()
        for _ in range(12):
            sim.tick()
        # Fault injection: an obstacle occupies the launch cell and is known.
        col, row = sim.home
        sim._truth[row, col] = 1
        sim._observed[row, col] = 1
        state = sim.abort()
        self.assertEqual(state["phase"], "blocked")
        self.assertNotEqual(state["position"], state["home"])
        self.assertIn("has not returned", state["reason"])
        self.assertEqual(sim.tick()["phase"], "blocked")

    def test_default_fixture_is_bounded_and_discovery_never_peeks_through_walls(self):
        sim = MissionSimulator()
        state = sim.start()
        observed = np.array(state["grid"]["cells"]).reshape(30, 30)
        self.assertEqual(observed[4, 25], -1)
        self.assertTrue(np.all(sim._truth[0, :] == 1))
        self.assertTrue(np.all(sim._truth[:, 0] == 1))
        self.assertEqual(state["steps"], 0)
        with self.assertRaises(ValueError):
            sim.start()

    def test_snapshot_is_detached_and_idle_does_not_move(self):
        sim = MissionSimulator()
        state = sim.snapshot()
        state["grid"]["cells"][0] = 123
        state["position"][0] = 0
        self.assertEqual(sim.tick()["phase"], "idle")
        self.assertEqual(sim.snapshot()["position"], [4, 25])
        self.assertEqual(sim.snapshot()["grid"]["cells"][0], -1)


if __name__ == "__main__":
    unittest.main()
