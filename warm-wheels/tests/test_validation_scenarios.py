import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from rover.mission import MissionSimulator
from validation import run_scenarios as runner


class ScenarioRunnerTests(unittest.TestCase):
    def test_all_scenarios_emit_json_and_csv_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            report = runner.run_scenarios(directory)
            saved = json.loads((Path(directory) / "results.json").read_text(encoding="utf-8"))
            with (Path(directory) / "results.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(report, saved)
            self.assertEqual(report["schema_version"], "1.0")
            self.assertEqual(report["summary"], {"total": 8, "passed": 8, "failed": 0, "all_passed": True})
            self.assertEqual(len(rows), 8)
            self.assertEqual(tuple(rows[0]), runner.CSV_FIELDS)
            self.assertEqual({row["scenario_id"] for row in rows}, set(runner.SCENARIOS))
            for item in report["results"]:
                self.assertTrue(item["checks"])
                self.assertTrue(all(check["passed"] for check in item["checks"]))
                for check in item["checks"]:
                    self.assertEqual(set(check), {"id", "passed", "actual", "expected"})
            self.assertTrue(all(len(digest) == 64 for digest in report["source_sha256"].values()))

    def test_deterministic_evidence_matches_frozen_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = runner.run_scenarios(root / "baseline", seed=17)
            second = runner.run_scenarios(root / "repeat", seed=17,
                                          baseline=root / "baseline" / "results.json")
            self.assertEqual(first["results"], second["results"])
            self.assertTrue(second["baseline_comparison"]["passed"])
            self.assertTrue(second["summary"]["all_passed"])

    def test_failed_scenario_returns_nonzero_and_preserves_evidence(self):
        def failing():
            return {"checks": [{"id": "injected_failure", "passed": False,
                                "actual": "failed", "expected": "pass"}], "metrics": {}}
        with tempfile.TemporaryDirectory() as directory, patch.dict(runner.SCENARIOS, {"test_failure": ("Runner failure-path test", failing)}, clear=True):
            with contextlib.redirect_stdout(io.StringIO()):
                code = runner.main(["--output", directory])
            report = json.loads((Path(directory) / "results.json").read_text(encoding="utf-8"))
            self.assertEqual(code, 1)
            self.assertEqual(report["summary"]["failed"], 1)
            self.assertFalse(report["summary"]["all_passed"])
            self.assertIn("injected_failure", (Path(directory) / "results.csv").read_text(encoding="utf-8"))

    def test_scenario_exception_becomes_a_failed_check(self):
        def explode():
            raise RuntimeError("synthetic test exception")
        with tempfile.TemporaryDirectory() as directory, patch.dict(runner.SCENARIOS, {"test_exception": ("Runner exception-path test", explode)}, clear=True):
            report = runner.run_scenarios(directory)
            self.assertFalse(report["summary"]["all_passed"])
            self.assertIn("synthetic test exception", report["results"][0]["checks"][0]["actual"])

    def test_changed_baseline_fails_even_when_scenarios_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = runner.run_scenarios(root / "baseline", scenario_names=["dropped_data"])
            baseline["source_sha256"]["rover/mission.py"] = "0" * 64
            frozen = root / "baseline" / "results.json"
            frozen.write_text(json.dumps(baseline), encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                code = runner.main(["--output", str(root / "repeat"), "--scenario", "dropped_data", "--baseline", str(frozen)])
            report = json.loads((root / "repeat" / "results.json").read_text(encoding="utf-8"))
            self.assertEqual(code, 1)
            self.assertEqual(report["summary"]["passed"], 1)
            self.assertEqual(report["summary"]["failed"], 0)
            self.assertFalse(report["baseline_comparison"]["source_hashes_match"])
            self.assertFalse(report["summary"]["all_passed"])

    def test_unknown_selection_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                runner.run_scenarios(directory, scenario_names=["nonexistent"])


class SyntheticFaultInterfaceTests(unittest.TestCase):
    def test_obstacle_injection_rejects_robot_overlap_and_invalid_coordinates(self):
        sim = MissionSimulator()
        with self.assertRaises(ValueError):
            sim.set_obstacle(20, 5)
        sim.start()
        for col, row in ((-1, 0), (30, 10), sim.position):
            with self.subTest(col=col, row=row), self.assertRaises(ValueError):
                sim.set_obstacle(col, row)
        with self.assertRaises(ValueError):
            sim.set_obstacle(20.0, 5)

    def test_hidden_obstacle_is_not_disclosed_until_observed(self):
        sim = MissionSimulator()
        initial = sim.start()
        changed = sim.set_obstacle(20, 5)
        index = 5 * changed["grid"]["width"] + 20
        self.assertEqual(initial["grid"]["cells"][index], -1)
        self.assertEqual(changed["grid"]["cells"][index], -1)
        observed = sim.set_obstacle(20, 5, observed=True)
        self.assertEqual(observed["grid"]["cells"][index], 1)


if __name__ == "__main__":
    unittest.main()
