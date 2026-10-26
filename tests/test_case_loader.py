from pathlib import Path
import unittest

from usv_colav.case_loader import list_encounter_cases, load_encounter_csv
from usv_colav.simulator import SimConfig, USVSimulator
from usv_colav.scenarios import build_scenario

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class UserScenarioCaseTests(unittest.TestCase):
    def test_all_user_csv_cases_are_readable_and_have_expected_target_count(self):
        cases = list_encounter_cases(PROJECT_ROOT / "test_cases", "all")
        self.assertEqual(len(cases), 268)
        expected = {"1V1": 1, "1V2": 2, "1V3": 3}
        counts = {group: 0 for group in expected}
        for group, path in cases:
            own, targets, metadata = load_encounter_csv(path)
            self.assertEqual(len(targets), expected[group], f"{group}/{path.name}")
            self.assertTrue(metadata["source_rows"])
            self.assertAlmostEqual(own.east_m, 0.0)
            self.assertAlmostEqual(own.north_m, 0.0)
            counts[group] += 1
        self.assertEqual(counts, {"1V1": 138, "1V2": 99, "1V3": 31})

    def test_predictive_safe_resolution_requires_margin(self):
        own, targets, note = build_scenario("head_on", num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(max_steps=20), note)
        sim.step_count = 12
        safe_metrics = [{"distance_m": 1200.0, "cpa_m": 900.0, "sda_m": sim.sda_m}]
        self.assertTrue(sim._predicted_safe_resolution(safe_metrics))
        unsafe_metrics = [{"distance_m": 1200.0, "cpa_m": 400.0, "sda_m": sim.sda_m}]
        self.assertFalse(sim._predicted_safe_resolution(unsafe_metrics))

    def test_predictive_safe_resolution_not_allowed_after_sda_violation(self):
        own, targets, note = build_scenario("head_on", num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(max_steps=20), note)
        sim.step_count = 12
        sim.sda_violations.add("TV1")
        safe_metrics = [{"distance_m": 1200.0, "cpa_m": 900.0, "sda_m": sim.sda_m}]
        self.assertFalse(sim._predicted_safe_resolution(safe_metrics))


if __name__ == "__main__":
    unittest.main()
