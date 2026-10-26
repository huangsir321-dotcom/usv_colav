import unittest
from usv_colav.scenarios import build_scenario, build_random_scenario
from usv_colav.simulator import SimConfig, USVSimulator
from usv_colav.rule_policy import choose_rule_action

class SimulatorTests(unittest.TestCase):
    def test_1v1_1v2_1v3(self):
        for n in (1, 2, 3):
            own, targets, note = build_random_scenario(n, seed=123+n)
            sim = USVSimulator(own, targets, SimConfig(max_steps=10), note)
            self.assertEqual(len(sim.targets), n)
            self.assertEqual(len(sim.observation_values()), 37)
            action = choose_rule_action(sim)
            _, _, _ = sim.step(action)
            self.assertEqual(sim.step_count, 1)

    def test_ccs_annex_d6_preset_has_three_targets(self):
        own, targets, note = build_scenario('ccs_d6_3', num_targets=3)
        self.assertEqual(len(targets), 3)
        self.assertAlmostEqual(targets[0].speed_kn, 18.0)
        self.assertAlmostEqual(targets[1].speed_kn, 8.0)
        self.assertAlmostEqual(targets[2].course_deg, 230.0)


    def test_full_horizon_mode_checks_final_safe_state_at_time_limit(self):
        from usv_colav.models import Vessel
        own = Vessel("OV", 0.0, 0.0, 10.0, 0.0, length_m=8.0, is_own=True,
                     planned_course_deg=0.0, planned_speed_kn=10.0)
        target = Vessel("TV1", 2000.0, 0.0, 10.0, 0.0, length_m=8.0)
        sim = USVSimulator(own, [target], SimConfig(max_steps=20, safe_resolution_early_stop=False,
                                                     sensor_position_sigma_m=0.0,
                                                     sensor_course_sigma_deg=0.0,
                                                     sensor_speed_sigma_kn=0.0))
        for _ in range(12):
            _, done, _ = sim.step((0.0, 1.0))
            self.assertFalse(done)
        for _ in range(8):
            _, done, _ = sim.step((0.0, 1.0))
        self.assertTrue(done)
        self.assertIn("达到仿真时限且预测航迹保持安全；安全通过", sim.terminal_reason)
        self.assertFalse(sim.collision)
        self.assertFalse(sim.sda_violations)

    def test_target_heading_is_fixed(self):
        own, targets, note = build_scenario('head_on', num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(max_steps=3), note)
        old_course = targets[0].course_deg
        for _ in range(3):
            sim.step((25.0, 1.0))
        self.assertAlmostEqual(targets[0].course_deg, old_course)

if __name__ == '__main__':
    unittest.main()
