import unittest
from usv_colav.scenarios import build_scenario, build_random_scenario
from usv_colav.simulator import SimConfig, USVSimulator
from usv_colav.models import StaticObstacle
from usv_colav.safety_filter import shield_action
from usv_colav.colregs import own_give_way

class SafetyLayerTests(unittest.TestCase):
    def test_yaw_rate_is_bounded_and_has_inertia(self):
        own, targets, note = build_scenario('head_on', num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(dt_s=1.0, max_steps=10), note)
        initial = sim.own.course_deg
        sim.step((45.0, 1.0))
        self.assertLessEqual(abs(sim.actual_yaw_rate_deg_s), sim.config.max_turn_rate_deg_s + 1e-8)
        self.assertLess(abs(sim.own.course_deg - initial), 45.0)

    def test_shield_returns_bounded_action(self):
        own, targets, note = build_random_scenario(3, seed=303)
        sim = USVSimulator(own, targets, SimConfig(max_steps=10), note)
        d, r, overridden, note = shield_action(sim, (999.0, 0.1))
        self.assertGreaterEqual(d, -sim.config.max_course_offset_deg)
        self.assertLessEqual(d, sim.config.max_course_offset_deg)
        self.assertGreaterEqual(r, 0.60)
        self.assertLessEqual(r, 1.0)
        self.assertTrue(isinstance(overridden, bool))
        self.assertTrue(note)


    def test_shield_requests_degraded_stop_when_sensors_fail(self):
        own, targets, note = build_scenario('head_on', num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(max_steps=10, failure_mode='sensor_dropout', failure_at_step=0), note)
        d, r, overridden, note = shield_action(sim, (0.0, 1.0))
        self.assertTrue(overridden)
        self.assertEqual(r, 0.0)
        self.assertIn("人工接管", note)


    def test_shield_requests_stop_when_all_target_detections_drop(self):
        own, targets, note = build_scenario('head_on', num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(max_steps=10, sensor_dropout_probability=1.0), note)
        d, r, overridden, note = shield_action(sim, (0.0, 1.0))
        self.assertTrue(overridden)
        self.assertEqual(r, 0.0)
        self.assertIn("无目标回波", note)

    def test_shield_looks_far_enough_ahead_for_static_obstacle(self):
        own, targets, note = build_scenario('head_on', num_targets=1)
        sim = USVSimulator(own, targets, SimConfig(max_steps=10, random_seed=3), note,
                           obstacles=[StaticObstacle('rock', 0.0, 1750.0, 100.0)])
        d, r, overridden, note = shield_action(sim, (0.0, 1.0))
        self.assertTrue(overridden)
        self.assertIn("替换", note)

    def test_give_way_classification_excludes_generic_astern(self):
        self.assertTrue(own_give_way('head_on'))
        self.assertTrue(own_give_way('crossing_target_starboard'))
        self.assertFalse(own_give_way('crossing_target_port'))
        self.assertFalse(own_give_way('own_being_overtaken'))

if __name__ == '__main__':
    unittest.main()
