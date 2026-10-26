import unittest

from usv_colav.models import StaticObstacle
from usv_colav.scenarios import build_scenario
from usv_colav.simulator import SimConfig, USVSimulator


class RobustnessTests(unittest.TestCase):
    def _sim(self, config=None, obstacles=None):
        own, targets, note = build_scenario("head_on", num_targets=1)
        return USVSimulator(own, targets, config or SimConfig(max_steps=30), note, obstacles=obstacles)

    def test_sensor_noise_is_reproducible_and_not_truth_state(self):
        sim = self._sim(SimConfig(random_seed=123, sensor_position_sigma_m=25.0, max_steps=30))
        self.assertNotEqual((sim.control_own().east_m, sim.control_own().north_m), sim.own.position)
        sim2 = self._sim(SimConfig(random_seed=123, sensor_position_sigma_m=25.0, max_steps=30))
        self.assertAlmostEqual(sim.control_own().east_m, sim2.control_own().east_m)

    def test_restricted_visibility_hides_distant_target(self):
        sim = self._sim(SimConfig(visibility_m=500.0, observation_range_nm=10.0, max_steps=30))
        self.assertEqual(len(sim.control_targets()), 0)
        self.assertEqual(sim.sensor_snapshot["visible_targets"], 0)
        self.assertEqual(len(sim.observation_values()), 37)

    def test_current_drift_moves_stopped_vessel(self):
        own, targets, note = build_scenario("head_on", num_targets=1)
        own.speed_kn = 0.0
        sim = USVSimulator(own, targets, SimConfig(current_speed_kn=1.0, current_dir_deg=90.0, max_steps=3), note)
        x0 = sim.own.east_m
        sim.step((0.0, 1.0))
        self.assertGreater(sim.own.east_m, x0)

    def test_obstacle_contact_terminates_episode(self):
        own, targets, note = build_scenario("head_on", num_targets=1)
        obstacle = StaticObstacle("test-rock", own.east_m, own.north_m + 10.0, 30.0)
        sim = USVSimulator(own, targets, SimConfig(max_steps=10), note, obstacles=[obstacle])
        _, done, reason = sim.step((0.0, 1.0))
        self.assertTrue(done)
        self.assertTrue(sim.collision)
        self.assertIn("障碍物", reason)
        self.assertIn("test-rock", sim.obstacle_contacts)

    def test_fault_modes_activate(self):
        expected_visible = {"gnss_loss": None, "compass_bias": None, "propulsion_loss": None,
                           "rudder_jam": None, "sensor_dropout": 0}
        for mode in expected_visible:
            with self.subTest(mode=mode):
                sim = self._sim(SimConfig(max_steps=5, failure_mode=mode, failure_at_step=0, random_seed=1))
                sim.step((20.0, 1.0))
                self.assertEqual(sim.active_failure, mode)
                if mode == "gnss_loss":
                    self.assertFalse(sim.sensor_snapshot["gnss_valid"])
                if mode == "sensor_dropout":
                    self.assertEqual(sim.sensor_snapshot["visible_targets"], 0)

    def test_propulsion_loss_reduces_speed_and_rudder_jam_holds_heading(self):
        sim_p = self._sim(SimConfig(max_steps=5, failure_mode="propulsion_loss", failure_at_step=0,
                                   max_accel_kn_s=100.0, random_seed=2))
        sim_p.step((0.0, 1.0))
        self.assertEqual(sim_p.own.speed_kn, 0.0)
        sim_r = self._sim(SimConfig(max_steps=5, failure_mode="rudder_jam", failure_at_step=0, random_seed=2))
        h0 = sim_r.own.course_deg
        sim_r.step((45.0, 1.0))
        self.assertAlmostEqual(sim_r.own.course_deg, h0)


    def test_time_limit_is_not_counted_as_safe_completion(self):
        sim = self._sim(SimConfig(max_steps=1, complete_after_pass=False, random_seed=3))
        _, done, reason = sim.step((0.0, 1.0))
        self.assertTrue(done)
        self.assertIn("未确认安全状态", reason)
        self.assertNotIn("安全通过", reason)

    def test_environment_and_fault_state_are_logged(self):
        sim = self._sim(SimConfig(current_speed_kn=1.0, wind_speed_kn=10.0, wave_height_m=1.0))
        state = sim.state_dict()
        self.assertIn("environment", state)
        self.assertIn("sensor", state)
        self.assertEqual(state["environment"]["current_speed_kn"], 1.0)

if __name__ == "__main__":
    unittest.main()
