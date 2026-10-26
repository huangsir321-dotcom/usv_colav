import unittest

from run_gui import USVColavApp
from usv_colav.models import Vessel
from usv_colav.simulator import SimConfig, USVSimulator


class _Var:
    def __init__(self, value=None):
        self.value = value

    def set(self, value):
        self.value = value

    def get(self):
        return self.value


class _Scale:
    def __init__(self):
        self.options = {}

    def configure(self, **kwargs):
        self.options.update(kwargs)


class RouteRecoveryTests(unittest.TestCase):
    def test_gui_mode_does_not_finish_until_ownship_recovers_planned_route(self):
        own = Vessel("OV", 300.0, 0.0, 10.0, 0.0, length_m=8.0, is_own=True,
                     planned_course_deg=0.0, planned_speed_kn=10.0)
        target = Vessel("TV1", 300.0, 10000.0, 10.0, 0.0, length_m=8.0)
        sim = USVSimulator(own, [target], SimConfig(
            dt_s=5.0, max_steps=600, require_route_recovery=True,
            sensor_position_sigma_m=0.0, sensor_course_sigma_deg=0.0, sensor_speed_sigma_kn=0.0,
        ))

        done = False
        for _ in range(600):
            action = sim.route_recovery_action() if sim.route_recovery_active else (0.0, 1.0)
            _, done, _reason = sim.step(action)
            if done:
                break

        self.assertTrue(done)
        self.assertIn("本船已恢复计划航行", sim.terminal_reason)
        self.assertTrue(sim.is_back_on_planned_route())
        self.assertGreater(sim.route_recovery_streak, 0)

    def test_positive_tcpa_does_not_trigger_route_recovery_just_because_cpa_is_safe(self):
        own = Vessel("OV", 0.0, 0.0, 10.0, 0.0, length_m=8.0, is_own=True,
                     planned_course_deg=0.0, planned_speed_kn=10.0)
        # This target is still converging in relative-motion terms, but its predicted
        # CPA is comfortably beyond SDA. We still wait for the actual passing phase.
        target = Vessel("TV1", 3000.0, 5000.0, 10.0, 90.0, length_m=8.0)
        sim = USVSimulator(own, [target], SimConfig(
            dt_s=5.0, max_steps=100, require_route_recovery=True,
            sensor_position_sigma_m=0.0, sensor_course_sigma_deg=0.0, sensor_speed_sigma_kn=0.0,
        ))
        for _ in range(12):
            sim.step((0.0, 1.0))
        metrics = sim.metrics()
        self.assertGreater(metrics[0]["tcpa_min"], 0.0)
        self.assertGreater(metrics[0]["cpa_m"], 1.5 * metrics[0]["sda_m"])
        self.assertFalse(sim.route_recovery_active)

    def test_route_recovery_action_turns_toward_route(self):
        own = Vessel("OV", 300.0, 0.0, 10.0, 0.0, length_m=8.0, is_own=True,
                     planned_course_deg=0.0, planned_speed_kn=10.0)
        target = Vessel("TV1", 300.0, 10000.0, 10.0, 0.0, length_m=8.0)
        sim = USVSimulator(own, [target], SimConfig(require_route_recovery=True))
        delta, speed = sim.route_recovery_action()
        self.assertLess(delta, 0.0)  # positive cross-track error -> turn left toward route
        self.assertEqual(speed, 1.0)


class TimelineHistoryTests(unittest.TestCase):
    def _app_with_sim(self, sim):
        app = USVColavApp.__new__(USVColavApp)
        app.sim = sim
        app.timeline_history = []
        app.timeline_cursor = 0
        app._updating_timeline = False
        app.timeline_scale = _Scale()
        app.timeline_index_var = _Var(0.0)
        app.timeline_label_var = _Var("")
        return app

    def test_timeline_can_restore_each_saved_state_and_trail(self):
        own = Vessel("OV", 0.0, 0.0, 10.0, 0.0, length_m=8.0, is_own=True,
                     planned_course_deg=0.0, planned_speed_kn=10.0)
        target = Vessel("TV1", 0.0, 3000.0, 8.0, 180.0, length_m=8.0)
        sim = USVSimulator(own, [target], SimConfig(max_steps=20, sensor_position_sigma_m=0.0,
                                                    sensor_course_sigma_deg=0.0, sensor_speed_sigma_kn=0.0))
        app = self._app_with_sim(sim)
        app._reset_timeline_history()
        app.sim.step((0.0, 1.0))
        app._append_timeline_frame()
        first_position = app.sim.own.position
        app.sim.step((0.0, 1.0))
        app._append_timeline_frame()
        self.assertEqual(len(app.timeline_history), 3)

        app._restore_timeline_frame(1)
        self.assertEqual(app.sim.step_count, 1)
        self.assertEqual(app.sim.own.position, first_position)
        self.assertEqual(len(app.sim.own.track), 2)
        self.assertEqual(app.timeline_cursor, 1)

        app._restore_timeline_frame(2)
        self.assertEqual(app.sim.step_count, 2)
        self.assertEqual(len(app.sim.own.track), 3)
        self.assertEqual(app.timeline_scale.options["to"], 2)


if __name__ == "__main__":
    unittest.main()
