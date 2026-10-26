import unittest

from run_gui import (
    UNLIMITED_PLAYBACK_LABEL,
    UNLIMITED_STEPS_PER_TICK,
    USVColavApp,
    playback_tick_plan,
)


class _Var:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


class _Root:
    def __init__(self):
        self.calls = []

    def after(self, delay, callback):
        self.calls.append(("after", delay, callback))
        return "after-id"

    def after_idle(self, callback):
        self.calls.append(("after_idle", 0, callback))
        return "idle-id"


class PlaybackModeTests(unittest.TestCase):
    def test_unlimited_mode_advances_one_step_per_frame_without_delay(self):
        steps, delay_ms = playback_tick_plan(UNLIMITED_PLAYBACK_LABEL)
        self.assertEqual(steps, 1)
        self.assertEqual(steps, UNLIMITED_STEPS_PER_TICK)
        self.assertEqual(delay_ms, 0)

    def test_accelerated_modes_change_delay_but_never_batch_steps(self):
        expected = {
            "0.25x": (1, 720),
            "0.5x": (1, 360),
            "1x": (1, 180),
            "2x": (1, 90),
            "5x": (1, 36),
            "10x": (1, 18),
            "20x": (1, 9),
            "50x": (1, 4),
        }
        for label, plan in expected.items():
            with self.subTest(label=label):
                self.assertEqual(playback_tick_plan(label), plan)

    def test_unlimited_mode_can_be_selected_by_alias(self):
        self.assertEqual(playback_tick_plan("unlimited"), (1, 0))
        self.assertEqual(playback_tick_plan("fastest"), (1, 0))

    def test_invalid_speed_falls_back_to_one_x(self):
        self.assertEqual(playback_tick_plan("invalid"), (1, 180))
        self.assertEqual(playback_tick_plan("0x"), (1, 180))
        self.assertEqual(playback_tick_plan("nanx"), (1, 180))

    def test_animation_callback_advances_exactly_one_step(self):
        app = USVColavApp.__new__(USVColavApp)
        app.root = _Root()
        app.running = True
        app.after_id = "old-id"
        app.playback_speed_var = _Var("50x")
        advances = []
        app._advance_steps = lambda count, render: advances.append((count, render))
        app._animation_tick()
        self.assertEqual(advances, [(1, True)])
        self.assertEqual(app.after_id, "after-id")
        self.assertEqual(app.root.calls[0][0:2], ("after", 4))

    def test_unlimited_animation_yields_to_idle_redraw_between_frames(self):
        app = USVColavApp.__new__(USVColavApp)
        app.root = _Root()
        app.running = True
        app.after_id = None
        app.playback_speed_var = _Var(UNLIMITED_PLAYBACK_LABEL)
        advances = []
        app._advance_steps = lambda count, render: advances.append((count, render))
        app._animation_tick()
        self.assertEqual(advances, [(1, True)])
        self.assertEqual(app.after_id, "idle-id")
        self.assertEqual(app.root.calls[0][0:2], ("after_idle", 0))


if __name__ == "__main__":
    unittest.main()
