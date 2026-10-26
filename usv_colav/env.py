"""Optional Gymnasium environment for Stable-Baselines3 PPO training."""
from __future__ import annotations
import math
try:
    import gymnasium as gym
    from gymnasium import spaces
    import numpy as np
except ImportError as exc:  # helpful message when the desktop simulator is used without RL extras
    raise ImportError("强化学习训练需要安装 requirements.txt：python -m pip install -r requirements.txt") from exc

from .scenarios import build_random_scenario
from .simulator import SimConfig, USVSimulator
from .models import StaticObstacle
from .safety_filter import shield_action

class USVColavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, fixed_num_targets: int | None = None, sda_nm: float = 0.30,
                 dt_s: float = 5.0, max_steps: int = 420, seed: int | None = None,
                 domain_randomization: bool = True):
        super().__init__()
        self.fixed_num_targets = fixed_num_targets
        self.sda_nm = float(sda_nm)
        self.dt_s = float(dt_s)
        self.max_steps = int(max_steps)
        self.base_seed = seed
        self.domain_randomization = bool(domain_randomization)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0], dtype=np.float32),
                                       high=np.array([1.0, 1.0], dtype=np.float32), dtype=np.float32)
        # 7 own-ship features + 3 targets * 10 features each.
        self.observation_space = spaces.Box(low=-5.0, high=5.0, shape=(37,), dtype=np.float32)
        self.sim: USVSimulator | None = None
        self._episode_no = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        self._episode_no += 1
        requested = options.get("num_targets", self.fixed_num_targets)
        if requested is None:
            requested = int(self.np_random.integers(1, 4))
        seed_value = int(self.np_random.integers(0, 2**31 - 1))
        own, targets, note = build_random_scenario(num_targets=int(requested), seed=seed_value)
        config = SimConfig(dt_s=self.dt_s, sda_nm=self.sda_nm, max_steps=self.max_steps,
                           complete_after_pass=True, random_seed=seed_value)
        obstacles = []
        if self.domain_randomization:
            # Domain randomization makes each PPO episode experience sensor error,
            # weather/current drift, variable visibility and occasional faults.
            config.sensor_position_sigma_m = float(self.np_random.uniform(3.0, 20.0))
            config.sensor_course_sigma_deg = float(self.np_random.uniform(0.5, 4.0))
            config.sensor_speed_sigma_kn = float(self.np_random.uniform(0.1, 0.8))
            config.sensor_dropout_probability = float(self.np_random.uniform(0.0, 0.08))
            config.visibility_m = float(self.np_random.uniform(1800.0, 18520.0))
            config.current_speed_kn = float(self.np_random.uniform(0.0, 1.5))
            config.current_dir_deg = float(self.np_random.uniform(0.0, 360.0))
            config.wind_speed_kn = float(self.np_random.uniform(0.0, 18.0))
            config.wind_from_deg = float(self.np_random.uniform(0.0, 360.0))
            config.wave_height_m = float(self.np_random.uniform(0.0, 1.8))
            config.wave_period_s = float(self.np_random.uniform(5.0, 10.0))
            if self.np_random.random() < 0.12:
                config.failure_mode = str(self.np_random.choice(["gnss_loss", "compass_bias", "propulsion_loss", "rudder_jam", "sensor_dropout"]))
                config.failure_at_step = int(self.np_random.integers(10, max(11, min(80, self.max_steps))))
            if self.np_random.random() < 0.25:
                bearing = float(self.np_random.uniform(-65.0, 65.0))
                distance = float(self.np_random.uniform(1000.0, 3000.0))
                angle = math.radians(float(own.planned_course_deg) + bearing)
                obstacles.append(StaticObstacle("OB-RAND", own.east_m + math.sin(angle)*distance,
                                                 own.north_m + math.cos(angle)*distance,
                                                 radius_m=float(self.np_random.uniform(30.0, 100.0))))
        self.sim = USVSimulator(own, targets, config, note + ("；开启环境随机化" if self.domain_randomization else "；固定名义环境"), obstacles=obstacles)
        self.sim._last_along_track = 0.0
        obs = np.asarray(self.sim.observation_values(), dtype=np.float32)
        return obs, {"num_targets": len(targets), "scenario_note": note}

    def step(self, action):
        if self.sim is None:
            raise RuntimeError("必须先调用 reset()")
        a = np.asarray(action, dtype=float).reshape(-1)
        if len(a) < 2:
            a = np.pad(a, (0, 2-len(a)))
        course_delta = float(np.clip(a[0], -1.0, 1.0)) * self.sim.config.max_course_offset_deg
        speed_ratio = 0.80 + 0.20 * float(np.clip(a[1], -1.0, 1.0))
        proposed = (course_delta, speed_ratio)
        safe_delta, safe_ratio, overridden, shield_note = shield_action(self.sim, proposed)
        self.sim.last_safety_filter = shield_note
        reward, done, reason = self.sim.step((safe_delta, safe_ratio))
        if overridden:
            reward -= 0.15
        truncated = bool(done and reason.startswith("达到仿真时长"))
        terminated = bool(done and not truncated)
        obs = np.asarray(self.sim.observation_values(), dtype=np.float32)
        min_sep = min(self.sim.min_distance_by_target.values(), default=math.inf)
        info = {
            "terminal_reason": reason,
            "collision": self.sim.collision,
            "sda_violations": sorted(self.sim.sda_violations),
            "min_separation_m": min_sep,
            "metrics": self.sim.metrics(),
            "safety_filter_overrode_action": overridden,
            "safety_filter_note": shield_note,
            "safe_completion": done and not self.sim.collision and not self.sim.sda_violations and "安全通过" in self.sim.terminal_reason,
        }
        return obs, float(reward), terminated, truncated, info

    def render(self):
        return None
