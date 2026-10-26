from __future__ import annotations
from dataclasses import dataclass
import math
import random
from typing import Any

from .geometry import (NM_TO_M, bearing_deg, cpa_tcpa, m_to_nm, mps_to_knots, relative_bearing,
                       velocity_from_course, wrap180, wrap360)
from .models import StaticObstacle, Vessel
from .colregs import classify_encounter, own_give_way, risk_level, risk_score


@dataclass
class SimConfig:
    # Integration and own-ship manoeuvring limits
    dt_s: float = 5.0
    sda_nm: float = 0.30
    max_turn_rate_deg_s: float = 2.5
    max_yaw_accel_deg_s2: float = 0.8
    max_accel_kn_s: float = 0.35
    max_course_offset_deg: float = 45.0
    max_steps: int = 420
    collision_radius_m: float = 20.0
    observation_range_nm: float = 10.0
    complete_after_pass: bool = True
    # Interactive episodes may stop as soon as a conservative projected-clearance
    # condition is met. Benchmark runners can disable early stop and simulate the
    # full encounter horizon before judging the final state.
    safe_resolution_early_stop: bool = True
    # GUI playback can require the own vessel to recover its planned route after
    # the encounter has safely cleared, instead of stopping as soon as targets pass.
    require_route_recovery: bool = False
    route_recovery_cross_track_m: float = 100.0
    route_recovery_course_error_deg: float = 4.0
    route_recovery_speed_error_kn: float = 1.5
    route_recovery_hold_steps: int = 3

    # Sensor error / limited visibility. Units: metres, degrees, knots.
    sensor_position_sigma_m: float = 4.0
    sensor_course_sigma_deg: float = 1.5
    sensor_speed_sigma_kn: float = 0.3
    sensor_dropout_probability: float = 0.0
    visibility_m: float = 18520.0

    # Environmental disturbance. Wind direction is the direction wind comes FROM;
    # current direction is the direction current flows TO (both nautical bearings).
    current_speed_kn: float = 0.0
    current_dir_deg: float = 0.0
    wind_speed_kn: float = 0.0
    wind_from_deg: float = 0.0
    wind_drift_coefficient: float = 0.025
    wave_height_m: float = 0.0
    wave_period_s: float = 7.0
    wave_drift_mps_per_m: float = 0.035
    wave_yaw_noise_deg_per_m: float = 0.04

    # Reproducible system fault injection. Supported modes:
    # none, gnss_loss, compass_bias, propulsion_loss, rudder_jam, sensor_dropout.
    failure_mode: str = "none"
    failure_at_step: int = 100
    compass_bias_deg: float = 25.0
    random_seed: int = 7
    obstacle_safety_margin_m: float = 15.0


class USVSimulator:
    """Planar USV encounter simulator with imperfect sensing and disturbance injection.

    Truth states are used for outcome scoring; controller-facing snapshots are noisy.
    This is a research/test model, not a hydrodynamic simulator or a certified safety case.
    """
    VALID_FAILURES = {"none", "gnss_loss", "compass_bias", "propulsion_loss", "rudder_jam", "sensor_dropout"}

    def __init__(self, own: Vessel, targets: list[Vessel], config: SimConfig | None = None,
                 scenario_note: str = "", obstacles: list[StaticObstacle] | None = None):
        self.config = config or SimConfig()
        if not 1 <= len(targets) <= 3:
            raise ValueError("本工程支持 1V1、1V2、1V3，即 1 至 3 艘目标船。")
        if self.config.failure_mode not in self.VALID_FAILURES:
            raise ValueError(f"failure_mode 必须是 {sorted(self.VALID_FAILURES)} 之一")
        if self.config.dt_s <= 0 or self.config.max_steps <= 0:
            raise ValueError("dt_s 和 max_steps 必须大于 0")
        if not 0.0 <= self.config.sensor_dropout_probability <= 1.0:
            raise ValueError("sensor_dropout_probability 必须在 [0, 1] 内")
        self.own = own
        self.targets = targets
        self.obstacles = list(obstacles or [])
        self.scenario_note = scenario_note
        self.rng = random.Random(self.config.random_seed)
        self._last_sensor_own: dict[str, float] | None = None
        self._last_sensor_targets: dict[str, dict[str, float]] = {}
        self.sensor_snapshot: dict[str, Any] = {}
        self.reset_stats()

    def reset_stats(self) -> None:
        self.step_count = 0
        self.elapsed_s = 0.0
        self.last_course_delta_deg = 0.0
        self.last_speed_ratio = 1.0
        self.actual_yaw_rate_deg_s = 0.0
        self.last_safety_filter = "未筛查"
        self.min_distance_by_target = {t.name: math.hypot(t.east_m-self.own.east_m, t.north_m-self.own.north_m) for t in self.targets}
        self.min_cpa_by_target = {t.name: math.inf for t in self.targets}
        self.min_obstacle_clearance_m = {o.name: math.inf for o in self.obstacles}
        self.sda_violations: set[str] = set()
        self.obstacle_contacts: set[str] = set()
        self.collision = False
        self.completed = False
        self.terminal_reason = "运行中"
        self.active_failure = "none"
        self.route_recovery_active = False
        self.route_recovery_streak = 0
        self.own.track = [self.own.position]
        for t in self.targets:
            t.track = [t.position]
        self._last_along_track = 0.0
        self._refresh_sensor_snapshot()

    @property
    def sda_m(self) -> float:
        return max(10.0, self.config.sda_nm * NM_TO_M)

    @property
    def visibility_range_m(self) -> float:
        return max(0.0, min(float(self.config.visibility_m), self.config.observation_range_nm * NM_TO_M))

    def _activate_fault_if_due(self) -> None:
        if self.config.failure_mode != "none" and self.step_count >= max(0, self.config.failure_at_step):
            self.active_failure = self.config.failure_mode

    def _refresh_sensor_snapshot(self) -> None:
        """Take one noisy sensor sample per simulation step; avoid redraw on every query."""
        self._activate_fault_if_due()
        cfg = self.config
        dropout_mode = self.active_failure == "sensor_dropout"
        gnss_lost = self.active_failure == "gnss_loss"
        compass_bias = cfg.compass_bias_deg if self.active_failure == "compass_bias" else 0.0

        if gnss_lost and self._last_sensor_own is not None:
            own_sample = dict(self._last_sensor_own)
            own_sample["position_valid"] = 0.0
        else:
            own_sample = {
                "east_m": self.own.east_m + self.rng.gauss(0.0, max(0.0, cfg.sensor_position_sigma_m)),
                "north_m": self.own.north_m + self.rng.gauss(0.0, max(0.0, cfg.sensor_position_sigma_m)),
                "course_deg": wrap360(self.own.course_deg + compass_bias + self.rng.gauss(0.0, max(0.0, cfg.sensor_course_sigma_deg))),
                "speed_kn": max(0.0, self.own.speed_kn + self.rng.gauss(0.0, max(0.0, cfg.sensor_speed_sigma_kn))),
                "position_valid": 1.0,
            }
            self._last_sensor_own = dict(own_sample)
        self.sensor_snapshot = {"own": own_sample, "targets": {}, "obstacles": [], "visible_targets": 0,
                                "failure_mode": self.active_failure, "gnss_valid": not gnss_lost,
                                "compass_bias_deg": compass_bias, "sensor_dropout": dropout_mode}

        for target in self.targets:
            true_distance = math.hypot(target.east_m-self.own.east_m, target.north_m-self.own.north_m)
            visible = true_distance <= self.visibility_range_m and not dropout_mode
            if visible and self.rng.random() < cfg.sensor_dropout_probability:
                visible = False
            if visible:
                sample = {
                    "east_m": target.east_m + self.rng.gauss(0.0, max(0.0, cfg.sensor_position_sigma_m)),
                    "north_m": target.north_m + self.rng.gauss(0.0, max(0.0, cfg.sensor_position_sigma_m)),
                    "course_deg": wrap360(target.course_deg + self.rng.gauss(0.0, max(0.0, cfg.sensor_course_sigma_deg))),
                    "speed_kn": max(0.0, target.speed_kn + self.rng.gauss(0.0, max(0.0, cfg.sensor_speed_sigma_kn))),
                    "visible": True,
                }
                self._last_sensor_targets[target.name] = dict(sample)
                self.sensor_snapshot["visible_targets"] += 1
            else:
                # Keep a last-known track for diagnostics only; controllers do not receive it
                # as a fresh detection, so limited visibility is explicit rather than hidden.
                last = self._last_sensor_targets.get(target.name)
                sample = dict(last) if last else {"east_m": target.east_m, "north_m": target.north_m,
                                                   "course_deg": target.course_deg, "speed_kn": target.speed_kn}
                sample["visible"] = False
            self.sensor_snapshot["targets"][target.name] = sample

        for obstacle in self.obstacles:
            dist = math.hypot(obstacle.east_m-own_sample["east_m"], obstacle.north_m-own_sample["north_m"])
            if dist <= self.visibility_range_m and not dropout_mode:
                self.sensor_snapshot["obstacles"].append({
                    "name": obstacle.name,
                    "east_m": obstacle.east_m + self.rng.gauss(0.0, max(0.0, cfg.sensor_position_sigma_m)),
                    "north_m": obstacle.north_m + self.rng.gauss(0.0, max(0.0, cfg.sensor_position_sigma_m)),
                    "radius_m": obstacle.radius_m,
                })

    def control_own(self) -> Vessel:
        s = self.sensor_snapshot.get("own", {})
        return Vessel(self.own.name, s.get("east_m", self.own.east_m), s.get("north_m", self.own.north_m),
                      s.get("speed_kn", self.own.speed_kn), s.get("course_deg", self.own.course_deg),
                      length_m=self.own.length_m, is_own=True,
                      planned_course_deg=self.own.planned_course_deg, planned_speed_kn=self.own.planned_speed_kn,
                      sda_nm=self.own.sda_nm)

    def control_targets(self) -> list[Vessel]:
        out = []
        samples = self.sensor_snapshot.get("targets", {})
        for target in self.targets:
            sample = samples.get(target.name, {})
            if not sample.get("visible", False):
                continue
            out.append(Vessel(target.name, sample["east_m"], sample["north_m"], sample["speed_kn"],
                              sample["course_deg"], length_m=target.length_m, is_own=False, sda_nm=target.sda_nm))
        return out

    def control_obstacles(self) -> list[StaticObstacle]:
        """Cached sensor detections; same obstacles are used for all candidate rollouts."""
        return [StaticObstacle(o["name"], o["east_m"], o["north_m"], o["radius_m"])
                for o in self.sensor_snapshot.get("obstacles", [])]


    def _cross_track_m(self) -> float:
        dx, dy = self.own.east_m, self.own.north_m
        h = math.radians(float(self.own.planned_course_deg))
        return dx * math.cos(h) - dy * math.sin(h)

    def _along_track_m(self) -> float:
        h = math.radians(float(self.own.planned_course_deg))
        return self.own.east_m * math.sin(h) + self.own.north_m * math.cos(h)

    def _ground_kinematics(self, vessel: Vessel, time_s: float | None = None) -> tuple[float, float]:
        """Return approximate SOG/COG including current, wind drift and wave drift."""
        vx, vy = velocity_from_course(vessel.speed_kn, vessel.course_deg)
        dvx, dvy = self.disturbance_velocity(vessel, self.elapsed_s if time_s is None else time_s)
        vx += dvx; vy += dvy
        return mps_to_knots(math.hypot(vx, vy)), bearing_deg(vx, vy)

    def _metrics_for(self, own: Vessel, targets: list[Vessel]) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        own_sog, own_cog = self._ground_kinematics(own)
        for t in targets:
            target_sog, target_cog = self._ground_kinematics(t)
            result = cpa_tcpa(own.position, own_sog, own_cog,
                              t.position, target_sog, target_cog)
            rb = relative_bearing(own.position, own_cog, t.position)
            encounter = classify_encounter(own_cog, own_sog, target_cog, target_sog, rb)
            pair_sda_m = max(10.0, (t.sda_nm * NM_TO_M) if t.sda_nm is not None else self.sda_m)
            risk = risk_level(result.current_distance_m, result.cpa_m, result.tcpa_min, pair_sda_m)
            out.append({"target": t.name, "distance_m": result.current_distance_m,
                "distance_nm": m_to_nm(result.current_distance_m), "cpa_m": result.cpa_m,
                "cpa_nm": m_to_nm(result.cpa_m), "tcpa_min": result.tcpa_min,
                "relative_speed_kn": result.relative_speed_mps / 0.514444,
                "relative_bearing_deg": rb, "encounter": encounter,
                "own_give_way": own_give_way(encounter), "risk": risk,
                "sda_m": pair_sda_m, "sda_nm": pair_sda_m / NM_TO_M})
        return out

    def metrics(self) -> list[dict[str, Any]]:
        """Ground-truth metrics, used only for evaluation/reward and post-run diagnostics."""
        return self._metrics_for(self.own, self.targets)

    def control_metrics(self) -> list[dict[str, Any]]:
        """Noisy detections available to controllers; non-visible targets are omitted."""
        return self._metrics_for(self.control_own(), self.control_targets())

    def obstacle_clearances(self) -> dict[str, float]:
        return {o.name: math.hypot(o.east_m-self.own.east_m, o.north_m-self.own.north_m) - o.radius_m
                for o in self.obstacles}

    def disturbance_velocity(self, vessel: Vessel, time_s: float) -> tuple[float, float]:
        """Approximate environmental drift velocity in m/s, not a hydrodynamic model."""
        cvx, cvy = velocity_from_course(self.config.current_speed_kn, self.config.current_dir_deg)
        wind_to = wrap360(self.config.wind_from_deg + 180.0)
        wvx, wvy = velocity_from_course(self.config.wind_speed_kn * self.config.wind_drift_coefficient, wind_to)
        period = max(1.0, self.config.wave_period_s)
        wave_speed = min(0.8, max(0.0, self.config.wave_height_m) * self.config.wave_drift_mps_per_m) * math.sin(2.0 * math.pi * time_s / period)
        h = math.radians(vessel.course_deg)
        # Wave-induced lateral drift to starboard in body coordinates.
        return cvx + wvx + wave_speed * math.cos(h), cvy + wvy - wave_speed * math.sin(h)

    def observation_values(self) -> list[float]:
        truth_metrics = {m["target"]: m for m in self.control_metrics()}
        own = self.control_own()
        track = self.config.observation_range_nm * NM_TO_M
        own_course_error = wrap180(own.course_deg - float(self.own.planned_course_deg))
        visible_dists = [m["distance_m"] for m in truth_metrics.values()]
        obstacle_dists = [max(0.0, math.hypot(o.east_m-own.east_m, o.north_m-own.north_m)-o.radius_m)
                          for o in self.control_obstacles()]
        min_dist = min(visible_dists + obstacle_dists, default=track)
        h_plan = math.radians(float(self.own.planned_course_deg))
        cross_track_est = own.east_m*math.cos(h_plan) - own.north_m*math.sin(h_plan)
        along_track_est = own.east_m*math.sin(h_plan) + own.north_m*math.cos(h_plan)
        vals = [own_course_error / max(1.0, self.config.max_course_offset_deg),
            own.speed_kn / max(1.0, float(self.own.planned_speed_kn or self.own.speed_kn)),
            max(-2.0, min(2.0, cross_track_est / track)),
            max(-2.0, min(2.0, along_track_est / track)),
            self.step_count / max(1, self.config.max_steps),
            self.last_course_delta_deg / max(1.0, self.config.max_course_offset_deg),
            min(2.0, min_dist / track)]
        for i in range(3):
            if i < len(self.targets):
                t = self.targets[i]
                m = truth_metrics.get(t.name)
                if m is None:
                    vals.extend([0.0] * 10)
                    continue
                sample = self.sensor_snapshot["targets"][t.name]
                dx = sample["east_m"] - own.east_m
                dy = sample["north_m"] - own.north_m
                ovx, ovy = velocity_from_course(own.speed_kn, own.course_deg)
                own_dvx, own_dvy = self.disturbance_velocity(own, self.elapsed_s)
                ovx, ovy = ovx + own_dvx, ovy + own_dvy
                tvx, tvy = velocity_from_course(sample["speed_kn"], sample["course_deg"])
                sample_proxy = Vessel(t.name, sample["east_m"], sample["north_m"], sample["speed_kn"], sample["course_deg"])
                target_dvx, target_dvy = self.disturbance_velocity(sample_proxy, self.elapsed_s)
                tvx, tvy = tvx + target_dvx, tvy + target_dvy
                tcpa = m["tcpa_min"]
                tcpa_scaled = 1.0 if math.isinf(tcpa) and tcpa > 0 else (-1.0 if math.isinf(tcpa) else max(-1.0, min(1.0, tcpa / 30.0)))
                vals.extend([max(-2.0, min(2.0, dx / track)), max(-2.0, min(2.0, dy / track)),
                    max(-2.0, min(2.0, (tvx-ovx) / 10.0)), max(-2.0, min(2.0, (tvy-ovy) / 10.0)),
                    max(0.0, min(2.0, m["distance_m"] / track)),
                    max(-1.0, min(1.0, m["relative_bearing_deg"] / 180.0)),
                    max(0.0, min(3.0, m["cpa_m"] / m["sda_m"]),), tcpa_scaled,
                    wrap180(sample["course_deg"] - own.course_deg) / 180.0,
                    1.0 if m["own_give_way"] else 0.0])
            else:
                vals.extend([0.0] * 10)
        return vals

    def _predicted_safe_resolution(self, metrics: list[dict[str, Any]]) -> bool:
        """True when every current straight-line CPA is safely clear with margin.

        This is an episode-completion criterion, not a steering command or a formal
        safety proof. It requires at least 60 s of closed-loop operation, no past
        safety-distance violation, current range > 1.25*SDA, and predicted CPA >
        1.20*SDA for every target. Outcome scoring still checks ground-truth events.
        """
        if self.step_count < 12 or self.collision or self.sda_violations or self.obstacle_contacts:
            return False
        if not metrics:
            return False
        for metric in metrics:
            sda = float(metric["sda_m"])
            distance = float(metric["distance_m"])
            cpa = float(metric["cpa_m"])
            if distance <= max(1.25 * sda, sda + 50.0):
                return False
            if cpa <= max(1.20 * sda, sda + 50.0):
                return False
        return True

    def _targets_cleared_for_route_recovery(self, metrics: list[dict[str, Any]]) -> bool:
        """Require the encounter to be established as past/safely separated first."""
        if self.step_count < 12 or not metrics:
            return False
        all_targets_past_or_nonconverging = all(
            (
                float(m["tcpa_min"]) < -0.25
                and float(m["distance_m"]) > float(m["sda_m"])
            )
            or (
                math.isinf(float(m["tcpa_min"]))
                and float(m["distance_m"]) > max(2.0 * float(m["sda_m"]), float(m["sda_m"]) + 500.0)
                and float(m["cpa_m"]) > 1.5 * float(m["sda_m"])
            )
            for m in metrics
        )
        # Do not begin route-recovery just because an early straight-line CPA looks
        # safe while a target is still converging. Wait until targets have physically
        # passed (or are far away with essentially no relative motion).
        return all_targets_past_or_nonconverging

    def is_back_on_planned_route(self) -> bool:
        """Check position, heading and speed against the own-ship's planned route."""
        course_error = abs(wrap180(self.own.course_deg - float(self.own.planned_course_deg)))
        speed_error = abs(self.own.speed_kn - float(self.own.planned_speed_kn or self.own.speed_kn))
        return (
            abs(self._cross_track_m()) <= self.config.route_recovery_cross_track_m
            and course_error <= self.config.route_recovery_course_error_deg
            and speed_error <= self.config.route_recovery_speed_error_kn
        )

    def route_recovery_action(self) -> tuple[float, float]:
        """Steer toward a point ahead on the planned route after targets clear.

        Returns the ordinary GUI action format (heading offset from planned course,
        speed ratio). The route is represented by the planned-course line through the
        local coordinate origin, matching the simulator's cross-track calculation.
        """
        own = self.own
        plan = float(own.planned_course_deg)
        h = math.radians(plan)
        along = self._along_track_m()
        # Aim well ahead along the route. A longer look-ahead reduces abrupt heading
        # commands, while lateral error determines the intercept angle naturally.
        lookahead = max(700.0, min(1800.0, abs(self._cross_track_m()) * 2.0))
        target_along = along + lookahead
        target_east = target_along * math.sin(h)
        target_north = target_along * math.cos(h)
        desired_course = bearing_deg(target_east - own.east_m, target_north - own.north_m)
        delta = wrap180(desired_course - plan)
        delta = max(-self.config.max_course_offset_deg,
                    min(self.config.max_course_offset_deg, delta))
        return float(delta), 1.0

    def step(self, action: tuple[float, float] | list[float]) -> tuple[float, bool, str]:
        if self.completed:
            return 0.0, True, self.terminal_reason
        self._activate_fault_if_due()
        desired_delta = max(-self.config.max_course_offset_deg,
                            min(self.config.max_course_offset_deg, float(action[0])))
        # 0.0 is reserved for the outer safety layer's emergency slowdown/stop request;
        # learned/rule policies continue to produce the historical 0.60..1.0 range.
        speed_ratio = max(0.0, min(1.0, float(action[1])))
        self.last_course_delta_deg = desired_delta
        self.last_speed_ratio = speed_ratio
        dt = float(self.config.dt_s)

        # Failure injection changes actuator behaviour, not ground-truth evaluation.
        propulsion_lost = self.active_failure == "propulsion_loss"
        rudder_jammed = self.active_failure == "rudder_jam"
        desired_course = wrap360(float(self.own.planned_course_deg) + desired_delta)
        heading_error = wrap180(desired_course - self.own.course_deg)
        if rudder_jammed:
            self.actual_yaw_rate_deg_s = 0.0
            applied_turn = 0.0
        else:
            desired_yaw_rate = max(-self.config.max_turn_rate_deg_s,
                                   min(self.config.max_turn_rate_deg_s, heading_error / max(dt, 1e-6)))
            yaw_delta = self.config.max_yaw_accel_deg_s2 * dt
            self.actual_yaw_rate_deg_s += max(-yaw_delta, min(yaw_delta, desired_yaw_rate-self.actual_yaw_rate_deg_s))
            self.actual_yaw_rate_deg_s = max(-self.config.max_turn_rate_deg_s,
                                             min(self.config.max_turn_rate_deg_s, self.actual_yaw_rate_deg_s))
            applied_turn = self.actual_yaw_rate_deg_s * dt
            if abs(heading_error) < abs(applied_turn):
                applied_turn = heading_error
                self.actual_yaw_rate_deg_s = 0.0
        if self.config.wave_height_m > 0 and not rudder_jammed:
            applied_turn += self.rng.gauss(0.0, self.config.wave_yaw_noise_deg_per_m * self.config.wave_height_m)
        self.own.course_deg = wrap360(self.own.course_deg + applied_turn)

        desired_speed = 0.0 if propulsion_lost else float(self.own.planned_speed_kn or self.own.speed_kn) * speed_ratio
        accel_step = self.config.max_accel_kn_s * dt
        self.own.speed_kn += max(-accel_step, min(accel_step, desired_speed-self.own.speed_kn))
        evx, evy = velocity_from_course(self.own.speed_kn, self.own.course_deg)
        dvx, dvy = self.disturbance_velocity(self.own, self.elapsed_s + dt)
        self.own.move_to(self.own.east_m + (evx+dvx)*dt, self.own.north_m + (evy+dvy)*dt)
        for target in self.targets:
            tvx, tvy = velocity_from_course(target.speed_kn, target.course_deg)
            dvx, dvy = self.disturbance_velocity(target, self.elapsed_s + dt)
            target.move_to(target.east_m + (tvx+dvx)*dt, target.north_m + (tvy+dvy)*dt)

        self.step_count += 1
        self.elapsed_s += dt
        self._refresh_sensor_snapshot()
        metrics = self.metrics()
        distances = [float(m["distance_m"]) for m in metrics]
        for t, m in zip(self.targets, metrics):
            d = float(m["distance_m"])
            self.min_distance_by_target[t.name] = min(self.min_distance_by_target[t.name], d)
            self.min_cpa_by_target[t.name] = min(self.min_cpa_by_target[t.name], float(m["cpa_m"]))
            if d < float(m["sda_m"]):
                self.sda_violations.add(t.name)

        obstacle_clearances = self.obstacle_clearances()
        for name, clearance in obstacle_clearances.items():
            self.min_obstacle_clearance_m[name] = min(self.min_obstacle_clearance_m[name], clearance)
            obstacle = next(o for o in self.obstacles if o.name == name)
            if clearance <= self.config.collision_radius_m + self.own.length_m / 2.0:
                self.obstacle_contacts.add(name)

        reward = 0.02 * max(-1.0, min(1.0, (self._along_track_m()-self._last_along_track)/100.0))
        self._last_along_track = self._along_track_m()
        reward -= 0.018 * abs(desired_delta) / max(1.0, self.config.max_course_offset_deg)
        reward -= 0.05 * (1.0-speed_ratio)
        reward -= 0.006 * abs(self._cross_track_m()) / 100.0
        for m in metrics:
            d, cpa, tcpa, pair_sda = float(m["distance_m"]), float(m["cpa_m"]), float(m["tcpa_min"]), float(m["sda_m"])
            if d < pair_sda:
                reward -= 0.65 + 1.5*(pair_sda-d)/pair_sda
            if tcpa > 0.0 and tcpa < 25.0 and cpa < pair_sda*1.4:
                reward -= 0.15*max(0.0, 1.4-cpa/pair_sda)*(1.0+(25.0-tcpa)/25.0)
            if risk_score(str(m["risk"])) == 2:
                reward -= 0.5
        for clearance in obstacle_clearances.values():
            if clearance < 2.0*self.config.obstacle_safety_margin_m + self.own.length_m:
                reward -= 0.5 + max(0.0, 2.0*self.config.obstacle_safety_margin_m-clearance)*0.02

        done, reason = False, "运行中"
        if self.obstacle_contacts:
            self.collision = True
            done, reason = True, "静态障碍物接触/碰撞失败"
            reward -= 100.0
        elif min(distances, default=math.inf) <= self.config.collision_radius_m:
            self.collision = True
            done, reason = True, "碰撞/极近距离失败"
            reward -= 100.0
        elif self.config.require_route_recovery:
            # Successful completion must happen only after (1) the encounter is
            # clearly past/safely separated and (2) own ship has recovered track,
            # heading and speed for several consecutive simulator steps.
            if not self.route_recovery_active and self._targets_cleared_for_route_recovery(metrics):
                self.route_recovery_active = True
                self.route_recovery_streak = 0
            if self.route_recovery_active and self.is_back_on_planned_route():
                self.route_recovery_streak += 1
            else:
                self.route_recovery_streak = 0
            if (self.route_recovery_active and
                    self.route_recovery_streak >= max(1, int(self.config.route_recovery_hold_steps))):
                done, reason = True, "目标安全分离且本船已恢复计划航行"
            elif self.step_count >= self.config.max_steps:
                done, reason = True, "达到仿真时限但本船未恢复计划航行"
        else:
            if (self.config.complete_after_pass and
                    all(float(m["tcpa_min"]) < -0.25 and float(m["distance_m"]) > float(m["sda_m"]) for m in metrics)):
                done, reason = True, "目标船已通过"
            elif (self.config.complete_after_pass and self.config.safe_resolution_early_stop
                  and self._predicted_safe_resolution(metrics)):
                # Interactive use may end once the current projected paths are safely
                # separated. Benchmark runners can disable this early stop to keep
                # simulating the full encounter horizon before judging the trajectory.
                done, reason = True, "预测航迹已确认安全分离"
            elif self.step_count >= self.config.max_steps:
                if self.config.complete_after_pass and self._predicted_safe_resolution(metrics):
                    done, reason = True, "达到仿真时限且预测航迹保持安全"
                else:
                    done, reason = True, "达到仿真时长"
        if done:
            self.completed = True
            if self.collision or self.sda_violations or self.obstacle_contacts:
                self.terminal_reason = reason + "；未满足全部安全约束"
                reward -= 40.0 if not self.collision else 0.0
            elif reason in {"目标船已通过", "预测航迹已确认安全分离", "达到仿真时限且预测航迹保持安全",
                            "目标安全分离且本船已恢复计划航行"}:
                self.terminal_reason = reason + "；安全通过"
                reward += 25.0
            else:
                # Reaching the episode limit is not evidence of a successful encounter.
                self.terminal_reason = reason + "；未确认安全状态"
                reward -= 5.0
            return reward, True, self.terminal_reason
        return reward, False, reason

    def state_dict(self) -> dict[str, Any]:
        return {
            "step": self.step_count, "elapsed_s": self.elapsed_s, "own": self.own.to_dict(),
            "targets": [t.to_dict() for t in self.targets], "metrics": self.metrics(),
            "min_distance_by_target": self.min_distance_by_target, "min_cpa_by_target": self.min_cpa_by_target,
            "sda_violations": sorted(self.sda_violations), "collision": self.collision,
            "obstacles": [o.to_dict() for o in self.obstacles], "obstacle_clearances_m": self.obstacle_clearances(),
            "obstacle_contacts": sorted(self.obstacle_contacts), "min_obstacle_clearance_m": self.min_obstacle_clearance_m,
            "terminal_reason": self.terminal_reason, "scenario_note": self.scenario_note,
            "route_recovery_active": self.route_recovery_active,
            "route_recovery_streak": self.route_recovery_streak,
            "own_cross_track_m": self._cross_track_m(),
            "own_on_planned_route": self.is_back_on_planned_route(),
            "actual_yaw_rate_deg_s": self.actual_yaw_rate_deg_s, "last_safety_filter": self.last_safety_filter,
            "environment": {"current_speed_kn": self.config.current_speed_kn, "current_dir_deg": self.config.current_dir_deg,
                "wind_speed_kn": self.config.wind_speed_kn, "wind_from_deg": self.config.wind_from_deg,
                "wave_height_m": self.config.wave_height_m, "wave_period_s": self.config.wave_period_s,
                "visibility_m": self.config.visibility_m},
            "sensor": {"visible_targets": self.sensor_snapshot.get("visible_targets", 0),
                "gnss_valid": self.sensor_snapshot.get("gnss_valid", True),
                "sensor_dropout": self.sensor_snapshot.get("sensor_dropout", False),
                "failure_mode": self.active_failure,
                "own_estimate": self.sensor_snapshot.get("own", {}),
                "target_estimates": self.sensor_snapshot.get("targets", {}),
                "obstacle_detections": self.sensor_snapshot.get("obstacles", [])},
        }
