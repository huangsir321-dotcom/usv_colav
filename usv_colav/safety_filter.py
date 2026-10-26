"""Conservative action shield for research simulation (not a certified safety case)."""
from __future__ import annotations
import math
from .geometry import NM_TO_M, velocity_from_course, wrap180, wrap360


def _rollout_min_sep(sim, delta: float, speed_ratio: float, horizon_s: float = 90.0) -> tuple[float, float]:
    """Predict target and obstacle clearance using controller-facing sensor estimates."""
    own = sim.control_own() if hasattr(sim, "control_own") else sim.own
    control_targets = sim.control_targets() if hasattr(sim, "control_targets") else sim.targets
    control_obstacles = sim.control_obstacles() if hasattr(sim, "control_obstacles") else getattr(sim, "obstacles", [])
    own_x, own_y = own.position
    course = float(own.course_deg)
    speed = float(own.speed_kn)
    goal = wrap360(float(own.planned_course_deg) + delta)
    base_speed = float(own.planned_speed_kn or own.speed_kn)
    dt = min(5.0, float(sim.config.dt_s))
    n = max(1, int(horizon_s / dt))
    targets = [(t.east_m, t.north_m, t.speed_kn, t.course_deg, t.sda_nm) for t in control_targets]
    obstacles = [(o.name, o.east_m, o.north_m, o.radius_m) for o in control_obstacles]
    min_sep = math.inf
    min_margin = math.inf
    for k in range(n):
        err = wrap180(goal-course)
        max_turn = sim.config.max_turn_rate_deg_s * dt
        if getattr(sim, "active_failure", "none") == "rudder_jam":
            pass
        else:
            course = wrap360(course + max(-max_turn, min(max_turn, err)))
        desired_speed = 0.0 if getattr(sim, "active_failure", "none") == "propulsion_loss" else base_speed * speed_ratio
        accel_step = sim.config.max_accel_kn_s * dt
        speed += max(-accel_step, min(accel_step, desired_speed-speed))
        vx, vy = velocity_from_course(speed, course)
        own_proxy = sim.control_own()
        own_proxy.east_m, own_proxy.north_m, own_proxy.course_deg, own_proxy.speed_kn = own_x, own_y, course, speed
        if hasattr(sim, "disturbance_velocity"):
            dvx, dvy = sim.disturbance_velocity(own_proxy, sim.elapsed_s + (k+1)*dt)
            vx += dvx; vy += dvy
        own_x += vx*dt; own_y += vy*dt
        next_targets = []
        for tx, ty, ts, tc, target_sda_nm in targets:
            tvx, tvy = velocity_from_course(ts, tc)
            if hasattr(sim, "disturbance_velocity"):
                target_proxy = sim.control_own()
                target_proxy.east_m, target_proxy.north_m = tx, ty
                target_proxy.course_deg, target_proxy.speed_kn = tc, ts
                dvx, dvy = sim.disturbance_velocity(target_proxy, sim.elapsed_s + (k+1)*dt)
                tvx += dvx; tvy += dvy
            tx += tvx*dt; ty += tvy*dt
            d = math.hypot(tx-own_x, ty-own_y)
            sda = (target_sda_nm*NM_TO_M) if target_sda_nm is not None else sim.sda_m
            min_sep = min(min_sep, d)
            min_margin = min(min_margin, d-max(10.0, sda))
            next_targets.append((tx, ty, ts, tc, target_sda_nm))
        targets = next_targets
        for _name, ox, oy, radius in obstacles:
            clearance = math.hypot(ox-own_x, oy-own_y) - (radius + own.length_m/2.0 + sim.config.obstacle_safety_margin_m)
            min_sep = min(min_sep, max(0.0, clearance))
            min_margin = min(min_margin, clearance)
    return min_sep, min_margin


def shield_action(sim, proposed: tuple[float, float]) -> tuple[float, float, bool, str]:
    """Keep proposed action if predicted clear; otherwise select a safer candidate.

    Returns (course offset, speed ratio, overridden, explanation). It is a heuristic
    action filter, not a formal reachable-set or COLREGs compliance guarantee.
    """
    delta = max(-sim.config.max_course_offset_deg, min(sim.config.max_course_offset_deg, float(proposed[0])))
    ratio = max(0.0, min(1.0, float(proposed[1])))
    failure = getattr(sim, "active_failure", "none")
    visible_targets = sim.control_targets() if hasattr(sim, "control_targets") else sim.targets
    no_target_detections = not visible_targets and bool(sim.targets)
    # Degraded-mode response: request zero throttle rather than pretending a safe
    # trajectory can be computed from missing GNSS/compass/radar information.
    if failure in {"gnss_loss", "compass_bias", "sensor_dropout", "rudder_jam", "propulsion_loss"}:
        labels = {"gnss_loss": "GNSS失效", "compass_bias": "罗经偏差", "sensor_dropout": "传感器掉线",
                  "rudder_jam": "舵机卡滞", "propulsion_loss": "推进失效"}
        return 0.0, 0.0, True, f"降级保护：检测到{labels[failure]}，请求停止推进并人工接管；不能保证避碰"
    if no_target_detections:
        return 0.0, 0.0, True, "降级保护：当前无目标回波，请求停止推进等待目标可探测/人工接管；不能保证避碰"
    ratio = max(0.60, ratio)
    # Extend the look-ahead when a detected static obstacle is farther than the
    # default 90 s horizon; otherwise the shield could call a distant obstacle safe
    # simply because the rollout ends before reaching it.
    horizon_s = 90.0
    own_est = sim.control_own() if hasattr(sim, "control_own") else sim.own
    sensed_obstacles = sim.control_obstacles() if hasattr(sim, "control_obstacles") else getattr(sim, "obstacles", [])
    if sensed_obstacles:
        nearest = min(math.hypot(o.east_m-own_est.east_m, o.north_m-own_est.north_m) for o in sensed_obstacles)
        speed_mps = max(1.0, own_est.speed_kn * 0.514444)
        horizon_s = min(360.0, max(horizon_s, nearest / speed_mps * 1.25))
    _, proposed_margin = _rollout_min_sep(sim, delta, ratio, horizon_s=horizon_s)
    # A 15 m numerical buffer avoids switching on tiny floating-point differences.
    if proposed_margin >= 15.0:
        return delta, ratio, False, "原动作通过预测安全筛查"
    candidates = []
    for d in (-45, -30, -20, -10, 0, 10, 20, 30, 45):
        if abs(d) > sim.config.max_course_offset_deg:
            continue
        for r in (1.0, 0.85, 0.70, 0.60):
            _, margin = _rollout_min_sep(sim, float(d), r, horizon_s=horizon_s)
            # Prefer starboard for head-on / give-way crossing when otherwise comparable.
            preference = 0.0
            for m in (sim.control_metrics() if hasattr(sim, "control_metrics") else sim.metrics()):
                if m["tcpa_min"] > 0 and m["cpa_m"] < 1.5 * m["sda_m"]:
                    if m["encounter"] in {"head_on", "crossing_target_starboard", "own_overtaking"}:
                        # Rule 14 and ordinary give-way behaviour: favour an early,
                        # substantial starboard alteration when it remains safe for all targets.
                        preference += 140.0 if d >= 10.0 else (-90.0 if d < 0.0 else -15.0)
                    elif m["encounter"] == "crossing_target_port":
                        # Stand-on vessel: prefer course/speed maintenance unless danger is urgent.
                        if m["tcpa_min"] > 5.0 and margin > 0:
                            preference += 18.0 if abs(d) < 1.0 and r > 0.95 else -8.0
                        else:
                            preference += 10.0 if d > 0 else 0.0
            score = min(margin, 1500.0) - 2.5 * abs(d) - 180.0 * (1.0 - r) + preference
            candidates.append((score, margin, d, r))
    if not candidates:
        return delta, min(ratio, 0.6), True, "无候选动作，保守减速"
    safe = [c for c in candidates if c[1] >= 15.0]
    pool = safe or candidates
    _, margin, chosen_delta, chosen_ratio = max(pool, key=lambda c: c[0])
    explanation = "已替换为预测安全候选动作" if safe else "警告：预测时域内无满足安全缓冲的动作，选择风险较低动作并应人工接管"
    return float(chosen_delta), float(chosen_ratio), True, explanation
