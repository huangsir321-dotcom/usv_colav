"""A transparent rule-based baseline used before an RL policy is trained."""
from __future__ import annotations
import math
from .geometry import NM_TO_M, cpa_tcpa, relative_bearing, wrap180, wrap360
from .colregs import classify_encounter, preferred_turn_sign


def _approach_heading(current: float, desired: float, max_delta: float) -> float:
    error = wrap180(desired - current)
    return wrap360(current + max(-max_delta, min(max_delta, error)))


def choose_rule_action(sim) -> tuple[float, float]:
    """Return (desired offset from route in degrees, speed ratio 0.60..1.0).

    Candidates are scored jointly against every target, so dodging one target
    while cutting in front of another is penalized. This is a baseline, not a
    proven-safe navigation controller.
    """
    cfg = sim.config
    own = sim.control_own() if hasattr(sim, "control_own") else sim.own
    targets = sim.control_targets() if hasattr(sim, "control_targets") else sim.targets
    deltas = [-45, -35, -25, -15, -8, 0, 8, 15, 25, 35, 45]
    speed_ratios = [1.0, 0.90, 0.80, 0.70, 0.60]
    best = (0.0, 1.0)
    best_score = -math.inf
    sda = sim.sda_m
    turn_step = cfg.max_turn_rate_deg_s * cfg.dt_s
    for delta in deltas:
        desired_course = wrap360(float(own.planned_course_deg) + delta)
        predicted_course = _approach_heading(own.course_deg, desired_course, turn_step)
        for speed_ratio in speed_ratios:
            predicted_speed = float(own.planned_speed_kn or own.speed_kn) * speed_ratio
            score = -0.030 * abs(delta) - 0.010 * abs(delta - sim.last_course_delta_deg) - 4.0 * (1.0 - speed_ratio)
            min_margin = math.inf
            urgent = []
            for target in targets:
                result = cpa_tcpa(own.position, predicted_speed, predicted_course,
                                  target.position, target.speed_kn, target.course_deg)
                rb = relative_bearing(own.position, own.course_deg, target.position)
                encounter = classify_encounter(own.course_deg, own.speed_kn,
                                               target.course_deg, target.speed_kn, rb)
                cpa = result.cpa_m
                tcpa = result.tcpa_min
                dist = result.current_distance_m
                target_sda = (target.sda_nm * NM_TO_M) if getattr(target, "sda_nm", None) is not None else sda
                margin = cpa - target_sda
                min_margin = min(min_margin, margin)
                # Treat negative TCPA as diverging, while current-range checks remain active.
                if dist < target_sda:
                    score -= 10.0 * (target_sda - dist) / target_sda + 2.0
                if tcpa > 0.0 and tcpa < 35.0:
                    if cpa < target_sda:
                        score -= 18.0 + 25.0 * (target_sda - cpa) / target_sda
                        urgent.append((target, encounter, tcpa))
                    elif cpa < 1.5 * target_sda:
                        score -= 5.0 * (1.5 - cpa / target_sda) * (1.0 + max(0.0, 20.0 - tcpa) / 20.0)
                    else:
                        score += min(2.0, (cpa / target_sda - 1.5) * 0.4)
                elif tcpa <= 0.0 and dist >= target_sda:
                    score += 0.25
                if encounter in {"head_on", "crossing_target_starboard", "own_overtaking"} and tcpa > 0.0 and cpa < 1.8 * target_sda:
                    if delta > 0:
                        score += 1.8
                    elif delta < 0:
                        score -= 1.2
            # Extra conservative margin and minimal route deviation.
            if min_margin > 0:
                score += min(3.0, min_margin / sda * 0.7)
            if delta == 0 and not urgent:
                score += 0.7
            if score > best_score:
                best_score, best = score, (float(delta), float(speed_ratio))
    return best
