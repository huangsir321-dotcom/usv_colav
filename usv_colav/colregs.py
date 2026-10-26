"""Explainable, simplified encounter classification and action preferences.

This is a research/demo implementation inspired by COLREGs concepts. It is not
an authoritative legal rules engine or a substitute for formal COLREGs review.
"""
from __future__ import annotations
from .geometry import wrap180


def classify_encounter(own_course_deg: float, own_speed_kn: float,
                       target_course_deg: float, target_speed_kn: float,
                       target_relative_bearing_deg: float) -> str:
    rel_bearing = wrap180(target_relative_bearing_deg)
    course_diff = abs(wrap180(target_course_deg - own_course_deg))
    # Own vessel approaching a slower vessel ahead on a similar course.
    if course_diff <= 25.0 and abs(rel_bearing) <= 35.0 and own_speed_kn > target_speed_kn + 0.8:
        return "own_overtaking"
    # Target is behind the beam and closing on own vessel in a similar direction.
    if course_diff <= 35.0 and abs(rel_bearing) >= 145.0 and target_speed_kn > own_speed_kn + 0.8:
        return "own_being_overtaken"
    if course_diff >= 150.0 and abs(rel_bearing) <= 25.0:
        return "head_on"
    if 0.0 < rel_bearing <= 112.5:
        return "crossing_target_starboard"
    if -112.5 <= rel_bearing < 0.0:
        return "crossing_target_port"
    if abs(rel_bearing) > 112.5:
        return "astern_or_overtaking"
    return "uncertain"


def own_give_way(encounter: str) -> bool:
    """Conservative responsibility label for the simplified power-driven model.

    Head-on: both alter to starboard; crossing: own vessel gives way when the
    target is on own starboard side; own overtaking: overtaking vessel gives way.
    """
    return encounter in {"head_on", "crossing_target_starboard", "own_overtaking"}


def preferred_turn_sign(encounter: str) -> int:
    """Return +1 for starboard preference, 0 for hold/preference unspecified."""
    if encounter in {"head_on", "crossing_target_starboard", "own_overtaking"}:
        return 1
    return 0


def risk_level(distance_m: float, cpa_m: float, tcpa_min: float, sda_m: float) -> str:
    """Transparent approximation of the three risk labels named by GB/T 46851."""
    if distance_m <= sda_m * 0.55 or (0.0 < tcpa_min <= 2.0 and cpa_m <= sda_m * 0.65):
        return "紧迫局面"
    if (tcpa_min > 0.0 and cpa_m < sda_m) or distance_m < sda_m:
        return "碰撞危险"
    return "无危险"


def risk_score(risk: str) -> int:
    return {"无危险": 0, "碰撞危险": 1, "紧迫局面": 2}.get(risk, 0)
