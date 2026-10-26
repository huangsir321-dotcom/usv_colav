"""Planar maritime encounter geometry. Heading: 0 deg north, clockwise positive."""
from __future__ import annotations
from dataclasses import dataclass
import math

KNOT_TO_MPS = 0.514444
NM_TO_M = 1852.0


def wrap360(angle: float) -> float:
    return float(angle % 360.0)


def wrap180(angle: float) -> float:
    return float((angle + 180.0) % 360.0 - 180.0)


def knots_to_mps(speed_kn: float) -> float:
    return float(speed_kn) * KNOT_TO_MPS


def mps_to_knots(speed_mps: float) -> float:
    return float(speed_mps) / KNOT_TO_MPS


def nm_to_m(distance_nm: float) -> float:
    return float(distance_nm) * NM_TO_M


def m_to_nm(distance_m: float) -> float:
    return float(distance_m) / NM_TO_M


def velocity_from_course(speed_kn: float, course_deg: float) -> tuple[float, float]:
    """Return (east_mps, north_mps) from nautical course (0=N, 90=E)."""
    speed = knots_to_mps(speed_kn)
    angle = math.radians(course_deg)
    return speed * math.sin(angle), speed * math.cos(angle)


def bearing_deg(dx_east_m: float, dy_north_m: float) -> float:
    """Absolute bearing from a relative position vector (0=N, 90=E)."""
    return wrap360(math.degrees(math.atan2(dx_east_m, dy_north_m)))


@dataclass(frozen=True)
class CPAResult:
    cpa_m: float
    tcpa_min: float
    relative_speed_mps: float
    current_distance_m: float
    closest_east_m: float
    closest_north_m: float


def cpa_tcpa(
    own_position: tuple[float, float],
    own_speed_kn: float,
    own_course_deg: float,
    target_position: tuple[float, float],
    target_speed_kn: float,
    target_course_deg: float,
) -> CPAResult:
    """Calculate linear relative-motion CPA/TCPA.

    TCPA is signed: positive means the closest point is in the future; negative
    means the two vessels have passed the linear-motion CPA point. For near-zero
    relative speed, TCPA is +inf and CPA is the present separation.
    """
    rx = target_position[0] - own_position[0]
    ry = target_position[1] - own_position[1]
    ovx, ovy = velocity_from_course(own_speed_kn, own_course_deg)
    tvx, tvy = velocity_from_course(target_speed_kn, target_course_deg)
    vx, vy = tvx - ovx, tvy - ovy
    vv = vx * vx + vy * vy
    current = math.hypot(rx, ry)
    if vv < 1e-10:
        return CPAResult(current, math.inf, math.sqrt(vv), current, rx, ry)
    t_sec = -(rx * vx + ry * vy) / vv
    cx = rx + vx * t_sec
    cy = ry + vy * t_sec
    return CPAResult(math.hypot(cx, cy), t_sec / 60.0, math.sqrt(vv), current, cx, cy)


def predicted_position(position: tuple[float, float], speed_kn: float, course_deg: float, seconds: float) -> tuple[float, float]:
    vx, vy = velocity_from_course(speed_kn, course_deg)
    return position[0] + vx * seconds, position[1] + vy * seconds


def relative_bearing(own_position: tuple[float, float], own_course_deg: float, target_position: tuple[float, float]) -> float:
    dx = target_position[0] - own_position[0]
    dy = target_position[1] - own_position[1]
    return wrap180(bearing_deg(dx, dy) - own_course_deg)
