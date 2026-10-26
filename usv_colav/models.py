from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from .geometry import wrap360

@dataclass
class Vessel:
    name: str
    east_m: float
    north_m: float
    speed_kn: float
    course_deg: float
    length_m: float = 10.0
    is_own: bool = False
    planned_course_deg: float | None = None
    planned_speed_kn: float | None = None
    sda_nm: float | None = None
    track: list[tuple[float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.course_deg = wrap360(self.course_deg)
        if self.planned_course_deg is None:
            self.planned_course_deg = self.course_deg
        else:
            self.planned_course_deg = wrap360(self.planned_course_deg)
        if self.planned_speed_kn is None:
            self.planned_speed_kn = self.speed_kn
        if not self.track:
            self.track.append((self.east_m, self.north_m))

    @property
    def position(self) -> tuple[float, float]:
        return self.east_m, self.north_m

    def move_to(self, east_m: float, north_m: float, keep_track: bool = True) -> None:
        self.east_m, self.north_m = float(east_m), float(north_m)
        if keep_track:
            self.track.append(self.position)
            if len(self.track) > 4000:
                del self.track[: len(self.track) - 4000]

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name, "east_m": self.east_m, "north_m": self.north_m,
            "speed_kn": self.speed_kn, "course_deg": self.course_deg,
            "length_m": self.length_m, "is_own": self.is_own,
            "planned_course_deg": self.planned_course_deg,
            "planned_speed_kn": self.planned_speed_kn, "sda_nm": self.sda_nm,
        }


@dataclass(frozen=True)
class StaticObstacle:
    """Circular static obstacle in local east/north coordinates."""
    name: str
    east_m: float
    north_m: float
    radius_m: float = 30.0

    def __post_init__(self) -> None:
        if self.radius_m <= 0:
            raise ValueError("静态障碍物 radius_m 必须大于 0")

    @property
    def position(self) -> tuple[float, float]:
        return float(self.east_m), float(self.north_m)

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "east_m": self.east_m, "north_m": self.north_m, "radius_m": self.radius_m}
