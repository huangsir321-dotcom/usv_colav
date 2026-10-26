from __future__ import annotations
from dataclasses import dataclass
import random
from .geometry import NM_TO_M, wrap360, velocity_from_course
from .models import Vessel

@dataclass(frozen=True)
class TargetSpec:
    speed_kn: float
    course_deg: float
    range_nm: float
    bearing_rel_deg: float
    length_m: float = 8.0
    name: str = "TV"
    sda_nm: float | None = None

@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    own_speed_kn: float
    own_course_deg: float
    targets: tuple[TargetSpec, ...]
    note: str = ""

PRESETS: dict[str, ScenarioSpec] = {
    "head_on": ScenarioSpec("对遇", 15.0, 0.0, (TargetSpec(12.0, 180.0, 3.0, 0.0, name="TV1"),), "仿真构造：两船近似对遇。"),
    "crossing_starboard": ScenarioSpec("右舷交叉", 12.0, 0.0, (TargetSpec(12.0, 270.0, 2.4, 60.0, name="TV1"),), "目标位于本船右舷侧，横穿本船航路。"),
    "crossing_port": ScenarioSpec("左舷交叉", 12.0, 0.0, (TargetSpec(12.0, 90.0, 2.4, -60.0, name="TV1"),), "目标位于本船左舷侧，横穿本船航路。"),
    "overtaking": ScenarioSpec("追越", 15.0, 0.0, (TargetSpec(8.0, 0.0, 2.0, 0.0, name="TV1"),), "本船航速较快，目标船位于前方同向航行。"),
    "ccs_d5": ScenarioSpec("CCS 附录 D.5（两目标）", 15.0, 0.0, (
        TargetSpec(18.0, 181.0, 6.22, -1.5, 200.0, "TV1"),
        TargetSpec(10.0, 0.0, 1.18, 20.4, 200.0, "TV2"),
    ), "使用附录 D.5 的初始参数；此仿真版本让目标船保持给定航向航速。"),
    "ccs_d6_3": ScenarioSpec("CCS 附录 D.6（取 TV1-TV3）", 15.0, 0.0, (
        TargetSpec(18.0, 181.0, 7.50, -1.0, 200.0, "TV1"),
        TargetSpec(8.0, 0.0, 2.17, 9.2, 200.0, "TV2"),
        TargetSpec(15.0, 230.0, 9.00, 23.7, 200.0, "TV3"),
    ), "从附录 D.6 的四目标参数中取 TV1-TV3，以适配 1V3；目标船保持给定航向航速。"),
}


def _target_from_spec(spec: TargetSpec, own_course: float) -> Vessel:
    bearing = wrap360(own_course + spec.bearing_rel_deg)
    r = spec.range_nm * NM_TO_M
    # 0 deg = north, clockwise positive.
    import math
    x = r * math.sin(math.radians(bearing))
    y = r * math.cos(math.radians(bearing))
    return Vessel(spec.name, x, y, spec.speed_kn, spec.course_deg, length_m=spec.length_m, is_own=False,
                  sda_nm=spec.sda_nm)


def build_scenario(preset: str = "head_on", num_targets: int | None = None,
                   custom_targets: list[TargetSpec] | None = None,
                   own_speed_kn: float | None = None, own_course_deg: float | None = None,
                   seed: int | None = None) -> tuple[Vessel, list[Vessel], str]:
    """Build an initial encounter. Target courses are fixed after reset by default."""
    if preset == "random":
        return build_random_scenario(num_targets=num_targets or 1, seed=seed)
    selected = PRESETS.get(preset, PRESETS["head_on"])
    own_speed = float(own_speed_kn if own_speed_kn is not None else selected.own_speed_kn)
    own_course = wrap360(float(own_course_deg if own_course_deg is not None else selected.own_course_deg))
    specs = list(custom_targets) if custom_targets is not None else list(selected.targets)
    count = max(1, min(3, int(num_targets if num_targets is not None else len(specs))))
    # When a preset does not contain enough targets, supplement with varied, deterministic positions.
    while len(specs) < count:
        idx = len(specs) + 1
        specs.append(TargetSpec(10.0 + idx, wrap360(200 + idx * 55), 2.5 + idx * 0.3,
                                -45.0 + idx * 40.0, 8.0, f"TV{idx}"))
    specs = specs[:count]
    own = Vessel("OV", 0.0, 0.0, own_speed, own_course, length_m=8.0, is_own=True,
                 planned_course_deg=own_course, planned_speed_kn=own_speed)
    targets = [_target_from_spec(s, own_course) for s in specs]
    return own, targets, selected.note


def build_random_scenario(num_targets: int = 1, seed: int | None = None) -> tuple[Vessel, list[Vessel], str]:
    rng = random.Random(seed)
    count = max(1, min(3, int(num_targets)))
    own_course = 0.0
    own_speed = rng.uniform(9.0, 15.0)
    own = Vessel("OV", 0.0, 0.0, own_speed, own_course, length_m=8.0, is_own=True,
                 planned_course_deg=own_course, planned_speed_kn=own_speed)
    targets: list[Vessel] = []
    categories = ["head_on", "crossing_starboard", "crossing_port", "overtaking"]
    for i in range(count):
        category = categories[i % len(categories)] if rng.random() < 0.7 else rng.choice(categories)
        distance = rng.uniform(1.4, 4.5)
        speed = rng.uniform(6.0, 17.0)
        if category == "head_on":
            brg = rng.uniform(-8.0, 8.0)
            course = wrap360(own_course + 180.0 + rng.uniform(-7.0, 7.0))
        elif category == "crossing_starboard":
            brg = rng.uniform(25.0, 85.0)
            course = rng.uniform(250.0, 295.0)
        elif category == "crossing_port":
            brg = rng.uniform(-85.0, -25.0)
            course = rng.uniform(65.0, 110.0)
        else:
            brg = rng.uniform(-12.0, 12.0)
            course = wrap360(own_course + rng.uniform(-8.0, 8.0))
            speed = min(speed, own_speed - rng.uniform(1.0, 5.0))
            speed = max(2.0, speed)
            distance = rng.uniform(1.5, 3.2)
        spec = TargetSpec(speed, course, distance, brg, 8.0, f"TV{i + 1}")
        targets.append(_target_from_spec(spec, own_course))
    return own, targets, "训练随机场景：包含对遇、交叉和追越类相对运动。"
