"""Load the user's 1V1 / 1V2 / 1V3 CSV encounter cases."""
from __future__ import annotations

import csv
import io
import math
from pathlib import Path
from typing import Any

from .models import Vessel

# The source dataset uses nautical convention: one degree latitude is 60 NM.
_METRES_PER_DEGREE = 60.0 * 1852.0


def _decode_csv(path: Path) -> str:
    raw = path.read_bytes()
    for encoding in ("utf-8-sig", "gb18030", "utf-16"):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise ValueError(f"无法识别 CSV 编码：{path}")


def _float(row: dict[str, Any], name: str, path: Path) -> float:
    value = row.get(name)
    if value is None or str(value).strip() == "":
        raise ValueError(f"{path.name} 缺少必填数值列 {name!r}")
    try:
        value_f = float(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path.name} 的 {name!r} 不是有效数字：{value!r}") from exc
    if not math.isfinite(value_f):
        raise ValueError(f"{path.name} 的 {name!r} 必须为有限数字")
    return value_f


def load_encounter_csv(path: str | Path) -> tuple[Vessel, list[Vessel], dict[str, Any]]:
    """Load one scene CSV as local east/north coordinates with own-ship at (0, 0).

    Each CSV row represents one target vessel. Own-ship data are taken from the
    first row; other rows must agree on own-ship initial pose/velocity within a
    small tolerance. Target coordinates are converted from lat/lon to a local
    tangent approximation suitable for the small study area of these cases.
    """
    source = Path(path)
    text = _decode_csv(source)
    rows = [r for r in csv.DictReader(io.StringIO(text)) if r and r.get("target_lat") not in (None, "")]
    if not rows:
        raise ValueError(f"场景文件没有目标船记录：{source}")

    own_lat = _float(rows[0], "ownship_lat", source)
    own_lon = _float(rows[0], "ownship_lon", source)
    own_speed = _float(rows[0], "ownship_sog(KN)", source)
    own_course = _float(rows[0], "ownship_cog(deg)", source)

    for idx, row in enumerate(rows[1:], start=2):
        for key, expected in (("ownship_lat", own_lat), ("ownship_lon", own_lon),
                              ("ownship_sog(KN)", own_speed), ("ownship_cog(deg)", own_course)):
            if abs(_float(row, key, source) - expected) > 1e-5:
                raise ValueError(f"{source.name} 第 {idx} 行的本船初始数据与第一行不一致：{key}")

    own = Vessel("OV", 0.0, 0.0, own_speed, own_course, length_m=8.0, is_own=True,
                 planned_course_deg=own_course, planned_speed_kn=own_speed)
    targets: list[Vessel] = []
    cos_lat = math.cos(math.radians(own_lat))
    source_meta: list[dict[str, Any]] = []
    for idx, row in enumerate(rows, start=1):
        target_lat = _float(row, "target_lat", source)
        target_lon = _float(row, "target_lon", source)
        east_m = math.radians(target_lon - own_lon) * 6371000.0 * cos_lat
        north_m = math.radians(target_lat - own_lat) * 6371000.0
        target = Vessel(
            f"TV{idx}", east_m, north_m,
            _float(row, "target_sog(KN)", source),
            _float(row, "target_cog(deg)", source),
            length_m=8.0,
        )
        targets.append(target)
        source_meta.append({
            "meeting_situation": str(row.get("meeting_situation", "") or "").strip(),
            "stage": str(row.get("stage", "") or "").strip(),
            "visibility": str(row.get("visibility", "") or "").strip(),
            "source_distance_nm": _maybe_float(row.get("distance(NM)")),
            "source_bearing_deg": _maybe_float(row.get("bearing(deg)")),
            "source_dcpa_nm": _maybe_float(row.get("DCPA(NM)")),
            "source_tcpa_min": _maybe_float(row.get("TCPA(min)")),
            "target_lat": target_lat,
            "target_lon": target_lon,
        })

    metadata = {
        "source_file": source.name,
        "ownship_lat": own_lat,
        "ownship_lon": own_lon,
        "ownship_sog_kn": own_speed,
        "ownship_cog_deg": own_course,
        "num_targets": len(targets),
        "source_rows": source_meta,
        "meeting_situations": " | ".join(x["meeting_situation"] for x in source_meta if x["meeting_situation"]),
    }
    return own, targets, metadata


def _maybe_float(value: Any) -> float | None:
    try:
        out = float(str(value).strip())
        return out if math.isfinite(out) else None
    except (TypeError, ValueError):
        return None


def list_encounter_cases(case_root: str | Path, group: str = "all") -> list[tuple[str, Path]]:
    """Return sorted (1V1/1V2/1V3, CSV path) entries."""
    root = Path(case_root)
    folder_by_group = {"1V1": "1V1", "1V2": "1V2", "1V3": "1V3"}
    groups = list(folder_by_group) if group.lower() == "all" else [group.upper()]
    if any(g not in folder_by_group for g in groups):
        raise ValueError("group 必须是 all、1V1、1V2 或 1V3")
    cases: list[tuple[str, Path]] = []
    for g in groups:
        folder = root / folder_by_group[g]
        if not folder.is_dir():
            raise FileNotFoundError(f"缺少测试用例目录：{folder}")
        paths = sorted(folder.glob("scene_*.csv"))
        expected_targets = {"1V1": 1, "1V2": 2, "1V3": 3}[g]
        for path in paths:
            cases.append((g, path))
    if not cases:
        raise FileNotFoundError(f"没有找到 scene_*.csv 测试用例：{root}")
    return cases
