from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path
from typing import Any

from usv_colav.case_loader import list_encounter_cases, load_encounter_csv
from usv_colav.geometry import NM_TO_M
from usv_colav.rule_policy import choose_rule_action
from usv_colav.safety_filter import shield_action
from usv_colav.simulator import SimConfig, USVSimulator

GROUP_SEED_OFFSET = {"1V1": 11000, "1V2": 22000, "1V3": 33000}


def run_case(group: str, path: Path, args: argparse.Namespace) -> dict[str, Any]:
    own, targets, meta = load_encounter_csv(path)
    match = re.search(r"scene_(\d+)", path.stem)
    scene_number = int(match.group(1)) if match else 0
    seed = int(args.seed) + GROUP_SEED_OFFSET[group] + scene_number
    if args.sensor_noise:
        sensor_pos, sensor_course, sensor_speed, dropout = 4.0, 1.5, 0.3, 0.01
    else:
        sensor_pos, sensor_course, sensor_speed, dropout = 0.0, 0.0, 0.0, 0.0
    visibility_nm = max(0.0, float(args.visibility_nm))
    config = SimConfig(
        sda_nm=float(args.sda_nm),
        max_steps=int(args.max_steps),
        # Full-horizon validation: do not stop early at the first projected-clearance
        # event; continue until all targets pass or the configured time horizon ends.
        safe_resolution_early_stop=False,
        random_seed=seed,
        sensor_position_sigma_m=sensor_pos,
        sensor_course_sigma_deg=sensor_course,
        sensor_speed_sigma_kn=sensor_speed,
        sensor_dropout_probability=dropout,
        observation_range_nm=max(30.0, visibility_nm),
        visibility_m=visibility_nm * NM_TO_M,
        current_speed_kn=float(args.current_speed_kn),
        current_dir_deg=float(args.current_dir_deg),
        wind_speed_kn=float(args.wind_speed_kn),
        wind_from_deg=float(args.wind_from_deg),
        wave_height_m=float(args.wave_height_m),
    )
    situation = meta["meeting_situations"]
    sim = USVSimulator(own, targets, config, f"{group}/{path.name}; {situation}")
    overrides = 0
    last_reason = "达到仿真时长"
    for _ in range(config.max_steps):
        proposed = choose_rule_action(sim)
        delta, ratio, overridden, filter_note = shield_action(sim, proposed)
        overrides += int(overridden)
        sim.last_safety_filter = filter_note
        _, done, last_reason = sim.step((delta, ratio))
        if done:
            break

    final_metrics = sim.metrics()
    passed = (
        not sim.collision
        and not sim.sda_violations
        and not sim.obstacle_contacts
        and "安全通过" in sim.terminal_reason
    )
    mins = list(sim.min_distance_by_target.values())
    min_seps = min(mins, default=math.inf)
    min_cpas = min(sim.min_cpa_by_target.values(), default=math.inf)
    final_min_dist = min((m["distance_m"] for m in final_metrics), default=math.inf)
    final_min_cpa = min((m["cpa_m"] for m in final_metrics), default=math.inf)
    source_dcpa = [r["source_dcpa_nm"] for r in meta["source_rows"] if r["source_dcpa_nm"] is not None]
    source_tcpa = [r["source_tcpa_min"] for r in meta["source_rows"] if r["source_tcpa_min"] is not None]
    return {
        "group": group,
        "scene": path.name,
        "targets": len(targets),
        "meeting_situations": situation,
        "source_stage": " | ".join(r["stage"] for r in meta["source_rows"] if r["stage"]),
        "ownship_sog_kn": meta["ownship_sog_kn"],
        "ownship_cog_deg": meta["ownship_cog_deg"],
        "source_dcpa_nm_min": min(source_dcpa) if source_dcpa else None,
        "source_dcpa_nm_max": max(source_dcpa) if source_dcpa else None,
        "source_tcpa_min_min": min(source_tcpa) if source_tcpa else None,
        "source_tcpa_min_max": max(source_tcpa) if source_tcpa else None,
        "safe_completion": int(passed),
        "collision": int(sim.collision),
        "sda_violation": int(bool(sim.sda_violations)),
        "obstacle_contact": int(bool(sim.obstacle_contacts)),
        "min_separation_m": round(min_seps, 3),
        "min_cpa_m": round(min_cpas, 3),
        "final_min_distance_m": round(final_min_dist, 3),
        "final_min_cpa_m": round(final_min_cpa, 3),
        "steps": sim.step_count,
        "elapsed_s": round(sim.elapsed_s, 1),
        "safety_filter_overrides": overrides,
        "terminal_reason": sim.terminal_reason,
        "sensor_noise": int(args.sensor_noise),
        "visibility_nm": visibility_nm,
        "current_speed_kn": config.current_speed_kn,
        "wind_speed_kn": config.wind_speed_kn,
        "wave_height_m": config.wave_height_m,
        "seed": seed,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="逐个运行用户提供的 1V1、1V2、1V3 CSV 测试场景（规则控制器 + 安全筛查层）"
    )
    parser.add_argument("--case-dir", default="test_cases", help="测试用例根目录，内含 1V1/1V2/1V3")
    parser.add_argument("--group", choices=["all", "1V1", "1V2", "1V3"], default="all")
    parser.add_argument("--sda-nm", type=float, default=0.30, help="安全距离阈值，海里")
    parser.add_argument("--max-steps", type=int, default=900)
    parser.add_argument("--seed", type=int, default=20261016)
    parser.add_argument("--out", default="results/test_case_evaluation.csv")
    parser.add_argument("--sensor-noise", action="store_true", help="叠加位置/航向/航速误差和 1%% 随机丢检")
    parser.add_argument("--visibility-nm", type=float, default=30.0,
                        help="感知范围（海里）；名义用例默认 30 NM，便于所有给定目标初始可见")
    parser.add_argument("--current-speed-kn", type=float, default=0.0)
    parser.add_argument("--current-dir-deg", type=float, default=0.0)
    parser.add_argument("--wind-speed-kn", type=float, default=0.0)
    parser.add_argument("--wind-from-deg", type=float, default=0.0)
    parser.add_argument("--wave-height-m", type=float, default=0.0)
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent
    case_root = Path(args.case_dir)
    if not case_root.is_absolute():
        case_root = project_root / case_root
    cases = list_encounter_cases(case_root, args.group)
    out_path = Path(args.out)
    if not out_path.is_absolute():
        out_path = project_root / out_path
    out_path.parent.mkdir(parents=True, exist_ok=True)

    results: list[dict[str, Any]] = []
    group_totals: dict[str, list[int]] = {}
    print(f"测试文件：{len(cases)} 个；策略：规则控制器 + 安全筛查层；SDA={args.sda_nm:.3f} NM")
    print(f"模式：{'传感器噪声' if args.sensor_noise else '确定性名义场景'}，能见距离={args.visibility_nm:.1f} NM")
    for i, (group, path) in enumerate(cases, start=1):
        result = run_case(group, path, args)
        results.append(result)
        counts = group_totals.setdefault(group, [0, 0, 0, 0, 0])
        counts[0] += 1
        counts[1] += result["safe_completion"]
        counts[2] += result["collision"]
        counts[3] += result["sda_violation"]
        counts[4] += result["obstacle_contact"]
        if not result["safe_completion"]:
            print(f"[FAIL] {group}/{path.name}: {result['terminal_reason']}; "
                  f"min_sep={result['min_separation_m']:.1f}m, min_CPA={result['min_cpa_m']:.1f}m")
        elif i % 20 == 0 or i == len(cases):
            print(f"进度 {i}/{len(cases)}；当前 {group}/{path.name} 通过；" 
                  f"min_sep={result['min_separation_m']:.1f}m")

    with out_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()) if results else [])
        if results:
            writer.writeheader()
            writer.writerows(results)

    print("\n========== 原始 CSV 场景回归汇总 ==========")
    for group in ("1V1", "1V2", "1V3"):
        if group not in group_totals:
            continue
        total, passed, collisions, sda, obstacles = group_totals[group]
        print(f"{group}: {passed}/{total} 通过；碰撞={collisions}，SDA违规={sda}，障碍物接触={obstacles}")
    total = len(results)
    passed = sum(r["safe_completion"] for r in results)
    collisions = sum(r["collision"] for r in results)
    sda = sum(r["sda_violation"] for r in results)
    obstacles = sum(r["obstacle_contact"] for r in results)
    print(f"总计：{passed}/{total} 通过（{100*passed/max(1,total):.2f}%）；"
          f"碰撞={collisions}，SDA违规={sda}，障碍物接触={obstacles}")
    print(f"逐用例结果：{out_path}")
    return 0 if total > 0 and passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
