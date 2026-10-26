"""Repeatable robustness matrix for sensor/environment/fault/obstacle conditions."""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path

from usv_colav.models import StaticObstacle
from usv_colav.rule_policy import choose_rule_action
from usv_colav.safety_filter import shield_action
from usv_colav.scenarios import build_scenario
from usv_colav.simulator import SimConfig, USVSimulator


def make_case(case: str, seed: int, max_steps: int):
    cfg = SimConfig(max_steps=max_steps, random_seed=seed, failure_at_step=12,
                    sensor_position_sigma_m=4.0, sensor_course_sigma_deg=1.5, sensor_speed_sigma_kn=0.3)
    obstacles = []
    if case == "sensor_noise":
        cfg.sensor_position_sigma_m, cfg.sensor_course_sigma_deg, cfg.sensor_speed_sigma_kn = 25.0, 5.0, 1.0
        cfg.sensor_dropout_probability = 0.12
    elif case == "wind_wave_current":
        cfg.current_speed_kn, cfg.current_dir_deg = 1.5, 45.0
        cfg.wind_speed_kn, cfg.wind_from_deg = 20.0, 270.0
        cfg.wave_height_m, cfg.wave_period_s = 2.0, 7.5
    elif case == "restricted_visibility":
        cfg.visibility_m = 800.0
        cfg.sensor_position_sigma_m, cfg.sensor_course_sigma_deg = 12.0, 3.5
    elif case == "static_obstacle":
        obstacles = [StaticObstacle("OB1", 0.0, 1750.0, 100.0)]
    elif case in {"gnss_loss", "compass_bias", "propulsion_loss", "rudder_jam", "sensor_dropout"}:
        cfg.failure_mode = case
    elif case != "nominal":
        raise ValueError(f"未知测试工况：{case}")
    cfg.random_seed = seed
    # Make copies independent between episodes via deterministic reconstruction.
    own, targets, note2 = build_scenario("head_on", num_targets=1)
    own.course_deg = own.planned_course_deg = 0.0
    return USVSimulator(own, targets, cfg, f"{note2}; stress={case}", obstacles=obstacles)


def run_episode(case: str, seed: int, max_steps: int) -> dict:
    sim = make_case(case, seed, max_steps)
    overrides = 0
    visible_seen = []
    total_reward = 0.0
    for _ in range(max_steps):
        proposed = choose_rule_action(sim)
        safe_action, safe_ratio, overridden, filter_note = shield_action(sim, proposed)
        sim.last_safety_filter = filter_note
        overrides += int(overridden)
        visible_seen.append(int(sim.sensor_snapshot.get("visible_targets", 0)))
        reward, done, reason = sim.step((safe_action, safe_ratio))
        total_reward += reward
        if done:
            break
    min_sep = min(sim.min_distance_by_target.values(), default=math.inf)
    min_obstacle = min(sim.min_obstacle_clearance_m.values(), default=math.inf)
    return {
        "case": case, "seed": seed, "num_targets": len(sim.targets), "steps": sim.step_count,
        "active_failure": sim.active_failure,
        "visible_targets_min": min(visible_seen, default=0), "visible_targets_max": max(visible_seen, default=0),
        "safety_filter_overrides": overrides,
        "safe_completion": int(not sim.collision and not sim.sda_violations and not sim.obstacle_contacts and "安全通过" in sim.terminal_reason),
        "vessel_collision": int(sim.collision and not sim.obstacle_contacts),
        "obstacle_contact": int(bool(sim.obstacle_contacts)),
        "sda_violation": int(bool(sim.sda_violations)),
        "min_target_separation_m": round(min_sep, 2),
        "min_obstacle_clearance_m": round(min_obstacle, 2) if math.isfinite(min_obstacle) else "NA",
        "return": round(total_reward, 3), "terminal_reason": sim.terminal_reason if sim.completed else reason,
        "obstacle_contacts": ";".join(sorted(sim.obstacle_contacts)),
        "sda_violations": ";".join(sorted(sim.sda_violations)),
        "environment_json": json.dumps({"current_kn": sim.config.current_speed_kn,
            "current_dir_deg": sim.config.current_dir_deg, "wind_kn": sim.config.wind_speed_kn,
            "wind_from_deg": sim.config.wind_from_deg, "wave_height_m": sim.config.wave_height_m,
            "visibility_m": sim.config.visibility_m, "sensor_position_sigma_m": sim.config.sensor_position_sigma_m,
            "sensor_course_sigma_deg": sim.config.sensor_course_sigma_deg,
            "sensor_dropout_probability": sim.config.sensor_dropout_probability}, ensure_ascii=False),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="传感器、风浪流、受限能见度、静态障碍物和系统故障压力测试")
    parser.add_argument("--episodes-per-case", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=120)
    parser.add_argument("--seed", type=int, default=20261015)
    parser.add_argument("--out", default="results/robustness_stress_test.csv")
    args = parser.parse_args()
    cases = ["nominal", "sensor_noise", "wind_wave_current", "restricted_visibility", "static_obstacle",
             "gnss_loss", "compass_bias", "propulsion_loss", "rudder_jam", "sensor_dropout"]
    rows = []
    for i, case in enumerate(cases):
        for ep in range(max(1, args.episodes_per_case)):
            seed = args.seed + i*100 + ep
            row = run_episode(case, seed, max(1, args.max_steps))
            rows.append(row)
            print(f"{case:22s} seed={seed} safe={row['safe_completion']} collision={row['vessel_collision']} "
                  f"obstacle={row['obstacle_contact']} SDA={row['sda_violation']} visible={row['visible_targets_min']}..{row['visible_targets_max']} "
                  f"steps={row['steps']} reason={row['terminal_reason']}")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\n样本数={len(rows)} 安全完成={sum(r['safe_completion'] for r in rows)} "
          f"船舶碰撞={sum(r['vessel_collision'] for r in rows)} 障碍物接触={sum(r['obstacle_contact'] for r in rows)} "
          f"SDA违规={sum(r['sda_violation'] for r in rows)}")
    print(f"逐回合 CSV：{out}")
    print("提示：该矩阵只用于软件回归/压力测试，不代表真实海况概率或实船安全结论。")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
