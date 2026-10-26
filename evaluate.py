from __future__ import annotations
import argparse, csv, json, math, random
from pathlib import Path
from usv_colav.scenarios import build_random_scenario, build_scenario
from usv_colav.simulator import SimConfig, USVSimulator
from usv_colav.rule_policy import choose_rule_action
from usv_colav.safety_filter import shield_action


def run_episode(policy: str, model, num_targets: int, episode: int, sda_nm: float, max_steps: int, seed: int):
    if episode % 4 == 0:
        own, targets, note = build_scenario("ccs_d6_3", num_targets=num_targets)
    elif episode % 4 == 1 and num_targets == 2:
        own, targets, note = build_scenario("ccs_d5", num_targets=2)
    else:
        own, targets, note = build_random_scenario(num_targets=num_targets, seed=seed + episode)
    sim = USVSimulator(own, targets, SimConfig(sda_nm=sda_nm, max_steps=max_steps), note)
    sim._last_along_track = 0.0
    total_reward = 0.0
    shield_overrides = 0
    reason = "达到仿真时长"
    for _ in range(max_steps):
        if policy == "rule":
            action = choose_rule_action(sim)
        else:
            import numpy as np
            obs = np.asarray(sim.observation_values(), dtype=np.float32)
            raw, _ = model.predict(obs, deterministic=True)
            raw = raw.tolist()
            action = (float(raw[0]) * sim.config.max_course_offset_deg,
                      0.80 + 0.20 * max(-1.0, min(1.0, float(raw[1]))))
        action, speed_ratio, overridden, note = shield_action(sim, action)
        sim.last_safety_filter = note
        shield_overrides += int(overridden)
        reward, done, reason = sim.step((action, speed_ratio))
        total_reward += reward
        if done:
            break
    min_sep = min(sim.min_distance_by_target.values(), default=math.inf)
    min_cpa = min(sim.min_cpa_by_target.values(), default=math.inf)
    return {
        "episode": episode, "num_targets": len(targets), "policy": policy,
        "safe_completion": int(not sim.collision and not sim.sda_violations and "安全通过" in sim.terminal_reason),
        "collision": int(sim.collision), "sda_violation": int(bool(sim.sda_violations)),
        "min_separation_m": round(min_sep, 2), "min_cpa_m": round(min_cpa, 2),
        "steps": sim.step_count, "return": round(total_reward, 3),
        "safety_filter_overrides": shield_overrides,
        "reason": sim.terminal_reason if sim.completed else reason,
        "metrics_json": json.dumps(sim.metrics(), ensure_ascii=False), "scenario_note": note,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="1V1/1V2/1V3 避碰仿真评估")
    ap.add_argument("--policy", choices=["rule", "ppo"], default="rule")
    ap.add_argument("--model", default="models/usv_colav_ppo.zip")
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--targets", type=int, choices=[1, 2, 3], default=None,
                    help="不指定时按 1V1、1V2、1V3 循环评估")
    ap.add_argument("--sda-nm", type=float, default=0.30)
    ap.add_argument("--max-steps", type=int, default=420)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--out", default="results/evaluation.csv")
    args = ap.parse_args()
    model = None
    if args.policy == "ppo":
        try:
            from stable_baselines3 import PPO
        except ImportError:
            print("缺少 stable-baselines3，请先运行：python -m pip install -r requirements.txt")
            return 2
        path = Path(args.model)
        if not path.exists():
            print(f"找不到模型：{path}。请先训练：python train_ppo.py --timesteps 300000")
            return 2
        model = PPO.load(str(path))
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for i in range(args.episodes):
        n = args.targets or (i % 3 + 1)
        row = run_episode(args.policy, model, n, i + 1, args.sda_nm, args.max_steps, args.seed)
        rows.append(row)
        print(f"ep={i+1:03d} targets=1V{n} safe={row['safe_completion']} collision={row['collision']} SDA_violation={row['sda_violation']} min_sep={row['min_separation_m']:.1f}m")
    with out_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        if rows:
            writer.writeheader(); writer.writerows(rows)
    safe = sum(r["safe_completion"] for r in rows)
    collisions = sum(r["collision"] for r in rows)
    violations = sum(r["sda_violation"] for r in rows)
    print("\n=== 汇总 ===")
    print(f"策略={args.policy} episodes={len(rows)} 安全通过率={safe/max(1,len(rows)):.1%} 碰撞数={collisions} SDA违规次数={violations}")
    print(f"逐回合数据：{out_path}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
