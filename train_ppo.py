from __future__ import annotations
import argparse
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="训练 PPO 无人艇避碰策略")
    parser.add_argument("--timesteps", type=int, default=300_000, help="总训练步数，建议先用 30000 做冒烟测试")
    parser.add_argument("--n-envs", type=int, default=4, help="并行环境数量")
    parser.add_argument("--sda-nm", type=float, default=0.30, help="安全会遇距离 SDA（海里，仿真参数）")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="models/usv_colav_ppo")
    parser.add_argument("--fixed-targets", type=int, choices=[1, 2, 3], default=None,
                        help="默认不固定，训练时随机生成 1V1/1V2/1V3")
    parser.add_argument("--no-domain-randomization", action="store_true",
                        help="关闭传感器/风浪流/能见度/障碍物/偶发故障随机化（默认开启）")
    args = parser.parse_args()
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.callbacks import EvalCallback
        from usv_colav.env import USVColavEnv
    except ImportError as exc:
        print("缺少强化学习依赖，请先运行：python -m pip install -r requirements.txt")
        print(f"详细信息：{exc}")
        return 2

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    (Path("logs") / "ppo_tensorboard").mkdir(parents=True, exist_ok=True)
    env = make_vec_env(lambda: USVColavEnv(fixed_num_targets=args.fixed_targets, sda_nm=args.sda_nm,
                                           seed=args.seed, domain_randomization=not args.no_domain_randomization),
                         n_envs=max(1, args.n_envs), seed=args.seed)
    eval_env = USVColavEnv(fixed_num_targets=args.fixed_targets, sda_nm=args.sda_nm,
                           seed=args.seed + 1, domain_randomization=not args.no_domain_randomization)
    eval_callback = EvalCallback(eval_env, best_model_save_path=str(out.parent / "best"),
                                 log_path=str(Path("logs") / "ppo_eval"), eval_freq=max(2000, 5000 // max(1, args.n_envs)),
                                 n_eval_episodes=8, deterministic=True, render=False)
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, n_steps=1024,
                batch_size=256, gamma=0.99, gae_lambda=0.95, ent_coef=0.01,
                clip_range=0.2, policy_kwargs={"net_arch": [256, 256]}, seed=args.seed,
                tensorboard_log=str(Path("logs") / "ppo_tensorboard"))
    try:
        model.learn(total_timesteps=args.timesteps, callback=eval_callback, progress_bar=False)
        model.save(str(out))
        print(f"训练完成，模型已保存：{out}.zip")
        print("注意：训练回报不等于满足 CCS 标准；还应使用 evaluate.py 执行分场景测试并检查 SDA 违规。")
    finally:
        env.close()
        eval_env.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
