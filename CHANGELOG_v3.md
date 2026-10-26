# v3 更新记录

在 v2 基础上，本版主要修改和新增：

- `usv_colav/models.py`：新增圆形静态障碍物模型 `StaticObstacle`。
- `usv_colav/simulator.py`：增加可复现的噪声传感器快照、目标探测/丢检和可见范围、洋流/风漂移/波浪扰动、真值与控制估计分离、障碍物净空/接触判定，以及 5 类系统故障注入。
- `usv_colav/rule_policy.py`：基线控制改为使用传感器估计状态。
- `usv_colav/safety_filter.py`：安全预测使用探测到的目标与障碍物；对远处障碍动态延长预测时域；在关键传感器/执行机构故障或无目标回波时请求停推进并提示人工接管。
- `usv_colav/env.py`：训练环境默认随机化传感器、能见度、风浪流、故障和静态障碍物。
- `train_ppo.py`：增加 `--no-domain-randomization` 对照开关。
- `run_gui.py`：增加环境/故障工况下拉菜单、故障触发步数、障碍物渲染、可见目标/故障显示和扩展日志字段。
- `evaluate.py`：评估时通过安全层筛查动作，并把安全层覆盖动作次数写入 CSV。
- `stress_test.py` / `run_stress_tests.bat`：新增 10 类固定种子的回归压力测试。
- `tests/test_robustness.py` 及 `tests/test_safety_layer.py`：覆盖传感器噪声、受限能见度、环境漂移、障碍物碰撞、故障模式、安全降级和未完成回合判定。
- `README.md`、`results/README.txt`：补充设置方法、限制、运行命令及样本结果。

验证结果：23 项单元测试通过；代码 `compileall` 通过；GUI 在虚拟显示环境下进入主事件循环且未观察到启动异常；日志写入烟雾测试通过。当前开发环境没有训练 PPO 所需的 Gymnasium / Stable-Baselines3 / PyTorch，因此压缩包不包含 PPO 权重。

注意：风浪流与故障响应均为研究用简化模型。安全层不是形式化安全证明，停推进指令也不能消除惯性/漂移，不能将此版本直接连接实船执行器。
