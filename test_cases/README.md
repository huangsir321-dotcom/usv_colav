# 用户提供的 1V1 / 1V2 / 1V3 原始场景

本目录保存本次上传压缩包中的原始场景 CSV，并保留压缩包中附带的预览图（如有），未用随机场景替代。

| 目录 | 来源压缩包 | CSV 场景数 | 每个 CSV 中的目标船数 |
|---|---|---:|---:|
| `1V1/` | `1t.zip` | 138 | 1 |
| `1V2/` | `2t_0720.zip` | 99 | 2 |
| `1V3/` | `3t.zip` | 31 | 3 |
| **总计** | | **268** | |

`manifest.csv` 列出每个文件、目标船数、来源压缩包以及是否有原始预览图。程序通过 `usv_colav/case_loader.py` 读取场景；CSV 的经纬度转成本地 East/North 坐标，航向采用 0°=北、90°=东、顺时针为正。目标船初始航速和航向按 CSV 给定值设置，仿真中目标船保持该航向/航速（除非配置了环境扰动）。

## 运行全部原始用例

在工程根目录执行：

```bat
python evaluate_test_cases.py --group all
```

只跑一组：

```bat
python evaluate_test_cases.py --group 1V1
python evaluate_test_cases.py --group 1V2
python evaluate_test_cases.py --group 1V3
```

加入位置、罗经、航速误差和少量随机丢检，做固定种子的观测噪声回归：

```bat
python evaluate_test_cases.py --group all --sensor-noise --out results/test_case_evaluation_sensor_noise.csv
```

## 通过判据

一个用例只有在全程无碰撞、无 SDA 违规、无障碍物接触，并且目标船物理通过，或时限结束时最终距离/CPA 满足安全余量，才会计为通过。预测安全分离条件要求至少运行 60 秒；所有目标当前距离大于 `max(1.25 × SDA, SDA + 50 m)`，且线性预测 CPA 大于 `max(1.20 × SDA, SDA + 50 m)`。单纯达到最大步数不能算通过；只有时限结束时最终状态仍满足上述距离/CPA 余量，且全程无碰撞和 SDA 违规，才会计为安全完成。碰撞、曾发生 SDA 违规或预测余量不足的超时都不会计为通过。

基准运行会关闭“预测安全后立即终止”，而是继续到目标物理通过或 900 步时限，以统计完整已仿真的轨迹最小分离距离。默认基准是确定性名义环境，`SDA=0.30 NM`，关闭随机传感器误差与风浪流，以便可重复比较原始场景。`--sensor-noise` 是额外的带噪声测试。静态障碍物、恶劣环境和系统故障属于独立鲁棒性测试，不应与原始会遇场景的名义通过率混为一谈。
