# 标准与开源参考

## 本工程使用的标准依据

- GB/T 46851—2025《智能船舶 避碰系统技术要求及测试方法》，发布日 2025-12-31，实施日 2026-07-01。国家标准信息公共服务平台：<https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=80291C01F4832E540B1B45EDC825450E>
- 中国船级社对该标准实施信息的介绍：<https://www.ccs.org.cn/ccswz/articleDetail?id=202601070477109104>

本工程参考条款：5.2（输入/输出）、7.1（态势判断）、7.3（局部路径规划）、7.5（外部显示）、7.6（报警）、7.7（数据存储）、8.2（单船场景）、8.3（多船场景）及附录 A/B/D。附录 B 的 SDA 表是资料性示例，表 B.1 的数据对象是 100 m 散货船，不能直接视为 USV 法定安全距离。本工程默认 SDA=0.30 NM 仅为可配置的仿真初始参数。

## 可用于进一步研究的公开项目

- NTNU Autonomous Ship Lab `colav-simulator`：Gymnasium/Stable-Baselines 兼容的自主船舶避碰仿真器，场景生成、仿真、可视化与评估模块较完整。<https://github.com/ntnu-itk-autonomous-ship-lab/colav-simulator>
- `colregs-core`：面向 DRL/ASV 研究的 COLREGs 会遇分类、CPA/TCPA、船舶领域与奖励组件。<https://github.com/KangHyoeun/colregs-core>
- `autonomous-vessel-navigation`：包含船舶运动模型、CPA/TCPA/会遇分类与 RL 对比研究的 2D 仿真基准。<https://github.com/captv89/autonomous-vessel-navigation>

这些项目可作为后续扩展起点；本 ZIP 中的实现是独立的教学/研究原型，不复制其源码。


## 本次运动学/规则安全层补充参考
- IMO, Convention on the International Regulations for Preventing Collisions at Sea (COLREGs), 1972: https://www.imo.org/en/about/conventions/pages/colreg.aspx — Rule 13 overtaking, Rule 14 head-on, Rule 15 crossing, Rules 16–17 give-way/stand-on. The implementation uses only simplified heuristics for a research prototype, not full legal interpretation.
- U.S. Coast Guard Navigation Rules Handbook (2024): https://www.navcen.uscg.gov/sites/default/files/pdf/navRules/Handbook/Nav%20Rules%20Handbook_Corrected_08-12-2024.pdf — rule text used as a public reference for the concepts above.
