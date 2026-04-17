# 项目中期检查表

编写时间：`2026-04-16`

编写依据：

- `README.md`
- `BUGFIX.md`
- `docx/current_system_flow.md`
- `docx/discussion_notes.md`
- `docx/execution_regression_checklist.md`
- `configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200/batch_runs_20260413_113943/batch_summary.json`
- `outputs/rl/eval_runs/ppo_usv_offshore_hotspot_ppo_only_20260413_134658/batch_summary.json`
- `.venv/bin/python -m pytest -q`

## 总体判断

- 当前项目已经完成“可运行基础闭环系统 + 多算法对比入口 + 回放日志链 + 第一版 RL 训练/评估入口”的中期核心目标。
- 项目阶段已经从“系统能否跑通”转入“结果稳定性、故障场景收口、对比实验和策略优化”的中后段推进。
- 当前验证体系已建立，但自动化回归尚未完全收口：本地执行 `.venv/bin/python -m pytest -q` 结果为 `421 passed, 2 failed, 5 subtests passed`，失败集中在 `sync_task_statuses` 的任务状态同步语义。

## 一、已完成的预期工作

| 检查项 | 当前状态 | 说明 |
| --- | --- | --- |
| Python 工程结构与基础工具链 | 已完成 | 已形成 `src/ + tests/ + configs/ + scripts/ + docx/ + outputs/` 的可维护工程结构，并接入 `pyproject.toml`、`pytest`、`ruff`。 |
| 基础海域环境与闭环仿真 | 已完成 | 已实现固定海域、障碍/风险区、`3 USV + 2 UAV`、栅格信息地图、基础任务与动态热点生成。 |
| 分层架构落地 | 已完成 | 任务层、规划层、执行层、事件层、仿真层均已独立成模块，并通过统一实验配置装配。 |
| 规则式任务分配与路径规划主线 | 已完成 | 已实现 `cost_aware / AEA / RHO / distributed_CBBA` 等任务层方案，以及多种 `USV/UAV` planner 组合。 |
| 执行层稳定性增强 | 已完成 | 已接入 `local_mpc_execution`、corridor owner/yield、动态 bottleneck、局部 recovery、return-to-patrol、安全 keepout 等机制。 |
| 事件与故障重规划链路 | 已完成 | 已支持 `agent_failure / speed_degradation / turn_rate_degradation`，并打通任务释放、接管与故障后运行约束。 |
| 回放、日志与回归检查工具 | 已完成 | 已能生成 HTML 回放、`events.jsonl`、`summary.json`，并提供日志检查脚本和固定 `1200 step` 执行层回归清单。 |
| 可复现实验数据集与批量评估 | 已完成 | 已建立 `experiment_datasets` 目录和 `batch.toml -> batch_results.jsonl / batch_summary.json` 的标准实验链。 |
| 第一版 RL 训练/评估管线 | 已完成 | 已具备 `Gymnasium + RLlib PPO` 的训练脚本、评估配置、推理适配器和三 seed 评估入口。 |
| 自动化测试体系 | 部分完成 | 测试体系已建成且覆盖面较大，但当前仍有 `2` 个与 `sync_task_statuses` 相关的失败用例未收口。 |

阶段性证据：

- `BUGFIX.md` 记录的最近两条主复核 seed 显示，`step_violation_count / flip_flop_count / supportless_claim_count / task_final_approach_exhausted` 已降到 `0`，说明控制层一致性问题已明显收敛。
- 当前任务分配对比数据集已经沉淀为正式 batch 产物，说明项目已具备“按统一口径反复复跑并比较算法”的能力。
- 当前 PPO 三 seed 评估产物已经落盘，说明 RL 主线至少完成了“环境接入、训练脚本、checkpoint 推理、评估汇总”这一整条链路。

## 二、原因分析及改进措施

| 当前问题或现象 | 原因分析 | 改进措施 |
| --- | --- | --- |
| 结果层波动仍然存在 | 动态热点、补能、失效应急、任务切换、局部执行和回巡航链路相互耦合，导致不同 seed 下 `coverage_ratio` 与 `stale_cells` 仍有明显波动。 | 继续固定主场景、固定 `3-seed / 1200-step` 口径复跑；每次改执行层或失效链路后，先过 `execution_regression_checklist.md`，再做 batch 对比。 |
| 故障后尾部 stale 与远端热点积压尚未完全压下 | `regional_recovery_debt` 已证明方向正确，但当前第一版阈值偏保守，高债务热点被重新放回候选的频率还不够。 | 优先微调 `regional_recovery_debt` 的高债务阈值和放行条件，不直接打散 `planner / recovery / local_mpc` 主链。 |
| `RETURN_TO_PATROL` 长时振荡与 final-approach 隐式丢单仍有残留 | 坏目标重建、释放原因链和末段失败语义还没有完全闭环，局部问题会以“返航振荡”或“任务被悄悄打回市场”的形式出现。 | 增强 `blocked_goal_signature`、release reason 和 candidate 轮换日志；针对返航目标重建和 `task_final_approach` 释放路径补更聚焦的测试。 |
| UAV-USV 固定支援配对尚不够严格 | 当前机制本质上还是“偏好 + 保护”，不是“预留 + 抢占”；首选 `USV` 一旦先被热点任务占住，就会退化到次优配对。 | 如果目标是严格固定带队，应新增更强的预留/抢占机制，而不是继续只调偏好排序或保护阈值。 |
| PPO 主线已能跑通，但尚未超过规则式主线 | 第一版 PPO 已建立训练与评估通路，但从现有三 seed 汇总看，热点确认和 stale 控制仍弱于当前规则式基线。 | 先把 PPO 作为探索性对照组保留；后续优先优化观测编码、奖励设计、候选槽位表达和评估对比，而不是提前替换规则式主线。 |
| 自动化回归尚未全绿 | `task_claim`/任务同步语义已经收紧，但 `sync_task_statuses` 的测试期望与当前实现可能尚未完全对齐，也不排除存在真实回归。 | 先明确 `RETURN_TO_PATROL` 空闲但已分配任务的最终语义，再同步修实现或更新测试，尽快把 `pytest` 收口到全绿。 |

补充判断：

- 当前项目的主矛盾已经不是“功能空白”，而是“已有功能较多后，如何把结果稳定性和语义一致性收口”。
- 因此下一阶段不建议再无节制扩展新算法入口，优先级应放在主线验证、故障场景尾部收口和正式对比实验固化。

## 三、下一步工作计划

### 1. 先收口验证闭环

- 修复或澄清 `sync_task_statuses` 相关的 `2` 个失败用例，恢复 `pytest` 全绿状态。
- 继续把执行层和任务同步的关键语义固化为测试，尤其是：
  - `RETURN_TO_PATROL` 下的新分配任务保持
  - `ASSIGNED -> REQUEUED` 的触发条件
  - final-approach 失败后的释放原因

### 2. 再压故障场景尾部波动

- 以 `agent_failure + local_mpc + 1200 step` 为固定基线，继续复跑并复核关键 step。
- 优先优化：
  - `regional_recovery_debt`
  - hotspot suppression 放行条件
  - failure 后 surviving `USV` 的热点接管效率

### 3. 收口返航振荡与隐式丢单

- 重点跟踪 `RETURN_TO_PATROL <-> RECOVERY` 长时来回切换问题。
- 核查热点确认任务在 final-approach 失败后的真实释放原因，避免“日志不报错，但语义上已丢单”的情况继续存在。

### 4. 固化正式实验链

- 继续维持并扩展当前自包含实验数据集目录，统一保留配置、README、batch 结果和代表性回放。
- 对规则式主线优先保持以下对比口径：
  - `cost_aware` 作为稳健基线
  - `RHO` 作为 freshness-first 主线
  - `distributed_CBBA` 作为分布式对照

### 5. 审慎推进 RL 主线

- 保留 PPO 训练/评估入口，继续做探索性迭代。
- 在没有稳定超过规则式主线之前，不建议把 PPO 升级为默认主线，只建议作为研究对照和后续优化方向保留。

## 阶段结论

- 从仓库当前内容看，项目已经完成中期应达到的“系统成型、能力成套、实验可复现、问题可定位”的核心目标。
- 当前阶段可以判定为：**中期主体工作已完成，项目进入稳定性收口与性能优化阶段**。
