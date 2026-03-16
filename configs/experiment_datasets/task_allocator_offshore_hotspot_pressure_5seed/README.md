# `task_allocator_offshore_hotspot_pressure_5seed`

这是当前任务层算法对比的**正式合格数据集**目录。

它不仅包含 batch 配置，还包含本轮已经确认可复用的日志与汇总结果。

## 数据集用途

用于对比：

- `basic_task_allocator`
- `cost_aware_centralized_allocator`

在同一高远海热点压力场景下的表现差异。

## 固定条件

- 场景：`offshore_hotspot_pressure`
- 步数：`1200`
- 随机种子：
  - `20260314`
  - `20260315`
  - `20260316`
  - `20260317`
  - `20260319`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 这组 `5-seed` 对比的聚合结果
- `basic_seed_*_events.jsonl`
  - `basic_task_allocator` 每个 seed 的逐事件日志
- `basic_seed_*_summary.json`
  - `basic_task_allocator` 每个 seed 的最终汇总
- `cost_aware_seed_*_events.jsonl`
  - `cost_aware_centralized_allocator` 每个 seed 的逐事件日志
- `cost_aware_seed_*_summary.json`
  - `cost_aware_centralized_allocator` 每个 seed 的最终汇总

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_5seed/batch.toml
```

## 说明

- 本目录存放的是“当前已确认可复用”的正式对比数据与日志
- 原始运行产物仍可继续放在 `outputs/` 下
- 若后续数据集更新，应在保持本目录结构稳定的前提下覆盖或新增版本说明
