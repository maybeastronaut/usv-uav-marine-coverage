# `task_allocator_offshore_hotspot_pressure_3seed_cooldown`

这是当前任务层第一轮对比在修正 `cost_aware` 不可达任务冷却之后的正式数据集目录。

## 数据集用途

用于在同一个高远海热点压力场景下，对比：

- `basic_task_allocator`
- `cost_aware_centralized_allocator`

并观察“不可达任务冷却”加入后，`cost_aware` 的行为是否更稳定。

## 固定条件

- 场景：`offshore_hotspot_pressure`
- 步数：`1200`
- 随机种子：
  - `20260324`
  - `20260325`
  - `20260326`
- `UAV` 规划器固定为：`uav_lawnmower_planner`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 本组 `3-seed` 对比的聚合结果与均值
- `basic_seed_*_events.jsonl`
  - `basic_task_allocator` 每个 seed 的逐事件日志
- `basic_seed_*_summary.json`
  - `basic_task_allocator` 每个 seed 的最终汇总
- `cost_aware_seed_*_events.jsonl`
  - 修正不可达任务冷却后的 `cost_aware_centralized_allocator` 每个 seed 的逐事件日志
- `cost_aware_seed_*_summary.json`
  - 修正不可达任务冷却后的 `cost_aware_centralized_allocator` 每个 seed 的最终汇总

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_3seed_cooldown/batch.toml
```

## 结论

- `basic_task_allocator` 仍然更偏向整体覆盖率和 `valid_cells` 维持。
- 修正不可达任务冷却后，`cost_aware_centralized_allocator` 的 `astar_blocked_calls` 明显下降，并且在这组 `3-seed` 对比里能确认更多热点。
- 当前这组数据反映的是一种取舍：
  - `basic` 更偏整体 freshness / coverage
  - `cost_aware` 更偏热点确认效率，同时整体覆盖略弱
