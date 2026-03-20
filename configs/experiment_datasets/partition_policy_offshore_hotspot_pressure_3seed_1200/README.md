# `partition_policy_offshore_hotspot_pressure_3seed_1200`

这是当前分区层在 `offshore_hotspot_pressure` 场景下的最新正式数据集目录。

## 数据集用途

用于在固定任务层与规划层条件下，对比：

- `baseline_fixed_partition`
- `soft_partition_policy`
- `weighted_voronoi_partition_policy`

并观察新的加权维诺图分区是否能在不明显恶化规划代价的前提下，同时兼顾：

- 热点确认数
- 整体 freshness
- blocked 行为

## 固定条件

- 场景：`offshore_hotspot_pressure`
- 步数：`1200`
- 随机种子：
  - `20260324`
  - `20260325`
  - `20260326`
- 任务分配固定为：`cost_aware_centralized_allocator`
- `UAV` 规划器固定为：`uav_lawnmower_planner`
- `USV` 规划器固定为：`astar_path_planner`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 本组最新 `3-seed` 对比的聚合结果与均值
- `baseline_partition_seed_*_events.jsonl`
  - 硬分区每个 seed 的逐事件日志
- `baseline_partition_seed_*_summary.json`
  - 硬分区每个 seed 的最终汇总
- `soft_partition_seed_*_events.jsonl`
  - 软分区每个 seed 的逐事件日志
- `soft_partition_seed_*_summary.json`
  - 软分区每个 seed 的最终汇总
- `weighted_voronoi_seed_*_events.jsonl`
  - 加权维诺图分区每个 seed 的逐事件日志
- `weighted_voronoi_seed_*_summary.json`
  - 加权维诺图分区每个 seed 的最终汇总

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/partition_policy_offshore_hotspot_pressure_3seed_1200/batch.toml
```

## 结论

- `baseline_fixed_partition`
  - 更偏热点确认
  - 但 `astar_blocked_calls` 和规划代价明显更高
- `soft_partition_policy`
  - 更偏整体 freshness 与稳定性
  - 平均 `coverage_ratio` 更高
  - 平均 `blocked` 更少
- `weighted_voronoi_partition_policy`
  - 当前最平衡
  - 平均 `confirmed_hotspots` 回升到 `24.0`
  - 平均 `valid_cells` 最高
  - 平均 `stale_cells` 最低
  - 平均 `astar_blocked_calls` 最低

这组最新正式数据说明：当前分区层主线已经可以从规则式 `soft_partition_policy` 进一步推进到 `weighted_voronoi_partition_policy`。
