# `partition_policy_offshore_hotspot_pressure_3seed_1200`

这是当前分区层在 `offshore_hotspot_pressure` 场景下的最新正式数据集目录。

## 实验定位

- 标签：`待重建基线`
- 当前判断：这组暂时不适合继续当作纯分区特征实验。
- 原因：当前主线下大量公共指标一起漂移，分区差异已经和执行链、热点链、统计链变化耦合在一起，难以把结果稳定归因到 partition policy 本身。

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
- `representative_weighted_voronoi_seed_20260324.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/partition_policy_offshore_hotspot_pressure_3seed_1200/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧结论。

- `baseline_fixed_partition`
  - 平均 `coverage_ratio = 0.8808`
  - 平均 `valid_cells = 849.3`
  - 平均 `stale_cells = 519.0`
  - 平均 `confirmed_hotspots = 13.3`
- `soft_partition_policy`
  - 平均 `coverage_ratio = 0.8527`
  - 平均 `valid_cells = 600.0`
  - 平均 `stale_cells = 768.3`
  - 平均 `confirmed_hotspots = 10.0`
- `weighted_voronoi_partition_policy`
  - 平均 `coverage_ratio = 0.8613`
  - 平均 `valid_cells = 849.7`
  - 平均 `stale_cells = 518.7`
  - 平均 `confirmed_hotspots = 19.3`

这组当前结果说明：

- `weighted_voronoi_partition_policy` 现在最明显的优势，是热点确认数最高，同时 `expanded_nodes` 也最低。
- `baseline_fixed_partition` 反而在当前主线下给出了最高 coverage，而且 `blocked` 行为更稳。
- `soft_partition_policy` 当前整体最弱，只是在少数 seed 上 `blocked` 稍低。
- 因此，当前主线下已经不能再简单说 `weighted_voronoi_partition_policy` 是“最平衡”的统一最优解。

## 代表性回放

- 推荐查看：`representative_weighted_voronoi_seed_20260324.html`
- 原因：这份回放最能体现当前主线下 `weighted_voronoi` 的热点确认倾向，以及它和旧结论的偏移。
