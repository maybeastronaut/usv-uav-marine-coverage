# `task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200`

这是当前任务层在 `weighted_voronoi_partition_policy` 新分区基线下的最新正式数据集目录。

## 数据集用途

用于在更强的新候选空间上，对比：

- `cost_aware_centralized_allocator`
- `aoi_energy_auction_allocator`
- `rho_task_allocator`

并观察：

- 哪个任务层算法更能利用 weighted Voronoi 打开的竞争空间
- 热点确认、freshness 与 blocked 行为会出现怎样的新取舍

## 固定条件

- 场景：`offshore_hotspot_pressure`
- 步数：`1200`
- 随机种子：
  - `20260324`
  - `20260325`
  - `20260326`
- 分区固定为：`weighted_voronoi_partition_policy`
- `UAV` 规划器固定为：`uav_lawnmower_planner`
- `USV` 规划器固定为：`astar_path_planner`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 本组最新 `3-seed` 对比的聚合结果与均值
- `cost_aware_seed_*_events.jsonl`
  - `cost_aware` 每个 seed 的逐事件日志
- `cost_aware_seed_*_summary.json`
  - `cost_aware` 每个 seed 的最终汇总
- `aea_seed_*_events.jsonl`
  - `AEA` 每个 seed 的逐事件日志
- `aea_seed_*_summary.json`
  - `AEA` 每个 seed 的最终汇总
- `rho_seed_*_events.jsonl`
  - `RHO` 每个 seed 的逐事件日志
- `rho_seed_*_summary.json`
  - `RHO` 每个 seed 的最终汇总

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200/batch.toml
```

## 结论

- `cost_aware`
  - 当前最均衡
  - 平均 `confirmed_hotspots = 24.0`
  - 平均 `valid_cells = 1013.3`
  - 平均 `stale_cells = 355.0`
  - 平均 `astar_blocked_calls = 2.3`
- `AEA`
  - 当前更保守
  - 平均 `blocked` 最低之一，调用数也最低
  - 但平均 `confirmed_hotspots`、`valid_cells` 都弱于 `cost_aware`
- `RHO`
  - 当前最像 freshness-first 版本
  - 平均 `valid_cells` 最高
  - 平均 `stale_cells` 最低
  - 平均 `confirmed_hotspots` 也略高于 `cost_aware`
  - `astar_blocked_calls = 0.0`

这组正式数据说明：在 weighted Voronoi 新分区基线下，`RHO` 终于真正与 `cost_aware` 拉开了差异，而 `AEA` 暂时没有成为新的最优任务层。
