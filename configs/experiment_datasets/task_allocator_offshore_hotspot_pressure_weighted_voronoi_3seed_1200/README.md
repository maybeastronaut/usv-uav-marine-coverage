# `task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200`

这是当前任务层在 `weighted_voronoi_partition_policy` 新分区基线下的最新正式数据集目录。

## 实验定位

- 标签：`当前主线综合表现`
- 当前判断：这组还能说明 allocator 的相对倾向，但不再是特别干净的算法特征实验。
- 原因：`AEA / RHO / cost_aware` 的差异仍然可见，但绝对结果已经明显受主线公共逻辑变化影响，更适合解释“当前主线下的综合表现”。

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
- `representative_aea_seed_20260325.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧结论。

- `cost_aware`
  - 平均 `coverage_ratio = 0.8613`
  - 平均 `valid_cells = 849.7`
  - 平均 `stale_cells = 518.7`
  - 平均 `confirmed_hotspots = 19.3`
- `AEA`
  - 平均 `coverage_ratio = 0.8691`
  - 平均 `valid_cells = 952.3`
  - 平均 `stale_cells = 416.0`
  - 平均 `confirmed_hotspots = 19.7`
- `RHO`
  - 平均 `coverage_ratio = 0.8597`
  - 平均 `valid_cells = 897.0`
  - 平均 `stale_cells = 471.3`
  - 平均 `confirmed_hotspots = 26.7`

这组当前结果说明：

- `AEA` 现在是更均衡的方案，在 coverage、valid_cells 和 stale_cells 上都优于另外两者。
- `RHO` 仍然最像“热点确认优先”的版本，确认热点数最高，但 blocked 与 freshness 表现不如 `AEA`。
- `cost_aware` 不再像旧结论里那样是当前最均衡方案，而是整体落在 `AEA` 和 `RHO` 之后。

## 代表性回放

- 推荐查看：`representative_aea_seed_20260325.html`
- 原因：这份回放最能代表当前主线下 `AEA` 作为“更均衡版本”的行为特征。
