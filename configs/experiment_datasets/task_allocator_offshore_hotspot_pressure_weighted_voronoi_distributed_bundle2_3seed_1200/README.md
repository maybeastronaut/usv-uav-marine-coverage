# `task_allocator_offshore_hotspot_pressure_weighted_voronoi_distributed_bundle2_3seed_1200`

这是当前“集中式 vs 分布式任务分配”主对比的正式数据集目录。

## 数据集用途

用于在固定更强的新分区基线后，对比：

- `cost_aware_centralized_allocator`
- `rho_task_allocator`
- `distributed_cbba_allocator (bundle = 2)`

并观察：

- 中心化任务分配与分布式任务分配在同场景、同分区、同规划条件下的差异
- 分布式任务分配是否能接近中心化基线
- freshness、热点确认与 blocked 行为会出现怎样的取舍

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
- `rho_seed_*_events.jsonl`
  - `RHO` 每个 seed 的逐事件日志
- `rho_seed_*_summary.json`
  - `RHO` 每个 seed 的最终汇总
- `distributed_cbba_bundle2_seed_*_events.jsonl`
  - `distributed_CBBA(bundle=2)` 每个 seed 的逐事件日志
- `distributed_cbba_bundle2_seed_*_summary.json`
  - `distributed_CBBA(bundle=2)` 每个 seed 的最终汇总

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_distributed_bundle2_3seed_1200/batch.toml
```

## 结论

- `cost_aware`
  - 当前最均衡
  - 平均 `confirmed_hotspots = 24.0`
  - 平均 `valid_cells = 1013.3`
  - 平均 `stale_cells = 355.0`
  - 平均 `astar_blocked_calls = 2.3`
- `RHO`
  - 当前仍是最强的 freshness-first 中心化方案
  - 平均 `valid_cells = 1063.3` 最高
  - 平均 `stale_cells = 305.0` 最低
  - 平均 `confirmed_hotspots = 24.3`
  - 平均 `astar_blocked_calls = 0.0`
- `distributed_CBBA(bundle=2)`
  - 当前已经能形成有效分布式协商结果
  - 平均 `astar_blocked_calls = 0.7` 较低，总调用也更少
  - 但平均 `confirmed_hotspots = 18.3`
  - 平均 `valid_cells = 991.3`
  - 平均 `stale_cells = 377.0`

这组正式数据说明：当前这版分布式任务分配已经可以和中心化任务层进行正式对比，但在该场景下整体表现仍落后于 `cost_aware / RHO` 两种中心化方案。
