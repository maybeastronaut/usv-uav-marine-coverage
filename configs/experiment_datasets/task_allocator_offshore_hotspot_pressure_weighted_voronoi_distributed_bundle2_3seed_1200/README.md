# `task_allocator_offshore_hotspot_pressure_weighted_voronoi_distributed_bundle2_3seed_1200`

这是当前“集中式 vs 分布式任务分配”主对比的正式数据集目录。

## 实验定位

- 标签：`当前主线综合表现`
- 当前判断：这组还能看出中心化与分布式任务分配的相对差异，但已经不适合当作纯算法画像。
- 原因：`RHO / cost_aware / distributed_CBBA(bundle=2)` 的相对强弱仍然可见，但这些结果已经明显受当前主线公共执行链影响。

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
- `representative_rho_seed_20260325.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_distributed_bundle2_3seed_1200/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧结论。

- `cost_aware`
  - 平均 `coverage_ratio = 0.8613`
  - 平均 `valid_cells = 849.7`
  - 平均 `stale_cells = 518.7`
  - 平均 `confirmed_hotspots = 19.3`
- `RHO`
  - 平均 `coverage_ratio = 0.8597`
  - 平均 `valid_cells = 897.0`
  - 平均 `stale_cells = 471.3`
  - 平均 `confirmed_hotspots = 26.7`
- `distributed_CBBA(bundle=2)`
  - 平均 `coverage_ratio = 0.8568`
  - 平均 `valid_cells = 837.7`
  - 平均 `stale_cells = 530.7`
  - 平均 `confirmed_hotspots = 20.7`

这组当前结果说明：

- `RHO` 仍然是这组三者里整体最强的一档，尤其在热点确认和 freshness 上优势最明显。
- `distributed_CBBA(bundle=2)` 不是失效，但当前主线下仍然明显落后于 `RHO`，与 `cost_aware` 相比也没有形成稳定优势。
- 这说明当前这版分布式任务分配可以正式参与对比，但还不能替代中心化 `RHO` 主线。

## 代表性回放

- 推荐查看：`representative_rho_seed_20260325.html`
- 原因：这份回放最能体现当前主线下 `RHO` 的确认热点优势，以及它与分布式版本的差异。
