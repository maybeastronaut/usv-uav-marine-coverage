# `distributed_cbba_bundle_compare_distributed_overlap_pressure_3seed_1200`

这是当前分布式 `CBBA-lite` 内部结构升级的正式验证数据集目录。

## 数据集用途

用于在固定：

- 场景：`distributed_overlap_pressure`
- 分区：`weighted_voronoi_partition_policy`
- 规划：`uav_lawnmower_planner + astar_path_planner`

的前提下，对比分布式 `CBBA` 的：

- `bundle = 1`
- `bundle = 2`

并观察：

- greedy 双任务 bundle 是否真的改变分布式协商结果
- 分布式市场是否不再对内部结构完全不敏感

## 固定条件

- 场景：`distributed_overlap_pressure`
- 步数：`1200`
- 随机种子：
  - `20260324`
  - `20260325`
  - `20260326`
- 分区固定为：`weighted_voronoi_partition_policy`
- 通信保持理想同步：`sync = 1`
- 广播范围不受限

## 目录内容

- `batch.toml`
- `comparison_summary.json`
- `cbba_bundle1_seed_*_events.jsonl`
- `cbba_bundle1_seed_*_summary.json`
- `cbba_bundle2_seed_*_events.jsonl`
- `cbba_bundle2_seed_*_summary.json`

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/distributed_cbba_bundle_compare_distributed_overlap_pressure_3seed_1200/batch.toml
```

## 结论

- `bundle = 1`
  - 平均 `coverage_ratio = 0.8996`
  - 平均 `valid_cells = 924.3`
  - 平均 `stale_cells = 444.0`
  - 平均 `confirmed_hotspots = 23.0`
- `bundle = 2`
  - 平均 `coverage_ratio = 0.9021`
  - 平均 `valid_cells = 929.0`
  - 平均 `stale_cells = 439.3`
  - 平均 `confirmed_hotspots = 19.0`

这组数据说明：`bundle = 2` 已经能够改变分布式 `CBBA-lite` 的最终行为，分布式协商不再像早期单任务版本那样对内部结构完全不敏感。
