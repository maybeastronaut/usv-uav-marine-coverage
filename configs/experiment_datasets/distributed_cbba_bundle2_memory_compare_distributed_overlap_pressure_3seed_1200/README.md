# `distributed_cbba_bundle2_memory_compare_distributed_overlap_pressure_3seed_1200`

这是当前 `distributed_CBBA(bundle=2)` 在局部 winner 记忆滞后约束下的正式验证数据集目录。

## 数据集用途

用于在固定：

- 场景：`distributed_overlap_pressure`
- 分区：`weighted_voronoi_partition_policy`
- 分布式任务层：`distributed_cbba_allocator (bundle = 2)`

的前提下，对比：

- `winner_memory_ttl = 0`
- `winner_memory_ttl = 5`
- `winner_memory_ttl = 10`

并观察：

- 局部 winner 记忆滞后是否会改变当前分布式协商结果

## 固定条件

- 场景：`distributed_overlap_pressure`
- 步数：`1200`
- 随机种子：
  - `20260324`
  - `20260325`
  - `20260326`
- 分区固定为：`weighted_voronoi_partition_policy`
- `bundle = 2`
- 同步频率固定为：`1`
- 广播范围不受限

## 目录内容

- `batch.toml`
- `comparison_summary.json`
- `cbba_bundle2_memory0_seed_*_events.jsonl`
- `cbba_bundle2_memory0_seed_*_summary.json`
- `cbba_bundle2_memory5_seed_*_events.jsonl`
- `cbba_bundle2_memory5_seed_*_summary.json`
- `cbba_bundle2_memory10_seed_*_events.jsonl`
- `cbba_bundle2_memory10_seed_*_summary.json`

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/distributed_cbba_bundle2_memory_compare_distributed_overlap_pressure_3seed_1200/batch.toml
```

## 结论

- `memory_ttl = 0 / 5 / 10` 三组均值完全一致：
  - `coverage_ratio = 0.902054`
  - `valid_cells = 929.0`
  - `stale_cells = 439.3`
  - `confirmed_hotspots = 19.0`

这组数据说明：在当前这版 `bundle = 2` 的 `distributed_CBBA` 中，局部 winner 记忆滞后仍然没有进一步改变最终协商结果。
