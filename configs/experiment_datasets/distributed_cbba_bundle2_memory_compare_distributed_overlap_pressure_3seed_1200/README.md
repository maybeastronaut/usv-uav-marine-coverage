# `distributed_cbba_bundle2_memory_compare_distributed_overlap_pressure_3seed_1200`

这是当前 `distributed_CBBA(bundle=2)` 在局部 winner 记忆滞后约束下的正式验证数据集目录。

## 实验定位

- 标签：`算法特征实验`
- 当前判断：这组仍然适合用来说明算法特点。
- 原因：在当前主线代码下，`memory_ttl = 0 / 5 / 10` 的相对关系仍然完全一致，说明这组实验还能稳定刻画 `bundle=2` 对 winner memory TTL 不敏感这一特征。

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
- `representative_cbba_bundle2_memory0_seed_20260324.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/distributed_cbba_bundle2_memory_compare_distributed_overlap_pressure_3seed_1200/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧数值。

- `memory_ttl = 0 / 5 / 10` 三组结果在当前主线下仍然完全一致。
- 三组的当前均值一致为：
  - `coverage_ratio = 0.849895`
  - `valid_cells = 817.7`
  - `stale_cells = 550.7`
  - `confirmed_hotspots = 30.7`
  - `active_hotspot_cells = 14.0`
- 这说明：当前这版 `bundle = 2` 的 `distributed_CBBA` 仍然对局部 winner 记忆滞后不敏感，`winner_memory_ttl` 不会改变最终协商结果。
- 需要注意的是，虽然相对结论没变，但绝对性能已经明显偏离本目录里的历史基线。

## 代表性回放

- 推荐查看：`representative_cbba_bundle2_memory0_seed_20260324.html`
- 原因：`memory0 / 5 / 10` 在当前主线下结果完全一致，因此保留一份 `memory0` 回放即可代表这组实验。
