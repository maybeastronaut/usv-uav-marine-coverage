# `distributed_cbba_bundle_compare_distributed_overlap_pressure_3seed_1200`

这是当前分布式 `CBBA-lite` 内部结构升级的正式验证数据集目录。

## 实验定位

- 标签：`当前主线综合表现`
- 当前判断：这组还能说明相对倾向，但已经不是特别干净的算法特征实验。
- 原因：当前主线下仍能看出 `bundle=1` 与 `bundle=2` 的行为差异，但这些差异已经明显叠加了公共执行链和热点链变化，不宜再直接当成纯 `bundle` 长度效应。

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
- `representative_cbba_bundle2_seed_20260326.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/distributed_cbba_bundle_compare_distributed_overlap_pressure_3seed_1200/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧数值。

- `bundle = 1`
  - 平均 `coverage_ratio = 0.8652`
  - 平均 `valid_cells = 903.3`
  - 平均 `stale_cells = 465.0`
  - 平均 `confirmed_hotspots = 26.0`
- `bundle = 2`
  - 平均 `coverage_ratio = 0.8499`
  - 平均 `valid_cells = 817.7`
  - 平均 `stale_cells = 550.7`
  - 平均 `confirmed_hotspots = 30.7`

这组当前结果说明：

- `bundle = 2` 依然会显著改变分布式 `CBBA-lite` 的最终行为，这条“内部结构会影响结果”的结论仍然成立。
- 但方向已经变了：当前主线下，`bundle = 2` 更像“更激进追热点确认”的版本，而不是整体更优版本。
- `bundle = 2` 的热点确认数更高，但整体 coverage、更整体 freshness 和 stale 表现都弱于 `bundle = 1`。

## 代表性回放

- 推荐查看：`representative_cbba_bundle2_seed_20260326.html`
- 原因：这个 seed 很典型地体现了当前主线下 `bundle = 2` “确认热点更多，但整体 freshness 更差”的特征。
