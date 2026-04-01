# `usv_planner_offshore_hotspot_pressure_3seed_800`

这是当前 `USV` 规划层第二轮对比的正式数据集目录。

## 实验定位

- 标签：`待重建基线`
- 当前判断：这组暂时不适合继续当作纯 planner 特征实验。
- 原因：当前主线下不仅 planner 相对关系变了，很多公共指标也一起翻转，说明结果已经不只是路径规划器本身的差异。

## 数据集用途

用于在同一个高远海热点压力场景下，对比：

- `astar_path_planner`
- `astar_smoother_path_planner`

并观察轻量平滑后处理是否能在不完全破坏仿真成本的前提下，提升 `USV` 热点精检表现。

## 固定条件

- 场景：`offshore_hotspot_pressure`
- 步数：`800`
- 随机种子：
  - `20260324`
  - `20260325`
  - `20260326`
- 任务分配固定为：`cost_aware_centralized_allocator`
- `UAV` 规划器固定为：`uav_lawnmower_planner`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 本组 `3-seed` 对比的聚合结果与均值
- `astar_seed_*_events.jsonl`
  - `astar_path_planner` 每个 seed 的逐事件日志
- `astar_seed_*_summary.json`
  - `astar_path_planner` 每个 seed 的最终汇总
- `astar_smoother_seed_*_events.jsonl`
  - `astar_smoother_path_planner` 每个 seed 的逐事件日志
- `astar_smoother_seed_*_summary.json`
  - `astar_smoother_path_planner` 每个 seed 的最终汇总
- `representative_astar_smoother_seed_20260325.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/usv_planner_offshore_hotspot_pressure_3seed_800/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧结论。

- `astar_path_planner`
  - 平均 `coverage_ratio = 0.8594`
  - 平均 `valid_cells = 982.7`
  - 平均 `stale_cells = 385.7`
  - 平均 `confirmed_hotspots = 9.7`
- `astar_smoother_path_planner`
  - 平均 `coverage_ratio = 0.8470`
  - 平均 `valid_cells = 1035.0`
  - 平均 `stale_cells = 333.3`
  - 平均 `confirmed_hotspots = 14.3`

这组当前结果说明：

- `astar_smoother_path_planner` 现在不只是确认热点更多，它在 `valid_cells`、`stale_cells` 和 `blocked` 上也更好。
- `astar_path_planner` 只保住了 coverage 略高这一项优势。
- 因此，当前主线下更接近“`astar + smoother` 整体更强，但 coverage 略低”的关系，而不是旧 README 里的那种 freshness 优势分工。

## 代表性回放

- 推荐查看：`representative_astar_smoother_seed_20260325.html`
- 原因：这份回放最能代表当前主线下 `astar + smoother` 在热点确认和 freshness 上同时占优的特征。
