# `usv_planner_return_to_patrol_stress_3seed_1200`

这是当前 `USV` 规划层在 `return_to_patrol_stress` 场景下的最新正式数据集目录。

## 实验定位

- 标签：`待重建基线`
- 当前判断：这组目前不适合继续当作纯 planner 特征说明。
- 原因：这组最容易被 patrol / return / hotspot / blocked 的公共逻辑变化污染，当前差异已经很难稳定归因到 planner 本身。

## 数据集用途

用于在更强调“任务完成后长距离回巡航、跨风险区往返与路径折返”的场景中，对比：

- `astar_path_planner`
- `astar_smoother_path_planner`

并观察 `astar + smoother` 是否能更清楚地体现：

- 更少的 `blocked` 规划
- 更低的规划展开成本
- 更高的热点精检完成数

## 固定条件

- 场景：`return_to_patrol_stress`
- 步数：`1200`
- 随机种子：
  - `20260334`
  - `20260335`
  - `20260336`
- 任务分配固定为：`cost_aware_centralized_allocator`
- `UAV` 规划器固定为：`uav_lawnmower_planner`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 本组最新 `3-seed` 对比的聚合结果与均值
- `astar_seed_*_events.jsonl`
  - `astar_path_planner` 每个 seed 的逐事件日志
- `astar_seed_*_summary.json`
  - `astar_path_planner` 每个 seed 的最终汇总
- `astar_smoother_seed_*_events.jsonl`
  - `astar_smoother_path_planner` 每个 seed 的逐事件日志
- `astar_smoother_seed_*_summary.json`
  - `astar_smoother_path_planner` 每个 seed 的最终汇总
- `representative_astar_seed_20260335.html`
  - 当前主线代码下的代表性回放

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/usv_planner_return_to_patrol_stress_3seed_1200/batch.toml
```

## 结论

以下结论基于当前主线代码于 `2026-04-01` 的重跑结果，不再沿用目录内历史 `comparison_summary.json` 的旧结论。

- `astar_path_planner`
  - 平均 `coverage_ratio = 0.8508`
  - 平均 `valid_cells = 812.3`
  - 平均 `stale_cells = 554.7`
  - 平均 `confirmed_hotspots = 11.0`
- `astar_smoother_path_planner`
  - 平均 `coverage_ratio = 0.8444`
  - 平均 `valid_cells = 764.0`
  - 平均 `stale_cells = 603.0`
  - 平均 `confirmed_hotspots = 15.0`

这组当前结果说明：

- `astar_smoother_path_planner` 现在只保住了“确认热点更多”这一项优势。
- `astar_path_planner` 反而在 coverage、valid_cells、stale_cells、blocked 和 expanded_nodes 上整体更好。
- 因此，当前主线下已经不能再说 `astar + smoother` 在这组 `return_to_patrol_stress` 实验里更优。

## 代表性回放

- 推荐查看：`representative_astar_seed_20260335.html`
- 原因：这份回放最能代表当前主线下 `astar` 在回巡航压力场景里的整体优势。
