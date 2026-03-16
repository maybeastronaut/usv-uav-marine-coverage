# `usv_planner_return_to_patrol_stress_3seed_1200`

这是当前 `USV` 规划层在 `return_to_patrol_stress` 场景下的最新正式数据集目录。

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

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/usv_planner_return_to_patrol_stress_3seed_1200/batch.toml
```

## 结论

- `astar_path_planner` 在这组最新 `3-seed` 对比里更偏向整体 coverage 与 freshness 维持：
  - 平均 `valid_cells` 更高
  - 平均 `stale_cells` 更低
- `astar_smoother_path_planner` 在这组最新结果里更能体现“任务后回巡航链”的优势：
  - 平均确认热点更多
  - 平均 `blocked_calls` 更少
  - 平均 `expanded_nodes` 更低
- 这组最新场景说明：
  - `return_to_patrol_stress` 适合放大 `astar + smoother` 在“热点完成与回巡航接入”上的优势
  - 但它不会自动把整体 freshness 一起拉高
