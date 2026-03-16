# `usv_planner_offshore_hotspot_pressure_3seed_800`

这是当前 `USV` 规划层第二轮对比的正式数据集目录。

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

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/usv_planner_offshore_hotspot_pressure_3seed_800/batch.toml
```

## 结论

- `astar_path_planner` 更偏向整体 freshness 维持，`valid_cells` 更高，`stale_cells` 更低，而且规划成本明显更轻。
- `astar_smoother_path_planner` 在这组 `3-seed` 对比里平均确认了更多热点，也取得了更高的平均 `coverage_ratio`。
- 当前这组数据反映的是另一种取舍：
  - `astar` 更轻、更稳、更偏整体信息维护
  - `astar + smoother` 更偏热点追踪与任务完成，但规划调用数和展开节点数显著更高
