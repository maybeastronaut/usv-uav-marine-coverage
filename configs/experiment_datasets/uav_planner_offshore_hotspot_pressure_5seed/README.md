# `uav_planner_offshore_hotspot_pressure_5seed`

这是当前 `UAV` 规划层对比的正式数据集目录。

它固定任务层为：

- `cost_aware_centralized_allocator`

只对比两套 `UAV` 搜索规划：

- `uav_lawnmower_planner`
- `uav_persistent_multi_region_coverage_planner`

## 固定条件

- 场景：`offshore_hotspot_pressure`
- 步数：`1200`
- 随机种子：
  - `20260314`
  - `20260315`
  - `20260316`
  - `20260317`
  - `20260319`

## 目录内容

- `batch.toml`
  - 当前数据集的可复现实验配置
- `comparison_summary.json`
  - 这组 `5-seed` 的聚合对比结果
- `cost_aware_lawnmower_seed_*_events.jsonl`
- `cost_aware_lawnmower_seed_*_summary.json`
- `cost_aware_persistent_seed_*_events.jsonl`
- `cost_aware_persistent_seed_*_summary.json`
- 必要时附代表性 `HTML`

## 复现实验

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/uav_planner_offshore_hotspot_pressure_5seed/batch.toml
```
