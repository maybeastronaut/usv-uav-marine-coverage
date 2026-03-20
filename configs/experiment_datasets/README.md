# 实验数据集目录

本目录用于放置**可复用、可对比、能突出算法特点**的实验数据集。

这里的“数据集”在当前项目里主要指：

- 固定场景
- 固定随机种子集合
- 固定步数
- 固定统计口径

这样后续切换不同算法时，只替换算法配置，不替换数据集定义，就能保证实验可比性。

## 当前约定

- 一个数据集配置文件只表达一种实验目标
- 文件名优先体现：
  - 对比层级
  - 场景名
  - 种子规模
- 当前阶段优先服务：
  - 任务层算法对比
  - 后续可扩展到 `UAV` 规划层与 `USV` 规划层对比

## 当前已落地的数据集

- `usv_planner_offshore_hotspot_pressure_3seed_800/`
  - 当前正式 `USV` 规划层对比数据集目录
  - 已包含：
    - `batch.toml`
    - 聚合对比结果
    - 每个 seed 的 `events.jsonl`
    - 每个 seed 的 `summary.json`
  - 适合观察 `astar` 与 `astar + smoother` 在高远海热点压力下的效果/成本取舍
- `usv_planner_return_to_patrol_stress_3seed_1200/`
  - 当前正式 `USV` 回巡航接入压力对比数据集目录
  - 已包含：
    - `batch.toml`
    - 聚合对比结果
    - 每个 seed 的 `events.jsonl`
    - 每个 seed 的 `summary.json`
  - 适合观察 `astar` 与 `astar + smoother` 在任务后长距离回巡航场景下的热点完成数、blocked 行为与规划成本差异
- `partition_policy_offshore_hotspot_pressure_3seed_1200/`
  - 当前正式分区层对比数据集目录
  - 已包含：
    - `batch.toml`
    - 聚合对比结果
    - 每个 seed 的 `events.jsonl`
    - 每个 seed 的 `summary.json`
  - 适合观察 `baseline_fixed / soft / weighted_voronoi` 在高远海热点压力下的 freshness、热点确认与 blocked 取舍
- `task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200/`
  - 当前正式任务层对比数据集目录
  - 已包含：
    - `batch.toml`
    - 聚合对比结果
    - 每个 seed 的 `events.jsonl`
    - 每个 seed 的 `summary.json`
  - 适合观察 `cost_aware / AEA / RHO` 在 `weighted_voronoi` 新分区基线上是否真正拉开差异

## 使用方式

当前这类数据集目录都应至少包含一个 `batch.toml`，可直接运行：

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/<dataset_name>/batch.toml
```

输出结果建议统一写到：

- `outputs/experiments/<layer>/<dataset_name>/`

这样可以把不同算法、不同数据集的结果长期整理在稳定目录下。
