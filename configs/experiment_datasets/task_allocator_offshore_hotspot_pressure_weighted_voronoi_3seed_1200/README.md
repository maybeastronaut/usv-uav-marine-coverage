# task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200

第一组正式任务分配算法对比。

目标问题：

- 在相同场景、相同分区、相同路径规划和相同执行层下
- `cost-aware / AEA / RHO`
- 谁更适合处理远海热点压力场景

固定条件：

- 场景：`offshore_hotspot_pressure`
- 分区：`weighted_voronoi_partition_policy`
- USV 路径规划：`astar_smoother_path_planner`
- UAV 搜索：`uav_lawnmower_planner`
- 执行策略：`local_mpc_execution`
- 不注入智能体受损事件
- `1200 step`
- `3` 个 seed：`20260324 / 20260325 / 20260326`

本目录文件：

- `offshore_hotspot_pressure_cost_aware_weighted_voronoi.toml`
- `offshore_hotspot_pressure_aoi_energy_weighted_voronoi.toml`
- `offshore_hotspot_pressure_rho_weighted_voronoi.toml`
- `batch.toml`
