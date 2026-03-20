# 项目讨论说明文档

用途：

- 只记录后续阶段的决策方向
- 只记录继续开发时需要特别注意的点
- 不再重复记录当前已经完成的系统事实

维护约定：

- 当前已实现结构、当前默认参数、当前主流程、当前已完成能力统一写入 `current_system_flow.md`
- 本文件只保留“后续要怎么做”以及“后续开发时要注意什么”
- 某项一旦已经稳定落地并成为当前事实，应从本文件移出，并覆盖写入 `current_system_flow.md`

## 1. 后续决策方向

### 1.1 总体原则

- 当前第一阶段基础版闭环系统已经形成，可作为后续研究与对比实验的 baseline
- 后续升级优先沿“任务层 -> 规划层 -> 执行层/控制器层 -> 更真实物理模型”推进
- 除非有明确研究目标，否则不要轻易破坏当前 baseline 的基本行为与可复现实验能力

### 1.2 明确升级顺序

- 当前分区层主线已经收口为：固定 `weighted_voronoi_partition_policy`
- 当前中心化任务层主线已经收口为：
  - `cost_aware_centralized_allocator`：当前最均衡、最稳的基线
  - `rho_task_allocator`：当前最有潜力的高级任务层，已经在 `valid_cells / stale_cells / blocked` 上体现出明显优势
  - `aoi_energy_auction_allocator`：当前仍保留用于对照，但不是下一步主推方向
- 原因：在 `weighted_voronoi` 新分区基线上，任务层差异终于被真正放大，结论已经比早期 `soft_partition` 阶段清楚很多
- 当前最值得推进的下一步不再是继续微调 `AEA` 常量，也不再是继续堆新的中心化任务层，而是：
  - 在固定 `weighted_voronoi_partition_policy + uav_lawnmower_planner + astar_path_planner` 的前提下，引入第一版**分布式任务分配算法**
  - 形成“中心化稳基线 -> 中心化高级算法 -> 分布式任务分配算法”的正式对照链
- 当前第一版 `distributed_cbba_allocator` 已经落地：
  - 保持 `uav_resupply` 继续走现有专门逻辑
  - 当前已支持 `bundle = 1/2`，其中 `bundle = 2` 采用贪心双任务 bundle，用于放大分布式协商与通信约束对最终分配结果的影响
  - 当前通信采用简化同步 winner 轮次，不追求复杂时延/丢包建模
  - 当前已实现第一步通信约束：可通过 `distributed_sync_interval_steps` 把全局同步改成“每隔 N 步同步一次”
  - 当前已实现第二步通信约束：可通过 `distributed_broadcast_range_m` 限制 `USV` 间广播范围，并按通信连通簇拆分本地 winner 协商
  - 当前已实现第三步通信约束：可通过 `distributed_winner_memory_ttl_steps` 让各 `USV` 在若干步内继续沿用本地 winner 记忆，从而模拟局部 winner 记忆滞后与信息过期
- 由于 `offshore_hotspot_pressure` 下的 `sync interval / broadcast range` 首轮三 seed 对比还没有真正拉开分布式行为，当前已新增 `distributed_overlap_pressure` 作为下一阶段主场景，用于放大 weighted Voronoi 下的多艇候选重叠、跨区重分配与局部通信约束影响
- 当前与分布式任务分配直接相关的正式数据集已经补齐：
  - `task_allocator_offshore_hotspot_pressure_weighted_voronoi_distributed_bundle2_3seed_1200`
  - `distributed_cbba_bundle_compare_distributed_overlap_pressure_3seed_1200`
  - `distributed_cbba_bundle2_memory_compare_distributed_overlap_pressure_3seed_1200`
- 当前分布式任务分配的最新正式结果已经比较清楚：
  - 在 `offshore_hotspot_pressure / weighted_voronoi / 3-seed / 1200-step` 下，`distributed_CBBA(bundle=2)` 已经能稳定进入正式对比，但整体指标仍弱于 `cost_aware / RHO`
  - 这说明当前分布式主线已经从“能不能跑通”推进到“能不能缩小与中心化方案差距”
- 当前分布式内部机制的最新判断也已经收口：
  - `bundle = 1 -> bundle = 2` 是有效结构升级，因为它已经能改变分布式 `CBBA-lite` 的最终行为
  - 但 `winner_memory_ttl = 0 / 5 / 10` 在 `bundle = 2` 版本下仍未改变最终指标
- 因而后续更合理的算法实验顺序应改成：
  - 先固定 `weighted_voronoi_partition_policy`
  - 再把“中心化 vs 分布式任务分配”作为正式主线持续补场景
  - 只有在确有必要时，再继续深入分布式内部通信机制
  - 再决定是否需要把 `AEA` 继续保留在主实验链里
- `hybrid_astar_path_planner` 继续保留，但暂时仍降级为探索型 planner；当前主线重点不在这里

### 1.3 任务层升级方向

- 当前任务层已经具备：
  - 基础责任分区 + 启发式分配
  - 第一版 `cost-aware centralized allocator`
  - 第一版 `aoi_energy_auction_allocator`
  - 第一版 `rho_task_allocator`
- 后续可升级方向包括：
  - `RHO` 的窗口长度、延迟惩罚和 AoI 回报权重调参
  - 第一版分布式 `CBBA-lite` / distributed auction
  - 分布式任务分配下的 winner 冲突、收敛步数和重分配开销建模
  - 后续再决定是否保留 `AEA` 作为长期主线
- 后续若做算法比较，应优先保留：
  - `cost_aware` 作为中心化稳基线
  - `RHO` 作为中心化高级基线
  - 新增分布式算法作为下一档主线

### 1.4 规划层升级方向

- 当前规划层已经具备：
  - `UAV` 割草机搜索
  - `UAV multi-region coverage planner`
  - `UAV persistent multi-region coverage planner`
  - `USV` 基础非完整约束 `A*`
  - `USV A* + smoother`
  - `USV hybrid A* + smoother`
  - 船体安全余量
  - 局部巡航段接入
- 后续可升级方向包括：
  - 更细致风险代价模型
  - 更稳定的回巡航接入策略
  - 更高级 `USV` 规划算法
  - 更复杂 `UAV` 搜索与会合规划
  - `UAV persistent multi-region coverage planner` 的区域权重与热点感知策略
  - `UAV persistent multi-region coverage planner` 的双机协同分区与区域内未完成覆盖量建模
  - `UAV persistent multi-region coverage planner` 的区域切换收益判定，避免只看 freshness debt 而忽略全局周期性覆盖
  - `USV A* + smoother` 的平滑容差、路径长度收益与 `blocked` 开销权衡
  - `USV hybrid A* + smoother` 的状态空间收敛与 fallback 使用边界，避免在全局接管时拖垮仿真性能

### 1.5 执行层与控制器升级方向

- 当前执行层已经不再只有第一阶段基础路径跟踪，还新增了第一版 `local_mpc_execution`
- 当前对这条线的阶段性判断已经更明确：
  - 第一版局部 `MPC` 如果“每一步都强制接管”，会把 `USV` 拉成低速原地抽动
  - 因此当前正确做法不是全时接管，而是“按需接管”
  - 也就是：只有局部风险真正出现时才启用 `MPC`，平时退回原路径跟踪
- 当前这一版已经按上述方向修正，后续若继续加强，应优先保持这个边界，不要再回到“全时接管”的执行策略
- 若后续引入更真实物理模型，则执行层必须同步升级
- 后续可考虑：
  - 在 `local_mpc_execution` 基础上继续提高 `USV-USV` 避碰稳定性
  - 事件触发的局部重规划，用于突然出现的海浪区/危险区
  - 更贴近船舶/飞行器动力学的控制输入约束
  - 更自然的局部避障与恢复动作
- 后续闭环层级建议保持：
  - 任务层决定“做什么”
  - 规划层决定“怎么走”
  - 执行层/控制器决定“能否按真实动力学走出来”

### 1.6 物理模型升级方向

- 当前系统仍属于第一阶段基础动力学基线
- 后续若继续提升真实性，可逐步引入：
  - 更强的非完整约束
  - 船舶惯性与侧滑
  - 海流、风扰动
  - 更真实能耗模型
- 物理模型升级后，应同步检查：
  - 规划层是否仍然与执行层一致
  - 控制器是否仍能稳定跟踪规划路径
  - 现有实验结果是否仍具有可比性

## 2. 后续开发注意事项

### 2.1 文档维护

- 修改当前行为、系统事实、默认参数时，更新 `current_system_flow.md`
- 修改后续路线、设计判断、升级方向时，更新本文件

### 2.2 基线保护

- 后续实验型改动应尽量与当前 baseline 解耦
- 若引入更高级算法，优先保留当前基础算法实现，方便做对比
- 若引入更真实控制器或动力学，优先避免直接覆盖现有第一阶段闭环实现

### 2.3 分层约束

- 任务层升级不要把路径与执行细节重新揉回任务层
- 规划层升级不要把恢复逻辑或状态机逻辑重新揉回 planner
- 执行层升级不要代替全局规划层承担完整路径搜索职责
- 反馈/恢复机制应继续与规划算法保持清晰边界

### 2.4 实验可比性

- 以后每次升级任务层、规划层、执行层或物理模型时，都应说明：
  - 升级属于哪一层
  - 相对当前 baseline 改进了什么
  - 是否影响已有实验的可比性
- `UAV` planner 对比时，优先固定任务层为 `cost_aware_centralized_allocator`，不要再把任务层差异和 `UAV` 搜索差异混在同一轮结论里
- `USV` planner 对比时，优先固定任务层为 `cost_aware_centralized_allocator`、固定 `UAV` 为 `uav_lawnmower_planner`，并优先使用 `planner_path_stress` 这类“中等热点压力 + 低近海任务噪声”的路径对比场景，而不是直接用热点极高压场景混入过强任务层扰动
- 若要进一步放大 `astar + smoother` 这类路径形状优化算法的优势，优先使用 `return_to_patrol_stress` 这类“中等热点压力 + 更低近海噪声 + 更强调任务后长距离回巡航”的场景，而不是继续单纯提高热点数量
- 当前最新三 seed（`20260334 / 20260335 / 20260336`）在 `return_to_patrol_stress` 下的正式结果表明：
  - `astar_smoother_path_planner` 更容易完成热点精检，平均 `blocked_calls` 与 `expanded_nodes` 也更低
  - 但 `astar_path_planner` 仍更偏向整体 freshness 维持，平均 `valid_cells` 更高、`stale_cells` 更低
  - 因此当前主线判断应保持为：
    - `astar + smoother` 适合作为“任务后回巡航链”优化方向继续收
    - `astar` 仍是“整体 coverage / freshness 更稳”的强 baseline
- 当前已知现象是：即使第二版 persistent planner 加入区域承诺、事件触发重排、AOI 去冲突和 sweep 端点接入，它在 `offshore_hotspot_pressure` 下仍可能略弱于固定 lawnmower；后续排查重点应放在“双机协同”和“区域内未完成覆盖量”而不是继续增加单机即时 freshness 贪心
- 当前对 `AEA` 的定位已经收口为：保留作中心化 auction-style 对照，但不再作为近期主推方向；后续若回到这条线，应优先做结构升级而不是继续微调局部权重
- 后续新增算法时，优先复用当前已经落地的：
  - 统一实验配置层
  - 场景目录
  - batch 运行入口
  - 正式实验数据集目录
- 若某轮实验被认定为“正式合格对比数据集”，应将其关键配置、代表性日志与汇总结果同步固化到 `configs/experiment_datasets/`
- `outputs/` 仍保留默认运行输出与复现实验落盘职责，不应在文档中被描述成“已不再承载正式数据链路”
- 后续新增数据集时，优先保持统一结构：
  - `batch.toml`
  - `README.md`
  - `comparison_summary.json`
  - 每个 seed 的 `events.jsonl`
  - 每个 seed 的 `summary.json`
  - 必要时附代表性 `HTML`
- 为避免 batch 复跑时把旧结果与新结果混在同一目录，当前 batch 运行默认应落到带时间戳的新输出目录；若后续要做“固定目录就地更新”，必须显式设计清理或归档机制后再引入
