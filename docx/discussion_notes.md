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

- 当前下一步优先做：在固定 `uav_lawnmower_planner + astar_path_planner` 基线下，正式比较三套任务层算法：
  - `basic_task_allocator`
  - `cost_aware_centralized_allocator`
  - `aoi_energy_auction_allocator`
  - `rho_task_allocator`
- 原因：当前 `USV` planner 主线已经形成可解释分场景结论，第三层更需要引入一个比 `cost_aware` 更高级、但仍保持中心化可解释的任务层算法
- 当前 `aoi_energy_auction_allocator` 的第一版定位应保持为：
  - 中心化 auction-style allocator，而不是完整分布式协商
  - 只 auction `baseline_service` 和 `hotspot_confirmation`
  - `uav_resupply` 继续保留现有专门逻辑
- 当前已补充一个更适合作为第三层主线对照的 `rho_task_allocator`：
  - 中心化 rolling-horizon allocator
  - 继续固定 `uav_resupply` 专门逻辑
  - 在严格责任区内用“任务基础价值 + AoI 回报 - 路径代价 - 延迟惩罚 - 能量保护惩罚”的短窗口分数做单任务滚动决策
- 当前已先将分区层拆成独立的 `tasking/partitioning/` 子目录，并保留兼容入口；当前保留两条明确路线：
  - `baseline_fixed_partition`：保持与历史硬责任区完全等价
  - `soft_partition_policy`：先只接到 `cost_aware_centralized_allocator`，用于验证“放开一个受控次候选”是否能给任务层创造真实竞争空间
- 当前又补入了第三条路线：
  - `backlog_aware_partition_policy`：在 `soft_partition_policy` 基础上进一步读取同一轮 backlog，优先解决“baseline 被热点长期挤压”这一类任务层前置约束问题
- 当前开始补入第四条路线：
  - `weighted_voronoi_partition_policy`：在分区层里直接用“距离 + 负载 + 支援保护”做轻量加权维诺图分区，先固定 `cost_aware_centralized_allocator` 验证它能否比纯规则式 `soft_partition_policy` 更稳地打开候选竞争空间
- 这样后续可以在“固定 baseline 分区，对比任务层算法”和“固定任务层，对比分区策略”之间做更干净的单因素实验，而不必再把硬责任区逻辑散落在各个 allocator 里
- 当前这轮分区层单因素结果已经说明：`soft_partition_policy` 是有效的结构变量，不再只是架构抽象
  - 它能明显降低 `cost_aware` 的 `blocked calls` 和整体规划代价
  - 也能把 `valid_cells` 拉高、`stale_cells` 压低
  - 但会牺牲一部分 `confirmed_hotspots`
- 因此下一步应优先验证：当 `AEA / RHO` 也接入 `soft_partition_policy` 后，它们能否在“已经存在真实候选竞争”的前提下体现出超越 `cost_aware` 的任务层价值，而不是继续被硬责任区锁死
- 同时，分区层本身的下一步主线已从“继续微调 backlog-aware”转成“验证 `weighted_voronoi_partition_policy` 是否能成为比 `soft_partition_policy` 更强的新基线”
- 当前这轮 `offshore_hotspot_pressure / 3-seed / 1200-step / cost_aware` 分区层正式复核已经给出清晰结论：
  - `weighted_voronoi_partition_policy` 相比 `soft_partition_policy`，同时拿到了更高的 `confirmed_hotspots`、更高的 `valid_cells`、更低的 `stale_cells`
  - `astar_total_calls` 与 `astar_blocked_calls` 也继续下降
  - 代价是 `coverage_ratio` 略低于 `soft_partition_policy`，且单次规划展开量更高
- 因而当前最值得推进的分区层主线已经从 `soft_partition_policy` 切到 `weighted_voronoi_partition_policy`
- 这意味着后续更合理的算法实验顺序应改成：
  - 先固定 `weighted_voronoi_partition_policy`
  - 再重新比较 `cost_aware / AEA / RHO`
  - 看任务层算法在这个更强的新候选空间上是否终于能稳定拉开差异
- 当前这轮 `offshore_hotspot_pressure / 3-seed / 1200-step / weighted_voronoi_partition_policy` 的任务层正式复核已经给出新的主线判断：
  - `cost_aware` 仍然是当前最均衡的中间基线：热点确认高、freshness 不差、blocked 很低
  - `AEA` 在更强的候选空间下并没有成为赢家，反而表现成更保守的版本：`blocked` 最低，但热点确认与 freshness 都弱于 `cost_aware`
  - `RHO` 终于真正和 `cost_aware` 拉开了差异，而且方向清晰：更高的 `valid_cells`、更低的 `stale_cells`、`blocked = 0`，同时热点确认数也略高
  - 但 `RHO` 也暴露出新的取舍：`coverage_ratio` 偏低，`expanded_nodes` 偏高
- 因此当前任务层主线已经不再是“继续围绕 AEA 调小常量”，而是：
  - 以 `weighted_voronoi_partition_policy` 为新分区基线
  - 重点推进 `RHO`，查清它为何能显著改善 freshness，但会压低整体 coverage
- 当前第一轮 `offshore_hotspot_pressure / seed=20260324 / 800-step` 的软分区快对比已经给出一个有用信号：
  - `RHO` 与 `cost_aware` 仍然完全重合，说明即使引入 `soft_partition_policy`，它的窗口分数也还没有真正翻动关键任务排序
  - `AEA` 则已经开始与 `cost_aware` 拉开差异：`valid_cells` 更高、`stale_cells` 更低、`blocked_calls` 更少，但 `coverage_ratio` 与 `confirmed_hotspots` 略低
- 因此后续应优先沿两条线继续推进：
  - 对 `RHO` 做分数分解诊断，确认是软分区开放范围仍不足，还是 `AoI / delay / path` 结构仍被当前分层顺序锁死
  - 对 `AEA` 在 soft 分区下补 `3-seed` 正式复核，确认当前这条“更稳、更少 blocked、但稍弱于热点完成”的趋势是否稳定
- 当前还已完成一个针对 `offshore_hotspot_pressure / seed=20260325 / 1200-step` 的结构性排查：
  - 将 `AEA` 从“热点层先分、baseline 层后分”改成了统一竞价池
  - 再进一步加入 `baseline stale bonus` 与 `hotspot backlog penalty`
  - 再进一步加入显式的 `baseline backlog guard penalty`
  - 但该坏 seed 的结果仍没有发生变化
  - 这说明当前 `AEA` 的问题已不再主要是任务分层先后，而更可能是：
    - `base_value / AoI / path_cost` 的尺度关系仍让热点任务持续占优
    - `soft_partition_policy` 开放出来的候选空间仍不足以翻动关键排序
- 当前 `offshore_hotspot_pressure / 3-seed / 1200-step / soft_partition_policy` 的正式复核还表明：
  - `AEA` 已经能比 `cost_aware` 拿到更多 `confirmed_hotspots`
  - 但代价是更低的 `coverage_ratio`、更高的 `blocked_calls` 与更大的 `expanded_nodes`
- 因此对 `AEA` 的下一步不应再继续围绕“是否分层”或“小幅常量调参”打转，而应优先考虑：
  - 更进一步的分区/候选开放策略
  - 或在分区层上引入显式的 backlog-aware 候选保留机制，而不是继续只在 `bid` 里微调局部奖励
- `hybrid_astar_path_planner` 继续保留，但暂时降级为探索型 planner；只有在 `A* + smoother` 与 `astar` 的主线结论不够用时，再继续集中调优 hybrid 参数
- 再往后才做：更复杂的分布式 `CBBA / auction allocator`
- 原因：那一层更适合作为 `AEA` 之后的下一档任务层对照，而不是当前主线第一优先

### 1.3 任务层升级方向

- 当前任务层已经具备：
  - 基础责任分区 + 启发式分配
  - 第一版 `cost-aware centralized allocator`
  - 第一版 `aoi_energy_auction_allocator`
- 当前最近确认的直接改进点：
  - 对“已有责任区候选但当前不可达”的任务增加短时冷却，避免同一热点在连续多个 step 里被重复做 `A*` 可达性检查
- 后续可升级方向包括：
  - `AEA` 的权重标定与固定数据集调参
  - `AEA` 的 AoI / energy 出价权重继续收敛
  - `RHO` 的窗口长度、延迟惩罚和 AoI 回报权重调参
  - 更复杂优先级规则
  - 弱分区与跨区支援
  - 多任务调度
  - 更高级分布式任务分配算法
- 后续若做算法比较，应保留当前基础任务分配器作为对照 baseline

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

- 当前执行层属于第一阶段基础可用版本
- 若后续引入更真实物理模型，则执行层必须同步升级
- 后续可考虑：
  - 更正式的路径跟踪控制器
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
- 当前最新一轮 `offshore_hotspot_pressure / 3-seed / 1200-step` 的任务层三方对比表明：
  - `aoi_energy_auction_allocator` 已经不是“只能跑通”的实验算法，而是对 `basic` 和 `cost_aware` 都具备竞争力
  - 相比 `basic_task_allocator`，`AEA` 更偏热点确认
  - 相比 `cost_aware_centralized_allocator`，`AEA` 当前只在 `offshore_hotspot_pressure` 下表现出轻微正信号，优势并不稳定
- 当前 `planner_path_stress / 3-seed / 1200-step` 的任务层三方对比表明：
  - `cost_aware_centralized_allocator` 目前仍是更稳的中间基线
  - `aoi_energy_auction_allocator` 第一版在路径压力更强的场景里还没体现出稳定优势
  - 因此下一步不应急着扩大 AEA 的算法复杂度，而应优先：
    - 调整 AoI / path / energy 相关权重
    - 引入更能放大 AoI 冲突的新环境
- 当前已新增 `aoi_revisit_pressure` 场景，用于专门放大“近但不急”和“远但更 stale”之间的任务价值冲突；后续 AEA 调参和正式对比应优先在该场景下展开
- 当前首轮 `aoi_revisit_pressure / 3-seed / 1200-step` 结果表明：
  - `cost_aware_centralized_allocator` 仍然优于第一版 `AEA`
  - `AEA` 和 `cost_aware` 的热点确认数已经打平，但 `AEA` 的 `valid_cells` 更少、`stale_cells` 更多
  - 这说明新场景方向是对的，但第一版 `AoI / path / energy` 权重还没有调到能稳定放大 AoI 优势
- 因此 `AEA` 的下一步优先级应收敛为：
  - 固定 `aoi_revisit_pressure`
  - 先做结构性调整，让 `AEA` 拥有真实多 agent 竞争
  - 再继续做权重调参
- 当前 bid 分解诊断已经确认：第一版 `AEA` 的一个关键问题是严格责任区下很多任务只有单一候选 `USV`，导致 auction 实际上没有发生；因此已先做一版轻量结构调整，将 `AEA` 放宽为“主责任区优先 + 1 个次优候选 `USV` 兜底”，只在主候选不可用或不可达时才启用次优候选
- 当前第一轮调参已先做一版“提高 `AoI` 收益权重、下调热点基础价值”的试探性更新；后续应在新的双候选结构下再看这版权重是否能把 `valid_cells / stale_cells` 拉回到至少接近 `cost_aware`
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
