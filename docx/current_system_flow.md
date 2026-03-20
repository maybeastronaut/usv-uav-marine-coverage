> 说明：后续如果 Codex 出现上下文缺失或记忆不完整，应优先以本文件作为“当前已实现项目结构与实际流程”的准确信息源，只按这里描述的已实现内容理解系统，不要把未实现规划当成现状。

# 当前已实现项目结构与流程说明

本文档只描述当前仓库里已经实现的内容。

维护约定：

- 本文件只记录“当前代码真实已经实现了什么”
- 只保留当前有效事实，不保留过时方案、备选路线和已放弃设想
- 每次代码行为、系统结构、默认参数或回放表现发生变化时，都应同步覆盖更新本文件
- 后续阶段的升级方向、方案权衡和研究决策统一写入 `discussion_notes.md`

补充说明：

- 当前 Python 源码包名已经统一为 `usv_uav_marine_coverage`
- 后续运行入口、导入路径和测试模块都应使用这一包名

不包含：

- 尚未实现的未来规划
- 仅在讨论中出现、但还没落地的方案
- 对后续算法阶段的设想

## 1. 当前项目已经实现了什么

截至目前，项目已经实现的是一个可运行的基础版异构多智能体闭环仿真系统，而不再只是环境与展示底座。当前系统已经具备：

- 二维连续海域场景
- 从左到右的三区带海域
- 伪随机障碍环境生成
- 中间风险区障碍与两条不规则可通过航道约束
- 远海区孤立障碍与风险区
- 近海区基础任务点动态生成
- 远海区任务热点动态生成
- 智能体第一阶段闭环数学模型
- `UAV` 能量约束与会合补能闭环
- `25m` 规则矩形栅格网络
- 基于 `footprint` 的栅格覆盖映射
- 基于栅格的信息地图层
- 第一版任务层、路径层与执行层接口
- 统一实验配置层
- 可复用实验场景目录
- 正式实验数据集目录
- 批量实验入口与统一实验汇总输出
- 五套可切换的任务分配算法
- 三套可切换的 `UAV` 搜索规划算法
- 三套可切换的 `USV` 路径规划算法
- 基础巡检与热点确认双任务源最小闭环
- 覆盖映射静态可视化测试
- 正式圆形 `footprint` 展示
- HTML 海图输出
- HTML 回放式仿真预览输出
- 回放式仿真同步日志输出
- 海图上的静态 `3 个 USV + 2 个 UAV` 外观示意
- 同一 HTML 内 `clean/debug` 双模式海图切换显示
- 智能体编号标签开关
- 当前帧规划路径虚线展示
- GitHub 语言统计已通过 `.gitattributes` 忽略生成型 HTML/日志产物

当前系统的真实边界是：

- 已经实现基础版任务层、路径层、执行层和回放分析链
- 已经实现第一阶段闭环动力学、基础任务分配、`UAV/USV` 分工、会合补能与局部恢复机制
- 已经进入更高级任务算法与规划算法对比阶段
- 已经接入第一版执行层局部 `MPC`
- 更高保真控制器与动力学建模当前仍未实现

## 2. 当前项目结构是什么意思

### 2.1 海域场景

当前系统里已经有一个固定海域场景：

- 整体大小：`1000m x 1000m`
- 近海区：`0m - 250m`
- 中间风险区：`250m - 450m`
- 远海区：`450m - 1000m`

这部分的作用是：

- 给整个项目提供统一的仿真空间
- 给障碍、智能体外观和后续任务提供共同底图

### 2.2 障碍环境

当前系统已经能生成一套障碍环境。

当前障碍结构包括：

- 中间风险区 `4` 个不规则障碍块
- 中间风险区至少 `2` 条不规则可通过航道约束
- 远海区 `3` 个孤立障碍
- 远海区 `2` 个风险区

这部分的作用是：

- 让海域不再是纯净空白地图
- 先把真实海洋环境中的复杂因素抽象出来
- 给后续路径规划和覆盖任务准备环境基础

当前障碍环境支持伪随机生成，也支持用固定 `seed` 复现同一张环境。

当前障碍生成器不再是“单次随机生成失败就报错”，而是：

- 先按外部 `seed` 做首轮确定性采样
- 若候选布局不满足航道或非重叠约束，则继续按同一外部 `seed` 的确定性序列自动重采样
- 直到找到合法障碍布局，或在极少数情况下达到最大尝试次数后再报错

补充说明：

- 当前环境快照与日志中会额外记录 `generation_attempts`
- 这使得后续批量实验不需要手工维护“可用 seed 白名单”

补充说明：

- 当前这版障碍环境已经落地到代码
- 但后续在接入 `USV` 路径规划与动态仿真联调时，这一部分仍可能继续调整

### 2.3 监测点与任务热点

当前系统不再在初始状态预放静态任务点位。

当前改为在仿真过程中动态维护两类任务源：

- 近海区随机生成基础任务点
- 海域内伪随机动态生成热点源

这些动态任务源当前的生成规则是：

- 都采用伪随机生成
- 会落在对应分区内部
- 会避开障碍与风险区
- 当前热点生成已收紧为“中心格及周边一圈邻域”都不得与障碍或风险区重叠
- 仿真中同时存在的 ground-truth hotspot 硬上限为 `12`
- 当前热点处理链已经明确为：
  - 真实热点可以在海域中伪随机产生
  - 只有当 `UAV` 在当前 step 观测到该热点所在格子时，才会完成热点初检
  - 只有完成 `UAV` 初检后，系统才会生成 `hotspot_confirmation`
  - `USV` 在任务点固定驻留若干步完成精检后，会直接确认该真热点
  - 一旦 `USV` 精检完成，该热点会从 ground-truth 层和活跃可视化层中一并移除

这部分的作用是：

- 先把后续任务层需要的基础点位放进环境
- 为 `UAV` 先发现热点、`USV` 后续复查的协同流程准备静态目标

### 2.4 智能体外观

当前系统里已经定义并展示了 `3 个 USV + 2 个 UAV` 的正式外观样式，这套外观既用于静态海图，也用于动态回放。

这部分的作用是：

- 先确认智能体在海图上的外观、大小和可读性
- 给后续动态仿真提前准备显示层

补充说明：

- 当前 `USV` 与 `UAV` 初始部署统一放在海图左侧

### 2.5 智能体基础数学模型

当前系统已经具备一套最小可用的智能体数学模型。

当前模型已经包含：

- 位置
- 朝向
- 当前速度
- 当前角速度
- 最大速度
- 最大加速度
- 最大减速度
- 最大转向角速度
- 巡航速度
- 到达容差
- 探测半径
- 覆盖半径
- 基础任务状态
- `UAV` 能量容量
- `UAV` 剩余电量
- `UAV` 能量消耗参数
- `UAV` 补能速率与保底电量阈值
- 平台参数配置

当前模型已经支持：

- 给智能体分配基础任务
- 从任务生成参考目标
- 根据参考目标计算轻量控制指令
- 按受约束航向/速度模型更新实际状态
- `UAV` 在搜索与转向过程中的能量消耗更新
- `UAV` 在会合补能阶段的能量恢复
- 判断一个点是否落入探测半径或覆盖半径

当前正式覆盖半径是：

- `USV = 50m`
- `UAV = 100m`

这部分的作用是：

- 为后续覆盖映射提供智能体状态输入
- 为后续动态仿真提供统一状态表达

补充说明：

- 当前数学模型第一阶段已经落地到代码
- 当前覆盖方法研究默认以这套第一阶段闭环模型作为基线
- 当前这套模型已经接入仿真主循环、任务层、路径层与执行层
- 第二阶段和第三阶段的增强动力学与控制器研究当前暂缓，尚未实现

### 2.5.1 执行层局部 MPC

当前执行层已经支持两种可切换执行策略：

- `phase_one_execution`
- `local_mpc_execution`

当前第一版 `local_mpc_execution` 的真实实现边界是：

- 当前只对 `USV` 生效，`UAV` 继续沿用原有路径跟踪
- 不改任务层和全局规划层，只在执行层对一步控制量做局部修正
- 当前采用短时域候选控制采样，而不是外部数值求解器
- 当前同时考虑：
  - 路径跟踪目标
  - 静态多边形风险障碍
  - 远海圆形障碍与风险区
  - 邻近 `USV` 的安全间距
- 当前已修正第一版“每一步都强制 MPC 接管”带来的 `USV` 低速原地抽动问题：
  - 现在只有在检测到局部风险时才启用 `MPC`
  - 无局部冲突时会退回原有路径跟踪
- 当前 `risk_area` 也已经纳入局部 `MPC` 接管判定，不再只对多边形障碍和圆形孤立障碍生效

这部分的作用是：

- 为 `USV` 增加第一版实时避障能力
- 给后续动态海浪区、临时危险区和事件触发局部重规划预留执行层入口

### 2.6 离散栅格网络

当前系统已经能把连续海域环境映射为规则矩形栅格网络。

当前栅格网络参数是：

- 栅格边长：`25m`
- 栅格规模：`40 x 40`

当前每个栅格已经能表达：

- 所属分区
- 是否障碍
- 是否风险区
- 是否为初始基础监测点所在栅格
- 是否为初始任务热点所在栅格
- 覆盖次数
- 覆盖来源
- 最近覆盖步

这部分的作用是：

- 为后续覆盖映射提供统一离散载体
- 为后续覆盖统计、任务统计和环境属性读取提供网格基础

补充说明：

- 当前系统已经支持按“中心点落入圆形 footprint 即覆盖”的规则更新栅格覆盖状态
- 当前静态覆盖预览使用静态演示智能体与正式 `footprint` 半径，用于验证覆盖映射是否与栅格位置对齐
- 静态覆盖预览属于验证视图；动态仿真中的覆盖与信息状态由回放主循环单独驱动

### 2.7 信息地图

当前系统已经具备一层独立的信息地图。

当前信息地图直接复用现有规则矩形栅格，并在每个栅格上记录：

- 信息是否有效
- 最后一次观测时间步
- 信息年龄
- 基础监测任务
- 真实热点状态
- 系统认知的热点状态
- UAV 初检与 USV 精检进度

当前信息地图已经支持：

- 随时间推移更新信息年龄
- 信息过期后转为 `stale_known`
- 当前默认信息失效阈值为 `400 steps`
- 当前近海基础任务生成频率已进一步下调，同时活跃上限收敛为 `1` 并加入更长的已服务点冷却时间，避免近海任务点过密刷新
- 栅格重新观测后立即恢复为 `valid`
- 近海/远海动态热点生成
- UAV 对当前观测到的真实热点完成初检
- USV 近距离连续停留若干步后会直接确认该热点并关闭任务
- 热点任务一旦完成，会立即从当前活跃热点可视化层移除，并记录为已关闭，后续不再被重复发现为待确认热点

补充说明：

- 当前信息地图不引入独立区域图，直接以栅格为最小信息单元
- 当前回放页面中的 `valid/stale cells` 只统计非障碍格，不再把障碍格计入信息新鲜度统计
- 静态障碍和风险区默认从一开始全局已知
- 热点在 `USV` 精检完成后会从环境真相层中移除

### 2.8 回放式仿真预览与最小闭环

当前系统已经能输出一份 HTML 回放式仿真预览。

这部分的作用是：

- 让当前阶段的智能体运动、覆盖、信息刷新和热点状态变化能被直接观察
- 在不进入完整任务层与路径层实现前，先验证整体表现是否符合预期

当前回放预览已经包含：

- `3 个 USV + 2 个 UAV` 的时序运动
- 轨迹显示
- `footprint` 显示开关
- 编号标签开关
- 时间步滑块
- 播放/暂停
- 覆盖率统计
- 热点状态分层：
  - 未经 `UAV` 初检的活跃热点显示为黄色
  - 已完成 `UAV` 初检、等待 `USV` 精检的热点显示为红色
  - `USV` 精检完成后热点立即消失
- 事件日志面板
- 同步输出的 `events.jsonl` 逐事件日志
- 同步输出的 `summary.json` 最终汇总日志
- 日志中已包含当前启发式任务指派依据、路径摘要、执行偏差与热点处理链骨架
- 基于 `tasking / planning / execution` 三层接口的第一版双任务源闭环
- `PATROL -> GO_TO_TASK -> ON_TASK -> GO_TO_RENDEZVOUS -> ON_RECHARGE -> RETURN_TO_PATROL` 的基础执行状态机
- `baseline_service + hotspot_confirmation` 任务生成、分配、`USV` 风险加权 `A*` 路径执行与任务关闭
- `uav_resupply` 低电量补能任务生成、最近 `USV` 会合、附着跟随充电与回巡航
- 当前任务层已支持五套可切换的任务分配算法：
  - `basic_task_allocator`
  - `cost_aware_centralized_allocator`
  - `aoi_energy_auction_allocator`
  - `distributed_cbba_allocator`
  - `rho_task_allocator`
- 当前 `basic_task_allocator` 负责“热点优先、同类按创建时间、责任分区内最低代价 `USV` 分配”的基础任务算法
- 当前 `cost_aware_centralized_allocator` 负责在严格责任区约束下，对 `baseline_service` 和 `hotspot_confirmation` 建立代价矩阵，并以“优先级分层 + 集中式贪心”方式做集中分配
- 当前 `cost_aware_centralized_allocator` 已额外加入“不可达任务冷却”机制：若某个任务对责任区内当前可用 `USV` 都不可达，则该任务会进入短时 backoff，避免在连续多个 step 内重复触发同一组 `A*` 可达性检查
- 当前 `aoi_energy_auction_allocator` 负责对 `baseline_service` 和 `hotspot_confirmation` 计算 “任务基础价值 + 目标格 AoI 收益 - 真实路径代价 - 低电量 UAV 支援保护惩罚” 的竞价分数，并在同一优先级层内按“主责任区 `USV` 优先，主候选不可用或不可达时再启用 1 个次优候选 `USV`”的轻量兜底机制做集中式贪心分配
- 当前 `rho_task_allocator` 负责在固定短窗口近似下，对 `baseline_service` 和 `hotspot_confirmation` 计算 “任务基础价值 + AoI 回报 - 路径代价 - 延迟惩罚 - 低电量 UAV 支援保护惩罚” 的滚动分数，并在严格责任区内按最高窗口分数做集中式贪心分配
- 当前 `distributed_cbba_allocator` 负责在当前分区层候选集合上，让每艘空闲 `USV` 本地计算 bid 并通过同步 winner 轮次解决冲突；当前第一版已支持 `bundle = 1/2` 的贪心有序任务 bundle，只处理 `baseline_service` 和 `hotspot_confirmation`，`uav_resupply` 继续走现有专门逻辑，并已支持通过 `distributed_sync_interval_steps` 配置“每隔 N 步同步一次”的有限通信频率，通过 `distributed_broadcast_range_m` 按 `USV` 通信连通簇拆分局部协商市场，以及通过 `distributed_winner_memory_ttl_steps` 模拟局部 winner 记忆滞后/信息过期
- 当前已将分区层正式拆到 `tasking/partitioning/` 子目录，并保留 `zone_partition_layer.py` 作为兼容包装入口；当前提供四套分区策略：
  - `baseline_fixed_partition`：与原硬责任区完全等价，近海任务主责任 `USV-1`、远海上半区主责任 `USV-2`、远海下半区主责任 `USV-3`、`uav_resupply` 对全部 `USV` 开放
  - `soft_partition_policy`：在保持主责任区不变的前提下，按“主责任 `USV` 不可用 / 次候选明显更近 / 任务已老化”三个条件为任务开放 1 个次候选 `USV`
  - `backlog_aware_partition_policy`：当前在 `soft_partition_policy` 基础上进一步读取同一轮 pending 任务集合；当 aged `baseline_service` backlog 过高时，会更积极地给 baseline 开放次候选并收紧 hotspot 的次候选开放；当 hotspot backlog 较高且 baseline backlog 不高时，再恢复对热点的次候选开放
  - `weighted_voronoi_partition_policy`：当前第一版用“任务点到 `USV` 的几何距离 + 忙碌惩罚 + 低电量 UAV 最近支援保护惩罚”计算轻量加权分区代价，并按固定 margin 受控开放 1 个次候选 `USV`
- 当前 `cost_aware + soft_partition_policy` 在 `offshore_hotspot_pressure / 3-seed / 1200-step` 下已经出现清晰分区层信号：
  - 相比 `baseline_fixed_partition`，`soft_partition_policy` 的 `coverage_ratio / valid_cells` 更高，`stale_cells / astar_blocked_calls / astar_total_calls` 更低
  - 但 `confirmed_hotspots` 下降，说明这版 soft 分区更偏整体 freshness 与稳定性，而不是更激进地追热点确认
- 当前 `backlog_aware_partition_policy` 已完成第一版实现、配置接线和单元测试，但首轮 `3-seed / 1200-step` 对比显示它尚未真正改变 `cost_aware` 的任务决策，当前更值得继续验证的是新的 `weighted_voronoi_partition_policy`
- 当前 `weighted_voronoi_partition_policy` 已完成第一版实现、配置接线和 `offshore_hotspot_pressure / 3-seed / 1200-step` 正式复核；在固定 `cost_aware_centralized_allocator` 下，相比 `soft_partition_policy`：
  - `confirmed_hotspots` 明显回升：`24.0 > 21.0`
  - `valid_cells` 继续提高：`1013.3 > 987.7`
  - `stale_cells` 继续下降：`355.0 < 380.7`
  - `astar_total_calls / astar_blocked_calls` 进一步下降：`185.3 / 2.3 < 263.3 / 12.0`
  - 但 `coverage_ratio` 略低于 `soft_partition_policy`，且 `astar_expanded_nodes` 高于 `soft_partition_policy`
- 因此当前分区层的最新判断已经收口为：
  - `baseline_fixed_partition`：更偏热点确认，但 blocked 与规划负担高
  - `soft_partition_policy`：更偏整体 freshness 与稳定性
  - `weighted_voronoi_partition_policy`：当前最有潜力成为新的主推分区基线，因为它在热点确认、freshness 和 blocked 控制之间给出了更平衡的结果
- 当前 `aoi_energy_auction_allocator` 已从“`hotspot_confirmation` 与 `baseline_service` 分层顺序分配”调整为“统一竞价池排序”：
  - 除 `uav_resupply` 外，其余任务现在会进入同一个 AoI-energy 市场统一比较
  - 这保证了 stale baseline 任务在结构上已经拥有和热点任务直接竞争的机会
- 当前 `aoi_energy_auction_allocator` 还已补入两项针对热点偏热的分数修正：
  - `baseline stale bonus`
  - `hotspot backlog penalty`
- 当前在更强的 `weighted_voronoi_partition_policy` 分区层下重新比较 `cost_aware / AEA / RHO`（`offshore_hotspot_pressure / 3-seed / 1200-step`）后，任务层画像已经明显分叉：
  - `cost_aware + weighted_voronoi`：当前最均衡，`confirmed_hotspots = 24.0`，`valid_cells = 1013.3`，`stale_cells = 355.0`，`astar_blocked_calls = 2.3`
  - `AEA + weighted_voronoi`：当前更保守，`blocked` 最低（`1.0`），总调用数也最低，但 `confirmed_hotspots = 21.0`，`valid_cells / stale_cells` 都弱于 `cost_aware`
  - `RHO + weighted_voronoi`：当前最像“freshness-first”版本，`valid_cells = 1063.3` 最高，`stale_cells = 305.0` 最低，`confirmed_hotspots = 24.3` 也略高于 `cost_aware`，而且 `astar_blocked_calls = 0.0`
  - 但 `RHO` 的代价是 `coverage_ratio` 更低，且 `astar_expanded_nodes` 高于 `cost_aware`
- 因此当前任务层的最新阶段性判断已经收口为：
  - 若强调整体稳健和平衡：`cost_aware + weighted_voronoi`
  - 若强调 freshness / stale 控制：`RHO + weighted_voronoi`
  - `AEA` 在新分区层下暂时仍未成为最优解
- 当前已经实现第一版 `RHO` 任务层，用于替代“只调 AEA 权重”的单一路线；该版本先固定 `uav_lawnmower_planner + astar_path_planner`，保持中心化和单任务滚动决策，目标是验证“短窗口任务价值优化”本身是否优于 `cost_aware`
- 当前“中心化 vs 分布式任务分配”的正式主对比已经完成（`offshore_hotspot_pressure / weighted_voronoi / 3-seed / 1200-step`）：
  - `cost_aware + weighted_voronoi`：当前最均衡，`confirmed_hotspots = 24.0`，`valid_cells = 1013.3`，`stale_cells = 355.0`
  - `RHO + weighted_voronoi`：当前仍是 freshness-first 主线，`valid_cells = 1063.3` 最高，`stale_cells = 305.0` 最低，`confirmed_hotspots = 24.3`
  - `distributed_CBBA(bundle=2) + weighted_voronoi`：当前已经能形成有效分布式协商结果，`astar_blocked_calls = 0.7` 较低、总调用更少，但 `confirmed_hotspots = 18.3`，`valid_cells = 991.3`，`stale_cells = 377.0`
- 当前分布式任务分配的最新阶段性判断已经收口为：
  - `distributed_CBBA(bundle=2)` 已经可以作为正式分布式任务层进入对比实验
  - 但在当前主场景下，整体表现仍弱于 `cost_aware / RHO` 两种中心化方案
  - 当前分布式研究价值更偏向“去中心化可行性与架构对比”，而不是已达到性能最优
- 当前任务层会将 `uav_resupply` 作为最高优先级紧急任务处理，并为低电量 `UAV` 选择最近 `USV` 作为会合补能对象
- 当前 `uav_resupply` 触发阈值采用“到最近 `USV` 的预计可达能耗 + 45 单位安全余量”
- 当前 `uav_resupply` 释放阈值采用“充到 `90%` 电量后脱离 `USV` 回到巡航”
- 当前 `UAV` 补能速率为每步 `12` 单位能量
- `USV` 仅在到达任务点并进入 `ON_TASK` 固定驻留 `5` 步后，才会完成热点确认或基础任务处理
- `UAV` 在低电量时会按“能否安全飞到最近 `USV`”提前触发补能，并从搜索巡航切换到 `GO_TO_RENDEZVOUS -> ON_RECHARGE -> RETURN_TO_PATROL`
- 近海基础任务点动态生成与海域内最多 `12` 个热点源动态存在
- 动态 hotspot 数量硬上限 `12`
- `UAV` 只会对当前观测到的真实热点完成初检，不再额外制造误报热点
- `USV` 对已完成 UAV 初检的热点执行精检时，完成驻留阈值后会直接确认并关闭热点

当前模块组织方式：

- `simulation/` 子包统一承载当前回放仿真相关模块
- `simulation/experiment_config.py` 已提供统一实验配置层，当前支持 baseline 配置 dataclass、可复用场景预设、TOML 加载、CLI 覆盖与配置摘要序列化
- `simulation/scenario_catalog.py` 已提供可复用实验场景目录，当前内置：
  - `baseline_patrol`：当前第一阶段 baseline 场景，任务密度适中，作为统一对照基线
  - `aoi_revisit_pressure`：AoI 优势放大场景，缩短信息超时并保持中等热点压力，同时保留少量近海基础任务干扰，适合观察“近但不急”和“远但更 stale”之间的任务价值冲突
  - `planner_path_stress`：`USV` 路径规划对比场景，保持中等远海热点压力并进一步压低近海基础任务干扰，适合观察跨风险区往返、任务接入与回巡航路径差异
  - `return_to_patrol_stress`：`USV` 回巡航接入压力场景，保持中等远海热点负载并进一步降低近海任务噪声，适合观察任务完成后长距离回巡航、跨风险区往返与路径折返差异
  - `offshore_hotspot_pressure`：远海热点压力场景，提高远海热点生成压力，适合观察热点响应与确认链路
  - `distributed_overlap_pressure`：分布式协商放大场景，同时提高近海基础任务、近海热点和远海热点活跃度，并缩短信息超时，适合放大 weighted Voronoi 下的多艇候选重叠、跨区重分配与局部通信约束影响
  - `nearshore_baseline_pressure`：近海基础任务压力场景，提高近海基础巡检任务密度，适合观察驻区巡航与基础任务处理能力
  - `mixed_task_pressure`：混合任务压力场景，同时提高近海基础任务与远海热点压力，适合观察多任务竞争下的整体调度表现
- `configs/experiment_datasets/` 已提供实验数据集配置目录，当前用于集中维护“固定场景 + 固定随机种子集合 + 固定步数 + 固定统计口径”的正式对比数据集
- `configs/experiment_datasets/` 当前已作为正式实验数据集目录使用，不再只是存放配置；目录内可同时保存：
  - `batch.toml`
  - `comparison_summary.json`
  - 每个 seed 的 `events.jsonl`
  - 每个 seed 的 `summary.json`
  - 代表性回放 `HTML`
- 当前已固化并保留的正式实验数据集包括：
  - `configs/experiment_datasets/usv_planner_offshore_hotspot_pressure_3seed_800/`
    - 用于在固定 `cost_aware_centralized_allocator + uav_lawnmower_planner` 下，对比 `astar_path_planner` 与 `astar_smoother_path_planner`
  - `configs/experiment_datasets/usv_planner_return_to_patrol_stress_3seed_1200/`
    - 用于在 `return_to_patrol_stress` 场景下，对比 `astar_path_planner` 与 `astar_smoother_path_planner` 在热点完成与回巡航链上的差异
  - `configs/experiment_datasets/partition_policy_offshore_hotspot_pressure_3seed_1200/`
    - 用于在固定 `cost_aware_centralized_allocator` 下，对比 `baseline_fixed_partition / soft_partition_policy / weighted_voronoi_partition_policy`
  - `configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_3seed_1200/`
    - 用于在固定 `weighted_voronoi_partition_policy` 下，对比 `cost_aware / AEA / RHO`
  - `configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_weighted_voronoi_distributed_bundle2_3seed_1200/`
    - 用于在固定 `weighted_voronoi_partition_policy` 下，对比 `cost_aware / RHO / distributed_CBBA(bundle=2)`
  - `configs/experiment_datasets/distributed_cbba_bundle_compare_distributed_overlap_pressure_3seed_1200/`
    - 用于在 `distributed_overlap_pressure` 下，对比 `distributed_CBBA` 的 `bundle = 1 / 2`
  - `configs/experiment_datasets/distributed_cbba_bundle2_memory_compare_distributed_overlap_pressure_3seed_1200/`
    - 用于在 `distributed_overlap_pressure` 下，对比 `distributed_CBBA(bundle=2)` 的 `winner_memory_ttl = 0 / 5 / 10`
- 当前分布式内部机制的已落地结论也已经收口：
  - `bundle = 1 -> bundle = 2` 会改变 `distributed_CBBA` 的最终行为，说明分布式协商不再对内部结构完全不敏感
  - 但在当前 `bundle = 2` 版本下，`winner_memory_ttl = 0 / 5 / 10` 的三组正式结果仍完全一致，说明局部 winner 记忆滞后尚未进一步改变最终协商轨迹
- 当前 `USV` 规划层的已落地对比结论也开始分场景收口：
  - `planner_path_stress`
    - 更适合观察 `astar_smoother_path_planner` 在整体 coverage / freshness 维持上的潜在优势
  - `return_to_patrol_stress`
    - 更适合观察 `astar_smoother_path_planner` 在热点完成数、任务后回巡航接入与 `blocked` 行为上的差异
- 当前正式数据集目录内已同时保存：
  - `batch.toml`
  - `comparison_summary.json`
  - 每个 seed 的 `events.jsonl`
  - 每个 seed 的 `summary.json`
- 当前实验配置中的 `[scenario]` 段会先选定一个场景预设，再允许 `[information_map]` 对该场景做局部覆盖，从而避免为不同算法重复复制整套场景参数
- 当前这套场景机制已经可以在“同一场景下切换不同算法”与“同一算法下切换不同场景”两种实验方式中复用，避免把场景参数和算法配置绑定在一起
- 当前实验配置层已支持选择：
  - `basic_task_allocator`
  - `cost_aware_centralized_allocator`
  - `aoi_energy_auction_allocator`
  作为任务层算法
- 当前 `aoi_energy_auction_allocator` 已具备基础可运行版本，并已接入统一实验配置层与仿真主流程；其第一版固定采用：
  - 中心化 auction-style 分配，而不是完整分布式协商
  - 只对 `baseline_service` 与 `hotspot_confirmation` 计算竞价分数
  - `uav_resupply` 继续保留现有专门逻辑
  - 竞价分数由“任务基础价值 + 目标格 AoI 收益 - 真实路径代价 - 低电量 UAV 支援保护惩罚”组成
- 当前实验配置层已支持选择：
  - `astar_path_planner`
  - `astar_smoother_path_planner`
  - `hybrid_astar_path_planner`
  作为 `USV` 路径规划算法
- 当前实验配置层已支持选择：
  - `uav_lawnmower_planner`
  - `uav_multi_region_coverage_planner`
  - `uav_persistent_multi_region_coverage_planner`
  作为 `UAV` 搜索规划算法
- `simulation/experiment_batch.py` 已提供批量实验入口，当前支持读取 batch TOML、顺序运行多组实验，并输出 `batch_results.jsonl` 与 `batch_summary.json`
- 当前 batch 运行默认会在配置指定的 `output_dir` 名后自动追加时间戳，写入一个新的输出目录，避免中途打断或重复复现实验时把新旧结果混到同一目录
- 当前 batch 运行已支持按 run 覆盖 `scenario`，从而可以在同一 base config 下复用多个实验场景做算法对比
- 当前 `outputs/` 目录仍是默认运行输出目录；batch 复现实验与未固化的中间产物继续写到这里
- 若某轮实验结果被认定为正式合格数据集，可将其关键配置、代表性日志、汇总结果与回放副本同步固化到 `configs/experiment_datasets/`，但不应假定 `outputs/` 已失去复现用途
- `simulation/simulation_core.py` 负责回放仿真的顶层时间步编排，并组织帧采集与日志采集
- `simulation/simulation_agent_runtime.py` 负责智能体单步推进、路径执行、恢复动作执行与巡航回接点计算
- `execution/progress_feedback.py` 负责 `USV` 的执行反馈判定，包括无进展检测、坏目标冷却以及是否进入 `RECOVERY` / 是否允许重规划
- `simulation/simulation_policy.py` 负责当前 demo 智能体与默认 patrol 数据装配；`USV` 巡航环由 `planning/usv_patrol_planner.py` 生成，`USV` 任务/巡航路径规划可按实验配置在：
  - `planning/astar_path_planner.py`
  - `planning/astar_smoother_path_planner.py`
  - `planning/hybrid_astar_path_planner.py`
  之间切换；`UAV` 搜索航线可按实验配置在：
  - `planning/uav_lawnmower_planner.py`
  - `planning/uav_multi_region_coverage_planner.py`
  - `planning/uav_persistent_multi_region_coverage_planner.py`
  之间切换
- `simulation/simulation_task_runtime.py` 负责任务生命周期同步、任务关闭收尾与任务决策摘要适配
- `simulation/simulation_logging.py` 负责结构化日志输出
- `simulation/simulation_replay_view.py` 负责 HTML 回放页面生成
- `simulation/__init__.py` 仅作为对外稳定门面，统一暴露公开接口

补充说明：

- 当前正式接入主循环的是 `baseline_service + hotspot_confirmation`
- 当前主循环已接入 `baseline_service + hotspot_confirmation + uav_resupply`
- 当前双任务源闭环为：近海基础监测点动态生成 `baseline_service`，远海动态热点经 `UAV` 初检后生成 `hotspot_confirmation`
- 当前 `UAV` 巡航已切换为分区割草机式搜索，而不再只是少量固定航点往返
- 当前 `UAV` 搜索规划已具备三套可切换实现：
  - `uav_lawnmower_planner`：固定上下分区割草机搜索 baseline
  - `uav_multi_region_coverage_planner`：远海固定四 AOI、多区域新鲜度优先排序、区域内割草机覆盖的第一版升级 planner
  - `uav_persistent_multi_region_coverage_planner`：远海固定四 AOI、freshness debt 优先、事件触发 AOI 重排、区域承诺覆盖的第二版升级 planner
- 当前 `USV` 路径规划已具备三套可切换实现：
  - `astar_path_planner`：当前 baseline，带朝向状态的风险加权网格 `A*`
  - `astar_smoother_path_planner`：轻量升级版，保留 baseline `A*` 的状态空间和运动原语，只对 waypoint 链做后处理平滑，并加入最大平滑段长约束，避免把任务路径压缩成会触发连续 `GO_TO_TASK` 重规划的超长直段
  - `hybrid_astar_path_planner`：改进版，使用更丰富的转向原语、更细的碰撞采样和后处理平滑，目标是降低锯齿折返和局部保守 blocked
- 当前 `USV` planner 切换已通过 `planning/usv_path_planner.py` 统一分发，任务层和 runtime 不再各自硬编码单一 `A*`
- 当前第一版 `uav_multi_region_coverage_planner` 不新增新的 UAV 执行状态，而是继续复用 `patrol_route_id + patrol_waypoint_index` 执行链；规划层会按当前信息地图动态重建跨 AOI 的 patrol route，并输出当前区域级航点段
- 当前第二版 `uav_persistent_multi_region_coverage_planner` 已新增并行 `UavCoverageState`，用来记录当前 AOI、区域内 route、承诺剩余航点和最近重排原因；它不改 `AgentExecutionState`，而是通过 `simulation/uav_coverage_runtime.py` 与 runtime 协同工作
- 当前第二版 `uav_persistent_multi_region_coverage_planner` 已进一步加入：
  - 双 `UAV` 间的 AOI 去冲突选择，尽量避免两架机同时扎进同一个远海区域
  - AOI 重新进入时的 sweep 端点接入与必要时 route 反向，避免从区域中段插入破坏割草机覆盖连续性
- 当前第二版 persistent planner 的 AOI 切换不是每步贪心重排，而是按以下事件触发：
  - 当前 AOI route 完成
  - patrol route 缺失/失效
  - 区域承诺释放后，其他 AOI 的 freshness debt 明显更高
  - 补能或回巡航恢复后需要重新选择 AOI
- 当前两版 `UAV` 多区域 planner 都属于“受近年文献方向启发的规则式工程实现”，不是对论文原算法的严格复现；当前主要借鉴的思想包括：
  - 海上多 `UAV` 覆盖规划里的“区域间访问顺序 + 区域内覆盖”分层思路
  - persistent search / adaptive replanning 里的“事件触发重排”思路
  - boustrophedon coverage redistribution 里的“区域内持续扫带覆盖”思路
  - Age of Information / freshness 驱动覆盖里的“以 stale ratio、信息年龄和最近访问时间作为调度信号”思路
- 当前第二版 `uav_persistent_multi_region_coverage_planner` 的具体方法可概括为：
  - 固定 `4` 个远海 AOI
  - AOI 内部继续使用割草机覆盖
  - 区域间按 `stale_ratio -> mean_information_age -> time_since_last_region_visit -> travel_cost -> region_id` 排序
  - 只在事件触发时做 AOI 重排，而不是每步重排
  - 进入 AOI 后执行最小区域承诺，并尽量避免双机扎堆到同一区域
- 当前 `USV` 默认巡航分区已调整为：`USV-1` 负责近海区，`USV-2/USV-3` 负责远海上/下两个分区
- 当前 `USV` 任务分配也已按同一责任区执行：近海任务默认只分给 `USV-1`，非近海任务按上下半区分别优先分给 `USV-2/USV-3`，从而形成“驻区巡航、驻区优先处理任务、任务完成后回本区”的初版分区机制
- 当前近海信息新鲜度阈值已单独提高到 `800 step`，其余海域保持 `400 step`，避免近海基础监测区域过快整体失效
- 当前 `USV-1` 的近海巡航已从四点矩形环改为多航线水平蛇形覆盖巡航，目标是在责任区内尽可能重访近海各条带区域，而不只是沿边界巡回
- 当前若 `USV-1` 因任务打断后需要回巡航，默认优先接回当前覆盖序列的前向航段，而不再简单吸附到最近的中部局部巡航段
- 当前 `USV` 的巡航、任务前往与回巡航路径都已切换为带朝向状态的基础非完整约束 `A*` 栅格路径
- 当前 `USV` 路径规划已补入船体安全余量，`A*` 会在规划阶段直接避开“质心可过但船体会擦边”的障碍邻近区域
- 当前 `USV` 的任务路径已改为条件触发重规划，而不再每一步都重建任务路径；当前默认在计划缺失、任务变化、目标变化、路径失效或明显偏离时才重规划
- 当前任务分配已加入 `USV` 可达性筛选；若某个 `USV` 的任务路径规划结果为 `blocked`，会释放当前分配并回巡，使任务重新进入待分配状态
- 当前执行层已从“逐航点追踪”收敛为“路径段前视跟踪”，以减少 `USV` 沿多航点路径执行时的折线感与抖动
- 当前执行层已补入 `USV` 局部反应式避障：当短视距跟踪目标会擦入障碍时，执行层会先在局部范围内偏转跟踪目标，再由规划层负责更大范围的重规划
- 当前 `USV` 的 `RETURN_TO_PATROL` 阶段也已接入同一套局部反应式避障，不再在回巡航阶段绕过执行层安全修正
- 当前 `USV` 若在 `GO_TO_TASK` 阶段因碰撞防护连续清空路径且原地不动，会主动释放当前任务并回到巡航恢复链路；若在 `PATROL` 阶段发生同类局部死锁，则会主动切换到下一个可达巡航点
- 当前 `USV` 已新增显式 `RECOVERY` 执行状态：当正常巡航、任务前往或回巡航阶段连续 `3` 步无进展，或连续 `2` 次被碰撞防护清空路径时，会进入局部脱困而不是继续全局重规划
- 当前 `RECOVERY` 采用固定局部恢复动作序列：停车、短距离后撤、左右 `25°` 前推、再扩大到左右 `45°` 前推；若恢复成功则回到任务链路或局部巡航段接入，若失败则对坏目标进入冷却
- 当前执行层已把“执行语义”和“反馈语义”拆开：`AgentExecutionState` 只保留阶段、任务、路径与巡航索引，`AgentProgressState` 专门维护 stalled steps、恢复尝试、冷却时间和坏目标记忆
- 当前 `USV` 的回巡航逻辑已从“最近全局巡航点”改为“最近局部巡航段接入点”，避免在障碍边缘反复尝试同一个远处 patrol point
- 当前 `USV` 对失败返回目标默认设置 `8` 步冷却，并保留失败目标签名，避免 `RETURN_TO_PATROL` 连续对同一坏目标每步重跑高成本 `A*`
- 当前结构化日志已补入 `A*` 规划诊断统计；每个 `step_snapshot` 的 `path_layer` 都会记录该步 `A*` 总调用次数、成功/blocked 次数，以及按 `allocator_reachability / runtime_patrol / runtime_go_to_task / runtime_return_to_patrol` 分组的展开节点数
- 当前 `USV` 的局部避障与碰撞防护已从“质心点判定”升级为“带船体安全余量的判定”，避免出现质心未入障碍但船体轮廓已压入障碍边界的假安全
- 当前 `USV` 障碍防护在截断平移时会保留本步更新后的航向，避免船体在障碍边缘因航向被冻结而长期零速卡死
- 当前航点推进已加入“越点推进”判定；当前视跟踪让 `USV` 实际越过中间航点时，会沿路径段投影自动推进索引，避免拿着旧航点慢慢减速至停死
- 当前 `USV` 巡航层已从跨区稀疏全局点改成近海局部安全巡航环，避免巡航 waypoint 直接落在风险障碍内部
- 当前 `USV` 的 `A*` 在局部姿态导致不可展开时，会回退到起始栅格中心并尝试更合适的起始航向，避免“全局可达但局部起步失败”被直接记成 `blocked`
- 当前 `USV` 在空闲 `PATROL` 且暂时没有有效巡航 plan 时，不会立即被无进展反馈误判进 `RECOVERY`；runtime 会优先继续补巡航 plan，再恢复正常巡航推进
- 当前 `cost_aware` 已从“任务级不可达冷却”升级为“`agent-task` 级 blocked 冷却”，避免同一热点在连续多个 step 内被同一艘 `USV` 反复做可达性检查
- 当前 `USV` 的 `RETURN_TO_PATROL` 已加入短窗口 plan 复用；在回巡航目标不变且当前路径仍有效时，不会连续多个 step 对同一目标重复重规划
- 当前 `USV` 的 `PATROL` 已加入最小重规划间隔；刚补过巡航 plan 的短暂衔接期不会立刻再次触发全量 `A*`
- 当前若 `USV` 在 `RETURN_TO_PATROL` 阶段被障碍边界连续截断并清空路径，运行时会自动切换到下一个可达巡航接入点，避免三个 `USV` 长时间一起冻结在障碍附近
- 当前 HTML 回放已新增独立的信息新鲜度图层，能够直接显示 `valid cells` 与 `stale cells`，而不再只能从日志里间接判断栅格信息时效
- 当前 HTML 回放的帧数据已从“每帧完整 SVG 图层字符串”改成“每帧原始 cell/agent 数据 + 单独轨迹数据，由前端即时渲染当前帧”，以避免长步数回放文件膨胀到数百 MB
- 当前 HTML 回放对 `USV` 已不再使用纯直线位置插值，而是使用与起终朝向一致的样条过渡，以更接近“沿船头推进并转弯”的视觉效果
- 当前回放渲染已修正世界坐标航向与 SVG 旋转方向相反的问题，避免 `USV` 出现“位移方向与船头朝向不一致”的视觉错误
- 当前 HTML 回放已不再直接展示智能体的真实历史轨迹，而是只展示每一帧当前 `active_plan` 对应的规划路径，并统一以虚线方式表达计划层
- 当前 HTML 回放右上角 summary 已改为混合口径展示：`Coverage / Valid Cells / Stale Cells` 表示当前帧状态，`UAV Checked Marks / Confirmed Hotspots / False Alarms` 表示截至当前步的累计热点事件总量
- 当前 `USV` 执行层已增加真实障碍穿越防护：若一步运动会进入多边形障碍或离岸孤岛，会在最后安全位置截断并清空路径，下一步重新规划
- 当前 `USV` 的速度控制已从过于保守的对准减速中放宽，使实际速度更接近建模时的船速基线

补充说明：

- 当前回放预览基于第一阶段闭环动力学、栅格覆盖和信息地图联动生成
- 当前回放采用 `USV` 固定巡航 + `UAV` 割草机搜索的巡航层，以及简化启发式热点响应
- 当前 `UAV` 已不再是无限续航体，而是带有低电量会合补能约束的搜索平台
- 当前每次生成回放 HTML 时，都会同步生成一份逐事件日志和一份最终汇总日志，便于不依赖页面直接分析算法行为
- 当前日志已经能记录“为什么指向这个目标、当前计划去哪里、执行偏差有多大、热点是如何被怀疑与确认”的基础信息
- 它主要用于可视化观察，不等同于完整的动态仿真正式实现

### 2.9 海图输出

当前系统已经能把海域、障碍和静态智能体一起输出成 HTML 海图。

这部分的作用是：

- 把当前环境层直接展示出来
- 让后续所有动态效果都能在这个底图上继续叠加

当前海图风格补充：

- 背景为白色
- 分区采用浅灰边界和轻标签
- 风险区采用浅红轮廓或淡红阴影
- 障碍采用深灰实体
- `USV` 覆盖采用浅蓝
- `UAV` 覆盖采用浅青绿
- 重叠覆盖采用浅金色
- 默认输出初始为 `clean` 的海图
- 同一 HTML 内可切换到 `debug` 视图，显示静态覆盖预览和 `footprint` 轮廓
- 同一 HTML 内可切换智能体编号标签显示与隐藏
- 当前规划路径图层位于 `footprint` 之上、本体与标签之下，并以虚线显示每一帧当前 `active_plan`

## 3. 当前系统实际运行流程

如果只看当前已经实现的系统，真实流程如下：

1. 解析命令行参数，确定运行模式、仿真步数、随机种子、输出路径以及是否生成 HTML
2. 若为仿真模式，则先解析统一实验配置
   - 可直接使用代码默认 baseline
   - 也可通过 `--config <toml>` 加载显式实验配置
   - 命令行 `--seed / --steps` 会覆盖配置文件中的同名参数
3. 若为批量实验模式，则解析 `--batch-config <toml>`
   - 每个 run 可绑定一个实验配置文件和一组覆盖参数
   - 当前批量实验会顺序执行所有 runs
   - 每轮 run 独立输出日志，批量层再统一汇总
4. 构建当前海域、障碍环境与 `25m` 栅格网络
5. 初始化 `3 个 USV + 2 个 UAV`、信息地图、任务记录、执行状态与反馈状态
6. 根据默认场景装配巡航数据
   - `USV-1` 负责近海覆盖式巡航
   - `USV-2/USV-3` 分别负责远海上/下半区巡航
   - `UAV-1/2` 分别负责远海上下子区域割草机搜索
7. 每个仿真步执行以下闭环：
   - 推进信息年龄，并按区域阈值更新 `valid/stale`
   - 维护近海基础任务点与海域 `12` 个热点源
   - `UAV` 搜索并发现热点，`USV` 在任务点固定驻留 `5` 步完成确认
   - 低电量 `UAV` 生成 `uav_resupply`，前往最近 `USV` 会合补能
   - 任务层生成/排序/分配 `baseline_service + hotspot_confirmation + uav_resupply`
   - 路径层为 `USV/UAV` 生成巡航、任务前往、会合或回巡航路径
     - 当前 `USV` 可按实验配置切换 `astar_path_planner / astar_smoother_path_planner / hybrid_astar_path_planner`
     - 当前 `USV` 主线对比重点已转为 `astar` 与 `astar + smoother`
   - 执行层跟踪当前路径，并在需要时触发局部避障、局部恢复与条件重规划
   - 更新覆盖映射、热点状态、信息地图与任务关闭结果
   - 采集逐步日志、规划统计和回放帧数据
8. 仿真结束后输出：
   - `HTML` 回放文件（可选）
   - `events.jsonl`
   - `summary.json`
9. 若为批量实验模式，则在所有 runs 结束后额外输出：
   - `batch_results.jsonl`
   - `batch_summary.json`
10. 日志与 summary 会同时写入当前解析后的实验配置摘要，便于后续对比实验复现
11. 若不是仿真模式，则输出静态海图 HTML

## 4. 当前系统输出的是什么

当前系统运行后，实际会输出两类结果：

1. 静态海图 HTML
2. 回放式仿真产物
   - `HTML` 回放页面
   - `events.jsonl`
   - `summary.json`

当前回放页面已经能显示：

- 海域、障碍、风险区
- `3 个 USV + 2 个 UAV` 的动态运动
- `footprint` 开关
- 编号标签开关
- 时间步滑块与播放控制
- `valid cells / stale cells`
- 基础任务点图层
- 活跃热点图层
- 当前规划路径虚线图层
- 事件日志面板与统计摘要

当前回放页面的几个关键口径是：

- `valid/stale cells` 只统计非障碍格
- 热点任务一旦完成，会从活跃热点图层中移除
- 页面当前不再展示智能体真实历史轨迹，只展示每一帧当前 `active_plan` 的规划路径虚线
- 当前帧数据采用前端即时渲染，而不是把每一帧完整 SVG 图层预渲染进 HTML

当前回放日志与 summary 还会额外输出：

- 当前解析后的 `experiment_config`
- 当前 baseline 选择的任务分配、路径规划和执行策略名称
- 当前信息地图关键参数摘要

当前批量实验层还会额外输出：

- `batch_results.jsonl`
  - 每个 run 一条记录
  - 包含 run label、配置摘要、结果路径和最终指标
- `batch_summary.json`
  - 汇总成功/失败 run 数量
  - 汇总成功 runs 的平均覆盖率、信息新鲜度、热点确认等基础指标

所以当前系统本质上已经是：

- 一个可运行的基础版异构多智能体闭环仿真系统
- 一个带日志输出的回放式分析系统
- 一个可继续升级任务层、规划层、执行层与控制层的第一阶段基线实现
