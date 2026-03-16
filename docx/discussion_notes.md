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

- 当前下一步优先做：在固定任务层和 `UAV lawnmower` 基线下，正式比较 `astar_path_planner` 与 `astar_smoother_path_planner`
- 原因：当前更需要一个“比 baseline 更平滑、但不显著放大状态空间和仿真开销”的 `USV` planner，中间版 `A* + smoother` 更适合作为下一轮主线对照
- `hybrid_astar_path_planner` 继续保留，但暂时降级为探索型 planner；只有在 `A* + smoother` 证明收益不足时，再继续集中调优 hybrid 参数
- 最后再做：`CBBA / auction allocator`
- 原因：更适合作为第二种任务层对比算法，用来和当前已落地的集中式代价分配器形成对照

### 1.3 任务层升级方向

- 当前任务层已经具备：
  - 基础责任分区 + 启发式分配
  - 第一版 `cost-aware centralized allocator`
- 当前最近确认的直接改进点：
  - 对“已有责任区候选但当前不可达”的任务增加短时冷却，避免同一热点在连续多个 step 里被重复做 `A*` 可达性检查
- 后续可升级方向包括：
  - 更复杂优先级规则
  - 弱分区与跨区支援
  - 多任务调度
  - 更高级任务分配算法
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
- 当前已知现象是：即使第二版 persistent planner 加入区域承诺、事件触发重排、AOI 去冲突和 sweep 端点接入，它在 `offshore_hotspot_pressure` 下仍可能略弱于固定 lawnmower；后续排查重点应放在“双机协同”和“区域内未完成覆盖量”而不是继续增加单机即时 freshness 贪心
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
