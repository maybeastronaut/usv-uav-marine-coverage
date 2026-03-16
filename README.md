# usv-uav-marine-coverage

Python project scaffold for follow-up development with Codex.

Current Python package name: `usv_uav_marine_coverage`. Use this package name for imports,
module execution, and future tests.

## Python Version

- Recommended: Python 3.11+
- Current local environment is configured with Python 3.11.15

## Project Structure

```text
.
|-- AGENTS.md
|-- README.md
|-- configs/
|   |-- cost_aware_allocator.toml
|   |-- planner_path_stress_cost_aware.toml
|   |-- planner_path_stress_cost_aware_astar_smoother.toml
|   |-- return_to_patrol_stress_cost_aware.toml
|   |-- return_to_patrol_stress_cost_aware_astar_smoother.toml
|   |-- offshore_hotspot_pressure_basic.toml
|   |-- offshore_hotspot_pressure_cost_aware.toml
|   |-- offshore_hotspot_pressure_cost_aware_astar_smoother.toml
|   |-- cost_aware_uav_persistent_multi_region.toml
|   |-- astar_smoother_baseline.toml
|   |-- hybrid_astar_baseline.toml
|   |-- experiment_datasets/
|   |   |-- README.md
|   |   |-- task_allocator_offshore_hotspot_pressure_3seed_cooldown/
|   |   |   |-- README.md
|   |   |   |-- batch.toml
|   |   |   `-- ...
|   |   |-- usv_planner_offshore_hotspot_pressure_3seed_800/
|   |   |   |-- README.md
|   |   |   |-- batch.toml
|   |   |   `-- ...
|   |   |-- usv_planner_return_to_patrol_stress_3seed_1200/
|   |   |   |-- README.md
|   |   |   |-- batch.toml
|   |   |   `-- ...
|   |   `-- task_allocator_offshore_hotspot_pressure_5seed/
|   |       |-- README.md
|   |       |-- batch.toml
|   |       `-- ...
|   |-- phase_one_batch.toml
|   |-- scenario_comparison_batch.toml
|   |-- uav_multi_region_coverage.toml
|   |-- uav_persistent_multi_region_coverage.toml
|   `-- phase_one_baseline.toml
|-- docx/
|   |-- current_system_flow.md
|   |-- discussion_notes.md
|   `-- generated/
|       |-- sea_map.html
|       |-- simulation_replay_events.jsonl
|       |-- simulation_replay_summary.json
|       `-- simulation_replay.html
|-- outputs/
|   `-- ...
|-- pyproject.toml
|-- src/
|   `-- usv_uav_marine_coverage/
|       |-- __init__.py
|       |-- __main__.py
|       |-- agent_model.py
|       |-- agent_overlay.py
|       |-- execution/
|       |   |-- __init__.py
|       |   |-- basic_state_machine.py
|       |   |-- execution_types.py
|       |   |-- path_follower.py
|       |   `-- progress_feedback.py
|       |-- environment.py
|       |-- grid.py
|       |-- information_map.py
|       |-- planning/
|       |   |-- __init__.py
|       |   |-- astar_path_planner.py
|       |   |-- astar_smoother_path_planner.py
|       |   |-- direct_line_planner.py
|       |   |-- fixed_patrol_planner.py
|       |   |-- hybrid_astar_path_planner.py
|       |   |-- path_types.py
|       |   |-- uav_lawnmower_planner.py
|       |   |-- uav_multi_region_coverage_planner.py
|       |   |-- uav_persistent_multi_region_coverage_planner.py
|       |   |-- usv_path_planner.py
|       |   `-- usv_patrol_planner.py
|       |-- simulation/
|       |   |-- __init__.py
|       |   |-- scenario_catalog.py
|       |   |-- experiment_config.py
|       |   |-- simulation_agent_runtime.py
|       |   |-- simulation_core.py
|       |   |-- simulation_logging.py
|       |   |-- simulation_policy.py
|       |   |-- simulation_replay_view.py
|       |   |-- simulation_task_runtime.py
|       |   `-- uav_coverage_runtime.py
|       |-- tasking/
|       |   |-- __init__.py
|       |   |-- allocator_common.py
|       |   |-- baseline_task_generator.py
|       |   |-- basic_task_allocator.py
|       |   |-- cost_aware_task_allocator.py
|       |   |-- hotspot_task_generator.py
|       |   |-- task_types.py
|       |   `-- uav_resupply_task_generator.py
|       `-- viewer.py
`-- tests/
    |-- test_agent_model.py
    |-- test_environment.py
    |-- test_execution.py
    |-- test_experiment_batch.py
    |-- test_experiment_config.py
    |-- test_grid.py
    |-- test_information_map.py
    |-- test_planning.py
    |-- test_simulation.py
    |-- test_simulation_runtime.py
    |-- test_smoke.py
    |-- test_tasking.py
    `-- test_viewer.py
```

## File Roles

- `AGENTS.md`: Codex 在本仓库中的开发约束、编码规范、文档同步规则和协作方式说明。
- `README.md`: 项目总览、环境配置方式、运行方式、测试方式和工程结构说明。
- `configs/phase_one_baseline.toml`: 当前第一阶段 baseline 的统一实验配置样例，集中定义仿真步数、随机种子、算法选择和信息地图参数。
- `configs/cost_aware_allocator.toml`: 当前第一版 `cost-aware centralized allocator` 的实验配置样例，用于与 baseline 任务分配器做直接对比。
- `configs/offshore_hotspot_pressure_basic.toml`: 当前 `offshore_hotspot_pressure` 单种子任务层对比用的基础任务分配配置样例。
- `configs/offshore_hotspot_pressure_cost_aware.toml`: 当前 `offshore_hotspot_pressure` 单种子任务层对比用的 `cost-aware` 任务分配配置样例。
- `configs/offshore_hotspot_pressure_cost_aware_astar_smoother.toml`: 当前 `offshore_hotspot_pressure` 下切换 `USV astar + smoother` 的实验配置样例。
- `configs/planner_path_stress_cost_aware.toml`: 当前 `planner_path_stress` 单种子 `USV planner` 对比用的 `cost-aware + astar` 配置样例。
- `configs/planner_path_stress_cost_aware_astar_smoother.toml`: 当前 `planner_path_stress` 单种子 `USV planner` 对比用的 `cost-aware + astar + smoother` 配置样例。
- `configs/return_to_patrol_stress_cost_aware.toml`: 当前 `return_to_patrol_stress` 单种子 `USV planner` 对比用的 `cost-aware + astar` 配置样例。
- `configs/return_to_patrol_stress_cost_aware_astar_smoother.toml`: 当前 `return_to_patrol_stress` 单种子 `USV planner` 对比用的 `cost-aware + astar + smoother` 配置样例。
- `configs/cost_aware_uav_persistent_multi_region.toml`: 当前固定 `cost_aware` 任务层、切换第二版 `UAV persistent multi-region` 规划器的组合实验配置样例。
- `configs/astar_smoother_baseline.toml`: 当前切换 `USV A* + smoother` 规划器的基础实验配置样例，用于和默认 `astar_path_planner` 做单因素对比。
- `configs/hybrid_astar_baseline.toml`: 当前切换 `USV hybrid A* + smoother` 规划器的基础实验配置样例，用于和默认 `astar_path_planner` 做单因素对比。
- `configs/uav_multi_region_coverage.toml`: 当前第一版 `UAV multi-region coverage planner` 的实验配置样例，用于与固定割草机搜索 baseline 做直接对比。
- `configs/uav_persistent_multi_region_coverage.toml`: 当前第二版 `UAV persistent multi-region coverage planner` 的实验配置样例，用于验证事件触发重排与区域承诺式覆盖。
- `configs/experiment_datasets/README.md`: 实验数据集目录说明，约定如何组织“能突出算法特点”的可复用对比数据集。
- `configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_5seed/`: 当前任务层算法对比用的正式数据集目录，固定 `offshore_hotspot_pressure` 场景、`5` 个随机种子和 `1200 step`，同时包含 batch 配置、聚合结果以及每个 seed 的日志与汇总。
- `configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_3seed_cooldown/`: 当前任务层第一轮对比在修正 `cost_aware` 不可达任务冷却后形成的正式数据集目录，固定 `offshore_hotspot_pressure` 场景、`3` 个随机种子和 `1200 step`，同时包含 batch 配置、聚合结果以及每个 seed 的日志与汇总。
- `configs/experiment_datasets/usv_planner_offshore_hotspot_pressure_3seed_800/`: 当前 `USV` 规划层对比用的正式数据集目录，固定 `offshore_hotspot_pressure` 场景、`3` 个随机种子和 `800 step`，同时包含 batch 配置、聚合结果以及每个 seed 的日志与汇总。
- `configs/experiment_datasets/usv_planner_return_to_patrol_stress_3seed_1200/`: 当前 `USV` 规划层在 `return_to_patrol_stress` 场景下的最新正式数据集目录，固定最新 `3` 个随机种子和 `1200 step`，同时包含 batch 配置、聚合结果以及每个 seed 的日志与汇总。
- `configs/phase_one_batch.toml`: 当前批量实验配置样例，用于多随机种子或多运行标签的批量仿真。
- `configs/scenario_comparison_batch.toml`: 当前按“共享 baseline 算法 + 多个可复用实验场景”做批量对比的样例。
- `docx/current_system_flow.md`: 基于当前已有代码整理的系统实际流程说明文档。
- `docx/discussion_notes.md`: 讨论阶段确认的建模方案、状态标记和后续实现依据。
- `docx/generated/sea_map.html`: 当前默认生成的 HTML 海图产物，可直接用于查看 `clean` 模式结果。
- `docx/generated/simulation_replay.html`: 当前生成的 HTML 回放式仿真预览产物，可直接查看多智能体运动、信息刷新与热点状态变化。
- `docx/generated/simulation_replay_events.jsonl`: 当前回放式仿真同步生成的逐事件日志文件，适合 AI 按时间步复盘整体行为，并分析任务指派、路径摘要、执行偏差与热点处理链。
- `docx/generated/simulation_replay_summary.json`: 当前回放式仿真同步生成的最终汇总日志文件，适合快速评估覆盖率、热点处理和路径长度等结果。
- `outputs/`: 默认输出目录。未显式指定 `--output` 时，静态海图和回放式仿真产物都会写到这里。
- `pyproject.toml`: Python 项目配置、依赖声明、`pytest` 和 `ruff` 的工具配置入口。
- `src/usv_uav_marine_coverage/__init__.py`: 项目包入口与基础版本信息。
- `src/usv_uav_marine_coverage/__main__.py`: 命令行运行入口，用于生成静态海图或回放式仿真预览 HTML，并设置页面初始显示模式。
- `src/usv_uav_marine_coverage/agent_model.py`: `USV/UAV` 的第一阶段闭环数学模型，包含平台参数、任务到参考目标转换、轻量控制指令、受约束航向/速度更新、`UAV` 能量消耗/补能约束以及探测/覆盖半径判定，也是当前覆盖方法研究默认采用的动力学基线。
- `src/usv_uav_marine_coverage/agent_overlay.py`: 海图上的静态智能体外观示意数据与几何辅助函数。
- `src/usv_uav_marine_coverage/execution/__init__.py`: 执行层子包入口，用于承载状态机与路径执行相关模块。
- `src/usv_uav_marine_coverage/execution/basic_state_machine.py`: 基础执行状态机模块，负责 `PATROL / GO_TO_TASK / ON_TASK / RETURN_TO_PATROL` 的第一版状态切换。
- `src/usv_uav_marine_coverage/execution/execution_types.py`: 执行层共享数据结构模块，统一执行状态、执行阶段与执行结果表达。
- `src/usv_uav_marine_coverage/execution/path_follower.py`: 路径执行层模块，负责将路径段转换为前视跟踪目标，并逐步推进路径段跟踪、航点切换与 `USV` 的局部反应式避障修正。
- `src/usv_uav_marine_coverage/execution/progress_feedback.py`: 执行反馈层模块，专门负责 `USV` 无进展检测、坏目标冷却与重规划/进入 `RECOVERY` 的判定，不直接调用 planner。
- `src/usv_uav_marine_coverage/environment.py`: 海域环境、三区带结构、障碍布局与伪随机环境生成逻辑；当前障碍环境已改为“确定性自动重采样直到合法”，避免部分 `seed` 因航道或重叠约束失败；初始状态不再预放基础监测点或任务热点。
- `src/usv_uav_marine_coverage/grid.py`: 将连续海域环境映射为 `25m` 规则矩形栅格网络，并提供基于智能体 `footprint` 的动态覆盖状态更新。
- `src/usv_uav_marine_coverage/information_map.py`: 基于栅格的动态信息地图层，负责信息时效、真实热点动态生成、UAV 初检与 USV 精检两阶段热点处理流程；当前近海基础任务生成频率已进一步下调、同时活跃上限收敛为 `1` 且已服务冷却延长，并为热点生成加入区域级障碍/风险区净空判定；当前信息时效阈值已改为“近海 `800 step`、其余区域 `400 step`”。
- `src/usv_uav_marine_coverage/planning/__init__.py`: 路径规划层子包入口，用于承载巡航规划与任务路径规划相关模块。
- `src/usv_uav_marine_coverage/planning/astar_path_planner.py`: `USV` 带朝向状态的基础非完整约束 `A*` 路径规划模块，负责在栅格海域中结合初始航向、障碍约束、船体安全余量与风险代价生成可执行任务路径。
- `src/usv_uav_marine_coverage/planning/astar_smoother_path_planner.py`: `USV A* + smoother` 规划模块，负责在保留 baseline `A*` 状态空间与运动原语的前提下，对搜索结果做轻量后处理平滑，降低锯齿折返，并通过最大平滑段长约束避免把任务路径压缩成会触发连续重规划的超长直段。
- `src/usv_uav_marine_coverage/planning/hybrid_astar_path_planner.py`: 改进版 `USV hybrid A*` 规划模块，负责在更丰富的运动原语和更细采样下生成路径，并通过后处理平滑减少锯齿折返。
- `src/usv_uav_marine_coverage/planning/direct_line_planner.py`: 直达式路径规划模块，负责当前 `UAV` 的任务前往、会合补能与基础回巡航直线路径生成。
- `src/usv_uav_marine_coverage/planning/fixed_patrol_planner.py`: 固定巡航规划模块，负责将当前 demo 巡航航点生成最小 patrol path。
- `src/usv_uav_marine_coverage/planning/path_types.py`: 路径规划层共享数据结构模块，统一路径、航点与规划结果表达。
- `src/usv_uav_marine_coverage/planning/uav_lawnmower_planner.py`: `UAV` 割草机式搜索规划模块，负责生成远海分区的 boustrophedon 搜索航线，并输出当前巡航段的 patrol path。
- `src/usv_uav_marine_coverage/planning/uav_multi_region_coverage_planner.py`: `UAV` 第一版多区域覆盖规划模块，负责将远海切分为固定四个 AOI，并按区域信息新鲜度优先顺序拼接区域级搜索航线。
- `src/usv_uav_marine_coverage/planning/uav_persistent_multi_region_coverage_planner.py`: `UAV` 第二版持续多区域覆盖规划模块，负责以固定四 AOI 为基础，按 freshness debt 做事件触发重排，并通过区域承诺、AOI 去冲突与 sweep 端点接入避免短视切换和区域内覆盖断裂。
- `src/usv_uav_marine_coverage/planning/usv_path_planner.py`: `USV` 路径规划分发模块，负责在 `astar_path_planner`、`astar_smoother_path_planner` 与 `hybrid_astar_path_planner` 之间按实验配置统一切换，避免任务层和 runtime 各自硬编码 planner。
- `src/usv_uav_marine_coverage/planning/usv_patrol_planner.py`: `USV` 巡航规划模块，负责生成默认的“`1` 艘近海、`2` 艘远海”分区巡航路线，并作为默认巡航层的路径算法实现；其中 `USV-1` 当前使用近海多航线水平蛇形巡航，并在回巡航时优先接回当前巡航序列的前向航段，以尽可能重访近海各区域。
- `src/usv_uav_marine_coverage/simulation/__init__.py`: 仿真回放子包的公开门面，保持现有对外接口稳定，并协调核心仿真、日志输出和回放页面生成。
- `src/usv_uav_marine_coverage/simulation/experiment_config.py`: 统一实验配置层，负责 baseline 配置 dataclass、TOML 加载、CLI 覆盖与配置摘要序列化，为后续算法对比与批量实验提供统一入口。
- `src/usv_uav_marine_coverage/simulation/experiment_batch.py`: 批量实验入口，负责加载 batch TOML、顺序运行多组配置，并输出统一的 `batch_results.jsonl` 与 `batch_summary.json`。
- `src/usv_uav_marine_coverage/simulation/scenario_catalog.py`: 可复用实验场景目录，统一维护 `baseline_patrol / planner_path_stress / return_to_patrol_stress / offshore_hotspot_pressure / nearshore_baseline_pressure / mixed_task_pressure` 等场景预设；实验配置与 batch 运行通过场景名复用这些预设，而不是重复拷贝整套参数。
- `src/usv_uav_marine_coverage/simulation/simulation_agent_runtime.py`: 回放式仿真的智能体运行时编排模块，负责按执行阶段调度 planner、follower、反馈层与恢复逻辑；当前 `RECOVERY` 机制、局部巡航段接入和碰撞防护保留在这里，但“是否重规划 / 是否进入恢复 / 是否对坏目标冷却”的判定已下沉到 `execution/progress_feedback.py`。
- `src/usv_uav_marine_coverage/simulation/simulation_core.py`: 回放式仿真的顶层编排流程，负责按时间步组织任务层、规划层、执行层、覆盖更新、信息刷新和回放帧采集。
- `src/usv_uav_marine_coverage/simulation/simulation_logging.py`: 回放式仿真的结构化日志层，负责 `events.jsonl`、`summary.json`、任务决策摘要、路径摘要、执行偏差、热点处理链以及实验配置摘要记录。
- 当前 `events.jsonl` 与 `summary.json` 已额外输出 `A*` 规划诊断统计，包括每步总调用次数、成功/blocked 次数，以及按 `allocator / patrol / go_to_task / return_to_patrol` 分组的调用与展开节点数，便于定位长步数仿真中的性能瓶颈。
- `src/usv_uav_marine_coverage/simulation/simulation_policy.py`: 当前回放预览的兼容辅助层，负责 demo 智能体和默认 patrol 数据装配，并按实验配置调用规划层生成 `USV/UAV` 巡航路线。
- `src/usv_uav_marine_coverage/simulation/uav_coverage_runtime.py`: 第二版 `UAV persistent multi-region` 巡航辅助模块，负责初始化与解析 `UavCoverageState`，并将 AOI 选择逻辑与大运行时文件解耦。
- `src/usv_uav_marine_coverage/simulation/simulation_replay_view.py`: 回放 HTML/SVG 视图层，负责页面结构、图层渲染、时间步控件和前端交互脚本；当前 `USV` 回放插值已改为基于船头朝向的样条过渡，并修正了世界坐标航向与 SVG 旋转方向不一致的问题；当前还新增了栅格信息新鲜度图层，可直接显示 `valid/stale` 信息分布，且 `valid/stale` 统计已改为只统计非障碍格；当前回放输出已从“每帧完整 SVG 预渲染”调整为“前端按原始数据即时渲染”，并且页面只展示智能体的当前规划路径虚线，不再直接展示真实历史轨迹；当前右上角 summary 已明确区分“当前帧覆盖/信息状态”和“累计热点事件统计”，避免把瞬时热点存量误读成累计确认结果。
- `src/usv_uav_marine_coverage/simulation/simulation_task_runtime.py`: 回放式仿真的任务运行时模块，负责任务生命周期同步、任务关闭收尾和任务分配摘要序列化；当前任务完成后的 `USV` 不再接回最近全局 waypoint，而是优先接入最近局部巡航段。
- `src/usv_uav_marine_coverage/tasking/__init__.py`: 任务层子包入口，用于承载任务类型、任务生成与任务分配相关模块。
- `src/usv_uav_marine_coverage/tasking/allocator_common.py`: 任务分配器共享 helper，统一责任区过滤、已有 assignment 保留和 `uav_resupply` 专用分配逻辑。
- `src/usv_uav_marine_coverage/tasking/baseline_task_generator.py`: 基础监测任务生成模块，负责从近海动态基础监测需求同步生成 `baseline_service` 任务。
- `src/usv_uav_marine_coverage/tasking/basic_task_allocator.py`: 第一版基础任务分配算法，负责按“热点优先、同类按创建时间”排序后，在责任分区内选择可达 `USV` 中规划代价最低的执行体；当前默认规则为 `USV-1` 负责近海任务、`USV-2/USV-3` 分别负责远海上/下半区任务，同一步内的 agent-task 可达性筛选已加短期缓存，避免重复触发相同 `A*`。
- `src/usv_uav_marine_coverage/tasking/cost_aware_task_allocator.py`: 第一版代价感知集中式任务分配算法，负责在严格责任区约束下对 `baseline_service` 和 `hotspot_confirmation` 构建代价矩阵，并以“优先级分层 + 集中式贪心”方式选择总代价最低的 `USV-task` 组合；当前已加入“不可达任务冷却”机制，避免同一热点在连续多个 step 内重复触发相同的 `A*` 可达性检查；`uav_resupply` 仍保持专门分配逻辑。
- `src/usv_uav_marine_coverage/tasking/hotspot_task_generator.py`: 热点任务生成模块，负责从当前已完成 `UAV` 初检的热点状态同步生成 `hotspot_confirmation` 精检任务。
- `src/usv_uav_marine_coverage/tasking/task_types.py`: 任务数据结构模块，统一任务类型、来源、生命周期与分配结果表达。
- `src/usv_uav_marine_coverage/tasking/uav_resupply_task_generator.py`: `UAV` 补能任务生成模块，负责按最近 `USV` 可达能耗提前触发并同步生成 `uav_resupply` 会合补能任务。
- `src/usv_uav_marine_coverage/viewer.py`: 海图 HTML/SVG 渲染逻辑，负责同一 HTML 内 `clean/debug` 切换视图、标签开关、预留轨迹层下的底图、障碍、监测点、热点、静态覆盖预览与静态智能体外观输出。
- `tests/test_agent_model.py`: 验证第一阶段闭环更新下的 `UAV/USV` 机动差异、到达减速逻辑以及探测/覆盖半径判定逻辑。
- `tests/test_environment.py`: 验证默认海域尺寸、三区带范围、障碍环境生成约束以及监测点/热点生成结果。
- `tests/test_execution.py`: 验证执行层路径段跟踪、局部避障和恢复链路的关键边界。
- `tests/test_experiment_batch.py`: 验证批量实验配置读取、失败不中断和批量汇总输出。
- `tests/test_experiment_config.py`: 验证统一实验配置层、场景预设解析和实验配置合法性校验。
- `tests/test_grid.py`: 验证离散栅格网络的分辨率、坐标映射、环境属性落格结果以及 `footprint` 覆盖映射逻辑。
- `tests/test_information_map.py`: 验证信息地图的时效刷新、热点生成、`UAV` 初检、`USV` 精检与假警报流程。
- `tests/test_planning.py`: 验证 `UAV/USV` 路径规划、巡航路径生成与规划边界。
- `tests/test_simulation.py`: 验证回放式仿真预览的帧生成、智能体运动和 HTML 控件/图层输出。
- `tests/test_simulation_runtime.py`: 验证回放式仿真运行时 helper 的单步推进、任务收尾与任务状态同步边界。
- `tests/test_smoke.py`: 验证项目包是否能被基础导入。
- `tests/test_tasking.py`: 验证任务生成、基础任务分配器和 `cost-aware centralized allocator` 的规则与边界。
- `tests/test_viewer.py`: 验证 `clean/debug` 模式下海图 HTML 的关键图层、静态覆盖预览开关与智能体标识是否正确。

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e ".[dev]"
```

## Run Sea Map

```bash
python -m usv_uav_marine_coverage
```

This phase only generates and displays the left-to-right three-zone sea area map.
The generated HTML now includes:

- a built-in `clean/debug` mode switch
- an agent label visibility switch
- a reserved trajectory layer for follow-up dynamic simulation

The default `clean` initial view includes:

- a pseudo-random obstacle layout
- a deterministic auto-resampling path that keeps retrying candidate obstacle layouts until they satisfy corridor and non-overlap constraints for the chosen external `seed`
- a middle-risk-zone layout that guarantees at least `2` irregular traversable routes without drawing fixed lanes
- `3` offshore islets and `2` offshore risk areas
- a static visual mockup of `3` USVs and `2` UAVs for appearance inspection only, all starting on the left side

The static map no longer places baseline monitoring points or hotspot tasks at step `0`.
In simulation, the replay now pseudo-randomly spawns ground-truth hotspots over time, with a hard
cap of `12` simultaneously active hotspots. Hotspot seeding still requires one-cell regional
clearance, so the hotspot center cell and its surrounding neighborhood cannot overlap offshore
obstacles or risk areas. A hotspot can only enter the task layer after the `UAV` has observed and
completed the coarse inspection on that hotspot cell; only then does the system create
`hotspot_confirmation` for `USV` fine inspection. Once the `USV` fine inspection finishes, the
hotspot is removed from the ground-truth layer and disappears from the active replay layer.

In the current replay baseline, UAV patrol is no longer driven by a few fixed waypoints.
Instead, each UAV now flies a split offshore lawnmower search route, so the path layer can
express large-area search behavior before more advanced planners are introduced.

The current UAV model also includes a first energy constraint:
- UAVs consume onboard energy while searching and maneuvering
- when remaining energy is no longer enough to safely reach the nearest USV rendezvous plus a `45`-unit safety margin, a `uav_resupply` task is generated
- once the UAV reaches the nearest USV, rendezvous is considered complete and the UAV follows the USV while recharging onboard
- the UAV recharges at `12` energy units per step and returns to patrol after reaching `90%` battery

The environment layer also supports a `25m` rectangular grid network (`40 x 40`), center-point coverage mapping, and confirmed circular footprints (`USV=50m`, `UAV=100m`).

The `debug` view in the same HTML additionally exposes the static coverage preview and footprint outlines, which are only used to verify that `footprint -> grid coverage` alignment is correct.

If you only want to generate the map page without opening a browser:

```bash
python -m usv_uav_marine_coverage --no-open --output /tmp/usv_uav_sea_map.html
```

If you omit `--output`, the default static map output path is `outputs/usv_uav_sea_map.html`.

If you want reproducible obstacle generation:

```bash
python -m usv_uav_marine_coverage --seed 20260314
```

The obstacle generator now keeps the external `seed` stable and internally
resamples candidate layouts until it finds a legal one, so historically failing
seeds can still be used in batch experiments without manual filtering.

If you want the page to open with the debug view selected:

```bash
python -m usv_uav_marine_coverage --mode debug
```

If you want the replay-style simulation preview:

```bash
python -m usv_uav_marine_coverage --simulate
```

If you want to run the replay with a unified experiment configuration:

```bash
python -m usv_uav_marine_coverage --simulate --config configs/phase_one_baseline.toml
```

Every experiment config can now include a reusable scenario section:

```toml
[scenario]
name = "baseline_patrol"
```

Current built-in reusable scenarios are:

- `baseline_patrol`
- `planner_path_stress`
- `return_to_patrol_stress`
- `offshore_hotspot_pressure`
- `nearshore_baseline_pressure`
- `mixed_task_pressure`

`planner_path_stress` is a dedicated USV-planner comparison scenario: it keeps a moderate
offshore hotspot load while reducing nearshore baseline-task noise, so planner differences
are more likely to show up in cross-risk-zone travel, task ingress, and return-to-patrol behavior.

`return_to_patrol_stress` pushes this one step further for post-task recovery: it keeps hotspot
pressure moderate while lowering nearshore noise and slightly tightening information freshness,
so differences in long return-to-patrol transitions and route-shape stability are easier to see.

The scenario preset provides reusable task-pressure defaults, while the `[algorithms]`
table remains free to switch allocators and planners. If one scenario needs small local
adjustments, the `[information_map]` table can still override the preset without creating
a brand-new scenario definition.

If you want to run the first cost-aware task-allocation comparison configuration:

```bash
python -m usv_uav_marine_coverage --simulate --config configs/cost_aware_allocator.toml
```

If you want to run the first UAV multi-region coverage configuration:

```bash
python -m usv_uav_marine_coverage --simulate --config configs/uav_multi_region_coverage.toml
```

If you want to run the second UAV persistent multi-region coverage configuration:

```bash
python -m usv_uav_marine_coverage --simulate --config configs/uav_persistent_multi_region_coverage.toml
```

If you want to run a batch experiment across multiple seeds or run labels:

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/phase_one_batch.toml
```

If you want to compare several reusable scenarios under one shared baseline config:

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/scenario_comparison_batch.toml
```

If you want to rerun the current formal `basic vs cost-aware` task-allocation comparison dataset:

```bash
python -m usv_uav_marine_coverage --simulate --batch-config configs/experiment_datasets/task_allocator_offshore_hotspot_pressure_5seed/batch.toml
```

If you want a longer replay:

```bash
python -m usv_uav_marine_coverage --simulate --steps 80
```

Every replay run now also writes two machine-readable log files next to the HTML output:

- `<stem>_events.jsonl`: step-by-step simulation events and state snapshots
- `<stem>_summary.json`: final replay summary for fast review

The first metadata record in `events.jsonl` and the `simulation` section in `summary.json`
now also include the resolved `experiment_config`, so future algorithm-comparison runs can be
reproduced from one explicit configuration snapshot.

Batch experiment runs additionally produce:

- `batch_results.jsonl`: one summarized record per run
- `batch_summary.json`: aggregate metrics across all successful runs, plus failure records when a run cannot be constructed

If you omit `--output`, the default replay output base path is `outputs/usv_uav_simulation_replay.html`.

If you want to write logs only and skip HTML generation:

```bash
python -m usv_uav_marine_coverage --simulate --no-html
```

The current replay logs already include a first schema for:

- task decisions and selection reasons
- path-plan summaries and replan reasons
- execution deviation snapshots
- hotspot detection / confirmation chain records

If you want to generate a leaner HTML without the static validation layers at all, call `build_map_html(..., show_coverage_preview=False, show_footprints=False)` from code.

## Run Tests

```bash
PYTHONPATH=src python -m pytest
```

If `pytest` is not installed in the current environment yet, install project dev dependencies first:

```bash
pip install -e ".[dev]"
```

## Run Lint

```bash
ruff check .
ruff format --check .
```
