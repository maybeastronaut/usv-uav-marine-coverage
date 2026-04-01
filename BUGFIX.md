# BUGFIX

本文档不再保留已闭环修复 bug 的逐条过程，只保留两类信息：

- 当前仍未完全收口的残留问题
- 后续编程时必须注意的警示项

已修复问题的细节如需追溯，请直接查看 git 历史和相关测试。

日志基准：

- `outputs/usv_uav_simulation_replay_events.jsonl`
- `outputs/usv_uav_simulation_replay_summary.json`

## 当前残留问题

基于最新两份复跑结果：

- [`replay_seed_20260324_residual_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_residual_fix_events.jsonl)
- [`replay_seed_20260314_residual_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260314_residual_fix_events.jsonl)

之前在 `20260324 refined` 日志里确认的 3 条控制层 bug 已经收口，当前更需要继续观察的是**结果层波动**，不是明确的状态错乱。

### 最近关闭的 3 条控制层 bug

1. `uav-resupply-UAV-1` 的 `support=None + claimed` 假 claim 已修复

- 原坏窗口：`1051-1102`
- 修复点：
  - `uav_resupply` 在 `support_agent_id is None` 时，日志层只允许 `pending_claim/unclaimed`
  - replay validation 新增 supportless-claim 不变量
- 最新验证：
  - `replay_seed_20260324_residual_fix_events.jsonl` 中
  - `supportless_claim_count = 0`
  - 原窗口内不再出现 `support_agent_id = None + claim_status = claimed/executing`

2. `USV-1` 的无任务 `return_to_patrol` recovery 累加死循环已修复

- 原坏窗口：`966-1065`
- 修复点：
  - 无任务返航在 recovery 超限后不再继续累加 `recovery_attempts`
  - 改为重建新的 return-to-patrol 目标并复位恢复预算
  - 刚重建返航目标的同一步不再立刻记成新一轮 stall
- 最新验证：
  - 原异常采样点仍可能短暂进入 `recovery`
  - 但 `recovery_attempts` 已不再一路累加，采样点均保持 `0`
  - 因此旧的“自激式恢复预算失控”现象已消失

3. `hotspot-confirmation-10-26` 的错误 `task_final_approach_exhausted` 已修复

- 原坏窗口：`603-658`
- 原释放点：`658`
- 修复点：
  - final-approach 现在区分“failed 候选”和“真正尝试过的候选”
  - 只有冻结候选真正完成一轮尝试后，才允许 exhaustion
- 最新验证：
  - `replay_seed_20260324_residual_fix_events.jsonl` 中
  - `task_final_approach_exhausted = 0`
  - 原窗口内不再出现 `released_task_reason = task_final_approach_exhausted`

### 最新控制层复验结果

- `seed = 20260324`
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `supportless_claim_count = 0`
  - `task_final_approach_exhausted = 0`
  - `task_plan_blocked = 0`
  - `coverage_ratio = 0.806735`
  - `stale_cells = 613`
- `seed = 20260314`
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `supportless_claim_count = 0`
  - `task_final_approach_exhausted = 0`
  - `task_plan_blocked = 0`
  - `coverage_ratio = 0.824088`
  - `stale_cells = 539`

当前判断：

- 这 3 条控制层 bug 已经收口
- 最新日志里没有再看到确认级的状态错乱
- 剩下更值得继续优化的是覆盖率和尾部 stale 的**结果层波动**

### 最新复核后新增记录的 2 条异常现象

基于最新 [`replay_seed_20260324_residual_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_residual_fix_events.jsonl) 的逐 step 复核，当前还应额外记录下面两条高风险现象。

1. `USV-1` 在无任务 `return_to_patrol` 下仍然长期振荡

- 范围：`881-1109`
- 现象：
  - `active_task_id = None`
  - 长时间在 `return_to_patrol <-> recovery` 之间反复切换
  - `return_target_source` 基本一直是 `fallback_patrol_waypoint`
  - `blocked_goal_signature` 长时间固定为 `return:4:45.0:320.0`
- 当前状态：
  - 旧 bug“`recovery_attempts` 一路失控累加”已经修掉
  - 但 agent 仍然没有真正脱离同一个返航死角
- 判断：
  - 这是新的控制行为问题，不应算正常返航
  - 后续应从“返航目标重建是否真正换出坏目标”这条链继续查

2. `USV-3` 的 `hotspot-confirmation-10-26` 仍然存在隐式丢单

- 范围：`601-710`
- 其中热点任务段：`601-658`
- 现象：
  - `USV-3` 持续对 `hotspot-confirmation-10-26` 做 final-approach 候选轮换
  - `603-657` 间多次出现 `go_to_task -> recovery -> 下一个 candidate`
  - `658` 这一步虽然不再显式写 `task_final_approach_exhausted`
  - 但任务仍被改成：
    - `status = requeued`
    - `assigned_agent_id = None`
    - `retry_after_step = 838`
  - 随后 `659` 起 `USV-3` 立即切去做 `uav-resupply-UAV-1`
- 判断：
  - 这不再是“显式 exhaustion”
  - 但语义上仍然是 final-approach 失败后把热点任务隐式打回市场
  - 后续需要确认释放原因链是否被吞掉，或任务是否仍被过早放弃

最近一轮修复后，下面几类旧问题仍然保持收住：

- `hotspot-confirmation-9-19 / 8-19` 在上中部风险带出口附近的重复 `task_final_approach_exhausted`
- `uav_resupply` support `USV` 已经脱钩，但任务层还晚一步继续保留 `assigned + support_agent_id`
- `uav_resupply` support `USV` 因实时追 `UAV` 位置而反复 `task_plan_blocked`
- failure suppression 在故障切换时把同一热点任务打回后又立刻原样派回，形成 `assigned -> requeued -> assigned` flip-flop

当前仍需要持续观察的另外一类问题是**结果层波动**：

- 同配置 replay 的覆盖率/热点收敛结果可能随调度链路变化而明显波动
- 因此后续每次改 execution / recovery / resupply sync / failure suppression 后，都必须重新复跑同场景验证
- 新增的“区域恢复债务”已经能把故障后远端晚爆发热点簇提前拉起，但当前第一版参数仍未把所有 seed 的 `stale_cells` 一次性压到目标线以下

### 近期区域恢复债务验证结论

本轮新增了一条仅作用于 `failure_hotspot_soft_partition + rho_task_allocator` 的内部机制：

- 对 `HOTSPOT_CONFIRMATION` 按 5x5 邻域计算 `regional_recovery_debt`
- 高债务热点可以跳出 failure-mode 的非 focus suppression
- `rho` 对故障后热点分配加入 `regional_recovery_bonus`

当前结论是：

- 这条机制**没有引入新的执行一致性 bug**
- 它对“故障后远端晚爆发热点簇长期失养”这个范式问题是有效的
- 但第一版参数仍偏保守，改善已经出现，指标还没有在所有目标 seed 上完全达线

最近一次验证结果：

- `seed = 20260314`
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `has_errors = false`
  - `coverage_ratio = 0.824818`
  - `stale_cells = 554`
- `seed = 20260324`
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `has_errors = false`
  - `coverage_ratio = 0.825769`
  - `stale_cells = 585`

对 `seed = 20260324`，右下晚爆发热点簇：

- `hotspot-confirmation-34-38`
- `hotspot-confirmation-36-36`
- `hotspot-confirmation-38-36`
- `hotspot-confirmation-38-38`

在本轮回放中已经全部完成，说明“区域债务驱动 surviving USV 提前去补远端簇”这条链路已经生效。

但当前还需要注意：

- `stale_cells = 585` 仍高于这轮目标线 `576`
- 当前日志里真正记录到的高权重区域债务拉起只有少数几次，说明第一版阈值偏高，很多长期 `pending` 热点还没有进入候选
- 后续如果继续优化，应优先微调：
  - `regional_recovery_debt.py` 的高债务阈值
  - 以及高债务放行条件
- 不要直接去改：
  - `UAV_RESUPPLY`
  - planner / recovery / final-approach
  - proximity override 半径

当前判断：

- 这条机制已经证明“方向正确”
- 但目前应视为**第一版保守实现**，还不是最终参数

### 近期固定 UAV-USV 支援配对验证结论

本轮新增了一条更明确的支援偏好机制：

- `UAV-1 -> USV-1`
- `UAV-2 -> USV-3`
- `USV-2` 仅作为 fallback，尽量留给基础区/常规任务

实现落点主要在：

- `tasking/uav_support_policy.py`
- `tasking/allocator_common.py`
- `tasking/partitioning/weighted_voronoi.py`
- `tasking/partitioning/failure_hotspot_soft_partition.py`

当前验证结论要分两层看：

1. 已收住的部分

- 日志一致性没有回归：
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `silent_requeue_count = 0`
- `UAV-1` 的支援绑定已经稳定偏向 `USV-1`
- 对应测试已经改成新语义并全部通过

2. 还没有完全达到用户期望的部分

- 在 `seed = 20260324` 里，`UAV-2` 早期仍然绑定到了 `USV-2`
- 关键窗口：
  - `step 50`：`hotspot-confirmation-20-26` 已经分给 `USV-3`
  - `step 77`：`uav-resupply-UAV-2` 创建并分配时，`USV-3` 仍持有该热点
  - 因此 allocator 只能把 `UAV-2` 的 support 分给 `USV-2`
- 也就是说，当前版本实现的是：
  - **固定支援偏好 + 更早的支援保护**
  - 但还不是：
  - **严格的固定带队/任务预留**

当前判断：

- 这条机制已经能把 `USV-2` 从一部分支援场景里解放出来
- 但在“preferred support USV 已提前被热点任务占住”的场景下，还不足以强制实现：
  - `USV-3` 跟 `UAV-2`
  - `USV-1` 跟 `UAV-1`
- 如果后续要严格落实用户要求，下一步应考虑：
  - 更强的预留/抢占机制
  - 或更早把 `USV-1/3` 从热点候选里硬性排除
  - 而不只是继续调偏好排序和保护阈值

近期还尝试过一版更细粒度的尾部规则：

- 只给“等待很久且距离最近健康 `USV` 很远”的孤立热点额外 debt bonus
- 不降低全局高债务阈值
- 不放大全局 `rho` bonus

这版的结果是：

- 没有引入新的执行一致性问题
- 整套测试仍通过
- 但对 `seed = 20260324` 的最终指标没有继续改善，结果仍停在：
  - `coverage_ratio = 0.825769`
  - `stale_cells = 585`

因此当前结论是：

- “老远孤立热点 bonus”作为局部保护规则是安全的
- 但它还不足以单独解决尾部 stale 积压
- 后续若继续优化，应优先查明这些孤立热点究竟是：
  - 长期被 suppression 压住
  - 还是已经放行，但始终在 `rho`/agent availability 阶段抢不过别的任务

最近一次基准验证结果：

- `step_violation_count = 0`
- `flip_flop_count = 0`
- `has_errors = false`
- `astar_blocked_calls = 0`
- `uav_resupply task_plan_blocked = 0`

注意：

- 上述结论只针对当前基准场景
- 后续如果改动任务分配、recovery、补给接管或日志收束逻辑，仍需要重新复跑验证

## 后续编程警示

### 1. 任务层和执行层必须同时成立

后续任何分配、撤单、重入队、补给接管逻辑，都必须同时检查：

- `task_layer.tasks`
- `task_layer.task_assignments`
- `execution_layer.tracking_updates`

禁止只改任务状态、不改执行态，或者只改执行态、不改任务挂账。

### 2. assigned 不是“形式上挂着”就算成立

只要任务还是 `assigned/in_progress`，它的 owner 或 support agent 就必须满足至少一项：

- `active_task_id` 指向该任务
- `pending_assigned_task_id` 指向该任务
- `claimed_task_id` 指向该任务

如果三者都空，且 agent 已在 `return_to_patrol/patrol/failed`，就应视为任务已脱钩。

### 3. 一个 agent 同一步不能同时挂旧任务和接新任务

后续写 claim/assignment 逻辑时，必须保证：

- 旧任务先释放干净
- 再接入新任务

不允许出现：

- 旧任务仍 `assigned` 给某 agent
- 同一步该 agent 已开始执行另一任务

### 4. support USV 的补给链路要单独校验

`uav_resupply` 不能完全套普通任务规则。后续修改时要单独检查：

- `owner_agent_id` 是需要补给的 `UAV`
- `support_agent_id` 才是真正需要执行 rendezvous 的 `USV`
- support `USV` 一旦被分配，执行层不能掉回 `return_to_patrol`
- support `USV` 不要每步直接追 `UAV` 实时坐标；应围绕稳定 `rendezvous_anchor_x/y` 收敛
- `uav_resupply` 不允许走 `task_final_approach_exhausted` 这类热点/基线专用释放分支
- `uav_resupply` 的 release 信号不能只认裸 `task_id`；至少要和当前 episode 一起校验
- `uav_resupply` 即使进入 `retry_after_step` 冷却，也要防“冷却到期当步重新 claim 又立即 release”
- `uav_resupply` 一旦 support `USV` 已经不再持有任务，任务层不能再多保留一步 `assigned + support_agent_id`
- allocator 保留既有 `uav_resupply` assignment 时，不能再把 `target_x/target_y` 重写成 support `USV` 的实时坐标

### 4.1 final approach 候选不能按动态排序下标直接轮换

这次上中部 `task_final_approach_exhausted` 的根因就是：

- 候选点列表会随着 agent 位置和朝向每步重排
- 但 progress 里只保存了 `candidate_index`

结果就是：

- 同一个 `candidate_index`
- 在不同 step 可能已经不是同一个物理候选点

后续如果再改 `task_final_approach_runtime.py`，必须保证：

- 同一任务 episode 内使用稳定候选集合
- 同一任务 episode 内要记住已经失败过的候选，recovery 后跳到真正未失败的候选
- recovery 后轮换的是同一份冻结列表里的下一个候选
- 不要因为固定的 3 次预算过早 exhaustion；至少要保证冻结候选真正耗尽后再释放
- 不能再把“动态重排后的 index”当成稳定候选身份

### 5. 静止时不要留下旧速度

后续任何步末收束、recovery、dock、snap、return-to-patrol 切换，都要确认：

- `x/y` 不动时，`speed_mps/turn_rate_degps` 是否仍保留旧值
- 如果是“正常静止”，速度和角速度必须同步清零

### 6. 新不变量建议优先补上

后续最值得优先补到 `validation_layer` 的两条规则：

- `assigned/in_progress` 任务的 owner 或 support agent，必须仍被 execution layer 真正持有
- 不允许出现 `assigned + pending_claim + return_to_patrol + target=None` 的挂账状态

### 7. recovery 退出不能只切 stage，不补执行目标

这次最后一个隐蔽问题就出在这里：

- `USV` 从 `recovery` 切回 `go_to_task` 时
- `execution_state.stage` 已恢复
- 但 `agent.task.target_x/target_y` 没在同一步补回去

结果会出现单步的 `go_to_task + active_task_id != None + target=None`，日志上看像“任务恢复了”，但执行参考点是空的。

后续凡是有：

- `recovery -> go_to_task`
- `recovery -> go_to_rendezvous`
- 任何“继续原任务”的 stage 恢复

都必须保证 stage、active_task_id、execution target 在同一步一起恢复。

### 8. `uav_resupply` 的 executing 判定必须双边同时成立

后续不要再把下面两件事拆开判断：

- 任务层 `status = in_progress`
- 日志层 `claim_status = executing`

对 `uav_resupply` 来说，只有在以下条件同时满足时才成立：

- `UAV` owner 处于 `on_recharge`
- support `USV` 处于 `on_task`
- 双方都还挂着同一个 `task_id`

如果只看到 `UAV` 进入了 `on_recharge`，但 support `USV` 还在 `yield/go_to_task/return_to_patrol`，那最多只能算 `assigned/claimed`，不能提前写成真正执行中。

### 9. release 到 `return_to_patrol` 时要同时清 motion

后续任何任务释放出口，尤其是：

- `task_plan_blocked`
- `task_progress_blocked`
- `recovery_exhausted`
- `task_final_approach_exhausted`

都不能只清 `active_task_id/target`，还要同步清：

- `speed_mps`
- `turn_rate_degps`

否则日志里会出现 `idle + target=None + speed>0` 的假运动状态。

### 10. failure 邻域热点不能只改 partition，不改 proximity override

这次右下角残留问题的真正卡点不是普通 `rho` 打分，而是：

- 故障邻域热点虽然已经放出 suppression
- 但如果 `allocate_hotspot_proximity_override_task()` 仍统一使用固定 `120m`
- 更新近、但不关键的新热点会先被 `nearby_hotspot_proximity_takeover` 抢走 surviving USV

后续只要修改 failure 模式热点恢复逻辑，就必须同时检查：

- `failure_hotspot_soft_partition.py`
- `rho_task_allocator.py`
- `allocator_common.py`

三者的优先级是否一致。否则很容易出现：

- partition 层看起来已经“允许接手”
- 但 proximity override 先一步把 USV 抢走
- 最终 replay 结果仍然不改善

### 10.1 区域恢复债务只应放大真正失养的区域簇

这次第一版的经验是：

- 区域债务必须优先描述“区域失养”，不能退化成“单个旧热点提权”
- 否则系统很容易从“远端尾部 backlog”变成“零散孤立热点抢占主流程”

后续如果继续调这条链，优先顺序应是：

- 先微调高债务阈值与放行条件
- 再观察 `rho` 的 bonus 是否需要放宽
- 最后才考虑碰 focus cluster 或 proximity override

不要跳过前两步直接去扩大抢占范围，否则很容易把当前已经稳定的补给链和热点分配又打乱。

### 11. failure suppression 释放热点后，要给原 agent 冷却

这次新回归的根因是：

- failure suppression 在故障切换时会释放一个已分配热点
- 但 `nearby_hotspot_proximity_takeover` 下一步又可能把同一热点原样派回给同一个 `USV`

后续如果再改 failure 模式热点策略，必须同时保证：

- 被 suppression 释放的任务要给原 agent 写入 `agent_retry_after_steps`
- proximity override 也必须尊重这份 cooldown

否则很容易重新出现：

- `assigned -> requeued -> assigned`
- 而且 owner 还是同一个 agent

## 当前复跑结论

基于以下命令复跑：

```bash
PYTHONPATH=src python -m usv_uav_marine_coverage --simulate --config configs/baseline_patrol_rho_failure_hotspot_first_soft_partition.toml --no-open --no-html
```

当前确认：

- 现有 `validation_layer` 结果为：
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `has_errors = false`
- 当前这份基准 replay 中，`task_final_approach_exhausted` 已不再出现
- 当前这份基准 replay 中，不再存在 `uav_resupply_support_not_holding_assignment` 这类单步错位
- 当前这份基准 replay 中，`uav_resupply task_plan_blocked` 已降为 `0`
- 当前这份基准 replay 中，`astar_blocked_calls = 0`
- 当前基准 replay 的 `coverage_ratio = 0.852555`
- 当前基准 replay 的 `stale_cells = 443`

## 2026-03-24 根治补充

### A. `USV-1` 无任务返航振荡已收敛

针对 `20260324` 中：

- `USV-1 @ 881-1109`
- 长期围绕同一个 `blocked_goal_signature`
- `return_to_patrol <-> recovery` 来回切换

这次改成了：

- 无任务返航目标带短期失败记忆
- 同一坏返航点会被短期拉黑
- recovery 后必须换不同 patrol access / fallback waypoint

最新验证结果：

- [`replay_seed_20260324_root_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_root_fix_events.jsonl) 里
- `881-1109` 不再出现反复 `recovery`
- `blocked_goal_signature` 与 `return_blocked_goal_signature` 在该窗口内不再长时间重复

### B. 热点 final-approach 隐式丢单已改成“持有降级”

针对 `20260324` 中：

- `USV-3 @ 601-710`
- `hotspot-confirmation-10-26`
- 旧行为是 final-approach 多次失败后被隐式打回市场

这次改成了：

- final-approach 候选耗尽后，热点任务先保持 `assigned`
- execution 回到普通 `go_to_task`
- 进入短期 `final_approach_backoff`
- 只有后续再次耗尽到上限时，才允许真正 release

最新验证结果：

- `658` 不再出现旧的隐式 `requeued`
- `601-710` 内 `hotspot-confirmation-10-26` 持续保持 `assigned -> USV-3`
- `silent_requeue_count = 0`

### C. 调度层 requeue 现在会写出释放原因

这次补了一个容易漏掉的日志语义问题：

- execution release 原因原本已经会记录
- 但 failure suppression / policy requeue 这类调度层释放，之前不会写到 task log

现在 replay 日志会对这类场景补出显式 `release_reason`，避免再出现：

- 任务从 `assigned/in_progress` 直接变成 `requeued`
- 但同一步没有任何可追踪原因

最新两份关键 seed 校验结果：

- [`replay_seed_20260324_root_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_root_fix_events.jsonl)
- [`replay_seed_20260314_root_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260314_root_fix_events.jsonl)

两份都为：

- `step_violation_count = 0`
- `flip_flop_count = 0`
- `silent_requeue_count = 0`
- `taskless_return_recovery_loop_count = 0`
- `has_errors = false`

### D. 当前剩余判断

这轮之后，主要残留问题已经不再是控制层 bug，而是结果层指标波动：

- `20260324 root fix`: `coverage_ratio = 0.844802`, `stale_cells = 660`
- `20260314 root fix`: `coverage_ratio = 0.824088`, `stale_cells = 539`

也就是说：

- 执行一致性和日志一致性这轮已经收口
- 后续如果继续优化，应优先改区域恢复/调度效果，不要再轻易碰这轮刚稳定下来的控制链

## 2026-03-25 双锚点换侧补充

### A. 热点/基线任务已接入“左侧/右侧接近锚点”

这次没有去改 A* 内核，而是在执行层补了一层宏观接近侧：

- 先为 `HOTSPOT_CONFIRMATION` / `BASELINE_SERVICE` 冻结一对 `left/right` 接近锚点
- `GO_TO_TASK` 优先朝当前侧锚点前进
- 进入锚点附近后，才切到原有 final-approach 局部候选

这样做的目标不是“多换几个热点附近的小点”，而是让 USV 真正有能力切换整条进场方向。

### B. recovery / backoff 现在会回锚点，而不是直接继续硬冲原目标

这次改动后：

- `enroute_anchor` 段卡住时，会优先切到另一侧锚点
- final-approach 候选耗尽进入 `hold_reset` 时，也会切侧并回锚点重来
- `final_approach_backoff` 期间不再直接拿原始 `task.target_x/y` 继续硬冲

对应最新回放：

- [`replay_seed_20260324_side_anchor_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_side_anchor_fix_events.jsonl)
- [`replay_seed_20260314_side_anchor_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260314_side_anchor_fix_events.jsonl)

两份都已经能在日志里直接看到：

- `task_approach_active_side`
- `task_approach_anchor_status`
- `task_approach_anchor_left/right`

### C. 日志和 replay 校验已同步接入接近侧状态

为避免“上一任务的 left/right 状态残留到下一任务”这种误判：

- 接近侧字段只会在当前 `active_task_id` 真正匹配时写进日志
- replay 校验新增了宏观接近侧循环检查，但只盯 `enroute_anchor`，不把同侧 final-approach 内部轮换误报成错误

最新校验结果：

- [`replay_seed_20260324_side_anchor_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_side_anchor_fix_events.jsonl)
- [`replay_seed_20260314_side_anchor_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260314_side_anchor_fix_events.jsonl)

两份都为：

- `step_violation_count = 0`
- `flip_flop_count = 0`
- `silent_requeue_count = 0`
- `taskless_return_recovery_loop_count = 0`
- `task_approach_same_side_recovery_loop_count = 0`
- `has_errors = false`

### D. 当前边界

这轮已经把“算法层完全没有宏观接近侧”这个缺口补上了，但它还不等于“所有坏热点都会立刻像人一样选最优左绕路线”。

当前更准确的状态是：

- 系统已经会显式记录和执行接近侧
- backoff / recovery 已经能回到锚点层，而不是只在目标附近抖动
- 后续如果 HTML 里还出现长时间不合理接近，下一步要优先怀疑“锚点几何本身还不够好”，而不是旧的 silent requeue / no-task recovery / supportless claim 这些已修问题

## 2026-03-25 双锚点止损收窄

### A. 已确认修掉的过度接管

新一轮止损后，双锚点不再默认接管所有 `BASELINE_SERVICE` / `HOTSPOT_CONFIRMATION`：

- `BASELINE_SERVICE` 已完全退回旧逻辑
- `HOTSPOT_CONFIRMATION` 默认也不再直接进入锚点态
- 只有热点任务在 `GO_TO_TASK` 下出现明确失败证据后，才会升级进入锚点换侧

对应验证：

- [`replay_seed_20260324_anchor_stoploss_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_anchor_stoploss_events.jsonl)
- `step 27` 的 `USV-2 / baseline-service-21-1` 已不再写出 `task_approach_*`

### B. 这次止损后仍然存在的新残留

虽然 baseline 被纠正了，但热点锚点升级链仍未完全收敛。

最新 replay 校验：

- [`replay_seed_20260324_anchor_stoploss_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_anchor_stoploss_events.jsonl)
- [`replay_seed_20260314_anchor_stoploss_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260314_anchor_stoploss_events.jsonl)

现在都已经不再是 `0 errors`，主要新问题是：

- `task_approach_same_side_recovery_loop_count > 0`
- `task_approach_side_switch_without_recovery_count > 0`

其中 `20260324` 最明显的是：

- `USV-3 / hotspot-confirmation-22-34`
- `929-970`
- 在 `left/right enroute_anchor` 之间来回切，仍然属于热点锚点升级态的未收敛行为

### C. 当前判断

这说明“先止损”已经成功收回了 baseline 的错误接管，但 hotspot 的锚点升级逻辑仍然太激进：

- 默认主流程 bug 已收住
- 升级态触发后的换侧/承诺期仍需继续收敛
- 下一步不该再扩大锚点覆盖范围，而应继续收缩热点升级态本身

## 2026-03-25 Local MPC 伪推进主修

### A. 这轮修的不是“发现后补救”，而是执行层主逻辑

针对 `USV-2 @ 361-600` 这类“有位移但长期没有净进展”的问题，这轮主修改到了：

- `local_mpc.py`
- `path_follower.py`
- `progress_feedback.py`
- `task_final_approach_runtime.py`
- `simulation_logging.py`
- `replay_validation.py`

根因收敛点：

- `local_mpc` 不再只偏向“局部更安全”，还会显式惩罚低净进展和横向漂移
- `GO_TO_TASK` 的远距离阶段不再过早被 local MPC 接管
- final-approach candidate 增加了低进展计数，不再允许长期锁死在同一 candidate
- `progress_feedback` 新增 `low_progress_loop` 只作为保险，不再承担主修角色

### B. 结果

最新 replay：

- [`replay_seed_20260324_mpc_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260324_mpc_fix_events.jsonl)
- [`replay_seed_20260314_mpc_fix_events.jsonl`](/Users/tom/CodeSpace/usv-uav-marine-coverage/outputs/seed_runs/replay_seed_20260314_mpc_fix_events.jsonl)

统一校验现在都是：

- `step_violation_count = 0`
- `flip_flop_count = 0`
- `silent_requeue_count = 0`
- `taskless_return_recovery_loop_count = 0`
- `task_approach_same_side_recovery_loop_count = 0`
- `task_approach_side_switch_without_recovery_count = 0`
- `pseudo_progress_loop_count = 0`
- `has_errors = false`

其中 `20260324` 最关键的变化是：

- `USV-2 @ 361-600` 已不再 200 多步锁死在同一 `final-approach candidate`
- `step 27` 的 baseline 继续保持无锚点态，没有回归到之前的过度接管

### C. 这轮顺手修掉的校验器假阳性

在 `20260314` 的 replay 里还暴露出一条新的 replay 校验假阳性：

- `hotspot-confirmation-21-26`
- 早期一轮旧的 `right` side 退出锚点态后被错误保留
- 到 `241` 重新升级成 `left` 时，被误报成 `side_switch_without_recovery`

这条不属于真实执行 bug，而是 `replay_validation.py` 没把“退出 approach escalation 生命周期”视为 side 记忆的终止条件。

现已修正：

- recovery 期间 `active_side=None` 也会正确记录恢复窗口
- 一旦任务退出锚点态，旧 side 记忆会被清空

### D. 当前边界

这轮已经把“伪推进不被识别”这类隐蔽 bug 收住了，但还不能等价成“所有风险区附近路径都已经最优”。

当前更准确的边界是：

- 系统已经不会轻易陷入长时间 `moving but not converging` 的局部循环
- 若后续还有难点，更大概率是接近几何本身不够优，而不是旧的 local MPC 伪推进链再次复发

## 2026-03-25 顺路确认机制

### A. 机制落点

“顺路确认”这次没有做成 allocator 中途抢单，而是落在确认链本身：

- `simulation_core._confirmation_indices_for_usv(...)`
- `information_map.apply_usv_confirmation(...)`
- `simulation_task_runtime.finalize_task_resolutions(...)`

新语义是：

- 已在执行热点确认的 USV，仍然可以继续确认自己的当前目标
- 正在执行热点/基线任务，或者处于 patrol / return_to_patrol 的健康 USV
- 若物理观测到一个已有 `HOTSPOT_CONFIRMATION` 任务、且信息状态为 `UAV_CHECKED` 的热点
- 可以顺路累计该热点的 USV 确认进度

### B. 为什么不走 allocator 抢单

之前试过把“顺路确认”做成 allocator 层的 passby 抢单，但 replay 里直接暴露出：

- task/execution 脱钩
- `silent_requeue`
- 一步内旧任务和新任务同时挂账

所以这类能力必须放在“观测到热点就累计确认”的确认链上，而不是中途改任务归属。

### C. 当前结果

这次改完后：

- allocator 层的 passby 抢单逻辑已撤掉
- `20260324_passby_confirm` 和 `20260314_passby_confirm` 的 replay 校验都恢复为：
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `silent_requeue_count = 0`
  - `pseudo_progress_loop_count = 0`
  - `has_errors = false`

这条机制的核心约束也已经固定下来：

- 顺路确认只发生在“已有热点任务 + USV 实际观测到热点”的前提下
- 不通过抢单、不改分配归属来模拟“顺路”

## 2026-03-25 锚点锁存修复

### A. 新暴露的根因

在 `20260324_passby_confirm` 里，`USV-3 @ 606-785` 暴露出一条新的执行层 bug：

- `hotspot-confirmation-12-31`
- `task_approach_active_side = left`
- `task_final_approach_candidate_index = 0`
- `recovery_attempts = 0`
- `stalled_steps = 0`

但执行层会在：

- `task_approach_anchor_status = enroute_anchor`
- `task_approach_anchor_status = final_approach`

之间反复来回切，肉眼表现为一直围着同一热点的左侧接近几何打转。

问题关键不在 allocator，也不在 low-progress 检测，而在状态机本身：

- 一旦 agent 从锚点切到 `final_approach`
- 下一步又重新用“当前是否还在锚点 18m 内”判断是否继续追锚点
- agent 只要朝 candidate 走开，就会马上离开锚点半径
- 于是状态机又把它打回 `enroute_anchor`

这会造成：

- 同一侧接近几何内的锚点/候选往返振荡
- 但因为 agent 每步仍有位移，所以不会触发传统 stall/recovery

### B. 修复方式

现已在 `task_final_approach_runtime.py` 把“进入 final approach”做成锚点阶段的锁存条件：

- 只要当前任务已经进入 `task_approach_anchor_status = final_approach`
- `task_approach_anchor_reached(...)` 就保持为 `True`
- 只有 recovery / backoff / side rotate 显式把状态改回 `enroute_anchor`
- 才会重新要求 agent 回到宏观锚点

也就是说：

- 锚点只负责“进入这一侧接近”
- 一旦已经进入该侧的 final-approach
- 不能因为几何上离开锚点半径就自动退回锚点态

### C. 验证结果

新增单测覆盖了两条边界：

- 已进入 `final_approach` 时，即使 agent 几何上离开锚点，也仍视为锚点阶段已完成
- 仍处于 `enroute_anchor` 时，锚点判定继续按几何距离生效

回归验证：

- `PYTHONPATH=src python -m unittest tests.test_simulation tests.test_simulation_runtime tests.test_tasking -q`
  - `Ran 226 tests ... OK`
- `20260324_anchor_latch_fix`
  - `step_violation_count = 0`
  - `flip_flop_count = 0`
  - `silent_requeue_count = 0`
  - `task_approach_same_side_recovery_loop_count = 0`
  - `pseudo_progress_loop_count = 0`
- `20260314_anchor_latch_fix`
  - 同样保持全部 replay 校验为 `0`

### D. 当前边界

这次修的是“已经进入 final-approach 后，又被错误打回锚点态”的状态机错误。

修完后：

- `USV-3` 不会再因为离开锚点 18m 范围就自动退回 `enroute_anchor`
- 但这不等于所有热点接近几何都已经最优
- 如果后续还有接近效率问题，优先应继续看 final-approach 几何本身，而不是这个锚点锁存链

## 2026-03-25 固定带队 Escort Phase

### A. 目标

把早期“UAV 低电后再临时找 support”的软偏好，收紧成更接近固定编队的前置机制：

- `USV-2` 留在基础区
- `USV-1` 从初期开始优先伴随 `UAV-1`
- `USV-3` 从初期开始优先伴随 `UAV-2`
- 早期普通热点/基线任务优先压给 `USV-2`

### B. 本轮实际落地

这次没有把约束做成全局分区过滤，而是收敛到两条主链：

- `uav_support_policy.py`
  - 引入初始 escort phase 概念
  - 初期补给 support 改成硬绑定：
    - `UAV-1 -> USV-1`
    - `UAV-2 -> USV-3`
  - 在 escort phase 内不再回退到 `USV-2`
- `rho_task_allocator.py`
  - 初期普通 `HOTSPOT_CONFIRMATION / BASELINE_SERVICE` 只允许 `USV-2` 进入候选
  - `USV-1/USV-3` 不再在早期被普通任务拉走
- `usv_patrol_planner.py`
  - 默认 patrol 角色改成：
    - `USV-1` 上方 offshore
    - `USV-2` nearshore/base
    - `USV-3` 下方 offshore

这样做的原因是：

- 直接改全局 `build_task_partition(...)` 会把所有 allocator 的通用语义一起改坏
- 当前基准场景走的是 `rho`，因此把硬约束收在 `rho + 初期补给链` 更稳

### C. 验证结果

单测回归：

- `PYTHONPATH=src python -m unittest tests.test_planning tests.test_tasking -q`
  - `Ran 138 tests ... OK`
- `PYTHONPATH=src python -m unittest tests.test_simulation tests.test_tasking tests.test_simulation_runtime -q`
  - `Ran 233 tests ... OK`

replay 复验：

- `20260314_fixed_escort`
  - `0 violation / 0 flip-flop / has_errors=false`
- `20260324_fixed_escort`
  - 早期 escort 行为已经符合预期：
    - `step 77`
      - `USV-1 -> uav-resupply-UAV-1`
      - `USV-3 -> uav-resupply-UAV-2`
      - `USV-2 -> hotspot-confirmation-20-26`
  - 说明“固定带队”主目标已达成

### D. 当前残留

`20260324_fixed_escort` 仍暴露出两段旧类残留，不属于 escort 绑定本身，但被新 patrol 角色更容易放大：

- `USV-3 @ 449-464`
  - `taskless_return_recovery_loop`
  - `blocked_goal_signature = return:0:520.0:880.0`
- `USV-1 @ 552-567`
  - `taskless_return_recovery_loop`
  - `blocked_goal_signature = return:0:520.0:420.0`

也就是说：

- 固定 escort 本身已经生效
- 但 `20260324` 这个 seed 下，新的 offshore patrol 入口点仍会触发两段 `return_to_patrol` 残留循环
- 这条残留应继续沿 `return_to_patrol_runtime.py / recovery_runtime.py / patrol route waypoint ordering` 追

## 2026-03-25 UAV 领航 / USV 软跟随 Escort

### A. 问题本质

之前的“固定带队”只做到了配对，没有做对跟随语义：

- 一部分 step 里 `USV-1/USV-3` 还在追旧的静态 rendezvous anchor
- 一部分 step 里虽然改成了追 `UAV` 当前点，但跟得太死
- 早期 escort 还会被进度反馈层误判成 `patrol:0` 卡滞，反复打进 recovery

真正需要的是：

- `UAV` 领航
- `USV` 只需要整体朝 `UAV` 当前航向收敛
- 不必逐步追同一个精确点
- 只有真实补给/会合时才进入更严格的 rendezvous 语义

### B. 本轮落地

这次把 escort 主语义改成“影子点软跟随”：

- `stage_runtime.py`
  - 初始 escort phase 内，`USV-1/USV-3` 的跟随目标不再是 `UAV` 本体点
  - 改成按 `UAV` 当前航向生成一个影子点：
    - 离得很远时，追 `UAV` 航向前方的引导点，保证大方向一致
    - 距离进入合理带后，改成更松的尾随点
  - 这样 `USV` 会被 `UAV` 领着走，但不会因为 `UAV` 轻微转向而死追
- 同时把 escort 跟随扩展到：
  - `PATROL`
  - `RETURN_TO_PATROL`
  - 避免刚进入 escort phase 又被旧 return 目标拉偏
- 初期固定配对的 `uav_resupply` support USV：
  - 不再追静态历史 anchor
  - 改成追本步开始时的 `UAV` 实时位姿

为了防止把 escort 跟随误报成旧的 patrol 卡滞，本轮还在 `progress_feedback.py` 加了一条很窄的豁免：

- 只对“初始 escort phase 的无任务跟随”生效
- 不影响普通 patrol / hotspot / baseline / uav_resupply 的 stall 判定

### C. 验证结果

单测回归：

- `PYTHONPATH=src python -m unittest tests.test_simulation tests.test_tasking tests.test_simulation_runtime -q`
  - `Ran 238 tests ... OK`

replay 校验：

- `replay_seed_20260324_uav_leads_usv_follow`
  - `0 violation`
  - `0 flip-flop`
  - `0 silent_requeue`
  - `0 taskless_return_recovery_loop`
  - `has_errors = false`
- `replay_seed_20260314_uav_leads_usv_follow`
  - 同样全绿

关键行为上也已经能确认：

- `USV-1` 不再追上方冻结旧点，而是随着 `UAV-1` 的航向变化逐步把参考目标往下修正
- `USV-3` 对 `UAV-2` 也是同样的动态软跟随

### D. 编程警示

后续不要再把 escort 语义写回下面两种错误版本：

- “固定配对 + 静态 rendezvous anchor”
- “固定配对 + 每步死追 UAV 当前点”

更稳的版本应该一直保持：

- `UAV` 决定大方向
- `USV` 跟随方向和相对队形
- 只有真实补给会合时才做严格点对点 rendezvous

## 2026-03-25 Escort 进入风险区入口卡死

### A. 现象

在 `replay_seed_20260324_uav_leads_usv_follow` 里，`step 19` 前后会出现：

- `UAV` 已经进入中间风险区
- `USV-1/USV-3` 仍在初始 escort 跟随阶段
- `USV-3` 可以前进，但 `USV-1` 会整段停在原地
- 日志表现为：
  - `USV-1.reference_target = (268.0, 288.216)`
  - `plan_status = blocked`
  - `stalled_steps = 0`
  - 视觉上像“跟随目标正确，但一进入入口就站住”

### B. 根因

不是 `USV-1` 不愿意跟随，而是 early escort 的风险区入口目标生成得太激进：

- 旧实现会在 `UAV` 进入风险区后，直接给 `USV` 一个 corridor 内部的插值中线点
- 例如上侧 corridor 里的 `(268.0, 288.216)`
- 这个点所在 cell 虽然不是障碍格，但在该 seed 下 clearance 不足
- 结果三个 USV planner 都会把它判成 `blocked`

换句话说：

- 问题不在“UAV 领航 / USV 跟随”语义本身
- 而在“跟随目标一开始就落进了风险区坏几何”

### C. 修复

这轮把 early escort 的风险区穿越改成更保守、也更稳定的两段式：

- `UAV` 还在 nearshore 时：
  - `USV` 继续用影子点软跟随
- `UAV` 已进入风险区，而 `USV` 还没穿过去时：
  - 不再追 corridor 内部插值点
  - 改为先追各自配对 corridor 的控制点
  - 也就是先吃真正可通的 corridor 入口 / 骨架节点
- `USV` 自己穿过 corridor 后：
  - 再恢复影子点软跟随

这样做的原因是：

- 先保证入口目标一定可达
- 再谈大方向软跟随
- 避免一开始就被坏几何卡死

### D. 验证

修复后，`20260324` 的 `step 18-25` 已经变成：

- `USV-1.reference_target = (250.0, 270.343)`
- `USV-3.reference_target = (250.0, 720.082)`
- 两者 `plan_status` 都是 `planned`
- 不再出现 `step 19` 那种 `USV-1` 原地不动、整段 `blocked`

整体验证：

- `PYTHONPATH=src python -m unittest tests.test_simulation tests.test_tasking tests.test_simulation_runtime -q`
  - `Ran 239 tests ... OK`
- `PYTHONPATH=src python scripts/check_replay_logs.py outputs/seed_runs/replay_seed_20260324_uav_leads_usv_follow_events.jsonl`
  - `has_errors = false`
- `PYTHONPATH=src python scripts/check_replay_logs.py outputs/seed_runs/replay_seed_20260314_uav_leads_usv_follow_events.jsonl`
  - `has_errors = false`

### E. 编程警示

后续不要在 early escort 穿越风险区时，直接用 corridor 内部的任意插值点做 staging goal。

更稳的约束是：

- 先吃可验证可通的 corridor 控制点
- 再往 corridor 内推进
- 过 corridor 以后再恢复更自由的软跟随

另外，当前固定带队的航道语义已经明确改成：

- `UAV-1 / USV-1 -> 下航道`
- `UAV-2 / USV-3 -> 上航道`

后续不要再混回旧版本的：

- `UAV-1 / USV-1 -> 上航道`
- `UAV-2 / USV-3 -> 下航道`

另外，当前固定带队阶段还增加了一条强约束：

- `step < 180` 的初始 fixed escort phase 内
- 如果 `UAV-1 / UAV-2` 的硬绑定 escort `USV-1 / USV-3` 仍然健康可用
- 就**不生成** `uav_resupply` 任务

这样做是为了避免 escort 还没把穿航道这段走完，就被低电 rendezvous 过早抢走。

### F. Escort Corridor 状态机残留与修复

在最新的 `UAV` 领航、`USV` 跟随版本里，又确认了两条 early escort 专属 bug：

1. `USV` 会在同一条 corridor 上的控制点之间来回回切  
2. 某些 corridor 控制点虽然在可通 corridor 骨架上，但对 `USV` clearance 判据来说仍然太贴边，planner 会把它们打成 `blocked`

这两条问题叠加后，会表现成：

- `USV-1` 先追 `(250.0, 270.343)`
- 然后切去 `(292.0, 312.045)` 这类控制点
- 但该控制点实际 clearance 不够，plan 变成 `blocked`
- 于是行为上要么停在原地，要么又被重新拉回前一个点

根治策略已经收敛成两条：

- corridor 控制点 progression 必须锁存，不能按当前坐标每步回退
- 进入下一控制点前，必须先跳过 planner clearance 不足的 control point

当前实现已经按这个约束收紧：

- `initial_escort_control_point_index` 只允许单调前进
- 遇到 `clearance` 不足的 corridor 点，会直接跳到下一可执行 control point

因此，escort 穿越阶段现在修的是：

- 不回切
- 不盯着坏 control point 停住
- 先吃“下一可执行的 corridor 节点”，再恢复后续软跟随

### G. 无任务 `return_to_patrol` 残留循环根因与收口

在最新一轮 escort/corridor 修复后，又暴露出一类更后段的 patrol 残留：

- agent 已经没有活动任务
- 进入 `return_to_patrol`
- 局部重规划成功，但仍会在 `return_to_patrol -> recovery -> idle -> return_to_patrol` 之间打转
- 机器校验会把它统计成 `taskless_return_recovery_loop`

这类问题的真正根因不是“找不到 patrol 点”，而是三条状态机缝隙叠加：

1. no-task return recovery 成功后，没有完整保留 `blocked_goal` 冷却信息  
2. 重规划次数变多后，仍然优先尝试过细的 `safe_value_access`，容易再次贴边失败  
3. agent 实际已经足够接近 patrol route，但状态机仍坚持再走一轮 `return_to_patrol`

这轮已经把三条一起收口：

- no-task return recovery 成功后，继续保留：
  - `return_blocked_goal_signature`
  - `return_blocked_goal_until_step`
  - `return_replan_generation`
- 当 `return_replan_generation > 0` 时，局部 rejoin 不再优先走 `safe_value_access`
  - 会改走更粗粒度的 patrol 接入点
- 如果 agent 已经足够接近 patrol route，且经历过多轮 no-task return replan
  - 就直接 `transition_to_patrol`
  - 不再继续多跑一轮形式上的 `return_to_patrol`

因此，当前 no-task return 的策略已经变成：

- 第一次失败：保留失败记忆，换更稳的 patrol 接入点
- 多次失败后：若已经靠近 patrol route，则直接并回 patrol
- 不再允许在同一个坏 return goal 或过细 rejoin 点上无限循环

### H. 最新验证结论

以最新回放：

- `outputs/seed_runs/replay_seed_20260324_uav_leads_usv_follow_latest_events.jsonl`

为准，当前机器校验结果已经回到全绿：

- `step_violation_count = 0`
- `flip_flop_count = 0`
- `silent_requeue_count = 0`
- `taskless_return_recovery_loop_count = 0`
- `task_approach_same_side_recovery_loop_count = 0`
- `task_approach_side_switch_without_recovery_count = 0`
- `pseudo_progress_loop_count = 0`
- `has_errors = false`

对应回放文件：

- `outputs/seed_runs/replay_seed_20260324_uav_leads_usv_follow_latest.html`

当前这份 replay 说明：

- early escort/corridor 入口卡死已收住
- corridor control point 回切已收住
- no-task `return_to_patrol` 残留循环也已收住

后续如果还要继续优化，重点就不该再放在控制链 bug 上，而应转到：

- hotspot 长期 `pending`
- `uav_resupply` 长时间独占健康 `USV`
- failure 之后的区域责任与任务拉起策略
