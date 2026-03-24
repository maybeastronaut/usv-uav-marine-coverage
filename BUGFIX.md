# BUGFIX

本文档不再保留已闭环修复 bug 的逐条过程，只保留两类信息：

- 当前仍未完全收口的残留问题
- 后续编程时必须注意的警示项

已修复问题的细节如需追溯，请直接查看 git 历史和相关测试。

日志基准：

- `outputs/usv_uav_simulation_replay_events.jsonl`
- `outputs/usv_uav_simulation_replay_summary.json`

## 当前残留问题

基于当前同配置复跑结果，当前**没有已确认的执行一致性 bug 残留**。

最近一轮修复后，下面四类问题都已经收住：

- `hotspot-confirmation-9-19 / 8-19` 在上中部风险带出口附近的重复 `task_final_approach_exhausted`
- `uav_resupply` support `USV` 已经脱钩，但任务层还晚一步继续保留 `assigned + support_agent_id`
- `uav_resupply` support `USV` 因实时追 `UAV` 位置而反复 `task_plan_blocked`
- failure suppression 在故障切换时把同一热点任务打回后又立刻原样派回，形成 `assigned -> requeued -> assigned` flip-flop

当前仍需要持续观察的是**结果层波动**，不是明确 bug：

- 同配置 replay 的覆盖率/热点收敛结果可能随调度链路变化而明显波动
- 因此后续每次改 execution / recovery / resupply sync / failure suppression 后，都必须重新复跑同场景验证

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
