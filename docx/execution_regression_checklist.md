# 执行层 `1200 step` 回归检查清单

用途：

- 固定一条可复现的长回放，专门用于执行层改动后的人工回归
- 避免修一个 step 级异常时，把之前已经修好的行为重新带坏
- 让后续对 `traffic / return_to_patrol / recharge / recovery / collision_guard / stage_runtime / local_mpc` 的修改都能落到同一条检查链上

当前基线：

- 配置文件：`outputs/step600_agent_failure_demo_local_mpc.toml`
- 默认长回放：
  - `outputs/step600_agent_failure_demo_local_mpc_latest.html`
  - `outputs/step600_agent_failure_demo_local_mpc_latest_events.jsonl`
  - `outputs/step600_agent_failure_demo_local_mpc_latest_summary.json`
- 仿真条件：
  - `1200 step`
  - `local_mpc_execution`
  - `step 600` 时 `USV-2 agent_failure`

推荐复跑命令：

```bash
PYTHONPATH=src .venv/bin/python -m usv_uav_marine_coverage \
  --simulate \
  --config outputs/step600_agent_failure_demo_local_mpc.toml \
  --steps 1200
```

最小 smoke：

```bash
PYTHONPATH=src .venv/bin/python -m usv_uav_marine_coverage \
  --simulate \
  --config outputs/step600_agent_failure_demo_local_mpc.toml \
  --steps 8 \
  --no-open \
  --no-html
```

## 必查时间点

### `step 50`

检查点：

- `USV-1 / USV-2` 在 `Upper Traversable Corridor` 不应双双抢道后大绕行
- 至少应有明确的 owner / yield 关系
- 不应出现“双方都进入 `RECOVERY`”的旧问题

期望观察：

- `yield_reason = corridor_wait_for_owner`
- corridor owner 明确
- 让行方在入口外等待，owner 正常通过

### `step 99`

检查点：

- 同向通过 corridor 时，不应再把后车长期压在入口外
- `USV-1` 不应表现为“看起来卡住但其实一直在傻等”

期望观察：

- 同向且前后间距足够时允许 follow-through
- `USV-1` 不应长时间反复 `YIELD <-> GO_TO_TASK`

### `step 330-398`

检查点：

- `UAV` 补能链不应抢走正在执行普通任务的 `USV`
- 某艘 `USV` 不应一边执行 `hotspot_confirmation`，一边又被当作补能支援艇拖在原地

期望观察：

- `uav_resupply` 只绑定健康且真正空闲的 `USV`
- `support_agent_id` 不应落到忙碌 `USV`

### `step 600`

检查点：

- `USV-2` 失效后应立刻退出运行链
- 当前任务应释放并进入后续接管
- 不应继续参与补能、任务分配或 traffic 邻居判定

期望观察：

- `health_status = failed`
- `speed_mps = 0`
- `task_reassignments > 0`

### `step 692+`

检查点：

- 其他 `USV` 不应围着 `USV-2` 残骸反复抖动
- 贴近残骸 keepout 的任务应被 `requeue`

期望观察：

- failed `USV` 变成静态 `wreck zone`
- `requeue_reason = "wreck_keepout"` 只在必要时触发
- 不再出现 `GO_TO_TASK -> RECOVERY` 围绕残骸循环

### `step 734`

检查点：

- `USV-3` 不应在同一小片区域里反复推翻回巡航目标
- 回巡航 access 选择应更偏“可通过”，而不是连续选到局部难接入目标

期望观察：

- 不再出现连续多轮 `RETURN_TO_PATROL -> RECOVERY -> RETURN_TO_PATROL`
- 回巡航目标切换次数明显受控

### `step 773`

检查点：

- `USV-3` 不应在恢复结束后先原地船头抖动，再晚一步继续前进
- `UAV` 也不应再被残骸 guard 错误卡在坏艇附近

期望观察：

- `USV-3` 连续前进
- `UAV` 的 `RETURN_TO_PATROL` 能正常离开 wreck 附近

### `step 921`

检查点：

- `USV-3` 不应贴着地图边缘走

期望观察：

- 边界已被纳入局部 `MPC` 的触发条件和安全代价
- 航迹会主动往内侧收

## 每次执行层改动后的最低回归要求

1. 跑短 smoke，确认主流程不崩
2. 跑这条 `1200 step` 长回放
3. 至少人工复核上面的关键时间点
4. 若行为有变化，必须同步更新：
   - `docx/current_system_flow.md`
   - `docx/discussion_notes.md`
   - 本清单

## 适用范围

这份清单优先用于以下改动后的回归：

- `local_mpc`
- `traffic_runtime`
- `return_to_patrol_runtime`
- `recharge_runtime`
- `recovery_runtime`
- `collision_guard`
- `stage_runtime`
- `simulation_agent_runtime`

若后续执行层主基线配置改变，应直接覆盖更新本文件，而不是同时保留多套口径。
