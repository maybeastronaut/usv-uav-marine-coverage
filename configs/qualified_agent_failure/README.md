# qualified_agent_failure

存放“智能体失效场景下，已经过当前回放校验并可作为合格基线复用”的单次运行配置。

当前收录规则：

- 配置包含明确的智能体失效事件，例如 `agent_failure`
- 已完成实际仿真重跑
- 生成的 replay summary / events 日志经过当前校验，没有 `step_violation`
- 若存在额外异常标记，不收录到本目录

当前已收录：

- `baseline_patrol_rho_failure_hotspot_first_soft_partition.toml`

当前已归档 HTML：

- `html/baseline_patrol_rho_failure_hotspot_first_soft_partition_seed_20260314.html`
- `html/baseline_patrol_rho_failure_hotspot_first_soft_partition_seed_20268007.html`
- `html/baseline_patrol_rho_failure_hotspot_first_soft_partition_usv1_failure_seed_20265248.html`

说明：

- 根目录下原始配置继续保留，便于历史兼容与直接比较
- 本目录只放“已验证合格”的版本；其他失效配置在完成复跑校验前先不移动进来
- `seed_20260314` 是当前主基线复跑结果，校验通过
- `seed_20268007` 是额外 5 个随机种子里综合表现最好且校验通过的一份
- `usv1_failure_seed_20265248` 是人工挑选归档的一份 `USV-1` 失效回放，校验通过
