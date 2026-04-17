# RL configs

集中式 `RLlib PPO` 的第一版 `USV` 高层任务分配配置。

当前目录约定：

- `ppo_usv_offshore_hotspot_train.toml`：训练配置
- `ppo_usv_offshore_hotspot_eval.toml`：训练后回放/评估配置
- `ppo_usv_offshore_hotspot_3seed_ppo_only.toml`：只跑 `PPO` 的三 seed 批量评估
- `ppo_usv_offshore_hotspot_3seed_compare.toml`：`PPO vs RHO vs cost-aware` 的三 seed 批量对比

训练脚本会把稳定 checkpoint 同步到：

- `outputs/rl/train_runs/ppo_usv_offshore_hotspot/latest_checkpoint`
