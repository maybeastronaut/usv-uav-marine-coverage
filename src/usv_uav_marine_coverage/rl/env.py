"""Gymnasium environment for centralized PPO-based USV task allocation."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from ray.tune.registry import register_env

from usv_uav_marine_coverage.simulation.experiment_config import (
    AlgorithmSelection,
    ExperimentConfig,
    build_default_experiment_config,
    load_experiment_config,
)
from usv_uav_marine_coverage.simulation.simulation_core import (
    SimulationRuntime,
    advance_simulation_runtime_step,
    initialize_simulation_runtime,
    prepare_simulation_runtime_step,
)

from .policy_allocator import (
    OBSERVATION_SIZE,
    apply_centralized_usv_allocation_actions,
    build_centralized_usv_allocation_context,
)

DEFAULT_TRAINING_SEEDS = (20260401, 20260402, 20260403, 20260404, 20260405)
CENTRALIZED_USV_ALLOCATOR_ENV_NAME = "centralized_usv_allocator_env"
_ENV_REGISTERED = False


class CentralizedUsvAllocatorEnv(gym.Env[np.ndarray, np.ndarray]):
    """One-step-per-tick centralized PPO environment for USV task allocation."""

    metadata = {"render_modes": []}

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__()
        self.env_config = dict(config or {})
        self.base_config_path = self._optional_path(self.env_config.get("base_config_path"))
        self.scenario_name = str(self.env_config.get("scenario_name", "offshore_hotspot_pressure"))
        self.episode_steps = int(self.env_config.get("episode_steps", 1200))
        self.dt_seconds = float(self.env_config.get("dt_seconds", 1.0))
        self.seed_sequence = tuple(
            int(seed) for seed in self.env_config.get("seed_sequence", DEFAULT_TRAINING_SEEDS)
        )
        if not self.seed_sequence:
            self.seed_sequence = DEFAULT_TRAINING_SEEDS
        self._episode_index = 0
        self.runtime: SimulationRuntime | None = None
        self._pending_context = None

        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(OBSERVATION_SIZE,),
            dtype=np.float32,
        )
        self.action_space = spaces.MultiDiscrete([6, 6, 6])

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        chosen_seed = seed if seed is not None else self.seed_sequence[self._episode_index % len(self.seed_sequence)]
        self._episode_index += 1
        experiment_config = self._build_episode_config(chosen_seed)
        self.runtime = initialize_simulation_runtime(experiment_config=experiment_config)
        self._pending_context = self._prepare_pending_context()
        return self._pending_context.observation.copy(), {"seed": chosen_seed}

    def step(
        self,
        action: np.ndarray | list[int] | tuple[int, ...],
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        if self.runtime is None or self._pending_context is None:
            raise RuntimeError("Environment must be reset before step().")

        previous_step_log = self.runtime.step_logs[-1]
        applied = apply_centralized_usv_allocation_actions(
            self.runtime.task_records,
            context=self._pending_context,
            action=action,
            agents=self.runtime.agents,
            execution_states=self.runtime.execution_states,
            step=self.runtime.pending_step.step if self.runtime.pending_step is not None else 0,
        )
        result = advance_simulation_runtime_step(
            self.runtime,
            allocation_override=(applied.task_records, applied.decisions),
        )
        reward = _compute_team_reward(
            previous_step_log=previous_step_log,
            current_step_log=result.step_log,
            invalid_action_count=applied.invalid_action_count,
            unnecessary_reassignments=applied.unnecessary_reassignments,
        )
        terminated = self.runtime.current_step >= self.runtime.effective_config.simulation.steps
        truncated = False
        info = {
            "step": result.step,
            "coverage_ratio": float(result.step_log["coverage"]["coverage_ratio"]),
            "invalid_action_count": applied.invalid_action_count,
            "unnecessary_reassignments": applied.unnecessary_reassignments,
        }
        if terminated:
            self._pending_context = None
            observation = np.zeros((OBSERVATION_SIZE,), dtype=np.float32)
        else:
            self._pending_context = self._prepare_pending_context()
            observation = self._pending_context.observation.copy()
        return observation, reward, terminated, truncated, info

    def _prepare_pending_context(self):
        assert self.runtime is not None
        prepare_simulation_runtime_step(self.runtime)
        assert self.runtime.pending_step is not None
        return build_centralized_usv_allocation_context(
            self.runtime.task_records,
            agents=self.runtime.agents,
            execution_states=self.runtime.execution_states,
            grid_map=self.runtime.grid_map,
            info_map=self.runtime.info_map,
            step=self.runtime.pending_step.step,
            usv_path_planner=self.runtime.effective_config.algorithms.usv_path_planner,
            zone_partition_policy=self.runtime.effective_config.algorithms.zone_partition_policy,
        )

    def _build_episode_config(self, seed: int) -> ExperimentConfig:
        if self.base_config_path is None:
            base_config = build_default_experiment_config(
                scenario_name=self.scenario_name,
                seed=seed,
                steps=self.episode_steps,
                dt_seconds=self.dt_seconds,
            )
        else:
            base_config = load_experiment_config(
                self.base_config_path,
                scenario_override=self.scenario_name,
                seed_override=seed,
                steps_override=self.episode_steps,
                dt_override=self.dt_seconds,
            )
        return replace(
            base_config,
            algorithms=AlgorithmSelection(
                task_allocator="rho_task_allocator",
                zone_partition_policy="weighted_voronoi_partition_policy",
                distributed_sync_interval_steps=1,
                distributed_broadcast_range_m=0.0,
                distributed_winner_memory_ttl_steps=0,
                distributed_bundle_length=1,
                usv_path_planner="astar_smoother_path_planner",
                uav_search_planner="uav_lawnmower_planner",
                execution_policy="local_mpc_execution",
            ),
        )

    @staticmethod
    def _optional_path(value: object) -> Path | None:
        if value is None:
            return None
        if not isinstance(value, str) or not value:
            raise ValueError("base_config_path must be a non-empty string when provided")
        return Path(value).resolve()


def register_centralized_usv_allocator_env() -> str:
    """Register the centralized PPO env once for both training and inference."""

    global _ENV_REGISTERED
    if not _ENV_REGISTERED:
        register_env(
            CENTRALIZED_USV_ALLOCATOR_ENV_NAME,
            lambda env_config: CentralizedUsvAllocatorEnv(env_config),
        )
        _ENV_REGISTERED = True
    return CENTRALIZED_USV_ALLOCATOR_ENV_NAME


def _compute_team_reward(
    *,
    previous_step_log: dict[str, Any],
    current_step_log: dict[str, Any],
    invalid_action_count: int,
    unnecessary_reassignments: int,
) -> float:
    previous_info = previous_step_log["information_map"]
    current_info = current_step_log["information_map"]
    confirmed_delta = sum(
        len(indices)
        for indices in current_step_log["task_layer"]["confirmed_by_agent"].values()
    )
    uav_checked_delta = sum(
        len(indices)
        for indices in current_step_log["task_layer"]["newly_uav_checked_by_agent"].values()
    )
    valid_delta = int(current_info["valid_cells"]) - int(previous_info["valid_cells"])
    stale_delta = int(current_info["stale_known_cells"]) - int(previous_info["stale_known_cells"])
    reward = 0.0
    reward += 2.0 * confirmed_delta
    reward += 0.5 * uav_checked_delta
    reward += 0.02 * valid_delta
    reward -= 0.02 * stale_delta
    reward -= 0.10 * invalid_action_count
    reward -= 0.05 * unnecessary_reassignments
    return float(reward)
