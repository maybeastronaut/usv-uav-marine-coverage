import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch, sentinel

import numpy as np

from usv_uav_marine_coverage.rl import (
    OBSERVATION_SIZE,
    CentralizedUsvAllocatorEnv,
    allocate_tasks_with_rllib_ppo_policy,
    build_centralized_usv_allocation_context,
)
from usv_uav_marine_coverage.rl import policy_allocator
from usv_uav_marine_coverage.simulation.experiment_config import (
    AlgorithmSelection,
    RLConfig,
    build_default_experiment_config,
    load_experiment_config,
    validate_experiment_config,
)
from usv_uav_marine_coverage.simulation.simulation_core import (
    advance_simulation_runtime_step,
    build_replay_from_runtime,
    build_simulation_replay,
    initialize_simulation_runtime,
    prepare_simulation_runtime_step,
)
from usv_uav_marine_coverage.tasking.task_types import (
    TaskRecord,
    TaskSource,
    TaskStatus,
    TaskType,
)


class RlAllocatorConfigTestCase(unittest.TestCase):
    def test_validate_rl_allocator_requires_checkpoint_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "missing_checkpoint.toml"
            path.write_text(
                "\n".join(
                    [
                        "[simulation]",
                        "seed = 20260324",
                        "steps = 5",
                        "dt_seconds = 1.0",
                        "",
                        "[scenario]",
                        'name = "offshore_hotspot_pressure"',
                        "",
                        "[algorithms]",
                        'task_allocator = "rllib_ppo_usv_allocator"',
                        'zone_partition_policy = "weighted_voronoi_partition_policy"',
                        'usv_path_planner = "astar_smoother_path_planner"',
                        'uav_search_planner = "uav_lawnmower_planner"',
                        'execution_policy = "local_mpc_execution"',
                    ]
                ),
                encoding="utf-8",
            )
            config = load_experiment_config(path)

        with self.assertRaisesRegex(ValueError, "checkpoint_path"):
            validate_experiment_config(config)


class SimulationRuntimeExtractionTestCase(unittest.TestCase):
    def test_runtime_helpers_match_build_simulation_replay(self) -> None:
        config = build_default_experiment_config(
            scenario_name="baseline_patrol",
            seed=20260324,
            steps=6,
            dt_seconds=1.0,
        )
        replay = build_simulation_replay(experiment_config=config)

        runtime = initialize_simulation_runtime(experiment_config=config)
        while runtime.current_step < config.simulation.steps:
            prepare_simulation_runtime_step(runtime)
            advance_simulation_runtime_step(runtime)
        runtime_replay = build_replay_from_runtime(runtime)

        self.assertEqual(len(runtime_replay.frames), len(replay.frames))
        self.assertEqual(runtime_replay.frames[-1].coverage_ratio, replay.frames[-1].coverage_ratio)
        self.assertEqual(
            runtime_replay.step_logs[-1]["coverage"]["coverage_ratio"],
            replay.step_logs[-1]["coverage"]["coverage_ratio"],
        )
        self.assertEqual(
            runtime_replay.step_logs[-1]["information_map"]["valid_cells"],
            replay.step_logs[-1]["information_map"]["valid_cells"],
        )


class CentralizedUsvAllocatorEnvTestCase(unittest.TestCase):
    def test_env_reset_and_step_return_fixed_observation_shape(self) -> None:
        env = CentralizedUsvAllocatorEnv(
            {
                "base_config_path": str(
                    Path("configs/offshore_hotspot_pressure_rho_weighted_voronoi.toml").resolve()
                ),
                "scenario_name": "offshore_hotspot_pressure",
                "episode_steps": 12,
                "dt_seconds": 1.0,
                "seed_sequence": [20260401],
            }
        )
        observation, info = env.reset()

        self.assertEqual(observation.shape, (OBSERVATION_SIZE,))
        self.assertEqual(info["seed"], 20260401)

        next_observation, reward, terminated, truncated, step_info = env.step(np.array([0, 0, 0]))

        self.assertEqual(next_observation.shape, (OBSERVATION_SIZE,))
        self.assertIsInstance(reward, float)
        self.assertFalse(truncated)
        self.assertIn("invalid_action_count", step_info)
        self.assertIn("coverage_ratio", step_info)
        self.assertFalse(terminated)

    def test_candidate_context_is_stable_for_same_seed(self) -> None:
        env = CentralizedUsvAllocatorEnv(
            {
                "base_config_path": str(
                    Path("configs/offshore_hotspot_pressure_rho_weighted_voronoi.toml").resolve()
                ),
                "scenario_name": "offshore_hotspot_pressure",
                "episode_steps": 12,
                "dt_seconds": 1.0,
                "seed_sequence": [20260402],
            }
        )
        obs_a, _ = env.reset(seed=20260402)
        slot_ids_a = tuple(
            tuple(slot.task_id for slot in view.slots)
            for view in env._pending_context.agent_views
        )
        obs_b, _ = env.reset(seed=20260402)
        slot_ids_b = tuple(
            tuple(slot.task_id for slot in view.slots)
            for view in env._pending_context.agent_views
        )

        self.assertTrue(np.array_equal(obs_a, obs_b))
        self.assertEqual(slot_ids_a, slot_ids_b)


class RlAllocatorInferenceTestCase(unittest.TestCase):
    def test_load_rllib_algorithm_registers_env_for_inference(self) -> None:
        policy_allocator._load_rllib_algorithm.cache_clear()
        with patch(
            "usv_uav_marine_coverage.rl.env.register_centralized_usv_allocator_env",
            return_value="centralized_usv_allocator_env",
        ) as register_env_mock, patch(
            "ray.rllib.algorithms.algorithm.Algorithm.from_checkpoint",
            return_value=sentinel.algorithm,
        ) as from_checkpoint_mock:
            algorithm = policy_allocator._load_rllib_algorithm("/tmp/test-checkpoint")

        self.assertIs(algorithm, sentinel.algorithm)
        register_env_mock.assert_called_once_with()
        from_checkpoint_mock.assert_called_once_with("/tmp/test-checkpoint")
        policy_allocator._load_rllib_algorithm.cache_clear()

    def test_compute_rllib_action_uses_rl_module_inference(self) -> None:
        class _FakeActionTensor:
            def __init__(self, values):
                self._values = np.asarray(values, dtype=np.int64)

            def detach(self):
                return self

            def cpu(self):
                return self

            def numpy(self):
                return self._values

        class _FakeModule:
            def forward_inference(self, batch):
                self.last_batch = batch
                return {"actions": _FakeActionTensor([[1, 2, 5]])}

        class _FakeAlgorithm:
            def __init__(self):
                self.module = _FakeModule()

            def get_module(self, policy_id):
                self.last_policy_id = policy_id
                return self.module

        algorithm = _FakeAlgorithm()
        with patch(
            "usv_uav_marine_coverage.rl.policy_allocator._load_rllib_algorithm",
            return_value=algorithm,
        ):
            action = policy_allocator._compute_rllib_action(
                np.zeros((OBSERVATION_SIZE,), dtype=np.float32),
                checkpoint_path="/tmp/test-checkpoint",
                policy_id="default_policy",
            )

        self.assertEqual(action, (1, 2, 5))
        self.assertEqual(algorithm.last_policy_id, "default_policy")

    def test_stub_inference_assigns_only_usv_side(self) -> None:
        config = build_default_experiment_config(
            scenario_name="offshore_hotspot_pressure",
            seed=20260324,
            steps=12,
            dt_seconds=1.0,
        )
        config = config.__class__(
            simulation=config.simulation,
            scenario=config.scenario,
            coordination=config.coordination,
            algorithms=AlgorithmSelection(
                task_allocator="rho_task_allocator",
                zone_partition_policy="weighted_voronoi_partition_policy",
                usv_path_planner="astar_smoother_path_planner",
                uav_search_planner="uav_lawnmower_planner",
                execution_policy="local_mpc_execution",
            ),
            information_map=config.information_map,
            rl=RLConfig(),
            events=config.events,
        )
        runtime = initialize_simulation_runtime(experiment_config=config)
        prepare_simulation_runtime_step(runtime)
        context = build_centralized_usv_allocation_context(
            runtime.task_records,
            agents=runtime.agents,
            execution_states=runtime.execution_states,
            grid_map=runtime.grid_map,
            info_map=runtime.info_map,
            step=runtime.pending_step.step,
            usv_path_planner=config.algorithms.usv_path_planner,
            zone_partition_policy=config.algorithms.zone_partition_policy,
            checkpoint_name="stub_checkpoint",
        )

        valid_first_slot = 0
        for candidate in context.agent_views[0].slots[1:]:
            if candidate.task_id is not None:
                valid_first_slot = candidate.slot_index
                break

        with tempfile.TemporaryDirectory() as tmp_dir:
            checkpoint_dir = Path(tmp_dir) / "checkpoint"
            checkpoint_dir.mkdir()
            with patch(
                "usv_uav_marine_coverage.rl.policy_allocator._compute_rllib_action",
                return_value=(valid_first_slot, 0, 0),
            ):
                updated_tasks, decisions = allocate_tasks_with_rllib_ppo_policy(
                    runtime.task_records,
                    agents=runtime.agents,
                    execution_states=runtime.execution_states,
                    grid_map=runtime.grid_map,
                    info_map=runtime.info_map,
                    step=runtime.pending_step.step,
                    usv_path_planner=config.algorithms.usv_path_planner,
                    zone_partition_policy=config.algorithms.zone_partition_policy,
                    checkpoint_path=str(checkpoint_dir),
                    policy_id="default_policy",
                )

        self.assertTrue(all(decision.agent_id.startswith("USV-") for decision in decisions))
        for task in updated_tasks:
            if task.task_type.value != "uav_resupply":
                continue
            self.assertTrue(task.assigned_agent_id is None or task.assigned_agent_id.startswith("UAV-"))

    def test_rl_allocator_auto_assigns_uav_resupply_support(self) -> None:
        config = build_default_experiment_config(
            scenario_name="offshore_hotspot_pressure",
            seed=20260324,
            steps=12,
            dt_seconds=1.0,
        )
        runtime = initialize_simulation_runtime(experiment_config=config)
        uav = next(agent for agent in runtime.agents if agent.agent_id == "UAV-1")
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.PENDING,
            priority=20,
            target_x=uav.x,
            target_y=uav.y,
            target_row=None,
            target_col=None,
            created_step=0,
            assigned_agent_id="UAV-1",
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            checkpoint_dir = Path(tmp_dir) / "checkpoint"
            checkpoint_dir.mkdir()
            with patch(
                "usv_uav_marine_coverage.rl.policy_allocator._compute_rllib_action",
                return_value=(0, 0, 0),
            ):
                updated_tasks, _ = allocate_tasks_with_rllib_ppo_policy(
                    (task,),
                    agents=runtime.agents,
                    execution_states=runtime.execution_states,
                    grid_map=runtime.grid_map,
                    info_map=runtime.info_map,
                    step=0,
                    usv_path_planner="astar_smoother_path_planner",
                    zone_partition_policy="weighted_voronoi_partition_policy",
                    checkpoint_path=str(checkpoint_dir),
                    policy_id="default_policy",
                )

        updated_task = updated_tasks[0]
        self.assertEqual(updated_task.status, TaskStatus.ASSIGNED)
        self.assertEqual(updated_task.assigned_agent_id, "UAV-1")
        self.assertIsNotNone(updated_task.support_agent_id)

    def test_rl_allocator_does_not_release_existing_uav_resupply_support(self) -> None:
        config = build_default_experiment_config(
            scenario_name="offshore_hotspot_pressure",
            seed=20260324,
            steps=12,
            dt_seconds=1.0,
        )
        runtime = initialize_simulation_runtime(experiment_config=config)
        uav = next(agent for agent in runtime.agents if agent.agent_id == "UAV-1")
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=uav.x,
            target_y=uav.y,
            target_row=None,
            target_col=None,
            created_step=0,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
            rendezvous_anchor_x=uav.x,
            rendezvous_anchor_y=uav.y,
        )
        context = build_centralized_usv_allocation_context(
            (task,),
            agents=runtime.agents,
            execution_states=runtime.execution_states,
            grid_map=runtime.grid_map,
            info_map=runtime.info_map,
            step=0,
            usv_path_planner="astar_smoother_path_planner",
            zone_partition_policy="weighted_voronoi_partition_policy",
            checkpoint_name="stub_checkpoint",
        )

        applied = policy_allocator.apply_centralized_usv_allocation_actions(
            (task,),
            context=context,
            action=(5, 0, 0),
            agents=runtime.agents,
            execution_states=runtime.execution_states,
            step=0,
        )

        updated_task = next(item for item in applied.task_records if item.task_id == task.task_id)
        self.assertEqual(updated_task.status, TaskStatus.ASSIGNED)
        self.assertEqual(updated_task.support_agent_id, "USV-1")
        self.assertGreaterEqual(applied.invalid_action_count, 1)

    def test_rl_allocator_protects_low_energy_resupply_support_usv_from_normal_slots(self) -> None:
        config = build_default_experiment_config(
            scenario_name="offshore_hotspot_pressure",
            seed=20260324,
            steps=12,
            dt_seconds=1.0,
        )
        runtime = initialize_simulation_runtime(experiment_config=config)
        agents = tuple(
            replace(agent, x=110.0, y=180.0, energy_level=20.0)
            if agent.agent_id == "UAV-1"
            else replace(agent, x=112.0, y=182.0)
            if agent.agent_id == "USV-1"
            else agent
            for agent in runtime.agents
        )
        uav = next(agent for agent in agents if agent.agent_id == "UAV-1")
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=uav.x,
            target_y=uav.y,
            target_row=None,
            target_col=None,
            created_step=0,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
            rendezvous_anchor_x=uav.x,
            rendezvous_anchor_y=uav.y,
        )
        context = build_centralized_usv_allocation_context(
            (task,),
            agents=agents,
            execution_states=runtime.execution_states,
            grid_map=runtime.grid_map,
            info_map=runtime.info_map,
            step=0,
            usv_path_planner="astar_smoother_path_planner",
            zone_partition_policy="weighted_voronoi_partition_policy",
            checkpoint_name="stub_checkpoint",
        )

        usv1_view = next(view for view in context.agent_views if view.agent_id == "USV-1")
        self.assertTrue(usv1_view.resupply_protected)
        self.assertIsNone(usv1_view.slots[1].task_id)
        self.assertIsNone(usv1_view.slots[2].task_id)
        self.assertIsNone(usv1_view.slots[3].task_id)
        self.assertEqual(usv1_view.slots[4].task_id, task.task_id)
        self.assertEqual(usv1_view.slots[4].task_type, TaskType.UAV_RESUPPLY)

        applied = policy_allocator.apply_centralized_usv_allocation_actions(
            (task,),
            context=context,
            action=(1, 0, 0),
            agents=agents,
            execution_states=runtime.execution_states,
            step=0,
        )

        updated_task = next(item for item in applied.task_records if item.task_id == task.task_id)
        self.assertEqual(updated_task.status, TaskStatus.ASSIGNED)
        self.assertEqual(updated_task.support_agent_id, "USV-1")
        self.assertGreaterEqual(applied.invalid_action_count, 1)
