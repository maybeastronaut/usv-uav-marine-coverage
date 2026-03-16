import unittest
from dataclasses import replace

from usv_uav_marine_coverage.environment import (
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import build_information_map
from usv_uav_marine_coverage.planning.path_types import PathPlan, PathPlanStatus, Waypoint
from usv_uav_marine_coverage.simulation.simulation_policy import build_demo_agent_states
from usv_uav_marine_coverage.tasking.baseline_task_generator import build_baseline_tasks
from usv_uav_marine_coverage.tasking.basic_task_allocator import (
    allocate_tasks_with_basic_policy,
)
from usv_uav_marine_coverage.tasking.cost_aware_task_allocator import (
    _reachable_cost,
    allocate_tasks_with_cost_aware_policy,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskSource, TaskStatus, TaskType
from usv_uav_marine_coverage.tasking.uav_resupply_task_generator import (
    build_uav_resupply_tasks,
)


class TaskingTestCase(unittest.TestCase):
    def _build_runtime_grid_map(self):
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        return build_grid_map(sea_map, obstacle_layout)

    def _build_idle_execution_states(self, agents):
        return {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }

    def test_build_uav_resupply_tasks_creates_task_for_low_energy_uav(self) -> None:
        agents = tuple(
            agent if agent.agent_id != "UAV-1" else replace(agent, energy_level=30.0)
            for agent in build_demo_agent_states()
        )

        tasks = build_uav_resupply_tasks(agents, step=7)

        self.assertEqual(len(tasks), 1)
        self.assertEqual(tasks[0].task_type, TaskType.UAV_RESUPPLY)
        self.assertEqual(tasks[0].assigned_agent_id, "UAV-1")

    def test_build_uav_resupply_tasks_triggers_before_reserve_if_usv_is_far(self) -> None:
        agents = (
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1"),
                x=0.0,
                y=0.0,
                energy_level=90.0,
            ),
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
                x=600.0,
                y=0.0,
            ),
        )

        tasks = build_uav_resupply_tasks(agents, step=9)

        self.assertEqual(len(tasks), 1)
        self.assertEqual(tasks[0].task_type, TaskType.UAV_RESUPPLY)
        self.assertEqual(tasks[0].target_x, 600.0)
        self.assertEqual(tasks[0].target_y, 0.0)

    def test_allocator_assigns_hotspot_confirmation_only_to_usv(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="hotspot-confirmation-10-10",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=180.0,
            target_y=150.0,
            target_row=10,
            target_col=10,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(decisions[0].agent_id, "USV-1")

    def test_allocator_keeps_existing_assignment_when_still_valid(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.GO_TO_TASK
                if agent.agent_id == "USV-2"
                else ExecutionStage.PATROL,
                active_task_id="hotspot-confirmation-8-8" if agent.agent_id == "USV-2" else None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="hotspot-confirmation-8-8",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=560.0,
            target_y=610.0,
            target_row=8,
            target_col=8,
            created_step=3,
            assigned_agent_id="USV-2",
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-2")
        self.assertEqual(decisions[0].selection_reason, "keep_existing_assignment")

    def test_allocator_assigns_baseline_service_to_nearest_available_usv(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="baseline-service-6-3",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=540.0,
            target_row=6,
            target_col=3,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(decisions[0].agent_id, "USV-1")
        self.assertEqual(decisions[0].task_type, TaskType.BASELINE_SERVICE)
        self.assertEqual(
            decisions[0].selection_reason,
            "lowest_cost_reachable_partition_usv_for_baseline_service",
        )

    def test_allocator_assigns_uav_resupply_to_uav_and_nearest_support_usv(self) -> None:
        agents = tuple(
            agent if agent.agent_id != "UAV-1" else replace(agent, energy_level=30.0)
            for agent in build_demo_agent_states()
        )
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.PENDING,
            priority=20,
            target_x=150.0,
            target_y=260.0,
            target_row=None,
            target_col=None,
            created_step=5,
            assigned_agent_id="UAV-1",
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "UAV-1")
        self.assertEqual(updated_tasks[0].support_agent_id, "USV-1")
        self.assertEqual(decisions[0].agent_id, "UAV-1")
        self.assertEqual(decisions[0].support_agent_id, "USV-1")
        self.assertEqual(decisions[0].selection_reason, "nearest_usv_rendezvous_for_uav_resupply")

    def test_build_baseline_tasks_preserves_hotspot_tasks(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        info_map.state_at(4, 2).has_baseline_task = True
        info_map.state_at(4, 2).baseline_task_priority = 2
        hotspot_task = TaskRecord(
            task_id="hotspot-confirmation-8-8",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=400.0,
            target_y=320.0,
            target_row=8,
            target_col=8,
            created_step=1,
        )

        synced_tasks = build_baseline_tasks(
            info_map,
            step=6,
            existing_tasks=(hotspot_task,),
        )

        self.assertEqual(len(synced_tasks), 2)
        self.assertEqual(synced_tasks[0].task_type, TaskType.BASELINE_SERVICE)
        self.assertEqual(synced_tasks[1], hotspot_task)

    def test_heuristic_allocator_prioritizes_hotspots_then_creation_time(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        tasks = (
            TaskRecord(
                task_id="baseline-service-late",
                task_type=TaskType.BASELINE_SERVICE,
                source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
                status=TaskStatus.PENDING,
                priority=1,
                target_x=180.0,
                target_y=180.0,
                target_row=2,
                target_col=2,
                created_step=1,
            ),
            TaskRecord(
                task_id="hotspot-confirmation-newer",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=700.0,
                target_y=760.0,
                target_row=3,
                target_col=3,
                created_step=4,
            ),
            TaskRecord(
                task_id="hotspot-confirmation-older",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=700.0,
                target_y=320.0,
                target_row=4,
                target_col=4,
                created_step=2,
            ),
        )

        _, decisions = allocate_tasks_with_basic_policy(
            tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(
            [decision.task_id for decision in decisions],
            [
                "hotspot-confirmation-older",
                "hotspot-confirmation-newer",
                "baseline-service-late",
            ],
        )
        self.assertEqual(
            [decision.agent_id for decision in decisions],
            ["USV-2", "USV-3", "USV-1"],
        )

    def test_heuristic_allocator_gives_last_available_usv_to_hotspot_first(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL
                if agent.agent_id == "USV-1"
                else ExecutionStage.GO_TO_TASK
                if agent.kind == "USV"
                else ExecutionStage.PATROL,
                active_task_id=None
                if agent.agent_id == "USV-1"
                else "busy-task"
                if agent.kind == "USV"
                else None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        tasks = (
            TaskRecord(
                task_id="baseline-service-4-2",
                task_type=TaskType.BASELINE_SERVICE,
                source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
                status=TaskStatus.PENDING,
                priority=1,
                target_x=37.5,
                target_y=112.5,
                target_row=4,
                target_col=2,
                created_step=1,
            ),
            TaskRecord(
                task_id="hotspot-confirmation-12-8",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=212.5,
                target_y=312.5,
                target_row=12,
                target_col=8,
                created_step=3,
            ),
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["hotspot-confirmation-12-8"], "USV-1")
        self.assertIsNone(assigned_by_id["baseline-service-4-2"])
        self.assertEqual(decisions[0].task_id, "hotspot-confirmation-12-8")

    def test_allocator_skips_unreachable_usv_candidates(self) -> None:
        grid_map = self._build_runtime_grid_map()
        agents = tuple(
            replace(agent, x=287.5, y=487.5) if agent.agent_id == "USV-2" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="baseline-service-reachable",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=112.5,
            target_y=487.5,
            target_row=19,
            target_col=4,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(
            decisions[0].selection_reason,
            "lowest_cost_reachable_partition_usv_for_baseline_service",
        )

    def test_allocator_limits_nearshore_tasks_to_usv_1_partition(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="baseline-service-nearshore-only",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=700.0,
            target_row=20,
            target_col=7,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(decisions[0].agent_id, "USV-1")

    def test_allocator_limits_upper_offshore_tasks_to_usv_2_partition(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        task = TaskRecord(
            task_id="hotspot-confirmation-upper-offshore",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-2")
        self.assertEqual(decisions[0].agent_id, "USV-2")

    def test_allocator_limits_lower_offshore_tasks_to_usv_3_partition(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-lower-offshore",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=760.0,
            target_row=30,
            target_col=28,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_basic_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-3")
        self.assertEqual(decisions[0].agent_id, "USV-3")

    def test_cost_aware_allocator_prioritizes_hotspot_layer(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        tasks = (
            TaskRecord(
                task_id="baseline-service-nearshore",
                task_type=TaskType.BASELINE_SERVICE,
                source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
                status=TaskStatus.PENDING,
                priority=1,
                target_x=180.0,
                target_y=520.0,
                target_row=12,
                target_col=7,
                created_step=1,
            ),
            TaskRecord(
                task_id="hotspot-confirmation-upper",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=700.0,
                target_y=320.0,
                target_row=10,
                target_col=28,
                created_step=2,
            ),
        )

        _, decisions = allocate_tasks_with_cost_aware_policy(
            tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(
            [decision.task_id for decision in decisions],
            ["hotspot-confirmation-upper", "baseline-service-nearshore"],
        )

    def test_cost_aware_allocator_picks_lower_cost_task_inside_partition(self) -> None:
        agents = build_demo_agent_states()
        execution_states = {
            agent.agent_id: AgentExecutionState(
                agent_id=agent.agent_id,
                stage=ExecutionStage.PATROL
                if agent.agent_id == "USV-2"
                else ExecutionStage.GO_TO_TASK
                if agent.kind == "USV"
                else ExecutionStage.PATROL,
                active_task_id=None
                if agent.agent_id == "USV-2"
                else "busy-task"
                if agent.kind == "USV"
                else None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id=agent.agent_id,
                patrol_waypoint_index=0,
            )
            for agent in agents
        }
        tasks = (
            TaskRecord(
                task_id="hotspot-confirmation-upper-near",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=700.0,
                target_y=320.0,
                target_row=10,
                target_col=28,
                created_step=4,
            ),
            TaskRecord(
                task_id="hotspot-confirmation-upper-far",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=900.0,
                target_y=320.0,
                target_row=10,
                target_col=36,
                created_step=2,
            ),
        )

        updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
            tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["hotspot-confirmation-upper-near"], "USV-2")
        self.assertIsNone(assigned_by_id["hotspot-confirmation-upper-far"])
        self.assertEqual(decisions[0].task_id, "hotspot-confirmation-upper-near")
        self.assertEqual(
            decisions[0].selection_reason,
            "lowest_cost_partition_usv_for_hotspot_confirmation",
        )

    def test_cost_aware_allocator_keeps_partition_even_if_other_usv_is_closer(self) -> None:
        agents = tuple(
            replace(agent, x=650.0, y=315.0) if agent.agent_id == "USV-3" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-upper-zone",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-2")
        self.assertEqual(decisions[0].agent_id, "USV-2")

    def test_cost_aware_allocator_requeues_unreachable_partition_candidate(self) -> None:
        grid_map = self._build_runtime_grid_map()
        agents = tuple(
            replace(agent, x=287.5, y=487.5) if agent.agent_id == "USV-2" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-upper-unreachable",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
        )

        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertIsNone(updated_tasks[0].assigned_agent_id)
        self.assertEqual(decisions, ())

    def test_cost_aware_allocator_preserves_uav_resupply_logic(self) -> None:
        agents = tuple(
            agent if agent.agent_id != "UAV-1" else replace(agent, energy_level=30.0)
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.PENDING,
            priority=20,
            target_x=150.0,
            target_y=260.0,
            target_row=None,
            target_col=None,
            created_step=5,
            assigned_agent_id="UAV-1",
        )

        updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "UAV-1")
        self.assertEqual(updated_tasks[0].support_agent_id, "USV-1")
        self.assertEqual(decisions[0].selection_reason, "nearest_usv_rendezvous_for_uav_resupply")

    def test_cost_aware_reachable_cost_uses_cache_for_same_agent_task_pair(self) -> None:
        from unittest.mock import patch

        grid_map = self._build_runtime_grid_map()
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        task = TaskRecord(
            task_id="baseline-service-cache",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=540.0,
            target_row=6,
            target_col=3,
            created_step=5,
        )
        cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}

        with patch(
            "usv_uav_marine_coverage.tasking.cost_aware_task_allocator.build_astar_path_plan"
        ) as planner:
            planner.return_value = PathPlan(
                plan_id="mock-plan",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id=task.task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=task.target_x, y=task.target_y),
                ),
                goal_x=task.target_x,
                goal_y=task.target_y,
                estimated_cost=42.0,
            )
            first_cost = _reachable_cost(
                task,
                agent=agent,
                grid_map=grid_map,
                reachability_cache=cache,
            )
            second_cost = _reachable_cost(
                task,
                agent=agent,
                grid_map=grid_map,
                reachability_cache=cache,
            )

        self.assertEqual(first_cost, 42.0)
        self.assertEqual(second_cost, 42.0)
        self.assertEqual(planner.call_count, 1)


if __name__ == "__main__":
    unittest.main()
