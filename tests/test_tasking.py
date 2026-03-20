import unittest
from dataclasses import replace
from unittest.mock import patch

from usv_uav_marine_coverage.environment import (
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import InformationValidity, build_information_map
from usv_uav_marine_coverage.planning.path_types import PathPlan, PathPlanStatus, Waypoint
from usv_uav_marine_coverage.simulation.simulation_policy import build_demo_agent_states
from usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator import (
    AGENT_TASK_BLOCKED_COOLDOWN_STEPS as AEA_AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
)
from usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator import (
    allocate_tasks_with_aoi_energy_auction_policy,
)
from usv_uav_marine_coverage.tasking.baseline_task_generator import build_baseline_tasks
from usv_uav_marine_coverage.tasking.basic_task_allocator import (
    allocate_tasks_with_basic_policy,
)
from usv_uav_marine_coverage.tasking.cost_aware_task_allocator import (
    AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
    _reachable_cost,
    allocate_tasks_with_cost_aware_policy,
)
from usv_uav_marine_coverage.tasking.distributed_cbba_allocator import (
    AGENT_TASK_BLOCKED_COOLDOWN_STEPS as CBBA_AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
)
from usv_uav_marine_coverage.tasking.distributed_cbba_allocator import (
    allocate_tasks_with_distributed_cbba_policy,
)
from usv_uav_marine_coverage.tasking.partitioning import (
    baseline_primary_usv_ids_for_task,
    baseline_secondary_usv_ids_for_task,
    build_baseline_task_partition,
    build_task_partition,
)
from usv_uav_marine_coverage.tasking.rho_task_allocator import (
    AGENT_TASK_BLOCKED_COOLDOWN_STEPS as RHO_AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
)
from usv_uav_marine_coverage.tasking.rho_task_allocator import (
    allocate_tasks_with_rho_policy,
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

    def test_baseline_partition_layer_matches_current_hard_zones(self) -> None:
        nearshore_task = TaskRecord(
            task_id="baseline-nearshore",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=150.0,
            target_row=6,
            target_col=7,
            created_step=3,
        )
        upper_offshore_task = TaskRecord(
            task_id="hotspot-upper",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=4,
        )
        lower_offshore_task = replace(
            upper_offshore_task,
            task_id="hotspot-lower",
            target_y=720.0,
            target_row=28,
        )

        self.assertEqual(
            build_baseline_task_partition(nearshore_task).primary_usv_ids,
            ("USV-1",),
        )
        self.assertEqual(
            build_baseline_task_partition(upper_offshore_task).primary_usv_ids,
            ("USV-2",),
        )
        self.assertEqual(
            build_baseline_task_partition(lower_offshore_task).primary_usv_ids,
            ("USV-3",),
        )
        self.assertEqual(baseline_secondary_usv_ids_for_task(nearshore_task), set())

    def test_baseline_partition_layer_keeps_uav_resupply_open_to_all_usvs(self) -> None:
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.PENDING,
            priority=20,
            target_x=600.0,
            target_y=500.0,
            target_row=None,
            target_col=None,
            created_step=9,
            assigned_agent_id="UAV-1",
        )

        partition = build_baseline_task_partition(task)

        self.assertEqual(
            baseline_primary_usv_ids_for_task(task),
            {"USV-1", "USV-2", "USV-3"},
        )
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(partition.partition_reason, "all_usvs_can_support_uav_resupply")

    def test_soft_partition_enables_secondary_when_other_offshore_usv_is_much_closer(self) -> None:
        agents = tuple(
            replace(agent, x=650.0, y=315.0) if agent.agent_id == "USV-3" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-soft-secondary",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        partition = build_task_partition(
            task,
            policy_name="soft_partition_policy",
            agents=agents,
            execution_states=execution_states,
            step=20,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-2",))
        self.assertEqual(partition.secondary_usv_ids, ("USV-3",))
        self.assertEqual(
            partition.partition_reason,
            "soft_partition_enable_secondary_for_distance_advantage",
        )

    def test_backlog_aware_partition_enables_secondary_for_fresh_baseline_under_backlog(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        fresh_baseline = TaskRecord(
            task_id="baseline-backlog-target",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=180.0,
            target_row=2,
            target_col=2,
            created_step=180,
        )
        aged_baseline_a = replace(fresh_baseline, task_id="baseline-aged-a", created_step=0)
        aged_baseline_b = replace(
            fresh_baseline,
            task_id="baseline-aged-b",
            created_step=10,
            target_y=220.0,
        )

        partition = build_task_partition(
            fresh_baseline,
            policy_name="backlog_aware_partition_policy",
            tasks=(fresh_baseline, aged_baseline_a, aged_baseline_b),
            agents=agents,
            execution_states=execution_states,
            step=200,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-1",))
        self.assertEqual(partition.secondary_usv_ids, ("USV-2",))
        self.assertEqual(
            partition.partition_reason,
            "backlog_aware_enable_secondary_for_baseline_backlog",
        )

    def test_backlog_aware_partition_holds_hotspot_secondary_for_baseline_backlog(self) -> None:
        agents = tuple(
            replace(agent, x=650.0, y=315.0) if agent.agent_id == "USV-3" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        hotspot_task = TaskRecord(
            task_id="hotspot-backlog-hold",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=180,
        )
        baseline_template = TaskRecord(
            task_id="baseline-backlog-template",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=180.0,
            target_row=2,
            target_col=2,
            created_step=0,
        )
        tasks = (
            hotspot_task,
            baseline_template,
            replace(baseline_template, task_id="baseline-backlog-b", created_step=20),
        )

        partition = build_task_partition(
            hotspot_task,
            policy_name="backlog_aware_partition_policy",
            tasks=tasks,
            agents=agents,
            execution_states=execution_states,
            step=200,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-2",))
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(
            partition.partition_reason,
            "backlog_aware_hold_secondary_for_baseline_backlog",
        )

    def test_weighted_voronoi_partition_prefers_nearest_usv(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="weighted-nearest",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=140.0,
            target_y=160.0,
            target_row=6,
            target_col=7,
            created_step=5,
        )

        partition = build_task_partition(
            task,
            policy_name="weighted_voronoi_partition_policy",
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-1",))
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(
            partition.partition_reason,
            "weighted_voronoi_keep_primary_only_outside_margin",
        )

    def test_weighted_voronoi_busy_penalty_can_demote_primary_candidate(self) -> None:
        agents = tuple(
            replace(agent, x=200.0, y=150.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=250.0, y=200.0)
            if agent.agent_id == "USV-2"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="existing-task",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        task = TaskRecord(
            task_id="weighted-busy",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=220.0,
            target_y=180.0,
            target_row=6,
            target_col=7,
            created_step=5,
        )

        partition = build_task_partition(
            task,
            policy_name="weighted_voronoi_partition_policy",
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-2",))
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(
            partition.partition_reason,
            "weighted_voronoi_keep_primary_only_outside_margin",
        )

    def test_weighted_voronoi_support_penalty_protects_nearest_support_usv(self) -> None:
        agents = tuple(
            replace(agent, energy_level=30.0, x=150.0, y=150.0)
            if agent.agent_id == "UAV-1"
            else replace(agent, x=130.0, y=150.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=170.0, y=150.0)
            if agent.agent_id == "USV-2"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="weighted-support-guard",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=135.0,
            target_y=150.0,
            target_row=6,
            target_col=7,
            created_step=5,
        )

        partition = build_task_partition(
            task,
            policy_name="weighted_voronoi_partition_policy",
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-2",))
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(
            partition.partition_reason,
            "weighted_voronoi_keep_primary_only_outside_margin",
        )

    def test_weighted_voronoi_secondary_margin_blocks_far_secondary(self) -> None:
        agents = tuple(
            replace(agent, x=150.0, y=150.0) if agent.agent_id == "USV-1" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="weighted-margin",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=160.0,
            target_y=160.0,
            target_row=6,
            target_col=7,
            created_step=5,
        )

        partition = build_task_partition(
            task,
            policy_name="weighted_voronoi_partition_policy",
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-1",))
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(
            partition.partition_reason,
            "weighted_voronoi_keep_primary_only_outside_margin",
        )

    def test_weighted_voronoi_keeps_uav_resupply_open_to_all_usvs(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.PENDING,
            priority=20,
            target_x=600.0,
            target_y=500.0,
            target_row=None,
            target_col=None,
            created_step=9,
            assigned_agent_id="UAV-1",
        )

        partition = build_task_partition(
            task,
            policy_name="weighted_voronoi_partition_policy",
            agents=agents,
            execution_states=execution_states,
        )

        self.assertEqual(partition.primary_usv_ids, ("USV-1", "USV-2", "USV-3"))
        self.assertEqual(partition.secondary_usv_ids, ())
        self.assertEqual(
            partition.partition_reason,
            "weighted_voronoi_all_usvs_can_support_uav_resupply",
        )

    def test_cost_aware_soft_partition_can_select_secondary_candidate(self) -> None:
        agents = tuple(
            replace(agent, x=650.0, y=315.0) if agent.agent_id == "USV-3" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-soft-selected-secondary",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
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
            step=20,
            zone_partition_policy="soft_partition_policy",
        )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-3")
        self.assertEqual(decisions[0].agent_id, "USV-3")

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
            step=20,
        )

        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertIsNone(updated_tasks[0].assigned_agent_id)
        self.assertEqual(
            updated_tasks[0].retry_after_step,
            20 + AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
        )
        self.assertEqual(
            updated_tasks[0].agent_retry_after_steps,
            (("USV-2", 20 + AGENT_TASK_BLOCKED_COOLDOWN_STEPS),),
        )
        self.assertEqual(decisions, ())

    def test_cost_aware_allocator_skips_reachability_during_agent_task_cooldown(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-cooldown",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
            retry_after_step=40,
            agent_retry_after_steps=(("USV-2", 40),),
        )

        with patch(
            "usv_uav_marine_coverage.tasking.cost_aware_task_allocator.build_usv_path_plan"
        ) as planner:
            updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
                (task,),
                agents=build_demo_agent_states(),
                execution_states=self._build_idle_execution_states(build_demo_agent_states()),
                grid_map=self._build_runtime_grid_map(),
                step=30,
            )

        self.assertEqual(planner.call_count, 0)
        self.assertEqual(updated_tasks[0].retry_after_step, 40)
        self.assertEqual(updated_tasks[0].agent_retry_after_steps, (("USV-2", 40),))
        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(decisions, ())

    def test_cost_aware_allocator_retries_after_agent_task_cooldown_expires(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-retry",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
            retry_after_step=30,
            agent_retry_after_steps=(("USV-2", 30),),
        )
        agents = build_demo_agent_states()
        grid_map = self._build_runtime_grid_map()

        with patch(
            "usv_uav_marine_coverage.tasking.cost_aware_task_allocator.build_usv_path_plan"
        ) as planner:
            planner.return_value = PathPlan(
                plan_id="mock-plan",
                planner_name="astar_path_planner",
                agent_id="USV-2",
                task_id=task.task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=120.0, y=500.0),
                    Waypoint(x=task.target_x, y=task.target_y),
                ),
                goal_x=task.target_x,
                goal_y=task.target_y,
                estimated_cost=42.0,
            )
            updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
                (task,),
                agents=agents,
                execution_states=self._build_idle_execution_states(agents),
                grid_map=grid_map,
                step=30,
            )

        self.assertEqual(planner.call_count, 1)
        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-2")
        self.assertIsNone(updated_tasks[0].retry_after_step)
        self.assertEqual(updated_tasks[0].agent_retry_after_steps, ())
        self.assertEqual(decisions[0].agent_id, "USV-2")

    def test_cost_aware_allocator_can_still_try_other_usv_while_one_agent_is_in_cooldown(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        task = TaskRecord(
            task_id="hotspot-confirmation-partial-cooldown",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=800.0,
            target_y=760.0,
            target_row=30,
            target_col=32,
            created_step=5,
            retry_after_step=40,
            agent_retry_after_steps=(("USV-2", 40),),
        )

        with patch(
            "usv_uav_marine_coverage.tasking.cost_aware_task_allocator.build_usv_path_plan"
        ) as planner:
            planner.return_value = PathPlan(
                plan_id="mock-plan-usv3",
                planner_name="astar_path_planner",
                agent_id="USV-3",
                task_id=task.task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agents[2].x, y=agents[2].y),
                    Waypoint(x=task.target_x, y=task.target_y),
                ),
                goal_x=task.target_x,
                goal_y=task.target_y,
                estimated_cost=55.0,
            )
            updated_tasks, decisions = allocate_tasks_with_cost_aware_policy(
                (task,),
                agents=agents,
                execution_states=self._build_idle_execution_states(agents),
                grid_map=self._build_runtime_grid_map(),
                step=30,
            )

        self.assertEqual(planner.call_count, 1)
        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-3")
        self.assertEqual(decisions[0].agent_id, "USV-3")

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
            "usv_uav_marine_coverage.tasking.cost_aware_task_allocator.build_usv_path_plan"
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

    def test_aoi_energy_auction_prefers_older_task_within_same_partition(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        info_map.state_at(6, 3).information_age = 200
        info_map.state_at(6, 3).validity = InformationValidity.STALE_KNOWN
        info_map.state_at(7, 3).information_age = 20
        info_map.state_at(7, 3).validity = InformationValidity.STALE_KNOWN
        older_task = TaskRecord(
            task_id="baseline-service-old",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=540.0,
            target_row=6,
            target_col=3,
            created_step=2,
        )
        newer_task = TaskRecord(
            task_id="baseline-service-new",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=600.0,
            target_row=7,
            target_col=3,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
            (older_task, newer_task),
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=10,
        )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["baseline-service-old"], "USV-1")
        self.assertEqual(assigned_by_id["baseline-service-new"], "USV-3")
        self.assertEqual(
            decisions[0].selection_reason,
            "highest_aoi_energy_bid_partition_usv_for_baseline_service",
        )
        self.assertIsNotNone(decisions[0].selection_details)
        assert decisions[0].selection_details is not None
        self.assertIn("base_value", decisions[0].selection_details)
        self.assertIn("aoi_gain", decisions[0].selection_details)
        self.assertIn("baseline_stale_bonus", decisions[0].selection_details)
        self.assertIn("path_cost", decisions[0].selection_details)
        self.assertIn("energy_penalty", decisions[0].selection_details)
        self.assertIn("hotspot_backlog_penalty", decisions[0].selection_details)
        self.assertIn("baseline_guard_penalty", decisions[0].selection_details)
        self.assertIn("final_bid", decisions[0].selection_details)
        self.assertTrue(decisions[0].candidate_agents)

    def test_aoi_energy_auction_allows_baseline_to_outbid_hotspot_in_same_market(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-usv-2",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-usv-3",
        )
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        info_map.state_at(6, 3).information_age = 220
        info_map.state_at(6, 3).validity = InformationValidity.STALE_KNOWN
        info_map.state_at(2, 2).information_age = 0
        info_map.state_at(2, 2).validity = InformationValidity.VALID

        baseline_task = TaskRecord(
            task_id="baseline-service-market-win",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=170.0,
            target_y=180.0,
            target_row=2,
            target_col=2,
            created_step=4,
        )
        hotspot_task = TaskRecord(
            task_id="hotspot-confirmation-market-lose",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=180.0,
            target_y=540.0,
            target_row=6,
            target_col=3,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
            (hotspot_task, baseline_task),
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=10,
        )

        task_by_id = {task.task_id: task for task in updated_tasks}
        self.assertEqual(task_by_id["baseline-service-market-win"].assigned_agent_id, "USV-1")
        self.assertIsNone(task_by_id["hotspot-confirmation-market-lose"].assigned_agent_id)
        self.assertEqual(
            decisions[0].selection_reason,
            "highest_aoi_energy_bid_partition_usv_for_baseline_service",
        )

    def test_aoi_energy_auction_backlog_penalty_can_flip_hotspot_bias(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-usv-2",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-usv-3",
        )
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        info_map.state_at(2, 2).information_age = 80
        info_map.state_at(2, 2).validity = InformationValidity.STALE_KNOWN

        baseline_task = TaskRecord(
            task_id="baseline-service-backlog-win",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=170.0,
            target_y=180.0,
            target_row=2,
            target_col=2,
            created_step=4,
        )
        hotspot_tasks = tuple(
            TaskRecord(
                task_id=f"hotspot-confirmation-backlog-{index}",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_INSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=170.0 + index,
                target_y=180.0 + index,
                target_row=2,
                target_col=2,
                created_step=5 + index,
            )
            for index in range(6)
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=20.0,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
                hotspot_tasks + (baseline_task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=grid_map,
                info_map=info_map,
                step=10,
            )

        task_by_id = {task.task_id: task for task in updated_tasks}
        self.assertEqual(task_by_id["baseline-service-backlog-win"].assigned_agent_id, "USV-1")
        self.assertEqual(
            decisions[0].selection_reason,
            "highest_aoi_energy_bid_partition_usv_for_baseline_service",
        )
        assert decisions[0].selection_details is not None
        self.assertGreater(decisions[0].selection_details["baseline_stale_bonus"], 0.0)
        first_candidate = decisions[0].candidate_agents[0]
        self.assertEqual(first_candidate["agent_id"], "USV-1")

    def test_aoi_energy_auction_baseline_guard_penalty_discourages_hotspot_detour(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-usv-2",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-usv-3",
        )
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        info_map.state_at(2, 2).information_age = 220
        info_map.state_at(2, 2).validity = InformationValidity.STALE_KNOWN

        baseline_task = TaskRecord(
            task_id="baseline-service-guard-win",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=170.0,
            target_y=180.0,
            target_row=2,
            target_col=2,
            created_step=4,
        )
        hotspot_task = TaskRecord(
            task_id="hotspot-confirmation-guard-lose",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=420.0,
            target_y=260.0,
            target_row=5,
            target_col=5,
            created_step=5,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            if task_id == hotspot_task.task_id:
                estimated_cost = 20.0
            else:
                estimated_cost = 25.0
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
                (hotspot_task, baseline_task),
                agents=agents,
                execution_states=execution_states,
                grid_map=grid_map,
                info_map=info_map,
                step=10,
                zone_partition_policy="soft_partition_policy",
            )

        task_by_id = {task.task_id: task for task in updated_tasks}
        self.assertEqual(task_by_id["baseline-service-guard-win"].assigned_agent_id, "USV-1")
        self.assertIsNone(task_by_id["hotspot-confirmation-guard-lose"].assigned_agent_id)
        self.assertEqual(
            decisions[0].selection_reason,
            "highest_aoi_energy_bid_partition_usv_for_baseline_service",
        )
        assert decisions[0].selection_details is not None
        self.assertGreater(decisions[0].selection_details["baseline_stale_bonus"], 0.0)
        candidate_by_agent = {item["agent_id"]: item for item in decisions[0].candidate_agents}
        self.assertIn("USV-1", candidate_by_agent)
        self.assertEqual(candidate_by_agent["USV-1"]["baseline_guard_penalty"], 0.0)

    def test_aoi_energy_auction_respects_agent_task_cooldown(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-aea-cooldown",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
            retry_after_step=40,
            agent_retry_after_steps=(("USV-2", 40),),
        )
        execution_states = self._build_idle_execution_states(build_demo_agent_states())
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task",
        )

        with patch(
            "usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator.build_usv_path_plan"
        ) as planner:
            updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
                (task,),
                agents=build_demo_agent_states(),
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=30,
            )

        self.assertEqual(planner.call_count, 0)
        self.assertEqual(updated_tasks[0].retry_after_step, 40)
        self.assertEqual(updated_tasks[0].agent_retry_after_steps, (("USV-2", 40),))
        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(decisions, ())

    def test_aoi_energy_auction_applies_blocked_retry_only_to_current_agent(self) -> None:
        agents = tuple(
            replace(agent, x=287.5, y=487.5) if agent.agent_id == "USV-2" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-aea-blocked",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
            info_map=build_information_map(self._build_runtime_grid_map()),
            step=20,
        )

        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(
            updated_tasks[0].agent_retry_after_steps,
            (("USV-2", 20 + AEA_AGENT_TASK_BLOCKED_COOLDOWN_STEPS),),
        )
        self.assertEqual(decisions, ())

    def test_aoi_energy_auction_energy_penalty_preserves_support_usv(self) -> None:
        agents = tuple(
            replace(agent, energy_level=50.0) if agent.agent_id == "UAV-1" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        protected_task = TaskRecord(
            task_id="baseline-service-nearshore-protected",
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
        competing_task = TaskRecord(
            task_id="baseline-service-upper-competing",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )
        info_map.state_at(6, 3).information_age = 120
        info_map.state_at(10, 28).information_age = 120

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=20.0,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
                (protected_task, competing_task),
                agents=agents,
                execution_states=execution_states,
                grid_map=grid_map,
                info_map=info_map,
                step=10,
            )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["baseline-service-upper-competing"], "USV-2")
        self.assertEqual(assigned_by_id["baseline-service-nearshore-protected"], "USV-1")
        decision_by_task = {decision.task_id: decision.agent_id for decision in decisions}
        self.assertEqual(decision_by_task["baseline-service-upper-competing"], "USV-2")

    def test_aoi_energy_auction_keeps_primary_only_when_primary_is_reachable(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-primary-only",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )
        info_map = build_information_map(self._build_runtime_grid_map())
        info_map.state_at(10, 28).information_age = 120

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            estimated_cost = 40.0 if agent.agent_id == "USV-2" else 44.0
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            _, decisions = allocate_tasks_with_aoi_energy_auction_policy(
                (task,),
                agents=build_demo_agent_states(),
                execution_states=self._build_idle_execution_states(build_demo_agent_states()),
                grid_map=self._build_runtime_grid_map(),
                info_map=info_map,
                step=10,
            )

        self.assertEqual(decisions[0].agent_id, "USV-2")
        candidate_agent_ids = tuple(
            candidate["agent_id"] for candidate in decisions[0].candidate_agents
        )
        self.assertEqual(candidate_agent_ids, ("USV-2",))

    def test_aoi_energy_auction_selects_secondary_when_primary_is_blocked(
        self,
    ) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-secondary-selected",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )
        info_map = build_information_map(self._build_runtime_grid_map())
        info_map.state_at(10, 28).information_age = 120

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            if agent.agent_id == "USV-2":
                return PathPlan(
                    plan_id=f"{agent.agent_id}-{task_id}",
                    planner_name=planner_name,
                    agent_id=agent.agent_id,
                    task_id=task_id,
                    status=PathPlanStatus.BLOCKED,
                    waypoints=(),
                    goal_x=goal_x,
                    goal_y=goal_y,
                    estimated_cost=0.0,
                )
            estimated_cost = 30.0
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_aoi_energy_auction_policy(
                (task,),
                agents=build_demo_agent_states(),
                execution_states=self._build_idle_execution_states(build_demo_agent_states()),
                grid_map=self._build_runtime_grid_map(),
                info_map=info_map,
                step=10,
            )

        self.assertEqual(updated_tasks[0].assigned_agent_id, "USV-3")
        self.assertEqual(decisions[0].agent_id, "USV-3")
        candidate_agent_ids = tuple(
            candidate["agent_id"] for candidate in decisions[0].candidate_agents
        )
        self.assertEqual(candidate_agent_ids, ("USV-3",))

    def test_rho_prefers_older_baseline_when_path_costs_match(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        older_task = TaskRecord(
            task_id="baseline-service-rho-old",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=180.0,
            target_y=160.0,
            target_row=6,
            target_col=3,
            created_step=5,
        )
        newer_task = TaskRecord(
            task_id="baseline-service-rho-new",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=190.0,
            target_y=170.0,
            target_row=7,
            target_col=3,
            created_step=20,
        )
        info_map.state_at(6, 3).information_age = 110
        info_map.state_at(7, 3).information_age = 10

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=30.0,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.rho_task_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_rho_policy(
                (older_task, newer_task),
                agents=agents,
                execution_states=execution_states,
                grid_map=grid_map,
                info_map=info_map,
                step=30,
            )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["baseline-service-rho-old"], "USV-1")
        self.assertEqual(
            decisions[0].selection_reason,
            "highest_rho_window_score_partition_usv_for_baseline_service",
        )
        self.assertIsNotNone(decisions[0].selection_details)
        assert decisions[0].selection_details is not None
        self.assertIn("aoi_reward", decisions[0].selection_details)
        self.assertIn("delay_penalty", decisions[0].selection_details)
        self.assertTrue(decisions[0].candidate_agents)

    def test_rho_respects_agent_task_cooldown(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-rho-cooldown",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
            retry_after_step=40,
            agent_retry_after_steps=(("USV-2", 40),),
        )
        execution_states = self._build_idle_execution_states(build_demo_agent_states())
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task",
        )

        with patch(
            "usv_uav_marine_coverage.tasking.rho_task_allocator.build_usv_path_plan"
        ) as planner:
            updated_tasks, decisions = allocate_tasks_with_rho_policy(
                (task,),
                agents=build_demo_agent_states(),
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=30,
            )

        self.assertEqual(planner.call_count, 0)
        self.assertEqual(updated_tasks[0].retry_after_step, 40)
        self.assertEqual(updated_tasks[0].agent_retry_after_steps, (("USV-2", 40),))
        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(decisions, ())

    def test_rho_applies_blocked_retry_only_to_current_agent(self) -> None:
        agents = tuple(
            replace(agent, x=287.5, y=487.5) if agent.agent_id == "USV-2" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-rho-blocked",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        updated_tasks, decisions = allocate_tasks_with_rho_policy(
            (task,),
            agents=agents,
            execution_states=execution_states,
            grid_map=self._build_runtime_grid_map(),
            info_map=build_information_map(self._build_runtime_grid_map()),
            step=20,
        )

        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(
            updated_tasks[0].agent_retry_after_steps,
            (("USV-2", 20 + RHO_AGENT_TASK_BLOCKED_COOLDOWN_STEPS),),
        )
        self.assertEqual(decisions, ())

    def test_distributed_cbba_resolves_conflict_and_reassigns_losing_agent(self) -> None:
        agents = tuple(
            replace(agent, x=160.0, y=170.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=170.0, y=180.0)
            if agent.agent_id == "USV-2"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        grid_map = self._build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        hotspot = TaskRecord(
            task_id="hotspot-cbba-shared",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=220.0,
            target_y=220.0,
            target_row=8,
            target_col=8,
            created_step=5,
        )
        baseline = TaskRecord(
            task_id="baseline-cbba-fallback",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=1,
            target_x=190.0,
            target_y=190.0,
            target_row=7,
            target_col=7,
            created_step=4,
        )
        info_map.state_at(8, 8).information_age = 120
        info_map.state_at(7, 7).information_age = 40

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            estimated_cost_map = {
                ("USV-1", "hotspot-cbba-shared"): 20.0,
                ("USV-2", "hotspot-cbba-shared"): 28.0,
                ("USV-1", "baseline-cbba-fallback"): 18.0,
                ("USV-2", "baseline-cbba-fallback"): 16.0,
            }
            estimated_cost = estimated_cost_map[(agent.agent_id, task_id)]
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (hotspot, baseline),
                agents=agents,
                execution_states=execution_states,
                grid_map=grid_map,
                info_map=info_map,
                step=30,
                zone_partition_policy="weighted_voronoi_partition_policy",
                sync_interval_steps=5,
            )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["hotspot-cbba-shared"], "USV-1")
        self.assertEqual(assigned_by_id["baseline-cbba-fallback"], "USV-2")
        decision_by_task = {decision.task_id: decision for decision in decisions}
        self.assertEqual(
            decision_by_task["hotspot-cbba-shared"].selection_reason,
            "distributed_cbba_winner_for_hotspot_confirmation",
        )
        self.assertEqual(
            decision_by_task["baseline-cbba-fallback"].selection_reason,
            "distributed_cbba_winner_for_baseline_service",
        )
        assert decision_by_task["hotspot-cbba-shared"].selection_details is not None
        assert decision_by_task["baseline-cbba-fallback"].selection_details is not None
        self.assertEqual(
            decision_by_task["hotspot-cbba-shared"].selection_details["cbba_round"],
            1,
        )
        self.assertEqual(
            decision_by_task["hotspot-cbba-shared"].selection_details["sync_interval_steps"],
            5,
        )
        self.assertEqual(
            decision_by_task["baseline-cbba-fallback"].selection_details["cbba_round"],
            2,
        )
        self.assertEqual(
            decision_by_task["baseline-cbba-fallback"].selection_details["sync_interval_steps"],
            5,
        )

    def test_distributed_cbba_skips_new_market_assignments_between_sync_steps(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-cbba-sync-wait",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan"
        ) as planner:
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=7,
                zone_partition_policy="weighted_voronoi_partition_policy",
                sync_interval_steps=5,
            )

        self.assertEqual(planner.call_count, 0)
        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertIsNone(updated_tasks[0].assigned_agent_id)
        self.assertEqual(decisions, ())

    def test_distributed_cbba_broadcast_range_limits_cross_component_competition(self) -> None:
        agents = (
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
                x=100.0,
                y=100.0,
            ),
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
                x=260.0,
                y=100.0,
            ),
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
                x=800.0,
                y=800.0,
            ),
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-cbba-range",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=160.0,
            target_y=100.0,
            target_row=4,
            target_col=6,
            created_step=5,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            estimated_cost_map = {
                ("USV-1", "hotspot-confirmation-cbba-range"): 50.0,
                ("USV-2", "hotspot-confirmation-cbba-range"): 10.0,
            }
            estimated_cost = estimated_cost_map[(agent.agent_id, task_id)]
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            unlimited_tasks, unlimited_decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=20,
                zone_partition_policy="weighted_voronoi_partition_policy",
                broadcast_range_m=0.0,
            )
        self.assertEqual(unlimited_tasks[0].assigned_agent_id, "USV-2")
        self.assertEqual(unlimited_decisions[0].agent_id, "USV-2")
        assert unlimited_decisions[0].selection_details is not None
        self.assertEqual(
            unlimited_decisions[0].selection_details["communication_component_size"],
            3,
        )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            ranged_tasks, ranged_decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=20,
                zone_partition_policy="weighted_voronoi_partition_policy",
                broadcast_range_m=100.0,
            )
        self.assertEqual(ranged_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(ranged_decisions[0].agent_id, "USV-1")
        assert ranged_decisions[0].selection_details is not None
        self.assertEqual(ranged_decisions[0].selection_details["communication_component_size"], 1)

    def test_distributed_cbba_winner_memory_delays_rebidding_until_expired(self) -> None:
        agents = (
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
                x=100.0,
                y=100.0,
            ),
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
                x=200.0,
                y=100.0,
            ),
            replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
                x=900.0,
                y=900.0,
            ),
        )
        execution_states = self._build_idle_execution_states(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-cbba-memory",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=150.0,
            target_y=100.0,
            target_row=4,
            target_col=6,
            created_step=5,
            distributed_winner_memories=(("USV-1", "USV-2", 10),),
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            estimated_cost_map = {
                ("USV-1", "hotspot-confirmation-cbba-memory"): 10.0,
                ("USV-2", "hotspot-confirmation-cbba-memory"): 30.0,
            }
            estimated_cost = estimated_cost_map[(agent.agent_id, task_id)]
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            remembered_tasks, remembered_decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=12,
                zone_partition_policy="weighted_voronoi_partition_policy",
                winner_memory_ttl_steps=5,
            )

        self.assertEqual(remembered_tasks[0].assigned_agent_id, "USV-2")
        self.assertEqual(remembered_decisions[0].agent_id, "USV-2")
        assert remembered_decisions[0].selection_details is not None
        self.assertEqual(remembered_decisions[0].selection_details["winner_memory_ttl_steps"], 5)

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            expired_tasks, expired_decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=16,
                zone_partition_policy="weighted_voronoi_partition_policy",
                winner_memory_ttl_steps=5,
            )

        self.assertEqual(expired_tasks[0].assigned_agent_id, "USV-1")
        self.assertEqual(expired_decisions[0].agent_id, "USV-1")

    def test_distributed_cbba_bundle2_assigns_two_ordered_tasks_to_same_agent(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-upper",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-lower",
        )
        first_task = TaskRecord(
            task_id="baseline-bundle-first",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=150.0,
            target_y=140.0,
            target_row=5,
            target_col=5,
            created_step=4,
        )
        second_task = TaskRecord(
            task_id="baseline-bundle-second",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=170.0,
            target_y=155.0,
            target_row=5,
            target_col=6,
            created_step=5,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=20.0 if task_id == "baseline-bundle-first" else 24.0,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (first_task, second_task),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=30,
                zone_partition_policy="weighted_voronoi_partition_policy",
                bundle_length=2,
            )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["baseline-bundle-first"], "USV-1")
        self.assertEqual(assigned_by_id["baseline-bundle-second"], "USV-1")
        self.assertEqual(len(decisions), 2)
        decision_by_task = {decision.task_id: decision for decision in decisions}
        first_details = decision_by_task["baseline-bundle-first"].selection_details
        second_details = decision_by_task["baseline-bundle-second"].selection_details
        assert first_details is not None
        assert second_details is not None
        self.assertEqual(first_details["bundle_length"], 2)
        self.assertEqual(second_details["bundle_length"], 2)
        self.assertEqual(first_details["bundle_position"], 1)
        self.assertEqual(second_details["bundle_position"], 2)
        self.assertEqual(
            first_details["bundle_task_ids"],
            ("baseline-bundle-first", "baseline-bundle-second"),
        )
        self.assertEqual(second_details["bundle_task_ids"], first_details["bundle_task_ids"])

    def test_distributed_cbba_bundle2_atomic_winner_keeps_second_task(self) -> None:
        agents = tuple(
            replace(agent, x=140.0, y=140.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=260.0, y=145.0)
            if agent.agent_id == "USV-2"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-lower",
        )
        first_task = TaskRecord(
            task_id="bundle-atomic-first",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=150.0,
            target_y=150.0,
            target_row=5,
            target_col=5,
            created_step=2,
        )
        second_task = TaskRecord(
            task_id="bundle-atomic-second",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=220.0,
            target_y=150.0,
            target_row=5,
            target_col=7,
            created_step=3,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            estimated_cost_map = {
                ("USV-1", "bundle-atomic-first"): 8.0,
                ("USV-1", "bundle-atomic-second"): 14.0,
                ("USV-2", "bundle-atomic-first"): 40.0,
                ("USV-2", "bundle-atomic-second"): 6.0,
            }
            estimated_cost = estimated_cost_map[(agent.agent_id, task_id)]
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (first_task, second_task),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=20,
                zone_partition_policy="weighted_voronoi_partition_policy",
                bundle_length=2,
            )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["bundle-atomic-first"], "USV-1")
        self.assertEqual(assigned_by_id["bundle-atomic-second"], "USV-1")
        self.assertEqual({decision.agent_id for decision in decisions}, {"USV-1"})

    def test_distributed_cbba_bundle2_uses_inter_task_cost_for_second_task(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-upper",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-lower",
        )
        first_task = TaskRecord(
            task_id="bundle-margin-first",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=150.0,
            target_y=150.0,
            target_row=5,
            target_col=5,
            created_step=2,
        )
        far_second = TaskRecord(
            task_id="bundle-margin-far",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=850.0,
            target_y=850.0,
            target_row=25,
            target_col=25,
            created_step=3,
        )
        near_second = TaskRecord(
            task_id="bundle-margin-near",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=180.0,
            target_y=165.0,
            target_row=6,
            target_col=6,
            created_step=4,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            estimated_cost_map = {
                ("USV-1", "bundle-margin-first"): 12.0,
                ("USV-1", "bundle-margin-far"): 10.0,
                ("USV-1", "bundle-margin-near"): 12.0,
            }
            estimated_cost = estimated_cost_map[(agent.agent_id, task_id)]
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=estimated_cost,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (first_task, far_second, near_second),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=20,
                zone_partition_policy="weighted_voronoi_partition_policy",
                bundle_length=2,
            )

        assigned_by_id = {task.task_id: task.assigned_agent_id for task in updated_tasks}
        self.assertEqual(assigned_by_id["bundle-margin-first"], "USV-1")
        self.assertEqual(assigned_by_id["bundle-margin-near"], "USV-1")
        self.assertIsNone(assigned_by_id["bundle-margin-far"])
        decision_by_task = {decision.task_id: decision for decision in decisions}
        second_details = decision_by_task["bundle-margin-near"].selection_details
        assert second_details is not None
        self.assertEqual(
            second_details["bundle_task_ids"],
            ("bundle-margin-first", "bundle-margin-near"),
        )

    def test_distributed_cbba_bundle_length_one_preserves_single_assignment(self) -> None:
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-2"] = replace(
            execution_states["USV-2"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-upper",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-lower",
        )
        first_task = TaskRecord(
            task_id="bundle-one-first",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=150.0,
            target_y=140.0,
            target_row=5,
            target_col=5,
            created_step=4,
        )
        second_task = TaskRecord(
            task_id="bundle-one-second",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.PENDING,
            priority=2,
            target_x=170.0,
            target_y=155.0,
            target_row=5,
            target_col=6,
            created_step=5,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=20.0 if task_id == "bundle-one-first" else 24.0,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (first_task, second_task),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=30,
                zone_partition_policy="weighted_voronoi_partition_policy",
                bundle_length=1,
            )

        assigned_task_ids = {
            task.task_id for task in updated_tasks if task.assigned_agent_id is not None
        }
        self.assertEqual(len(assigned_task_ids), 1)
        self.assertEqual(len(decisions), 1)
        assert decisions[0].selection_details is not None
        self.assertEqual(decisions[0].selection_details["bundle_length"], 1)

    def test_distributed_cbba_respects_agent_task_cooldown(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-cbba-cooldown",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
            retry_after_step=40,
            agent_retry_after_steps=(("USV-2", 40),),
        )
        agents = build_demo_agent_states()
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-1"] = replace(
            execution_states["USV-1"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task-nearshore",
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task",
        )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan"
        ) as planner:
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=30,
                zone_partition_policy="weighted_voronoi_partition_policy",
            )

        self.assertEqual(planner.call_count, 0)
        self.assertEqual(updated_tasks[0].retry_after_step, 40)
        self.assertEqual(updated_tasks[0].agent_retry_after_steps, (("USV-2", 40),))
        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(decisions, ())

    def test_distributed_cbba_applies_blocked_retry_to_current_agent_only(self) -> None:
        agents = tuple(
            replace(agent, x=620.0, y=320.0)
            if agent.agent_id == "USV-2"
            else replace(agent, x=150.0, y=150.0)
            if agent.agent_id == "USV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = self._build_idle_execution_states(agents)
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="occupied-task",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-cbba-blocked",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=700.0,
            target_y=320.0,
            target_row=10,
            target_col=28,
            created_step=5,
        )

        def _mock_plan(agent, *, goal_x, goal_y, planner_name, task_id, **_kwargs):
            if agent.agent_id == "USV-2":
                return PathPlan(
                    plan_id=f"{agent.agent_id}-{task_id}",
                    planner_name=planner_name,
                    agent_id=agent.agent_id,
                    task_id=task_id,
                    status=PathPlanStatus.BLOCKED,
                    waypoints=(),
                    goal_x=goal_x,
                    goal_y=goal_y,
                    estimated_cost=0.0,
                )
            return PathPlan(
                plan_id=f"{agent.agent_id}-{task_id}",
                planner_name=planner_name,
                agent_id=agent.agent_id,
                task_id=task_id,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=goal_x, y=goal_y)),
                goal_x=goal_x,
                goal_y=goal_y,
                estimated_cost=30.0,
            )

        with patch(
            "usv_uav_marine_coverage.tasking.distributed_cbba_allocator.build_usv_path_plan",
            side_effect=_mock_plan,
        ):
            updated_tasks, decisions = allocate_tasks_with_distributed_cbba_policy(
                (task,),
                agents=agents,
                execution_states=execution_states,
                grid_map=self._build_runtime_grid_map(),
                info_map=build_information_map(self._build_runtime_grid_map()),
                step=20,
                zone_partition_policy="weighted_voronoi_partition_policy",
            )

        self.assertEqual(updated_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(decisions, ())
        self.assertEqual(
            updated_tasks[0].agent_retry_after_steps,
            (("USV-2", 20 + CBBA_AGENT_TASK_BLOCKED_COOLDOWN_STEPS),),
        )


if __name__ == "__main__":
    unittest.main()
