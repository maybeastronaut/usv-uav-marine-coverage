import unittest
from dataclasses import replace
from math import hypot
from unittest.mock import patch

from usv_uav_marine_coverage.agent_model import AgentState, AgentTaskState, HealthStatus, TaskMode
from usv_uav_marine_coverage.environment import (
    CircularFeature,
    ObstacleLayout,
    PolygonObstacle,
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.execution.collision_guard import (
    _apply_usv_collision_guard,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionOutcome,
    ExecutionStage,
    UavCoverageState,
)
from usv_uav_marine_coverage.execution.path_follower import (
    follow_path_step_with_local_mpc as real_follow_path_step_with_local_mpc,
)
from usv_uav_marine_coverage.execution.progress_feedback import (
    build_initial_progress_states,
)
from usv_uav_marine_coverage.execution.return_to_patrol_runtime import (
    _build_local_patrol_return_transition,
    _is_safe_patrol_access_candidate,
    _should_refresh_patrol_plan,
    _should_refresh_return_plan,
)
from usv_uav_marine_coverage.execution.task_approach import (
    USV_TASK_EDGE_APPROACH_BUFFER_M,
    can_start_hotspot_confirmation,
    resolve_usv_task_approach_target,
)
from usv_uav_marine_coverage.execution.task_claim_runtime import (
    select_claimable_task_for_agent,
)
from usv_uav_marine_coverage.execution.task_final_approach_runtime import (
    advance_task_final_approach_after_failure,
    task_final_approach_release_cooldown_steps,
    task_final_approach_satisfied,
)
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    InformationValidity,
    build_information_map,
)
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.path_types import PathPlan, PathPlanStatus, Waypoint
from usv_uav_marine_coverage.planning.usv_patrol_planner import PatrolSegmentAccess
from usv_uav_marine_coverage.simulation.simulation_agent_runtime import (
    _apply_corridor_directive,
    _build_corridor_directives,
    _build_dynamic_bottleneck_directives,
    _evaluate_agent_progress,
    _pre_step_transition,
    advance_agents_one_step,
    build_initial_execution_states,
)
from usv_uav_marine_coverage.simulation.simulation_policy import (
    build_demo_agent_states,
    build_patrol_routes,
)
from usv_uav_marine_coverage.simulation.simulation_task_runtime import (
    finalize_task_resolutions,
    sync_task_statuses,
)
from usv_uav_marine_coverage.tasking.task_types import (
    TaskRecord,
    TaskSource,
    TaskStatus,
    TaskType,
)


def _build_runtime_grid_map():
    sea_map = build_default_sea_map()
    obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
    return build_grid_map(sea_map, obstacle_layout)


class SimulationRuntimeTestCase(unittest.TestCase):
    def test_select_claimable_task_prefers_in_progress_active_task(self) -> None:
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-b",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
        )
        task_records = (
            TaskRecord(
                task_id="task-a",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_INSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=700.0,
                target_y=880.0,
                target_row=35,
                target_col=28,
                created_step=10,
                assigned_agent_id="USV-3",
            ),
            TaskRecord(
                task_id="task-b",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_INSPECTED,
                status=TaskStatus.IN_PROGRESS,
                priority=10,
                target_x=712.5,
                target_y=887.5,
                target_row=35,
                target_col=28,
                created_step=9,
                assigned_agent_id="USV-3",
            ),
        )

        selected = select_claimable_task_for_agent(
            task_records,
            agent_id="USV-3",
            execution_state=execution_state,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected.task_id, "task-b")

    def test_patrol_agent_transitions_to_new_task(self) -> None:
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=160.0,
            y=620.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        progress_state = AgentProgressState(agent_id="USV-1")
        patrol_routes = build_patrol_routes()
        active_task = TaskRecord(
            task_id="hotspot-confirmation-29-19",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=737.5,
            target_row=29,
            target_col=19,
            created_step=268,
            assigned_agent_id="USV-1",
        )

        transitioned = _pre_step_transition(
            usv,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
            task_records=(active_task,),
            grid_map=grid_map,
            info_map=info_map,
        )

        self.assertEqual(transitioned.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(transitioned.active_task_id, active_task.task_id)

    def test_return_to_patrol_agent_transitions_to_new_task(self) -> None:
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=678.609,
            y=912.862,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
            return_target_x=640.0,
            return_target_y=912.0,
        )
        progress_state = AgentProgressState(agent_id="USV-3")
        patrol_routes = build_patrol_routes()
        active_task = TaskRecord(
            task_id="hotspot-confirmation-35-28",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=712.5,
            target_y=887.5,
            target_row=35,
            target_col=28,
            created_step=828,
            assigned_agent_id="USV-3",
        )

        transitioned = _pre_step_transition(
            usv,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
            task_records=(active_task,),
            grid_map=grid_map,
            info_map=info_map,
        )

        self.assertEqual(transitioned.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(transitioned.active_task_id, active_task.task_id)

    def test_resolve_usv_task_approach_target_shifts_edge_baseline_inward(self) -> None:
        grid_map = _build_runtime_grid_map()
        usv = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        task = TaskRecord(
            task_id="baseline-service-edge",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=1,
            target_x=12.5,
            target_y=462.5,
            target_row=18,
            target_col=0,
            created_step=4,
            assigned_agent_id="USV-2",
        )

        approach_x, approach_y = resolve_usv_task_approach_target(
            usv,
            task=task,
            grid_map=grid_map,
        )

        self.assertGreater(approach_x, task.target_x)
        self.assertGreaterEqual(approach_y, USV_TASK_EDGE_APPROACH_BUFFER_M)
        self.assertLessEqual(
            hypot(approach_x - task.target_x, approach_y - task.target_y),
            usv.coverage_radius - usv.arrival_tolerance_m + 1e-6,
        )

    def test_resolve_usv_task_approach_target_shifts_edge_hotspot_inward(self) -> None:
        grid_map = _build_runtime_grid_map()
        usv = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        task = TaskRecord(
            task_id="hotspot-confirmation-edge",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=662.5,
            target_y=12.5,
            target_row=0,
            target_col=26,
            created_step=12,
            assigned_agent_id="USV-1",
        )

        approach_x, approach_y = resolve_usv_task_approach_target(
            usv,
            task=task,
            grid_map=grid_map,
        )

        self.assertGreater(approach_y, task.target_y)
        self.assertLess(
            hypot(approach_x - task.target_x, approach_y - task.target_y),
            hypot(usv.x - task.target_x, usv.y - task.target_y),
        )
        self.assertLessEqual(
            hypot(approach_x - task.target_x, approach_y - task.target_y),
            usv.coverage_radius - usv.arrival_tolerance_m + 1e-6,
        )

    def test_resolve_usv_task_approach_target_uses_hotspot_standoff_point(self) -> None:
        grid_map = _build_runtime_grid_map()
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=906.571,
            y=386.998,
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-standoff",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=962.5,
            target_y=412.5,
            target_row=16,
            target_col=38,
            created_step=20,
            assigned_agent_id="USV-3",
        )

        approach_x, approach_y = resolve_usv_task_approach_target(
            usv,
            task=task,
            grid_map=grid_map,
        )

        self.assertLess(
            hypot(approach_x - task.target_x, approach_y - task.target_y),
            hypot(usv.x - task.target_x, usv.y - task.target_y),
        )
        self.assertLessEqual(
            hypot(approach_x - task.target_x, approach_y - task.target_y),
            usv.coverage_radius - usv.arrival_tolerance_m + 1e-6,
        )
        self.assertNotEqual((approach_x, approach_y), (task.target_x, task.target_y))

    def test_resolve_usv_task_approach_target_rotates_hotspot_candidate_after_recovery(
        self,
    ) -> None:
        grid_map = _build_runtime_grid_map()
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=835.846,
            y=627.599,
            heading_deg=78.856,
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-reapproach",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=837.5,
            target_y=687.5,
            target_row=27,
            target_col=33,
            created_step=20,
            assigned_agent_id="USV-3",
        )

        default_target = resolve_usv_task_approach_target(
            usv,
            task=task,
            grid_map=grid_map,
            hotspot_reapproach_attempt=0,
        )
        alternate_target = resolve_usv_task_approach_target(
            usv,
            task=task,
            grid_map=grid_map,
            hotspot_reapproach_attempt=1,
        )

        self.assertNotEqual(default_target, alternate_target)
        self.assertLessEqual(
            hypot(alternate_target[0] - task.target_x, alternate_target[1] - task.target_y),
            usv.coverage_radius - usv.arrival_tolerance_m + 1e-6,
        )

    def test_hotspot_confirmation_enters_on_task_inside_coverage_radius(self) -> None:
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=643.679,
            y=105.221,
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-near-cover",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=662.5,
            target_y=87.5,
            target_row=3,
            target_col=26,
            created_step=20,
            assigned_agent_id="USV-1",
        )

        should_enter = can_start_hotspot_confirmation(
            usv,
            task=task,
        )

        self.assertTrue(should_enter)

    def test_hotspot_confirmation_does_not_enter_on_task_outside_coverage_radius(self) -> None:
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
            x=496.502,
            y=294.258,
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-outside-cover",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=20,
            assigned_agent_id="USV-2",
        )

        should_enter = can_start_hotspot_confirmation(
            usv,
            task=task,
        )

        self.assertFalse(should_enter)

    def test_task_final_approach_satisfied_for_baseline_inside_coverage_radius(self) -> None:
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
            x=37.5,
            y=462.5,
        )
        task = TaskRecord(
            task_id="baseline-service-near-cover",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=1,
            target_x=12.5,
            target_y=462.5,
            target_row=18,
            target_col=0,
            created_step=4,
            assigned_agent_id="USV-2",
        )

        self.assertTrue(task_final_approach_satisfied(usv, task=task))

    def test_apply_corridor_directive_keeps_yield_state_until_sticky_expiry(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.YIELD,
            active_task_id="hotspot-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            yield_target_x=222.0,
            yield_target_y=270.0,
            yield_reason="corridor_wait_for_owner",
            reserved_corridor_name="Upper Traversable Corridor",
            corridor_owner_agent_id="USV-2",
            corridor_reservation_until_step=54,
            pre_yield_stage=ExecutionStage.GO_TO_TASK,
        )
        owner_agent = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=260.0,
            y=270.0,
            heading_deg=0.0,
            speed_mps=3.5,
            max_speed_mps=5.0,
            detection_radius=50.0,
            coverage_radius=50.0,
            task=AgentTaskState(),
            energy_capacity=100.0,
            energy_level=100.0,
            cruise_speed_mps=4.0,
            max_turn_rate_degps=35.0,
        )
        owner_execution_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-2",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
        )

        updated_state = _apply_corridor_directive(
            state,
            directive=None,
            step=52,
            owner_agent=owner_agent,
            owner_execution_state=owner_execution_state,
        )

        self.assertEqual(updated_state.stage, ExecutionStage.YIELD)
        self.assertEqual(updated_state.reserved_corridor_name, "Upper Traversable Corridor")
        self.assertEqual(updated_state.corridor_owner_agent_id, "USV-2")

    def test_apply_corridor_directive_keeps_waiting_after_sticky_expiry(
        self,
    ) -> None:
        state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.YIELD,
            active_task_id="hotspot-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            yield_target_x=222.0,
            yield_target_y=270.0,
            yield_reason="corridor_wait_for_owner",
            reserved_corridor_name="Upper Traversable Corridor",
            corridor_owner_agent_id="USV-2",
            corridor_reservation_until_step=54,
            pre_yield_stage=ExecutionStage.GO_TO_TASK,
        )
        owner_agent = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=260.0,
            y=270.0,
            heading_deg=0.0,
            speed_mps=3.5,
            max_speed_mps=5.0,
            detection_radius=50.0,
            coverage_radius=50.0,
            task=AgentTaskState(),
            energy_capacity=100.0,
            energy_level=100.0,
            cruise_speed_mps=4.0,
            max_turn_rate_degps=35.0,
        )
        owner_execution_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-2",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
            reserved_corridor_name="Upper Traversable Corridor",
            corridor_owner_agent_id="USV-2",
        )

        updated_state = _apply_corridor_directive(
            state,
            directive=None,
            step=55,
            owner_agent=owner_agent,
            owner_execution_state=owner_execution_state,
        )

        self.assertEqual(updated_state.stage, ExecutionStage.YIELD)
        self.assertEqual(updated_state.reserved_corridor_name, "Upper Traversable Corridor")
        self.assertEqual(updated_state.corridor_owner_agent_id, "USV-2")

    def test_apply_corridor_directive_releases_sticky_yield_when_owner_recovers(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.YIELD,
            active_task_id="hotspot-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            yield_target_x=222.0,
            yield_target_y=270.0,
            yield_reason="corridor_wait_for_owner",
            reserved_corridor_name="Upper Traversable Corridor",
            corridor_owner_agent_id="USV-2",
            corridor_reservation_until_step=54,
            pre_yield_stage=ExecutionStage.GO_TO_TASK,
        )
        owner_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        )
        owner_state = replace(
            build_initial_execution_states((owner_agent,))["USV-2"],
            stage=ExecutionStage.RECOVERY,
        )

        updated_state = _apply_corridor_directive(
            state,
            directive=None,
            step=52,
            owner_agent=owner_agent,
            owner_execution_state=owner_state,
        )

        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertIsNone(updated_state.reserved_corridor_name)
        self.assertIsNone(updated_state.corridor_owner_agent_id)

    def test_apply_corridor_directive_releases_sticky_yield_when_owner_stalls(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.YIELD,
            active_task_id="hotspot-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            yield_target_x=222.0,
            yield_target_y=270.0,
            yield_reason="corridor_wait_for_owner",
            reserved_corridor_name="Upper Traversable Corridor",
            corridor_owner_agent_id="USV-2",
            corridor_reservation_until_step=54,
            pre_yield_stage=ExecutionStage.GO_TO_TASK,
        )
        owner_agent = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=260.0,
            y=270.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=5.0,
            detection_radius=50.0,
            coverage_radius=50.0,
            task=AgentTaskState(),
            energy_capacity=100.0,
            energy_level=100.0,
            cruise_speed_mps=4.0,
            max_turn_rate_degps=35.0,
        )
        owner_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-2",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
        )
        owner_progress_state = AgentProgressState(
            agent_id="USV-2",
            stalled_steps=2,
        )

        updated_state = _apply_corridor_directive(
            state,
            directive=None,
            step=52,
            owner_agent=owner_agent,
            owner_execution_state=owner_state,
            owner_progress_state=owner_progress_state,
        )

        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertIsNone(updated_state.reserved_corridor_name)
        self.assertIsNone(updated_state.corridor_owner_agent_id)

    def test_advance_agents_one_step_keeps_patrol_agents_stable(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
        )

        usv_before = next(agent for agent in agents if agent.agent_id == "USV-1")
        usv_after = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        usv_state = updated_execution_states["USV-1"]

        self.assertEqual(usv_state.stage, ExecutionStage.PATROL)
        self.assertNotEqual((usv_before.x, usv_before.y), (usv_after.x, usv_after.y))
        self.assertIsNotNone(usv_state.active_plan)
        assert usv_state.active_plan is not None

    def test_advance_agents_one_step_uses_local_mpc_execution_policy_for_usv(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()

        with patch(
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime.follow_path_step_with_local_mpc",
            wraps=real_follow_path_step_with_local_mpc,
        ) as local_mpc_mock:
            advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(),
                patrol_routes=build_patrol_routes(),
                grid_map=grid_map,
                dt_seconds=1.0,
                execution_policy="local_mpc_execution",
            )

        self.assertTrue(local_mpc_mock.called)

    def test_advance_agents_one_step_places_one_usv_into_yield_for_shared_corridor(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        upper_corridor = obstacle_layout.traversable_corridors[0]
        entry_x, entry_y = upper_corridor.control_points[-1]
        mid_x, mid_y = upper_corridor.control_points[2]
        exit_x, exit_y = upper_corridor.control_points[0]
        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=entry_x + 38.0, y=entry_y + 3.0)
        agents[1] = replace(agents[1], x=entry_x + 38.0, y=entry_y - 3.0)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        patrol_routes = build_patrol_routes(agents=agents)
        grid_map = build_grid_map(sea_map, obstacle_layout)

        for agent_id, task_id in (
            ("USV-1", "hotspot-confirmation-corridor-1"),
            ("USV-2", "hotspot-confirmation-corridor-2"),
        ):
            execution_states[agent_id] = AgentExecutionState(
                agent_id=agent_id,
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id=task_id,
                active_plan=PathPlan(
                    plan_id=f"plan-{agent_id}",
                    planner_name="astar_path_planner",
                    agent_id=agent_id,
                    task_id=task_id,
                    status=PathPlanStatus.PLANNED,
                    waypoints=(
                        Waypoint(x=entry_x + 38.0, y=entry_y),
                        Waypoint(x=entry_x, y=entry_y),
                        Waypoint(x=mid_x, y=mid_y),
                        Waypoint(x=exit_x, y=exit_y),
                    ),
                    goal_x=exit_x,
                    goal_y=exit_y,
                    estimated_cost=180.0,
                ),
                current_waypoint_index=1,
                patrol_route_id=agent_id,
                patrol_waypoint_index=0,
            )

        tasks = (
            TaskRecord(
                task_id="hotspot-confirmation-corridor-1",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=exit_x,
                target_y=exit_y,
                target_row=10,
                target_col=10,
                created_step=1,
                assigned_agent_id="USV-1",
            ),
            TaskRecord(
                task_id="hotspot-confirmation-corridor-2",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=exit_x,
                target_y=exit_y,
                target_row=10,
                target_col=11,
                created_step=1,
                assigned_agent_id="USV-2",
            ),
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=tasks,
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
        )

        yield_states = [
            state
            for state in updated_execution_states.values()
            if state.stage == ExecutionStage.YIELD
        ]
        self.assertEqual(len(yield_states), 1)
        self.assertEqual(yield_states[0].reserved_corridor_name, upper_corridor.name)
        self.assertEqual(yield_states[0].yield_reason, "corridor_wait_for_owner")
        self.assertIsNone(yield_states[0].reserved_bottleneck_zone_id)

    def test_build_corridor_directives_allows_same_direction_following_with_spacing(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        upper_corridor = obstacle_layout.traversable_corridors[0]
        entry_x, entry_y = upper_corridor.control_points[0]
        mid_x, mid_y = upper_corridor.control_points[2]
        exit_x, exit_y = upper_corridor.control_points[-1]

        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=entry_x + 18.0, y=entry_y + 2.0)
        agents[1] = replace(agents[1], x=mid_x + 4.0, y=mid_y + 1.0, speed_mps=4.5)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)

        for agent_id in ("USV-1", "USV-2"):
            execution_states[agent_id] = AgentExecutionState(
                agent_id=agent_id,
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id=f"task-{agent_id}",
                active_plan=PathPlan(
                    plan_id=f"plan-{agent_id}",
                    planner_name="astar_path_planner",
                    agent_id=agent_id,
                    task_id=f"task-{agent_id}",
                    status=PathPlanStatus.PLANNED,
                    waypoints=(
                        Waypoint(x=entry_x, y=entry_y),
                        Waypoint(x=mid_x, y=mid_y),
                        Waypoint(x=exit_x, y=exit_y),
                    ),
                    goal_x=exit_x,
                    goal_y=exit_y,
                    estimated_cost=180.0,
                ),
                current_waypoint_index=0,
                patrol_route_id=agent_id,
                patrol_waypoint_index=0,
            )

        directives = _build_corridor_directives(
            agents=agents,
            execution_states=execution_states,
            obstacle_layout=obstacle_layout,
        )

        self.assertEqual(directives["USV-2"].reason, "corridor_owner")
        self.assertEqual(directives["USV-1"].reason, "corridor_follow_owner")
        self.assertFalse(directives["USV-1"].should_yield)

    def test_build_corridor_directives_keeps_same_direction_follower_waiting_outside_entry(
        self,
    ) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        upper_corridor = obstacle_layout.traversable_corridors[0]
        entry_x, entry_y = upper_corridor.control_points[0]
        mid_x, mid_y = upper_corridor.control_points[2]
        exit_x, exit_y = upper_corridor.control_points[-1]

        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=entry_x - 36.0, y=entry_y + 2.0)
        agents[1] = replace(agents[1], x=mid_x + 4.0, y=mid_y + 1.0, speed_mps=4.5)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)

        for agent_id in ("USV-1", "USV-2"):
            execution_states[agent_id] = AgentExecutionState(
                agent_id=agent_id,
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id=f"task-{agent_id}",
                active_plan=PathPlan(
                    plan_id=f"plan-{agent_id}",
                    planner_name="astar_path_planner",
                    agent_id=agent_id,
                    task_id=f"task-{agent_id}",
                    status=PathPlanStatus.PLANNED,
                    waypoints=(
                        Waypoint(x=entry_x, y=entry_y),
                        Waypoint(x=mid_x, y=mid_y),
                        Waypoint(x=exit_x, y=exit_y),
                    ),
                    goal_x=exit_x,
                    goal_y=exit_y,
                    estimated_cost=180.0,
                ),
                current_waypoint_index=0,
                patrol_route_id=agent_id,
                patrol_waypoint_index=0,
            )

        directives = _build_corridor_directives(
            agents=agents,
            execution_states=execution_states,
            obstacle_layout=obstacle_layout,
        )

        self.assertEqual(directives["USV-2"].reason, "corridor_owner")
        self.assertEqual(directives["USV-1"].reason, "corridor_wait_for_owner")
        self.assertTrue(directives["USV-1"].should_yield)

    def test_build_corridor_directives_penalizes_stalled_owner_candidates(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        upper_corridor = obstacle_layout.traversable_corridors[0]
        entry_x, entry_y = upper_corridor.control_points[0]
        mid_x, mid_y = upper_corridor.control_points[2]
        exit_x, exit_y = upper_corridor.control_points[-1]

        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=entry_x + 8.0, y=entry_y + 1.0, speed_mps=4.2)
        agents[1] = replace(agents[1], x=mid_x + 3.0, y=mid_y + 1.0, speed_mps=0.0)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        progress_states["USV-2"] = AgentProgressState(agent_id="USV-2", stalled_steps=2)

        for agent_id in ("USV-1", "USV-2"):
            execution_states[agent_id] = AgentExecutionState(
                agent_id=agent_id,
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id=f"task-{agent_id}",
                active_plan=PathPlan(
                    plan_id=f"plan-{agent_id}",
                    planner_name="astar_path_planner",
                    agent_id=agent_id,
                    task_id=f"task-{agent_id}",
                    status=PathPlanStatus.PLANNED,
                    waypoints=(
                        Waypoint(x=entry_x, y=entry_y),
                        Waypoint(x=mid_x, y=mid_y),
                        Waypoint(x=exit_x, y=exit_y),
                    ),
                    goal_x=exit_x,
                    goal_y=exit_y,
                    estimated_cost=180.0,
                ),
                current_waypoint_index=0,
                patrol_route_id=agent_id,
                patrol_waypoint_index=0,
            )

        directives = _build_corridor_directives(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            obstacle_layout=obstacle_layout,
        )

        self.assertEqual(directives["USV-1"].reason, "corridor_owner")
        self.assertEqual(directives["USV-2"].reason, "corridor_wait_for_owner")
        self.assertTrue(directives["USV-2"].should_yield)

    def test_build_dynamic_bottleneck_directives_places_one_usv_into_yield_near_map_edge(
        self,
    ) -> None:
        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=543.369, y=143.434)
        agents[1] = replace(agents[1], x=571.808, y=94.321)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        patrol_routes = build_patrol_routes(agents=agents)

        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-confirmation-edge-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        execution_states["USV-2"] = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
            return_target_x=562.5,
            return_target_y=112.5,
        )
        active_tasks_by_agent = {
            "USV-1": TaskRecord(
                task_id="hotspot-confirmation-edge-1",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=562.5,
                target_y=87.5,
                target_row=3,
                target_col=22,
                created_step=1,
                assigned_agent_id="USV-1",
            ),
            "USV-2": None,
            "USV-3": None,
            "UAV-1": None,
            "UAV-2": None,
        }

        directives = _build_dynamic_bottleneck_directives(
            agents=agents,
            execution_states=execution_states,
            active_tasks_by_agent=active_tasks_by_agent,
            patrol_routes=patrol_routes,
            obstacle_layout=None,
            corridor_directives={},
            step=240,
        )

        self.assertEqual(set(directives), {"USV-1", "USV-2"})
        yield_directive = next(
            directive for directive in directives.values() if directive.should_yield
        )
        owner_directive = next(
            directive for directive in directives.values() if not directive.should_yield
        )
        self.assertEqual(yield_directive.reason, "dynamic_bottleneck_wait_for_owner")
        self.assertEqual(yield_directive.owner_agent_id, owner_directive.owner_agent_id)
        yielding_agent = next(
            agent for agent in agents if agent.agent_id != owner_directive.owner_agent_id
        )
        self.assertGreater(
            hypot(
                yield_directive.hold_x - yielding_agent.x,
                yield_directive.hold_y - yielding_agent.y,
            ),
            1.0,
        )

    def test_dynamic_bottleneck_skips_agents_with_fixed_corridor_directive(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        upper_corridor = obstacle_layout.traversable_corridors[0]
        entry_x, entry_y = upper_corridor.control_points[-1]
        mid_x, mid_y = upper_corridor.control_points[2]
        exit_x, exit_y = upper_corridor.control_points[0]
        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=entry_x + 38.0, y=entry_y + 3.0)
        agents[1] = replace(agents[1], x=entry_x + 38.0, y=entry_y - 3.0)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        patrol_routes = build_patrol_routes(agents=agents)

        for agent_id, task_id in (
            ("USV-1", "hotspot-confirmation-corridor-1"),
            ("USV-2", "hotspot-confirmation-corridor-2"),
        ):
            execution_states[agent_id] = AgentExecutionState(
                agent_id=agent_id,
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id=task_id,
                active_plan=PathPlan(
                    plan_id=f"plan-{agent_id}",
                    planner_name="astar_path_planner",
                    agent_id=agent_id,
                    task_id=task_id,
                    status=PathPlanStatus.PLANNED,
                    waypoints=(
                        Waypoint(x=entry_x + 38.0, y=entry_y),
                        Waypoint(x=entry_x, y=entry_y),
                        Waypoint(x=mid_x, y=mid_y),
                        Waypoint(x=exit_x, y=exit_y),
                    ),
                    goal_x=exit_x,
                    goal_y=exit_y,
                    estimated_cost=180.0,
                ),
                current_waypoint_index=1,
                patrol_route_id=agent_id,
                patrol_waypoint_index=0,
            )

        tasks = {
            "USV-1": TaskRecord(
                task_id="hotspot-confirmation-corridor-1",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=exit_x,
                target_y=exit_y,
                target_row=10,
                target_col=10,
                created_step=1,
                assigned_agent_id="USV-1",
            ),
            "USV-2": TaskRecord(
                task_id="hotspot-confirmation-corridor-2",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=exit_x,
                target_y=exit_y,
                target_row=10,
                target_col=11,
                created_step=1,
                assigned_agent_id="USV-2",
            ),
            "USV-3": None,
            "UAV-1": None,
            "UAV-2": None,
        }

        corridor_directives = {
            "USV-1": object(),
            "USV-2": object(),
        }
        directives = _build_dynamic_bottleneck_directives(
            agents=agents,
            execution_states=execution_states,
            active_tasks_by_agent=tasks,
            patrol_routes=patrol_routes,
            obstacle_layout=obstacle_layout,
            corridor_directives=corridor_directives,
            step=36,
        )

        self.assertEqual(directives, {})

    def test_advance_agents_one_step_clamps_local_mpc_usv_inside_map_bounds(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()

        def _out_of_bounds_result(agent, execution_state, **kwargs):
            return (
                replace(
                    agent,
                    x=grid_map.width + 25.0,
                    y=-10.0,
                    speed_mps=3.0,
                ),
                execution_state,
                ExecutionOutcome.ADVANCING,
            )

        with patch(
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime.follow_path_step_with_local_mpc",
            side_effect=_out_of_bounds_result,
        ):
            updated_agents, updated_execution_states, _ = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(),
                patrol_routes=build_patrol_routes(),
                grid_map=grid_map,
                dt_seconds=1.0,
                execution_policy="local_mpc_execution",
            )

        usv = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        self.assertGreaterEqual(usv.x, 0.0)
        self.assertLessEqual(usv.x, grid_map.width)
        self.assertGreaterEqual(usv.y, 0.0)
        self.assertLessEqual(usv.y, grid_map.height)
        self.assertEqual(usv.speed_mps, 0.0)
        self.assertIsNone(updated_execution_states["USV-1"].active_plan)

    def test_should_refresh_return_plan_reuses_recent_planned_path(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        execution_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=PathPlan(
                plan_id="return-plan",
                planner_name="astar_path_planner",
                agent_id="USV-2",
                task_id=None,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=agent.x + 10.0, y=agent.y),),
                goal_x=720.0,
                goal_y=620.0,
                estimated_cost=30.0,
            ),
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=1,
            return_target_x=720.0,
            return_target_y=620.0,
            last_return_plan_step=10,
        )

        with patch(
            "usv_uav_marine_coverage.execution.return_to_patrol_runtime.should_replan_return",
            return_value=True,
        ):
            self.assertFalse(
                _should_refresh_return_plan(
                    agent,
                    execution_state=execution_state,
                    step=12,
                    usv_path_planner="astar_path_planner",
                )
            )
            self.assertTrue(
                _should_refresh_return_plan(
                    agent,
                    execution_state=execution_state,
                    step=14,
                    usv_path_planner="astar_path_planner",
                )
            )

    def test_should_refresh_patrol_plan_honors_minimum_replan_interval(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=PathPlan(
                plan_id="patrol-plan",
                planner_name="astar_path_planner",
                agent_id="USV-1",
                task_id=None,
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=120.0, y=500.0), Waypoint(x=180.0, y=500.0)),
                goal_x=180.0,
                goal_y=500.0,
                estimated_cost=18.0,
            ),
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            last_patrol_plan_step=20,
        )

        with patch(
            "usv_uav_marine_coverage.execution.return_to_patrol_runtime.should_replan_patrol",
            return_value=True,
        ):
            self.assertFalse(
                _should_refresh_patrol_plan(
                    agent,
                    execution_state=execution_state,
                    goal_x=180.0,
                    goal_y=500.0,
                    step=21,
                    usv_path_planner="astar_path_planner",
                )
            )
            self.assertTrue(
                _should_refresh_patrol_plan(
                    agent,
                    execution_state=execution_state,
                    goal_x=180.0,
                    goal_y=500.0,
                    step=22,
                    usv_path_planner="astar_path_planner",
                )
            )

    def test_evaluate_agent_progress_does_not_enter_recovery_for_idle_patrol_without_plan(
        self,
    ) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3")
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=2,
        )
        progress_state = AgentProgressState(agent_id="USV-3", stalled_steps=2)
        patrol_route = build_patrol_routes()["USV-3"]

        _, updated_progress_state = _evaluate_agent_progress(
            agent,
            updated_agent=agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=None,
            patrol_routes={"USV-3": patrol_route},
            step=922,
        )

        self.assertEqual(updated_progress_state, AgentProgressState(agent_id="USV-3"))

    def test_advance_agents_one_step_can_use_hybrid_astar_for_usv_patrol(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
            usv_path_planner="hybrid_astar_path_planner",
        )

        usv_state = updated_execution_states["USV-1"]
        assert usv_state.active_plan is not None
        self.assertEqual(usv_state.active_plan.planner_name, "hybrid_astar_path_planner")

    def test_advance_agents_one_step_can_use_astar_smoother_for_usv_patrol(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
            usv_path_planner="astar_smoother_path_planner",
        )

        usv_state = updated_execution_states["USV-1"]
        assert usv_state.active_plan is not None
        self.assertEqual(usv_state.active_plan.planner_name, "astar_smoother_path_planner")

    def test_apply_usv_collision_guard_clamps_motion_before_polygon_obstacle(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        previous_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=271.5,
            y=562.5,
            heading_deg=75.0,
            speed_mps=2.0,
        )
        advanced_agent = replace(
            previous_agent,
            x=274.0,
            y=571.0,
            heading_deg=76.0,
            speed_mps=2.2,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-1",
            active_plan=build_astar_path_plan(
                previous_agent,
                grid_map=build_grid_map(sea_map, obstacle_layout),
                goal_x=520.0,
                goal_y=240.0,
                planner_name="astar_path_planner",
                task_id="task-1",
            ),
            current_waypoint_index=2,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
        )

        guarded_agent, guarded_state = _apply_usv_collision_guard(
            previous_agent,
            advanced_agent=advanced_agent,
            execution_state=execution_state,
            obstacle_layout=obstacle_layout,
        )

        self.assertLess(guarded_agent.x, advanced_agent.x)
        self.assertLess(guarded_agent.y, advanced_agent.y)
        self.assertEqual(guarded_agent.speed_mps, 0.0)
        self.assertEqual(guarded_agent.heading_deg, advanced_agent.heading_deg)
        self.assertEqual(guarded_agent.turn_rate_degps, advanced_agent.turn_rate_degps)
        self.assertIsNone(guarded_state.active_plan)
        self.assertEqual(guarded_state.current_waypoint_index, 0)

    def test_advance_agents_one_step_supports_uav_multi_region_patrol(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes(
            uav_search_planner="uav_multi_region_coverage_planner",
            agents=agents,
            info_map=info_map,
        )

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            dt_seconds=1.0,
            uav_search_planner="uav_multi_region_coverage_planner",
        )

        uav_before = next(agent for agent in agents if agent.agent_id == "UAV-1")
        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        uav_state = updated_execution_states["UAV-1"]

        self.assertEqual(uav_state.stage, ExecutionStage.PATROL)
        self.assertNotEqual((uav_before.x, uav_before.y), (uav_after.x, uav_after.y))
        self.assertIsNotNone(uav_state.active_plan)
        assert uav_state.active_plan is not None
        self.assertEqual(
            uav_state.active_plan.planner_name,
            "uav_multi_region_coverage_planner",
        )

    def test_uav_multi_region_patrol_does_not_freeze_on_nearby_first_waypoint(self) -> None:
        agents = tuple(
            replace(agent, x=705.931, y=502.165) if agent.agent_id == "UAV-1" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes(
            uav_search_planner="uav_multi_region_coverage_planner",
            agents=agents,
            info_map=info_map,
        )

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            dt_seconds=1.0,
            uav_search_planner="uav_multi_region_coverage_planner",
        )

        uav_before = next(agent for agent in agents if agent.agent_id == "UAV-1")
        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        uav_state = updated_execution_states["UAV-1"]

        self.assertNotEqual((uav_before.x, uav_before.y), (uav_after.x, uav_after.y))
        self.assertIsNotNone(uav_state.active_plan)
        assert uav_state.active_plan is not None
        self.assertEqual(
            uav_state.active_plan.planner_name,
            "uav_multi_region_coverage_planner",
        )

    def test_advance_agents_one_step_supports_uav_persistent_multi_region_patrol(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        uav_coverage_states: dict[str, UavCoverageState] = {}
        patrol_routes = build_patrol_routes(
            uav_search_planner="uav_persistent_multi_region_coverage_planner",
            agents=agents,
            info_map=info_map,
            uav_coverage_states=uav_coverage_states,
        )

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            uav_coverage_states=uav_coverage_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            dt_seconds=1.0,
            uav_search_planner="uav_persistent_multi_region_coverage_planner",
        )

        uav_before = next(agent for agent in agents if agent.agent_id == "UAV-1")
        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        uav_state = updated_execution_states["UAV-1"]

        self.assertNotEqual((uav_before.x, uav_before.y), (uav_after.x, uav_after.y))
        self.assertIsNotNone(uav_state.active_plan)
        assert uav_state.active_plan is not None
        self.assertEqual(
            uav_state.active_plan.planner_name,
            "uav_persistent_multi_region_coverage_planner",
        )
        self.assertIsNotNone(uav_coverage_states["UAV-1"].current_region_id)

    def test_uav_persistent_multi_region_patrol_keeps_region_commitment(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        uav_coverage_states: dict[str, UavCoverageState] = {}
        patrol_routes = build_patrol_routes(
            uav_search_planner="uav_persistent_multi_region_coverage_planner",
            agents=agents,
            info_map=info_map,
            uav_coverage_states=uav_coverage_states,
        )

        current_region_id = uav_coverage_states["UAV-1"].current_region_id
        for step in range(1, 4):
            agents, execution_states, progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                uav_coverage_states=uav_coverage_states,
                task_records=(),
                patrol_routes=patrol_routes,
                grid_map=grid_map,
                info_map=info_map,
                dt_seconds=1.0,
                step=step,
                uav_search_planner="uav_persistent_multi_region_coverage_planner",
            )

        self.assertEqual(uav_coverage_states["UAV-1"].current_region_id, current_region_id)

    def test_uav_persistent_multi_region_initializes_two_uavs_in_distinct_regions(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        uav_coverage_states: dict[str, UavCoverageState] = {}
        patrol_routes = build_patrol_routes(
            uav_search_planner="uav_persistent_multi_region_coverage_planner",
            agents=agents,
            info_map=info_map,
            uav_coverage_states=uav_coverage_states,
        )

        advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            uav_coverage_states=uav_coverage_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            dt_seconds=1.0,
            uav_search_planner="uav_persistent_multi_region_coverage_planner",
        )

        self.assertIsNotNone(uav_coverage_states["UAV-1"].current_region_id)
        self.assertIsNotNone(uav_coverage_states["UAV-2"].current_region_id)
        self.assertNotEqual(
            uav_coverage_states["UAV-1"].current_region_id,
            uav_coverage_states["UAV-2"].current_region_id,
        )

    def test_apply_usv_collision_guard_respects_hull_clearance_near_obstacle_edge(self) -> None:
        obstacle_layout = ObstacleLayout(
            seed=1,
            traversable_corridors=(),
            risk_zone_obstacles=(
                PolygonObstacle(
                    name="Edge Obstacle",
                    zone_name="Middle Risk Zone",
                    points=((250.0, 300.0), (450.0, 300.0), (450.0, 360.0), (250.0, 360.0)),
                ),
            ),
            offshore_features=(
                CircularFeature(
                    name="Ignore Risk Area",
                    feature_type="risk_area",
                    x=0.0,
                    y=0.0,
                    radius=1.0,
                ),
            ),
            nearshore_monitor_points=(),
            offshore_hotspots=(),
        )
        previous_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=230.0,
            y=330.0,
            heading_deg=0.0,
            speed_mps=4.0,
        )
        advanced_agent = replace(
            previous_agent,
            x=240.0,
            y=330.0,
            heading_deg=0.0,
            speed_mps=4.2,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-clearance",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )

        guarded_agent, guarded_state = _apply_usv_collision_guard(
            previous_agent,
            advanced_agent=advanced_agent,
            execution_state=execution_state,
            obstacle_layout=obstacle_layout,
        )

        self.assertLess(guarded_agent.x, advanced_agent.x)
        self.assertEqual(guarded_agent.speed_mps, 0.0)
        self.assertIsNone(guarded_state.active_plan)
        self.assertEqual(guarded_state.current_waypoint_index, 0)

    def test_advance_agents_one_step_advances_uav_along_lawnmower_patrol(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        patrol_routes = build_patrol_routes()
        grid_map = _build_runtime_grid_map()
        uav_before = next(agent for agent in agents if agent.agent_id == "UAV-1")

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            dt_seconds=1.0,
        )

        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        uav_state = updated_execution_states["UAV-1"]
        self.assertEqual(uav_state.stage, ExecutionStage.PATROL)
        self.assertEqual(uav_state.patrol_waypoint_index, 0)
        self.assertGreater(patrol_routes["UAV-1"][0][0], 450.0)
        self.assertGreater(uav_after.x, uav_before.x)
        self.assertLess(uav_after.y, uav_before.y)

    def test_advance_agents_one_step_transitions_assigned_usv_to_task_execution(self) -> None:
        agents = build_demo_agent_states()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-10-10",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=1,
            target_x=520.0,
            target_y=240.0,
            target_row=10,
            target_col=10,
            created_step=3,
            assigned_agent_id="USV-1",
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
        )

        usv_state = updated_execution_states["USV-1"]
        self.assertEqual(usv_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(usv_state.active_task_id, task.task_id)
        self.assertIsNotNone(usv_state.active_plan)
        assert usv_state.active_plan is not None
        self.assertEqual(usv_state.active_plan.planner_name, "astar_path_planner")
        self.assertGreater(len(usv_state.active_plan.waypoints), 2)

    def test_advance_agents_one_step_keeps_usv_task_plan_progress_without_replanning(
        self,
    ) -> None:
        base_agents = build_demo_agent_states()
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="baseline-service-10-10",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=1,
            target_x=520.0,
            target_y=240.0,
            target_row=10,
            target_col=10,
            created_step=3,
            assigned_agent_id="USV-1",
        )
        original_usv = next(agent for agent in base_agents if agent.agent_id == "USV-1")
        plan = build_astar_path_plan(
            original_usv,
            grid_map=grid_map,
            goal_x=task.target_x,
            goal_y=task.target_y,
            planner_name="astar_path_planner",
            task_id=task.task_id,
        )
        self.assertGreater(len(plan.waypoints), 4)
        progressed_agent = replace(
            original_usv,
            x=plan.waypoints[2].x,
            y=plan.waypoints[2].y,
        )
        agents = tuple(
            progressed_agent if agent.agent_id == "USV-1" else agent for agent in base_agents
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id=task.task_id,
            active_plan=plan,
            current_waypoint_index=3,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
        )

        usv_state = updated_execution_states["USV-1"]
        self.assertIs(usv_state.active_plan, plan)
        self.assertGreaterEqual(usv_state.current_waypoint_index, 3)

    def test_advance_agents_one_step_transitions_low_energy_uav_to_rendezvous(self) -> None:
        agents = tuple(
            replace(agent, energy_level=30.0) if agent.agent_id == "UAV-1" else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=110.0,
            target_y=180.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
        )

        uav_state = updated_execution_states["UAV-1"]
        self.assertEqual(uav_state.stage, ExecutionStage.GO_TO_RENDEZVOUS)
        self.assertEqual(uav_state.active_task_id, task.task_id)

    def test_advance_agents_one_step_keeps_recharging_uav_attached_to_support_usv(self) -> None:
        agents = tuple(
            replace(agent, energy_level=20.0)
            if agent.agent_id == "UAV-1"
            else replace(
                agent,
                speed_mps=5.0,
                heading_deg=35.0,
                turn_rate_degps=8.0,
                task=agent.task,
            )
            if agent.agent_id == "USV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["UAV-1"] = AgentExecutionState(
            agent_id="UAV-1",
            stage=ExecutionStage.ON_RECHARGE,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="UAV-1",
            patrol_waypoint_index=0,
        )
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.IN_PROGRESS,
            priority=20,
            target_x=110.0,
            target_y=180.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=_build_runtime_grid_map(),
            dt_seconds=1.0,
        )

        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        usv_after = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        self.assertEqual(updated_execution_states["UAV-1"].stage, ExecutionStage.ON_RECHARGE)
        self.assertEqual((uav_after.x, uav_after.y), (usv_after.x, usv_after.y))
        self.assertEqual(uav_after.heading_deg, usv_after.heading_deg)
        self.assertEqual(uav_after.speed_mps, usv_after.speed_mps)
        self.assertGreater(uav_after.energy_level, 20.0)

    def test_advance_agents_one_step_does_not_rendezvous_with_degraded_support_usv(self) -> None:
        agents = tuple(
            replace(agent, energy_level=30.0)
            if agent.agent_id == "UAV-1"
            else replace(agent, health_status=HealthStatus.DEGRADED, speed_multiplier=0.5)
            if agent.agent_id == "USV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["UAV-1"] = AgentExecutionState(
            agent_id="UAV-1",
            stage=ExecutionStage.GO_TO_RENDEZVOUS,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="UAV-1",
            patrol_waypoint_index=0,
        )
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=110.0,
            target_y=180.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=_build_runtime_grid_map(),
            dt_seconds=1.0,
        )

        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        self.assertEqual(updated_execution_states["UAV-1"].stage, ExecutionStage.PATROL)
        self.assertEqual(updated_execution_states["UAV-1"].active_task_id, None)
        self.assertEqual(uav_after.task.mode, TaskMode.IDLE)

    def test_uav_return_to_patrol_is_not_blocked_by_failed_usv_wreck_zone(self) -> None:
        agents = tuple(
            replace(
                agent,
                x=823.206,
                y=535.098,
                health_status=HealthStatus.FAILED,
                is_operational=False,
                max_speed_mps=0.0,
                cruise_speed_mps=0.0,
                speed_mps=0.0,
            )
            if agent.agent_id == "USV-2"
            else replace(
                agent,
                x=809.281,
                y=555.259,
                heading_deg=-90.0,
                speed_mps=0.0,
                energy_level=165.316,
            )
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["UAV-1"] = AgentExecutionState(
            agent_id="UAV-1",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="UAV-1",
            patrol_waypoint_index=0,
            return_target_x=809.281,
            return_target_y=542.192,
            return_target_source="fallback_legacy_local",
        )

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=build_patrol_routes(),
            grid_map=_build_runtime_grid_map(),
            dt_seconds=1.0,
            obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
            execution_policy="local_mpc_execution",
        )

        uav_before = next(agent for agent in agents if agent.agent_id == "UAV-1")
        uav_after = next(agent for agent in updated_agents if agent.agent_id == "UAV-1")
        self.assertEqual(updated_execution_states["UAV-1"].stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertNotEqual((uav_before.x, uav_before.y), (uav_after.x, uav_after.y))

    def test_advance_agents_one_step_rejoins_patrol_from_return_stage(self) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        grid_map = _build_runtime_grid_map()
        usv = next(agent for agent in agents if agent.agent_id == "USV-1")
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            return_target_x=usv.x,
            return_target_y=usv.y,
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            dt_seconds=1.0,
        )

        usv_state = updated_execution_states["USV-1"]
        self.assertEqual(usv_state.stage, ExecutionStage.PATROL)
        self.assertEqual(usv_state.patrol_waypoint_index, 1)

    def test_build_local_patrol_return_transition_filters_edge_accesses(self) -> None:
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_route = (
            (320.0, 580.0),
            (920.0, 580.0),
            (920.0, 880.0),
            (320.0, 880.0),
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=910.0,
            y=930.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=2,
        )
        progress_state = AgentProgressState(agent_id="USV-3")

        updated_state = _build_local_patrol_return_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_route,
            grid_map=grid_map,
            info_map=info_map,
            task_records=(),
        )

        self.assertEqual(updated_state.return_target_source, "fallback_relaxed_endpoint")
        self.assertLess(updated_state.return_target_x or 0.0, 920.0)
        self.assertNotEqual(
            (updated_state.return_target_x, updated_state.return_target_y),
            (920.0, 880.0),
        )

    def test_build_local_patrol_return_transition_prefers_hotspot_and_stale_pressure(
        self,
    ) -> None:
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        for cell in grid_map.flat_cells:
            state = info_map.state_at(cell.row, cell.col)
            state.validity = InformationValidity.VALID
            state.information_age = 0
        for row, col in ((35, 24), (34, 24), (35, 25)):
            state = info_map.state_at(row, col)
            state.validity = InformationValidity.STALE_KNOWN
            state.information_age = 240

        patrol_route = (
            (320.0, 580.0),
            (920.0, 580.0),
            (920.0, 880.0),
            (320.0, 880.0),
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=620.0,
            y=730.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=2,
        )
        progress_state = AgentProgressState(agent_id="USV-3")
        hotspot_task = TaskRecord(
            task_id="hotspot-confirmation-24-25",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=612.5,
            target_y=887.5,
            target_row=35,
            target_col=24,
            created_step=20,
        )

        updated_state = _build_local_patrol_return_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_route,
            grid_map=grid_map,
            info_map=info_map,
            task_records=(hotspot_task,),
        )

        self.assertEqual(updated_state.return_target_source, "safe_value_access")
        self.assertAlmostEqual(updated_state.return_target_y or 0.0, 880.0, places=3)
        self.assertAlmostEqual(updated_state.return_target_x or 0.0, 620.0, places=3)

    def test_build_local_patrol_return_transition_falls_back_to_waypoint_without_segment_access(
        self,
    ) -> None:
        grid_map = _build_runtime_grid_map()
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        progress_state = AgentProgressState(agent_id="USV-1")
        patrol_route = ((140.0, 140.0),)

        updated_state = _build_local_patrol_return_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_route,
            grid_map=grid_map,
            info_map=build_information_map(grid_map),
            task_records=(),
        )

        self.assertEqual(updated_state.return_target_source, "fallback_patrol_waypoint")
        self.assertEqual(
            (updated_state.return_target_x, updated_state.return_target_y),
            patrol_route[0],
        )

    def test_advance_agents_one_step_uses_astar_for_usv_return_to_patrol(self) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        grid_map = _build_runtime_grid_map()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        rejoin_index, return_x, return_y = 5, 320.0, 650.0
        execution_states["USV-3"] = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=rejoin_index,
            return_target_x=return_x,
            return_target_y=return_y,
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
            dt_seconds=1.0,
        )

        updated_state = updated_execution_states["USV-3"]
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertIsNotNone(updated_state.active_plan)
        assert updated_state.active_plan is not None
        self.assertEqual(updated_state.active_plan.planner_name, "astar_path_planner")
        self.assertGreaterEqual(len(updated_state.active_plan.waypoints), 2)

    def test_advance_agents_one_step_retargets_blocked_return_to_patrol_waypoint(self) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        patrol_routes["USV-1"] = (
            (287.5, 487.5),
            (80.0, 120.0),
            (190.0, 120.0),
            (190.0, 280.0),
        )
        grid_map = _build_runtime_grid_map()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            return_target_x=287.5,
            return_target_y=487.5,
        )

        _, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
            dt_seconds=1.0,
        )

        updated_state = updated_execution_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertEqual(updated_state.patrol_waypoint_index, 1)
        self.assertEqual(
            (updated_state.return_target_x, updated_state.return_target_y),
            (80.0, 120.0),
        )
        self.assertIsNotNone(updated_state.active_plan)
        assert updated_state.active_plan is not None
        self.assertEqual(updated_state.active_plan.status.value, "planned")

    def test_advance_agents_one_step_releases_blocked_usv_task_back_to_return_stage(self) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        grid_map = _build_runtime_grid_map()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        task = TaskRecord(
            task_id="baseline-service-blocked",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=1,
            target_x=287.5,
            target_y=487.5,
            target_row=19,
            target_col=11,
            created_step=4,
            assigned_agent_id="USV-1",
        )

        with patch(
            "usv_uav_marine_coverage.execution.stage_runtime.build_usv_path_plan",
            return_value=PathPlan(
                plan_id="blocked-plan",
                planner_name="astar_path_planner",
                agent_id="USV-1",
                task_id=task.task_id,
                status=PathPlanStatus.BLOCKED,
                waypoints=(),
                goal_x=task.target_x,
                goal_y=task.target_y,
                estimated_cost=0.0,
            ),
        ):
            _, updated_execution_states, _ = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=patrol_routes,
                grid_map=grid_map,
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
            )

        updated_state = updated_execution_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertIsNone(updated_state.active_task_id)
        self.assertIsNone(updated_state.active_plan)

    def test_advance_agents_one_step_releases_stalled_usv_task_after_collision_guard(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        grid_map = _build_runtime_grid_map()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        task = TaskRecord(
            task_id="baseline-service-stall",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=1,
            target_x=62.5,
            target_y=262.5,
            target_row=10,
            target_col=2,
            created_step=4,
            assigned_agent_id="USV-3",
        )

        def _fake_follow_path_step(agent, execution_state, *, dt_seconds, obstacle_layout=None):
            return (agent, execution_state, ExecutionOutcome.ADVANCING)

        def _fake_collision_guard(
            previous_agent,
            *,
            advanced_agent,
            execution_state,
            obstacle_layout,
        ):
            return (
                advanced_agent,
                replace(execution_state, active_plan=None, current_waypoint_index=0),
            )

        progress_states = build_initial_progress_states(agents)

        with (
            patch(
                "usv_uav_marine_coverage.simulation.simulation_agent_runtime.follow_path_step",
                side_effect=_fake_follow_path_step,
            ),
            patch(
                "usv_uav_marine_coverage.execution.collision_guard._apply_usv_collision_guard",
                side_effect=_fake_collision_guard,
            ),
        ):
            updated_agents, updated_execution_states, _ = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=patrol_routes,
                grid_map=grid_map,
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
            )

        updated_state = updated_execution_states["USV-3"]
        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-3")
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertIsNone(updated_state.active_task_id)
        self.assertEqual(updated_agent.speed_mps, 0.0)

    def test_advance_agents_one_step_retargets_stalled_patrol_usv(self) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        patrol_routes["USV-2"] = (
            (287.5, 487.5),
            (620.0, 120.0),
            (780.0, 160.0),
            (900.0, 300.0),
        )
        grid_map = _build_runtime_grid_map()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        usv = next(agent for agent in agents if agent.agent_id == "USV-2")
        plan = build_astar_path_plan(
            usv,
            grid_map=grid_map,
            goal_x=620.0,
            goal_y=120.0,
            planner_name="astar_path_planner",
            task_id=None,
        )
        execution_states["USV-2"] = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=1,
        )

        def _fake_follow_path_step(agent, execution_state, *, dt_seconds, obstacle_layout=None):
            return (agent, execution_state, ExecutionOutcome.ADVANCING)

        def _fake_collision_guard(
            previous_agent,
            *,
            advanced_agent,
            execution_state,
            obstacle_layout,
        ):
            return (
                advanced_agent,
                replace(execution_state, active_plan=None, current_waypoint_index=0),
            )

        with (
            patch(
                "usv_uav_marine_coverage.simulation.simulation_agent_runtime.follow_path_step",
                side_effect=_fake_follow_path_step,
            ),
            patch(
                "usv_uav_marine_coverage.execution.collision_guard._apply_usv_collision_guard",
                side_effect=_fake_collision_guard,
            ),
        ):
            updated_agents, updated_execution_states, _ = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(),
                patrol_routes=patrol_routes,
                grid_map=grid_map,
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
            )

        updated_state = updated_execution_states["USV-2"]
        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-2")
        self.assertEqual(updated_state.stage, ExecutionStage.PATROL)
        self.assertGreater(updated_state.patrol_waypoint_index, 1)
        self.assertIsNotNone(updated_state.active_plan)
        self.assertEqual(updated_agent.speed_mps, 0.0)

    def test_advance_agents_one_step_enters_recovery_after_three_stalled_steps(self) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        grid_map = _build_runtime_grid_map()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        task = TaskRecord(
            task_id="hotspot-confirmation-stall",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=520.0,
            target_y=240.0,
            target_row=10,
            target_col=10,
            created_step=5,
            assigned_agent_id="USV-1",
        )

        def _fake_follow_path_step(agent, execution_state, *, dt_seconds, obstacle_layout=None):
            return (agent, execution_state, ExecutionOutcome.ADVANCING)

        with patch(
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime.follow_path_step",
            side_effect=_fake_follow_path_step,
        ):
            updated_agents = agents
            updated_states = execution_states
            updated_progress_states = progress_states
            for step in range(1, 4):
                updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
                    agents=updated_agents,
                    execution_states=updated_states,
                    progress_states=updated_progress_states,
                    task_records=(task,),
                    patrol_routes=patrol_routes,
                    grid_map=grid_map,
                    dt_seconds=1.0,
                    step=step,
                )

        usv_state = updated_states["USV-1"]
        usv_progress = updated_progress_states["USV-1"]
        self.assertEqual(usv_state.stage, ExecutionStage.RECOVERY)
        self.assertEqual(usv_progress.pre_recovery_stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(usv_progress.pre_recovery_task_id, task.task_id)

    def test_recovery_success_returns_usv_to_task_execution(self) -> None:
        agents = build_demo_agent_states()
        usv = next(agent for agent in agents if agent.agent_id == "USV-1")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RECOVERY,
            active_task_id="hotspot-confirmation-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        progress_states["USV-1"] = AgentProgressState(
            agent_id="USV-1",
            pre_recovery_stage=ExecutionStage.GO_TO_TASK,
            pre_recovery_task_id="hotspot-confirmation-1",
            recovery_step_index=2,
            blocked_goal_signature="task:hotspot-confirmation-1",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-1",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=520.0,
            target_y=240.0,
            target_row=10,
            target_col=10,
            created_step=5,
            assigned_agent_id="USV-1",
        )

        recovered_agent = replace(usv, x=usv.x + 12.0, y=usv.y + 4.0)
        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, True),
        ):
            _, updated_states, updated_progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=build_patrol_routes(),
                grid_map=_build_runtime_grid_map(),
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
                step=10,
            )

        updated_state = updated_states["USV-1"]
        updated_progress = updated_progress_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_state.active_task_id, task.task_id)
        self.assertIsNone(updated_progress.pre_recovery_stage)
        self.assertEqual(updated_progress.task_final_approach_attempt_count, 1)
        self.assertEqual(updated_progress.task_final_approach_task_id, task.task_id)
        self.assertGreaterEqual(updated_progress.task_final_approach_candidate_index, 0)
        self.assertIsNotNone(updated_progress.task_final_approach_candidate_x)
        self.assertIsNotNone(updated_progress.task_final_approach_candidate_y)

    def test_advance_task_final_approach_after_failure_exhausts_on_failure_budget(self) -> None:
        grid_map = _build_runtime_grid_map()
        usv = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=835.846,
            y=627.599,
            heading_deg=78.856,
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-exhaustion",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=837.5,
            target_y=687.5,
            target_row=27,
            target_col=33,
            created_step=20,
            assigned_agent_id="USV-3",
        )
        progress_state = AgentProgressState(
            agent_id="USV-3",
            task_final_approach_task_id=task.task_id,
            task_final_approach_candidate_index=2,
            task_final_approach_attempt_count=2,
        )

        selection, exhausted = advance_task_final_approach_after_failure(
            usv,
            task=task,
            grid_map=grid_map,
            progress_state=progress_state,
        )

        self.assertTrue(exhausted)
        assert selection is not None
        self.assertEqual(selection.status, "exhausted")
        self.assertEqual(
            task_final_approach_release_cooldown_steps(task),
            180,
        )

    def test_recovery_success_rejoins_local_patrol_segment(self) -> None:
        agents = build_demo_agent_states()
        usv = next(agent for agent in agents if agent.agent_id == "USV-2")
        patrol_routes = build_patrol_routes()
        patrol_routes["USV-2"] = (
            (620.0, 120.0),
            (900.0, 120.0),
            (900.0, 420.0),
            (620.0, 420.0),
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-2"] = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.RECOVERY,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=2,
        )
        progress_states["USV-2"] = AgentProgressState(
            agent_id="USV-2",
            pre_recovery_stage=ExecutionStage.RETURN_TO_PATROL,
            recovery_step_index=2,
            blocked_goal_signature="return:2:900.0:420.0",
        )
        recovered_agent = replace(usv, x=870.0, y=260.0)
        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, True),
        ):
            _, updated_states, updated_progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(),
                patrol_routes=patrol_routes,
                grid_map=_build_runtime_grid_map(),
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
                step=12,
            )

        updated_state = updated_states["USV-2"]
        updated_progress = updated_progress_states["USV-2"]
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertTrue(updated_state.rejoin_to_segment)
        self.assertNotEqual(updated_state.return_target_y, 420.0)
        self.assertIsNotNone(updated_state.active_plan)
        assert updated_state.active_plan is not None
        self.assertEqual(updated_state.active_plan.status, PathPlanStatus.PLANNED)
        self.assertIsNone(updated_progress.pre_recovery_stage)

    def test_safe_patrol_access_candidate_rejects_too_close_access(self) -> None:
        grid_map = _build_runtime_grid_map()
        usv = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3")
        usv = replace(usv, x=942.976, y=753.197)
        patrol_route = (
            (920.0, 704.5),
            (920.0, 753.2),
            (987.5, 820.5),
        )
        access = PatrolSegmentAccess(
            access_x=933.593,
            access_y=737.197,
            segment_start_index=1,
            segment_end_index=2,
            access_distance=18.549,
        )

        self.assertFalse(
            _is_safe_patrol_access_candidate(
                usv,
                access,
                patrol_route=patrol_route,
                blocked_goal_signature=None,
                grid_map=grid_map,
                relax_endpoint_filter=False,
            )
        )

    def test_finalize_task_resolutions_pushes_on_task_agent_to_return_to_patrol(self) -> None:
        for hotspot_state in (
            HotspotKnowledgeState.CONFIRMED,
            HotspotKnowledgeState.FALSE_ALARM,
        ):
            with self.subTest(hotspot_state=hotspot_state):
                agents = build_demo_agent_states()
                patrol_routes = build_patrol_routes()
                execution_states = build_initial_execution_states(agents)
                progress_states = build_initial_progress_states(agents)
                execution_states["USV-1"] = AgentExecutionState(
                    agent_id="USV-1",
                    stage=ExecutionStage.ON_TASK,
                    active_task_id="hotspot-confirmation-5-5",
                    active_plan=None,
                    current_waypoint_index=0,
                    patrol_route_id="USV-1",
                    patrol_waypoint_index=0,
                )
                sea_map = build_default_sea_map()
                obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
                grid_map = build_grid_map(sea_map, obstacle_layout)
                info_map = build_information_map(grid_map)
                target_row = 5
                target_col = 5
                info_map.state_at(target_row, target_col).known_hotspot_state = hotspot_state
                cell = grid_map.cell_at(target_row, target_col)
                task = TaskRecord(
                    task_id="hotspot-confirmation-5-5",
                    task_type=TaskType.HOTSPOT_CONFIRMATION,
                    source=TaskSource.UAV_SUSPECTED,
                    status=TaskStatus.IN_PROGRESS,
                    priority=10,
                    target_x=cell.center_x,
                    target_y=cell.center_y,
                    target_row=target_row,
                    target_col=target_col,
                    created_step=2,
                    assigned_agent_id="USV-1",
                )

                (
                    resolved_tasks,
                    updated_execution_states,
                    updated_progress_states,
                ) = finalize_task_resolutions(
                    agents=agents,
                    execution_states=execution_states,
                    progress_states=progress_states,
                    task_records=(task,),
                    patrol_routes=patrol_routes,
                    info_map=info_map,
                    step=6,
                )

                self.assertEqual(resolved_tasks[0].status, TaskStatus.COMPLETED)
                self.assertEqual(resolved_tasks[0].completed_step, 6)
                self.assertEqual(
                    updated_execution_states["USV-1"].stage,
                    ExecutionStage.RETURN_TO_PATROL,
                )
                self.assertEqual(
                    updated_progress_states["USV-1"],
                    AgentProgressState(agent_id="USV-1"),
                )
                resolved_state = info_map.state_at(target_row, target_col)
                self.assertEqual(resolved_state.known_hotspot_state, HotspotKnowledgeState.NONE)
                self.assertEqual(resolved_state.hotspot_resolution_step, 6)

    def test_finalize_task_resolutions_completes_uav_resupply_and_returns_to_patrol(
        self,
    ) -> None:
        agents = tuple(
            replace(agent, energy_level=agent.energy_capacity * 0.9)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["UAV-1"] = AgentExecutionState(
            agent_id="UAV-1",
            stage=ExecutionStage.ON_RECHARGE,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="UAV-1",
            patrol_waypoint_index=0,
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.IN_PROGRESS,
            priority=20,
            target_x=110.0,
            target_y=180.0,
            target_row=None,
            target_col=None,
            created_step=2,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        resolved_tasks, updated_execution_states, _ = finalize_task_resolutions(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=patrol_routes,
            info_map=info_map,
            step=6,
        )

        self.assertEqual(resolved_tasks[0].status, TaskStatus.COMPLETED)
        self.assertEqual(
            updated_execution_states["UAV-1"].stage,
            ExecutionStage.RETURN_TO_PATROL,
        )

    def test_finalize_task_resolutions_keeps_uav_resupply_active_below_release_ratio(
        self,
    ) -> None:
        agents = tuple(
            replace(agent, energy_level=150.0) if agent.agent_id == "UAV-1" else agent
            for agent in build_demo_agent_states()
        )
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["UAV-1"] = AgentExecutionState(
            agent_id="UAV-1",
            stage=ExecutionStage.ON_RECHARGE,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="UAV-1",
            patrol_waypoint_index=0,
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.IN_PROGRESS,
            priority=20,
            target_x=110.0,
            target_y=180.0,
            target_row=None,
            target_col=None,
            created_step=2,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        resolved_tasks, updated_execution_states, _ = finalize_task_resolutions(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=patrol_routes,
            info_map=info_map,
            step=6,
        )

        self.assertEqual(resolved_tasks[0].status, TaskStatus.IN_PROGRESS)
        self.assertEqual(updated_execution_states["UAV-1"].stage, ExecutionStage.ON_RECHARGE)

    def test_sync_task_statuses_covers_assigned_in_progress_requeued_and_completed(
        self,
    ) -> None:
        execution_states = {
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id="task-assigned",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
            "USV-2": AgentExecutionState(
                agent_id="USV-2",
                stage=ExecutionStage.ON_TASK,
                active_task_id="task-in-progress",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-2",
                patrol_waypoint_index=0,
            ),
            "USV-3": AgentExecutionState(
                agent_id="USV-3",
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-3",
                patrol_waypoint_index=0,
            ),
        }
        progress_states = {
            "USV-1": AgentProgressState(agent_id="USV-1"),
            "USV-2": AgentProgressState(agent_id="USV-2"),
            "USV-3": AgentProgressState(agent_id="USV-3"),
        }
        task_records = (
            TaskRecord(
                task_id="task-assigned",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.PENDING,
                priority=10,
                target_x=10.0,
                target_y=20.0,
                target_row=1,
                target_col=2,
                created_step=1,
                assigned_agent_id="USV-1",
            ),
            TaskRecord(
                task_id="task-in-progress",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=30.0,
                target_y=40.0,
                target_row=3,
                target_col=4,
                created_step=1,
                assigned_agent_id="USV-2",
            ),
            TaskRecord(
                task_id="task-requeued",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.USV_ANOMALY,
                status=TaskStatus.ASSIGNED,
                priority=6,
                target_x=50.0,
                target_y=60.0,
                target_row=5,
                target_col=6,
                created_step=1,
                assigned_agent_id="USV-3",
            ),
            TaskRecord(
                task_id="task-completed",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.COMPLETED,
                priority=3,
                target_x=70.0,
                target_y=80.0,
                target_row=7,
                target_col=8,
                created_step=1,
                assigned_agent_id="USV-1",
                completed_step=5,
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)
        statuses = {task.task_id: task.status for task in synced_tasks}
        assignments = {task.task_id: task.assigned_agent_id for task in synced_tasks}

        self.assertEqual(statuses["task-assigned"], TaskStatus.ASSIGNED)
        self.assertEqual(statuses["task-in-progress"], TaskStatus.IN_PROGRESS)
        self.assertEqual(statuses["task-requeued"], TaskStatus.ASSIGNED)
        self.assertEqual(assignments["task-requeued"], "USV-3")
        self.assertEqual(statuses["task-completed"], TaskStatus.COMPLETED)

    def test_sync_task_statuses_keeps_new_assignment_for_idle_returning_agent(self) -> None:
        execution_states = {
            "USV-3": AgentExecutionState(
                agent_id="USV-3",
                stage=ExecutionStage.RETURN_TO_PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-3",
                patrol_waypoint_index=0,
            )
        }
        progress_states = {"USV-3": AgentProgressState(agent_id="USV-3")}
        task_records = (
            TaskRecord(
                task_id="hotspot-confirmation-35-27",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_INSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=687.5,
                target_y=887.5,
                target_row=35,
                target_col=27,
                created_step=537,
                assigned_agent_id="USV-3",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.ASSIGNED)
        self.assertEqual(synced_tasks[0].assigned_agent_id, "USV-3")

    def test_finalize_task_resolutions_clears_completed_task_owner(self) -> None:
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        execution_states["UAV-1"] = AgentExecutionState(
            agent_id="UAV-1",
            stage=ExecutionStage.ON_RECHARGE,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="UAV-1",
            patrol_waypoint_index=0,
        )
        progress_states = build_initial_progress_states(agents)
        uav = replace(
            next(agent for agent in agents if agent.agent_id == "UAV-1"),
            energy_level=195.0,
        )
        agents = tuple(uav if agent.agent_id == "UAV-1" else agent for agent in agents)
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.IN_PROGRESS,
            priority=20,
            target_x=100.0,
            target_y=200.0,
            target_row=None,
            target_col=None,
            created_step=10,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        resolved_tasks, _, _ = finalize_task_resolutions(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=patrol_routes,
            info_map=info_map,
            step=11,
        )

        self.assertEqual(resolved_tasks[0].status, TaskStatus.COMPLETED)
        self.assertIsNone(resolved_tasks[0].assigned_agent_id)
        self.assertIsNone(resolved_tasks[0].support_agent_id)

    def test_sync_task_statuses_adds_agent_retry_after_task_final_approach_release(self) -> None:
        execution_states = {
            "USV-3": AgentExecutionState(
                agent_id="USV-3",
                stage=ExecutionStage.RETURN_TO_PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-3",
                patrol_waypoint_index=0,
            )
        }
        progress_states = {
            "USV-3": AgentProgressState(
                agent_id="USV-3",
                released_task_id="hotspot-confirmation-3-26",
                released_task_retry_until_step=1280,
                released_task_reason="task_final_approach_exhausted",
            )
        }
        task_records = (
            TaskRecord(
                task_id="hotspot-confirmation-3-26",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=662.5,
                target_y=87.5,
                target_row=3,
                target_col=26,
                created_step=1005,
                assigned_agent_id="USV-3",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(synced_tasks[0].assigned_agent_id)
        self.assertEqual(synced_tasks[0].retry_after_step, 1280)
        self.assertEqual(
            synced_tasks[0].agent_retry_after_steps,
            (("USV-3", 1280),),
        )

    def test_finalize_task_resolutions_completes_baseline_service_and_returns_to_patrol(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.ON_TASK,
            active_task_id="baseline-service-4-2",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        target_row = 4
        target_col = 2
        baseline_state = info_map.state_at(target_row, target_col)
        baseline_state.has_baseline_task = True
        baseline_state.baseline_task_priority = 1
        baseline_state.baseline_service_progress = info_map.config.baseline_service_steps - 1
        cell = grid_map.cell_at(target_row, target_col)
        task = TaskRecord(
            task_id="baseline-service-4-2",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.IN_PROGRESS,
            priority=1,
            target_x=cell.center_x,
            target_y=cell.center_y,
            target_row=target_row,
            target_col=target_col,
            created_step=2,
            assigned_agent_id="USV-1",
        )

        (
            resolved_tasks,
            updated_execution_states,
            updated_progress_states,
        ) = finalize_task_resolutions(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=patrol_routes,
            info_map=info_map,
            step=6,
        )

        self.assertEqual(resolved_tasks[0].status, TaskStatus.COMPLETED)
        self.assertEqual(resolved_tasks[0].completed_step, 6)
        self.assertFalse(baseline_state.has_baseline_task)
        self.assertEqual(baseline_state.baseline_last_served_step, 6)
        self.assertEqual(baseline_state.baseline_service_progress, 0)
        self.assertEqual(
            updated_execution_states["USV-1"].stage,
            ExecutionStage.RETURN_TO_PATROL,
        )
        self.assertEqual(updated_progress_states["USV-1"], AgentProgressState(agent_id="USV-1"))


if __name__ == "__main__":
    unittest.main()
