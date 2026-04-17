import unittest
from dataclasses import replace
from math import cos, hypot, radians, sin
from types import SimpleNamespace
from unittest.mock import patch

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    AgentTaskState,
    ControlCommand,
    HealthStatus,
    TaskMode,
)
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
    _should_activate_local_mpc,
    follow_path_step_with_local_mpc as real_follow_path_step_with_local_mpc,
)
from usv_uav_marine_coverage.execution.progress_feedback import (
    build_initial_progress_states,
    evaluate_usv_progress,
    should_replan_task,
)
from usv_uav_marine_coverage.execution.local_mpc import compute_local_mpc_decision
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
    activate_task_approach_escalation,
    advance_task_final_approach_after_failure,
    apply_task_final_approach_selection,
    build_task_final_approach,
    current_task_approach_anchor,
    prepare_task_approach_state,
    rotate_task_approach_side,
    select_task_final_approach_candidate,
    task_approach_anchor_reached,
    task_final_approach_release_cooldown_steps,
    task_final_approach_satisfied,
)
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    InformationValidity,
    apply_usv_confirmation,
    build_information_map,
)
from usv_uav_marine_coverage.planning.astar_path_planner import (
    _point_has_clearance,
    build_astar_path_plan,
)
from usv_uav_marine_coverage.planning.path_types import PathPlan, PathPlanStatus, Waypoint
from usv_uav_marine_coverage.planning.usv_patrol_planner import PatrolSegmentAccess
from usv_uav_marine_coverage.simulation.simulation_agent_runtime import (
    _apply_bottleneck_directive,
    _apply_corridor_directive,
    _build_corridor_directives,
    _build_dynamic_bottleneck_directives,
    _evaluate_agent_progress,
    _pre_step_transition,
    advance_agents_one_step,
    build_initial_execution_states,
)
from usv_uav_marine_coverage.simulation.replay_validation import (
    summarize_replay_validation,
    validate_step_snapshot,
)
from usv_uav_marine_coverage.simulation.simulation_logging import serialize_task_record
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


def _resolve_test_initial_escort_follow_goal(agent: AgentState, escort_uav: AgentState, grid_map):
    direction_dx = 0.0
    direction_dy = 0.0
    if escort_uav.task.target_x is not None and escort_uav.task.target_y is not None:
        direction_dx = escort_uav.task.target_x - escort_uav.x
        direction_dy = escort_uav.task.target_y - escort_uav.y
    if abs(direction_dx) < 1e-6 and abs(direction_dy) < 1e-6:
        heading_rad = radians(escort_uav.heading_deg)
        direction_dx = cos(heading_rad)
        direction_dy = sin(heading_rad)
    direction_norm = hypot(direction_dx, direction_dy)
    if direction_norm < 1e-6:
        return (escort_uav.x, escort_uav.y)
    direction_dx /= direction_norm
    direction_dy /= direction_norm

    def _build_shadow_goal():
        separation = hypot(escort_uav.x - agent.x, escort_uav.y - agent.y)
        offset_m = -90.0
        if separation > 180.0:
            offset_m = 60.0
        return (
            escort_uav.x + direction_dx * offset_m,
            escort_uav.y + direction_dy * offset_m,
        )

    obstacle_layout = build_obstacle_layout(build_default_sea_map(), seed=20260314)
    corridor = min(
        obstacle_layout.traversable_corridors,
        key=lambda candidate: hypot(
            candidate.control_points[0][0] - agent.x,
            candidate.control_points[0][1] - agent.y,
        ),
    )
    corridor_start_x = corridor.control_points[0][0]
    corridor_end_x = corridor.control_points[-1][0]
    if escort_uav.x > corridor_end_x and agent.x > corridor_end_x:
        goal_x, goal_y = _build_shadow_goal()
    else:
        control_point_index = 0

        def _advance_to_clear_control_point(index: int) -> int:
            while index < len(corridor.control_points) - 1:
                point_x, point_y = corridor.control_points[index]
                if _point_has_clearance(grid_map, point_x, point_y):
                    break
                index += 1
            return index

        if agent.x >= corridor_start_x:
            while (
                control_point_index < len(corridor.control_points) - 1
                and agent.x >= corridor.control_points[control_point_index][0] - 5.0
            ):
                control_point_index += 1
        control_point_index = _advance_to_clear_control_point(control_point_index)
        goal_x, goal_y = corridor.control_points[control_point_index]
    goal_x = min(max(goal_x, 0.0), grid_map.width)
    goal_y = min(max(goal_y, 0.0), grid_map.height)
    return (goal_x, goal_y)


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
            step=0,
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
            step=0,
        )

        self.assertEqual(transitioned.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(transitioned.active_task_id, active_task.task_id)

    def test_initial_fixed_escort_phase_makes_usv_patrol_follow_paired_uav(self) -> None:
        agents = build_demo_agent_states()
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        obstacle_layout = build_obstacle_layout(build_default_sea_map(), seed=20260314)
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        next_agents, next_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=20,
        )

        original_by_id = {agent.agent_id: agent for agent in agents}
        next_by_id = {agent.agent_id: agent for agent in next_agents}
        expected_usv1_target = _resolve_test_initial_escort_follow_goal(
            original_by_id["USV-1"],
            escort_uav=original_by_id["UAV-1"],
            grid_map=grid_map,
        )
        expected_usv3_target = _resolve_test_initial_escort_follow_goal(
            original_by_id["USV-3"],
            escort_uav=original_by_id["UAV-2"],
            grid_map=grid_map,
        )

        self.assertAlmostEqual(
            next_by_id["USV-1"].task.target_x,
            expected_usv1_target[0],
            places=3,
        )
        self.assertAlmostEqual(
            next_by_id["USV-1"].task.target_y,
            expected_usv1_target[1],
            places=3,
        )
        self.assertAlmostEqual(
            next_by_id["USV-3"].task.target_x,
            expected_usv3_target[0],
            places=3,
        )
        self.assertAlmostEqual(
            next_by_id["USV-3"].task.target_y,
            expected_usv3_target[1],
            places=3,
        )
        self.assertEqual(next_execution_states["USV-1"].stage, ExecutionStage.PATROL)
        self.assertEqual(next_execution_states["USV-3"].stage, ExecutionStage.PATROL)

    def test_initial_fixed_escort_follow_commits_to_corridor_from_first_step(self) -> None:
        agents = build_demo_agent_states()
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        obstacle_layout = build_obstacle_layout(build_default_sea_map(), seed=20260314)
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        next_agents, _, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=1,
        )

        next_by_id = {agent.agent_id: agent for agent in next_agents}

        self.assertAlmostEqual(next_by_id["USV-1"].task.target_x, 250.0, places=3)
        self.assertLess(next_by_id["USV-1"].task.target_y, 500.0)
        self.assertAlmostEqual(next_by_id["USV-3"].task.target_x, 250.0, places=3)
        self.assertGreater(next_by_id["USV-3"].task.target_y, 500.0)

    def test_initial_fixed_escort_follow_expires_after_initial_phase(self) -> None:
        agents = build_demo_agent_states()
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        next_agents, _, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            dt_seconds=1.0,
            step=400,
        )

        original_by_id = {agent.agent_id: agent for agent in agents}
        next_by_id = {agent.agent_id: agent for agent in next_agents}

        self.assertNotAlmostEqual(
            next_by_id["USV-1"].task.target_x,
            original_by_id["UAV-1"].x,
            places=3,
        )
        self.assertNotAlmostEqual(
            next_by_id["USV-3"].task.target_x,
            original_by_id["UAV-2"].x,
            places=3,
        )

    def test_initial_fixed_escort_follow_uses_corridor_staging_inside_risk_zone(self) -> None:
        agents = tuple(
            replace(agent, x=320.0, y=190.0, heading_deg=-20.0)
            if agent.agent_id == "UAV-1"
            else replace(agent, x=150.0, y=230.0, heading_deg=10.0)
            if agent.agent_id == "USV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        next_agents, _, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=19,
        )

        usv1_after = next(agent for agent in next_agents if agent.agent_id == "USV-1")
        self.assertGreaterEqual(usv1_after.task.target_x, 250.0)
        self.assertLess(usv1_after.task.target_x, 450.0)
        self.assertLess(usv1_after.task.target_y, 500.0)

    def test_initial_fixed_escort_follow_leaves_corridor_after_uav_exits_risk_belt(self) -> None:
        agents = tuple(
            replace(agent, x=520.0, y=250.0, heading_deg=0.0)
            if agent.agent_id == "UAV-1"
            else replace(agent, x=460.0, y=240.0, heading_deg=10.0)
            if agent.agent_id == "USV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        next_agents, _, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=40,
        )

        usv1_after = next(agent for agent in next_agents if agent.agent_id == "USV-1")
        self.assertIsNone(usv1_after.task.target_x)
        self.assertIsNone(usv1_after.task.target_y)

    def test_initial_fixed_escort_follow_keeps_corridor_until_usv_exits_too(self) -> None:
        agents = tuple(
            replace(agent, x=520.0, y=250.0, heading_deg=0.0)
            if agent.agent_id == "UAV-1"
            else replace(agent, x=320.0, y=240.0, heading_deg=10.0)
            if agent.agent_id == "USV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)

        next_agents, _, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=40,
        )

        usv1_after = next(agent for agent in next_agents if agent.agent_id == "USV-1")
        self.assertLessEqual(usv1_after.task.target_x, 450.0)

    def test_initial_fixed_escort_follow_does_not_fall_back_to_previous_corridor_point(self) -> None:
        agents = tuple(
            replace(agent, x=248.0, y=296.0, heading_deg=-120.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=520.0, y=250.0, heading_deg=0.0)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        progress_states["USV-1"] = replace(
            progress_states["USV-1"],
            initial_escort_corridor_index=0,
            initial_escort_control_point_index=1,
        )

        next_agents, _, next_progress_states = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=40,
        )

        usv1_after = next(agent for agent in next_agents if agent.agent_id == "USV-1")
        self.assertGreater(usv1_after.task.target_x, 250.0)
        self.assertEqual(next_progress_states["USV-1"].initial_escort_control_point_index, 3)

    def test_initial_fixed_escort_follow_skips_blocked_corridor_control_points(self) -> None:
        agents = tuple(
            replace(agent, x=250.5, y=266.4, heading_deg=20.5)
            if agent.agent_id == "USV-1"
            else replace(agent, x=520.0, y=250.0, heading_deg=0.0)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260324)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        progress_states["USV-1"] = replace(
            progress_states["USV-1"],
            initial_escort_corridor_index=0,
            initial_escort_control_point_index=1,
        )

        next_agents, _, next_progress_states = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=46,
        )

        usv1_after = next(agent for agent in next_agents if agent.agent_id == "USV-1")
        self.assertGreaterEqual(usv1_after.task.target_x, 392.0)
        self.assertEqual(next_progress_states["USV-1"].initial_escort_control_point_index, 3)

    def test_initial_fixed_escort_phase_redirects_taskless_return_to_paired_uav(self) -> None:
        agents = build_demo_agent_states()
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_routes = build_patrol_routes()
        obstacle_layout = build_obstacle_layout(build_default_sea_map(), seed=20260314)
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = replace(
            execution_states["USV-1"],
            stage=ExecutionStage.RETURN_TO_PATROL,
            return_target_x=320.0,
            return_target_y=650.0,
        )
        execution_states["USV-3"] = replace(
            execution_states["USV-3"],
            stage=ExecutionStage.RETURN_TO_PATROL,
            return_target_x=640.0,
            return_target_y=912.0,
        )

        next_agents, next_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(),
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=1.0,
            step=20,
        )

        original_by_id = {agent.agent_id: agent for agent in agents}
        next_by_id = {agent.agent_id: agent for agent in next_agents}
        expected_usv1_target = _resolve_test_initial_escort_follow_goal(
            original_by_id["USV-1"],
            escort_uav=original_by_id["UAV-1"],
            grid_map=grid_map,
        )
        expected_usv3_target = _resolve_test_initial_escort_follow_goal(
            original_by_id["USV-3"],
            escort_uav=original_by_id["UAV-2"],
            grid_map=grid_map,
        )

        self.assertAlmostEqual(next_by_id["USV-1"].task.target_x, expected_usv1_target[0], places=3)
        self.assertAlmostEqual(next_by_id["USV-1"].task.target_y, expected_usv1_target[1], places=3)
        self.assertAlmostEqual(next_by_id["USV-3"].task.target_x, expected_usv3_target[0], places=3)
        self.assertAlmostEqual(next_by_id["USV-3"].task.target_y, expected_usv3_target[1], places=3)
        self.assertEqual(next_execution_states["USV-1"].stage, ExecutionStage.PATROL)
        self.assertEqual(next_execution_states["USV-3"].stage, ExecutionStage.PATROL)

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

    def test_build_dynamic_bottleneck_directives_prefers_uav_resupply_support_owner(self) -> None:
        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=543.369, y=143.434)
        agents[1] = replace(agents[1], x=571.808, y=94.321)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        patrol_routes = build_patrol_routes(agents=agents)

        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        execution_states["USV-2"] = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-confirmation-edge-2",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
        )
        active_tasks_by_agent = {
            "USV-1": TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=562.5,
                target_y=87.5,
                target_row=None,
                target_col=None,
                created_step=1,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
            "USV-2": TaskRecord(
                task_id="hotspot-confirmation-edge-2",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=562.5,
                target_y=112.5,
                target_row=4,
                target_col=22,
                created_step=1,
                assigned_agent_id="USV-2",
            ),
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
        owner_directive = next(
            directive for directive in directives.values() if not directive.should_yield
        )
        self.assertEqual(owner_directive.owner_agent_id, "USV-1")

    def test_build_dynamic_bottleneck_directives_includes_recovery_stage_resupply_support(self) -> None:
        agents = list(build_demo_agent_states())
        agents[0] = replace(agents[0], x=543.369, y=143.434)
        agents[1] = replace(agents[1], x=571.808, y=94.321)
        agents = tuple(agents)
        execution_states = build_initial_execution_states(agents)
        patrol_routes = build_patrol_routes(agents=agents)

        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RECOVERY,
            active_task_id="uav-resupply-UAV-1",
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
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=562.5,
                target_y=87.5,
                target_row=None,
                target_col=None,
                created_step=1,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
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

    def test_apply_bottleneck_directive_keeps_yield_state_until_sticky_expiry(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.YIELD,
            active_task_id="hotspot-3",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
            yield_target_x=688.0,
            yield_target_y=873.0,
            yield_reason="dynamic_bottleneck_wait_for_owner",
            reserved_bottleneck_zone_id="zone-edge-1",
            bottleneck_owner_agent_id="USV-1",
            bottleneck_reservation_until_step=246,
            pre_yield_stage=ExecutionStage.GO_TO_TASK,
        )

        updated_state = _apply_bottleneck_directive(
            state,
            directive=None,
            step=244,
        )

        self.assertEqual(updated_state.stage, ExecutionStage.YIELD)
        self.assertEqual(updated_state.reserved_bottleneck_zone_id, "zone-edge-1")
        self.assertEqual(updated_state.bottleneck_owner_agent_id, "USV-1")

    def test_apply_bottleneck_directive_releases_yield_after_sticky_expiry(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.YIELD,
            active_task_id="hotspot-3",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
            yield_target_x=688.0,
            yield_target_y=873.0,
            yield_reason="dynamic_bottleneck_wait_for_owner",
            reserved_bottleneck_zone_id="zone-edge-1",
            bottleneck_owner_agent_id="USV-1",
            bottleneck_reservation_until_step=246,
            pre_yield_stage=ExecutionStage.GO_TO_TASK,
        )

        updated_state = _apply_bottleneck_directive(
            state,
            directive=None,
            step=247,
        )

        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertIsNone(updated_state.reserved_bottleneck_zone_id)
        self.assertIsNone(updated_state.bottleneck_owner_agent_id)

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

    def test_evaluate_agent_progress_enters_recovery_for_stalled_uav_support_transit(
        self,
    ) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3")
        task = TaskRecord(
            task_id="uav-resupply-UAV-2",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=512.5,
            target_y=662.5,
            target_row=None,
            target_col=None,
            created_step=88,
            assigned_agent_id="UAV-2",
            support_agent_id="USV-3",
        )
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id=task.task_id,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=2,
        )
        progress_state = AgentProgressState(
            agent_id="USV-3",
            stalled_steps=2,
            last_target_distance=hypot(task.target_x - agent.x, task.target_y - agent.y),
        )
        patrol_route = build_patrol_routes()["USV-3"]

        updated_execution_state, updated_progress_state = _evaluate_agent_progress(
            agent,
            updated_agent=agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=task,
            patrol_routes={"USV-3": patrol_route},
            step=285,
        )

        self.assertEqual(updated_execution_state.stage, ExecutionStage.RECOVERY)
        self.assertEqual(updated_progress_state.pre_recovery_stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_progress_state.pre_recovery_task_id, task.task_id)

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
        self.assertIsNotNone(usv_state.active_plan)
        assert usv_state.active_plan is not None
        self.assertEqual(usv_state.active_plan.task_id, task.task_id)
        self.assertGreaterEqual(usv_state.current_waypoint_index, 0)

    def test_should_replan_task_tolerates_small_uav_resupply_support_goal_drift(self) -> None:
        usv = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=520.0,
            target_y=240.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
            rendezvous_anchor_x=520.0,
            rendezvous_anchor_y=240.0,
        )
        plan = PathPlan(
            plan_id="plan-uav-resupply-small-drift",
            planner_name="astar_path_planner",
            agent_id="USV-1",
            task_id=task.task_id,
            status=PathPlanStatus.PLANNED,
            waypoints=(Waypoint(x=160.0, y=180.0), Waypoint(x=200.0, y=210.0)),
            goal_x=520.0,
            goal_y=240.0,
            estimated_cost=85.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id=task.task_id,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )

        should_replan = should_replan_task(
            usv,
            execution_state=execution_state,
            active_task=task,
            goal_x=545.0,
            goal_y=255.0,
        )

        self.assertFalse(should_replan)

    def test_should_replan_task_replans_uav_resupply_support_after_large_goal_drift(
        self,
    ) -> None:
        usv = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=520.0,
            target_y=240.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
            rendezvous_anchor_x=520.0,
            rendezvous_anchor_y=240.0,
        )
        plan = PathPlan(
            plan_id="plan-uav-resupply-large-drift",
            planner_name="astar_path_planner",
            agent_id="USV-1",
            task_id=task.task_id,
            status=PathPlanStatus.PLANNED,
            waypoints=(Waypoint(x=160.0, y=180.0), Waypoint(x=200.0, y=210.0)),
            goal_x=520.0,
            goal_y=240.0,
            estimated_cost=85.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id=task.task_id,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )

        should_replan = should_replan_task(
            usv,
            execution_state=execution_state,
            active_task=task,
            goal_x=565.0,
            goal_y=290.0,
        )

        self.assertTrue(should_replan)

    def test_advance_agents_one_step_keeps_uav_resupply_support_plan_for_small_uav_drift(
        self,
    ) -> None:
        base_agents = tuple(
            replace(agent, x=545.0, y=255.0, energy_level=20.0)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=520.0,
            target_y=240.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
            rendezvous_anchor_x=520.0,
            rendezvous_anchor_y=240.0,
        )
        original_usv = next(agent for agent in base_agents if agent.agent_id == "USV-1")
        plan = build_astar_path_plan(
            original_usv,
            grid_map=grid_map,
            goal_x=520.0,
            goal_y=240.0,
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

        updated_agents, updated_execution_states, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
            step=400,
        )

        usv_state = updated_execution_states["USV-1"]
        usv_after = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        self.assertIs(usv_state.active_plan, plan)
        self.assertEqual(usv_after.task.target_x, plan.goal_x)
        self.assertEqual(usv_after.task.target_y, plan.goal_y)

    def test_initial_fixed_escort_support_tracks_live_uav_position(self) -> None:
        agents = tuple(
            replace(agent, x=545.0, y=255.0, energy_level=20.0)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=520.0,
            target_y=240.0,
            target_row=None,
            target_col=None,
            created_step=3,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
            rendezvous_anchor_x=520.0,
            rendezvous_anchor_y=240.0,
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id=task.task_id,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )

        updated_agents, _, _ = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=(task,),
            patrol_routes=build_patrol_routes(),
            grid_map=grid_map,
            dt_seconds=1.0,
            step=20,
        )

        original_by_id = {agent.agent_id: agent for agent in agents}
        updated_by_id = {agent.agent_id: agent for agent in updated_agents}
        self.assertEqual(updated_by_id["USV-1"].task.target_x, original_by_id["UAV-1"].x)
        self.assertEqual(updated_by_id["USV-1"].task.target_y, original_by_id["UAV-1"].y)

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
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.ON_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
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

    def test_advance_agents_one_step_holds_support_usv_stationary_during_uav_resupply(
        self,
    ) -> None:
        agents = tuple(
            replace(agent, x=210.0, y=320.0, speed_mps=4.0, turn_rate_degps=6.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=216.0, y=320.0, energy_level=20.0)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.IN_PROGRESS,
            priority=20,
            target_x=210.0,
            target_y=320.0,
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

        usv_after = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        self.assertEqual(updated_execution_states["USV-1"].stage, ExecutionStage.ON_TASK)
        self.assertEqual(usv_after.task.mode, TaskMode.IDLE)
        self.assertEqual(usv_after.speed_mps, 0.0)
        self.assertEqual(usv_after.turn_rate_degps, 0.0)

    def test_advance_agents_one_step_stabilizes_support_usv_after_recharge_snap(
        self,
    ) -> None:
        agents = tuple(
            replace(agent, x=210.0, y=320.0, speed_mps=4.0, turn_rate_degps=6.0)
            if agent.agent_id == "USV-1"
            else replace(agent, x=260.0, y=320.0, energy_level=20.0)
            if agent.agent_id == "UAV-1"
            else agent
            for agent in build_demo_agent_states()
        )
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
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
            target_x=210.0,
            target_y=320.0,
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
        self.assertEqual((uav_after.x, uav_after.y), (usv_after.x, usv_after.y))
        self.assertEqual(updated_execution_states["UAV-1"].stage, ExecutionStage.ON_RECHARGE)
        self.assertEqual(updated_execution_states["USV-1"].stage, ExecutionStage.ON_TASK)
        self.assertEqual(usv_after.task.mode, TaskMode.IDLE)
        self.assertEqual(usv_after.speed_mps, 0.0)
        self.assertEqual(usv_after.turn_rate_degps, 0.0)

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
            step=400,
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

    def test_build_local_patrol_return_transition_uses_coarse_rejoin_after_return_replan(
        self,
    ) -> None:
        grid_map = _build_runtime_grid_map()
        info_map = build_information_map(grid_map)
        patrol_route = (
            (520.0, 580.0),
            (920.0, 580.0),
            (920.0, 880.0),
            (520.0, 880.0),
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=936.0,
            y=770.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
        )
        progress_state = AgentProgressState(
            agent_id="USV-3",
            return_replan_generation=1,
            blocked_goal_signature="return:0:887.5:762.5",
        )

        updated_state = _build_local_patrol_return_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_route,
            grid_map=grid_map,
            info_map=info_map,
            task_records=(),
            skip_blocked_goal=True,
        )

        self.assertNotEqual(updated_state.return_target_source, "safe_value_access")

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
            step=400,
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
            step=400,
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
            updated_agents, updated_execution_states, updated_progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=patrol_routes,
                grid_map=grid_map,
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
            )

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        updated_state = updated_execution_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertIsNone(updated_state.active_task_id)
        self.assertIsNone(updated_state.active_plan)
        self.assertEqual(updated_agent.speed_mps, 0.0)
        self.assertEqual(updated_agent.turn_rate_degps, 0.0)
        self.assertEqual(updated_progress_states["USV-1"].released_task_id, task.task_id)
        self.assertEqual(
            updated_progress_states["USV-1"].released_task_reason,
            "task_plan_blocked",
        )
        self.assertGreater(updated_progress_states["USV-1"].released_task_retry_until_step, 0)

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
            updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
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

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        updated_state = updated_states["USV-1"]
        updated_progress = updated_progress_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_state.active_task_id, task.task_id)
        self.assertIsNone(updated_progress.pre_recovery_stage)
        self.assertEqual(updated_progress.task_approach_task_id, task.task_id)
        self.assertIn(updated_progress.task_approach_active_side, {"left", "right"})
        self.assertEqual(updated_progress.task_approach_anchor_status, "enroute_anchor")
        anchor = current_task_approach_anchor(updated_progress, task_id=task.task_id)
        self.assertIsNotNone(anchor)
        assert anchor is not None
        self.assertEqual(
            updated_agent.task.target_x,
            anchor[0],
        )
        self.assertEqual(
            updated_agent.task.target_y,
            anchor[1],
        )

    def test_recovery_does_not_resume_hotspot_task_inside_offshore_risk_buffer(self) -> None:
        agents = tuple(
            replace(agent, x=993.825, y=884.13, heading_deg=-102.707)
            if agent.agent_id == "USV-3"
            else agent
            for agent in build_demo_agent_states()
        )
        usv = next(agent for agent in agents if agent.agent_id == "USV-3")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-3"] = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RECOVERY,
            active_task_id="hotspot-confirmation-31-24",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
        )
        progress_states["USV-3"] = AgentProgressState(
            agent_id="USV-3",
            pre_recovery_stage=ExecutionStage.GO_TO_TASK,
            pre_recovery_task_id="hotspot-confirmation-31-24",
            recovery_step_index=2,
            blocked_goal_signature="task:hotspot-confirmation-31-24",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-31-24",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=612.5,
            target_y=787.5,
            target_row=31,
            target_col=24,
            created_step=996,
            assigned_agent_id="USV-3",
        )
        recovered_agent = replace(
            usv,
            x=993.988,
            y=883.533,
            heading_deg=-74.707,
            speed_mps=0.619,
        )

        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, True),
        ), patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._find_local_patrol_access",
            return_value=SimpleNamespace(access=SimpleNamespace(access_x=0.0, access_y=0.0)),
        ):
            updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=build_patrol_routes(),
                grid_map=_build_runtime_grid_map(),
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
                step=1078,
            )

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-3")
        updated_state = updated_states["USV-3"]
        updated_progress = updated_progress_states["USV-3"]
        self.assertEqual(updated_state.stage, ExecutionStage.RECOVERY)
        self.assertEqual(updated_state.active_task_id, task.task_id)
        self.assertEqual(updated_progress.recovery_step_index, 3)
        self.assertEqual(updated_progress.pre_recovery_stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_progress.pre_recovery_task_id, task.task_id)
        self.assertEqual(updated_agent.task.target_x, recovered_agent.task.target_x)
        self.assertEqual(updated_agent.task.target_y, recovered_agent.task.target_y)

    def test_recovery_success_returns_uav_resupply_support_without_final_approach_release(
        self,
    ) -> None:
        agents = tuple(
            replace(agent, energy_level=20.0) if agent.agent_id == "UAV-1" else agent
            for agent in build_demo_agent_states()
        )
        usv = next(agent for agent in agents if agent.agent_id == "USV-1")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RECOVERY,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        progress_states["USV-1"] = AgentProgressState(
            agent_id="USV-1",
            pre_recovery_stage=ExecutionStage.GO_TO_TASK,
            pre_recovery_task_id="uav-resupply-UAV-1",
            recovery_step_index=2,
            blocked_goal_signature="task:uav-resupply-UAV-1",
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
            created_step=5,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        recovered_agent = replace(usv, x=usv.x + 12.0, y=usv.y + 4.0)
        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, True),
        ), patch(
            "usv_uav_marine_coverage.execution.recovery_runtime.advance_task_final_approach_after_failure",
            side_effect=AssertionError("uav_resupply should not use task final approach recovery"),
        ):
            updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
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

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        updated_state = updated_states["USV-1"]
        updated_progress = updated_progress_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_state.active_task_id, task.task_id)
        self.assertIsNone(updated_state.active_plan)
        self.assertIsNone(updated_progress.pre_recovery_stage)
        self.assertIsNone(updated_progress.released_task_id)
        self.assertIsNone(updated_progress.released_task_reason)
        self.assertIsNone(updated_progress.task_final_approach_task_id)
        self.assertEqual(updated_agent.task.mode, TaskMode.CONFIRM)

    def test_recovery_exhaustion_keeps_uav_resupply_support_task_assigned(self) -> None:
        agents = tuple(
            replace(agent, energy_level=20.0) if agent.agent_id == "UAV-1" else agent
            for agent in build_demo_agent_states()
        )
        usv = next(agent for agent in agents if agent.agent_id == "USV-1")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RECOVERY,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        progress_states["USV-1"] = AgentProgressState(
            agent_id="USV-1",
            pre_recovery_stage=ExecutionStage.GO_TO_TASK,
            pre_recovery_task_id="uav-resupply-UAV-1",
            recovery_attempts=1,
            recovery_step_index=5,
            blocked_goal_signature="task:uav-resupply-UAV-1",
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
            created_step=5,
            assigned_agent_id="UAV-1",
            support_agent_id="USV-1",
        )

        recovered_agent = replace(usv, x=usv.x + 4.0, y=usv.y + 2.0)
        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, False),
        ):
            updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
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

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-1")
        updated_state = updated_states["USV-1"]
        updated_progress = updated_progress_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_state.active_task_id, task.task_id)
        self.assertEqual(updated_agent.task.mode, TaskMode.CONFIRM)
        self.assertEqual(updated_agent.task.target_x, task.target_x)
        self.assertEqual(updated_agent.task.target_y, task.target_y)
        self.assertIsNone(updated_progress.released_task_id)
        self.assertIsNone(updated_progress.released_task_reason)

    def test_advance_task_final_approach_after_failure_exhausts_after_all_frozen_candidates_fail(
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
            task_final_approach_frozen_candidates=(
                (824.5, 687.5),
                (837.5, 674.5),
                (850.5, 687.5),
            ),
            task_final_approach_candidate_index=2,
            task_final_approach_failed_candidate_indexes=(0, 1),
            task_final_approach_attempted_candidate_indexes=(0, 1, 2),
            task_final_approach_attempt_count=2,
            task_final_approach_backoff_task_id=task.task_id,
            task_final_approach_hold_reset_count=2,
        )

        selection, exhausted, hold_reset = advance_task_final_approach_after_failure(
            usv,
            task=task,
            grid_map=grid_map,
            progress_state=progress_state,
        )

        self.assertTrue(exhausted)
        self.assertFalse(hold_reset)
        assert selection is not None
        self.assertEqual(selection.status, "exhausted")
        self.assertEqual(selection.failed_candidate_indexes, (0, 1, 2))
        self.assertEqual(
            task_final_approach_release_cooldown_steps(task),
            180,
        )

    def test_task_final_approach_selection_keeps_frozen_candidates_across_agent_motion(self) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-stable",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=100,
            assigned_agent_id="USV-1",
        )
        initial_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=561.121,
            y=469.844,
            heading_deg=-129.409,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=task.target_x, target_y=task.target_y),
        )
        moved_agent = replace(
            initial_agent,
            x=537.266,
            y=427.654,
            heading_deg=-115.0,
        )
        initial_progress = AgentProgressState(agent_id="USV-1")
        initial_selection = select_task_final_approach_candidate(
            initial_agent,
            task=task,
            grid_map=grid_map,
            progress_state=initial_progress,
        )
        dynamic_after_motion = build_task_final_approach(
            moved_agent,
            task=task,
            grid_map=grid_map,
        )

        frozen_progress = apply_task_final_approach_selection(
            initial_progress,
            selection=initial_selection,
        )
        frozen_selection = select_task_final_approach_candidate(
            moved_agent,
            task=task,
            grid_map=grid_map,
            progress_state=frozen_progress,
        )

        self.assertNotEqual(
            (dynamic_after_motion.candidates[0].x, dynamic_after_motion.candidates[0].y),
            (initial_selection.candidate_x, initial_selection.candidate_y),
        )
        self.assertEqual(
            (frozen_selection.candidate_x, frozen_selection.candidate_y),
            (initial_selection.candidate_x, initial_selection.candidate_y),
        )
        self.assertEqual(
            frozen_selection.frozen_candidates,
            frozen_progress.task_final_approach_frozen_candidates,
        )

    def test_advance_task_final_approach_after_failure_uses_frozen_candidate_order(self) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-rotate",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=100,
            assigned_agent_id="USV-1",
        )
        initial_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=561.121,
            y=469.844,
            heading_deg=-129.409,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=task.target_x, target_y=task.target_y),
        )
        moved_agent = replace(
            initial_agent,
            x=509.633,
            y=333.288,
            heading_deg=-79.611,
        )
        initial_progress = AgentProgressState(agent_id="USV-1")
        initial_selection = select_task_final_approach_candidate(
            initial_agent,
            task=task,
            grid_map=grid_map,
            progress_state=initial_progress,
        )
        frozen_progress = apply_task_final_approach_selection(
            initial_progress,
            selection=initial_selection,
        )
        dynamic_after_motion = build_task_final_approach(
            moved_agent,
            task=task,
            grid_map=grid_map,
        )

        rotated_selection, exhausted, hold_reset = advance_task_final_approach_after_failure(
            moved_agent,
            task=task,
            grid_map=grid_map,
            progress_state=frozen_progress,
        )

        self.assertFalse(exhausted)
        self.assertFalse(hold_reset)
        assert rotated_selection is not None
        self.assertEqual(rotated_selection.candidate_index, 1)
        self.assertEqual(rotated_selection.failed_candidate_indexes, (0,))
        self.assertEqual(
            (rotated_selection.candidate_x, rotated_selection.candidate_y),
            frozen_progress.task_final_approach_frozen_candidates[1],
        )
        self.assertNotEqual(
            (dynamic_after_motion.candidates[1].x, dynamic_after_motion.candidates[1].y),
            (rotated_selection.candidate_x, rotated_selection.candidate_y),
        )

    def test_advance_task_final_approach_after_failure_wraps_to_next_unfailed_candidate(self) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-wrap",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=100,
            assigned_agent_id="USV-1",
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=509.633,
            y=333.288,
            heading_deg=-79.611,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=task.target_x, target_y=task.target_y),
        )
        frozen_candidates = tuple(
            (candidate.x, candidate.y)
            for candidate in build_task_final_approach(agent, task=task, grid_map=grid_map).candidates[:4]
        )
        progress_state = AgentProgressState(
            agent_id="USV-1",
            task_final_approach_task_id=task.task_id,
            task_final_approach_frozen_candidates=frozen_candidates,
            task_final_approach_candidate_index=2,
            task_final_approach_candidate_x=frozen_candidates[2][0],
            task_final_approach_candidate_y=frozen_candidates[2][1],
            task_final_approach_failed_candidate_indexes=(0, 2, 3),
            task_final_approach_attempted_candidate_indexes=(0, 2, 3),
            task_final_approach_attempt_count=3,
        )

        rotated_selection, exhausted, hold_reset = advance_task_final_approach_after_failure(
            agent,
            task=task,
            grid_map=grid_map,
            progress_state=progress_state,
        )

        self.assertFalse(exhausted)
        self.assertFalse(hold_reset)
        assert rotated_selection is not None
        self.assertEqual(rotated_selection.candidate_index, 1)
        self.assertEqual(rotated_selection.failed_candidate_indexes, (0, 2, 3))

    def test_select_task_final_approach_candidate_skips_failed_current_candidate(self) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-skip-failed",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=100,
            assigned_agent_id="USV-1",
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=537.266,
            y=427.654,
            heading_deg=-115.0,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=task.target_x, target_y=task.target_y),
        )
        initial_selection = select_task_final_approach_candidate(
            agent,
            task=task,
            grid_map=grid_map,
            progress_state=AgentProgressState(agent_id="USV-1"),
        )
        progress_state = AgentProgressState(
            agent_id="USV-1",
            task_final_approach_task_id=task.task_id,
            task_final_approach_frozen_candidates=initial_selection.frozen_candidates,
            task_final_approach_candidate_index=initial_selection.candidate_index,
            task_final_approach_candidate_x=initial_selection.candidate_x,
            task_final_approach_candidate_y=initial_selection.candidate_y,
            task_final_approach_failed_candidate_indexes=(initial_selection.candidate_index,),
            task_final_approach_attempted_candidate_indexes=(initial_selection.candidate_index,),
            task_final_approach_attempt_count=1,
            task_final_approach_status="rotated_after_recovery",
        )

        selection = select_task_final_approach_candidate(
            agent,
            task=task,
            grid_map=grid_map,
            progress_state=progress_state,
        )

        self.assertNotEqual(selection.candidate_index, initial_selection.candidate_index)
        self.assertEqual(
            selection.failed_candidate_indexes,
            (initial_selection.candidate_index,),
        )

    def test_advance_task_final_approach_after_failure_requires_attempted_cycle_before_exhaustion(
        self,
    ) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-attempted-cycle",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=100,
            assigned_agent_id="USV-1",
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=509.633,
            y=333.288,
            heading_deg=-79.611,
            task=AgentTaskState(mode=TaskMode.CONFIRM, target_x=task.target_x, target_y=task.target_y),
        )
        frozen_candidates = tuple(
            (candidate.x, candidate.y)
            for candidate in build_task_final_approach(agent, task=task, grid_map=grid_map).candidates[:3]
        )
        progress_state = AgentProgressState(
            agent_id="USV-1",
            task_final_approach_task_id=task.task_id,
            task_final_approach_frozen_candidates=frozen_candidates,
            task_final_approach_candidate_index=2,
            task_final_approach_candidate_x=frozen_candidates[2][0],
            task_final_approach_candidate_y=frozen_candidates[2][1],
            task_final_approach_failed_candidate_indexes=(0, 1),
            task_final_approach_attempted_candidate_indexes=(0,),
            task_final_approach_attempt_count=2,
        )

        rotated_selection, exhausted, hold_reset = advance_task_final_approach_after_failure(
            agent,
            task=task,
            grid_map=grid_map,
            progress_state=progress_state,
        )

        self.assertFalse(exhausted)
        self.assertFalse(hold_reset)
        assert rotated_selection is not None
        self.assertEqual(rotated_selection.candidate_index, 1)

    def test_activate_task_approach_escalation_builds_frozen_left_right_anchors(self) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-anchor",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=662.5,
            target_y=262.5,
            target_row=10,
            target_col=26,
            created_step=50,
            assigned_agent_id="USV-3",
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=720.0,
            y=420.0,
            heading_deg=-110.0,
        )
        progress_state = activate_task_approach_escalation(
            AgentProgressState(agent_id="USV-3"),
            agent=agent,
            task=task,
            grid_map=grid_map,
            step=610,
        )

        self.assertEqual(progress_state.task_approach_escalation_task_id, task.task_id)
        self.assertEqual(progress_state.task_approach_task_id, task.task_id)
        self.assertIn(progress_state.task_approach_active_side, {"left", "right"})
        self.assertIsNotNone(progress_state.task_approach_anchor_left_x)
        self.assertIsNotNone(progress_state.task_approach_anchor_left_y)
        self.assertIsNotNone(progress_state.task_approach_anchor_right_x)
        self.assertIsNotNone(progress_state.task_approach_anchor_right_y)
        self.assertNotEqual(
            (
                progress_state.task_approach_anchor_left_x,
                progress_state.task_approach_anchor_left_y,
            ),
            (
                progress_state.task_approach_anchor_right_x,
                progress_state.task_approach_anchor_right_y,
            ),
        )

    def test_rotate_task_approach_side_switches_to_opposite_anchor(self) -> None:
        progress_state = AgentProgressState(
            agent_id="USV-3",
            task_approach_escalation_task_id="hotspot-confirmation-anchor",
            task_approach_task_id="hotspot-confirmation-anchor",
            task_approach_anchor_left_x=610.0,
            task_approach_anchor_left_y=240.0,
            task_approach_anchor_right_x=710.0,
            task_approach_anchor_right_y=240.0,
            task_approach_active_side="left",
            task_approach_anchor_status="enroute_anchor",
        )

        rotated = rotate_task_approach_side(
            progress_state,
            task_id="hotspot-confirmation-anchor",
            step=610,
        )

        self.assertEqual(rotated.task_approach_active_side, "right")
        self.assertEqual(rotated.task_approach_anchor_status, "enroute_anchor")
        self.assertEqual(rotated.task_approach_failed_sides, ("left",))
        self.assertEqual(
            current_task_approach_anchor(rotated, task_id="hotspot-confirmation-anchor"),
            (710.0, 240.0),
        )
        self.assertEqual(rotated.task_approach_commit_until_step, 634)

    def test_task_approach_anchor_reached_stays_latched_in_final_approach(self) -> None:
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=868.5,
            y=399.2,
        )
        progress_state = AgentProgressState(
            agent_id="USV-3",
            task_approach_escalation_task_id="hotspot-confirmation-12-31",
            task_approach_task_id="hotspot-confirmation-12-31",
            task_approach_anchor_left_x=895.912,
            task_approach_anchor_left_y=409.952,
            task_approach_anchor_right_x=924.504,
            task_approach_anchor_right_y=262.702,
            task_approach_active_side="left",
            task_approach_anchor_status="final_approach",
        )

        self.assertTrue(
            task_approach_anchor_reached(
                progress_state,
                agent=agent,
                task_id="hotspot-confirmation-12-31",
            )
        )

    def test_task_approach_anchor_reached_uses_geometry_before_final_approach(self) -> None:
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3"),
            x=868.5,
            y=399.2,
        )
        progress_state = AgentProgressState(
            agent_id="USV-3",
            task_approach_escalation_task_id="hotspot-confirmation-12-31",
            task_approach_task_id="hotspot-confirmation-12-31",
            task_approach_anchor_left_x=895.912,
            task_approach_anchor_left_y=409.952,
            task_approach_anchor_right_x=924.504,
            task_approach_anchor_right_y=262.702,
            task_approach_active_side="left",
            task_approach_anchor_status="enroute_anchor",
        )

        self.assertFalse(
            task_approach_anchor_reached(
                progress_state,
                agent=agent,
                task_id="hotspot-confirmation-12-31",
            )
        )

    def test_baseline_task_does_not_initialize_macro_approach_anchors(self) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="baseline-service-21-1",
            task_type=TaskType.BASELINE_SERVICE,
            source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
            status=TaskStatus.ASSIGNED,
            priority=3,
            target_x=37.5,
            target_y=537.5,
            target_row=21,
            target_col=1,
            created_step=22,
            assigned_agent_id="USV-2",
        )
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        progress_state = prepare_task_approach_state(
            AgentProgressState(agent_id="USV-2"),
            agent=agent,
            task=task,
            grid_map=grid_map,
        )

        self.assertIsNone(progress_state.task_approach_escalation_task_id)
        self.assertIsNone(progress_state.task_approach_task_id)
        self.assertIsNone(progress_state.task_approach_active_side)

    def test_compute_local_mpc_decision_penalizes_low_progress_lateral_drift(self) -> None:
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
            x=900.0,
            y=520.0,
            heading_deg=0.0,
        )
        tracking_target = Waypoint(x=900.5, y=562.5)
        low_progress_command = ControlCommand(target_speed_mps=1.0, target_heading_deg=30.0)
        converging_command = ControlCommand(target_speed_mps=2.0, target_heading_deg=90.0)

        with patch(
            "usv_uav_marine_coverage.execution.local_mpc._candidate_commands",
            return_value=(low_progress_command, converging_command),
        ), patch(
            "usv_uav_marine_coverage.execution.local_mpc._simulate_rollout",
            side_effect=[
                (41.8, 9.0, 7.0),
                (35.0, 8.2, 0.3),
            ],
        ):
            decision = compute_local_mpc_decision(
                agent,
                tracking_target=tracking_target,
                dt_seconds=1.0,
                obstacle_layout=None,
            )

        self.assertEqual(decision.command, converging_command)
        self.assertAlmostEqual(decision.predicted_terminal_distance_m, 35.0)
        self.assertAlmostEqual(decision.predicted_lateral_drift_m, 0.3)

    def test_should_activate_local_mpc_skips_far_go_to_task_risk_area(self) -> None:
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
            x=600.0,
            y=300.0,
            heading_deg=0.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-confirmation-22-34",
            active_plan=PathPlan(
                plan_id="plan-1",
                planner_name="astar_smoother_path_planner",
                agent_id="USV-2",
                task_id="hotspot-confirmation-22-34",
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=650.0, y=300.0), Waypoint(x=700.0, y=300.0)),
                goal_x=700.0,
                goal_y=300.0,
                estimated_cost=100.0,
            ),
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
        )
        obstacle_layout = ObstacleLayout(
            seed=1,
            traversable_corridors=(),
            risk_zone_obstacles=(),
            offshore_features=(
                CircularFeature(
                    name="risk-bubble",
                    feature_type="risk_area",
                    x=625.0,
                    y=304.0,
                    radius=12.0,
                ),
            ),
            nearshore_monitor_points=(),
            offshore_hotspots=(),
        )

        should_activate = _should_activate_local_mpc(
            agent,
            execution_state=execution_state,
            tracking_target=Waypoint(x=610.0, y=300.0),
            obstacle_layout=obstacle_layout,
            neighboring_agents=(),
            wreck_zones=(),
            grid_width=1000.0,
            grid_height=1000.0,
        )

        self.assertFalse(should_activate)

    def test_should_activate_local_mpc_for_nearby_usv_during_far_go_to_task(self) -> None:
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
            x=485.5,
            y=286.919,
            heading_deg=0.394,
            speed_mps=5.499,
        )
        neighboring_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=489.052,
            y=288.728,
            heading_deg=-157.95,
            speed_mps=0.0,
        )
        execution_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="hotspot-confirmation-5-34",
            active_plan=PathPlan(
                plan_id="plan-1",
                planner_name="astar_smoother_path_planner",
                agent_id="USV-2",
                task_id="hotspot-confirmation-5-34",
                status=PathPlanStatus.PLANNED,
                waypoints=(Waypoint(x=490.0, y=287.0), Waypoint(x=889.37, y=164.37)),
                goal_x=889.37,
                goal_y=164.37,
                estimated_cost=100.0,
            ),
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
        )

        should_activate = _should_activate_local_mpc(
            agent,
            execution_state=execution_state,
            tracking_target=Waypoint(x=889.37, y=164.37),
            obstacle_layout=None,
            neighboring_agents=(neighboring_agent,),
            wreck_zones=(),
            grid_width=1000.0,
            grid_height=1000.0,
        )

        self.assertTrue(should_activate)

    def test_evaluate_usv_progress_triggers_recovery_for_low_progress_loop(self) -> None:
        task = TaskRecord(
            task_id="hotspot-confirmation-low-progress",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=100.0,
            target_y=0.0,
            target_row=1,
            target_col=1,
            created_step=0,
            assigned_agent_id="USV-2",
        )
        execution_state = AgentExecutionState(
            agent_id="USV-2",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id=task.task_id,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-2",
            patrol_waypoint_index=0,
        )
        progress_state = AgentProgressState(
            agent_id="USV-2",
            task_final_approach_task_id=task.task_id,
            task_final_approach_candidate_index=0,
        )

        should_enter_recovery = False
        for step in range(20):
            previous_agent = replace(
                next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2"),
                x=step * 0.2,
                y=0.0,
            )
            updated_agent = replace(previous_agent, x=previous_agent.x + 0.2, speed_mps=0.2)
            evaluation = evaluate_usv_progress(
                previous_agent,
                updated_agent=updated_agent,
                execution_state=execution_state,
                progress_state=progress_state,
                active_task=task,
                target_x=100.0,
                target_y=0.0,
                path_cleared=False,
                step=step,
            )
            progress_state = evaluation.progress_state
            should_enter_recovery = evaluation.should_enter_recovery

        self.assertTrue(should_enter_recovery)
        self.assertTrue(progress_state.low_progress_loop_active)
        self.assertEqual(progress_state.task_final_approach_low_progress_count, 1)

    def test_evaluate_usv_progress_does_not_flag_initial_fixed_escort_follow(self) -> None:
        previous_agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=150.0,
            y=180.0,
        )
        updated_agent = replace(previous_agent, x=150.2, y=180.2, speed_mps=0.3)
        execution_state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        progress_state = AgentProgressState(agent_id="USV-1", stalled_steps=2)

        evaluation = evaluate_usv_progress(
            previous_agent,
            updated_agent=updated_agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=None,
            target_x=260.0,
            target_y=210.0,
            path_cleared=False,
            step=20,
        )

        self.assertFalse(evaluation.should_enter_recovery)
        self.assertEqual(evaluation.progress_state.stalled_steps, 0)
        self.assertIsNone(evaluation.progress_state.blocked_goal_signature)

    def test_summarize_replay_validation_detects_pseudo_progress_loop(self) -> None:
        step_snapshots = []
        for step in range(20):
            x = 950.0 + (0.5 if step % 2 == 0 else -0.5)
            y = 530.0 + (0.5 if step % 3 == 0 else -0.5)
            step_snapshots.append(
                {
                    "step": step,
                    "agent_states": (
                        {
                            "agent_id": "USV-2",
                            "x": x,
                            "y": y,
                            "speed_mps": 1.2,
                            "turn_rate_degps": 28.0,
                        },
                    ),
                    "execution_layer": {
                        "tracking_updates": (
                            {
                                "agent_id": "USV-2",
                                "execution_stage": "go_to_task",
                                "active_task_id": "hotspot-confirmation-22-34",
                                "task_final_approach_candidate_index": 0,
                                "task_final_approach_candidate_x": 900.5,
                                "task_final_approach_candidate_y": 562.5,
                                "stalled_steps": 0,
                            },
                        ),
                    },
                    "task_layer": {"task_assignments": (), "tasks": ()},
                    "validation_layer": {"violation_count": 0, "violations": ()},
                }
            )

        summary = summarize_replay_validation(step_snapshots)

        self.assertEqual(summary["pseudo_progress_loop_count"], 1)

    def test_summarize_replay_validation_allows_side_switch_after_recovery_without_side(self) -> None:
        step_snapshots = [
            {
                "step": 238,
                "agent_states": (),
                "execution_layer": {
                    "tracking_updates": (
                        {
                            "agent_id": "USV-1",
                            "execution_stage": "go_to_task",
                            "active_task_id": "hotspot-confirmation-21-26",
                            "task_approach_task_id": "hotspot-confirmation-21-26",
                            "task_approach_active_side": "right",
                        },
                    ),
                },
                "task_layer": {"task_assignments": (), "tasks": ()},
                "validation_layer": {"violation_count": 0, "violations": ()},
            },
            {
                "step": 239,
                "agent_states": (),
                "execution_layer": {
                    "tracking_updates": (
                        {
                            "agent_id": "USV-1",
                            "execution_stage": "recovery",
                            "active_task_id": "hotspot-confirmation-21-26",
                            "task_approach_task_id": "hotspot-confirmation-21-26",
                            "task_approach_active_side": None,
                        },
                    ),
                },
                "task_layer": {"task_assignments": (), "tasks": ()},
                "validation_layer": {"violation_count": 0, "violations": ()},
            },
            {
                "step": 240,
                "agent_states": (),
                "execution_layer": {
                    "tracking_updates": (
                        {
                            "agent_id": "USV-1",
                            "execution_stage": "recovery",
                            "active_task_id": "hotspot-confirmation-21-26",
                            "task_approach_task_id": "hotspot-confirmation-21-26",
                            "task_approach_active_side": None,
                        },
                    ),
                },
                "task_layer": {"task_assignments": (), "tasks": ()},
                "validation_layer": {"violation_count": 0, "violations": ()},
            },
            {
                "step": 241,
                "agent_states": (),
                "execution_layer": {
                    "tracking_updates": (
                        {
                            "agent_id": "USV-1",
                            "execution_stage": "go_to_task",
                            "active_task_id": "hotspot-confirmation-21-26",
                            "task_approach_task_id": "hotspot-confirmation-21-26",
                            "task_approach_active_side": "left",
                        },
                    ),
                },
                "task_layer": {"task_assignments": (), "tasks": ()},
                "validation_layer": {"violation_count": 0, "violations": ()},
            },
            {
                "step": 242,
                "agent_states": (),
                "execution_layer": {
                    "tracking_updates": (
                        {
                            "agent_id": "USV-1",
                            "execution_stage": "go_to_task",
                            "active_task_id": "hotspot-confirmation-21-26",
                            "task_approach_task_id": "hotspot-confirmation-21-26",
                            "task_approach_active_side": "right",
                        },
                    ),
                },
                "task_layer": {"task_assignments": (), "tasks": ()},
                "validation_layer": {"violation_count": 0, "violations": ()},
            },
        ]

        summary = summarize_replay_validation(step_snapshots)

        self.assertEqual(summary["task_approach_side_switch_without_recovery_count"], 1)
        self.assertEqual(
            summary["task_approach_side_switches_without_recovery"][0]["step"],
            242,
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
        self.assertEqual(
            updated_progress.return_blocked_goal_signature,
            "return:2:900.0:420.0",
        )
        self.assertEqual(updated_progress.return_blocked_goal_until_step, 108)
        self.assertEqual(updated_progress.return_replan_generation, 1)
        self.assertIsNone(updated_progress.pre_recovery_stage)

    def test_recovery_rebuilds_taskless_return_to_patrol_without_accumulating_attempts(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        patrol_routes["USV-1"] = (
            (180.0, 160.0),
            (300.0, 160.0),
            (300.0, 300.0),
        )
        usv = next(agent for agent in agents if agent.agent_id == "USV-1")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RECOVERY,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
            return_target_x=180.0,
            return_target_y=160.0,
            return_target_source="fallback_patrol_waypoint",
        )
        progress_states["USV-1"] = AgentProgressState(
            agent_id="USV-1",
            recovery_attempts=4,
            recovery_step_index=5,
            pre_recovery_stage=ExecutionStage.RETURN_TO_PATROL,
            pre_recovery_task_id=None,
            blocked_goal_signature="return:0:180.0:160.0",
        )

        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(usv, False),
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
                step=20,
            )

        updated_state = updated_states["USV-1"]
        updated_progress = updated_progress_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.RETURN_TO_PATROL)
        self.assertNotEqual(
            (updated_state.return_target_x, updated_state.return_target_y),
            (180.0, 160.0),
        )
        self.assertEqual(
            updated_progress.return_blocked_goal_signature,
            "return:0:180.0:160.0",
        )
        self.assertEqual(updated_progress.return_blocked_goal_until_step, 116)
        self.assertEqual(updated_progress.recovery_attempts, 0)
        self.assertEqual(updated_progress.stalled_steps, 0)
        self.assertIsNone(updated_progress.pre_recovery_stage)

    def test_recovery_success_directly_rejoins_patrol_after_repeated_return_replans(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        usv = replace(
            next(agent for agent in agents if agent.agent_id == "USV-3"),
            x=936.0,
            y=770.0,
        )
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-3"] = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RECOVERY,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=2,
        )
        progress_states["USV-3"] = AgentProgressState(
            agent_id="USV-3",
            pre_recovery_stage=ExecutionStage.RETURN_TO_PATROL,
            pre_recovery_task_id=None,
            recovery_step_index=2,
            blocked_goal_signature="return:2:912.5:762.5",
            return_replan_generation=3,
        )
        recovered_agent = replace(usv, x=934.5, y=769.5)

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
                step=20,
            )

        updated_state = updated_states["USV-3"]
        updated_progress = updated_progress_states["USV-3"]
        self.assertEqual(updated_state.stage, ExecutionStage.PATROL)
        self.assertEqual(updated_state.patrol_waypoint_index, 2)
        self.assertEqual(updated_progress.return_replan_generation, 3)
        self.assertIsNone(updated_progress.pre_recovery_stage)

    def test_recovery_exhausted_taskless_return_directly_rejoins_patrol_when_close(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        usv = replace(
            next(agent for agent in agents if agent.agent_id == "USV-1"),
            x=721.0,
            y=481.0,
        )
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.RECOVERY,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=3,
        )
        progress_states["USV-1"] = AgentProgressState(
            agent_id="USV-1",
            recovery_step_index=5,
            pre_recovery_stage=ExecutionStage.RETURN_TO_PATROL,
            pre_recovery_task_id=None,
            blocked_goal_signature="return:3:712.5:462.5",
            return_replan_generation=2,
        )

        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(usv, False),
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
                step=20,
            )

        updated_state = updated_states["USV-1"]
        updated_progress = updated_progress_states["USV-1"]
        self.assertEqual(updated_state.stage, ExecutionStage.PATROL)
        self.assertEqual(updated_state.patrol_waypoint_index, 3)
        self.assertIsNone(updated_progress.pre_recovery_stage)

    def test_recovery_switches_hotspot_to_opposite_approach_side_before_final_approach(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        usv = next(agent for agent in agents if agent.agent_id == "USV-3")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-3"] = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RECOVERY,
            active_task_id="hotspot-confirmation-10-26",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
        )
        progress_states["USV-3"] = AgentProgressState(
            agent_id="USV-3",
            pre_recovery_stage=ExecutionStage.GO_TO_TASK,
            pre_recovery_task_id="hotspot-confirmation-10-26",
            recovery_step_index=2,
            blocked_goal_signature="task:hotspot-confirmation-10-26",
            task_approach_task_id="hotspot-confirmation-10-26",
            task_approach_anchor_left_x=620.0,
            task_approach_anchor_left_y=220.0,
            task_approach_anchor_right_x=710.0,
            task_approach_anchor_right_y=310.0,
            task_approach_active_side="left",
            task_approach_anchor_status="enroute_anchor",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-10-26",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=662.5,
            target_y=262.5,
            target_row=10,
            target_col=26,
            created_step=85,
            assigned_agent_id="USV-3",
        )

        recovered_agent = replace(usv, x=usv.x + 6.0, y=usv.y + 3.0)
        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, True),
        ):
            updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=build_patrol_routes(),
                grid_map=_build_runtime_grid_map(),
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
                step=610,
            )

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-3")
        updated_progress = updated_progress_states["USV-3"]
        updated_state = updated_states["USV-3"]
        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_progress.task_approach_escalation_task_id, task.task_id)
        self.assertIn(updated_progress.task_approach_active_side, {"left", "right"})
        self.assertEqual(updated_progress.task_approach_anchor_status, "enroute_anchor")
        anchor = current_task_approach_anchor(updated_progress, task_id=task.task_id)
        self.assertIsNotNone(anchor)
        assert anchor is not None
        self.assertEqual(updated_agent.task.target_x, anchor[0])
        self.assertEqual(updated_agent.task.target_y, anchor[1])

    def test_hold_reset_retargets_hotspot_to_opposite_approach_anchor(self) -> None:
        agents = build_demo_agent_states()
        usv = next(agent for agent in agents if agent.agent_id == "USV-3")
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        execution_states["USV-3"] = AgentExecutionState(
            agent_id="USV-3",
            stage=ExecutionStage.RECOVERY,
            active_task_id="hotspot-confirmation-10-26",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-3",
            patrol_waypoint_index=0,
        )
        progress_states["USV-3"] = AgentProgressState(
            agent_id="USV-3",
            pre_recovery_stage=ExecutionStage.GO_TO_TASK,
            pre_recovery_task_id="hotspot-confirmation-10-26",
            recovery_step_index=2,
            blocked_goal_signature="task:hotspot-confirmation-10-26",
            task_approach_escalation_task_id="hotspot-confirmation-10-26",
            task_approach_task_id="hotspot-confirmation-10-26",
            task_approach_anchor_left_x=620.0,
            task_approach_anchor_left_y=220.0,
            task_approach_anchor_right_x=710.0,
            task_approach_anchor_right_y=310.0,
            task_approach_active_side="left",
            task_approach_anchor_status="final_approach",
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-10-26",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=662.5,
            target_y=262.5,
            target_row=10,
            target_col=26,
            created_step=85,
            assigned_agent_id="USV-3",
        )

        recovered_agent = replace(usv, x=usv.x + 6.0, y=usv.y + 3.0)
        with patch(
            "usv_uav_marine_coverage.execution.recovery_runtime._execute_recovery_motion",
            return_value=(recovered_agent, True),
        ), patch(
            "usv_uav_marine_coverage.execution.recovery_runtime.advance_task_final_approach_after_failure",
            return_value=(None, False, True),
        ):
            updated_agents, updated_states, updated_progress_states = advance_agents_one_step(
                agents=agents,
                execution_states=execution_states,
                progress_states=progress_states,
                task_records=(task,),
                patrol_routes=build_patrol_routes(),
                grid_map=_build_runtime_grid_map(),
                obstacle_layout=build_obstacle_layout(build_default_sea_map(), seed=20260314),
                dt_seconds=1.0,
                step=658,
            )

        updated_agent = next(agent for agent in updated_agents if agent.agent_id == "USV-3")
        updated_progress = updated_progress_states["USV-3"]
        updated_state = updated_states["USV-3"]
        self.assertEqual(updated_state.stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_progress.task_approach_active_side, "right")
        self.assertEqual(updated_progress.task_approach_anchor_status, "enroute_anchor")
        self.assertEqual(updated_progress.task_final_approach_backoff_task_id, task.task_id)
        self.assertGreater(updated_progress.task_final_approach_backoff_until_step, 658)
        self.assertEqual(updated_agent.task.target_x, 710.0)
        self.assertEqual(updated_agent.task.target_y, 310.0)

    def test_validate_step_snapshot_flags_baseline_macro_approach_mode(self) -> None:
        step_snapshot = {
            "step": 27,
            "agent_states": (),
            "execution_layer": {
                "tracking_updates": (
                    {
                        "agent_id": "USV-2",
                        "execution_stage": "go_to_task",
                        "active_task_id": "baseline-service-21-1",
                        "task_approach_task_id": "baseline-service-21-1",
                        "task_approach_active_side": "left",
                    },
                ),
            },
            "task_layer": {
                "task_assignments": (),
                "tasks": (
                    {
                        "task_id": "baseline-service-21-1",
                        "task_type": "baseline_service",
                        "status": "assigned",
                        "assigned_agent_id": "USV-2",
                        "support_agent_id": None,
                        "claim_status": "claimed",
                    },
                ),
            },
        }

        violations = validate_step_snapshot(step_snapshot)

        self.assertTrue(
            any(
                violation["code"] == "baseline_task_should_not_use_macro_approach"
                for violation in violations
            )
        )

    def test_advance_task_final_approach_after_failure_prefers_hold_reset_before_release(
        self,
    ) -> None:
        grid_map = _build_runtime_grid_map()
        task = TaskRecord(
            task_id="hotspot-confirmation-hold-reset",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
            target_x=487.5,
            target_y=237.5,
            target_row=9,
            target_col=19,
            created_step=100,
            assigned_agent_id="USV-1",
        )
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1"),
            x=509.633,
            y=333.288,
            heading_deg=-79.611,
        )
        frozen_candidates = (
            (500.0, 250.0),
            (475.0, 250.0),
        )
        progress_state = AgentProgressState(
            agent_id="USV-1",
            task_final_approach_task_id=task.task_id,
            task_final_approach_frozen_candidates=frozen_candidates,
            task_final_approach_candidate_index=1,
            task_final_approach_candidate_x=frozen_candidates[1][0],
            task_final_approach_candidate_y=frozen_candidates[1][1],
            task_final_approach_failed_candidate_indexes=(0,),
            task_final_approach_attempted_candidate_indexes=(0, 1),
            task_final_approach_attempt_count=1,
        )

        selection, exhausted, hold_reset = advance_task_final_approach_after_failure(
            agent,
            task=task,
            grid_map=grid_map,
            progress_state=progress_state,
        )

        self.assertFalse(exhausted)
        self.assertTrue(hold_reset)
        assert selection is not None
        self.assertEqual(selection.status, "hold_reset")
        self.assertEqual(selection.failed_candidate_indexes, (0, 1))

    def test_serialize_task_record_surfaces_release_reason_after_owner_clears(self) -> None:
        info_map = build_information_map(_build_runtime_grid_map())
        task = TaskRecord(
            task_id="hotspot-confirmation-release",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskStatus.REQUEUED,
            priority=10,
            target_x=662.5,
            target_y=562.5,
            target_row=22,
            target_col=26,
            created_step=85,
            assigned_agent_id=None,
            retry_after_step=838,
        )
        progress_states = {
            "USV-3": AgentProgressState(
                agent_id="USV-3",
                released_task_id=task.task_id,
                released_task_created_step=task.created_step,
                released_task_step=658,
                released_task_retry_until_step=838,
                released_task_reason="task_final_approach_exhausted",
            )
        }

        serialized = serialize_task_record(
            task,
            step=658,
            info_map=info_map,
            execution_states={},
            progress_states=progress_states,
        )

        self.assertEqual(serialized["release_reason"], "task_final_approach_exhausted")

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

    def test_finalize_task_resolutions_completes_opportunistically_confirmed_pending_hotspot(
        self,
    ) -> None:
        agents = build_demo_agent_states()
        patrol_routes = build_patrol_routes()
        execution_states = build_initial_execution_states(agents)
        progress_states = build_initial_progress_states(agents)
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        target_row = 5
        target_col = 5
        cell = grid_map.cell_at(target_row, target_col)
        state = info_map.state_at(target_row, target_col)
        state.ground_truth_hotspot = True
        state.ground_truth_hotspot_id = 9
        state.known_hotspot_state = HotspotKnowledgeState.UAV_CHECKED

        usv = replace(
            next(agent for agent in agents if agent.agent_id == "USV-1"),
            x=cell.center_x,
            y=cell.center_y,
        )
        agents = tuple(usv if agent.agent_id == "USV-1" else agent for agent in agents)
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="baseline-service-4-4",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )
        task = TaskRecord(
            task_id="hotspot-confirmation-5-5",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.PENDING,
            priority=10,
            target_x=cell.center_x,
            target_y=cell.center_y,
            target_row=target_row,
            target_col=target_col,
            created_step=2,
        )

        for current_step in range(1, 6):
            apply_usv_confirmation(
                info_map,
                usv,
                ((target_row, target_col),),
                step=current_step,
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
        self.assertEqual(resolved_tasks[0].assigned_agent_id, None)
        self.assertEqual(updated_execution_states["USV-1"].stage, ExecutionStage.GO_TO_TASK)
        self.assertEqual(updated_progress_states["USV-1"].agent_id, "USV-1")
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
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.ON_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
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
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.ON_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
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

    def test_finalize_task_resolutions_keeps_uav_resupply_active_until_support_on_task(
        self,
    ) -> None:
        agents = tuple(
            replace(agent, energy_level=agent.energy_capacity * 0.95)
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
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.YIELD,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
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
            status=TaskStatus.ASSIGNED,
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

        self.assertEqual(resolved_tasks[0].status, TaskStatus.ASSIGNED)
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

    def test_sync_task_statuses_keeps_supportless_uav_resupply_pending(self) -> None:
        execution_states = {
            "UAV-2": AgentExecutionState(
                agent_id="UAV-2",
                stage=ExecutionStage.PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-2",
                patrol_waypoint_index=0,
            )
        }
        progress_states = {"UAV-2": AgentProgressState(agent_id="UAV-2")}
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-2",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=513.562,
                target_y=677.26,
                target_row=None,
                target_col=None,
                created_step=88,
                assigned_agent_id="UAV-2",
                support_agent_id=None,
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.PENDING)
        self.assertEqual(synced_tasks[0].assigned_agent_id, "UAV-2")
        self.assertIsNone(synced_tasks[0].support_agent_id)

    def test_sync_task_statuses_requeues_assigned_task_when_owner_no_longer_holds_it(self) -> None:
        execution_states = {
            "USV-2": AgentExecutionState(
                agent_id="USV-2",
                stage=ExecutionStage.RETURN_TO_PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-2",
                patrol_waypoint_index=0,
            )
        }
        progress_states = {"USV-2": AgentProgressState(agent_id="USV-2")}
        task_records = (
            TaskRecord(
                task_id="hotspot-confirmation-18-20",
                task_type=TaskType.HOTSPOT_CONFIRMATION,
                source=TaskSource.UAV_SUSPECTED,
                status=TaskStatus.ASSIGNED,
                priority=10,
                target_x=462.5,
                target_y=512.5,
                target_row=18,
                target_col=20,
                created_step=401,
                assigned_agent_id="USV-2",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(synced_tasks[0].assigned_agent_id)

    def test_serialize_task_record_keeps_supportless_uav_resupply_out_of_claimed_state(
        self,
    ) -> None:
        info_map = build_information_map(_build_runtime_grid_map())
        execution_states = {
            "UAV-1": AgentExecutionState(
                agent_id="UAV-1",
                stage=ExecutionStage.GO_TO_RENDEZVOUS,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-1",
                patrol_waypoint_index=0,
            )
        }
        progress_states = {
            "UAV-1": AgentProgressState(
                agent_id="UAV-1",
                claimed_task_id="uav-resupply-UAV-1",
            )
        }
        pending_task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.ASSIGNED,
            priority=20,
            target_x=400.0,
            target_y=500.0,
            target_row=None,
            target_col=None,
            created_step=88,
            assigned_agent_id="UAV-1",
            support_agent_id=None,
        )
        requeued_task = replace(pending_task, status=TaskStatus.REQUEUED)

        serialized_pending = serialize_task_record(
            pending_task,
            step=100,
            info_map=info_map,
            execution_states=execution_states,
            progress_states=progress_states,
        )
        serialized_requeued = serialize_task_record(
            requeued_task,
            step=100,
            info_map=info_map,
            execution_states=execution_states,
            progress_states=progress_states,
        )

        self.assertEqual(serialized_pending["claim_status"], "pending_claim")
        self.assertEqual(serialized_requeued["claim_status"], "unclaimed")

    def test_serialize_task_record_infers_uav_resupply_support_drop_reason(self) -> None:
        info_map = build_information_map(_build_runtime_grid_map())
        current_task = TaskRecord(
            task_id="uav-resupply-UAV-1",
            task_type=TaskType.UAV_RESUPPLY,
            source=TaskSource.SYSTEM_LOW_BATTERY,
            status=TaskStatus.REQUEUED,
            priority=20,
            target_x=400.0,
            target_y=500.0,
            target_row=None,
            target_col=None,
            created_step=88,
            assigned_agent_id="UAV-1",
            support_agent_id=None,
        )
        previous_task = replace(
            current_task,
            status=TaskStatus.ASSIGNED,
            support_agent_id="USV-1",
        )

        serialized = serialize_task_record(
            current_task,
            step=100,
            info_map=info_map,
            execution_states={},
            progress_states={},
            previous_task=previous_task,
        )

        self.assertEqual(serialized["release_reason"], "uav_resupply_support_dropped")

    def test_validate_step_snapshot_flags_supportless_uav_resupply_claim(self) -> None:
        step_snapshot = {
            "step": 1054,
            "agent_states": [],
            "execution_layer": {"tracking_updates": []},
            "task_layer": {
                "task_assignments": [],
                "tasks": [
                    {
                        "task_id": "uav-resupply-UAV-1",
                        "task_type": "uav_resupply",
                        "status": "requeued",
                        "assigned_agent_id": "UAV-1",
                        "support_agent_id": None,
                        "claim_status": "claimed",
                    }
                ],
            },
        }

        violations = validate_step_snapshot(step_snapshot)

        self.assertTrue(
            any(violation["code"] == "uav_resupply_supportless_claim" for violation in violations)
        )

    def test_sync_task_statuses_requeues_blocked_uav_support_assignment(self) -> None:
        execution_states = {
            "UAV-1": AgentExecutionState(
                agent_id="UAV-1",
                stage=ExecutionStage.GO_TO_RENDEZVOUS,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-1",
                patrol_waypoint_index=0,
            ),
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.RETURN_TO_PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
        }
        progress_states = {
            "UAV-1": AgentProgressState(agent_id="UAV-1"),
            "USV-1": AgentProgressState(
                agent_id="USV-1",
                released_task_id="uav-resupply-UAV-1",
                released_task_created_step=88,
                released_task_retry_until_step=24,
                released_task_reason="task_plan_blocked",
            ),
        }
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=400.0,
                target_y=500.0,
                target_row=None,
                target_col=None,
                created_step=88,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.REQUEUED)
        self.assertEqual(synced_tasks[0].assigned_agent_id, "UAV-1")
        self.assertIsNone(synced_tasks[0].support_agent_id)
        self.assertEqual(synced_tasks[0].retry_after_step, 24)
        self.assertEqual(
            synced_tasks[0].agent_retry_after_steps,
            (("USV-1", 24),),
        )

    def test_sync_task_statuses_consumes_release_only_on_matching_step(self) -> None:
        execution_states = {
            "UAV-1": AgentExecutionState(
                agent_id="UAV-1",
                stage=ExecutionStage.GO_TO_RENDEZVOUS,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-1",
                patrol_waypoint_index=0,
            ),
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.RETURN_TO_PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
        }
        progress_states = {
            "UAV-1": AgentProgressState(agent_id="UAV-1"),
            "USV-1": AgentProgressState(
                agent_id="USV-1",
                released_task_id="uav-resupply-UAV-1",
                released_task_created_step=88,
                released_task_step=24,
                released_task_retry_until_step=36,
                released_task_reason="task_plan_blocked",
            ),
        }
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=400.0,
                target_y=500.0,
                target_row=None,
                target_col=None,
                created_step=88,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
        )

        stale_sync = sync_task_statuses(
            task_records,
            execution_states,
            progress_states,
            step=25,
        )
        fresh_sync = sync_task_statuses(
            task_records,
            execution_states,
            progress_states,
            step=24,
        )

        self.assertEqual(stale_sync[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(stale_sync[0].support_agent_id)
        self.assertIsNone(stale_sync[0].retry_after_step)
        self.assertEqual(fresh_sync[0].status, TaskStatus.REQUEUED)
        self.assertIsNone(fresh_sync[0].support_agent_id)

    def test_sync_task_statuses_ignores_stale_uav_support_release_from_old_episode(self) -> None:
        execution_states = {
            "UAV-1": AgentExecutionState(
                agent_id="UAV-1",
                stage=ExecutionStage.GO_TO_RENDEZVOUS,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-1",
                patrol_waypoint_index=0,
            ),
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.GO_TO_TASK,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
        }
        progress_states = {
            "UAV-1": AgentProgressState(agent_id="UAV-1"),
            "USV-1": AgentProgressState(
                agent_id="USV-1",
                released_task_id="uav-resupply-UAV-1",
                released_task_created_step=88,
                released_task_retry_until_step=24,
                released_task_reason="task_plan_blocked",
            ),
        }
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=400.0,
                target_y=500.0,
                target_row=None,
                target_col=None,
                created_step=120,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.ASSIGNED)
        self.assertEqual(synced_tasks[0].support_agent_id, "USV-1")
        self.assertIsNone(synced_tasks[0].retry_after_step)

    def test_sync_task_statuses_keeps_uav_resupply_assigned_until_support_on_task(self) -> None:
        execution_states = {
            "UAV-1": AgentExecutionState(
                agent_id="UAV-1",
                stage=ExecutionStage.ON_RECHARGE,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-1",
                patrol_waypoint_index=0,
            ),
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.YIELD,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
        }
        progress_states = {
            "UAV-1": AgentProgressState(agent_id="UAV-1"),
            "USV-1": AgentProgressState(agent_id="USV-1"),
        }
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=400.0,
                target_y=500.0,
                target_row=None,
                target_col=None,
                created_step=88,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.ASSIGNED)

    def test_sync_task_statuses_requeues_uav_resupply_when_support_drops_assignment(self) -> None:
        execution_states = {
            "UAV-1": AgentExecutionState(
                agent_id="UAV-1",
                stage=ExecutionStage.GO_TO_RENDEZVOUS,
                active_task_id="uav-resupply-UAV-1",
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="UAV-1",
                patrol_waypoint_index=0,
            ),
            "USV-1": AgentExecutionState(
                agent_id="USV-1",
                stage=ExecutionStage.RETURN_TO_PATROL,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
                patrol_route_id="USV-1",
                patrol_waypoint_index=0,
            ),
        }
        progress_states = {
            "UAV-1": AgentProgressState(agent_id="UAV-1"),
            "USV-1": AgentProgressState(agent_id="USV-1"),
        }
        task_records = (
            TaskRecord(
                task_id="uav-resupply-UAV-1",
                task_type=TaskType.UAV_RESUPPLY,
                source=TaskSource.SYSTEM_LOW_BATTERY,
                status=TaskStatus.ASSIGNED,
                priority=20,
                target_x=410.841,
                target_y=321.491,
                target_row=None,
                target_col=None,
                created_step=363,
                assigned_agent_id="UAV-1",
                support_agent_id="USV-1",
            ),
        )

        synced_tasks = sync_task_statuses(task_records, execution_states, progress_states)

        self.assertEqual(synced_tasks[0].status, TaskStatus.REQUEUED)
        self.assertEqual(synced_tasks[0].assigned_agent_id, "UAV-1")
        self.assertIsNone(synced_tasks[0].support_agent_id)

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
        execution_states["USV-1"] = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.ON_TASK,
            active_task_id="uav-resupply-UAV-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
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
                released_task_created_step=1005,
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
