import unittest
from dataclasses import replace
from unittest.mock import patch

from usv_uav_marine_coverage.environment import (
    CircularFeature,
    ObstacleLayout,
    PolygonObstacle,
    build_default_sea_map,
    build_obstacle_layout,
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
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    build_information_map,
)
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.path_types import PathPlan, PathPlanStatus, Waypoint
from usv_uav_marine_coverage.simulation.simulation_agent_runtime import (
    _apply_usv_collision_guard,
    _evaluate_agent_progress,
    _should_refresh_patrol_plan,
    _should_refresh_return_plan,
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
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime.should_replan_return",
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
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime.should_replan_patrol",
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
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
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
            task_id="hotspot-confirmation-10-10",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_SUSPECTED,
            status=TaskStatus.ASSIGNED,
            priority=10,
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
                "usv_uav_marine_coverage.simulation.simulation_agent_runtime._apply_usv_collision_guard",
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
                "usv_uav_marine_coverage.simulation.simulation_agent_runtime._apply_usv_collision_guard",
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
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime._execute_recovery_motion",
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
            "usv_uav_marine_coverage.simulation.simulation_agent_runtime._execute_recovery_motion",
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
        self.assertIsNone(updated_progress.pre_recovery_stage)

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

        synced_tasks = sync_task_statuses(task_records, execution_states)
        statuses = {task.task_id: task.status for task in synced_tasks}
        assignments = {task.task_id: task.assigned_agent_id for task in synced_tasks}

        self.assertEqual(statuses["task-assigned"], TaskStatus.ASSIGNED)
        self.assertEqual(statuses["task-in-progress"], TaskStatus.IN_PROGRESS)
        self.assertEqual(statuses["task-requeued"], TaskStatus.REQUEUED)
        self.assertIsNone(assignments["task-requeued"])
        self.assertEqual(statuses["task-completed"], TaskStatus.COMPLETED)

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
