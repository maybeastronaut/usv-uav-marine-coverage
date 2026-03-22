import unittest
from dataclasses import replace

from usv_uav_marine_coverage.environment import (
    CircularFeature,
    ObstacleLayout,
    PolygonObstacle,
)
from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_on_task,
    transition_to_patrol,
    transition_to_recovery,
    transition_to_return_to_patrol,
    transition_to_task,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    ExecutionOutcome,
    ExecutionStage,
)
from usv_uav_marine_coverage.execution.path_follower import (
    follow_path_step,
    follow_path_step_with_local_mpc,
)
from usv_uav_marine_coverage.planning.path_types import PathPlan, PathPlanStatus, Waypoint
from usv_uav_marine_coverage.simulation.simulation_policy import build_demo_agent_states


class ExecutionTestCase(unittest.TestCase):
    def test_basic_state_machine_transitions_cover_four_core_stages(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=0,
        )

        state = transition_to_task(state, task_id="task-1")
        self.assertEqual(state.stage, ExecutionStage.GO_TO_TASK)
        state = transition_to_on_task(state)
        self.assertEqual(state.stage, ExecutionStage.ON_TASK)
        state = transition_to_return_to_patrol(
            state,
            return_target_x=100.0,
            return_target_y=120.0,
            patrol_waypoint_index=2,
        )
        self.assertEqual(state.stage, ExecutionStage.RETURN_TO_PATROL)
        state = transition_to_patrol(state)
        self.assertEqual(state.stage, ExecutionStage.PATROL)

    def test_basic_state_machine_can_enter_recovery_and_return_to_patrol(self) -> None:
        state = AgentExecutionState(
            agent_id="USV-1",
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-1",
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id="USV-1",
            patrol_waypoint_index=2,
        )

        state = transition_to_recovery(state)
        self.assertEqual(state.stage, ExecutionStage.RECOVERY)
        self.assertIsNone(state.active_plan)
        self.assertEqual(state.current_waypoint_index, 0)

    def test_path_follower_advances_waypoint_index_when_intermediate_waypoint_is_reached(
        self,
    ) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=PathPlan(
                plan_id="plan-1",
                planner_name="fixed_patrol_planner",
                agent_id=agent.agent_id,
                task_id=None,
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=agent.x + 5.0, y=agent.y),
                    Waypoint(x=agent.x + 40.0, y=agent.y),
                ),
                goal_x=agent.x + 40.0,
                goal_y=agent.y,
                estimated_cost=40.0,
            ),
            current_waypoint_index=0,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        advanced_agent, updated_state, outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        self.assertEqual(updated_state.current_waypoint_index, 2)
        self.assertGreaterEqual(advanced_agent.x, agent.x)

    def test_path_follower_reports_task_site_reached_at_final_waypoint(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-1",
            active_plan=PathPlan(
                plan_id="plan-2",
                planner_name="direct_line_planner",
                agent_id=agent.agent_id,
                task_id="task-1",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=agent.x + 5.0, y=agent.y),
                ),
                goal_x=agent.x + 5.0,
                goal_y=agent.y,
                estimated_cost=5.0,
            ),
            current_waypoint_index=0,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        _, updated_state, outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
        )

        self.assertEqual(outcome, ExecutionOutcome.TASK_SITE_REACHED)
        self.assertEqual(updated_state.current_waypoint_index, 2)

    def test_path_follower_keeps_patrol_advancing_after_start_waypoint_skip(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=PathPlan(
                plan_id="plan-3",
                planner_name="uav_lawnmower_planner",
                agent_id=agent.agent_id,
                task_id=None,
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=agent.x + 200.0, y=agent.y - 80.0),
                ),
                goal_x=agent.x + 200.0,
                goal_y=agent.y - 80.0,
                estimated_cost=215.0,
            ),
            current_waypoint_index=0,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        advanced_agent, updated_state, outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        self.assertEqual(updated_state.current_waypoint_index, 1)
        self.assertGreater(advanced_agent.x, agent.x)

    def test_path_follower_uses_segment_lookahead_for_usv(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        agent = replace(agent, x=140.0, y=180.0, speed_mps=4.0)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-2",
            active_plan=PathPlan(
                plan_id="plan-4",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id="task-2",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=110.0, y=180.0),
                    Waypoint(x=210.0, y=180.0),
                    Waypoint(x=310.0, y=180.0),
                ),
                goal_x=310.0,
                goal_y=180.0,
                estimated_cost=200.0,
            ),
            current_waypoint_index=1,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        advanced_agent, updated_state, outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        assert advanced_agent.task.target_x is not None
        self.assertGreater(advanced_agent.task.target_x, 210.0)
        self.assertLessEqual(advanced_agent.task.target_x, 310.0)
        self.assertEqual(updated_state.current_waypoint_index, 1)

    def test_path_follower_advances_when_agent_has_already_passed_waypoint(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        agent = replace(agent, x=282.24, y=297.673, heading_deg=-62.659, speed_mps=0.947)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task_id=None,
            active_plan=PathPlan(
                plan_id="plan-5",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id=None,
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=227.017, y=453.395),
                    Waypoint(x=237.5, y=437.5),
                    Waypoint(x=237.5, y=412.5),
                    Waypoint(x=237.5, y=387.5),
                    Waypoint(x=237.5, y=362.5),
                    Waypoint(x=237.5, y=337.5),
                    Waypoint(x=262.5, y=312.5),
                    Waypoint(x=287.5, y=287.5),
                ),
                goal_x=280.0,
                goal_y=280.0,
                estimated_cost=220.0,
            ),
            current_waypoint_index=5,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
            return_target_x=280.0,
            return_target_y=280.0,
        )

        _, updated_state, outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
        )

        self.assertEqual(outcome, ExecutionOutcome.PATROL_REJOINED)
        self.assertEqual(updated_state.current_waypoint_index, 8)

    def test_path_follower_offsets_usv_tracking_target_away_from_nearby_obstacle(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        agent = replace(agent, x=120.0, y=180.0, heading_deg=0.0, speed_mps=3.0)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-avoid",
            active_plan=PathPlan(
                plan_id="plan-avoid",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id="task-avoid",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=220.0, y=180.0),
                ),
                goal_x=220.0,
                goal_y=180.0,
                estimated_cost=100.0,
            ),
            current_waypoint_index=1,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )
        obstacle_layout = ObstacleLayout(
            seed=1,
            traversable_corridors=(),
            risk_zone_obstacles=(
                PolygonObstacle(
                    name="Test Obstacle",
                    zone_name="Middle Risk Zone",
                    points=((150.0, 160.0), (190.0, 160.0), (190.0, 200.0), (150.0, 200.0)),
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

        advanced_agent, _, outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
            obstacle_layout=obstacle_layout,
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        assert advanced_agent.task.target_y is not None
        self.assertNotEqual(advanced_agent.task.target_y, 180.0)

    def test_local_mpc_path_follower_slows_for_nearby_usv_conflict(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        neighbor = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        agent = replace(agent, x=120.0, y=220.0, heading_deg=0.0, speed_mps=3.0)
        neighbor = replace(neighbor, x=165.0, y=220.0, heading_deg=180.0, speed_mps=0.0)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-mpc-agent",
            active_plan=PathPlan(
                plan_id="plan-mpc-agent",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id="task-mpc-agent",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=240.0, y=220.0),
                ),
                goal_x=240.0,
                goal_y=220.0,
                estimated_cost=120.0,
            ),
            current_waypoint_index=1,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        advanced_agent, _, outcome = follow_path_step_with_local_mpc(
            agent,
            state,
            dt_seconds=1.0,
            neighboring_agents=(neighbor,),
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        self.assertLess(advanced_agent.speed_mps, agent.speed_mps)
        self.assertLess(advanced_agent.x, agent.x + agent.cruise_speed_mps)

    def test_local_mpc_path_follower_matches_default_when_no_local_conflict(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        agent = replace(agent, x=120.0, y=220.0, heading_deg=0.0, speed_mps=3.0)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-mpc-clear",
            active_plan=PathPlan(
                plan_id="plan-mpc-clear",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id="task-mpc-clear",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=220.0, y=220.0),
                ),
                goal_x=220.0,
                goal_y=220.0,
                estimated_cost=100.0,
            ),
            current_waypoint_index=1,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        default_agent, default_state, default_outcome = follow_path_step(
            agent,
            state,
            dt_seconds=1.0,
        )
        mpc_agent, mpc_state, mpc_outcome = follow_path_step_with_local_mpc(
            agent,
            state,
            dt_seconds=1.0,
            neighboring_agents=(),
        )

        self.assertEqual(mpc_outcome, default_outcome)
        self.assertEqual(mpc_state.current_waypoint_index, default_state.current_waypoint_index)
        self.assertAlmostEqual(mpc_agent.x, default_agent.x, places=6)
        self.assertAlmostEqual(mpc_agent.y, default_agent.y, places=6)

    def test_local_mpc_path_follower_treats_offshore_risk_area_as_slowdown_hazard(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        agent = replace(agent, x=520.0, y=420.0, heading_deg=0.0, speed_mps=3.0)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-mpc-risk",
            active_plan=PathPlan(
                plan_id="plan-mpc-risk",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id="task-mpc-risk",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=650.0, y=420.0),
                ),
                goal_x=650.0,
                goal_y=420.0,
                estimated_cost=130.0,
            ),
            current_waypoint_index=1,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )
        obstacle_layout = ObstacleLayout(
            seed=2,
            traversable_corridors=(),
            risk_zone_obstacles=(),
            offshore_features=(
                CircularFeature(
                    name="Dynamic Risk",
                    feature_type="risk_area",
                    x=580.0,
                    y=420.0,
                    radius=24.0,
                ),
            ),
            nearshore_monitor_points=(),
            offshore_hotspots=(),
        )

        advanced_agent, _, outcome = follow_path_step_with_local_mpc(
            agent,
            state,
            dt_seconds=1.0,
            obstacle_layout=obstacle_layout,
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        self.assertLess(advanced_agent.speed_mps, agent.speed_mps)
        self.assertLess(advanced_agent.x, agent.x + agent.cruise_speed_mps)

    def test_local_mpc_path_follower_avoids_hugging_map_edge(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-3")
        agent = replace(agent, x=996.0, y=906.0, heading_deg=-48.0, speed_mps=3.9)
        state = AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.GO_TO_TASK,
            active_task_id="task-mpc-edge",
            active_plan=PathPlan(
                plan_id="plan-mpc-edge",
                planner_name="astar_path_planner",
                agent_id=agent.agent_id,
                task_id="task-mpc-edge",
                status=PathPlanStatus.PLANNED,
                waypoints=(
                    Waypoint(x=agent.x, y=agent.y),
                    Waypoint(x=987.5, y=808.5),
                ),
                goal_x=987.5,
                goal_y=808.5,
                estimated_cost=98.0,
            ),
            current_waypoint_index=1,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )

        advanced_agent, _, outcome = follow_path_step_with_local_mpc(
            agent,
            state,
            dt_seconds=1.0,
            grid_width=1000.0,
            grid_height=1000.0,
        )

        self.assertEqual(outcome, ExecutionOutcome.ADVANCING)
        self.assertLess(advanced_agent.x, 1000.0)
        self.assertLess(advanced_agent.x, agent.x + 1.0)


if __name__ == "__main__":
    unittest.main()
