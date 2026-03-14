import unittest

from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_on_task,
    transition_to_patrol,
    transition_to_return_to_patrol,
    transition_to_task,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    ExecutionOutcome,
    ExecutionStage,
)
from usv_uav_marine_coverage.execution.path_follower import follow_path_step
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

        self.assertEqual(outcome, ExecutionOutcome.WAYPOINT_REACHED)
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


if __name__ == "__main__":
    unittest.main()
