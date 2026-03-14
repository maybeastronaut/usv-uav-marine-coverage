import unittest

from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.fixed_patrol_planner import build_fixed_patrol_plan
from usv_uav_marine_coverage.simulation.simulation_policy import (
    build_demo_agent_states,
    build_patrol_routes,
)


class PlanningTestCase(unittest.TestCase):
    def test_fixed_patrol_planner_targets_current_patrol_waypoint(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        patrol_routes = build_patrol_routes()

        plan = build_fixed_patrol_plan(
            agent,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
            patrol_routes=patrol_routes,
        )

        self.assertEqual(plan.planner_name, "fixed_patrol_planner")
        self.assertEqual(len(plan.waypoints), 2)
        self.assertEqual((plan.goal_x, plan.goal_y), patrol_routes[agent.agent_id][0])

    def test_direct_line_planner_returns_two_waypoint_path(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")

        plan = build_direct_line_plan(
            agent,
            goal_x=250.0,
            goal_y=260.0,
            planner_name="direct_line_planner",
            task_id="task-1",
        )

        self.assertEqual(plan.planner_name, "direct_line_planner")
        self.assertEqual(len(plan.waypoints), 2)
        self.assertGreater(plan.estimated_cost, 0.0)


if __name__ == "__main__":
    unittest.main()
