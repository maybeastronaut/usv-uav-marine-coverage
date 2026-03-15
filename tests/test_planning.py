import unittest
from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import USV_COLLISION_CLEARANCE_M, AgentTaskState
from usv_uav_marine_coverage.environment import (
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.fixed_patrol_planner import build_fixed_patrol_plan
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.uav_lawnmower_planner import (
    build_lawnmower_route,
    build_uav_lawnmower_plan,
)
from usv_uav_marine_coverage.planning.usv_patrol_planner import (
    build_default_usv_patrol_routes,
    find_local_patrol_segment_access,
    find_progressive_patrol_segment_access,
)
from usv_uav_marine_coverage.simulation.simulation_policy import (
    build_demo_agent_states,
    build_patrol_routes,
)


class PlanningTestCase(unittest.TestCase):
    def test_usv_patrol_planner_assigns_one_nearshore_and_two_offshore_patrol_zones(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        patrol_routes = build_default_usv_patrol_routes(sea_map)

        expected_zones = {
            "USV-1": "Nearshore Zone",
            "USV-2": "Offshore Zone",
            "USV-3": "Offshore Zone",
        }
        for agent_id, expected_zone in expected_zones.items():
            with self.subTest(agent_id=agent_id):
                for x, y in patrol_routes[agent_id]:
                    cell = grid_map.locate_cell(x, y)
                    self.assertEqual(cell.zone_name, expected_zone)
                    self.assertFalse(cell.has_obstacle)

    def test_usv_1_nearshore_patrol_route_uses_multi_lane_coverage_pattern(self) -> None:
        sea_map = build_default_sea_map()
        patrol_routes = build_default_usv_patrol_routes(sea_map)

        usv_1_route = patrol_routes["USV-1"]
        unique_xs = {round(x, 3) for x, _ in usv_1_route}
        unique_ys = {round(y, 3) for _, y in usv_1_route}

        self.assertGreater(len(usv_1_route), 4)
        self.assertEqual(unique_xs, {45.0, 210.0})
        self.assertGreater(len(unique_ys), 2)
        self.assertGreaterEqual(min(unique_xs), sea_map.nearshore.x_start + 45.0)
        self.assertLessEqual(max(unique_xs), sea_map.nearshore.x_end - 40.0)

    def test_progressive_patrol_access_prefers_forward_segment_for_dense_route(self) -> None:
        patrol_route = (
            (45.0, 140.0),
            (210.0, 140.0),
            (210.0, 230.0),
            (45.0, 230.0),
            (45.0, 320.0),
            (210.0, 320.0),
        )

        access = find_progressive_patrol_segment_access(
            agent_x=120.0,
            agent_y=245.0,
            patrol_route=patrol_route,
            preferred_end_index=4,
        )

        assert access is not None
        self.assertEqual(access.segment_end_index, 4)
        self.assertAlmostEqual(access.access_x, 45.0)
        self.assertAlmostEqual(access.access_y, 245.0)

    def test_build_patrol_routes_assembles_usv_and_uav_patrol_sets(self) -> None:
        patrol_routes = build_patrol_routes()

        self.assertIn("USV-1", patrol_routes)
        self.assertIn("USV-2", patrol_routes)
        self.assertIn("USV-3", patrol_routes)
        self.assertIn("UAV-1", patrol_routes)
        self.assertIn("UAV-2", patrol_routes)
        self.assertGreater(len(patrol_routes["UAV-1"]), 4)

    def test_find_local_patrol_segment_access_returns_segment_projection(self) -> None:
        patrol_route = (
            (120.0, 120.0),
            (320.0, 120.0),
            (320.0, 320.0),
            (120.0, 320.0),
        )

        access = find_local_patrol_segment_access(
            agent_x=245.0,
            agent_y=200.0,
            patrol_route=patrol_route,
            preferred_end_index=2,
        )

        assert access is not None
        self.assertEqual(access.segment_start_index, 1)
        self.assertEqual(access.segment_end_index, 2)
        self.assertAlmostEqual(access.access_x, 320.0)
        self.assertAlmostEqual(access.access_y, 200.0)

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

    def test_lawnmower_route_alternates_sweep_direction(self) -> None:
        route = build_lawnmower_route(
            min_x=480.0,
            max_x=920.0,
            min_y=120.0,
            max_y=460.0,
            lane_spacing=170.0,
        )

        self.assertEqual(route[0], (480.0, 120.0))
        self.assertEqual(route[1], (920.0, 120.0))
        self.assertEqual(route[2], (920.0, 290.0))
        self.assertEqual(route[3], (480.0, 290.0))
        self.assertEqual(route[-2], (480.0, 460.0))
        self.assertEqual(route[-1], (920.0, 460.0))

    def test_uav_lawnmower_planner_targets_current_search_waypoint(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        patrol_routes = build_patrol_routes()

        plan = build_uav_lawnmower_plan(
            agent,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
            patrol_routes=patrol_routes,
        )

        self.assertEqual(plan.planner_name, "uav_lawnmower_planner")
        self.assertEqual(len(plan.waypoints), 2)
        self.assertEqual((plan.goal_x, plan.goal_y), patrol_routes[agent.agent_id][0])
        self.assertGreater(len(patrol_routes[agent.agent_id]), 4)

    def test_astar_path_planner_builds_multi_waypoint_usv_route(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")

        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=587.5,
            goal_y=537.5,
            planner_name="astar_path_planner",
            task_id="hotspot-confirmation-21-23",
        )

        self.assertEqual(plan.planner_name, "astar_path_planner")
        self.assertGreater(len(plan.waypoints), 2)
        self.assertGreater(plan.estimated_cost, 0.0)
        self.assertTrue(
            all(
                waypoint.row is None
                or not grid_map.cell_at(waypoint.row, waypoint.col or 0).has_obstacle
                for waypoint in plan.waypoints[1:]
            )
        )

    def test_astar_path_planner_respects_start_heading_on_first_move(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")
        agent = replace(
            agent,
            x=162.5,
            y=212.5,
            heading_deg=0.0,
            task=AgentTaskState(),
        )

        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=162.5,
            goal_y=462.5,
            planner_name="astar_path_planner",
            task_id="heading-constrained-task",
        )

        self.assertEqual(plan.planner_name, "astar_path_planner")
        self.assertGreater(len(plan.waypoints), 2)
        first_motion_waypoint = plan.waypoints[1]
        self.assertGreater(
            first_motion_waypoint.x,
            agent.x,
            "The first motion primitive should honor the east-facing start heading.",
        )

    def test_astar_path_planner_recovers_from_locally_awkward_start_pose(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        agent = replace(
            agent,
            x=44.117,
            y=296.76,
            heading_deg=-125.549,
            task=AgentTaskState(),
        )

        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=190.0,
            goal_y=420.0,
            planner_name="astar_path_planner",
            task_id=None,
        )

        self.assertEqual(plan.status, PathPlanStatus.PLANNED)
        self.assertGreater(len(plan.waypoints), 2)

    def test_astar_path_planner_blocks_goal_that_is_clear_for_centroid_but_not_for_hull(
        self,
    ) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")

        inflated_goal_cell = next(
            cell
            for cell in grid_map.flat_cells
            if not cell.has_obstacle
            and any(
                obstacle_cell.has_obstacle
                and _distance_point_to_cell(cell.center_x, cell.center_y, obstacle_cell)
                <= USV_COLLISION_CLEARANCE_M
                for obstacle_cell in grid_map.flat_cells
            )
        )

        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=inflated_goal_cell.center_x,
            goal_y=inflated_goal_cell.center_y,
            planner_name="astar_path_planner",
            task_id="clearance-blocked-goal",
        )

        self.assertEqual(plan.status, PathPlanStatus.BLOCKED)


def _distance_point_to_cell(x: float, y: float, cell) -> float:
    dx = max(cell.x_min - x, 0.0, x - cell.x_max)
    dy = max(cell.y_min - y, 0.0, y - cell.y_max)
    return hypot(dx, dy)


if __name__ == "__main__":
    unittest.main()
