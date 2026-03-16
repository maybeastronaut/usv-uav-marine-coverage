import unittest
from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import USV_COLLISION_CLEARANCE_M, AgentTaskState
from usv_uav_marine_coverage.environment import (
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.execution.execution_types import UavCoverageState
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import (
    InformationValidity,
    build_information_map,
)
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.astar_smoother_path_planner import (
    build_astar_smoother_path_plan,
)
from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.fixed_patrol_planner import build_fixed_patrol_plan
from usv_uav_marine_coverage.planning.hybrid_astar_path_planner import (
    build_hybrid_astar_path_plan,
)
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.uav_lawnmower_planner import (
    build_lawnmower_route,
    build_uav_lawnmower_plan,
)
from usv_uav_marine_coverage.planning.uav_multi_region_coverage_planner import (
    build_fixed_uav_search_regions,
    build_uav_multi_region_plan,
    build_uav_multi_region_route,
    rank_uav_search_regions,
    select_uav_multi_region_waypoint_index,
)
from usv_uav_marine_coverage.planning.uav_persistent_multi_region_coverage_planner import (
    advance_uav_region_waypoint,
    build_empty_uav_coverage_state,
    build_uav_persistent_multi_region_plan,
    build_uav_region_coverage_route,
    rank_persistent_uav_search_regions,
    select_persistent_uav_region,
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

    def test_multi_region_route_builds_four_fixed_offshore_aois(self) -> None:
        regions = build_fixed_uav_search_regions(
            min_x=485.0,
            max_x=940.0,
            min_y=120.0,
            max_y=880.0,
        )

        self.assertEqual(
            tuple(region.region_id for region in regions),
            ("upper_left", "upper_right", "lower_left", "lower_right"),
        )
        self.assertEqual(len(regions), 4)

    def test_multi_region_ranking_prefers_staler_region_before_nearer_region(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        regions = build_fixed_uav_search_regions(
            min_x=sea_map.offshore.x_start + 35.0,
            max_x=sea_map.offshore.x_end - 60.0,
            min_y=120.0,
            max_y=880.0,
        )

        for row in grid_map.cells:
            for cell in row:
                if cell.zone_name != "Offshore Zone" or cell.has_obstacle:
                    continue
                state = info_map.state_at(cell.row, cell.col)
                if cell.center_x <= (regions[0].max_x) and cell.center_y <= (regions[0].max_y):
                    state.information_age = 900
                    state.validity = InformationValidity.STALE_KNOWN
                else:
                    state.information_age = 0
                    state.validity = InformationValidity.VALID

        ranked = rank_uav_search_regions(
            agent=agent,
            info_map=info_map,
            regions=regions,
        )

        self.assertEqual(ranked[0].region_id, "upper_left")

    def test_multi_region_route_preserves_boustrophedon_segments(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")

        route = build_uav_multi_region_route(
            agent,
            info_map=None,
            min_x=485.0,
            max_x=940.0,
            min_y=120.0,
            max_y=880.0,
            lane_spacing=170.0,
        )

        self.assertGreater(len(route), 8)
        self.assertTrue(
            any(route[index][0] != route[index + 1][0] for index in range(len(route) - 1))
        )
        self.assertTrue(
            any(route[index][1] == route[index + 1][1] for index in range(len(route) - 1))
        )

    def test_uav_multi_region_planner_targets_current_search_waypoint(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        patrol_routes = build_patrol_routes(
            uav_search_planner="uav_multi_region_coverage_planner",
            agents=build_demo_agent_states(),
        )

        plan = build_uav_multi_region_plan(
            agent,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
            patrol_routes=patrol_routes,
        )

        self.assertEqual(plan.planner_name, "uav_multi_region_coverage_planner")
        self.assertEqual(len(plan.waypoints), 2)
        self.assertEqual((plan.goal_x, plan.goal_y), patrol_routes[agent.agent_id][0])

    def test_multi_region_waypoint_selection_skips_nearby_patrol_waypoint(self) -> None:
        agent = replace(
            next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1"),
            x=705.931,
            y=502.165,
        )
        patrol_route = (
            (711.0, 500.0),
            (711.0, 580.0),
            (711.0, 760.0),
        )

        waypoint_index = select_uav_multi_region_waypoint_index(
            agent,
            patrol_route=patrol_route,
            patrol_waypoint_index=0,
        )

        self.assertEqual(waypoint_index, 1)

    def test_persistent_multi_region_ranking_prefers_staler_then_older_region(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        regions = build_fixed_uav_search_regions(
            min_x=sea_map.offshore.x_start + 35.0,
            max_x=sea_map.offshore.x_end - 60.0,
            min_y=120.0,
            max_y=880.0,
        )

        upper_right = regions[1]
        lower_left = regions[2]
        for row in grid_map.cells:
            for cell in row:
                if cell.zone_name != "Offshore Zone" or cell.has_obstacle:
                    continue
                state = info_map.state_at(cell.row, cell.col)
                if (
                    upper_right.min_x <= cell.center_x <= upper_right.max_x
                    and upper_right.min_y <= cell.center_y <= upper_right.max_y
                ):
                    state.information_age = 850
                    state.validity = InformationValidity.STALE_KNOWN
                elif (
                    lower_left.min_x <= cell.center_x <= lower_left.max_x
                    and lower_left.min_y <= cell.center_y <= lower_left.max_y
                ):
                    state.information_age = 700
                    state.validity = InformationValidity.STALE_KNOWN
                else:
                    state.information_age = 0
                    state.validity = InformationValidity.VALID

        ranked = rank_persistent_uav_search_regions(
            agent=agent,
            info_map=info_map,
            regions=regions,
            coverage_state=build_empty_uav_coverage_state(agent.agent_id),
            step=40,
        )

        self.assertEqual(ranked[0].region_id, "upper_right")
        self.assertGreaterEqual(ranked[0].stale_ratio, ranked[1].stale_ratio)

    def test_persistent_multi_region_selection_keeps_committed_region(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        regions = build_fixed_uav_search_regions(
            min_x=sea_map.offshore.x_start + 35.0,
            max_x=sea_map.offshore.x_end - 60.0,
            min_y=120.0,
            max_y=880.0,
        )
        current_state = UavCoverageState(
            agent_id=agent.agent_id,
            current_region_id="upper_left",
            region_route=((600.0, 180.0), (880.0, 180.0), (880.0, 350.0)),
            region_waypoint_index=1,
            region_entry_step=10,
            committed_waypoints_remaining=2,
            last_replan_step=10,
            last_replan_reason="forced_region_replan",
            region_last_visit_steps=(("upper_left", 10),),
        )
        upper_right = regions[1]
        for row in grid_map.cells:
            for cell in row:
                if cell.zone_name != "Offshore Zone" or cell.has_obstacle:
                    continue
                state = info_map.state_at(cell.row, cell.col)
                if (
                    upper_right.min_x <= cell.center_x <= upper_right.max_x
                    and upper_right.min_y <= cell.center_y <= upper_right.max_y
                ):
                    state.information_age = 900
                    state.validity = InformationValidity.STALE_KNOWN
                else:
                    state.information_age = 0
                    state.validity = InformationValidity.VALID

        selected_state, selected_score = select_persistent_uav_region(
            agent=agent,
            info_map=info_map,
            regions=regions,
            coverage_state=current_state,
            step=20,
        )

        self.assertEqual(selected_state.current_region_id, "upper_left")
        self.assertEqual(selected_score.region_id, "upper_left")

    def test_persistent_multi_region_selection_replans_when_route_is_complete(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        regions = build_fixed_uav_search_regions(
            min_x=sea_map.offshore.x_start + 35.0,
            max_x=sea_map.offshore.x_end - 60.0,
            min_y=120.0,
            max_y=880.0,
        )
        current_state = UavCoverageState(
            agent_id=agent.agent_id,
            current_region_id="upper_left",
            region_route=((600.0, 180.0),),
            region_waypoint_index=1,
            region_entry_step=0,
            committed_waypoints_remaining=0,
            last_replan_step=0,
            last_replan_reason="forced_region_replan",
            region_last_visit_steps=(("upper_left", 0),),
        )

        selected_state, _ = select_persistent_uav_region(
            agent=agent,
            info_map=info_map,
            regions=regions,
            coverage_state=current_state,
            step=70,
        )

        self.assertNotEqual(selected_state.current_region_id, None)
        self.assertGreater(len(selected_state.region_route), 0)
        self.assertEqual(selected_state.region_waypoint_index, 0)

    def test_persistent_multi_region_selection_prefers_unoccupied_region(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        regions = build_fixed_uav_search_regions(
            min_x=sea_map.offshore.x_start + 35.0,
            max_x=sea_map.offshore.x_end - 60.0,
            min_y=120.0,
            max_y=880.0,
        )
        current_state = build_empty_uav_coverage_state(agent.agent_id)

        selected_state, selected_score = select_persistent_uav_region(
            agent=agent,
            info_map=info_map,
            regions=regions,
            coverage_state=current_state,
            step=0,
            force_replan=True,
            occupied_region_ids=frozenset({"upper_left"}),
        )

        self.assertNotEqual(selected_score.region_id, "upper_left")
        self.assertEqual(selected_state.current_region_id, selected_score.region_id)

    def test_persistent_multi_region_selection_enters_from_nearest_route_endpoint(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        info_map = build_information_map(grid_map)
        base_agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        regions = build_fixed_uav_search_regions(
            min_x=sea_map.offshore.x_start + 35.0,
            max_x=sea_map.offshore.x_end - 60.0,
            min_y=120.0,
            max_y=880.0,
        )
        target_region = next(region for region in regions if region.region_id == "lower_right")
        base_route = build_uav_region_coverage_route(target_region)
        agent = replace(base_agent, x=base_route[-1][0], y=base_route[-1][1] - 5.0)

        selected_state, selected_score = select_persistent_uav_region(
            agent=agent,
            info_map=info_map,
            regions=(target_region,),
            coverage_state=build_empty_uav_coverage_state(agent.agent_id),
            step=0,
            force_replan=True,
        )

        self.assertEqual(selected_score.region_id, "lower_right")
        self.assertEqual(selected_state.region_waypoint_index, 0)
        self.assertEqual(selected_state.region_route[0], base_route[-1])
        self.assertEqual(selected_state.region_route[-1], base_route[0])

    def test_persistent_multi_region_plan_matches_current_region_waypoint(self) -> None:
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "UAV-1")
        coverage_state = UavCoverageState(
            agent_id=agent.agent_id,
            current_region_id="lower_right",
            region_route=((820.0, 620.0), (880.0, 620.0)),
            region_waypoint_index=0,
            region_entry_step=0,
            committed_waypoints_remaining=2,
        )

        plan = build_uav_persistent_multi_region_plan(
            agent,
            coverage_state=coverage_state,
        )

        self.assertEqual(plan.planner_name, "uav_persistent_multi_region_coverage_planner")
        self.assertEqual((plan.goal_x, plan.goal_y), (820.0, 620.0))

    def test_advance_uav_region_waypoint_clears_completed_route(self) -> None:
        coverage_state = UavCoverageState(
            agent_id="UAV-1",
            current_region_id="upper_left",
            region_route=((600.0, 180.0),),
            region_waypoint_index=0,
            region_entry_step=0,
            committed_waypoints_remaining=1,
        )

        updated_state = advance_uav_region_waypoint(coverage_state)

        self.assertIsNone(updated_state.current_region_id)
        self.assertEqual(updated_state.region_route, ())
        self.assertEqual(updated_state.last_replan_reason, "region_route_complete")

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

    def test_hybrid_astar_path_planner_builds_smoothed_usv_route(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")

        astar_plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=587.5,
            goal_y=537.5,
            planner_name="astar_path_planner",
            task_id="compare-task",
        )
        hybrid_plan = build_hybrid_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=587.5,
            goal_y=537.5,
            planner_name="hybrid_astar_path_planner",
            task_id="compare-task",
        )

        self.assertEqual(hybrid_plan.planner_name, "hybrid_astar_path_planner")
        self.assertEqual(hybrid_plan.status, PathPlanStatus.PLANNED)
        self.assertGreater(len(hybrid_plan.waypoints), 2)
        self.assertLessEqual(len(hybrid_plan.waypoints), len(astar_plan.waypoints))
        self.assertGreater(hybrid_plan.estimated_cost, 0.0)

    def test_astar_smoother_path_planner_builds_smoothed_usv_route(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-1")

        astar_plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=587.5,
            goal_y=537.5,
            planner_name="astar_path_planner",
            task_id="compare-task",
        )
        smoother_plan = build_astar_smoother_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=587.5,
            goal_y=537.5,
            planner_name="astar_smoother_path_planner",
            task_id="compare-task",
        )

        self.assertEqual(smoother_plan.planner_name, "astar_smoother_path_planner")
        self.assertEqual(smoother_plan.status, PathPlanStatus.PLANNED)
        self.assertGreater(len(smoother_plan.waypoints), 2)
        self.assertLessEqual(len(smoother_plan.waypoints), len(astar_plan.waypoints))
        self.assertGreater(smoother_plan.estimated_cost, 0.0)

    def test_hybrid_astar_can_plan_out_from_tight_start_pose(self) -> None:
        sea_map = build_default_sea_map()
        obstacle_layout = build_obstacle_layout(sea_map, seed=20260327)
        grid_map = build_grid_map(sea_map, obstacle_layout)
        agent = next(agent for agent in build_demo_agent_states() if agent.agent_id == "USV-2")
        agent = replace(
            agent,
            x=304.793,
            y=289.764,
            heading_deg=-30.321,
            task=AgentTaskState(),
        )

        plan = build_hybrid_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=520.0,
            goal_y=420.0,
            planner_name="hybrid_astar_path_planner",
            task_id="tight-start-hybrid",
        )

        self.assertEqual(plan.status, PathPlanStatus.PLANNED)
        self.assertGreater(len(plan.waypoints), 2)


def _distance_point_to_cell(x: float, y: float, cell) -> float:
    dx = max(cell.x_min - x, 0.0, x - cell.x_max)
    dy = max(cell.y_min - y, 0.0, y - cell.y_max)
    return hypot(dx, dy)


if __name__ == "__main__":
    unittest.main()
