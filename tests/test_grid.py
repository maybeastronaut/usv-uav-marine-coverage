import unittest

from usv_uav_marine_coverage.agent_model import AgentState, AgentTaskState
from usv_uav_marine_coverage.environment import build_default_sea_map, build_obstacle_layout
from usv_uav_marine_coverage.grid import (
    DEFAULT_CELL_SIZE,
    CoverageState,
    apply_agent_coverage,
    build_grid_coverage_map,
    build_grid_map,
    locate_grid_index,
)


class GridTestCase(unittest.TestCase):
    def test_grid_map_matches_confirmed_resolution(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, layout)

        self.assertEqual(grid_map.cell_size, DEFAULT_CELL_SIZE)
        self.assertEqual(grid_map.rows, 40)
        self.assertEqual(grid_map.cols, 40)
        self.assertEqual(grid_map.total_cells, 1600)
        self.assertEqual(grid_map.cell_at(0, 0).center_x, 12.5)
        self.assertEqual(grid_map.cell_at(0, 0).center_y, 12.5)
        self.assertEqual(grid_map.cell_at(39, 39).center_x, 987.5)
        self.assertEqual(grid_map.cell_at(39, 39).center_y, 987.5)

    def test_grid_map_projects_environment_features(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, layout)

        nearshore_cells = [cell for cell in grid_map.flat_cells if cell.zone_name == sea_map.nearshore.name]
        risk_cells = [cell for cell in grid_map.flat_cells if cell.zone_name == sea_map.risk.name]
        offshore_cells = [cell for cell in grid_map.flat_cells if cell.zone_name == sea_map.offshore.name]

        self.assertEqual(len(nearshore_cells), 400)
        self.assertEqual(len(risk_cells), 320)
        self.assertEqual(len(offshore_cells), 880)
        self.assertGreater(sum(cell.has_obstacle for cell in grid_map.flat_cells), 0)
        self.assertGreater(sum(cell.has_risk_area for cell in grid_map.flat_cells), 0)
        self.assertEqual(sum(cell.has_baseline_point for cell in grid_map.flat_cells), 2)
        self.assertEqual(sum(cell.has_task_hotspot for cell in grid_map.flat_cells), 5)

        for target in layout.nearshore_monitor_points:
            cell = grid_map.locate_cell(target.x, target.y)
            self.assertTrue(cell.has_baseline_point)
            self.assertFalse(cell.has_obstacle)
            self.assertFalse(cell.has_risk_area)

        for hotspot in layout.offshore_hotspots:
            cell = grid_map.locate_cell(hotspot.x, hotspot.y)
            self.assertTrue(cell.has_task_hotspot)
            self.assertFalse(cell.has_obstacle)
            self.assertFalse(cell.has_risk_area)

    def test_locate_grid_index_maps_boundary_points(self) -> None:
        self.assertEqual(
            locate_grid_index(x=0.0, y=0.0, width=1000.0, height=1000.0, cell_size=25.0),
            (0, 0),
        )
        self.assertEqual(
            locate_grid_index(x=999.9, y=999.9, width=1000.0, height=1000.0, cell_size=25.0),
            (39, 39),
        )
        self.assertEqual(
            locate_grid_index(x=1000.0, y=1000.0, width=1000.0, height=1000.0, cell_size=25.0),
            (39, 39),
        )

    def test_apply_agent_coverage_updates_dynamic_grid_state(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, layout)
        coverage_map = build_grid_coverage_map(grid_map)
        agent = AgentState(
            agent_id="UAV-1",
            kind="UAV",
            x=610.0,
            y=250.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=18.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
        )

        covered_cells = apply_agent_coverage(coverage_map, agent, step=3)

        self.assertGreater(len(covered_cells), 0)
        sample_row, sample_col = covered_cells[0]
        sample_state = coverage_map.state_at(sample_row, sample_col)
        self.assertEqual(sample_state.coverage_count, 1)
        self.assertEqual(sample_state.covered_by_uav, 1)
        self.assertEqual(sample_state.covered_by_usv, 0)
        self.assertEqual(sample_state.last_covered_step, 3)
        self.assertEqual(sample_state.coverage_state, CoverageState.COVERED)
        self.assertGreater(coverage_map.covered_ratio(), 0.0)

    def test_apply_agent_coverage_accumulates_source_specific_counts(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, layout)
        coverage_map = build_grid_coverage_map(grid_map)
        usv = AgentState(
            agent_id="USV-1",
            kind="USV",
            x=160.0,
            y=240.0,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=8.0,
            detection_radius=70.0,
            coverage_radius=45.0,
            task=AgentTaskState(),
        )

        covered_cells = apply_agent_coverage(coverage_map, usv, step=5)
        row, col = covered_cells[0]
        apply_agent_coverage(coverage_map, usv, step=6)
        updated_state = coverage_map.state_at(row, col)

        self.assertEqual(updated_state.covered_by_usv, 2)
        self.assertEqual(updated_state.coverage_count, 2)
        self.assertEqual(updated_state.last_covered_step, 6)

    def test_apply_agent_coverage_skips_obstacle_cells(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        grid_map = build_grid_map(sea_map, layout)
        coverage_map = build_grid_coverage_map(grid_map)

        obstacle_cell = next(cell for cell in grid_map.flat_cells if cell.has_obstacle)
        agent = AgentState(
            agent_id="UAV-2",
            kind="UAV",
            x=obstacle_cell.center_x,
            y=obstacle_cell.center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=18.0,
            detection_radius=90.0,
            coverage_radius=30.0,
            task=AgentTaskState(),
        )

        apply_agent_coverage(coverage_map, agent, step=4)
        obstacle_state = coverage_map.state_at(obstacle_cell.row, obstacle_cell.col)

        self.assertEqual(obstacle_state.coverage_count, 0)
        self.assertEqual(obstacle_state.coverage_state, CoverageState.UNCOVERED)
