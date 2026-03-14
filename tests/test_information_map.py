import unittest
from random import Random

from usv_uav_marine_coverage.agent_model import AgentState, AgentTaskState
from usv_uav_marine_coverage.environment import build_default_sea_map, build_obstacle_layout
from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    InformationMapConfig,
    InformationValidity,
    TaskStatus,
    advance_information_age,
    apply_uav_detection,
    apply_usv_confirmation,
    build_information_map,
    observe_cells,
    spawn_hotspots,
)


class InformationMapTestCase(unittest.TestCase):
    def setUp(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        self.grid_map = build_grid_map(sea_map, layout)

    def test_initial_information_map_inherits_baseline_tasks(self) -> None:
        info_map = build_information_map(self.grid_map)

        baseline_cells = [
            info_map.state_at(cell.row, cell.col)
            for cell in self.grid_map.flat_cells
            if cell.has_baseline_point
        ]

        self.assertEqual(len(baseline_cells), 2)
        self.assertTrue(all(cell.has_baseline_task for cell in baseline_cells))
        self.assertTrue(all(cell.baseline_task_priority == 1 for cell in baseline_cells))

    def test_observe_cells_refreshes_last_observed_age_and_validity(self) -> None:
        info_map = build_information_map(self.grid_map)
        observed_indices = ((5, 6), (7, 8))

        observe_cells(info_map, observed_indices, observer_id="UAV-1", step=4)

        for row, col in observed_indices:
            state = info_map.state_at(row, col)
            self.assertEqual(state.last_observed_step, 4)
            self.assertEqual(state.information_age, 0)
            self.assertEqual(state.validity, InformationValidity.VALID)

    def test_information_expires_to_stale_known_after_timeout(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(information_timeout_steps=2),
        )
        observe_cells(info_map, ((4, 4),), observer_id="UAV-1", step=0)

        advance_information_age(info_map, step=3)

        state = info_map.state_at(4, 4)
        self.assertEqual(state.information_age, 3)
        self.assertEqual(state.validity, InformationValidity.STALE_KNOWN)

    def test_reobserve_cell_restores_validity_from_stale_known(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(information_timeout_steps=1),
        )
        observe_cells(info_map, ((3, 3),), observer_id="UAV-1", step=0)
        advance_information_age(info_map, step=2)

        observe_cells(info_map, ((3, 3),), observer_id="USV-1", step=3)

        state = info_map.state_at(3, 3)
        self.assertEqual(state.last_observed_step, 3)
        self.assertEqual(state.information_age, 0)
        self.assertEqual(state.validity, InformationValidity.VALID)

    def test_spawn_hotspots_can_fill_nearshore_and_offshore_with_offshore_bias(self) -> None:
        config = InformationMapConfig(
            max_active_hotspots=2000,
            nearshore_hotspot_spawn_probability=0.1,
            offshore_hotspot_spawn_probability=0.9,
        )
        info_map = build_information_map(self.grid_map, config)

        spawn_hotspots(info_map, step=1, rng=Random(7))

        nearshore_count = 0
        offshore_count = 0
        for cell in self.grid_map.flat_cells:
            state = info_map.state_at(cell.row, cell.col)
            if not state.ground_truth_hotspot:
                continue
            if cell.zone_name == "Nearshore Zone":
                nearshore_count += 1
            if cell.zone_name == "Offshore Zone":
                offshore_count += 1

        self.assertGreater(nearshore_count, 0)
        self.assertGreater(offshore_count, 0)
        self.assertGreater(offshore_count, nearshore_count)

    def test_spawn_hotspots_respects_active_hotspot_cap(self) -> None:
        config = InformationMapConfig(
            max_active_hotspots=3,
            nearshore_hotspot_spawn_probability=1.0,
            offshore_hotspot_spawn_probability=1.0,
        )
        info_map = build_information_map(self.grid_map, config)

        spawned = spawn_hotspots(info_map, step=1, rng=Random(1))

        self.assertEqual(len(spawned), 3)
        self.assertEqual(info_map.active_hotspot_count, 3)

    def test_uav_marks_true_hotspot_as_suspected(self) -> None:
        info_map = build_information_map(self.grid_map)
        hotspot_cell = next(
            cell
            for cell in self.grid_map.flat_cells
            if cell.zone_name == "Offshore Zone"
            and not cell.has_obstacle
            and not cell.has_risk_area
            and not cell.has_baseline_point
        )
        state = info_map.state_at(hotspot_cell.row, hotspot_cell.col)
        state.ground_truth_hotspot = True
        state.ground_truth_hotspot_id = 9
        state.ground_truth_hotspot_created_step = 3

        uav = AgentState(
            agent_id="UAV-1",
            kind="UAV",
            x=hotspot_cell.center_x,
            y=hotspot_cell.center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
        )

        detected = apply_uav_detection(
            info_map,
            uav,
            ((hotspot_cell.row, hotspot_cell.col),),
            step=5,
            rng=Random(3),
        )

        self.assertEqual(detected, ((hotspot_cell.row, hotspot_cell.col),))
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.SUSPECTED)
        self.assertEqual(state.known_hotspot_id, 9)
        self.assertEqual(state.suspected_by, "UAV-1")

    def test_uav_can_raise_false_alarm_on_non_hotspot_cell(self) -> None:
        config = InformationMapConfig(uav_false_alarm_probability=1.0)
        info_map = build_information_map(self.grid_map, config)
        plain_cell = next(
            cell
            for cell in self.grid_map.flat_cells
            if cell.zone_name == "Nearshore Zone"
            and not cell.has_obstacle
            and not cell.has_risk_area
            and not cell.has_baseline_point
        )
        uav = AgentState(
            agent_id="UAV-2",
            kind="UAV",
            x=plain_cell.center_x,
            y=plain_cell.center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
        )

        apply_uav_detection(
            info_map,
            uav,
            ((plain_cell.row, plain_cell.col),),
            step=2,
            rng=Random(9),
        )

        state = info_map.state_at(plain_cell.row, plain_cell.col)
        self.assertFalse(state.ground_truth_hotspot)
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.SUSPECTED)

    def test_usv_confirmation_requires_multiple_observations(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(usv_confirmation_steps=3),
        )
        cell = next(
            candidate for candidate in self.grid_map.flat_cells if not candidate.has_obstacle
        )
        state = info_map.state_at(cell.row, cell.col)
        state.ground_truth_hotspot = True
        state.ground_truth_hotspot_id = 5
        state.known_hotspot_state = HotspotKnowledgeState.SUSPECTED

        usv = AgentState(
            agent_id="USV-1",
            kind="USV",
            x=cell.center_x,
            y=cell.center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(),
        )

        resolved_1 = apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=4)
        resolved_2 = apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=5)

        self.assertEqual(resolved_1, ())
        self.assertEqual(resolved_2, ())
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.SUSPECTED)
        self.assertEqual(state.confirmation_progress, 2)

    def test_usv_confirms_real_hotspot_after_threshold(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(usv_confirmation_steps=2),
        )
        cell = next(
            candidate for candidate in self.grid_map.flat_cells if not candidate.has_obstacle
        )
        state = info_map.state_at(cell.row, cell.col)
        state.ground_truth_hotspot = True
        state.ground_truth_hotspot_id = 11
        state.known_hotspot_state = HotspotKnowledgeState.SUSPECTED

        usv = AgentState(
            agent_id="USV-2",
            kind="USV",
            x=cell.center_x,
            y=cell.center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(),
        )

        apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=1)
        resolved = apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=2)

        self.assertEqual(resolved, ((cell.row, cell.col),))
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.CONFIRMED)
        self.assertEqual(state.confirmed_by, "USV-2")
        self.assertEqual(state.task_status, TaskStatus.ASSIGNED)
        self.assertEqual(state.assigned_agent_id, "USV-2")

    def test_usv_marks_false_alarm_after_threshold(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(usv_confirmation_steps=2),
        )
        cell = next(
            candidate for candidate in self.grid_map.flat_cells if not candidate.has_obstacle
        )
        state = info_map.state_at(cell.row, cell.col)
        state.known_hotspot_state = HotspotKnowledgeState.SUSPECTED

        usv = AgentState(
            agent_id="USV-3",
            kind="USV",
            x=cell.center_x,
            y=cell.center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=70.0,
            coverage_radius=40.0,
            task=AgentTaskState(),
        )

        apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=2)
        resolved = apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=3)

        self.assertEqual(resolved, ((cell.row, cell.col),))
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.FALSE_ALARM)
        self.assertEqual(state.confirmed_by, "USV-3")
        self.assertEqual(state.task_status, TaskStatus.IDLE)
        self.assertIsNone(state.assigned_agent_id)
