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
    spawn_baseline_tasks,
    spawn_hotspots,
)
from usv_uav_marine_coverage.tasking.hotspot_task_generator import (
    sync_hotspot_confirmation_tasks,
)
from usv_uav_marine_coverage.tasking.task_types import (
    TaskRecord,
    TaskSource,
    TaskStatus as TaskRecordStatus,
    TaskType,
)


class InformationMapTestCase(unittest.TestCase):
    def setUp(self) -> None:
        sea_map = build_default_sea_map()
        layout = build_obstacle_layout(sea_map, seed=20260314)
        self.grid_map = build_grid_map(sea_map, layout)

    def test_initial_information_map_starts_without_baseline_tasks(self) -> None:
        info_map = build_information_map(self.grid_map)

        baseline_cells = [
            info_map.state_at(cell.row, cell.col)
            for cell in self.grid_map.flat_cells
            if info_map.state_at(cell.row, cell.col).has_baseline_task
        ]

        self.assertEqual(len(baseline_cells), 0)

    def test_spawn_baseline_tasks_creates_nearshore_tasks_dynamically(self) -> None:
        config = InformationMapConfig(
            max_active_baseline_tasks=2,
            nearshore_baseline_spawn_probability=1.0,
        )
        info_map = build_information_map(self.grid_map, config)

        spawned = spawn_baseline_tasks(info_map, step=1, rng=Random(5))

        self.assertEqual(len(spawned), 2)
        self.assertEqual(info_map.active_baseline_task_count, 2)
        self.assertTrue(
            all(
                self.grid_map.cell_at(row, col).zone_name == "Nearshore Zone"
                for row, col in spawned
            )
        )

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
            InformationMapConfig(
                information_timeout_steps=2,
                nearshore_information_timeout_steps=2,
            ),
        )
        observe_cells(info_map, ((4, 4),), observer_id="UAV-1", step=0)

        advance_information_age(info_map, step=3)

        state = info_map.state_at(4, 4)
        self.assertEqual(state.information_age, 3)
        self.assertEqual(state.validity, InformationValidity.STALE_KNOWN)

    def test_reobserve_cell_restores_validity_from_stale_known(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(
                information_timeout_steps=1,
                nearshore_information_timeout_steps=1,
            ),
        )
        observe_cells(info_map, ((3, 3),), observer_id="UAV-1", step=0)
        advance_information_age(info_map, step=2)

        observe_cells(info_map, ((3, 3),), observer_id="USV-1", step=3)

        state = info_map.state_at(3, 3)
        self.assertEqual(state.last_observed_step, 3)
        self.assertEqual(state.information_age, 0)
        self.assertEqual(state.validity, InformationValidity.VALID)

    def test_nearshore_uses_longer_information_timeout_than_offshore(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(
                information_timeout_steps=400,
                nearshore_information_timeout_steps=800,
            ),
        )
        nearshore_cell = next(
            cell
            for cell in self.grid_map.flat_cells
            if cell.zone_name == "Nearshore Zone" and not cell.has_obstacle
        )
        offshore_cell = next(
            cell
            for cell in self.grid_map.flat_cells
            if cell.zone_name == "Offshore Zone" and not cell.has_obstacle
        )

        observe_cells(
            info_map,
            ((nearshore_cell.row, nearshore_cell.col), (offshore_cell.row, offshore_cell.col)),
            observer_id="UAV-1",
            step=0,
        )
        advance_information_age(info_map, step=500)

        self.assertEqual(
            info_map.state_at(nearshore_cell.row, nearshore_cell.col).validity,
            InformationValidity.VALID,
        )
        self.assertEqual(
            info_map.state_at(offshore_cell.row, offshore_cell.col).validity,
            InformationValidity.STALE_KNOWN,
        )

    def test_spawn_hotspots_seeds_exact_cap_on_eligible_cells(self) -> None:
        config = InformationMapConfig(
            max_active_hotspots=12,
        )
        info_map = build_information_map(self.grid_map, config)

        spawned = spawn_hotspots(info_map, step=0, rng=Random(7))

        self.assertEqual(len(spawned), 12)
        self.assertEqual(info_map.active_hotspot_count, 12)
        self.assertTrue(
            all(
                not self.grid_map.cell_at(row, col).has_obstacle
                and not self.grid_map.cell_at(row, col).has_risk_area
                for row, col in spawned
            )
        )
        self.assertTrue(
            all(
                all(
                    self.grid_map.cell_at(candidate_row, candidate_col).zone_name == "Offshore Zone"
                    and not self.grid_map.cell_at(candidate_row, candidate_col).has_obstacle
                    and not self.grid_map.cell_at(candidate_row, candidate_col).has_risk_area
                    for candidate_row in range(
                        max(0, row - 1),
                        min(self.grid_map.rows - 1, row + 1) + 1,
                    )
                    for candidate_col in range(
                        max(0, col - 1),
                        min(self.grid_map.cols - 1, col + 1) + 1,
                    )
                )
                for row, col in spawned
            )
        )

    def test_spawn_hotspots_respects_active_hotspot_cap(self) -> None:
        config = InformationMapConfig(
            max_active_hotspots=12,
        )
        info_map = build_information_map(self.grid_map, config)

        spawned = spawn_hotspots(info_map, step=0, rng=Random(1))
        spawned_again = spawn_hotspots(info_map, step=1, rng=Random(2))

        self.assertEqual(len(spawned), 12)
        self.assertEqual(info_map.active_hotspot_count, 12)
        self.assertEqual(spawned_again, ())

    def test_uav_detection_checks_all_observed_hotspots(self) -> None:
        info_map = build_information_map(self.grid_map, InformationMapConfig())
        candidate_cells = [
            cell
            for cell in self.grid_map.flat_cells
            if not cell.has_obstacle and not cell.has_risk_area
        ][:4]
        for index, cell in enumerate(candidate_cells, start=1):
            state = info_map.state_at(cell.row, cell.col)
            state.ground_truth_hotspot = True
            state.ground_truth_hotspot_id = index
        uav = AgentState(
            agent_id="UAV-3",
            kind="UAV",
            x=candidate_cells[0].center_x,
            y=candidate_cells[0].center_y,
            heading_deg=0.0,
            speed_mps=0.0,
            max_speed_mps=0.0,
            detection_radius=120.0,
            coverage_radius=80.0,
            task=AgentTaskState(),
        )
        observe_cells(
            info_map,
            tuple((cell.row, cell.col) for cell in candidate_cells),
            observer_id="UAV-3",
            step=2,
        )

        detected = apply_uav_detection(
            info_map,
            uav,
            tuple((cell.row, cell.col) for cell in candidate_cells),
            step=2,
            rng=Random(11),
        )

        self.assertEqual(len(detected), 4)
        self.assertEqual(info_map.active_uav_checked_hotspot_count, 4)

    def test_spawn_baseline_tasks_respects_recent_service_cooldown(self) -> None:
        config = InformationMapConfig(
            max_active_baseline_tasks=2,
            nearshore_baseline_spawn_probability=1.0,
            baseline_respawn_cooldown_steps=120,
        )
        info_map = build_information_map(self.grid_map, config)
        candidate = next(
            cell
            for cell in self.grid_map.flat_cells
            if cell.zone_name == "Nearshore Zone"
            and not cell.has_obstacle
            and not cell.has_risk_area
        )
        state = info_map.state_at(candidate.row, candidate.col)
        state.baseline_last_served_step = 50

        spawned = spawn_baseline_tasks(info_map, step=100, rng=Random(7))

        self.assertNotIn((candidate.row, candidate.col), spawned)
        self.assertFalse(state.has_baseline_task)

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
        observe_cells(
            info_map,
            ((hotspot_cell.row, hotspot_cell.col),),
            observer_id="UAV-1",
            step=5,
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

    def test_uav_does_not_expose_stale_hotspot_without_current_step_observation(self) -> None:
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
        state.ground_truth_hotspot_id = 12
        state.last_observed_step = 1
        state.information_age = 10
        state.validity = InformationValidity.STALE_KNOWN

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

        self.assertEqual(detected, ())
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.NONE)

    def test_uav_does_not_reexpose_hotspot_from_old_valid_information(self) -> None:
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
        state.ground_truth_hotspot_id = 13
        state.last_observed_step = 4
        state.information_age = 0
        state.validity = InformationValidity.VALID

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

        self.assertEqual(detected, ())
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.NONE)

    def test_suspected_hotspot_can_remain_after_region_becomes_stale(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(
                information_timeout_steps=1,
                nearshore_information_timeout_steps=1,
            ),
        )
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
        state.ground_truth_hotspot_id = 14

        observe_cells(
            info_map,
            ((hotspot_cell.row, hotspot_cell.col),),
            observer_id="UAV-1",
            step=2,
        )
        apply_uav_detection(
            info_map,
            AgentState(
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
            ),
            ((hotspot_cell.row, hotspot_cell.col),),
            step=2,
            rng=Random(4),
        )

        advance_information_age(info_map, step=4)

        self.assertEqual(state.validity, InformationValidity.STALE_KNOWN)
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.SUSPECTED)

    def test_hotspot_confirmation_task_only_comes_from_suspected_state(self) -> None:
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
        state.ground_truth_hotspot_id = 15

        tasks_before_detection = sync_hotspot_confirmation_tasks(
            info_map,
            step=0,
            existing_tasks=(),
        )

        observe_cells(
            info_map,
            ((hotspot_cell.row, hotspot_cell.col),),
            observer_id="UAV-1",
            step=1,
        )
        apply_uav_detection(
            info_map,
            AgentState(
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
            ),
            ((hotspot_cell.row, hotspot_cell.col),),
            step=1,
            rng=Random(4),
        )
        tasks_after_detection = sync_hotspot_confirmation_tasks(
            info_map,
            step=1,
            existing_tasks=(),
        )

        self.assertEqual(tasks_before_detection, ())
        self.assertEqual(len(tasks_after_detection), 1)
        self.assertEqual(tasks_after_detection[0].task_type, TaskType.HOTSPOT_CONFIRMATION)

    def test_hotspot_confirmation_reopens_when_new_uav_checked_hotspot_reuses_cell(self) -> None:
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
        state.ground_truth_hotspot_id = 17
        state.known_hotspot_state = HotspotKnowledgeState.UAV_CHECKED
        state.known_hotspot_id = 17
        state.uav_checked_by = "UAV-1"

        existing_completed_task = TaskRecord(
            task_id=f"hotspot-confirmation-{hotspot_cell.row}-{hotspot_cell.col}",
            task_type=TaskType.HOTSPOT_CONFIRMATION,
            source=TaskSource.UAV_INSPECTED,
            status=TaskRecordStatus.COMPLETED,
            priority=10,
            target_x=hotspot_cell.center_x,
            target_y=hotspot_cell.center_y,
            target_row=hotspot_cell.row,
            target_col=hotspot_cell.col,
            created_step=12,
            completed_step=40,
        )

        refreshed_tasks = sync_hotspot_confirmation_tasks(
            info_map,
            step=55,
            existing_tasks=(existing_completed_task,),
        )

        self.assertEqual(len(refreshed_tasks), 1)
        self.assertEqual(refreshed_tasks[0].task_id, existing_completed_task.task_id)
        self.assertEqual(refreshed_tasks[0].status, TaskRecordStatus.PENDING)
        self.assertEqual(refreshed_tasks[0].created_step, 55)
        self.assertIsNone(refreshed_tasks[0].completed_step)

    def test_uav_does_not_mark_non_hotspot_cell_as_suspected(self) -> None:
        config = InformationMapConfig()
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

        detected = apply_uav_detection(
            info_map,
            uav,
            ((plain_cell.row, plain_cell.col),),
            step=2,
            rng=Random(9),
        )

        state = info_map.state_at(plain_cell.row, plain_cell.col)
        self.assertEqual(detected, ())
        self.assertFalse(state.ground_truth_hotspot)
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.NONE)

    def test_usv_confirmation_requires_multiple_observations(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(usv_confirmation_steps=5),
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
        resolved_3 = apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=6)
        resolved_4 = apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=7)

        self.assertEqual(resolved_1, ())
        self.assertEqual(resolved_2, ())
        self.assertEqual(resolved_3, ())
        self.assertEqual(resolved_4, ())
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.SUSPECTED)
        self.assertEqual(state.confirmation_progress, 4)

    def test_usv_confirms_real_hotspot_after_threshold(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(usv_confirmation_steps=5),
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

        for current_step in range(1, 5):
            apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=current_step)
        resolved = apply_usv_confirmation(
            info_map,
            usv,
            ((cell.row, cell.col),),
            step=5,
            rng=Random(1),
        )

        self.assertEqual(resolved, ((cell.row, cell.col),))
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.CONFIRMED)
        self.assertEqual(state.confirmed_by, "USV-2")
        self.assertEqual(state.task_status, TaskStatus.ASSIGNED)
        self.assertEqual(state.assigned_agent_id, "USV-2")

    def test_usv_confirms_uav_checked_hotspot_after_threshold(self) -> None:
        info_map = build_information_map(
            self.grid_map,
            InformationMapConfig(usv_confirmation_steps=5),
        )
        cell = next(
            candidate for candidate in self.grid_map.flat_cells if not candidate.has_obstacle
        )
        state = info_map.state_at(cell.row, cell.col)
        state.ground_truth_hotspot = True
        state.known_hotspot_state = HotspotKnowledgeState.UAV_CHECKED

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

        for current_step in range(1, 5):
            apply_usv_confirmation(info_map, usv, ((cell.row, cell.col),), step=current_step)
        resolved = apply_usv_confirmation(
            info_map,
            usv,
            ((cell.row, cell.col),),
            step=5,
            rng=Random(2),
        )

        self.assertEqual(resolved, ((cell.row, cell.col),))
        self.assertEqual(state.known_hotspot_state, HotspotKnowledgeState.CONFIRMED)
        self.assertEqual(state.confirmed_by, "USV-3")
        self.assertEqual(state.task_status, TaskStatus.ASSIGNED)
        self.assertEqual(state.assigned_agent_id, "USV-3")
        self.assertTrue(state.ground_truth_hotspot)
