"""Dynamic information-map layer built on top of the static sea grid."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from random import Random

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.grid import GridMap


class InformationValidity(StrEnum):
    """Information freshness for one grid cell."""

    VALID = "valid"
    STALE_KNOWN = "stale_known"


class HotspotKnowledgeState(StrEnum):
    """System knowledge state for one hotspot cell."""

    NONE = "none"
    SUSPECTED = "suspected"
    CONFIRMED = "confirmed"
    FALSE_ALARM = "false_alarm"


class TaskStatus(StrEnum):
    """Basic task execution state stored on one information cell."""

    IDLE = "idle"
    ASSIGNED = "assigned"
    IN_SERVICE = "in_service"
    COMPLETED = "completed"


@dataclass(frozen=True)
class InformationMapConfig:
    """First-version information-map parameters."""

    information_timeout_steps: int = 400
    nearshore_information_timeout_steps: int = 800
    max_active_baseline_tasks: int = 1
    max_active_hotspots: int = 12
    max_suspected_hotspots: int = 12
    nearshore_baseline_spawn_probability: float = 0.0002
    baseline_respawn_cooldown_steps: int = 240
    uav_false_alarm_probability: float = 0.08
    usv_true_hotspot_probability: float = 0.9
    usv_confirmation_steps: int = 5
    baseline_service_steps: int = 5
    nearshore_hotspot_spawn_probability: float = 0.0
    offshore_hotspot_spawn_probability: float = 0.04
    hotspot_clearance_cells: int = 1


@dataclass
class GridInformationCell:
    """Dynamic information state attached to one static grid cell."""

    last_observed_step: int | None = None
    information_age: int = 0
    validity: InformationValidity = InformationValidity.STALE_KNOWN
    has_baseline_task: bool = False
    baseline_task_priority: int = 0
    baseline_last_served_step: int | None = None
    baseline_service_progress: int = 0
    ground_truth_hotspot: bool = False
    ground_truth_hotspot_id: int | None = None
    ground_truth_hotspot_created_step: int | None = None
    known_hotspot_state: HotspotKnowledgeState = HotspotKnowledgeState.NONE
    known_hotspot_id: int | None = None
    suspected_by: str | None = None
    confirmed_by: str | None = None
    suspicion_step: int | None = None
    hotspot_resolution_step: int | None = None
    confirmation_progress: int = 0
    assigned_agent_id: str | None = None
    task_status: TaskStatus = TaskStatus.IDLE


@dataclass
class InformationMap:
    """Dynamic information-map layer for the current sea grid."""

    grid_map: GridMap
    config: InformationMapConfig
    states: list[list[GridInformationCell]]
    next_hotspot_id: int = 1

    def state_at(self, row: int, col: int) -> GridInformationCell:
        return self.states[row][col]

    @property
    def active_hotspot_count(self) -> int:
        return sum(1 for row in self.states for state in row if state.ground_truth_hotspot)

    @property
    def active_baseline_task_count(self) -> int:
        return sum(1 for row in self.states for state in row if state.has_baseline_task)

    @property
    def active_suspected_hotspot_count(self) -> int:
        return sum(
            1
            for row in self.states
            for state in row
            if state.known_hotspot_state == HotspotKnowledgeState.SUSPECTED
        )


def build_information_map(
    grid_map: GridMap,
    config: InformationMapConfig | None = None,
) -> InformationMap:
    """Create the first-version information map from one static grid map."""

    actual_config = config or InformationMapConfig()
    states: list[list[GridInformationCell]] = []
    for row in grid_map.cells:
        state_row: list[GridInformationCell] = []
        for cell in row:
            timeout_steps = _information_timeout_for_zone(
                actual_config,
                cell.zone_name,
            )
            state_row.append(
                GridInformationCell(
                    information_age=timeout_steps + 1,
                    validity=InformationValidity.STALE_KNOWN,
                )
            )
        states.append(state_row)
    return InformationMap(grid_map=grid_map, config=actual_config, states=states)


def observe_cells(
    info_map: InformationMap,
    observed_indices: tuple[tuple[int, int], ...],
    observer_id: str,
    step: int,
) -> None:
    """Refresh information validity for the cells observed during one step."""

    if step < 0:
        raise ValueError("Simulation step must be non-negative.")

    for row, col in observed_indices:
        state = info_map.state_at(row, col)
        state.last_observed_step = step
        state.information_age = 0
        state.validity = InformationValidity.VALID
        if state.known_hotspot_state == HotspotKnowledgeState.FALSE_ALARM:
            state.suspected_by = observer_id
        if state.known_hotspot_state == HotspotKnowledgeState.NONE:
            state.confirmation_progress = 0


def advance_information_age(info_map: InformationMap, step: int) -> None:
    """Advance information age for all cells and update their validity."""

    if step < 0:
        raise ValueError("Simulation step must be non-negative.")

    for row_index, row in enumerate(info_map.states):
        for col_index, state in enumerate(row):
            timeout_steps = _information_timeout_for_zone(
                info_map.config,
                info_map.grid_map.cell_at(row_index, col_index).zone_name,
            )
            if state.last_observed_step is None:
                state.information_age = step + 1
                state.validity = InformationValidity.STALE_KNOWN
                continue

            state.information_age = max(0, step - state.last_observed_step)
            if state.information_age > timeout_steps:
                state.validity = InformationValidity.STALE_KNOWN
            else:
                state.validity = InformationValidity.VALID


def _information_timeout_for_zone(config: InformationMapConfig, zone_name: str) -> int:
    if zone_name == "Nearshore Zone":
        return config.nearshore_information_timeout_steps
    return config.information_timeout_steps


def spawn_hotspots(
    info_map: InformationMap,
    step: int,
    rng: Random,
) -> tuple[tuple[int, int], ...]:
    """Seed up to the configured hotspot cap on random eligible cells."""

    if step < 0:
        raise ValueError("Simulation step must be non-negative.")

    remaining_slots = info_map.config.max_active_hotspots - info_map.active_hotspot_count
    if remaining_slots <= 0:
        return ()

    candidate_indices: list[tuple[int, int]] = []
    for row_index, row in enumerate(info_map.grid_map.cells):
        for col_index, cell in enumerate(row):
            if not _is_hotspot_eligible(cell):
                continue
            state = info_map.state_at(row_index, col_index)
            if state.ground_truth_hotspot or state.has_baseline_task:
                continue
            if not _is_hotspot_region_eligible(info_map, row_index, col_index):
                continue
            candidate_indices.append((row_index, col_index))

    rng.shuffle(candidate_indices)
    spawned_indices: list[tuple[int, int]] = []
    for row_index, col_index in candidate_indices[:remaining_slots]:
        state = info_map.state_at(row_index, col_index)
        hotspot_id = info_map.next_hotspot_id
        info_map.next_hotspot_id += 1
        state.ground_truth_hotspot = True
        state.ground_truth_hotspot_id = hotspot_id
        state.ground_truth_hotspot_created_step = step
        spawned_indices.append((row_index, col_index))
    return tuple(spawned_indices)


def spawn_baseline_tasks(
    info_map: InformationMap,
    step: int,
    rng: Random,
) -> tuple[tuple[int, int], ...]:
    """Spawn baseline service points dynamically in the nearshore zone."""

    if step < 0:
        raise ValueError("Simulation step must be non-negative.")

    spawned_indices: list[tuple[int, int]] = []
    for row_index, row in enumerate(info_map.grid_map.cells):
        for col_index, cell in enumerate(row):
            if info_map.active_baseline_task_count >= info_map.config.max_active_baseline_tasks:
                return tuple(spawned_indices)
            if not _is_baseline_eligible(cell):
                continue

            state = info_map.state_at(row_index, col_index)
            if state.has_baseline_task:
                continue
            if state.ground_truth_hotspot:
                continue
            if (
                state.baseline_last_served_step is not None
                and step - state.baseline_last_served_step
                <= info_map.config.baseline_respawn_cooldown_steps
            ):
                continue
            if rng.random() >= info_map.config.nearshore_baseline_spawn_probability:
                continue

            state.has_baseline_task = True
            state.baseline_task_priority = 1
            state.baseline_service_progress = 0
            state.assigned_agent_id = None
            state.task_status = TaskStatus.IDLE
            spawned_indices.append((row_index, col_index))
    return tuple(spawned_indices)


def apply_uav_detection(
    info_map: InformationMap,
    agent: AgentState,
    observed_indices: tuple[tuple[int, int], ...],
    step: int,
    rng: Random,
) -> tuple[tuple[int, int], ...]:
    """Mark observed cells as suspected based on true hotspots or UAV false alarms."""

    detected_indices: list[tuple[int, int]] = []
    for row, col in observed_indices:
        state = info_map.state_at(row, col)
        if state.known_hotspot_state in {
            HotspotKnowledgeState.CONFIRMED,
            HotspotKnowledgeState.FALSE_ALARM,
        }:
            continue
        if state.hotspot_resolution_step is not None:
            continue

        if not state.ground_truth_hotspot:
            continue
        if (
            state.known_hotspot_state != HotspotKnowledgeState.SUSPECTED
            and info_map.active_suspected_hotspot_count >= info_map.config.max_suspected_hotspots
        ):
            continue

        state.known_hotspot_state = HotspotKnowledgeState.SUSPECTED
        state.known_hotspot_id = state.ground_truth_hotspot_id
        state.suspected_by = agent.agent_id
        state.suspicion_step = step
        state.confirmation_progress = 0
        detected_indices.append((row, col))

    return tuple(detected_indices)


def apply_usv_confirmation(
    info_map: InformationMap,
    agent: AgentState,
    observed_indices: tuple[tuple[int, int], ...],
    step: int,
    rng: Random | None = None,
) -> tuple[tuple[int, int], ...]:
    """Advance USV confirmation progress and finalize suspected hotspots."""

    confirmation_rng = rng or Random(0)
    resolved_indices: list[tuple[int, int]] = []
    for row, col in observed_indices:
        state = info_map.state_at(row, col)
        if state.known_hotspot_state != HotspotKnowledgeState.SUSPECTED:
            continue

        state.confirmation_progress += 1
        if state.confirmation_progress < info_map.config.usv_confirmation_steps:
            continue

        state.confirmed_by = agent.agent_id
        is_true_hotspot = state.ground_truth_hotspot and (
            confirmation_rng.random() < info_map.config.usv_true_hotspot_probability
        )
        if is_true_hotspot:
            state.known_hotspot_state = HotspotKnowledgeState.CONFIRMED
            state.known_hotspot_id = state.ground_truth_hotspot_id
            state.task_status = TaskStatus.ASSIGNED
            state.assigned_agent_id = agent.agent_id
        else:
            state.known_hotspot_state = HotspotKnowledgeState.FALSE_ALARM
            state.task_status = TaskStatus.IDLE
            state.assigned_agent_id = None
        resolved_indices.append((row, col))

    return tuple(resolved_indices)


def _is_hotspot_eligible(cell) -> bool:
    return not cell.has_obstacle and not cell.has_risk_area


def _is_baseline_eligible(cell) -> bool:
    return cell.zone_name == "Nearshore Zone" and not cell.has_obstacle and not cell.has_risk_area


def _is_hotspot_region_eligible(
    info_map: InformationMap,
    row_index: int,
    col_index: int,
) -> bool:
    clearance = info_map.config.hotspot_clearance_cells
    row_min = max(0, row_index - clearance)
    row_max = min(info_map.grid_map.rows - 1, row_index + clearance)
    col_min = max(0, col_index - clearance)
    col_max = min(info_map.grid_map.cols - 1, col_index + clearance)

    for candidate_row in range(row_min, row_max + 1):
        for candidate_col in range(col_min, col_max + 1):
            cell = info_map.grid_map.cell_at(candidate_row, candidate_col)
            if cell.zone_name != "Offshore Zone":
                return False
            if cell.has_obstacle or cell.has_risk_area:
                return False
    return True
