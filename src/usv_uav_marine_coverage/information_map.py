"""Dynamic information-map layer built on top of the static sea grid."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from random import Random

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.grid import GridMap


class InformationValidity(str, Enum):
    """Information freshness for one grid cell."""

    VALID = "valid"
    STALE_KNOWN = "stale_known"


class HotspotKnowledgeState(str, Enum):
    """System knowledge state for one hotspot cell."""

    NONE = "none"
    SUSPECTED = "suspected"
    CONFIRMED = "confirmed"
    FALSE_ALARM = "false_alarm"


class TaskStatus(str, Enum):
    """Basic task execution state stored on one information cell."""

    IDLE = "idle"
    ASSIGNED = "assigned"
    IN_SERVICE = "in_service"
    COMPLETED = "completed"


@dataclass(frozen=True)
class InformationMapConfig:
    """First-version information-map parameters."""

    information_timeout_steps: int = 30
    max_active_hotspots: int = 6
    uav_false_alarm_probability: float = 0.08
    usv_confirmation_steps: int = 3
    nearshore_hotspot_spawn_probability: float = 0.01
    offshore_hotspot_spawn_probability: float = 0.04


@dataclass
class GridInformationCell:
    """Dynamic information state attached to one static grid cell."""

    last_observed_step: int | None = None
    information_age: int = 0
    validity: InformationValidity = InformationValidity.STALE_KNOWN
    has_baseline_task: bool = False
    baseline_task_priority: int = 0
    baseline_last_served_step: int | None = None
    ground_truth_hotspot: bool = False
    ground_truth_hotspot_id: int | None = None
    ground_truth_hotspot_created_step: int | None = None
    known_hotspot_state: HotspotKnowledgeState = HotspotKnowledgeState.NONE
    known_hotspot_id: int | None = None
    suspected_by: str | None = None
    confirmed_by: str | None = None
    suspicion_step: int | None = None
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
        return sum(
            1
            for row in self.states
            for state in row
            if state.ground_truth_hotspot
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
            state_row.append(
                GridInformationCell(
                    information_age=actual_config.information_timeout_steps + 1,
                    validity=InformationValidity.STALE_KNOWN,
                    has_baseline_task=cell.has_baseline_point,
                    baseline_task_priority=1 if cell.has_baseline_point else 0,
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

    for row in info_map.states:
        for state in row:
            if state.last_observed_step is None:
                state.information_age = step + 1
                state.validity = InformationValidity.STALE_KNOWN
                continue

            state.information_age = max(0, step - state.last_observed_step)
            if state.information_age > info_map.config.information_timeout_steps:
                state.validity = InformationValidity.STALE_KNOWN
            else:
                state.validity = InformationValidity.VALID


def spawn_hotspots(
    info_map: InformationMap,
    step: int,
    rng: Random,
) -> tuple[tuple[int, int], ...]:
    """Spawn persistent ground-truth hotspots on eligible nearshore/offshore cells."""

    if step < 0:
        raise ValueError("Simulation step must be non-negative.")

    spawned_indices: list[tuple[int, int]] = []
    for row_index, row in enumerate(info_map.grid_map.cells):
        for col_index, cell in enumerate(row):
            if info_map.active_hotspot_count >= info_map.config.max_active_hotspots:
                return tuple(spawned_indices)
            if not _is_hotspot_eligible(cell):
                continue

            state = info_map.state_at(row_index, col_index)
            if state.ground_truth_hotspot:
                continue

            spawn_probability = _spawn_probability_for_zone(
                zone_name=cell.zone_name,
                config=info_map.config,
            )
            if rng.random() >= spawn_probability:
                continue

            hotspot_id = info_map.next_hotspot_id
            info_map.next_hotspot_id += 1
            state.ground_truth_hotspot = True
            state.ground_truth_hotspot_id = hotspot_id
            state.ground_truth_hotspot_created_step = step
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
        if state.known_hotspot_state == HotspotKnowledgeState.CONFIRMED:
            continue

        should_mark_suspected = state.ground_truth_hotspot
        if not should_mark_suspected:
            should_mark_suspected = rng.random() < info_map.config.uav_false_alarm_probability

        if not should_mark_suspected:
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
) -> tuple[tuple[int, int], ...]:
    """Advance USV confirmation progress and finalize suspected hotspots."""

    resolved_indices: list[tuple[int, int]] = []
    for row, col in observed_indices:
        state = info_map.state_at(row, col)
        if state.known_hotspot_state != HotspotKnowledgeState.SUSPECTED:
            continue

        state.confirmation_progress += 1
        if state.confirmation_progress < info_map.config.usv_confirmation_steps:
            continue

        state.confirmed_by = agent.agent_id
        if state.ground_truth_hotspot:
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
    return not cell.has_obstacle and not cell.has_risk_area and not cell.has_baseline_point


def _spawn_probability_for_zone(
    *,
    zone_name: str,
    config: InformationMapConfig,
) -> float:
    if zone_name == "Nearshore Zone":
        return config.nearshore_hotspot_spawn_probability
    if zone_name == "Offshore Zone":
        return config.offshore_hotspot_spawn_probability
    return 0.0
