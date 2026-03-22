"""Unified USV task-final-approach helpers."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, cos, degrees, hypot, radians, sin

from usv_uav_marine_coverage.agent_model import AgentState, can_cover_point
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType

from .execution_types import AgentProgressState
from .local_mpc import LOCAL_MPC_EDGE_CLEARANCE_M

USV_TASK_EDGE_APPROACH_BUFFER_M = LOCAL_MPC_EDGE_CLEARANCE_M + 1.0
TASK_FINAL_APPROACH_ANGLES_DEG = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)
TASK_FINAL_APPROACH_FAILURE_BUDGET = 3
TASK_FINAL_APPROACH_RELEASE_COOLDOWN_STEPS = 180
TASK_FINAL_APPROACH_SEGMENT_SAMPLES = 5


@dataclass(frozen=True)
class TaskFinalApproachCandidate:
    """One execution-reachable candidate inside the task coverage footprint."""

    index: int
    x: float
    y: float


@dataclass(frozen=True)
class TaskFinalApproachPlan:
    """All candidate approach points for one task at the current step."""

    task_id: str
    candidates: tuple[TaskFinalApproachCandidate, ...]


@dataclass(frozen=True)
class TaskFinalApproachSelection:
    """The active final-approach candidate selected for the current step."""

    task_id: str
    candidate_index: int
    candidate_x: float
    candidate_y: float
    attempt_count: int
    status: str
    candidate_count: int


def build_task_final_approach(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
) -> TaskFinalApproachPlan:
    """Build one ordered set of candidate approach points for a task."""

    if agent.kind != "USV" or task.task_type not in {
        TaskType.BASELINE_SERVICE,
        TaskType.HOTSPOT_CONFIRMATION,
    }:
        return TaskFinalApproachPlan(
            task_id=task.task_id,
            candidates=(TaskFinalApproachCandidate(index=0, x=task.target_x, y=task.target_y),),
        )

    max_approach_offset = max(agent.coverage_radius - agent.arrival_tolerance_m, 0.0)
    if max_approach_offset <= 0.0:
        return TaskFinalApproachPlan(
            task_id=task.task_id,
            candidates=(TaskFinalApproachCandidate(index=0, x=task.target_x, y=task.target_y),),
        )

    radial_candidate = _build_radial_candidate(
        agent,
        task=task,
        grid_map=grid_map,
        max_approach_offset=max_approach_offset,
    )
    ring_candidates = _build_ring_candidates(
        task=task,
        grid_map=grid_map,
        max_approach_offset=max_approach_offset,
    )
    ranked_candidates = _rank_candidates(
        agent,
        candidates=[radial_candidate, *ring_candidates],
        grid_map=grid_map,
    )
    return TaskFinalApproachPlan(
        task_id=task.task_id,
        candidates=tuple(
            TaskFinalApproachCandidate(index=index, x=candidate_x, y=candidate_y)
            for index, (candidate_x, candidate_y) in enumerate(ranked_candidates)
        ),
    )


def select_task_final_approach_candidate(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
    progress_state: AgentProgressState,
) -> TaskFinalApproachSelection:
    """Select the active approach candidate for the current task step."""

    approach_plan = build_task_final_approach(agent, task=task, grid_map=grid_map)
    candidate_count = len(approach_plan.candidates)
    if candidate_count == 0:
        return TaskFinalApproachSelection(
            task_id=task.task_id,
            candidate_index=0,
            candidate_x=task.target_x,
            candidate_y=task.target_y,
            attempt_count=0,
            status="active",
            candidate_count=0,
        )

    if progress_state.task_final_approach_task_id == task.task_id:
        candidate_index = progress_state.task_final_approach_candidate_index
        attempt_count = progress_state.task_final_approach_attempt_count
        status = progress_state.task_final_approach_status or "active"
    else:
        candidate_index = 0
        attempt_count = 0
        status = "active"
    candidate_index = min(max(candidate_index, 0), candidate_count - 1)
    candidate = approach_plan.candidates[candidate_index]
    return TaskFinalApproachSelection(
        task_id=task.task_id,
        candidate_index=candidate.index,
        candidate_x=candidate.x,
        candidate_y=candidate.y,
        attempt_count=attempt_count,
        status=status,
        candidate_count=candidate_count,
    )


def apply_task_final_approach_selection(
    progress_state: AgentProgressState,
    *,
    selection: TaskFinalApproachSelection,
) -> AgentProgressState:
    """Persist one selected final-approach candidate into progress state."""

    return replace(
        progress_state,
        task_final_approach_task_id=selection.task_id,
        task_final_approach_candidate_index=selection.candidate_index,
        task_final_approach_candidate_x=selection.candidate_x,
        task_final_approach_candidate_y=selection.candidate_y,
        task_final_approach_attempt_count=selection.attempt_count,
        task_final_approach_status=selection.status,
    )


def advance_task_final_approach_after_failure(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
    progress_state: AgentProgressState,
) -> tuple[TaskFinalApproachSelection | None, bool]:
    """Rotate to the next candidate after one failed final-approach attempt."""

    approach_plan = build_task_final_approach(agent, task=task, grid_map=grid_map)
    candidate_count = len(approach_plan.candidates)
    if candidate_count == 0:
        return (None, True)
    next_attempt_count = progress_state.task_final_approach_attempt_count + 1
    current_index = (
        progress_state.task_final_approach_candidate_index
        if progress_state.task_final_approach_task_id == task.task_id
        else 0
    )
    next_index = current_index + 1
    if next_attempt_count >= TASK_FINAL_APPROACH_FAILURE_BUDGET or next_index >= candidate_count:
        last_index = max(min(current_index, candidate_count - 1), 0)
        last_candidate = approach_plan.candidates[last_index]
        return (
            TaskFinalApproachSelection(
                task_id=task.task_id,
                candidate_index=max(current_index, 0),
                candidate_x=last_candidate.x,
                candidate_y=last_candidate.y,
                attempt_count=next_attempt_count,
                status="exhausted",
                candidate_count=candidate_count,
            ),
            True,
        )
    candidate = approach_plan.candidates[next_index]
    return (
        TaskFinalApproachSelection(
            task_id=task.task_id,
            candidate_index=candidate.index,
            candidate_x=candidate.x,
            candidate_y=candidate.y,
            attempt_count=next_attempt_count,
            status="rotated_after_recovery",
            candidate_count=candidate_count,
        ),
        False,
    )


def task_final_approach_satisfied(agent: AgentState, *, task: TaskRecord) -> bool:
    """Return whether the agent is already inside the task executable neighborhood."""

    return (
        agent.kind == "USV"
        and task.task_type in {TaskType.BASELINE_SERVICE, TaskType.HOTSPOT_CONFIRMATION}
        and can_cover_point(agent, task.target_x, task.target_y)
    )


def resolve_usv_task_approach_target(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
    hotspot_reapproach_attempt: int = 0,
) -> tuple[float, float]:
    """Compatibility wrapper returning one selected task-final-approach target."""

    if hotspot_reapproach_attempt < 0:
        hotspot_reapproach_attempt = 0
    approach_plan = build_task_final_approach(agent, task=task, grid_map=grid_map)
    if not approach_plan.candidates:
        return (task.target_x, task.target_y)
    candidate_index = min(hotspot_reapproach_attempt, len(approach_plan.candidates) - 1)
    candidate = approach_plan.candidates[candidate_index]
    return (candidate.x, candidate.y)


def can_start_hotspot_confirmation(agent: AgentState, *, task: TaskRecord) -> bool:
    """Compatibility wrapper for hotspot confirmation checks."""

    return task_final_approach_satisfied(agent, task=task)


def task_final_approach_release_cooldown_steps(task: TaskRecord) -> int:
    """Return the default agent-task cooldown after exhausting final-approach retries."""

    if task.task_type == TaskType.HOTSPOT_CONFIRMATION:
        return TASK_FINAL_APPROACH_RELEASE_COOLDOWN_STEPS
    return TASK_FINAL_APPROACH_RELEASE_COOLDOWN_STEPS // 2


def _build_radial_candidate(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
    max_approach_offset: float,
) -> tuple[float, float]:
    desired_x = min(
        max(task.target_x, USV_TASK_EDGE_APPROACH_BUFFER_M),
        grid_map.width - USV_TASK_EDGE_APPROACH_BUFFER_M,
    )
    desired_y = min(
        max(task.target_y, USV_TASK_EDGE_APPROACH_BUFFER_M),
        grid_map.height - USV_TASK_EDGE_APPROACH_BUFFER_M,
    )
    target_distance = hypot(agent.x - task.target_x, agent.y - task.target_y)
    if target_distance <= max_approach_offset:
        return _constrain_within_task_coverage(
            task.target_x,
            task.target_y,
            candidate_x=desired_x,
            candidate_y=desired_y,
            max_approach_offset=max_approach_offset,
        )

    scale = max_approach_offset / target_distance
    stand_x = task.target_x + (agent.x - task.target_x) * scale
    stand_y = task.target_y + (agent.y - task.target_y) * scale
    return _constrain_within_task_coverage(
        task.target_x,
        task.target_y,
        candidate_x=stand_x,
        candidate_y=stand_y,
        max_approach_offset=max_approach_offset,
    )


def _build_ring_candidates(
    *,
    task: TaskRecord,
    grid_map: GridMap,
    max_approach_offset: float,
) -> tuple[tuple[float, float], ...]:
    candidates: list[tuple[float, float]] = []
    for angle_deg in TASK_FINAL_APPROACH_ANGLES_DEG:
        angle_rad = radians(angle_deg)
        raw_x = task.target_x + max_approach_offset * cos(angle_rad)
        raw_y = task.target_y + max_approach_offset * sin(angle_rad)
        desired_x = min(
            max(raw_x, USV_TASK_EDGE_APPROACH_BUFFER_M),
            grid_map.width - USV_TASK_EDGE_APPROACH_BUFFER_M,
        )
        desired_y = min(
            max(raw_y, USV_TASK_EDGE_APPROACH_BUFFER_M),
            grid_map.height - USV_TASK_EDGE_APPROACH_BUFFER_M,
        )
        candidates.append(
            _constrain_within_task_coverage(
                task.target_x,
                task.target_y,
                candidate_x=desired_x,
                candidate_y=desired_y,
                max_approach_offset=max_approach_offset,
            )
        )
    return tuple(candidates)


def _rank_candidates(
    agent: AgentState,
    *,
    candidates: list[tuple[float, float]],
    grid_map: GridMap,
) -> tuple[tuple[float, float], ...]:
    unique_candidates: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()
    for candidate_x, candidate_y in candidates:
        signature = (round(candidate_x, 3), round(candidate_y, 3))
        if signature in seen:
            continue
        seen.add(signature)
        unique_candidates.append((candidate_x, candidate_y))

    def _score(candidate: tuple[float, float]) -> tuple[float, float, float, float, float]:
        candidate_x, candidate_y = candidate
        heading_to_candidate = degrees(atan2(candidate_y - agent.y, candidate_x - agent.x))
        heading_delta = abs(((heading_to_candidate - agent.heading_deg + 180.0) % 360.0) - 180.0)
        travel_distance = hypot(candidate_x - agent.x, candidate_y - agent.y)
        edge_margin = min(
            candidate_x,
            candidate_y,
            grid_map.width - candidate_x,
            grid_map.height - candidate_y,
        )
        hazard_clearance = _hazard_clearance(candidate_x, candidate_y, grid_map)
        approach_clearance = _approach_clearance(agent, candidate_x, candidate_y, grid_map)
        return (
            -approach_clearance,
            -hazard_clearance,
            heading_delta,
            travel_distance,
            -edge_margin,
        )

    return tuple(sorted(unique_candidates, key=_score))


def _approach_clearance(
    agent: AgentState,
    candidate_x: float,
    candidate_y: float,
    grid_map: GridMap,
) -> float:
    best_clearance = float("inf")
    for index in range(1, TASK_FINAL_APPROACH_SEGMENT_SAMPLES + 1):
        ratio = index / TASK_FINAL_APPROACH_SEGMENT_SAMPLES
        sample_x = agent.x + (candidate_x - agent.x) * ratio
        sample_y = agent.y + (candidate_y - agent.y) * ratio
        best_clearance = min(best_clearance, _hazard_clearance(sample_x, sample_y, grid_map))
    return best_clearance


def _hazard_clearance(x: float, y: float, grid_map: GridMap) -> float:
    best_distance = float("inf")
    for cell in grid_map.flat_cells:
        if not (cell.has_obstacle or cell.has_risk_area):
            continue
        best_distance = min(best_distance, hypot(x - cell.center_x, y - cell.center_y))
    return best_distance


def _constrain_within_task_coverage(
    target_x: float,
    target_y: float,
    *,
    candidate_x: float,
    candidate_y: float,
    max_approach_offset: float,
) -> tuple[float, float]:
    offset_x = candidate_x - target_x
    offset_y = candidate_y - target_y
    offset_distance = hypot(offset_x, offset_y)
    if offset_distance <= max_approach_offset:
        return (candidate_x, candidate_y)
    if offset_distance <= 0.0:
        return (target_x, target_y)
    scale = max_approach_offset / offset_distance
    return (
        target_x + offset_x * scale,
        target_y + offset_y * scale,
    )
