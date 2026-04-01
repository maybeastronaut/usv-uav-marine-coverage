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
TASK_FINAL_APPROACH_RELEASE_COOLDOWN_STEPS = 180
TASK_FINAL_APPROACH_SEGMENT_SAMPLES = 5
TASK_FINAL_APPROACH_BACKOFF_STEPS = 48
TASK_FINAL_APPROACH_MAX_HOLD_RESETS = 2
TASK_APPROACH_MIN_ANCHOR_DISTANCE_M = 80.0
TASK_APPROACH_MIN_SIDE_OFFSET_M = 45.0
TASK_APPROACH_ANCHOR_REACHED_M = 18.0
TASK_APPROACH_SIDE_COMMIT_STEPS = 24
TASK_APPROACH_LEFT = "left"
TASK_APPROACH_RIGHT = "right"
_TASK_APPROACH_SIDES = (TASK_APPROACH_LEFT, TASK_APPROACH_RIGHT)


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
class TaskApproachAnchors:
    """One frozen pair of macro approach anchors for a task episode."""

    task_id: str
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    preferred_side: str


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
    frozen_candidates: tuple[tuple[float, float], ...] = ()
    failed_candidate_indexes: tuple[int, ...] = ()
    attempted_candidate_indexes: tuple[int, ...] = ()


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

    frozen_candidates = _resolve_frozen_task_final_approach_candidates(
        agent,
        task=task,
        grid_map=grid_map,
        progress_state=progress_state,
    )
    candidate_count = len(frozen_candidates)
    if candidate_count == 0:
        return TaskFinalApproachSelection(
            task_id=task.task_id,
            candidate_index=0,
            candidate_x=task.target_x,
            candidate_y=task.target_y,
            attempt_count=0,
            status="active",
            candidate_count=0,
            frozen_candidates=(),
            failed_candidate_indexes=(),
            attempted_candidate_indexes=(),
        )

    attempted_candidate_indexes = _normalize_attempted_candidate_indexes(
        progress_state=progress_state,
        task_id=task.task_id,
        candidate_count=candidate_count,
    )
    failed_candidate_indexes = _normalize_failed_candidate_indexes(
        progress_state=progress_state,
        task_id=task.task_id,
        candidate_count=candidate_count,
    )
    effective_failed_candidate_indexes = tuple(
        index for index in failed_candidate_indexes if index in attempted_candidate_indexes
    )
    if progress_state.task_final_approach_task_id == task.task_id:
        candidate_index = _find_next_available_candidate_index(
            candidate_count,
            failed_candidate_indexes=effective_failed_candidate_indexes,
            start_index=progress_state.task_final_approach_candidate_index,
        )
        attempt_count = progress_state.task_final_approach_attempt_count
        status = progress_state.task_final_approach_status or "active"
    else:
        candidate_index = _find_next_available_candidate_index(
            candidate_count,
            failed_candidate_indexes=(),
            start_index=0,
        )
        attempt_count = 0
        status = "active"
    if candidate_index is None:
        candidate_index = min(
            max(progress_state.task_final_approach_candidate_index, 0),
            candidate_count - 1,
        )
        status = "exhausted"
    candidate_x, candidate_y = frozen_candidates[candidate_index]
    return TaskFinalApproachSelection(
        task_id=task.task_id,
        candidate_index=candidate_index,
        candidate_x=candidate_x,
        candidate_y=candidate_y,
        attempt_count=attempt_count,
        status=status,
        candidate_count=candidate_count,
        frozen_candidates=frozen_candidates,
        failed_candidate_indexes=effective_failed_candidate_indexes,
        attempted_candidate_indexes=attempted_candidate_indexes,
    )


def apply_task_final_approach_selection(
    progress_state: AgentProgressState,
    *,
    selection: TaskFinalApproachSelection,
) -> AgentProgressState:
    """Persist one selected final-approach candidate into progress state."""

    attempted_candidate_indexes = set(selection.attempted_candidate_indexes)
    candidate_changed = (
        progress_state.task_final_approach_task_id != selection.task_id
        or progress_state.task_final_approach_candidate_index != selection.candidate_index
    )
    if (
        selection.status != "rotated_after_recovery"
        and selection.candidate_count > 0
        and 0 <= selection.candidate_index < selection.candidate_count
    ):
        attempted_candidate_indexes.add(selection.candidate_index)
    return replace(
        progress_state,
        task_final_approach_task_id=selection.task_id,
        task_final_approach_frozen_candidates=selection.frozen_candidates,
        task_final_approach_candidate_index=selection.candidate_index,
        task_final_approach_candidate_x=selection.candidate_x,
        task_final_approach_candidate_y=selection.candidate_y,
        task_final_approach_failed_candidate_indexes=selection.failed_candidate_indexes,
        task_final_approach_attempted_candidate_indexes=tuple(
            sorted(attempted_candidate_indexes)
        ),
        task_final_approach_attempt_count=selection.attempt_count,
        task_final_approach_status=selection.status,
        task_final_approach_low_progress_count=(
            0
            if candidate_changed
            else progress_state.task_final_approach_low_progress_count
        ),
    )


def advance_task_final_approach_after_failure(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
    progress_state: AgentProgressState,
) -> tuple[TaskFinalApproachSelection | None, bool, bool]:
    """Rotate to the next candidate after one failed final-approach attempt."""

    frozen_candidates = _resolve_frozen_task_final_approach_candidates(
        agent,
        task=task,
        grid_map=grid_map,
        progress_state=progress_state,
    )
    candidate_count = len(frozen_candidates)
    if candidate_count == 0:
        return (None, True, False)
    next_attempt_count = progress_state.task_final_approach_attempt_count + 1
    attempted_candidate_indexes = set(
        _normalize_attempted_candidate_indexes(
            progress_state=progress_state,
            task_id=task.task_id,
            candidate_count=candidate_count,
        )
    )
    failed_candidate_indexes = set(
        _normalize_failed_candidate_indexes(
            progress_state=progress_state,
            task_id=task.task_id,
            candidate_count=candidate_count,
        )
    )
    current_index = (
        progress_state.task_final_approach_candidate_index
        if progress_state.task_final_approach_task_id == task.task_id
        else 0
    )
    current_index = min(max(current_index, 0), candidate_count - 1)
    attempted_candidate_indexes.add(current_index)
    failed_candidate_indexes.intersection_update(attempted_candidate_indexes)
    failed_candidate_indexes.add(current_index)
    normalized_attempted_candidate_indexes = tuple(sorted(attempted_candidate_indexes))
    normalized_failed_candidate_indexes = tuple(sorted(failed_candidate_indexes))
    next_index = _find_next_available_candidate_index(
        candidate_count,
        failed_candidate_indexes=normalized_failed_candidate_indexes,
        start_index=current_index + 1,
    )
    if next_index is None:
        last_index = max(min(current_index, candidate_count - 1), 0)
        last_candidate_x, last_candidate_y = frozen_candidates[last_index]
        if _should_hold_task_after_final_approach_failure(
            progress_state,
            task=task,
        ):
            return (
                TaskFinalApproachSelection(
                    task_id=task.task_id,
                    candidate_index=last_index,
                    candidate_x=last_candidate_x,
                    candidate_y=last_candidate_y,
                    attempt_count=next_attempt_count,
                    status="hold_reset",
                    candidate_count=candidate_count,
                    frozen_candidates=frozen_candidates,
                    failed_candidate_indexes=normalized_failed_candidate_indexes,
                    attempted_candidate_indexes=normalized_attempted_candidate_indexes,
                ),
                False,
                True,
            )
        return (
            TaskFinalApproachSelection(
                task_id=task.task_id,
                candidate_index=last_index,
                candidate_x=last_candidate_x,
                candidate_y=last_candidate_y,
                attempt_count=next_attempt_count,
                status="exhausted",
                candidate_count=candidate_count,
                frozen_candidates=frozen_candidates,
                failed_candidate_indexes=normalized_failed_candidate_indexes,
                attempted_candidate_indexes=normalized_attempted_candidate_indexes,
            ),
            True,
            False,
        )
    candidate_x, candidate_y = frozen_candidates[next_index]
    return (
        TaskFinalApproachSelection(
            task_id=task.task_id,
            candidate_index=next_index,
            candidate_x=candidate_x,
            candidate_y=candidate_y,
            attempt_count=next_attempt_count,
            status="rotated_after_recovery",
            candidate_count=candidate_count,
            frozen_candidates=frozen_candidates,
            failed_candidate_indexes=normalized_failed_candidate_indexes,
            attempted_candidate_indexes=normalized_attempted_candidate_indexes,
        ),
        False,
        False,
    )


def task_final_approach_backoff_active(
    progress_state: AgentProgressState,
    *,
    task_id: str,
    step: int,
) -> bool:
    """Return whether one task should temporarily skip final-approach candidates."""

    return (
        progress_state.task_final_approach_backoff_task_id == task_id
        and progress_state.task_final_approach_backoff_until_step > step
    )


def task_final_approach_hold_reset_count(
    progress_state: AgentProgressState,
    *,
    task_id: str,
) -> int:
    """Return the hold-reset count for one task episode."""

    if progress_state.task_final_approach_backoff_task_id != task_id:
        return 0
    return progress_state.task_final_approach_hold_reset_count


def task_approach_escalation_active(
    progress_state: AgentProgressState,
    *,
    task_id: str,
) -> bool:
    """Return whether one task is already in macro approach escalation mode."""

    return progress_state.task_approach_escalation_task_id == task_id


def supports_task_approach_anchors(
    task: TaskRecord,
    *,
    progress_state: AgentProgressState | None = None,
) -> bool:
    """Return whether one task should use the macro approach-side layer."""

    return (
        task.task_type == TaskType.HOTSPOT_CONFIRMATION
        and progress_state is not None
        and task_approach_escalation_active(progress_state, task_id=task.task_id)
    )


def activate_task_approach_escalation(
    progress_state: AgentProgressState,
    *,
    agent: AgentState,
    task: TaskRecord,
    grid_map: GridMap,
    step: int,
) -> AgentProgressState:
    """Enable macro approach-side escalation for one hotspot task."""

    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return progress_state
    anchors = resolve_task_approach_anchors(
        progress_state,
        agent=agent,
        task=task,
        grid_map=grid_map,
    )
    return replace(
        progress_state,
        task_approach_escalation_task_id=task.task_id,
        task_approach_commit_until_step=step + TASK_APPROACH_SIDE_COMMIT_STEPS,
        task_approach_task_id=task.task_id,
        task_approach_anchor_left_x=anchors.left_x,
        task_approach_anchor_left_y=anchors.left_y,
        task_approach_anchor_right_x=anchors.right_x,
        task_approach_anchor_right_y=anchors.right_y,
        task_approach_active_side=anchors.preferred_side,
        task_approach_failed_sides=(),
        task_approach_anchor_status="enroute_anchor",
    )


def prepare_task_approach_state(
    progress_state: AgentProgressState,
    *,
    agent: AgentState,
    task: TaskRecord,
    grid_map: GridMap,
) -> AgentProgressState:
    """Ensure one task episode has frozen left/right approach anchors."""

    if not supports_task_approach_anchors(task, progress_state=progress_state):
        return progress_state
    anchors = resolve_task_approach_anchors(
        progress_state,
        agent=agent,
        task=task,
        grid_map=grid_map,
    )
    if progress_state.task_approach_task_id == task.task_id:
        return progress_state
    return replace(
        progress_state,
        task_approach_escalation_task_id=task.task_id,
        task_approach_task_id=task.task_id,
        task_approach_anchor_left_x=anchors.left_x,
        task_approach_anchor_left_y=anchors.left_y,
        task_approach_anchor_right_x=anchors.right_x,
        task_approach_anchor_right_y=anchors.right_y,
        task_approach_active_side=anchors.preferred_side,
        task_approach_failed_sides=(),
        task_approach_anchor_status="enroute_anchor",
        task_approach_commit_until_step=progress_state.task_approach_commit_until_step,
    )


def resolve_task_approach_anchors(
    progress_state: AgentProgressState,
    *,
    agent: AgentState,
    task: TaskRecord,
    grid_map: GridMap,
) -> TaskApproachAnchors:
    """Return the frozen macro left/right approach anchors for one task episode."""

    if (
        progress_state.task_approach_task_id == task.task_id
        and progress_state.task_approach_anchor_left_x is not None
        and progress_state.task_approach_anchor_left_y is not None
        and progress_state.task_approach_anchor_right_x is not None
        and progress_state.task_approach_anchor_right_y is not None
    ):
        preferred_side = progress_state.task_approach_active_side or TASK_APPROACH_LEFT
        if preferred_side not in _TASK_APPROACH_SIDES:
            preferred_side = TASK_APPROACH_LEFT
        return TaskApproachAnchors(
            task_id=task.task_id,
            left_x=progress_state.task_approach_anchor_left_x,
            left_y=progress_state.task_approach_anchor_left_y,
            right_x=progress_state.task_approach_anchor_right_x,
            right_y=progress_state.task_approach_anchor_right_y,
            preferred_side=preferred_side,
        )
    return _build_task_approach_anchors(agent, task=task, grid_map=grid_map)


def current_task_approach_anchor(
    progress_state: AgentProgressState,
    *,
    task_id: str,
) -> tuple[float, float] | None:
    """Return the active macro approach anchor for one task, if any."""

    if progress_state.task_approach_task_id != task_id:
        return None
    side = progress_state.task_approach_active_side
    if side == TASK_APPROACH_LEFT:
        if (
            progress_state.task_approach_anchor_left_x is None
            or progress_state.task_approach_anchor_left_y is None
        ):
            return None
        return (
            progress_state.task_approach_anchor_left_x,
            progress_state.task_approach_anchor_left_y,
        )
    if side == TASK_APPROACH_RIGHT:
        if (
            progress_state.task_approach_anchor_right_x is None
            or progress_state.task_approach_anchor_right_y is None
        ):
            return None
        return (
            progress_state.task_approach_anchor_right_x,
            progress_state.task_approach_anchor_right_y,
        )
    return None


def task_approach_anchor_reached(
    progress_state: AgentProgressState,
    *,
    agent: AgentState,
    task_id: str,
) -> bool:
    """Return whether the current agent has reached the active macro anchor."""

    if (
        progress_state.task_approach_task_id == task_id
        and progress_state.task_approach_anchor_status == "final_approach"
    ):
        # Once we have committed from the macro anchor into final approach,
        # keep that phase latched until recovery/backoff explicitly sends us
        # back to the anchor. Otherwise moving away from the anchor toward the
        # final-approach candidate immediately bounces us back to enroute_anchor.
        return True
    anchor = current_task_approach_anchor(progress_state, task_id=task_id)
    if anchor is None:
        return True
    return hypot(anchor[0] - agent.x, anchor[1] - agent.y) <= max(
        TASK_APPROACH_ANCHOR_REACHED_M,
        agent.arrival_tolerance_m * 2.0,
    )


def set_task_approach_anchor_status(
    progress_state: AgentProgressState,
    *,
    task_id: str,
    status: str,
) -> AgentProgressState:
    """Persist one task approach-anchor phase label."""

    if progress_state.task_approach_task_id != task_id:
        return progress_state
    return replace(progress_state, task_approach_anchor_status=status)


def rotate_task_approach_side(
    progress_state: AgentProgressState,
    *,
    task_id: str,
    step: int,
) -> AgentProgressState:
    """Switch one task episode to the opposite macro approach side."""

    if (
        progress_state.task_approach_task_id != task_id
        or progress_state.task_approach_escalation_task_id != task_id
    ):
        return progress_state
    current_side = progress_state.task_approach_active_side
    if current_side not in _TASK_APPROACH_SIDES:
        current_side = TASK_APPROACH_LEFT
    next_side = TASK_APPROACH_RIGHT if current_side == TASK_APPROACH_LEFT else TASK_APPROACH_LEFT
    failed_sides = set(progress_state.task_approach_failed_sides)
    failed_sides.add(current_side)
    return replace(
        progress_state,
        task_approach_active_side=next_side,
        task_approach_failed_sides=tuple(sorted(failed_sides)),
        task_approach_anchor_status="enroute_anchor",
        task_approach_commit_until_step=step + TASK_APPROACH_SIDE_COMMIT_STEPS,
    )


def reset_task_final_approach_progress(
    progress_state: AgentProgressState,
    *,
    task_id: str | None = None,
) -> AgentProgressState:
    """Clear only final-approach local candidate state while keeping macro side memory."""

    if task_id is not None and progress_state.task_final_approach_task_id != task_id:
        return progress_state
    return replace(
        progress_state,
        task_final_approach_task_id=None,
        task_final_approach_frozen_candidates=(),
        task_final_approach_candidate_index=-1,
        task_final_approach_candidate_x=None,
        task_final_approach_candidate_y=None,
        task_final_approach_failed_candidate_indexes=(),
        task_final_approach_attempted_candidate_indexes=(),
        task_final_approach_attempt_count=0,
        task_final_approach_status=None,
        task_final_approach_low_progress_count=0,
    )


def _should_hold_task_after_final_approach_failure(
    progress_state: AgentProgressState,
    *,
    task: TaskRecord,
) -> bool:
    if task.task_type not in {TaskType.BASELINE_SERVICE, TaskType.HOTSPOT_CONFIRMATION}:
        return False
    return (
        task_final_approach_hold_reset_count(progress_state, task_id=task.task_id)
        < TASK_FINAL_APPROACH_MAX_HOLD_RESETS
    )


def _normalize_failed_candidate_indexes(
    *,
    progress_state: AgentProgressState,
    task_id: str,
    candidate_count: int,
) -> tuple[int, ...]:
    if progress_state.task_final_approach_task_id != task_id or candidate_count <= 0:
        return ()
    normalized_indexes: list[int] = []
    seen_indexes: set[int] = set()
    for index in progress_state.task_final_approach_failed_candidate_indexes:
        if index < 0 or index >= candidate_count or index in seen_indexes:
            continue
        seen_indexes.add(index)
        normalized_indexes.append(index)
    return tuple(normalized_indexes)


def _normalize_attempted_candidate_indexes(
    *,
    progress_state: AgentProgressState,
    task_id: str,
    candidate_count: int,
) -> tuple[int, ...]:
    if progress_state.task_final_approach_task_id != task_id or candidate_count <= 0:
        return ()
    normalized_indexes: list[int] = []
    seen_indexes: set[int] = set()
    for index in progress_state.task_final_approach_attempted_candidate_indexes:
        if index < 0 or index >= candidate_count or index in seen_indexes:
            continue
        seen_indexes.add(index)
        normalized_indexes.append(index)
    return tuple(normalized_indexes)


def _find_next_available_candidate_index(
    candidate_count: int,
    *,
    failed_candidate_indexes: tuple[int, ...],
    start_index: int,
) -> int | None:
    if candidate_count <= 0:
        return None
    blocked_indexes = set(failed_candidate_indexes)
    normalized_start_index = max(start_index, 0) % candidate_count
    for offset in range(candidate_count):
        candidate_index = (normalized_start_index + offset) % candidate_count
        if candidate_index not in blocked_indexes:
            return candidate_index
    return None


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


def _build_task_approach_anchors(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
) -> TaskApproachAnchors:
    anchor_distance = max(agent.coverage_radius * 2.5, TASK_APPROACH_MIN_ANCHOR_DISTANCE_M)
    side_offset = max(agent.coverage_radius * 1.5, TASK_APPROACH_MIN_SIDE_OFFSET_M)
    radial_dx = agent.x - task.target_x
    radial_dy = agent.y - task.target_y
    radial_distance = hypot(radial_dx, radial_dy)
    if radial_distance <= 1e-6:
        heading_rad = radians(agent.heading_deg)
        radial_ux = -cos(heading_rad)
        radial_uy = -sin(heading_rad)
    else:
        radial_ux = radial_dx / radial_distance
        radial_uy = radial_dy / radial_distance
    tangent_left_x = -radial_uy
    tangent_left_y = radial_ux
    center_x = task.target_x + radial_ux * anchor_distance
    center_y = task.target_y + radial_uy * anchor_distance
    left_x, left_y = _clamp_anchor_to_map(
        center_x + tangent_left_x * side_offset,
        center_y + tangent_left_y * side_offset,
        grid_map=grid_map,
    )
    right_x, right_y = _clamp_anchor_to_map(
        center_x - tangent_left_x * side_offset,
        center_y - tangent_left_y * side_offset,
        grid_map=grid_map,
    )
    left_score = _score_task_approach_anchor(agent, left_x, left_y, grid_map=grid_map)
    right_score = _score_task_approach_anchor(agent, right_x, right_y, grid_map=grid_map)
    preferred_side = TASK_APPROACH_LEFT if left_score <= right_score else TASK_APPROACH_RIGHT
    return TaskApproachAnchors(
        task_id=task.task_id,
        left_x=left_x,
        left_y=left_y,
        right_x=right_x,
        right_y=right_y,
        preferred_side=preferred_side,
    )


def _score_task_approach_anchor(
    agent: AgentState,
    anchor_x: float,
    anchor_y: float,
    *,
    grid_map: GridMap,
) -> tuple[float, float, float]:
    return (
        -_approach_clearance(agent, anchor_x, anchor_y, grid_map),
        -_hazard_clearance(anchor_x, anchor_y, grid_map),
        hypot(anchor_x - agent.x, anchor_y - agent.y),
    )


def _clamp_anchor_to_map(x: float, y: float, *, grid_map: GridMap) -> tuple[float, float]:
    return (
        min(max(x, USV_TASK_EDGE_APPROACH_BUFFER_M), grid_map.width - USV_TASK_EDGE_APPROACH_BUFFER_M),
        min(max(y, USV_TASK_EDGE_APPROACH_BUFFER_M), grid_map.height - USV_TASK_EDGE_APPROACH_BUFFER_M),
    )


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


def _resolve_frozen_task_final_approach_candidates(
    agent: AgentState,
    *,
    task: TaskRecord,
    grid_map: GridMap,
    progress_state: AgentProgressState,
) -> tuple[tuple[float, float], ...]:
    """Return the stable candidate set for one task episode."""

    if (
        progress_state.task_final_approach_task_id == task.task_id
        and progress_state.task_final_approach_frozen_candidates
    ):
        return progress_state.task_final_approach_frozen_candidates

    approach_plan = build_task_final_approach(agent, task=task, grid_map=grid_map)
    return tuple((candidate.x, candidate.y) for candidate in approach_plan.candidates)


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
