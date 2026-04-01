"""Shared helpers for task-allocation policies."""

from __future__ import annotations

from dataclasses import replace
from math import ceil, hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    can_support_uav_resupply,
    estimate_uav_energy_to_point,
    is_operational_agent,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan

from . import partitioning as zone_partition_layer
from .partitioning.failure_hotspot_soft_partition import is_failed_neighbor_recovery_hotspot
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType
from .uav_support_policy import preferred_support_usv_order_for_uav
from .uav_support_policy import (
    fixed_initial_escort_support_usv_id_for_uav,
    is_initial_fixed_escort_phase,
    is_reserved_for_initial_escort,
)

NEARSHORE_X_END_M = zone_partition_layer.NEARSHORE_X_END_M
OFFSHORE_Y_SPLIT_M = zone_partition_layer.OFFSHORE_Y_SPLIT_M
RETURN_TO_PATROL_OPPORTUNISTIC_TASK_RADIUS_M = 120.0
HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M = RETURN_TO_PATROL_OPPORTUNISTIC_TASK_RADIUS_M
FAILED_NEIGHBOR_HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M = 320.0
HOTSPOT_PROXIMITY_OVERRIDE_MIN_GRACE_STEPS = 12
HOTSPOT_PROXIMITY_OVERRIDE_PROGRESS_FLOOR_M_PER_STEP = 4.0
HOTSPOT_PROXIMITY_OVERRIDE_GRACE_BUFFER_STEPS = 4


def task_sort_key(task: TaskRecord) -> tuple[int, int, int, str]:
    """Return the current priority order for task scheduling."""

    status_rank = 1 if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED} else 0
    if task.task_type == TaskType.UAV_RESUPPLY:
        type_rank = 0
    elif task.task_type == TaskType.HOTSPOT_CONFIRMATION:
        type_rank = 1
    else:
        type_rank = 2
    return (status_rank, type_rank, task.created_step, task.task_id)


def can_keep_existing_assignment(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
) -> bool:
    """Return whether one current task assignment is still valid."""

    if task.assigned_agent_id is None:
        return False
    agent = agent_by_id.get(task.assigned_agent_id)
    if agent is None:
        return False
    if not is_operational_agent(agent):
        return False
    if task.task_type == TaskType.UAV_RESUPPLY:
        if agent.kind != "UAV":
            return False
        if task.support_agent_id is None:
            return False
        support_agent = agent_by_id.get(task.support_agent_id)
        return support_agent is not None and can_support_uav_resupply(support_agent)
    if agent.kind != "USV":
        return False
    execution_state = execution_states.get(task.assigned_agent_id)
    if execution_state is None:
        return False
    return execution_state.active_task_id in {None, task.task_id}


def is_reserved_for_uav_support(
    agent_id: str,
    *,
    task_records: tuple[TaskRecord, ...],
    exclude_task_id: str | None = None,
) -> bool:
    """Return whether one USV is already reserved as an active UAV-support platform."""

    for task in task_records:
        if task.task_id == exclude_task_id:
            continue
        if task.task_type != TaskType.UAV_RESUPPLY:
            continue
        if task.status not in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
            continue
        if task.support_agent_id == agent_id:
            return True
    return False


def is_available_for_new_assignment(
    execution_state: AgentExecutionState | None,
    *,
    agent: AgentState | None = None,
    task: TaskRecord | None = None,
    agent_id: str | None = None,
    task_records: tuple[TaskRecord, ...] = (),
) -> bool:
    """Return whether one agent execution state can accept a new task."""

    if execution_state is None:
        return False
    if execution_state.active_task_id is not None:
        return False
    if execution_state.stage == ExecutionStage.PATROL:
        if agent_id is None:
            return True
        return not is_reserved_for_uav_support(agent_id, task_records=task_records)
    if (
        execution_state.stage == ExecutionStage.RETURN_TO_PATROL
        and agent is not None
        and task is not None
        and _can_pick_up_task_while_returning_to_patrol(agent, task=task)
    ):
        if agent_id is None:
            return True
        return not is_reserved_for_uav_support(agent_id, task_records=task_records)
    return False


def is_available_for_candidate_pool(
    execution_state: AgentExecutionState | None,
    *,
    agent_id: str | None = None,
    task_records: tuple[TaskRecord, ...] = (),
) -> bool:
    """Return whether one agent can stay in the new-task candidate pool."""

    if execution_state is None:
        return False
    if execution_state.active_task_id is not None:
        return False
    if execution_state.stage not in {ExecutionStage.PATROL, ExecutionStage.RETURN_TO_PATROL}:
        return False
    if agent_id is None:
        return True
    return not is_reserved_for_uav_support(agent_id, task_records=task_records)


def _can_pick_up_task_while_returning_to_patrol(agent: AgentState, *, task: TaskRecord) -> bool:
    """Return whether one returning USV can opportunistically pick up this task."""

    if task.task_type == TaskType.UAV_RESUPPLY:
        return False
    return distance_to_task(agent, task) <= RETURN_TO_PATROL_OPPORTUNISTIC_TASK_RADIUS_M


def is_available_for_uav_support(
    execution_state: AgentExecutionState | None,
    *,
    agent_id: str,
    task_records: tuple[TaskRecord, ...] = (),
) -> bool:
    """Return whether one USV can act as a rendezvous support platform now.

    UAV resupply should not hijack an actively working USV, but a healthy USV
    that is already back in patrol flow is a valid support candidate.
    """

    if execution_state is None:
        return False
    if is_reserved_for_uav_support(agent_id, task_records=task_records):
        return False
    if execution_state.stage == ExecutionStage.PATROL and execution_state.active_task_id is None:
        return True
    return execution_state.stage == ExecutionStage.RETURN_TO_PATROL


def distance_to_task(agent: AgentState, task: TaskRecord) -> float:
    """Return the direct-line distance between one agent and one task target."""

    return round(hypot(agent.x - task.target_x, agent.y - task.target_y), 3)


def stabilize_uav_resupply_task(
    task: TaskRecord,
    *,
    selected_uav: AgentState | None = None,
) -> TaskRecord:
    """Keep the task target aligned with the stable rendezvous anchor."""

    if task.task_type != TaskType.UAV_RESUPPLY:
        return task
    anchor_x, anchor_y = uav_resupply_anchor(task, selected_uav=selected_uav)
    return replace(
        task,
        target_x=anchor_x,
        target_y=anchor_y,
        rendezvous_anchor_x=anchor_x,
        rendezvous_anchor_y=anchor_y,
    )


def uav_resupply_anchor(
    task: TaskRecord,
    *,
    selected_uav: AgentState | None = None,
) -> tuple[float, float]:
    """Return the stable rendezvous anchor for one UAV-resupply episode."""

    if task.rendezvous_anchor_x is not None and task.rendezvous_anchor_y is not None:
        return (task.rendezvous_anchor_x, task.rendezvous_anchor_y)
    if selected_uav is not None:
        return (selected_uav.x, selected_uav.y)
    return (task.target_x, task.target_y)


def is_task_waiting_for_retry(
    task: TaskRecord,
    *,
    step: int | None,
) -> bool:
    """Return whether the whole task is still inside its global retry cooldown."""

    return task.retry_after_step is not None and step is not None and step < task.retry_after_step


def preferred_usv_ids_for_task(task: TaskRecord) -> set[str]:
    """Return the baseline fixed-partition primary USV set for one task."""

    return zone_partition_layer.baseline_primary_usv_ids_for_task(task)


def allocate_uav_resupply_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState] | None = None,
    grid_map: GridMap | None = None,
    task_records: tuple[TaskRecord, ...] = (),
    step: int | None = None,
    usv_path_planner: str = "astar_path_planner",
) -> tuple[TaskRecord, TaskAssignment | None]:
    """Allocate one UAV-resupply task using the nearest supporting USV."""

    selected_uav = agent_by_id.get(task.assigned_agent_id or "")
    if selected_uav is None or selected_uav.kind != "UAV":
        return (replace(task, status=TaskStatus.PENDING, assigned_agent_id=None), None)
    task = stabilize_uav_resupply_task(task, selected_uav=selected_uav)
    anchor_x, anchor_y = uav_resupply_anchor(task, selected_uav=selected_uav)

    usv_candidates = [
        agent
        for agent in agent_by_id.values()
        if can_support_uav_resupply(agent)
        and not is_agent_task_in_cooldown(task, agent_id=agent.agent_id, step=step)
        and (
            execution_states is None
            or is_available_for_uav_support(
                execution_states.get(agent.agent_id),
                agent_id=agent.agent_id,
                task_records=task_records,
            )
        )
        and not is_reserved_for_uav_support(
            agent.agent_id,
            task_records=task_records,
            exclude_task_id=task.task_id,
        )
    ]
    if is_initial_fixed_escort_phase(step, agent_by_id=agent_by_id):
        preferred_support_id = fixed_initial_escort_support_usv_id_for_uav(
            selected_uav.agent_id,
            step=step,
            agent_by_id=agent_by_id,
        )
        if preferred_support_id is not None:
            usv_candidates = [
                agent for agent in usv_candidates if agent.agent_id == preferred_support_id
            ]
    if not usv_candidates:
        return (
            replace(task, status=TaskStatus.PENDING, support_agent_id=None),
            None,
        )

    support_usv = _select_reachable_support_usv(
        selected_uav,
        usv_candidates=tuple(usv_candidates),
        grid_map=grid_map,
        task=task,
        usv_path_planner=usv_path_planner,
    )
    if support_usv is None:
        return (
            replace(task, status=TaskStatus.PENDING, support_agent_id=None),
            None,
        )
    selection_score = round(
        hypot(support_usv.x - anchor_x, support_usv.y - anchor_y),
        3,
    )
    updated_task = replace(
        task,
        status=TaskStatus.ASSIGNED,
        assigned_agent_id=selected_uav.agent_id,
        support_agent_id=support_usv.agent_id,
        target_x=anchor_x,
        target_y=anchor_y,
    )
    return (
        updated_task,
        TaskAssignment(
            task_id=task.task_id,
            task_type=task.task_type,
            agent_id=selected_uav.agent_id,
            support_agent_id=support_usv.agent_id,
            selection_reason="nearest_usv_rendezvous_for_uav_resupply",
            selection_score=selection_score,
        ),
    )


def _select_reachable_support_usv(
    selected_uav: AgentState,
    *,
    usv_candidates: tuple[AgentState, ...],
    grid_map: GridMap | None,
    task: TaskRecord,
    usv_path_planner: str,
) -> AgentState | None:
    anchor_x, anchor_y = uav_resupply_anchor(task, selected_uav=selected_uav)
    preferred_support_order = preferred_support_usv_order_for_uav(
        selected_uav.agent_id,
        available_usv_ids=tuple(candidate.agent_id for candidate in usv_candidates),
    )
    support_rank = {
        agent_id: rank for rank, agent_id in enumerate(preferred_support_order)
    }
    ranked_candidates = sorted(
        usv_candidates,
        key=lambda agent: (
            support_rank.get(agent.agent_id, len(preferred_support_order)),
            hypot(agent.x - anchor_x, agent.y - anchor_y),
        ),
    )
    if grid_map is None:
        return ranked_candidates[0] if ranked_candidates else None
    for candidate in ranked_candidates:
        plan = build_usv_path_plan(
            candidate,
            grid_map=grid_map,
            goal_x=anchor_x,
            goal_y=anchor_y,
            planner_name=usv_path_planner,
            task_id=task.task_id,
            stats_context="allocator_uav_resupply_support_reachability",
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate
    return None


def allocate_hotspot_proximity_override_task(
    task: TaskRecord,
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    task_records: tuple[TaskRecord, ...],
    reserved_agent_ids: set[str] = frozenset(),
    step: int | None = None,
) -> tuple[TaskRecord, TaskAssignment] | None:
    """Force-assign one nearby hotspot to an idle patrol/returning USV.

    In failure emergency mode, this override may preempt a not-yet-started
    UAV-resupply reservation that only holds the support USV in ``ASSIGNED``.
    """

    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return None
    if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED, TaskStatus.IN_PROGRESS}:
        return None
    if _has_active_executor_for_task(task, execution_states=execution_states):
        return None

    failure_mode_active = any(
        agent.kind == "USV" and not is_operational_agent(agent) for agent in agents
    )
    if not failure_mode_active:
        return None
    override_radius_m = _hotspot_proximity_override_radius(task, agents=agents)
    candidates: list[tuple[AgentState, AgentExecutionState, float]] = []
    for agent in agents:
        if agent.kind != "USV" or not is_operational_agent(agent):
            continue
        if is_reserved_for_initial_escort(
            agent.agent_id,
            step=step,
            agent_by_id={candidate.agent_id: candidate for candidate in agents},
        ):
            continue
        if is_agent_task_in_cooldown(task, agent_id=agent.agent_id, step=step):
            continue
        execution_state = execution_states.get(agent.agent_id)
        if execution_state is None:
            continue
        if _has_in_progress_uav_support(agent.agent_id, task_records=task_records):
            continue
        if not _can_preempt_assigned_uav_support_safely(
            agent.agent_id,
            agents=agents,
            execution_states=execution_states,
            task_records=task_records,
        ):
            continue
        if execution_state.active_task_id is not None:
            continue
        distance_m = distance_to_task(agent, task)
        if execution_state.stage not in {ExecutionStage.PATROL, ExecutionStage.RETURN_TO_PATROL}:
            continue
        if agent.agent_id in reserved_agent_ids:
            continue
        if distance_m > override_radius_m:
            continue
        candidates.append((agent, execution_state, distance_m))

    if not candidates:
        return None

    selected_agent, selected_state, selection_score = min(
        candidates,
        key=lambda item: (
            item[2],
            0 if item[1].stage == ExecutionStage.PATROL else 1,
            item[0].agent_id,
        ),
    )
    suppression_grace_steps = max(
        HOTSPOT_PROXIMITY_OVERRIDE_MIN_GRACE_STEPS,
        ceil(selection_score / HOTSPOT_PROXIMITY_OVERRIDE_PROGRESS_FLOOR_M_PER_STEP)
        + HOTSPOT_PROXIMITY_OVERRIDE_GRACE_BUFFER_STEPS,
    )
    return (
        replace(
            task,
            status=TaskStatus.ASSIGNED,
            assigned_agent_id=selected_agent.agent_id,
            suppression_grace_until_step=(
                None
                if step is None
                else max(task.suppression_grace_until_step or 0, step + suppression_grace_steps)
            ),
        ),
        TaskAssignment(
            task_id=task.task_id,
            task_type=task.task_type,
            agent_id=selected_agent.agent_id,
            support_agent_id=None,
            selection_reason="nearby_hotspot_proximity_takeover",
            selection_score=selection_score,
            selection_details={
                "override_radius_m": override_radius_m,
                "override_distance_m": selection_score,
                "override_stage": selected_state.stage.value,
                "override_mode": "idle_takeover",
                "preempted_support_task_ids": list(
                    _preemptible_uav_support_task_ids(
                        selected_agent.agent_id,
                        task_records=task_records,
                    )
                ),
            },
        ),
    )


def _hotspot_proximity_override_radius(
    task: TaskRecord,
    *,
    agents: tuple[AgentState, ...],
) -> float:
    if is_failed_neighbor_recovery_hotspot(task, agents=agents):
        return FAILED_NEIGHBOR_HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M
    return HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M


def release_preempted_uav_resupply_tasks(
    updated_tasks: list[TaskRecord],
    decisions: list[TaskAssignment],
    *,
    support_agent_id: str,
) -> tuple[list[TaskRecord], list[TaskAssignment], tuple[str, ...]]:
    """Release UAV-resupply reservations preempted by one nearby hotspot override."""

    preempted_task_ids = tuple(
        task.task_id
        for task in updated_tasks
        if task.task_type == TaskType.UAV_RESUPPLY
        and task.support_agent_id == support_agent_id
        and task.status == TaskStatus.ASSIGNED
    )
    if not preempted_task_ids:
        return (updated_tasks, decisions, ())

    released_tasks = [
        replace(task, status=TaskStatus.PENDING, support_agent_id=None)
        if task.task_id in preempted_task_ids
        else task
        for task in updated_tasks
    ]
    released_decisions = [
        decision for decision in decisions if decision.task_id not in preempted_task_ids
    ]
    return (released_tasks, released_decisions, preempted_task_ids)


def selection_score_for_task(
    task: TaskRecord,
    *,
    selected_agent: AgentState,
    agent_by_id: dict[str, AgentState],
) -> float:
    """Return one comparable selection score for logs and summaries."""

    if task.task_type != TaskType.UAV_RESUPPLY or task.support_agent_id is None:
        return distance_to_task(selected_agent, task)
    support_agent = agent_by_id.get(task.support_agent_id)
    if support_agent is None:
        return distance_to_task(selected_agent, task)
    anchor_x, anchor_y = uav_resupply_anchor(task, selected_uav=selected_agent)
    return round(hypot(support_agent.x - anchor_x, support_agent.y - anchor_y), 3)


def _has_active_executor_for_task(
    task: TaskRecord,
    *,
    execution_states: dict[str, AgentExecutionState],
) -> bool:
    """Return whether one task is already being actively executed by its assignee."""

    if task.assigned_agent_id is None:
        return False
    execution_state = execution_states.get(task.assigned_agent_id)
    if execution_state is None:
        return False
    if execution_state.active_task_id != task.task_id:
        return False
    return execution_state.stage in {
        ExecutionStage.GO_TO_TASK,
        ExecutionStage.ON_TASK,
    }


def _has_in_progress_uav_support(
    agent_id: str,
    *,
    task_records: tuple[TaskRecord, ...],
) -> bool:
    """Return whether one USV is already actively performing UAV recharge support."""

    return any(
        task.task_type == TaskType.UAV_RESUPPLY
        and task.support_agent_id == agent_id
        and task.status == TaskStatus.IN_PROGRESS
        for task in task_records
    )


def _preemptible_uav_support_task_ids(
    agent_id: str,
    *,
    task_records: tuple[TaskRecord, ...],
) -> tuple[str, ...]:
    """Return the not-yet-started UAV-resupply tasks that this USV may preempt."""

    return tuple(
        task.task_id
        for task in task_records
        if task.task_type == TaskType.UAV_RESUPPLY
        and task.support_agent_id == agent_id
        and task.status == TaskStatus.ASSIGNED
    )


def _can_preempt_assigned_uav_support_safely(
    agent_id: str,
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    task_records: tuple[TaskRecord, ...],
) -> bool:
    """Return whether preempting one pending UAV-support reservation is safe.

    We only allow hotspot proximity takeover to reclaim a support USV if every
    affected UAV still has at least one alternative healthy support USV that is
    presently available and reachable with its remaining energy.
    """

    preemptible_tasks = tuple(
        task
        for task in task_records
        if task.task_type == TaskType.UAV_RESUPPLY
        and task.support_agent_id == agent_id
        and task.status == TaskStatus.ASSIGNED
    )
    if not preemptible_tasks:
        return True

    agent_by_id = {agent.agent_id: agent for agent in agents}
    for task in preemptible_tasks:
        uav = agent_by_id.get(task.assigned_agent_id or "")
        if uav is None or uav.kind != "UAV":
            continue
        alternatives = tuple(
            support_agent
            for support_agent in agents
            if support_agent.agent_id != agent_id
            and can_support_uav_resupply(support_agent)
            and is_available_for_uav_support(
                execution_states.get(support_agent.agent_id),
                agent_id=support_agent.agent_id,
                task_records=task_records,
            )
            and not is_reserved_for_uav_support(
                support_agent.agent_id,
                task_records=task_records,
                exclude_task_id=task.task_id,
            )
        )
        if not alternatives:
            return False
        if not any(
            estimate_uav_energy_to_point(uav, support_agent.x, support_agent.y)
            <= uav.energy_level
            for support_agent in alternatives
        ):
            return False
    return True


def is_agent_task_in_cooldown(task: TaskRecord, *, agent_id: str, step: int | None) -> bool:
    if step is None:
        return False
    return any(
        candidate_agent_id == agent_id and step < retry_after_step
        for candidate_agent_id, retry_after_step in task.agent_retry_after_steps
    )
