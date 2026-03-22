"""Shared helpers for task-allocation policies."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    can_support_uav_resupply,
    estimate_uav_energy_to_point,
    is_operational_agent,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage

from . import partitioning as zone_partition_layer
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

NEARSHORE_X_END_M = zone_partition_layer.NEARSHORE_X_END_M
OFFSHORE_Y_SPLIT_M = zone_partition_layer.OFFSHORE_Y_SPLIT_M
RETURN_TO_PATROL_OPPORTUNISTIC_TASK_RADIUS_M = 120.0
HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M = RETURN_TO_PATROL_OPPORTUNISTIC_TASK_RADIUS_M


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


def preferred_usv_ids_for_task(task: TaskRecord) -> set[str]:
    """Return the baseline fixed-partition primary USV set for one task."""

    return zone_partition_layer.baseline_primary_usv_ids_for_task(task)


def allocate_uav_resupply_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState] | None = None,
    task_records: tuple[TaskRecord, ...] = (),
) -> tuple[TaskRecord, TaskAssignment | None]:
    """Allocate one UAV-resupply task using the nearest supporting USV."""

    selected_uav = agent_by_id.get(task.assigned_agent_id or "")
    if selected_uav is None or selected_uav.kind != "UAV":
        return (replace(task, status=TaskStatus.PENDING, assigned_agent_id=None), None)

    usv_candidates = [
        agent
        for agent in agent_by_id.values()
        if can_support_uav_resupply(agent)
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
    if not usv_candidates:
        return (
            replace(task, status=TaskStatus.PENDING, support_agent_id=None),
            None,
        )

    support_usv = min(
        usv_candidates,
        key=lambda agent: hypot(agent.x - selected_uav.x, agent.y - selected_uav.y),
    )
    selection_score = round(
        hypot(support_usv.x - selected_uav.x, support_usv.y - selected_uav.y),
        3,
    )
    updated_task = replace(
        task,
        status=TaskStatus.ASSIGNED,
        assigned_agent_id=selected_uav.agent_id,
        support_agent_id=support_usv.agent_id,
        target_x=support_usv.x,
        target_y=support_usv.y,
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


def allocate_hotspot_proximity_override_task(
    task: TaskRecord,
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    task_records: tuple[TaskRecord, ...],
    reserved_agent_ids: set[str] = frozenset(),
) -> tuple[TaskRecord, TaskAssignment] | None:
    """Force-assign one nearby hotspot to an idle patrol/returning USV.

    In failure emergency mode, this override may preempt a not-yet-started
    UAV-resupply reservation that only holds the support USV in ``ASSIGNED``.
    """

    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return None
    if not any(agent.kind == "USV" and not is_operational_agent(agent) for agent in agents):
        return None
    if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED, TaskStatus.IN_PROGRESS}:
        return None
    if _has_active_executor_for_task(task, execution_states=execution_states):
        return None

    candidates: list[tuple[AgentState, AgentExecutionState, float]] = []
    for agent in agents:
        if agent.kind != "USV" or not is_operational_agent(agent):
            continue
        if agent.agent_id in reserved_agent_ids:
            continue
        execution_state = execution_states.get(agent.agent_id)
        if execution_state is None or execution_state.active_task_id is not None:
            continue
        if execution_state.stage not in {ExecutionStage.PATROL, ExecutionStage.RETURN_TO_PATROL}:
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
        distance_m = distance_to_task(agent, task)
        if distance_m > HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M:
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
    return (
        replace(
            task,
            status=TaskStatus.ASSIGNED,
            assigned_agent_id=selected_agent.agent_id,
        ),
        TaskAssignment(
            task_id=task.task_id,
            task_type=task.task_type,
            agent_id=selected_agent.agent_id,
            support_agent_id=None,
            selection_reason="nearby_hotspot_proximity_takeover",
            selection_score=selection_score,
            selection_details={
                "override_radius_m": HOTSPOT_PROXIMITY_OVERRIDE_RADIUS_M,
                "override_distance_m": selection_score,
                "override_stage": selected_state.stage.value,
                "preempted_support_task_ids": list(
                    _preemptible_uav_support_task_ids(
                        selected_agent.agent_id,
                        task_records=task_records,
                    )
                ),
            },
        ),
    )


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
    return round(hypot(support_agent.x - selected_agent.x, support_agent.y - selected_agent.y), 3)


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
