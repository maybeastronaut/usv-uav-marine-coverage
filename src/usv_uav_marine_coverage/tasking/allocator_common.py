"""Shared helpers for task-allocation policies."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage

from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

NEARSHORE_X_END_M = 250.0
OFFSHORE_Y_SPLIT_M = 500.0


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
    if task.task_type == TaskType.UAV_RESUPPLY:
        if agent.kind != "UAV":
            return False
        return task.support_agent_id is not None
    if agent.kind != "USV":
        return False
    execution_state = execution_states.get(task.assigned_agent_id)
    if execution_state is None:
        return False
    return execution_state.active_task_id in {None, task.task_id}


def is_available_for_new_assignment(execution_state: AgentExecutionState | None) -> bool:
    """Return whether one agent execution state can accept a new task."""

    if execution_state is None:
        return False
    return execution_state.active_task_id is None and execution_state.stage == ExecutionStage.PATROL


def distance_to_task(agent: AgentState, task: TaskRecord) -> float:
    """Return the direct-line distance between one agent and one task target."""

    return round(hypot(agent.x - task.target_x, agent.y - task.target_y), 3)


def preferred_usv_ids_for_task(task: TaskRecord) -> set[str]:
    """Return the hard responsibility-zone filter for one task."""

    if task.task_type == TaskType.UAV_RESUPPLY:
        return {"USV-1", "USV-2", "USV-3"}
    if task.target_x < NEARSHORE_X_END_M:
        return {"USV-1"}
    if task.target_y < OFFSHORE_Y_SPLIT_M:
        return {"USV-2"}
    return {"USV-3"}


def allocate_uav_resupply_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
) -> tuple[TaskRecord, TaskAssignment | None]:
    """Allocate one UAV-resupply task using the nearest supporting USV."""

    selected_uav = agent_by_id.get(task.assigned_agent_id or "")
    if selected_uav is None or selected_uav.kind != "UAV":
        return (replace(task, status=TaskStatus.PENDING, assigned_agent_id=None), None)

    usv_candidates = [agent for agent in agent_by_id.values() if agent.kind == "USV"]
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
