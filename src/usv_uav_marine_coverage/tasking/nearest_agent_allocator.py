"""Nearest-agent task allocation for the first hotspot-confirmation loop."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage

from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType


def allocate_nearest_agents(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Assign eligible hotspot-confirmation tasks to the nearest available USV."""

    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []

    for task in tasks:
        if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
            updated_tasks.append(task)
            continue
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            updated_tasks.append(task)
            continue

        if _can_keep_existing_assignment(
            task, agent_by_id=agent_by_id, execution_states=execution_states
        ):
            updated_tasks.append(task)
            decisions.append(
                TaskAssignment(
                    task_id=task.task_id,
                    agent_id=task.assigned_agent_id or "",
                    selection_reason="keep_existing_assignment",
                    selection_score=_distance_to_task(
                        agent_by_id[task.assigned_agent_id or ""],
                        task,
                    ),
                )
            )
            continue

        candidates = [
            agent
            for agent in agents
            if agent.kind == "USV"
            and _is_available_for_new_assignment(
                execution_states.get(agent.agent_id),
            )
        ]
        if not candidates:
            updated_tasks.append(
                replace(
                    task,
                    status=TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING,
                    assigned_agent_id=None,
                )
            )
            continue

        selected = min(candidates, key=lambda agent: _distance_to_task(agent, task))
        score = _distance_to_task(selected, task)
        updated_tasks.append(
            replace(
                task,
                status=TaskStatus.ASSIGNED,
                assigned_agent_id=selected.agent_id,
            )
        )
        decisions.append(
            TaskAssignment(
                task_id=task.task_id,
                agent_id=selected.agent_id,
                selection_reason="nearest_available_usv",
                selection_score=score,
            )
        )

    return (tuple(updated_tasks), tuple(decisions))


def _can_keep_existing_assignment(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
) -> bool:
    if task.assigned_agent_id is None:
        return False
    agent = agent_by_id.get(task.assigned_agent_id)
    if agent is None or agent.kind != "USV":
        return False
    execution_state = execution_states.get(task.assigned_agent_id)
    if execution_state is None:
        return False
    return execution_state.active_task_id in {None, task.task_id}


def _is_available_for_new_assignment(execution_state: AgentExecutionState | None) -> bool:
    if execution_state is None:
        return False
    return execution_state.active_task_id is None and execution_state.stage == ExecutionStage.PATROL


def _distance_to_task(agent: AgentState, task: TaskRecord) -> float:
    return round(hypot(agent.x - task.target_x, agent.y - task.target_y), 3)
