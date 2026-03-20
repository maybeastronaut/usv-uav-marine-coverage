"""Task lifecycle helpers for the replay simulation."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.agent_model import AgentState, has_completed_uav_resupply
from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_return_to_patrol,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionStage,
)
from usv_uav_marine_coverage.execution.progress_feedback import reset_progress_state
from usv_uav_marine_coverage.information_map import HotspotKnowledgeState, InformationMap
from usv_uav_marine_coverage.planning.usv_patrol_planner import (
    find_local_patrol_segment_access,
)
from usv_uav_marine_coverage.tasking.task_types import (
    TaskAssignment,
    TaskRecord,
    TaskStatus,
    TaskType,
)


def select_assigned_task_for_agent(
    task_records: tuple[TaskRecord, ...],
    agent_id: str,
) -> TaskRecord | None:
    """Return the assigned in-flight task for one agent, if any."""

    for task in task_records:
        if task.assigned_agent_id != agent_id:
            continue
        if task.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
            return task
    return None


def sync_task_statuses(
    task_records: tuple[TaskRecord, ...],
    execution_states: dict[str, AgentExecutionState],
) -> tuple[TaskRecord, ...]:
    """Sync task lifecycle states from the current execution states."""

    synced_tasks: list[TaskRecord] = []
    for task in task_records:
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            synced_tasks.append(task)
            continue
        if task.assigned_agent_id is None:
            synced_tasks.append(task)
            continue

        execution_state = execution_states.get(task.assigned_agent_id)
        if execution_state is None:
            synced_tasks.append(replace(task, status=TaskStatus.REQUEUED, assigned_agent_id=None))
            continue

        if execution_state.active_task_id == task.task_id and execution_state.stage in {
            ExecutionStage.ON_TASK,
            ExecutionStage.ON_RECHARGE,
        }:
            synced_tasks.append(replace(task, status=TaskStatus.IN_PROGRESS))
            continue
        if execution_state.active_task_id == task.task_id and execution_state.stage in {
            ExecutionStage.GO_TO_TASK,
            ExecutionStage.GO_TO_RENDEZVOUS,
            ExecutionStage.RECOVERY,
        }:
            synced_tasks.append(replace(task, status=TaskStatus.ASSIGNED))
            continue
        if execution_state.active_task_id is None and execution_state.stage in {
            ExecutionStage.PATROL,
            ExecutionStage.RETURN_TO_PATROL,
        }:
            synced_tasks.append(replace(task, status=TaskStatus.REQUEUED, assigned_agent_id=None))
            continue
        synced_tasks.append(task)
    return tuple(synced_tasks)


def finalize_task_resolutions(
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    task_records: tuple[TaskRecord, ...],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    info_map: InformationMap,
    step: int,
) -> tuple[
    tuple[TaskRecord, ...],
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    """Finalize task resolutions and move completed agents into patrol return."""

    agent_by_id = {agent.agent_id: agent for agent in agents}
    next_execution_states = dict(execution_states)
    next_progress_states = dict(progress_states)
    resolved_tasks: list[TaskRecord] = []

    for task in task_records:
        if task.status == TaskStatus.COMPLETED:
            resolved_tasks.append(task)
            continue

        if task.task_type == TaskType.HOTSPOT_CONFIRMATION:
            resolved_task, next_execution_states, next_progress_states = _finalize_hotspot_task(
                task,
                agent_by_id=agent_by_id,
                execution_states=next_execution_states,
                progress_states=next_progress_states,
                patrol_routes=patrol_routes,
                info_map=info_map,
                step=step,
            )
            resolved_tasks.append(resolved_task)
            continue

        if task.task_type == TaskType.BASELINE_SERVICE:
            resolved_task, next_execution_states, next_progress_states = _finalize_baseline_task(
                task,
                agent_by_id=agent_by_id,
                execution_states=next_execution_states,
                progress_states=next_progress_states,
                patrol_routes=patrol_routes,
                info_map=info_map,
                step=step,
            )
            resolved_tasks.append(resolved_task)
            continue

        if task.task_type == TaskType.UAV_RESUPPLY:
            (
                resolved_task,
                next_execution_states,
                next_progress_states,
            ) = _finalize_uav_resupply_task(
                task,
                agent_by_id=agent_by_id,
                execution_states=next_execution_states,
                progress_states=next_progress_states,
                patrol_routes=patrol_routes,
                step=step,
            )
            resolved_tasks.append(resolved_task)
            continue

        resolved_tasks.append(task)

    return (tuple(resolved_tasks), next_execution_states, next_progress_states)


def serialize_task_assignment(assignment: TaskAssignment) -> dict[str, object]:
    """Serialize one assignment decision for the step log layer."""

    candidate_agents = list(assignment.candidate_agents)
    if not candidate_agents:
        candidate_agents = [
            {
                "agent_id": assignment.agent_id,
                "support_agent_id": assignment.support_agent_id,
            }
        ]

    return {
        "task_id": assignment.task_id,
        "task_type": assignment.task_type.value,
        "selected_agent": assignment.agent_id,
        "support_agent": assignment.support_agent_id,
        "candidate_agents": candidate_agents,
        "selection_reason": assignment.selection_reason,
        "selection_score": round(assignment.selection_score, 3),
        "selection_details": assignment.selection_details,
    }


def _finalize_hotspot_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    info_map: InformationMap,
    step: int,
) -> tuple[
    TaskRecord,
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    assert task.target_row is not None and task.target_col is not None
    state = info_map.state_at(task.target_row, task.target_col)
    if state.known_hotspot_state not in {
        HotspotKnowledgeState.CONFIRMED,
        HotspotKnowledgeState.FALSE_ALARM,
    }:
        return (task, execution_states, progress_states)
    state.ground_truth_hotspot = False
    state.ground_truth_hotspot_id = None
    state.ground_truth_hotspot_created_step = None
    state.hotspot_resolution_step = step
    state.known_hotspot_state = HotspotKnowledgeState.NONE
    state.known_hotspot_id = None
    state.uav_checked_by = None
    state.uav_check_step = None
    state.uav_inspection_progress = 0
    state.usv_inspection_progress = 0
    state.task_status = TaskStatus.COMPLETED
    state.assigned_agent_id = None
    state.confirmed_by = None
    return _complete_task_and_return(
        task,
        agent_by_id=agent_by_id,
        execution_states=execution_states,
        progress_states=progress_states,
        patrol_routes=patrol_routes,
        step=step,
    )


def _finalize_baseline_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    info_map: InformationMap,
    step: int,
) -> tuple[
    TaskRecord,
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    assert task.target_row is not None and task.target_col is not None
    state = info_map.state_at(task.target_row, task.target_col)
    if not state.has_baseline_task:
        return _complete_task_and_return(
            task,
            agent_by_id=agent_by_id,
            execution_states=execution_states,
            progress_states=progress_states,
            patrol_routes=patrol_routes,
            step=step,
        )
    if task.assigned_agent_id is None:
        return (task, execution_states, progress_states)

    execution_state = execution_states.get(task.assigned_agent_id)
    if execution_state is None or execution_state.stage != ExecutionStage.ON_TASK:
        return (task, execution_states, progress_states)

    state.task_status = state.task_status.IN_SERVICE
    state.assigned_agent_id = task.assigned_agent_id
    state.baseline_service_progress += 1
    if state.baseline_service_progress < info_map.config.baseline_service_steps:
        return (task, execution_states, progress_states)

    state.has_baseline_task = False
    state.baseline_last_served_step = step
    state.baseline_service_progress = 0
    state.task_status = state.task_status.COMPLETED
    state.assigned_agent_id = None
    return _complete_task_and_return(
        task,
        agent_by_id=agent_by_id,
        execution_states=execution_states,
        progress_states=progress_states,
        patrol_routes=patrol_routes,
        step=step,
    )


def _complete_task_and_return(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    step: int,
) -> tuple[
    TaskRecord,
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    next_execution_states = dict(execution_states)
    next_progress_states = dict(progress_states)
    completed_task = replace(
        task,
        status=TaskStatus.COMPLETED,
        completed_step=step,
    )
    if task.assigned_agent_id is None:
        return (completed_task, next_execution_states, next_progress_states)

    agent = agent_by_id.get(task.assigned_agent_id)
    execution_state = next_execution_states.get(task.assigned_agent_id)
    if agent is None or execution_state is None:
        return (completed_task, next_execution_states, next_progress_states)
    if execution_state.stage not in {ExecutionStage.ON_TASK, ExecutionStage.ON_RECHARGE}:
        return (completed_task, next_execution_states, next_progress_states)

    patrol_route = patrol_routes[agent.agent_id]
    access = find_local_patrol_segment_access(
        agent_x=agent.x,
        agent_y=agent.y,
        patrol_route=patrol_route,
        preferred_end_index=execution_state.patrol_waypoint_index,
    )
    if access is None:
        patrol_index = execution_state.patrol_waypoint_index
        return_x, return_y = patrol_route[patrol_index]
        rejoin_to_segment = False
    else:
        patrol_index = access.segment_end_index
        return_x = access.access_x
        return_y = access.access_y
        rejoin_to_segment = True
    next_execution_states[task.assigned_agent_id] = transition_to_return_to_patrol(
        execution_state,
        return_target_x=return_x,
        return_target_y=return_y,
        patrol_waypoint_index=patrol_index,
        rejoin_to_segment=rejoin_to_segment,
    )
    progress_state = next_progress_states.get(task.assigned_agent_id)
    if progress_state is not None:
        next_progress_states[task.assigned_agent_id] = reset_progress_state(progress_state)
    return (completed_task, next_execution_states, next_progress_states)


def _finalize_uav_resupply_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    step: int,
) -> tuple[
    TaskRecord,
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    if task.assigned_agent_id is None:
        return (task, execution_states, progress_states)

    agent = agent_by_id.get(task.assigned_agent_id)
    execution_state = execution_states.get(task.assigned_agent_id)
    if agent is None or execution_state is None:
        return (task, execution_states, progress_states)
    if execution_state.stage != ExecutionStage.ON_RECHARGE:
        return (task, execution_states, progress_states)
    if not has_completed_uav_resupply(agent):
        return (task, execution_states, progress_states)
    return _complete_task_and_return(
        task,
        agent_by_id=agent_by_id,
        execution_states=execution_states,
        progress_states=progress_states,
        patrol_routes=patrol_routes,
        step=step,
    )
