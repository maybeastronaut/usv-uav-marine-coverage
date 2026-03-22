"""Explicit task-claim helpers between tasking and execution."""

from __future__ import annotations

from dataclasses import dataclass

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskStatus, TaskType

from .basic_state_machine import transition_to_rendezvous, transition_to_task
from .execution_types import AgentExecutionState, AgentProgressState, ExecutionStage
from .return_to_patrol_runtime import _build_local_patrol_return_transition


@dataclass(frozen=True)
class TaskClaimDecision:
    """Execution-side view of which task, if any, is currently claimable."""

    active_task: TaskRecord | None
    execution_state: AgentExecutionState
    pending_assigned_task_id: str | None
    claimed_task_id: str | None
    claim_transition_reason: str | None


def find_task_by_id(
    task_records: tuple[TaskRecord, ...],
    task_id: str | None,
) -> TaskRecord | None:
    """Return one task record by id, if present."""

    if task_id is None:
        return None
    for task in task_records:
        if task.task_id == task_id:
            return task
    return None


def select_claimable_task_for_agent(
    task_records: tuple[TaskRecord, ...],
    *,
    agent_id: str,
    execution_state: AgentExecutionState | None = None,
) -> TaskRecord | None:
    """Choose the single task the execution layer should currently honor."""

    claimable = [
        task
        for task in task_records
        if task.assigned_agent_id == agent_id
        and task.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
    ]
    if not claimable:
        return None

    active_task_id = None if execution_state is None else execution_state.active_task_id
    claimable.sort(
        key=lambda task: (
            0 if task.task_id == active_task_id else 1,
            0 if task.status == TaskStatus.IN_PROGRESS else 1,
            -task.priority,
            task.created_step,
            task.task_id,
        )
    )
    return claimable[0]


def claim_task_for_execution(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    task_records: tuple[TaskRecord, ...],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    grid_map: GridMap,
    info_map: InformationMap | None,
) -> TaskClaimDecision:
    """Project tasking assignments into a single execution-side claim decision."""

    active_task = select_claimable_task_for_agent(
        task_records,
        agent_id=agent.agent_id,
        execution_state=execution_state,
    )
    effective_stage = execution_state.pre_yield_stage or execution_state.stage
    next_execution_state = execution_state
    claim_transition_reason: str | None = None

    if (
        effective_stage in {ExecutionStage.PATROL, ExecutionStage.RETURN_TO_PATROL}
        and active_task is not None
    ):
        if active_task.task_type == TaskType.UAV_RESUPPLY:
            next_execution_state = transition_to_rendezvous(
                execution_state,
                task_id=active_task.task_id,
            )
        else:
            next_execution_state = transition_to_task(
                execution_state,
                task_id=active_task.task_id,
            )
        claim_transition_reason = "accepted_assigned_task"
    elif (
        effective_stage
        in {
            ExecutionStage.GO_TO_TASK,
            ExecutionStage.ON_TASK,
            ExecutionStage.GO_TO_RENDEZVOUS,
            ExecutionStage.ON_RECHARGE,
        }
        and active_task is None
    ):
        next_execution_state = _build_local_patrol_return_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_routes[agent.agent_id],
            grid_map=grid_map,
            info_map=info_map,
            task_records=task_records,
        )
        claim_transition_reason = "released_to_return_to_patrol"
    elif active_task is not None:
        if execution_state.active_task_id == active_task.task_id:
            claim_transition_reason = "continue_claimed_task"
        else:
            claim_transition_reason = "pending_assigned_task"

    claimed_task_id = None
    if active_task is not None and next_execution_state.active_task_id == active_task.task_id:
        claimed_task_id = active_task.task_id

    return TaskClaimDecision(
        active_task=active_task,
        execution_state=next_execution_state,
        pending_assigned_task_id=None if active_task is None else active_task.task_id,
        claimed_task_id=claimed_task_id,
        claim_transition_reason=claim_transition_reason,
    )
