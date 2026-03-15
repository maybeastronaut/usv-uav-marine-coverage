"""Basic task allocation for the current tasking layer."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus

from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

NEARSHORE_X_END_M = 250.0
OFFSHORE_Y_SPLIT_M = 500.0


def allocate_tasks_with_basic_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None = None,
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Apply the first basic task-allocation policy for the current simulation."""

    scheduled_tasks = sorted(tasks, key=_task_sort_key)
    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    reserved_agent_ids: set[str] = set()
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}

    for task in scheduled_tasks:
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            updated_tasks.append(task)
            continue

        if _can_keep_existing_assignment(
            task, agent_by_id=agent_by_id, execution_states=execution_states
        ):
            selected_agent = agent_by_id[task.assigned_agent_id or ""]
            if task.task_type == TaskType.UAV_RESUPPLY and task.support_agent_id is not None:
                support_agent = agent_by_id.get(task.support_agent_id)
                if support_agent is not None:
                    task = replace(
                        task,
                        target_x=support_agent.x,
                        target_y=support_agent.y,
                    )
            selection_score = _selection_score_for_task(
                task,
                selected_agent=selected_agent,
                agent_by_id=agent_by_id,
            )
            updated_tasks.append(task)
            if task.assigned_agent_id is not None and selected_agent.kind == "USV":
                reserved_agent_ids.add(task.assigned_agent_id)
            decisions.append(
                TaskAssignment(
                    task_id=task.task_id,
                    task_type=task.task_type,
                    agent_id=selected_agent.agent_id,
                    support_agent_id=task.support_agent_id,
                    selection_reason="keep_existing_assignment",
                    selection_score=selection_score,
                )
            )
            continue

        if task.task_type == TaskType.UAV_RESUPPLY:
            updated_task, decision = _allocate_uav_resupply_task(
                task,
                agent_by_id=agent_by_id,
            )
            updated_tasks.append(updated_task)
            if decision is not None:
                decisions.append(decision)
            continue

        candidates = [
            agent
            for agent in agents
            if agent.kind == "USV"
            and agent.agent_id not in reserved_agent_ids
            and _is_available_for_new_assignment(execution_states.get(agent.agent_id))
        ]
        candidates = [
            agent for agent in candidates if agent.agent_id in _preferred_usv_ids_for_task(task)
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

        reachable_candidates = _reachable_usv_candidates(
            task,
            candidates=candidates,
            grid_map=grid_map,
            reachability_cache=reachability_cache,
        )
        if not reachable_candidates:
            updated_tasks.append(
                replace(
                    task,
                    status=TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING,
                    assigned_agent_id=None,
                )
            )
            continue

        selected, score = min(reachable_candidates, key=lambda item: item[1])
        updated_tasks.append(
            replace(
                task,
                status=TaskStatus.ASSIGNED,
                assigned_agent_id=selected.agent_id,
            )
        )
        reserved_agent_ids.add(selected.agent_id)
        decisions.append(
            TaskAssignment(
                task_id=task.task_id,
                task_type=task.task_type,
                agent_id=selected.agent_id,
                support_agent_id=None,
                selection_reason=_selection_reason_for_task(task),
                selection_score=score,
            )
        )

    return (tuple(sorted(updated_tasks, key=lambda item: item.task_id)), tuple(decisions))


def _task_sort_key(task: TaskRecord) -> tuple[int, int, int, str]:
    status_rank = 1 if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED} else 0
    if task.task_type == TaskType.UAV_RESUPPLY:
        type_rank = 0
    elif task.task_type == TaskType.HOTSPOT_CONFIRMATION:
        type_rank = 1
    else:
        type_rank = 2
    return (status_rank, type_rank, task.created_step, task.task_id)


def _can_keep_existing_assignment(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
    execution_states: dict[str, AgentExecutionState],
) -> bool:
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


def _is_available_for_new_assignment(execution_state: AgentExecutionState | None) -> bool:
    if execution_state is None:
        return False
    return execution_state.active_task_id is None and execution_state.stage == ExecutionStage.PATROL


def _distance_to_task(agent: AgentState, task: TaskRecord) -> float:
    return round(hypot(agent.x - task.target_x, agent.y - task.target_y), 3)


def _selection_reason_for_task(task: TaskRecord) -> str:
    if task.task_type == TaskType.BASELINE_SERVICE:
        return "lowest_cost_reachable_partition_usv_for_baseline_service"
    return "lowest_cost_reachable_partition_usv_for_hotspot_confirmation"


def _preferred_usv_ids_for_task(task: TaskRecord) -> set[str]:
    if task.task_type == TaskType.UAV_RESUPPLY:
        return {"USV-1", "USV-2", "USV-3"}
    if task.target_x < NEARSHORE_X_END_M:
        return {"USV-1"}
    if task.target_y < OFFSHORE_Y_SPLIT_M:
        return {"USV-2"}
    return {"USV-3"}


def _reachable_usv_candidates(
    task: TaskRecord,
    *,
    candidates: list[AgentState],
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
) -> list[tuple[AgentState, float]]:
    if grid_map is None:
        return [(agent, _distance_to_task(agent, task)) for agent in candidates]

    reachable: list[tuple[AgentState, float]] = []
    for agent in candidates:
        cache_key = (
            agent.agent_id,
            task.task_id,
            round(task.target_x, 3),
            round(task.target_y, 3),
        )
        cached = reachability_cache.get(cache_key)
        if cached is not None:
            is_reachable, selection_score = cached
            if is_reachable:
                reachable.append((agent, selection_score))
            continue
        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=task.target_x,
            goal_y=task.target_y,
            planner_name="astar_path_planner",
            task_id=task.task_id,
            stats_context="allocator_reachability",
        )
        if plan.status != PathPlanStatus.PLANNED:
            reachability_cache[cache_key] = (False, float("inf"))
            continue
        reachability_cache[cache_key] = (True, plan.estimated_cost)
        reachable.append((agent, plan.estimated_cost))
    return reachable


def _allocate_uav_resupply_task(
    task: TaskRecord,
    *,
    agent_by_id: dict[str, AgentState],
) -> tuple[TaskRecord, TaskAssignment | None]:
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


def _selection_score_for_task(
    task: TaskRecord,
    *,
    selected_agent: AgentState,
    agent_by_id: dict[str, AgentState],
) -> float:
    if task.task_type != TaskType.UAV_RESUPPLY or task.support_agent_id is None:
        return _distance_to_task(selected_agent, task)
    support_agent = agent_by_id.get(task.support_agent_id)
    if support_agent is None:
        return _distance_to_task(selected_agent, task)
    return round(hypot(support_agent.x - selected_agent.x, support_agent.y - selected_agent.y), 3)
