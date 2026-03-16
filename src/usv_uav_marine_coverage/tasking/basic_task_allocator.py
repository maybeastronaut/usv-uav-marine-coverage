"""Basic task allocation for the current tasking layer."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan

from .allocator_common import (
    allocate_uav_resupply_task,
    can_keep_existing_assignment,
    distance_to_task,
    is_available_for_new_assignment,
    preferred_usv_ids_for_task,
    selection_score_for_task,
    task_sort_key,
)
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType


def allocate_tasks_with_basic_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None = None,
    usv_path_planner: str = "astar_path_planner",
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Apply the first basic task-allocation policy for the current simulation."""

    scheduled_tasks = sorted(tasks, key=task_sort_key)
    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    reserved_agent_ids: set[str] = set()
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}

    for task in scheduled_tasks:
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            updated_tasks.append(task)
            continue

        if can_keep_existing_assignment(
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
            selection_score = selection_score_for_task(
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
            updated_task, decision = allocate_uav_resupply_task(
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
            and is_available_for_new_assignment(execution_states.get(agent.agent_id))
        ]
        candidates = [
            agent for agent in candidates if agent.agent_id in preferred_usv_ids_for_task(task)
        ]
        if not candidates:
            updated_tasks.append(_mark_task_unassigned(task))
            continue

        reachable_candidates = _reachable_usv_candidates(
            task,
            candidates=candidates,
            grid_map=grid_map,
            reachability_cache=reachability_cache,
            usv_path_planner=usv_path_planner,
        )
        if not reachable_candidates:
            updated_tasks.append(_mark_task_unassigned(task))
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


def _selection_reason_for_task(task: TaskRecord) -> str:
    if task.task_type == TaskType.BASELINE_SERVICE:
        return "lowest_cost_reachable_partition_usv_for_baseline_service"
    return "lowest_cost_reachable_partition_usv_for_hotspot_confirmation"


def _reachable_usv_candidates(
    task: TaskRecord,
    *,
    candidates: list[AgentState],
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    usv_path_planner: str,
) -> list[tuple[AgentState, float]]:
    if grid_map is None:
        return [(agent, distance_to_task(agent, task)) for agent in candidates]

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
        plan = build_usv_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=task.target_x,
            goal_y=task.target_y,
            planner_name=usv_path_planner,
            task_id=task.task_id,
            stats_context="allocator_reachability",
        )
        if plan.status != PathPlanStatus.PLANNED:
            reachability_cache[cache_key] = (False, float("inf"))
            continue
        reachability_cache[cache_key] = (True, plan.estimated_cost)
        reachable.append((agent, plan.estimated_cost))
    return reachable


def _mark_task_unassigned(task: TaskRecord) -> TaskRecord:
    return replace(
        task,
        status=TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING,
        assigned_agent_id=None,
    )
