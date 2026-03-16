"""Cost-aware centralized task allocation for comparative experiments."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus

from .allocator_common import (
    allocate_uav_resupply_task,
    can_keep_existing_assignment,
    is_available_for_new_assignment,
    preferred_usv_ids_for_task,
    selection_score_for_task,
    task_sort_key,
)
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

HOTSPOT_COST_BIAS = 20.0
BASELINE_AGE_BONUS_PER_STEP = 0.5
MAX_BASELINE_AGE_BONUS = 12.0


def allocate_tasks_with_cost_aware_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None = None,
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Apply the first cost-aware centralized task-allocation policy."""

    scheduled_tasks = sorted(tasks, key=task_sort_key)
    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    reserved_agent_ids: set[str] = set()
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}

    pending_hotspots: list[TaskRecord] = []
    pending_baselines: list[TaskRecord] = []

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
            if selected_agent.kind == "USV" and task.assigned_agent_id is not None:
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

        if task.task_type == TaskType.HOTSPOT_CONFIRMATION:
            pending_hotspots.append(task)
        else:
            pending_baselines.append(task)

    for task_group in (pending_hotspots, pending_baselines):
        assigned_tasks, group_decisions, newly_reserved = _allocate_one_priority_layer(
            task_group,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            reserved_agent_ids=reserved_agent_ids,
            reachability_cache=reachability_cache,
        )
        updated_tasks.extend(assigned_tasks)
        decisions.extend(group_decisions)
        reserved_agent_ids.update(newly_reserved)

    return (tuple(sorted(updated_tasks, key=lambda item: item.task_id)), tuple(decisions))


def _allocate_one_priority_layer(
    tasks: list[TaskRecord],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None,
    reserved_agent_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
) -> tuple[list[TaskRecord], list[TaskAssignment], set[str]]:
    if not tasks:
        return ([], [], set())

    remaining_tasks = {task.task_id: task for task in tasks}
    available_agents = {
        agent.agent_id: agent
        for agent in agents
        if agent.kind == "USV"
        and agent.agent_id not in reserved_agent_ids
        and is_available_for_new_assignment(execution_states.get(agent.agent_id))
    }
    assigned_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    layer_reserved: set[str] = set()
    age_bonuses = _baseline_age_bonuses(tasks)

    while remaining_tasks and available_agents:
        candidate_pairs = _build_candidate_pairs(
            tuple(remaining_tasks.values()),
            candidate_agents=tuple(available_agents.values()),
            grid_map=grid_map,
            reachability_cache=reachability_cache,
            age_bonuses=age_bonuses,
        )
        if not candidate_pairs:
            break

        selected_task, selected_agent, selected_score = min(
            candidate_pairs,
            key=lambda item: (item[2], item[0].created_step, item[0].task_id, item[1].agent_id),
        )
        assigned_tasks.append(
            replace(
                selected_task,
                status=TaskStatus.ASSIGNED,
                assigned_agent_id=selected_agent.agent_id,
            )
        )
        decisions.append(
            TaskAssignment(
                task_id=selected_task.task_id,
                task_type=selected_task.task_type,
                agent_id=selected_agent.agent_id,
                support_agent_id=None,
                selection_reason=_selection_reason_for_task(selected_task),
                selection_score=selected_score,
            )
        )
        layer_reserved.add(selected_agent.agent_id)
        remaining_tasks.pop(selected_task.task_id)
        available_agents.pop(selected_agent.agent_id)

    for task in remaining_tasks.values():
        assigned_tasks.append(_mark_task_unassigned(task))

    return (assigned_tasks, decisions, layer_reserved)


def _build_candidate_pairs(
    tasks: tuple[TaskRecord, ...],
    *,
    candidate_agents: tuple[AgentState, ...],
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    age_bonuses: dict[str, float],
) -> list[tuple[TaskRecord, AgentState, float]]:
    pairs: list[tuple[TaskRecord, AgentState, float]] = []

    for task in tasks:
        preferred_ids = preferred_usv_ids_for_task(task)
        for agent in candidate_agents:
            if agent.agent_id not in preferred_ids:
                continue
            estimated_cost = _reachable_cost(
                task,
                agent=agent,
                grid_map=grid_map,
                reachability_cache=reachability_cache,
            )
            if estimated_cost is None:
                continue
            pairs.append(
                (
                    task,
                    agent,
                    _total_cost_for_task(
                        task,
                        estimated_cost=estimated_cost,
                        age_bonuses=age_bonuses,
                    ),
                )
            )
    return pairs


def _reachable_cost(
    task: TaskRecord,
    *,
    agent: AgentState,
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
) -> float | None:
    if grid_map is None:
        return round(((agent.x - task.target_x) ** 2 + (agent.y - task.target_y) ** 2) ** 0.5, 3)

    cache_key = (
        agent.agent_id,
        task.task_id,
        round(task.target_x, 3),
        round(task.target_y, 3),
    )
    cached = reachability_cache.get(cache_key)
    if cached is not None:
        is_reachable, cost = cached
        return cost if is_reachable else None

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
        return None

    reachability_cache[cache_key] = (True, plan.estimated_cost)
    return plan.estimated_cost


def _total_cost_for_task(
    task: TaskRecord,
    *,
    estimated_cost: float,
    age_bonuses: dict[str, float],
) -> float:
    if task.task_type == TaskType.HOTSPOT_CONFIRMATION:
        return round(estimated_cost - HOTSPOT_COST_BIAS, 3)
    return round(estimated_cost - age_bonuses.get(task.task_id, 0.0), 3)


def _baseline_age_bonuses(tasks: list[TaskRecord]) -> dict[str, float]:
    baseline_tasks = [task for task in tasks if task.task_type == TaskType.BASELINE_SERVICE]
    if not baseline_tasks:
        return {}

    newest_created_step = max(task.created_step for task in baseline_tasks)
    return {
        task.task_id: min(
            (newest_created_step - task.created_step) * BASELINE_AGE_BONUS_PER_STEP,
            MAX_BASELINE_AGE_BONUS,
        )
        for task in baseline_tasks
    }


def _selection_reason_for_task(task: TaskRecord) -> str:
    if task.task_type == TaskType.BASELINE_SERVICE:
        return "lowest_cost_partition_usv_for_baseline_service"
    return "lowest_cost_partition_usv_for_hotspot_confirmation"


def _mark_task_unassigned(task: TaskRecord) -> TaskRecord:
    return replace(
        task,
        status=TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING,
        assigned_agent_id=None,
    )
