"""Cost-aware centralized task allocation for comparative experiments."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.agent_model import AgentState, is_operational_agent
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan

from .allocator_common import (
    allocate_hotspot_proximity_override_task,
    allocate_uav_resupply_task,
    can_keep_existing_assignment,
    is_available_for_candidate_pool,
    is_available_for_new_assignment,
    release_preempted_uav_resupply_tasks,
    selection_score_for_task,
    stabilize_uav_resupply_task,
    task_sort_key,
)
from .partitioning import build_task_partition
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

HOTSPOT_COST_BIAS = 20.0
BASELINE_AGE_BONUS_PER_STEP = 0.5
MAX_BASELINE_AGE_BONUS = 12.0
AGENT_TASK_BLOCKED_COOLDOWN_STEPS = 12
UNREACHABLE_TASK_COOLDOWN_STEPS = AGENT_TASK_BLOCKED_COOLDOWN_STEPS


def allocate_tasks_with_cost_aware_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None = None,
    step: int | None = None,
    usv_path_planner: str = "astar_path_planner",
    zone_partition_policy: str = "baseline_fixed_partition",
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
        task = _prune_expired_agent_retries(task, step)
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            updated_tasks.append(task)
            continue

        proximity_override = allocate_hotspot_proximity_override_task(
            task,
            agents=agents,
            execution_states=execution_states,
            task_records=scheduled_tasks,
            reserved_agent_ids=reserved_agent_ids,
            step=step,
        )
        if proximity_override is not None:
            updated_task, decision = proximity_override
            updated_tasks, decisions, preempted_task_ids = release_preempted_uav_resupply_tasks(
                updated_tasks,
                decisions,
                support_agent_id=decision.agent_id,
            )
            if preempted_task_ids:
                decision = replace(
                    decision,
                    selection_details={
                        **(decision.selection_details or {}),
                        "preempted_support_task_ids": list(preempted_task_ids),
                    },
                )
            updated_tasks.append(updated_task)
            reserved_agent_ids.add(decision.agent_id)
            decisions.append(decision)
            continue

        if can_keep_existing_assignment(
            task, agent_by_id=agent_by_id, execution_states=execution_states
        ):
            selected_agent = agent_by_id[task.assigned_agent_id or ""]
            if task.task_type == TaskType.UAV_RESUPPLY:
                task = stabilize_uav_resupply_task(task, selected_uav=selected_agent)
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
                execution_states=execution_states,
                grid_map=grid_map,
                task_records=tuple(updated_tasks),
                usv_path_planner=usv_path_planner,
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
            task_records=tuple(updated_tasks),
            reserved_agent_ids=reserved_agent_ids,
            reachability_cache=reachability_cache,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
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
    task_records: tuple[TaskRecord, ...],
    reserved_agent_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    step: int | None,
    usv_path_planner: str,
    zone_partition_policy: str,
) -> tuple[list[TaskRecord], list[TaskAssignment], set[str]]:
    if not tasks:
        return ([], [], set())

    remaining_tasks = {task.task_id: task for task in tasks}
    available_agents = {
        agent.agent_id: agent
        for agent in agents
        if agent.kind == "USV"
        and is_operational_agent(agent)
        and agent.agent_id not in reserved_agent_ids
        and is_available_for_candidate_pool(
            execution_states.get(agent.agent_id),
            agent_id=agent.agent_id,
            task_records=task_records,
        )
    }
    assigned_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    layer_reserved: set[str] = set()
    age_bonuses = _baseline_age_bonuses(tasks)
    current_step = max((task.created_step for task in tasks), default=0) if step is None else step
    blocked_agent_ids_by_task: dict[str, set[str]] = {}

    while remaining_tasks and available_agents:
        candidate_pairs, newly_blocked_agent_ids_by_task = _build_candidate_pairs(
            tuple(remaining_tasks.values()),
            candidate_agents=tuple(available_agents.values()),
            grid_map=grid_map,
            reachability_cache=reachability_cache,
            age_bonuses=age_bonuses,
            step=current_step,
            usv_path_planner=usv_path_planner,
            all_agents=agents,
            execution_states=execution_states,
            zone_partition_policy=zone_partition_policy,
        )
        for task_id, agent_ids in newly_blocked_agent_ids_by_task.items():
            blocked_agent_ids_by_task.setdefault(task_id, set()).update(agent_ids)
        if not candidate_pairs:
            break

        selected_task, selected_agent, selected_score, selected_partition_reason = min(
            candidate_pairs,
            key=lambda item: (item[2], item[0].created_step, item[0].task_id, item[1].agent_id),
        )
        assigned_tasks.append(
            _mark_task_assigned(selected_task, selected_agent_id=selected_agent.agent_id)
        )
        decisions.append(
            TaskAssignment(
                task_id=selected_task.task_id,
                task_type=selected_task.task_type,
                agent_id=selected_agent.agent_id,
                support_agent_id=None,
                selection_reason=_selection_reason_for_task(selected_task),
                selection_score=selected_score,
                selection_details={"partition_reason": selected_partition_reason},
            )
        )
        layer_reserved.add(selected_agent.agent_id)
        remaining_tasks.pop(selected_task.task_id)
        available_agents.pop(selected_agent.agent_id)

    for task in remaining_tasks.values():
        assigned_tasks.append(
            _mark_task_unassigned(
                task,
                step=current_step,
                blocked_agent_ids=blocked_agent_ids_by_task.get(task.task_id, set()),
            )
        )

    return (assigned_tasks, decisions, layer_reserved)


def _build_candidate_pairs(
    tasks: tuple[TaskRecord, ...],
    *,
    candidate_agents: tuple[AgentState, ...],
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    age_bonuses: dict[str, float],
    step: int,
    usv_path_planner: str,
    all_agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    zone_partition_policy: str,
) -> tuple[list[tuple[TaskRecord, AgentState, float, str]], dict[str, set[str]]]:
    pairs: list[tuple[TaskRecord, AgentState, float, str]] = []
    blocked_agent_ids_by_task: dict[str, set[str]] = {}

    for task in tasks:
        partition = build_task_partition(
            task,
            policy_name=zone_partition_policy,
            tasks=tasks,
            agents=all_agents,
            execution_states=execution_states,
            step=step,
        )
        preferred_ids = set(partition.primary_usv_ids) | set(partition.secondary_usv_ids)
        for agent in candidate_agents:
            if agent.agent_id not in preferred_ids:
                continue
            if not is_available_for_new_assignment(
                execution_states.get(agent.agent_id),
                agent=agent,
                task=task,
                agent_id=agent.agent_id,
                task_records=tasks,
            ):
                continue
            if _is_agent_task_in_cooldown(task, agent_id=agent.agent_id, step=step):
                continue
            estimated_cost = _reachable_cost(
                task,
                agent=agent,
                grid_map=grid_map,
                reachability_cache=reachability_cache,
                usv_path_planner=usv_path_planner,
            )
            if estimated_cost is None:
                blocked_agent_ids_by_task.setdefault(task.task_id, set()).add(agent.agent_id)
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
                    partition.partition_reason,
                )
            )
    return (pairs, blocked_agent_ids_by_task)


def _reachable_cost(
    task: TaskRecord,
    *,
    agent: AgentState,
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    usv_path_planner: str = "astar_path_planner",
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


def _is_agent_task_in_cooldown(task: TaskRecord, *, agent_id: str, step: int) -> bool:
    return any(
        candidate_agent_id == agent_id and step < retry_after_step
        for candidate_agent_id, retry_after_step in task.agent_retry_after_steps
    )


def _prune_expired_agent_retries(task: TaskRecord, step: int | None) -> TaskRecord:
    if step is None or not task.agent_retry_after_steps:
        return task
    active_retries = tuple(
        (agent_id, retry_after_step)
        for agent_id, retry_after_step in task.agent_retry_after_steps
        if step < retry_after_step
    )
    if active_retries == task.agent_retry_after_steps:
        return task
    return replace(
        task,
        retry_after_step=max(
            (retry_after_step for _, retry_after_step in active_retries),
            default=None,
        ),
        agent_retry_after_steps=active_retries,
    )


def _mark_task_assigned(task: TaskRecord, *, selected_agent_id: str) -> TaskRecord:
    retained_retries = tuple(
        (agent_id, retry_after_step)
        for agent_id, retry_after_step in task.agent_retry_after_steps
        if agent_id != selected_agent_id
    )
    return replace(
        task,
        status=TaskStatus.ASSIGNED,
        assigned_agent_id=selected_agent_id,
        retry_after_step=max(
            (retry_after_step for _, retry_after_step in retained_retries),
            default=None,
        ),
        agent_retry_after_steps=retained_retries,
    )


def _mark_task_unassigned(
    task: TaskRecord,
    *,
    step: int,
    blocked_agent_ids: set[str],
) -> TaskRecord:
    retry_by_agent = {
        agent_id: retry_after for agent_id, retry_after in task.agent_retry_after_steps
    }
    for agent_id in blocked_agent_ids:
        retry_by_agent[agent_id] = max(
            step + AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
            retry_by_agent.get(agent_id, 0),
        )
    agent_retry_after_steps = tuple(sorted(retry_by_agent.items()))
    return replace(
        task,
        status=TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING,
        assigned_agent_id=None,
        retry_after_step=max(
            (retry_after for _, retry_after in agent_retry_after_steps),
            default=None,
        ),
        agent_retry_after_steps=agent_retry_after_steps,
    )
