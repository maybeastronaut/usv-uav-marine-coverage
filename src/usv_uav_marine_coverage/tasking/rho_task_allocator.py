"""Rolling-horizon centralized task allocation for comparative experiments."""

from __future__ import annotations

from dataclasses import dataclass, replace

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    distance_to_point,
    energy_ratio,
    is_operational_agent,
    needs_uav_resupply,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan

from .allocator_common import (
    allocate_hotspot_proximity_override_task,
    allocate_uav_resupply_task,
    can_keep_existing_assignment,
    is_agent_task_in_cooldown,
    is_task_waiting_for_retry,
    is_available_for_candidate_pool,
    is_available_for_new_assignment,
    release_preempted_uav_resupply_tasks,
    selection_score_for_task,
    stabilize_uav_resupply_task,
    task_sort_key,
)
from .partitioning import build_task_partition
from .partitioning.failure_hotspot_soft_partition import (
    failed_neighbor_recovery_bonus,
    failure_hotspot_emergency_active,
    should_suppress_baseline_after_failure,
    should_suppress_non_focus_hotspot_after_failure,
)
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

HOTSPOT_BASE_VALUE = 84.0
BASELINE_BASE_VALUE = 44.0
AOI_REWARD_PER_STEP = 0.42
MAX_AOI_REWARD = 48.0
PATH_COST_WEIGHT = 0.82
RHO_WINDOW_STEPS = 180
DELAY_PENALTY_PER_STEP = 0.18
MAX_DELAY_PENALTY = 26.0
LOW_ENERGY_SUPPORT_RATIO = 0.35
LOW_ENERGY_SUPPORT_PENALTY = 18.0
AGENT_TASK_BLOCKED_COOLDOWN_STEPS = 12


@dataclass(frozen=True)
class RhoCandidate:
    """One rolling-horizon score estimate for one task-agent pair."""

    task: TaskRecord
    agent: AgentState
    score: float
    base_value: float
    aoi_reward: float
    path_cost: float
    delay_penalty: float
    energy_penalty: float
    failure_recovery_bonus: float
    partition_reason: str


def allocate_tasks_with_rho_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None = None,
    info_map: InformationMap | None = None,
    step: int | None = None,
    usv_path_planner: str = "astar_path_planner",
    zone_partition_policy: str = "baseline_fixed_partition",
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Apply one minimal rolling-horizon task-allocation policy."""

    scheduled_tasks = sorted(tasks, key=task_sort_key)
    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    reserved_agent_ids: set[str] = set()
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}
    protected_support_usv_ids = _protected_support_usv_ids(agents)

    pending_hotspots: list[TaskRecord] = []
    pending_baselines: list[TaskRecord] = []
    released_task_ids: set[str] = set()
    force_available_agent_ids: set[str] = set()

    for task in scheduled_tasks:
        task = _prune_expired_agent_retries(task, step)
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            updated_tasks.append(task)
            continue
        if is_task_waiting_for_retry(task, step=step) and (
            task.status in {TaskStatus.PENDING, TaskStatus.REQUEUED}
            or (task.task_type == TaskType.UAV_RESUPPLY and task.support_agent_id is None)
        ):
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

        if zone_partition_policy == "failure_triggered_hotspot_first_soft_partition_policy" and (
            should_suppress_baseline_after_failure(
                task,
                tasks=scheduled_tasks,
                agents=agents,
            )
            or should_suppress_non_focus_hotspot_after_failure(
                task,
                tasks=scheduled_tasks,
                agents=agents,
                step=step,
            )
        ):
            if task.assigned_agent_id is not None:
                released_task_ids.add(task.task_id)
                force_available_agent_ids.add(task.assigned_agent_id)
                task = _requeue_suppressed_task(
                    task,
                    step=step,
                    released_agent_id=task.assigned_agent_id,
                )
            else:
                task = replace(
                    task,
                    status=TaskStatus.PENDING,
                    assigned_agent_id=None,
                )

        if can_keep_existing_assignment(
            task,
            agent_by_id=agent_by_id,
            execution_states=execution_states,
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
                step=step,
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
            grid_map=grid_map,
            info_map=info_map,
            task_records=tuple(updated_tasks),
            reserved_agent_ids=reserved_agent_ids,
            protected_support_usv_ids=protected_support_usv_ids,
            reachability_cache=reachability_cache,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            all_agents=agents,
            execution_states=execution_states,
            force_available_agent_ids=force_available_agent_ids,
            released_task_ids=released_task_ids,
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
    info_map: InformationMap | None,
    task_records: tuple[TaskRecord, ...],
    reserved_agent_ids: set[str],
    protected_support_usv_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    step: int | None,
    usv_path_planner: str,
    zone_partition_policy: str,
    all_agents: tuple[AgentState, ...],
    force_available_agent_ids: set[str],
    released_task_ids: set[str],
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
        and (
            is_available_for_candidate_pool(
                execution_states.get(agent.agent_id),
                agent_id=agent.agent_id,
                task_records=task_records,
            )
            or _is_force_available_after_emergency_release(
                execution_states.get(agent.agent_id),
                agent_id=agent.agent_id,
                force_available_agent_ids=force_available_agent_ids,
                released_task_ids=released_task_ids,
            )
        )
    }
    assigned_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    layer_reserved: set[str] = set()
    blocked_agent_ids_by_task: dict[str, set[str]] = {}
    current_step = max((task.created_step for task in tasks), default=0) if step is None else step

    while remaining_tasks and available_agents:
        candidate_pairs, newly_blocked = _build_candidate_pairs(
            tuple(remaining_tasks.values()),
            candidate_agents=tuple(available_agents.values()),
            task_context=(
                tuple(task_records) + tuple(assigned_tasks) + tuple(remaining_tasks.values())
            ),
            grid_map=grid_map,
            info_map=info_map,
            protected_support_usv_ids=protected_support_usv_ids,
            reachability_cache=reachability_cache,
            step=current_step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            all_agents=all_agents,
            execution_states=execution_states,
            force_available_agent_ids=force_available_agent_ids,
            released_task_ids=released_task_ids,
        )
        for task_id, agent_ids in newly_blocked.items():
            blocked_agent_ids_by_task.setdefault(task_id, set()).update(agent_ids)
        if not candidate_pairs:
            break

        selected_candidate = max(
            candidate_pairs,
            key=lambda item: (
                item.score,
                -item.task.created_step,
                item.task.task_id,
                item.agent.agent_id,
            ),
        )
        selected_task = selected_candidate.task
        selected_agent = selected_candidate.agent
        assigned_tasks.append(
            _mark_task_assigned(selected_task, selected_agent_id=selected_agent.agent_id)
        )
        candidate_scores = tuple(
            sorted(
                (
                    {
                        "agent_id": candidate.agent.agent_id,
                        "support_agent_id": None,
                        "bid_score": round(candidate.score, 3),
                        "base_value": round(candidate.base_value, 3),
                        "aoi_reward": round(candidate.aoi_reward, 3),
                        "path_cost": round(candidate.path_cost, 3),
                        "delay_penalty": round(candidate.delay_penalty, 3),
                        "energy_penalty": round(candidate.energy_penalty, 3),
                        "failure_recovery_bonus": round(candidate.failure_recovery_bonus, 3),
                        "partition_reason": candidate.partition_reason,
                    }
                    for candidate in candidate_pairs
                    if candidate.task.task_id == selected_task.task_id
                ),
                key=lambda item: (-float(item["bid_score"]), str(item["agent_id"])),
            )
        )
        decisions.append(
            TaskAssignment(
                task_id=selected_task.task_id,
                task_type=selected_task.task_type,
                agent_id=selected_agent.agent_id,
                support_agent_id=None,
                selection_reason=_selection_reason_for_task(selected_task),
                selection_score=selected_candidate.score,
                selection_details={
                    "base_value": round(selected_candidate.base_value, 3),
                    "aoi_reward": round(selected_candidate.aoi_reward, 3),
                    "path_cost": round(selected_candidate.path_cost, 3),
                    "delay_penalty": round(selected_candidate.delay_penalty, 3),
                    "energy_penalty": round(selected_candidate.energy_penalty, 3),
                    "failure_recovery_bonus": round(
                        selected_candidate.failure_recovery_bonus,
                        3,
                    ),
                    "partition_reason": selected_candidate.partition_reason,
                    "final_bid": round(selected_candidate.score, 3),
                },
                candidate_agents=candidate_scores,
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
    task_context: tuple[TaskRecord, ...],
    grid_map: GridMap | None,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    all_agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    force_available_agent_ids: set[str],
    released_task_ids: set[str],
) -> tuple[list[RhoCandidate], dict[str, set[str]]]:
    pairs: list[RhoCandidate] = []
    blocked_agent_ids_by_task: dict[str, set[str]] = {}

    for task in tasks:
        partition = build_task_partition(
            task,
            policy_name=zone_partition_policy,
            tasks=_partition_context_tasks(
                tasks=task_context,
                all_agents=all_agents,
                execution_states=execution_states,
                force_available_agent_ids=force_available_agent_ids,
                released_task_ids=released_task_ids,
            ),
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
                task_records=task_context,
            ):
                continue
            if is_agent_task_in_cooldown(task, agent_id=agent.agent_id, step=step):
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
                _build_rho_candidate(
                    task,
                    agent=agent,
                    estimated_cost=estimated_cost,
                    info_map=info_map,
                    protected_support_usv_ids=protected_support_usv_ids,
                    failure_recovery_bonus=failed_neighbor_recovery_bonus(
                        task,
                        agents=all_agents,
                    ),
                    partition_reason=partition.partition_reason,
                )
            )
    return (pairs, blocked_agent_ids_by_task)


def _build_rho_candidate(
    task: TaskRecord,
    *,
    agent: AgentState,
    estimated_cost: float,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
    failure_recovery_bonus: float,
    partition_reason: str,
) -> RhoCandidate:
    base_value = (
        HOTSPOT_BASE_VALUE
        if task.task_type == TaskType.HOTSPOT_CONFIRMATION
        else BASELINE_BASE_VALUE
    )
    aoi_reward = _aoi_reward_for_task(task, info_map=info_map)
    delay_penalty = _delay_penalty_for_candidate(agent, estimated_cost=estimated_cost)
    energy_penalty = (
        LOW_ENERGY_SUPPORT_PENALTY if agent.agent_id in protected_support_usv_ids else 0.0
    )
    score = round(
        base_value
        + aoi_reward
        + failure_recovery_bonus
        - (estimated_cost * PATH_COST_WEIGHT)
        - delay_penalty
        - energy_penalty,
        3,
    )
    return RhoCandidate(
        task=task,
        agent=agent,
        score=score,
        base_value=base_value,
        aoi_reward=round(aoi_reward, 3),
        path_cost=round(estimated_cost, 3),
        delay_penalty=round(delay_penalty, 3),
        energy_penalty=energy_penalty,
        failure_recovery_bonus=round(failure_recovery_bonus, 3),
        partition_reason=partition_reason,
    )


def _partition_context_tasks(
    *,
    tasks: tuple[TaskRecord, ...],
    all_agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    force_available_agent_ids: set[str],
    released_task_ids: set[str],
) -> tuple[TaskRecord, ...]:
    if not failure_hotspot_emergency_active(tasks=tasks, agents=all_agents):
        return tasks
    if not released_task_ids:
        return tasks

    normalized: list[TaskRecord] = []
    for task in tasks:
        if task.task_id not in released_task_ids:
            normalized.append(task)
            continue
        normalized.append(
            replace(
                task,
                status=TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING,
                assigned_agent_id=None,
            )
        )
    return tuple(normalized)


def _is_force_available_after_emergency_release(
    execution_state: AgentExecutionState | None,
    *,
    agent_id: str,
    force_available_agent_ids: set[str],
    released_task_ids: set[str],
) -> bool:
    if agent_id not in force_available_agent_ids:
        return False
    if execution_state is None:
        return False
    return execution_state.active_task_id in released_task_ids


def _aoi_reward_for_task(task: TaskRecord, *, info_map: InformationMap | None) -> float:
    if info_map is None or task.target_row is None or task.target_col is None:
        return 0.0
    state = info_map.state_at(task.target_row, task.target_col)
    return min(state.information_age * AOI_REWARD_PER_STEP, MAX_AOI_REWARD)


def _delay_penalty_for_candidate(agent: AgentState, *, estimated_cost: float) -> float:
    nominal_speed = max(agent.cruise_speed_mps, 1.0)
    arrival_steps = min(estimated_cost / nominal_speed, float(RHO_WINDOW_STEPS))
    return min(arrival_steps * DELAY_PENALTY_PER_STEP, MAX_DELAY_PENALTY)


def _protected_support_usv_ids(agents: tuple[AgentState, ...]) -> set[str]:
    usvs = tuple(agent for agent in agents if agent.kind == "USV")
    if not usvs:
        return set()

    protected: set[str] = set()
    for agent in agents:
        if agent.kind != "UAV":
            continue
        if not needs_uav_resupply(agent) and energy_ratio(agent) > LOW_ENERGY_SUPPORT_RATIO:
            continue
        nearest_usv = min(usvs, key=lambda usv: distance_to_point(agent, usv.x, usv.y))
        protected.add(nearest_usv.agent_id)
    return protected


def _selection_reason_for_task(task: TaskRecord) -> str:
    if task.task_type == TaskType.BASELINE_SERVICE:
        return "highest_rho_window_score_partition_usv_for_baseline_service"
    return "highest_rho_window_score_partition_usv_for_hotspot_confirmation"


def _reachable_cost(
    task: TaskRecord,
    *,
    agent: AgentState,
    grid_map: GridMap | None,
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    usv_path_planner: str,
) -> float | None:
    if grid_map is None:
        return round(distance_to_point(agent, task.target_x, task.target_y), 3)

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
    next_status = task.status
    if next_status != TaskStatus.REQUEUED:
        next_status = TaskStatus.REQUEUED if task.assigned_agent_id else TaskStatus.PENDING
    return replace(
        task,
        status=next_status,
        assigned_agent_id=None,
        suppression_grace_until_step=None,
        retry_after_step=max(
            (retry_after for _, retry_after in agent_retry_after_steps),
            default=None,
        ),
        agent_retry_after_steps=agent_retry_after_steps,
    )


def _requeue_suppressed_task(
    task: TaskRecord,
    *,
    step: int | None,
    released_agent_id: str,
) -> TaskRecord:
    """Release one failure-suppressed task without immediately handing it back."""

    if step is None:
        return replace(
            task,
            status=TaskStatus.REQUEUED,
            assigned_agent_id=None,
            suppression_grace_until_step=None,
        )
    retry_by_agent = {
        agent_id: retry_after for agent_id, retry_after in task.agent_retry_after_steps
    }
    retry_by_agent[released_agent_id] = max(
        step + AGENT_TASK_BLOCKED_COOLDOWN_STEPS,
        retry_by_agent.get(released_agent_id, 0),
    )
    return replace(
        task,
        status=TaskStatus.REQUEUED,
        assigned_agent_id=None,
        suppression_grace_until_step=None,
        retry_after_step=task.retry_after_step,
        agent_retry_after_steps=tuple(sorted(retry_by_agent.items())),
    )
