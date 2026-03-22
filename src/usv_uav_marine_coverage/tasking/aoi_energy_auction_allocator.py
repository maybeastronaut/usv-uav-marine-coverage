"""AoI-Energy auction-style centralized task allocation for comparative experiments."""

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
    is_available_for_candidate_pool,
    is_available_for_new_assignment,
    release_preempted_uav_resupply_tasks,
    selection_score_for_task,
    task_sort_key,
)
from .partitioning import (
    OFFSHORE_Y_SPLIT_M,
    TaskPartitionView,
    build_task_partition,
)
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

HOTSPOT_BASE_VALUE = 82.0
BASELINE_BASE_VALUE = 48.0
AOI_GAIN_PER_STEP = 0.26
MAX_AOI_GAIN = 36.0
LOW_ENERGY_SUPPORT_RATIO = 0.35
LOW_ENERGY_SUPPORT_PENALTY = 18.0
AGENT_TASK_BLOCKED_COOLDOWN_STEPS = 12
BASELINE_STALE_BONUS_PER_STEP = 0.14
MAX_BASELINE_STALE_BONUS = 24.0
HOTSPOT_BACKLOG_THRESHOLD = 3
HOTSPOT_BACKLOG_PENALTY_PER_TASK = 10.0
MAX_HOTSPOT_BACKLOG_PENALTY = 30.0
BASELINE_BACKLOG_GUARD_AGE_STEPS = 160.0
BASELINE_BACKLOG_GUARD_PENALTY = 90.0


@dataclass(frozen=True)
class AuctionBid:
    """One candidate bid for one task-agent pair."""

    task: TaskRecord
    agent: AgentState
    score: float
    base_value: float
    aoi_gain: float
    baseline_stale_bonus: float
    path_cost: float
    energy_penalty: float
    hotspot_backlog_penalty: float
    baseline_guard_penalty: float


@dataclass(frozen=True)
class AuctionMarketContext:
    """Lightweight market state used to bias bidding under backlog."""

    pending_hotspots: int
    pending_baselines: int
    baseline_guard_agent_ids: tuple[str, ...]
    task_records: tuple[TaskRecord, ...]
    execution_states: dict[str, AgentExecutionState]


def allocate_tasks_with_aoi_energy_auction_policy(
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
    """Apply the first AoI-Energy auction-style task-allocation policy."""

    scheduled_tasks = sorted(tasks, key=task_sort_key)
    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    reserved_agent_ids: set[str] = set()
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}
    protected_support_usv_ids = _protected_support_usv_ids(agents)

    pending_market_tasks: list[TaskRecord] = []

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
            task,
            agent_by_id=agent_by_id,
            execution_states=execution_states,
        ):
            selected_agent = agent_by_id[task.assigned_agent_id or ""]
            if task.task_type == TaskType.UAV_RESUPPLY and task.support_agent_id is not None:
                support_agent = agent_by_id.get(task.support_agent_id)
                if support_agent is not None:
                    task = replace(task, target_x=support_agent.x, target_y=support_agent.y)
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
                task_records=tuple(updated_tasks),
            )
            updated_tasks.append(updated_task)
            if decision is not None:
                decisions.append(decision)
            continue

        pending_market_tasks.append(task)

    assigned_tasks, group_decisions, newly_reserved = _allocate_one_priority_layer(
        pending_market_tasks,
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
    current_step = max((task.created_step for task in tasks), default=0) if step is None else step
    blocked_agent_ids_by_task: dict[str, set[str]] = {}

    while remaining_tasks and available_agents:
        candidate_pairs, newly_blocked_agent_ids_by_task = _build_candidate_pairs(
            tuple(remaining_tasks.values()),
            candidate_agents=tuple(available_agents.values()),
            grid_map=grid_map,
            info_map=info_map,
            protected_support_usv_ids=protected_support_usv_ids,
            reachability_cache=reachability_cache,
            step=current_step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            all_agents=all_agents,
            execution_states=execution_states,
        )
        for task_id, agent_ids in newly_blocked_agent_ids_by_task.items():
            blocked_agent_ids_by_task.setdefault(task_id, set()).update(agent_ids)
        if not candidate_pairs:
            break

        selected_bid = min(
            candidate_pairs,
            key=lambda item: (
                -item.score,
                item.task.created_step,
                item.task.task_id,
                item.agent.agent_id,
            ),
        )
        selected_task = selected_bid.task
        selected_agent = selected_bid.agent
        assigned_tasks.append(
            _mark_task_assigned(selected_task, selected_agent_id=selected_agent.agent_id)
        )
        candidate_scores = tuple(
            sorted(
                (
                    {
                        "agent_id": bid.agent.agent_id,
                        "support_agent_id": None,
                        "bid_score": round(bid.score, 3),
                        "base_value": round(bid.base_value, 3),
                        "aoi_gain": round(bid.aoi_gain, 3),
                        "baseline_stale_bonus": round(bid.baseline_stale_bonus, 3),
                        "path_cost": round(bid.path_cost, 3),
                        "energy_penalty": round(bid.energy_penalty, 3),
                        "hotspot_backlog_penalty": round(bid.hotspot_backlog_penalty, 3),
                        "baseline_guard_penalty": round(bid.baseline_guard_penalty, 3),
                    }
                    for bid in candidate_pairs
                    if bid.task.task_id == selected_task.task_id
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
                selection_score=selected_bid.score,
                selection_details={
                    "base_value": round(selected_bid.base_value, 3),
                    "aoi_gain": round(selected_bid.aoi_gain, 3),
                    "baseline_stale_bonus": round(selected_bid.baseline_stale_bonus, 3),
                    "path_cost": round(selected_bid.path_cost, 3),
                    "energy_penalty": round(selected_bid.energy_penalty, 3),
                    "hotspot_backlog_penalty": round(selected_bid.hotspot_backlog_penalty, 3),
                    "baseline_guard_penalty": round(selected_bid.baseline_guard_penalty, 3),
                    "final_bid": round(selected_bid.score, 3),
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
    grid_map: GridMap | None,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    all_agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
) -> tuple[list[AuctionBid], dict[str, set[str]]]:
    pairs: list[AuctionBid] = []
    blocked_agent_ids_by_task: dict[str, set[str]] = {}
    market_context = AuctionMarketContext(
        pending_hotspots=sum(task.task_type == TaskType.HOTSPOT_CONFIRMATION for task in tasks),
        pending_baselines=sum(task.task_type == TaskType.BASELINE_SERVICE for task in tasks),
        baseline_guard_agent_ids=_baseline_guard_agent_ids(
            tasks,
            info_map=info_map,
            step=step,
            zone_partition_policy=zone_partition_policy,
            all_agents=all_agents,
            execution_states=execution_states,
        ),
        task_records=tasks,
        execution_states=execution_states,
    )

    for task in tasks:
        partition = build_task_partition(
            task,
            policy_name=zone_partition_policy,
            tasks=tasks,
            agents=all_agents,
            execution_states=execution_states,
            step=step,
        )
        primary_ids = set(partition.primary_usv_ids)
        candidate_ids = _aea_candidate_usv_ids_for_task(task, partition=partition)
        primary_candidate_ids = tuple(
            candidate_id for candidate_id in candidate_ids if candidate_id in primary_ids
        )
        primary_pairs, primary_blocked_ids = _build_bid_group(
            task,
            candidate_ids=primary_candidate_ids,
            candidate_agents=candidate_agents,
            grid_map=grid_map,
            info_map=info_map,
            protected_support_usv_ids=protected_support_usv_ids,
            reachability_cache=reachability_cache,
            step=step,
            usv_path_planner=usv_path_planner,
            market_context=market_context,
        )
        if primary_pairs:
            pairs.extend(primary_pairs)
            continue
        if primary_blocked_ids:
            blocked_agent_ids_by_task.setdefault(task.task_id, set()).update(primary_blocked_ids)

        secondary_candidate_ids = tuple(
            candidate_id for candidate_id in candidate_ids if candidate_id not in primary_ids
        )
        secondary_pairs, secondary_blocked_ids = _build_bid_group(
            task,
            candidate_ids=secondary_candidate_ids,
            candidate_agents=candidate_agents,
            grid_map=grid_map,
            info_map=info_map,
            protected_support_usv_ids=protected_support_usv_ids,
            reachability_cache=reachability_cache,
            step=step,
            usv_path_planner=usv_path_planner,
            market_context=market_context,
        )
        pairs.extend(secondary_pairs)
        if secondary_blocked_ids:
            blocked_agent_ids_by_task.setdefault(task.task_id, set()).update(secondary_blocked_ids)
    return (pairs, blocked_agent_ids_by_task)


def _aea_candidate_usv_ids_for_task(
    task: TaskRecord,
    *,
    partition: TaskPartitionView,
) -> tuple[str, ...]:
    """Return the primary USV plus one secondary fallback candidate for AEA."""

    primary_ids = set(partition.primary_usv_ids)
    if task.task_type == TaskType.UAV_RESUPPLY:
        return tuple(sorted(primary_ids))

    primary_id = next(iter(sorted(primary_ids)))
    secondary_ids = set(partition.secondary_usv_ids)
    if secondary_ids:
        secondary_id = next(iter(sorted(secondary_ids)))
    elif primary_id == "USV-1":
        secondary_id = "USV-2" if task.target_y < OFFSHORE_Y_SPLIT_M else "USV-3"
    else:
        secondary_id = "USV-3" if primary_id == "USV-2" else "USV-2"
    return (primary_id, secondary_id)


def _build_bid_group(
    task: TaskRecord,
    *,
    candidate_ids: tuple[str, ...],
    candidate_agents: tuple[AgentState, ...],
    grid_map: GridMap | None,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    step: int,
    usv_path_planner: str,
    market_context: AuctionMarketContext,
) -> tuple[list[AuctionBid], set[str]]:
    pairs: list[AuctionBid] = []
    blocked_agent_ids: set[str] = set()
    if not candidate_ids:
        return (pairs, blocked_agent_ids)

    for agent in candidate_agents:
        if agent.agent_id not in candidate_ids:
            continue
        if not is_available_for_new_assignment(
            market_context.execution_states.get(agent.agent_id),
            agent=agent,
            task=task,
            agent_id=agent.agent_id,
            task_records=market_context.task_records,
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
            blocked_agent_ids.add(agent.agent_id)
            continue
        pairs.append(
            _build_auction_bid(
                task,
                agent=agent,
                estimated_cost=estimated_cost,
                info_map=info_map,
                protected_support_usv_ids=protected_support_usv_ids,
                market_context=market_context,
            )
        )
    return (pairs, blocked_agent_ids)


def _build_auction_bid(
    task: TaskRecord,
    *,
    agent: AgentState,
    estimated_cost: float,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
    market_context: AuctionMarketContext,
) -> AuctionBid:
    base_value = (
        HOTSPOT_BASE_VALUE
        if task.task_type == TaskType.HOTSPOT_CONFIRMATION
        else BASELINE_BASE_VALUE
    )
    aoi_gain = _aoi_gain_for_task(task, info_map=info_map)
    baseline_stale_bonus = _baseline_stale_bonus_for_task(task, info_map=info_map)
    energy_penalty = (
        LOW_ENERGY_SUPPORT_PENALTY if agent.agent_id in protected_support_usv_ids else 0.0
    )
    hotspot_backlog_penalty = _hotspot_backlog_penalty_for_task(
        task,
        market_context=market_context,
    )
    baseline_guard_penalty = _baseline_guard_penalty_for_bid(
        task,
        agent=agent,
        market_context=market_context,
    )
    score = round(
        base_value
        + aoi_gain
        + baseline_stale_bonus
        - estimated_cost
        - energy_penalty
        - hotspot_backlog_penalty,
        3,
    )
    score = round(
        score - baseline_guard_penalty,
        3,
    )
    return AuctionBid(
        task=task,
        agent=agent,
        score=score,
        base_value=base_value,
        aoi_gain=round(aoi_gain, 3),
        baseline_stale_bonus=round(baseline_stale_bonus, 3),
        path_cost=round(estimated_cost, 3),
        energy_penalty=energy_penalty,
        hotspot_backlog_penalty=round(hotspot_backlog_penalty, 3),
        baseline_guard_penalty=round(baseline_guard_penalty, 3),
    )


def _aoi_gain_for_task(task: TaskRecord, *, info_map: InformationMap | None) -> float:
    if info_map is None or task.target_row is None or task.target_col is None:
        return 0.0
    state = info_map.state_at(task.target_row, task.target_col)
    return min(state.information_age * AOI_GAIN_PER_STEP, MAX_AOI_GAIN)


def _baseline_stale_bonus_for_task(task: TaskRecord, *, info_map: InformationMap | None) -> float:
    if task.task_type != TaskType.BASELINE_SERVICE:
        return 0.0
    if info_map is None or task.target_row is None or task.target_col is None:
        return 0.0
    state = info_map.state_at(task.target_row, task.target_col)
    return min(state.information_age * BASELINE_STALE_BONUS_PER_STEP, MAX_BASELINE_STALE_BONUS)


def _hotspot_backlog_penalty_for_task(
    task: TaskRecord,
    *,
    market_context: AuctionMarketContext,
) -> float:
    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return 0.0
    backlog = max(market_context.pending_hotspots - HOTSPOT_BACKLOG_THRESHOLD, 0)
    return min(backlog * HOTSPOT_BACKLOG_PENALTY_PER_TASK, MAX_HOTSPOT_BACKLOG_PENALTY)


def _baseline_guard_penalty_for_bid(
    task: TaskRecord,
    *,
    agent: AgentState,
    market_context: AuctionMarketContext,
) -> float:
    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return 0.0
    if agent.agent_id not in market_context.baseline_guard_agent_ids:
        return 0.0
    return BASELINE_BACKLOG_GUARD_PENALTY


def _baseline_guard_agent_ids(
    tasks: tuple[TaskRecord, ...],
    *,
    info_map: InformationMap | None,
    step: int,
    zone_partition_policy: str,
    all_agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
) -> tuple[str, ...]:
    if info_map is None:
        return ()

    guard_ids: set[str] = set()
    for task in tasks:
        if task.task_type != TaskType.BASELINE_SERVICE:
            continue
        if task.target_row is None or task.target_col is None:
            continue
        state = info_map.state_at(task.target_row, task.target_col)
        if state.information_age < BASELINE_BACKLOG_GUARD_AGE_STEPS:
            continue
        partition = build_task_partition(
            task,
            policy_name=zone_partition_policy,
            tasks=tasks,
            agents=all_agents,
            execution_states=execution_states,
            step=step,
        )
        guard_ids.update(partition.primary_usv_ids)
    return tuple(sorted(guard_ids))


def _protected_support_usv_ids(agents: tuple[AgentState, ...]) -> set[str]:
    usvs = tuple(agent for agent in agents if agent.kind == "USV" and is_operational_agent(agent))
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
        return "highest_aoi_energy_bid_partition_usv_for_baseline_service"
    return "highest_aoi_energy_bid_partition_usv_for_hotspot_confirmation"


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
            (retry_after_step for _, retry_after_step in active_retries), default=None
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
            (retry_after_step for _, retry_after_step in retained_retries), default=None
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
            (retry_after for _, retry_after in agent_retry_after_steps), default=None
        ),
        agent_retry_after_steps=agent_retry_after_steps,
    )
