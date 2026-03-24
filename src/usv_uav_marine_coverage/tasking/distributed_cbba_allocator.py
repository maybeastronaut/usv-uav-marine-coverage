"""Light-weight distributed CBBA-style task allocation for comparative experiments."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import hypot

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
    stabilize_uav_resupply_task,
    task_sort_key,
)
from .partitioning import build_task_partition
from .task_types import TaskAssignment, TaskRecord, TaskStatus, TaskType

HOTSPOT_BASE_VALUE = 86.0
BASELINE_BASE_VALUE = 48.0
AOI_REWARD_PER_STEP = 0.32
MAX_AOI_REWARD = 36.0
PATH_COST_WEIGHT = 0.84
LOW_ENERGY_SUPPORT_RATIO = 0.35
LOW_ENERGY_SUPPORT_PENALTY = 18.0
DEFAULT_CBBA_BUNDLE_LENGTH = 1
AGENT_TASK_BLOCKED_COOLDOWN_STEPS = 12


@dataclass(frozen=True)
class CbbaBid:
    """One local bid proposal for one task-agent pair."""

    task: TaskRecord
    agent: AgentState
    bid_score: float
    base_value: float
    aoi_reward: float
    path_cost: float
    energy_penalty: float


@dataclass(frozen=True)
class CbbaBundleBid:
    """One local ordered task bundle proposed by one agent."""

    agent: AgentState
    task_bids: tuple[CbbaBid, ...]
    marginal_scores: tuple[float, ...]
    bundle_total_score: float
    earliest_created_step: int


def allocate_tasks_with_distributed_cbba_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None = None,
    info_map: InformationMap | None = None,
    step: int | None = None,
    usv_path_planner: str = "astar_path_planner",
    zone_partition_policy: str = "weighted_voronoi_partition_policy",
    sync_interval_steps: int = 1,
    broadcast_range_m: float = 0.0,
    winner_memory_ttl_steps: int = 0,
    bundle_length: int = DEFAULT_CBBA_BUNDLE_LENGTH,
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Apply one minimal distributed CBBA-style task-allocation policy."""
    if bundle_length not in {1, 2}:
        raise ValueError("distributed CBBA bundle_length must be 1 or 2")

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
        task = _prune_expired_winner_memories(
            task,
            step=step,
            winner_memory_ttl_steps=winner_memory_ttl_steps,
        )
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
                usv_path_planner=usv_path_planner,
            )
            updated_tasks.append(updated_task)
            if decision is not None:
                decisions.append(decision)
            continue

        pending_market_tasks.append(task)

    if _is_cbba_sync_step(step, sync_interval_steps=sync_interval_steps):
        assigned_tasks, market_decisions = _allocate_cbba_market(
            pending_market_tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            task_records=tuple(updated_tasks),
            reserved_agent_ids=reserved_agent_ids,
            protected_support_usv_ids=protected_support_usv_ids,
            reachability_cache=reachability_cache,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            sync_interval_steps=sync_interval_steps,
            broadcast_range_m=broadcast_range_m,
            winner_memory_ttl_steps=winner_memory_ttl_steps,
            bundle_length=bundle_length,
        )
    else:
        assigned_tasks = pending_market_tasks
        market_decisions = []
    updated_tasks.extend(assigned_tasks)
    decisions.extend(market_decisions)

    return (tuple(sorted(updated_tasks, key=lambda item: item.task_id)), tuple(decisions))


def _is_cbba_sync_step(step: int | None, *, sync_interval_steps: int) -> bool:
    if sync_interval_steps <= 1 or step is None:
        return True
    return step % sync_interval_steps == 0


def _allocate_cbba_market(
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
    sync_interval_steps: int,
    broadcast_range_m: float,
    winner_memory_ttl_steps: int,
    bundle_length: int,
) -> tuple[list[TaskRecord], list[TaskAssignment]]:
    if not tasks:
        return ([], [])

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
    current_step = max((task.created_step for task in tasks), default=0) if step is None else step
    assigned_tasks: list[TaskRecord] = []
    decisions: list[TaskAssignment] = []
    blocked_agent_ids_by_task: dict[str, set[str]] = {}
    round_index = 0

    while remaining_tasks and available_agents:
        round_index += 1
        round_bundles: list[CbbaBundleBid] = []
        component_sizes_by_task: dict[str, int] = {}
        communication_components = _build_communication_components(
            tuple(available_agents.values()),
            broadcast_range_m=broadcast_range_m,
        )
        task_component_indices = _assign_tasks_to_components(
            tuple(remaining_tasks.values()),
            communication_components=communication_components,
            agents=agents,
            execution_states=execution_states,
            step=current_step,
            zone_partition_policy=zone_partition_policy,
            available_agent_ids=set(available_agents),
        )

        for component_index, component_agents in enumerate(communication_components):
            component_tasks = tuple(
                task
                for task in remaining_tasks.values()
                if task_component_indices.get(task.task_id) == component_index
            )
            if not component_tasks:
                continue
            component_agent_ids = {agent.agent_id for agent in component_agents}
            component_available_agents = tuple(
                available_agents[agent_id]
                for agent_id in component_agent_ids
                if agent_id in available_agents
            )
            if not component_available_agents:
                continue
            for task in component_tasks:
                component_sizes_by_task[task.task_id] = len(component_available_agents)
            for agent in component_available_agents:
                local_bundle, newly_blocked = _build_local_bundle_for_agent(
                    agent,
                    tasks=component_tasks,
                    agents=agents,
                    execution_states=execution_states,
                    grid_map=grid_map,
                    info_map=info_map,
                    protected_support_usv_ids=protected_support_usv_ids,
                    reachability_cache=reachability_cache,
                    step=current_step,
                    usv_path_planner=usv_path_planner,
                    zone_partition_policy=zone_partition_policy,
                    winner_memory_ttl_steps=winner_memory_ttl_steps,
                    bundle_length=bundle_length,
                )
                for task_id, blocked_agent_ids in newly_blocked.items():
                    blocked_agent_ids_by_task.setdefault(task_id, set()).update(blocked_agent_ids)
                if local_bundle is None:
                    continue
                round_bundles.append(local_bundle)

        if not round_bundles:
            break

        bundles_by_task_id: dict[str, list[CbbaBundleBid]] = {}
        for bundle in round_bundles:
            for task_bid in bundle.task_bids:
                bundles_by_task_id.setdefault(task_bid.task.task_id, []).append(bundle)

        for winning_bundle in sorted(
            round_bundles,
            key=lambda item: (
                -item.bundle_total_score,
                item.earliest_created_step,
                item.agent.agent_id,
            ),
        ):
            agent = winning_bundle.agent
            bundle_task_ids = tuple(task_bid.task.task_id for task_bid in winning_bundle.task_bids)
            if agent.agent_id not in available_agents:
                continue
            if any(task_id not in remaining_tasks for task_id in bundle_task_ids):
                continue
            for bundle_position, (task_bid, marginal_score) in enumerate(
                zip(
                    winning_bundle.task_bids,
                    winning_bundle.marginal_scores,
                    strict=True,
                ),
                start=1,
            ):
                task = task_bid.task
                candidate_scores = tuple(
                    sorted(
                        (
                            {
                                "agent_id": candidate.agent.agent_id,
                                "support_agent_id": None,
                                "bid_score": round(
                                    _task_score_for_bundle(
                                        candidate,
                                        task_id=task.task_id,
                                    ),
                                    3,
                                ),
                                "bundle_total_score": round(candidate.bundle_total_score, 3),
                                "bundle_position": _task_position_in_bundle(
                                    candidate,
                                    task_id=task.task_id,
                                ),
                                "base_value": round(
                                    _task_bid_for_bundle(
                                        candidate,
                                        task_id=task.task_id,
                                    ).base_value,
                                    3,
                                ),
                                "aoi_reward": round(
                                    _task_bid_for_bundle(
                                        candidate,
                                        task_id=task.task_id,
                                    ).aoi_reward,
                                    3,
                                ),
                                "path_cost": round(
                                    _task_bid_for_bundle(
                                        candidate,
                                        task_id=task.task_id,
                                    ).path_cost,
                                    3,
                                ),
                                "energy_penalty": round(
                                    _task_bid_for_bundle(
                                        candidate,
                                        task_id=task.task_id,
                                    ).energy_penalty,
                                    3,
                                ),
                            }
                            for candidate in bundles_by_task_id[task.task_id]
                        ),
                        key=lambda item: (
                            -float(item["bundle_total_score"]),
                            int(item["bundle_position"]),
                            str(item["agent_id"]),
                        ),
                    )
                )
                assigned_task = _mark_task_assigned(task, selected_agent_id=agent.agent_id)
                assigned_task = _update_winner_memories(
                    assigned_task,
                    observer_agent_ids=tuple(
                        candidate.agent.agent_id for candidate in bundles_by_task_id[task.task_id]
                    ),
                    winner_agent_id=agent.agent_id,
                    step=current_step,
                    winner_memory_ttl_steps=winner_memory_ttl_steps,
                )
                assigned_tasks.append(assigned_task)
                decisions.append(
                    TaskAssignment(
                        task_id=task.task_id,
                        task_type=task.task_type,
                        agent_id=agent.agent_id,
                        support_agent_id=None,
                        selection_reason=_selection_reason_for_task(task),
                        selection_score=round(marginal_score, 3),
                        selection_details={
                            "cbba_round": round_index,
                            "bundle_length": len(winning_bundle.task_bids),
                            "bundle_position": bundle_position,
                            "bundle_task_ids": bundle_task_ids,
                            "bundle_total_score": round(winning_bundle.bundle_total_score, 3),
                            "marginal_score": round(marginal_score, 3),
                            "sync_interval_steps": sync_interval_steps,
                            "broadcast_range_m": round(broadcast_range_m, 3),
                            "winner_memory_ttl_steps": winner_memory_ttl_steps,
                            "communication_component_size": component_sizes_by_task.get(
                                task.task_id, 1
                            ),
                            "base_value": round(task_bid.base_value, 3),
                            "aoi_reward": round(task_bid.aoi_reward, 3),
                            "path_cost": round(task_bid.path_cost, 3),
                            "energy_penalty": round(task_bid.energy_penalty, 3),
                            "final_bid": round(task_bid.bid_score, 3),
                        },
                        candidate_agents=candidate_scores,
                    )
                )
                remaining_tasks.pop(task.task_id)
            available_agents.pop(agent.agent_id)

    for task in remaining_tasks.values():
        assigned_tasks.append(
            _mark_task_unassigned(
                task,
                step=current_step,
                blocked_agent_ids=blocked_agent_ids_by_task.get(task.task_id, set()),
            )
        )

    return (assigned_tasks, decisions)


def _build_local_bundle_for_agent(
    agent: AgentState,
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap | None,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
    reachability_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    winner_memory_ttl_steps: int,
    bundle_length: int,
) -> tuple[CbbaBundleBid | None, dict[str, set[str]]]:
    local_bids: list[CbbaBid] = []
    blocked_agent_ids_by_task: dict[str, set[str]] = {}

    for task in tasks:
        partition = build_task_partition(
            task,
            policy_name=zone_partition_policy,
            tasks=tasks,
            agents=agents,
            execution_states=execution_states,
            step=step,
        )
        preferred_ids = set(partition.primary_usv_ids) | set(partition.secondary_usv_ids)
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
        remembered_winner_id = _remembered_winner_for_agent(
            task,
            observer_agent_id=agent.agent_id,
            step=step,
            winner_memory_ttl_steps=winner_memory_ttl_steps,
        )
        if remembered_winner_id not in {None, agent.agent_id}:
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
        local_bids.append(
            _build_cbba_bid(
                task,
                agent=agent,
                estimated_cost=estimated_cost,
                info_map=info_map,
                protected_support_usv_ids=protected_support_usv_ids,
            )
        )

    if not local_bids:
        return (None, blocked_agent_ids_by_task)
    ranked_bids = sorted(
        local_bids,
        key=lambda item: (-item.bid_score, item.task.created_step, item.task.task_id),
    )
    first_bid = ranked_bids[0]
    task_bids = [first_bid]
    marginal_scores = [first_bid.bid_score]

    if bundle_length >= 2:
        remaining_bids = [
            bid for bid in ranked_bids[1:] if bid.task.task_id != first_bid.task.task_id
        ]
        second_choice: tuple[CbbaBid, float] | None = None
        for candidate in remaining_bids:
            inter_task_cost = hypot(
                first_bid.task.target_x - candidate.task.target_x,
                first_bid.task.target_y - candidate.task.target_y,
            )
            marginal_score = round(
                candidate.bid_score - (inter_task_cost * PATH_COST_WEIGHT),
                3,
            )
            if second_choice is None or (
                marginal_score,
                -candidate.task.created_step,
                candidate.task.task_id,
            ) > (
                second_choice[1],
                -second_choice[0].task.created_step,
                second_choice[0].task.task_id,
            ):
                second_choice = (candidate, marginal_score)
        if second_choice is not None:
            task_bids.append(second_choice[0])
            marginal_scores.append(second_choice[1])

    bundle_total_score = round(sum(marginal_scores), 3)
    return (
        CbbaBundleBid(
            agent=agent,
            task_bids=tuple(task_bids),
            marginal_scores=tuple(marginal_scores),
            bundle_total_score=bundle_total_score,
            earliest_created_step=min(bid.task.created_step for bid in task_bids),
        ),
        blocked_agent_ids_by_task,
    )


def _build_cbba_bid(
    task: TaskRecord,
    *,
    agent: AgentState,
    estimated_cost: float,
    info_map: InformationMap | None,
    protected_support_usv_ids: set[str],
) -> CbbaBid:
    base_value = (
        HOTSPOT_BASE_VALUE
        if task.task_type == TaskType.HOTSPOT_CONFIRMATION
        else BASELINE_BASE_VALUE
    )
    aoi_reward = _aoi_reward_for_task(task, info_map=info_map)
    energy_penalty = (
        LOW_ENERGY_SUPPORT_PENALTY if agent.agent_id in protected_support_usv_ids else 0.0
    )
    bid_score = round(
        base_value + aoi_reward - (estimated_cost * PATH_COST_WEIGHT) - energy_penalty,
        3,
    )
    return CbbaBid(
        task=task,
        agent=agent,
        bid_score=bid_score,
        base_value=base_value,
        aoi_reward=round(aoi_reward, 3),
        path_cost=round(estimated_cost, 3),
        energy_penalty=round(energy_penalty, 3),
    )


def _aoi_reward_for_task(task: TaskRecord, *, info_map: InformationMap | None) -> float:
    if info_map is None or task.target_row is None or task.target_col is None:
        return 0.0
    state = info_map.state_at(task.target_row, task.target_col)
    return min(state.information_age * AOI_REWARD_PER_STEP, MAX_AOI_REWARD)


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


def _build_communication_components(
    agents: tuple[AgentState, ...],
    *,
    broadcast_range_m: float,
) -> tuple[tuple[AgentState, ...], ...]:
    if not agents:
        return ()
    if broadcast_range_m <= 0.0:
        return (tuple(sorted(agents, key=lambda item: item.agent_id)),)

    remaining = {agent.agent_id: agent for agent in agents}
    components: list[tuple[AgentState, ...]] = []
    while remaining:
        start_agent_id = next(iter(remaining))
        pending = [start_agent_id]
        component_ids: set[str] = set()
        while pending:
            current_id = pending.pop()
            if current_id in component_ids:
                continue
            current_agent = remaining[current_id]
            component_ids.add(current_id)
            for other_id, other_agent in remaining.items():
                if other_id in component_ids:
                    continue
                if (
                    distance_to_point(current_agent, other_agent.x, other_agent.y)
                    <= broadcast_range_m
                ):
                    pending.append(other_id)
        component_agents = tuple(
            sorted(
                (remaining.pop(agent_id) for agent_id in component_ids),
                key=lambda item: item.agent_id,
            )
        )
        components.append(component_agents)
    return tuple(components)


def _assign_tasks_to_components(
    tasks: tuple[TaskRecord, ...],
    *,
    communication_components: tuple[tuple[AgentState, ...], ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    step: int,
    zone_partition_policy: str,
    available_agent_ids: set[str],
) -> dict[str, int]:
    if len(communication_components) <= 1:
        return {task.task_id: 0 for task in tasks}

    component_index_by_agent: dict[str, int] = {}
    for component_index, component in enumerate(communication_components):
        for agent in component:
            component_index_by_agent[agent.agent_id] = component_index

    assignment: dict[str, int] = {}
    for task in tasks:
        partition = build_task_partition(
            task,
            policy_name=zone_partition_policy,
            tasks=tasks,
            agents=agents,
            execution_states=execution_states,
            step=step,
        )
        primary_ids = [
            agent_id for agent_id in partition.primary_usv_ids if agent_id in available_agent_ids
        ]
        candidate_ids = primary_ids or [
            agent_id
            for agent_id in (tuple(partition.primary_usv_ids) + tuple(partition.secondary_usv_ids))
            if agent_id in available_agent_ids
        ]
        if not candidate_ids:
            continue
        anchor_agent_id = candidate_ids[0]
        component_index = component_index_by_agent.get(anchor_agent_id)
        if component_index is not None:
            assignment[task.task_id] = component_index
    return assignment


def _selection_reason_for_task(task: TaskRecord) -> str:
    if task.task_type == TaskType.BASELINE_SERVICE:
        return "distributed_cbba_winner_for_baseline_service"
    return "distributed_cbba_winner_for_hotspot_confirmation"


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


def _task_bid_for_bundle(bundle: CbbaBundleBid, *, task_id: str) -> CbbaBid:
    for task_bid in bundle.task_bids:
        if task_bid.task.task_id == task_id:
            return task_bid
    raise ValueError(f"Task {task_id!r} not present in bundle")


def _task_position_in_bundle(bundle: CbbaBundleBid, *, task_id: str) -> int:
    for index, task_bid in enumerate(bundle.task_bids, start=1):
        if task_bid.task.task_id == task_id:
            return index
    raise ValueError(f"Task {task_id!r} not present in bundle")


def _task_score_for_bundle(bundle: CbbaBundleBid, *, task_id: str) -> float:
    for task_bid, marginal_score in zip(bundle.task_bids, bundle.marginal_scores, strict=True):
        if task_bid.task.task_id == task_id:
            return marginal_score
    raise ValueError(f"Task {task_id!r} not present in bundle")


def _remembered_winner_for_agent(
    task: TaskRecord,
    *,
    observer_agent_id: str,
    step: int,
    winner_memory_ttl_steps: int,
) -> str | None:
    if winner_memory_ttl_steps <= 0:
        return None
    for candidate_observer_id, winner_agent_id, known_step in task.distributed_winner_memories:
        if candidate_observer_id != observer_agent_id:
            continue
        if step - known_step > winner_memory_ttl_steps:
            return None
        return winner_agent_id
    return None


def _prune_expired_winner_memories(
    task: TaskRecord,
    *,
    step: int | None,
    winner_memory_ttl_steps: int,
) -> TaskRecord:
    if step is None or winner_memory_ttl_steps <= 0 or not task.distributed_winner_memories:
        return task
    active_memories = tuple(
        (observer_agent_id, winner_agent_id, known_step)
        for observer_agent_id, winner_agent_id, known_step in task.distributed_winner_memories
        if step - known_step <= winner_memory_ttl_steps
    )
    if active_memories == task.distributed_winner_memories:
        return task
    return replace(task, distributed_winner_memories=active_memories)


def _update_winner_memories(
    task: TaskRecord,
    *,
    observer_agent_ids: tuple[str, ...],
    winner_agent_id: str | None,
    step: int,
    winner_memory_ttl_steps: int,
) -> TaskRecord:
    if winner_memory_ttl_steps <= 0:
        return task
    memory_by_observer = {
        observer_agent_id: (winner_agent_id, known_step)
        for observer_agent_id, winner_agent_id, known_step in task.distributed_winner_memories
    }
    for observer_agent_id in observer_agent_ids:
        memory_by_observer[observer_agent_id] = (winner_agent_id, step)
    updated_memories = tuple(
        sorted(
            (
                (observer_agent_id, remembered_winner_id, known_step)
                for observer_agent_id, (
                    remembered_winner_id,
                    known_step,
                ) in memory_by_observer.items()
            ),
            key=lambda item: item[0],
        )
    )
    return replace(task, distributed_winner_memories=updated_memories)
