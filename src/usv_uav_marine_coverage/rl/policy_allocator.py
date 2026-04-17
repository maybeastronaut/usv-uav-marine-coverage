"""Centralized RL task-allocation helpers for USV high-level decisions."""

from __future__ import annotations

from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path

import numpy as np

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    HealthStatus,
    distance_to_point,
    energy_ratio,
    is_operational_agent,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    ExecutionStage,
)
from usv_uav_marine_coverage.execution.task_claim_runtime import (
    find_task_by_id,
    select_claimable_task_for_agent,
)
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    InformationMap,
    InformationValidity,
)
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan
from usv_uav_marine_coverage.tasking.allocator_common import (
    allocate_uav_resupply_task,
    can_keep_existing_assignment,
    is_agent_task_in_cooldown,
    is_available_for_candidate_pool,
    is_task_waiting_for_retry,
    protected_support_usv_ids_for_low_energy_uavs,
    selection_score_for_task,
    stabilize_uav_resupply_task,
    task_sort_key,
)
from usv_uav_marine_coverage.tasking.partitioning import build_task_partition
from usv_uav_marine_coverage.tasking.task_types import (
    TaskAssignment,
    TaskRecord,
    TaskStatus,
    TaskType,
)

USV_ACTION_SLOT_COUNT = 6
USV_AGENT_ORDER = ("USV-1", "USV-2", "USV-3")
DEFAULT_POLICY_ID = "default_policy"
GLOBAL_FEATURE_DIM = 6
USV_STATE_FEATURE_DIM = 9
SLOT_FEATURE_DIM = 9
OBSERVATION_SIZE = GLOBAL_FEATURE_DIM + len(USV_AGENT_ORDER) * (
    USV_STATE_FEATURE_DIM + USV_ACTION_SLOT_COUNT * SLOT_FEATURE_DIM
)


@dataclass(frozen=True)
class UsvTaskCandidateSlot:
    """One fixed-slot action candidate exposed to the RL policy."""

    slot_index: int
    slot_name: str
    task_id: str | None
    task_type: TaskType | None
    target_x: float | None
    target_y: float | None
    distance_m: float
    priority: int
    aoi_steps: int
    in_partition: bool
    reachable: bool
    overflow: bool
    selection_score: float
    partition_reason: str | None = None


@dataclass(frozen=True)
class UsvAgentAllocationView:
    """One USV-specific fixed observation/action view."""

    agent_id: str
    current_task_id: str | None
    current_task_type: TaskType | None
    current_task_distance_m: float
    available_for_new_assignment: bool
    resupply_protected: bool
    slots: tuple[UsvTaskCandidateSlot, ...]


@dataclass(frozen=True)
class CentralizedUsvAllocationContext:
    """Full centralized policy input for the current simulation step."""

    observation: np.ndarray
    usv_order: tuple[str, ...]
    agent_views: tuple[UsvAgentAllocationView, ...]
    checkpoint_name: str | None = None


@dataclass(frozen=True)
class AppliedUsvAllocation:
    """Result of applying one centralized USV action."""

    task_records: tuple[TaskRecord, ...]
    decisions: tuple[TaskAssignment, ...]
    invalid_action_count: int
    unnecessary_reassignments: int


@dataclass(frozen=True)
class _RankedTaskCandidate:
    task: TaskRecord
    reachable: bool
    distance_m: float
    selection_score: float
    aoi_steps: int
    in_partition: bool
    partition_reason: str


def build_centralized_usv_allocation_context(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap,
    info_map: InformationMap,
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    checkpoint_name: str | None = None,
) -> CentralizedUsvAllocationContext:
    """Build the fixed-slot observation and action context for centralized PPO."""

    usv_agents = _ordered_usv_agents(agents)
    observation_values = _build_global_feature_block(info_map, step=step)
    agent_views: list[UsvAgentAllocationView] = []
    path_cache: dict[tuple[str, str, float, float], tuple[bool, float]] = {}
    protected_support_usv_ids = protected_support_usv_ids_for_low_energy_uavs(agents)

    for agent in usv_agents:
        execution_state = execution_states.get(agent.agent_id)
        current_task = select_claimable_task_for_agent(
            tasks,
            agent_id=agent.agent_id,
            execution_state=execution_state,
        )
        available_for_new_assignment = is_available_for_candidate_pool(
            execution_state,
            agent_id=agent.agent_id,
            task_records=tasks,
        )
        current_task_distance = (
            0.0
            if current_task is None
            else round(distance_to_point(agent, current_task.target_x, current_task.target_y), 3)
        )
        slots = _build_usv_action_slots(
            agent,
            current_task=current_task,
            tasks=tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            path_cache=path_cache,
            protected_for_resupply=agent.agent_id in protected_support_usv_ids,
        )
        agent_view = UsvAgentAllocationView(
            agent_id=agent.agent_id,
            current_task_id=None if current_task is None else current_task.task_id,
            current_task_type=None if current_task is None else current_task.task_type,
            current_task_distance_m=current_task_distance,
            available_for_new_assignment=available_for_new_assignment,
            resupply_protected=agent.agent_id in protected_support_usv_ids,
            slots=slots,
        )
        agent_views.append(agent_view)
        observation_values.extend(
            _build_usv_state_block(
                agent,
                execution_state=execution_state,
                current_task=current_task,
                grid_map=grid_map,
            )
        )
        for slot in slots:
            observation_values.extend(_build_slot_feature_block(slot, grid_map=grid_map, step=step))

    observation = np.asarray(observation_values, dtype=np.float32)
    if observation.shape != (OBSERVATION_SIZE,):
        raise ValueError(
            f"Unexpected centralized PPO observation size {observation.shape}; "
            f"expected {(OBSERVATION_SIZE,)}"
        )
    return CentralizedUsvAllocationContext(
        observation=observation,
        usv_order=tuple(agent.agent_id for agent in usv_agents),
        agent_views=tuple(agent_views),
        checkpoint_name=checkpoint_name,
    )


def apply_centralized_usv_allocation_actions(
    tasks: tuple[TaskRecord, ...],
    *,
    context: CentralizedUsvAllocationContext,
    action: np.ndarray | list[int] | tuple[int, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    step: int,
) -> AppliedUsvAllocation:
    """Apply one centralized fixed-slot action to the current task set."""

    normalized_action = _normalize_action_array(action)
    if len(normalized_action) != len(context.usv_order):
        raise ValueError(
            f"Centralized PPO action length {len(normalized_action)} does not match "
            f"{len(context.usv_order)} USVs"
        )

    task_by_id = {task.task_id: task for task in tasks}
    agent_by_id = {agent.agent_id: agent for agent in agents}
    current_task_by_agent = {
        view.agent_id: find_task_by_id(tasks, view.current_task_id) for view in context.agent_views
    }
    claimed_task_ids: set[str] = set()
    decisions: list[TaskAssignment] = []
    invalid_action_count = 0
    unnecessary_reassignments = 0

    for view, selected_slot in zip(context.agent_views, normalized_action, strict=True):
        agent = agent_by_id[view.agent_id]
        current_task = current_task_by_agent[view.agent_id]
        execution_state = execution_states.get(view.agent_id)
        slot = view.slots[selected_slot]

        if selected_slot == 0:
            if view.resupply_protected and current_task is not None and current_task.task_type != TaskType.UAV_RESUPPLY:
                invalid_action_count += 1
                continue
            if current_task is None:
                invalid_action_count += 1
                continue
            decisions.append(
                _build_rl_task_assignment(
                    task=current_task,
                    agent=agent,
                    slot=slot,
                    checkpoint_name=context.checkpoint_name,
                    action_label="keep_current_task",
                )
            )
            claimed_task_ids.add(current_task.task_id)
            continue

        if selected_slot == 5:
            if view.resupply_protected:
                invalid_action_count += 1
                continue
            if current_task is None:
                invalid_action_count += 1
                continue
            if current_task.task_type == TaskType.UAV_RESUPPLY:
                invalid_action_count += 1
                continue
            task_by_id[current_task.task_id] = _release_task_from_usv(
                current_task,
                agent_id=agent.agent_id,
            )
            unnecessary_reassignments += 1
            decisions.append(
                _build_rl_task_assignment(
                    task=current_task,
                    agent=agent,
                    slot=slot,
                    checkpoint_name=context.checkpoint_name,
                    action_label="release_to_patrol",
                )
            )
            continue

        if not slot.reachable or slot.task_id is None:
            invalid_action_count += 1
            continue

        selected_task = task_by_id.get(slot.task_id)
        if selected_task is None:
            invalid_action_count += 1
            continue
        if selected_task.task_id in claimed_task_ids:
            invalid_action_count += 1
            continue
        if view.resupply_protected and selected_task.task_type != TaskType.UAV_RESUPPLY:
            invalid_action_count += 1
            continue
        # Hard constraint: PPO should not assign tasks that are irrationally far away (>500m)
        if hasattr(selected_task, 'target_x') and selected_task.target_x is not None:
            dist = (
                (agent.x - selected_task.target_x)**2 + 
                (agent.y - selected_task.target_y)**2
            )**0.5
            if dist > 500.0:
                invalid_action_count += 1
                if current_task is not None:
                    decisions.append(
                        _build_rl_task_assignment(
                            task=current_task,
                            agent=agent,
                            slot=view.slots[0], # The default slot 0 is keep_current_task
                            checkpoint_name=context.checkpoint_name,
                            action_label="keep_current_task",
                        )
                    )
                    claimed_task_ids.add(current_task.task_id)
                continue

        if not _can_agent_switch_to_slot(
            agent=agent,
            execution_state=execution_state,
            current_task=current_task,
            selected_task=selected_task,
        ):
            invalid_action_count += 1
            continue

        if current_task is not None and current_task.task_id != selected_task.task_id:
            task_by_id[current_task.task_id] = _release_task_from_usv(
                current_task,
                agent_id=agent.agent_id,
            )
            unnecessary_reassignments += 1

        updated_task = _assign_task_to_usv(
            selected_task,
            support_agent_id=agent.agent_id if selected_task.task_type == TaskType.UAV_RESUPPLY else None,
            selected_agent_id=agent.agent_id if selected_task.task_type != TaskType.UAV_RESUPPLY else None,
        )
        task_by_id[selected_task.task_id] = updated_task
        claimed_task_ids.add(selected_task.task_id)
        decisions.append(
            _build_rl_task_assignment(
                task=updated_task,
                agent=agent,
                slot=slot,
                checkpoint_name=context.checkpoint_name,
                action_label="assign_slot",
            )
        )

    updated_tasks = tuple(sorted(task_by_id.values(), key=lambda item: item.task_id))
    return AppliedUsvAllocation(
        task_records=updated_tasks,
        decisions=tuple(decisions),
        invalid_action_count=invalid_action_count,
        unnecessary_reassignments=unnecessary_reassignments,
    )


def allocate_tasks_with_rllib_ppo_policy(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap,
    info_map: InformationMap,
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    checkpoint_path: str | None,
    policy_id: str = DEFAULT_POLICY_ID,
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Run centralized PPO inference and return the updated task assignments."""

    if not checkpoint_path:
        raise ValueError("checkpoint_path is required for rllib_ppo_usv_allocator")
    checkpoint = Path(checkpoint_path).resolve()
    prepared_tasks, safety_decisions = _apply_rule_based_uav_resupply_fallback(
        tasks,
        agents=agents,
        execution_states=execution_states,
        grid_map=grid_map,
        step=step,
        usv_path_planner=usv_path_planner,
    )
    context = build_centralized_usv_allocation_context(
        prepared_tasks,
        agents=agents,
        execution_states=execution_states,
        grid_map=grid_map,
        info_map=info_map,
        step=step,
        usv_path_planner=usv_path_planner,
        zone_partition_policy=zone_partition_policy,
        checkpoint_name=checkpoint.name,
    )
    action = _compute_rllib_action(context.observation, checkpoint_path=str(checkpoint), policy_id=policy_id)
    applied = apply_centralized_usv_allocation_actions(
        prepared_tasks,
        context=context,
        action=action,
        agents=agents,
        execution_states=execution_states,
        step=step,
    )
    return (
        applied.task_records,
        _merge_task_assignments(
            primary=applied.decisions,
            secondary=safety_decisions,
        ),
    )


def _apply_rule_based_uav_resupply_fallback(
    tasks: tuple[TaskRecord, ...],
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap,
    step: int,
    usv_path_planner: str,
) -> tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]]:
    """Keep UAV low-battery rendezvous on the rule-based safety path.

    PPO only owns the high-level USV task-selection problem. Safety-critical
    UAV recharge support remains mandatory and should not disappear just because
    the policy skips a slot.
    """

    agent_by_id = {agent.agent_id: agent for agent in agents}
    updated_tasks = list(tasks)
    decisions: list[TaskAssignment] = []

    for task in sorted(updated_tasks, key=task_sort_key):
        if task.task_type != TaskType.UAV_RESUPPLY:
            continue
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED, TaskStatus.IN_PROGRESS}:
            continue
        if can_keep_existing_assignment(
            task,
            agent_by_id=agent_by_id,
            execution_states=execution_states,
        ):
            continue
        updated_task, decision = allocate_uav_resupply_task(
            task,
            agent_by_id=agent_by_id,
            execution_states=execution_states,
            grid_map=grid_map,
            task_records=tuple(updated_tasks),
            step=step,
            usv_path_planner=usv_path_planner,
        )
        for index, existing_task in enumerate(updated_tasks):
            if existing_task.task_id == task.task_id:
                updated_tasks[index] = updated_task
                break
        if decision is not None:
            decisions.append(decision)

    return (tuple(sorted(updated_tasks, key=lambda item: item.task_id)), tuple(decisions))


def _merge_task_assignments(
    *,
    primary: tuple[TaskAssignment, ...],
    secondary: tuple[TaskAssignment, ...],
) -> tuple[TaskAssignment, ...]:
    """Merge task decisions, preferring the later PPO-side decision per task."""

    merged: dict[str, TaskAssignment] = {decision.task_id: decision for decision in secondary}
    merged.update({decision.task_id: decision for decision in primary})
    return tuple(merged.values())


def _build_global_feature_block(info_map: InformationMap, *, step: int) -> list[float]:
    total_coverable = max(1, sum(1 for cell in info_map.grid_map.flat_cells if not cell.has_obstacle))
    valid_cells = 0
    stale_cells = 0
    active_hotspot_backlog = 0
    confirmed_hotspots = 0
    uav_checked_cells = 0
    for cell in info_map.grid_map.flat_cells:
        if cell.has_obstacle:
            continue
        state = info_map.state_at(cell.row, cell.col)
        if state.validity == InformationValidity.VALID:
            valid_cells += 1
        else:
            stale_cells += 1
        if state.known_hotspot_state == HotspotKnowledgeState.CONFIRMED:
            confirmed_hotspots += 1
        elif state.known_hotspot_state == HotspotKnowledgeState.UAV_CHECKED:
            uav_checked_cells += 1
            active_hotspot_backlog += 1
        elif state.ground_truth_hotspot and state.known_hotspot_state == HotspotKnowledgeState.NONE:
            active_hotspot_backlog += 1
    return [
        min(step / 1200.0, 1.0),
        valid_cells / total_coverable,
        stale_cells / total_coverable,
        min(active_hotspot_backlog / 24.0, 1.0),
        min(confirmed_hotspots / 24.0, 1.0),
        uav_checked_cells / total_coverable,
    ]


def _build_usv_state_block(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState | None,
    current_task: TaskRecord | None,
    grid_map: GridMap,
) -> list[float]:
    current_distance = (
        0.0
        if current_task is None
        else min(distance_to_point(agent, current_task.target_x, current_task.target_y), 2_000.0)
        / 2_000.0
    )
    return [
        agent.x / grid_map.width,
        agent.y / grid_map.height,
        (agent.heading_deg % 360.0) / 360.0,
        0.0 if agent.max_speed_mps <= 0.0 else agent.speed_mps / max(agent.max_speed_mps, 1.0),
        energy_ratio(agent),
        _encode_health_status(agent.health_status),
        _encode_execution_stage(None if execution_state is None else execution_state.stage),
        _encode_task_type(None if current_task is None else current_task.task_type),
        current_distance,
    ]


def _build_slot_feature_block(
    slot: UsvTaskCandidateSlot,
    *,
    grid_map: GridMap,
    step: int,
) -> list[float]:
    return [
        _encode_task_type(slot.task_type),
        0.0 if slot.target_x is None else slot.target_x / grid_map.width,
        0.0 if slot.target_y is None else slot.target_y / grid_map.height,
        min(slot.distance_m, 2_000.0) / 2_000.0,
        min(slot.aoi_steps, 400) / 400.0,
        min(slot.priority, 20) / 20.0,
        1.0 if slot.in_partition else 0.0,
        1.0 if slot.reachable else 0.0,
        1.0 if slot.overflow else 0.0,
    ]


def _build_usv_action_slots(
    agent: AgentState,
    *,
    current_task: TaskRecord | None,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap,
    info_map: InformationMap,
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    path_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
    protected_for_resupply: bool = False,
) -> tuple[UsvTaskCandidateSlot, ...]:
    ranked_hotspots_in_partition: list[_RankedTaskCandidate] = []
    ranked_hotspots_overflow: list[_RankedTaskCandidate] = []
    ranked_baselines_in_partition: list[_RankedTaskCandidate] = []
    ranked_uav_resupply: list[_RankedTaskCandidate] = []

    for task in tasks:
        candidate = _rank_task_candidate_for_agent(
            task,
            agent=agent,
            all_tasks=tasks,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            path_cache=path_cache,
        )
        if candidate is None or not candidate.reachable:
            continue
        if task.task_type == TaskType.HOTSPOT_CONFIRMATION:
            if candidate.in_partition:
                ranked_hotspots_in_partition.append(candidate)
            else:
                ranked_hotspots_overflow.append(candidate)
        elif task.task_type == TaskType.BASELINE_SERVICE and candidate.in_partition:
            ranked_baselines_in_partition.append(candidate)
        elif task.task_type == TaskType.UAV_RESUPPLY:
            ranked_uav_resupply.append(candidate)

    ranked_hotspots_in_partition.sort(key=_candidate_sort_key)
    ranked_hotspots_overflow.sort(key=_candidate_sort_key)
    ranked_baselines_in_partition.sort(key=_candidate_sort_key)
    ranked_uav_resupply.sort(key=_candidate_sort_key)

    if protected_for_resupply:
        hotspot_primary: list[_RankedTaskCandidate] = []
        hotspot_secondary: list[_RankedTaskCandidate] = []
        baseline_primary: list[_RankedTaskCandidate] = []
        resupply_slot_candidates = ranked_uav_resupply[:1]
    else:
        hotspot_primary = ranked_hotspots_in_partition[:1]
        hotspot_secondary = ranked_hotspots_in_partition[1:2] or ranked_hotspots_overflow[:1]
        baseline_primary = ranked_baselines_in_partition[:1]
        resupply_slot_candidates = ranked_uav_resupply[:1] or ranked_hotspots_overflow[:1]
    slots = [
        _build_keep_slot(current_task, agent=agent),
        _candidate_to_slot(1, "best_partition_hotspot", hotspot_primary),
        _candidate_to_slot(
            2,
            "secondary_hotspot_or_overflow",
            hotspot_secondary,
        ),
        _candidate_to_slot(3, "best_partition_baseline", baseline_primary),
        _candidate_to_slot(
            4,
            "best_uav_resupply_or_overflow",
            resupply_slot_candidates,
        ),
        _build_release_slot(current_task, agent=agent),
    ]
    return tuple(slots)


def _rank_task_candidate_for_agent(
    task: TaskRecord,
    *,
    agent: AgentState,
    all_tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap,
    info_map: InformationMap,
    step: int,
    usv_path_planner: str,
    zone_partition_policy: str,
    path_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
) -> _RankedTaskCandidate | None:
    if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED, TaskStatus.IN_PROGRESS}:
        return None
    if is_task_waiting_for_retry(task, step=step):
        return None
    if is_agent_task_in_cooldown(task, agent_id=agent.agent_id, step=step):
        return None
    if not is_operational_agent(agent):
        return None
    if task.task_type not in {
        TaskType.HOTSPOT_CONFIRMATION,
        TaskType.BASELINE_SERVICE,
        TaskType.UAV_RESUPPLY,
    }:
        return None
    if task.task_type == TaskType.UAV_RESUPPLY:
        if task.assigned_agent_id is None:
            return None
        if task.support_agent_id not in {None, agent.agent_id}:
            return None
    elif task.assigned_agent_id not in {None, agent.agent_id}:
        return None

    partition = build_task_partition(
        task,
        policy_name=zone_partition_policy,
        tasks=all_tasks,
        agents=agents,
        execution_states=execution_states,
        info_map=info_map,
        step=step,
    )
    in_partition = agent.agent_id in partition.primary_usv_ids
    reachable, distance_m = _estimate_task_reachability(
        task,
        agent=agent,
        grid_map=grid_map,
        usv_path_planner=usv_path_planner,
        path_cache=path_cache,
    )
    if not reachable:
        return None
    aoi_steps = max(step - task.created_step, 0)
    selection_score = round(
        distance_m - task.priority * 12.0 - min(aoi_steps, 180) * 0.35,
        3,
    )
    return _RankedTaskCandidate(
        task=task,
        reachable=reachable,
        distance_m=distance_m,
        selection_score=selection_score,
        aoi_steps=aoi_steps,
        in_partition=in_partition,
        partition_reason=partition.partition_reason,
    )


def _estimate_task_reachability(
    task: TaskRecord,
    *,
    agent: AgentState,
    grid_map: GridMap,
    usv_path_planner: str,
    path_cache: dict[tuple[str, str, float, float], tuple[bool, float]],
) -> tuple[bool, float]:
    cache_key = (agent.agent_id, task.task_id, task.target_x, task.target_y)
    cached = path_cache.get(cache_key)
    if cached is not None:
        return cached
    path_plan = build_usv_path_plan(
        agent,
        grid_map=grid_map,
        goal_x=task.target_x,
        goal_y=task.target_y,
        planner_name=usv_path_planner,
        task_id=task.task_id,
        stats_context="rl_allocator_reachability",
    )
    reachable = path_plan.status != PathPlanStatus.BLOCKED
    distance_m = (
        round(path_plan.estimated_cost, 3)
        if reachable
        else round(distance_to_point(agent, task.target_x, task.target_y), 3)
    )
    path_cache[cache_key] = (reachable, distance_m)
    return path_cache[cache_key]


def _candidate_sort_key(candidate: _RankedTaskCandidate) -> tuple[float, int, str]:
    return (candidate.selection_score, candidate.task.created_step, candidate.task.task_id)


def _build_keep_slot(current_task: TaskRecord | None, *, agent: AgentState) -> UsvTaskCandidateSlot:
    if current_task is None:
        return UsvTaskCandidateSlot(
            slot_index=0,
            slot_name="keep_current_task",
            task_id=None,
            task_type=None,
            target_x=None,
            target_y=None,
            distance_m=0.0,
            priority=0,
            aoi_steps=0,
            in_partition=True,
            reachable=False,
            overflow=False,
            selection_score=0.0,
        )
    return UsvTaskCandidateSlot(
        slot_index=0,
        slot_name="keep_current_task",
        task_id=current_task.task_id,
        task_type=current_task.task_type,
        target_x=current_task.target_x,
        target_y=current_task.target_y,
        distance_m=round(distance_to_point(agent, current_task.target_x, current_task.target_y), 3),
        priority=current_task.priority,
        aoi_steps=0,
        in_partition=True,
        reachable=True,
        overflow=False,
        selection_score=0.0,
    )


def _build_release_slot(current_task: TaskRecord | None, *, agent: AgentState) -> UsvTaskCandidateSlot:
    if current_task is None:
        return UsvTaskCandidateSlot(
            slot_index=5,
            slot_name="release_to_patrol",
            task_id=None,
            task_type=None,
            target_x=None,
            target_y=None,
            distance_m=0.0,
            priority=0,
            aoi_steps=0,
            in_partition=False,
            reachable=False,
            overflow=False,
            selection_score=0.0,
        )
    return UsvTaskCandidateSlot(
        slot_index=5,
        slot_name="release_to_patrol",
        task_id=current_task.task_id,
        task_type=current_task.task_type,
        target_x=current_task.target_x,
        target_y=current_task.target_y,
        distance_m=round(distance_to_point(agent, current_task.target_x, current_task.target_y), 3),
        priority=current_task.priority,
        aoi_steps=0,
        in_partition=False,
        reachable=True,
        overflow=False,
        selection_score=0.0,
    )


def _candidate_to_slot(
    slot_index: int,
    slot_name: str,
    candidates: list[_RankedTaskCandidate],
) -> UsvTaskCandidateSlot:
    if not candidates:
        return UsvTaskCandidateSlot(
            slot_index=slot_index,
            slot_name=slot_name,
            task_id=None,
            task_type=None,
            target_x=None,
            target_y=None,
            distance_m=0.0,
            priority=0,
            aoi_steps=0,
            in_partition=False,
            reachable=False,
            overflow=False,
            selection_score=0.0,
        )
    candidate = candidates[0]
    task = candidate.task
    return UsvTaskCandidateSlot(
        slot_index=slot_index,
        slot_name=slot_name,
        task_id=task.task_id,
        task_type=task.task_type,
        target_x=task.target_x,
        target_y=task.target_y,
        distance_m=candidate.distance_m,
        priority=task.priority,
        aoi_steps=candidate.aoi_steps,
        in_partition=candidate.in_partition,
        reachable=candidate.reachable,
        overflow=not candidate.in_partition and task.task_type == TaskType.HOTSPOT_CONFIRMATION,
        selection_score=candidate.selection_score,
        partition_reason=candidate.partition_reason,
    )


def _assign_task_to_usv(
    task: TaskRecord,
    *,
    selected_agent_id: str | None,
    support_agent_id: str | None,
) -> TaskRecord:
    retained_retries = tuple(
        (agent_id, retry_after_step)
        for agent_id, retry_after_step in task.agent_retry_after_steps
        if agent_id not in {selected_agent_id, support_agent_id}
    )
    if task.task_type == TaskType.UAV_RESUPPLY:
        return replace(
            task,
            status=TaskStatus.ASSIGNED,
            assigned_agent_id=task.assigned_agent_id,
            support_agent_id=support_agent_id,
            retry_after_step=max(
                (retry_after_step for _, retry_after_step in retained_retries),
                default=None,
            ),
            agent_retry_after_steps=retained_retries,
        )
    return replace(
        task,
        status=TaskStatus.ASSIGNED,
        assigned_agent_id=selected_agent_id,
        support_agent_id=None,
        retry_after_step=max(
            (retry_after_step for _, retry_after_step in retained_retries),
            default=None,
        ),
        agent_retry_after_steps=retained_retries,
    )


def _release_task_from_usv(task: TaskRecord, *, agent_id: str) -> TaskRecord:
    if task.task_type == TaskType.UAV_RESUPPLY and task.support_agent_id == agent_id:
        return replace(
            task,
            status=TaskStatus.REQUEUED,
            support_agent_id=None,
        )
    if task.assigned_agent_id == agent_id:
        return replace(
            task,
            status=TaskStatus.REQUEUED,
            assigned_agent_id=None,
            support_agent_id=None,
        )
    return task


def _can_agent_switch_to_slot(
    *,
    agent: AgentState,
    execution_state: AgentExecutionState | None,
    current_task: TaskRecord | None,
    selected_task: TaskRecord,
) -> bool:
    if current_task is not None and current_task.task_id == selected_task.task_id:
        return True
    if current_task is not None and current_task.task_type == TaskType.UAV_RESUPPLY:
        return False
    if current_task is not None:
        if execution_state is None:
            return False
        return execution_state.stage in {
            ExecutionStage.PATROL,
            ExecutionStage.RETURN_TO_PATROL,
        } and execution_state.active_task_id is None
    return execution_state is not None and is_available_for_candidate_pool(
        execution_state,
        agent_id=agent.agent_id,
        task_records=(),
    )


def _build_rl_task_assignment(
    *,
    task: TaskRecord,
    agent: AgentState,
    slot: UsvTaskCandidateSlot,
    checkpoint_name: str | None,
    action_label: str,
) -> TaskAssignment:
    task_for_score = task
    if task.task_type == TaskType.UAV_RESUPPLY and task.assigned_agent_id is not None:
        task_for_score = stabilize_uav_resupply_task(
            task,
            selected_uav=agent if agent.kind == "UAV" else None,
        )
    selection_details = {
        "allocator": "rllib_ppo_usv_allocator",
        "action_label": action_label,
        "slot_index": slot.slot_index,
        "slot_name": slot.slot_name,
        "candidate_task_id": slot.task_id,
        "candidate_task_type": None if slot.task_type is None else slot.task_type.value,
        "candidate_reachable": slot.reachable,
        "candidate_in_partition": slot.in_partition,
        "candidate_overflow": slot.overflow,
        "checkpoint_name": checkpoint_name,
    }
    return TaskAssignment(
        task_id=task.task_id,
        task_type=task.task_type,
        agent_id=agent.agent_id,
        support_agent_id=task.support_agent_id,
        selection_reason="rl_policy_assignment",
        selection_score=selection_score_for_task(
            task_for_score,
            selected_agent=agent,
            agent_by_id={agent.agent_id: agent},
        ),
        selection_details=selection_details,
    )


def _ordered_usv_agents(agents: tuple[AgentState, ...]) -> tuple[AgentState, ...]:
    usv_agents = [agent for agent in agents if agent.kind == "USV"]
    sort_order = {agent_id: index for index, agent_id in enumerate(USV_AGENT_ORDER)}
    usv_agents.sort(key=lambda agent: (sort_order.get(agent.agent_id, 999), agent.agent_id))
    return tuple(usv_agents)


def _normalize_action_array(action: np.ndarray | list[int] | tuple[int, ...]) -> tuple[int, ...]:
    normalized = np.asarray(action, dtype=np.int64).reshape(-1)
    clipped = np.clip(normalized, 0, USV_ACTION_SLOT_COUNT - 1)
    return tuple(int(value) for value in clipped.tolist())


@lru_cache(maxsize=4)
def _load_rllib_algorithm(checkpoint_path: str):
    import ray.rllib.algorithms.algorithm as algorithm_module
    from usv_uav_marine_coverage.rl.env import register_centralized_usv_allocator_env

    local_storage_dir = (Path.cwd() / "outputs" / "rl" / "ray_results").resolve()
    local_storage_dir.mkdir(parents=True, exist_ok=True)
    algorithm_module.DEFAULT_STORAGE_PATH = str(local_storage_dir)
    register_centralized_usv_allocator_env()
    return algorithm_module.Algorithm.from_checkpoint(checkpoint_path)


def _compute_rllib_action(
    observation: np.ndarray,
    *,
    checkpoint_path: str,
    policy_id: str,
) -> tuple[int, ...]:
    import torch
    from ray.rllib.core.columns import Columns
    from ray.rllib.utils.spaces.space_utils import batch

    algorithm = _load_rllib_algorithm(checkpoint_path)
    rl_module = algorithm.get_module(policy_id)
    if rl_module is None:
        raise ValueError(f"RLlib checkpoint does not expose module {policy_id!r}")
    module_output = rl_module.forward_inference(
        {"obs": torch.from_numpy(batch([observation])).float()}
    )
    action = module_output.get(Columns.ACTIONS)
    if action is None:
        action_dist_inputs = module_output.get(Columns.ACTION_DIST_INPUTS)
        if action_dist_inputs is None:
            raise ValueError("RLlib inference output is missing both ACTIONS and logits")
        action_dist = rl_module.get_inference_action_dist_cls().from_logits(action_dist_inputs)
        action = action_dist.to_deterministic().sample()
    if hasattr(action, "detach"):
        action = action.detach().cpu().numpy()
    action = np.asarray(action)
    if action.ndim >= 2:
        action = action[0]
    return _normalize_action_array(action)


def _encode_task_type(task_type: TaskType | None) -> float:
    mapping = {
        None: 0.0,
        TaskType.BASELINE_SERVICE: 0.25,
        TaskType.HOTSPOT_CONFIRMATION: 0.5,
        TaskType.UAV_RESUPPLY: 0.75,
    }
    return mapping.get(task_type, 1.0)


def _encode_execution_stage(stage: ExecutionStage | None) -> float:
    ordered = [
        None,
        ExecutionStage.PATROL,
        ExecutionStage.GO_TO_TASK,
        ExecutionStage.ON_TASK,
        ExecutionStage.GO_TO_RENDEZVOUS,
        ExecutionStage.ON_RECHARGE,
        ExecutionStage.RETURN_TO_PATROL,
        ExecutionStage.YIELD,
        ExecutionStage.RECOVERY,
        ExecutionStage.FAILED,
    ]
    return ordered.index(stage) / max(len(ordered) - 1, 1)


def _encode_health_status(status: HealthStatus) -> float:
    mapping = {
        HealthStatus.HEALTHY: 0.0,
        HealthStatus.DEGRADED: 0.5,
        HealthStatus.FAILED: 1.0,
    }
    return mapping[status]
