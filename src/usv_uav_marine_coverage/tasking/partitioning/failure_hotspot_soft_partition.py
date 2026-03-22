"""Failure-triggered hotspot-first soft partition policy."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    HealthStatus,
    energy_ratio,
    is_operational_agent,
    needs_uav_resupply,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage

from ..task_types import TaskRecord, TaskStatus, TaskType
from .partition_types import TaskPartitionView
from .weighted_voronoi import build_weighted_voronoi_task_partition

FAILURE_SOFT_ACTIVE_TASK_LOAD_PENALTY = 55.0
FAILURE_SOFT_BUSY_STAGE_PENALTY = 110.0
FAILURE_SOFT_RETURN_TO_PATROL_PENALTY = 30.0
FAILURE_SOFT_SUPPORT_RESERVATION_PENALTY = 180.0
FAILURE_SOFT_TASK_SWITCH_PENALTY = 45.0
FAILURE_SOFT_BASELINE_EMERGENCY_PENALTY = 220.0
FAILURE_SOFT_HOTSPOT_SECONDARY_MARGIN = 120.0
FAILURE_SOFT_FOCUS_HOTSPOT_SECONDARY_MARGIN = 220.0
FAILURE_SOFT_CLUSTER_LINK_DISTANCE_M = 180.0
FAILURE_SOFT_MIN_FOCUS_CLUSTER_SIZE = 2
FAILURE_SOFT_AGED_ISOLATED_HOTSPOT_RELEASE_STEPS = 180


def build_failure_hotspot_first_soft_partition(
    task: TaskRecord,
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
) -> TaskPartitionView:
    """Return a failure-mode soft partition when at least one USV has failed."""

    if not _has_failed_usv(agents):
        return build_weighted_voronoi_task_partition(
            task,
            agents=agents,
            execution_states=execution_states,
        )

    usvs = tuple(agent for agent in agents if agent.kind == "USV" and is_operational_agent(agent))
    if task.task_type == TaskType.UAV_RESUPPLY:
        usv_ids = tuple(agent.agent_id for agent in usvs)
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=usv_ids,
            secondary_usv_ids=(),
            partition_reason="failure_mode_all_usvs_can_support_uav_resupply",
        )
    if not usvs:
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=(),
            secondary_usv_ids=(),
            partition_reason="failure_mode_no_available_usv_state",
        )
    if should_suppress_baseline_after_failure(task, tasks=tasks, agents=agents):
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=(),
            secondary_usv_ids=(),
            partition_reason="failure_mode_baseline_suppressed_for_hotspot_backlog",
        )
    if should_suppress_non_focus_hotspot_after_failure(task, tasks=tasks, agents=agents):
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=(),
            secondary_usv_ids=(),
            partition_reason="failure_mode_hotspot_outside_focus_cluster_deferred",
        )

    support_reserved_ids = _failure_mode_support_reserved_usv_ids(agents=agents, tasks=tasks)
    focus_task_ids = failure_hotspot_focus_task_ids(tasks=tasks, agents=agents)
    hotspot_secondary_margin = (
        FAILURE_SOFT_FOCUS_HOTSPOT_SECONDARY_MARGIN
        if task.task_id in focus_task_ids
        else FAILURE_SOFT_HOTSPOT_SECONDARY_MARGIN
    )
    scored_candidates = sorted(
        (
            (
                _failure_soft_partition_cost(
                    agent,
                    task=task,
                    execution_state=execution_states.get(agent.agent_id),
                    tasks=tasks,
                    support_reserved_ids=support_reserved_ids,
                ),
                _agent_failure_mode_load(
                    agent.agent_id,
                    tasks=tasks,
                    execution_state=execution_states.get(agent.agent_id),
                ),
                agent.agent_id,
            )
            for agent in usvs
        ),
        key=lambda item: (item[0], item[1], item[2]),
    )

    primary_cost, primary_load, primary_id = scored_candidates[0]
    if len(scored_candidates) == 1:
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(),
            partition_reason="failure_mode_single_survivor_primary_only",
        )

    secondary_cost, secondary_load, secondary_id = scored_candidates[1]
    if (
        task.task_type == TaskType.HOTSPOT_CONFIRMATION
        and secondary_cost <= primary_cost + hotspot_secondary_margin
        and (secondary_load < primary_load or task.task_id in focus_task_ids)
    ):
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(secondary_id,),
            partition_reason=(
                "failure_mode_hotspot_focus_cluster_secondary"
                if task.task_id in focus_task_ids
                else "failure_mode_hotspot_secondary_load_balance"
            ),
        )

    if task.task_type == TaskType.HOTSPOT_CONFIRMATION:
        return TaskPartitionView(
            policy_name="failure_triggered_hotspot_first_soft_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(),
            partition_reason="failure_mode_hotspot_primary_only",
        )

    return TaskPartitionView(
        policy_name="failure_triggered_hotspot_first_soft_partition_policy",
        primary_usv_ids=(primary_id,),
        secondary_usv_ids=(),
        partition_reason="failure_mode_baseline_deferred",
    )


def _has_failed_usv(agents: tuple[AgentState, ...]) -> bool:
    return any(
        agent.kind == "USV" and agent.health_status == HealthStatus.FAILED
        for agent in agents
    )


def failure_hotspot_emergency_active(
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
) -> bool:
    """Return whether failure-mode hotspot suppression should be active now."""

    return _has_failed_usv(agents) and _has_hotspot_backlog(tasks)


def should_suppress_baseline_after_failure(
    task: TaskRecord,
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
) -> bool:
    """Return whether one baseline task should yield to hotspot backlog."""

    return (
        task.task_type == TaskType.BASELINE_SERVICE
        and failure_hotspot_emergency_active(tasks=tasks, agents=agents)
    )


def should_suppress_non_focus_hotspot_after_failure(
    task: TaskRecord,
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
) -> bool:
    """Return whether one hotspot task should yield to the active focus cluster."""

    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return False
    hotspot_tasks = tuple(
        candidate
        for candidate in tasks
        if candidate.task_type == TaskType.HOTSPOT_CONFIRMATION
        and candidate.status not in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}
    )
    clusters = _build_hotspot_clusters(hotspot_tasks)
    focus_task_ids = _focus_task_ids_from_clusters(
        clusters=clusters,
        tasks=hotspot_tasks,
        agents=agents,
    )
    if not focus_task_ids or task.task_id in focus_task_ids:
        return False
    return not _should_release_aged_isolated_hotspot(
        task,
        clusters=clusters,
        focus_task_ids=focus_task_ids,
        tasks=hotspot_tasks,
    )


def failure_hotspot_focus_task_ids(
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
) -> frozenset[str]:
    """Return the current hotspot focus cluster after one USV failure."""

    if not failure_hotspot_emergency_active(tasks=tasks, agents=agents):
        return frozenset()

    hotspot_tasks = tuple(
        task
        for task in tasks
        if task.task_type == TaskType.HOTSPOT_CONFIRMATION
        and task.status not in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}
    )
    clusters = _build_hotspot_clusters(hotspot_tasks)
    return _focus_task_ids_from_clusters(clusters=clusters, tasks=hotspot_tasks, agents=agents)


def _has_hotspot_backlog(tasks: tuple[TaskRecord, ...]) -> bool:
    return any(
        task.task_type == TaskType.HOTSPOT_CONFIRMATION
        and task.status not in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}
        for task in tasks
    )


def _build_hotspot_clusters(tasks: tuple[TaskRecord, ...]) -> tuple[tuple[TaskRecord, ...], ...]:
    remaining = list(tasks)
    clusters: list[tuple[TaskRecord, ...]] = []
    while remaining:
        seed = remaining.pop()
        cluster = [seed]
        changed = True
        while changed:
            changed = False
            still_remaining: list[TaskRecord] = []
            for candidate in remaining:
                if any(
                    hypot(candidate.target_x - task.target_x, candidate.target_y - task.target_y)
                    <= FAILURE_SOFT_CLUSTER_LINK_DISTANCE_M
                    for task in cluster
                ):
                    cluster.append(candidate)
                    changed = True
                else:
                    still_remaining.append(candidate)
            remaining = still_remaining
        clusters.append(tuple(cluster))
    return tuple(clusters)


def _focus_task_ids_from_clusters(
    *,
    clusters: tuple[tuple[TaskRecord, ...], ...],
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
) -> frozenset[str]:
    if not failure_hotspot_emergency_active(tasks=tasks, agents=agents):
        return frozenset()
    if not clusters:
        return frozenset()
    valid_clusters = tuple(
        cluster for cluster in clusters if len(cluster) >= FAILURE_SOFT_MIN_FOCUS_CLUSTER_SIZE
    )
    if not valid_clusters:
        return frozenset()
    focus_cluster = max(
        valid_clusters,
        key=lambda cluster: (
            len(cluster),
            round(sum(task.target_x for task in cluster) / len(cluster), 3),
            round(sum(task.target_y for task in cluster) / len(cluster), 3),
            -min(task.created_step for task in cluster),
        ),
    )
    return frozenset(task.task_id for task in focus_cluster)


def _should_release_aged_isolated_hotspot(
    task: TaskRecord,
    *,
    clusters: tuple[tuple[TaskRecord, ...], ...],
    focus_task_ids: frozenset[str],
    tasks: tuple[TaskRecord, ...],
) -> bool:
    """Return whether one old isolated hotspot can escape focus-cluster suppression."""

    if task.status not in {TaskStatus.PENDING, TaskStatus.REQUEUED}:
        return False
    task_cluster = next(
        (
            cluster
            for cluster in clusters
            if any(candidate.task_id == task.task_id for candidate in cluster)
        ),
        (),
    )
    if len(task_cluster) != 1:
        return False
    focus_tasks = tuple(candidate for candidate in tasks if candidate.task_id in focus_task_ids)
    if not focus_tasks:
        return False
    newest_focus_created_step = max(candidate.created_step for candidate in focus_tasks)
    return (
        task.created_step + FAILURE_SOFT_AGED_ISOLATED_HOTSPOT_RELEASE_STEPS
        <= newest_focus_created_step
    )


def _failure_soft_partition_cost(
    agent: AgentState,
    *,
    task: TaskRecord,
    execution_state: AgentExecutionState | None,
    tasks: tuple[TaskRecord, ...],
    support_reserved_ids: set[str],
) -> float:
    cost = hypot(agent.x - task.target_x, agent.y - task.target_y)
    cost += (
        _agent_failure_mode_load(
            agent.agent_id,
            tasks=tasks,
            execution_state=execution_state,
        )
        * FAILURE_SOFT_ACTIVE_TASK_LOAD_PENALTY
    )
    cost += _busy_stage_penalty(execution_state)
    if agent.agent_id in support_reserved_ids:
        cost += FAILURE_SOFT_SUPPORT_RESERVATION_PENALTY
    if execution_state is not None and execution_state.active_task_id not in {None, task.task_id}:
        cost += FAILURE_SOFT_TASK_SWITCH_PENALTY
    if task.task_type == TaskType.BASELINE_SERVICE:
        cost += FAILURE_SOFT_BASELINE_EMERGENCY_PENALTY
    return round(cost, 3)


def _agent_failure_mode_load(
    agent_id: str,
    *,
    tasks: tuple[TaskRecord, ...],
    execution_state: AgentExecutionState | None,
) -> int:
    active_task_count = sum(
        1
        for task in tasks
        if task.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
        and (task.assigned_agent_id == agent_id or task.support_agent_id == agent_id)
    )
    if execution_state is None:
        return active_task_count
    if execution_state.active_task_id is None:
        return active_task_count
    return max(active_task_count, 1)


def _busy_stage_penalty(execution_state: AgentExecutionState | None) -> float:
    if execution_state is None:
        return FAILURE_SOFT_BUSY_STAGE_PENALTY
    if execution_state.stage == ExecutionStage.PATROL and execution_state.active_task_id is None:
        return 0.0
    if execution_state.stage == ExecutionStage.RETURN_TO_PATROL:
        return FAILURE_SOFT_RETURN_TO_PATROL_PENALTY
    return FAILURE_SOFT_BUSY_STAGE_PENALTY


def _failure_mode_support_reserved_usv_ids(
    *,
    agents: tuple[AgentState, ...],
    tasks: tuple[TaskRecord, ...],
) -> set[str]:
    reserved_ids = {
        task.support_agent_id
        for task in tasks
        if task.task_type == TaskType.UAV_RESUPPLY
        and task.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
        and task.support_agent_id is not None
    }
    operational_usvs = tuple(
        agent for agent in agents if agent.kind == "USV" and is_operational_agent(agent)
    )
    if not operational_usvs:
        return set(reserved_ids)

    for agent in agents:
        if agent.kind != "UAV":
            continue
        if not needs_uav_resupply(agent) and energy_ratio(agent) > 0.4:
            continue
        nearest_usv = min(
            operational_usvs,
            key=lambda usv: hypot(agent.x - usv.x, agent.y - usv.y),
        )
        reserved_ids.add(nearest_usv.agent_id)
    return reserved_ids
