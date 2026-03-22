"""Light-weight weighted Voronoi task partition policy."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    energy_ratio,
    is_operational_agent,
    needs_uav_resupply,
)
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage

from ..task_types import TaskRecord, TaskType
from .partition_types import TaskPartitionView

WEIGHTED_VORONOI_BUSY_PENALTY = 140.0
WEIGHTED_VORONOI_SUPPORT_PENALTY = 180.0
WEIGHTED_VORONOI_SUPPORT_RATIO = 0.4
WEIGHTED_VORONOI_SECONDARY_MARGIN = 120.0


def build_weighted_voronoi_task_partition(
    task: TaskRecord,
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
) -> TaskPartitionView:
    """Return one primary+secondary partition view based on weighted distance."""

    usvs = tuple(agent for agent in agents if agent.kind == "USV" and is_operational_agent(agent))
    if task.task_type == TaskType.UAV_RESUPPLY:
        usv_ids = tuple(agent.agent_id for agent in usvs)
        return TaskPartitionView(
            policy_name="weighted_voronoi_partition_policy",
            primary_usv_ids=usv_ids,
            secondary_usv_ids=(),
            partition_reason="weighted_voronoi_all_usvs_can_support_uav_resupply",
        )

    protected_support_ids = _protected_support_usv_ids(agents)
    scored_candidates = []
    for agent in usvs:
        execution_state = execution_states.get(agent.agent_id)
        if execution_state is None:
            continue
        scored_candidates.append(
            (
                _weighted_partition_cost(
                    agent,
                    task=task,
                    execution_state=execution_state,
                    protected_support_ids=protected_support_ids,
                ),
                agent.agent_id,
            )
        )

    if not scored_candidates:
        return TaskPartitionView(
            policy_name="weighted_voronoi_partition_policy",
            primary_usv_ids=(),
            secondary_usv_ids=(),
            partition_reason="weighted_voronoi_no_available_usv_state",
        )

    scored_candidates.sort(key=lambda item: (item[0], item[1]))
    primary_cost, primary_id = scored_candidates[0]
    if len(scored_candidates) == 1:
        return TaskPartitionView(
            policy_name="weighted_voronoi_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(),
            partition_reason="weighted_voronoi_primary_only_single_candidate",
        )

    secondary_cost, secondary_id = scored_candidates[1]
    if secondary_cost <= primary_cost + WEIGHTED_VORONOI_SECONDARY_MARGIN:
        return TaskPartitionView(
            policy_name="weighted_voronoi_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(secondary_id,),
            partition_reason="weighted_voronoi_open_secondary_within_margin",
        )

    return TaskPartitionView(
        policy_name="weighted_voronoi_partition_policy",
        primary_usv_ids=(primary_id,),
        secondary_usv_ids=(),
        partition_reason="weighted_voronoi_keep_primary_only_outside_margin",
    )


def _weighted_partition_cost(
    agent: AgentState,
    *,
    task: TaskRecord,
    execution_state: AgentExecutionState,
    protected_support_ids: set[str],
) -> float:
    distance_cost = hypot(agent.x - task.target_x, agent.y - task.target_y)
    load_penalty = (
        0.0
        if execution_state.stage == ExecutionStage.PATROL and execution_state.active_task_id is None
        else WEIGHTED_VORONOI_BUSY_PENALTY
    )
    support_penalty = (
        0.0 if agent.agent_id not in protected_support_ids else WEIGHTED_VORONOI_SUPPORT_PENALTY
    )
    return round(distance_cost + load_penalty + support_penalty, 3)


def _protected_support_usv_ids(agents: tuple[AgentState, ...]) -> set[str]:
    usvs = tuple(agent for agent in agents if agent.kind == "USV" and is_operational_agent(agent))
    if not usvs:
        return set()

    protected: set[str] = set()
    for agent in agents:
        if agent.kind != "UAV":
            continue
        if not needs_uav_resupply(agent) and energy_ratio(agent) > WEIGHTED_VORONOI_SUPPORT_RATIO:
            continue
        nearest_usv = min(usvs, key=lambda usv: hypot(agent.x - usv.x, agent.y - usv.y))
        protected.add(nearest_usv.agent_id)
    return protected
