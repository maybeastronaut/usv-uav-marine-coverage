"""Light-weight soft task partition policy."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState, ExecutionStage

from ..task_types import TaskRecord, TaskType
from .baseline_fixed import OFFSHORE_Y_SPLIT_M, build_baseline_task_partition
from .partition_types import TaskPartitionView

SOFT_PARTITION_DISTANCE_ADVANTAGE_M = 80.0
SOFT_PARTITION_AGE_THRESHOLD_STEPS = 90


def build_soft_task_partition(
    task: TaskRecord,
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    step: int | None,
) -> TaskPartitionView:
    """Return one light-weight soft partition with a conditional secondary bidder."""

    baseline = build_baseline_task_partition(task)
    if task.task_type == TaskType.UAV_RESUPPLY:
        return baseline

    primary_id = baseline.primary_usv_ids[0]
    secondary_id = _default_secondary_usv_id(task, primary_id=primary_id)
    if secondary_id is None:
        return baseline

    agent_by_id = {agent.agent_id: agent for agent in agents}
    primary_agent = agent_by_id.get(primary_id)
    secondary_agent = agent_by_id.get(secondary_id)
    if primary_agent is None or secondary_agent is None:
        return TaskPartitionView(
            policy_name="soft_partition_policy",
            primary_usv_ids=baseline.primary_usv_ids,
            secondary_usv_ids=(),
            partition_reason="soft_partition_missing_agent_state_fallback_to_baseline",
        )

    primary_available = _is_available_for_new_assignment(execution_states.get(primary_id))
    secondary_available = _is_available_for_new_assignment(execution_states.get(secondary_id))
    if not secondary_available:
        return TaskPartitionView(
            policy_name="soft_partition_policy",
            primary_usv_ids=baseline.primary_usv_ids,
            secondary_usv_ids=(),
            partition_reason="soft_partition_secondary_unavailable_fallback_to_baseline",
        )

    primary_distance = _distance_to_task(primary_agent, task)
    secondary_distance = _distance_to_task(secondary_agent, task)
    task_age = 0 if step is None else max(step - task.created_step, 0)

    if not primary_available:
        return TaskPartitionView(
            policy_name="soft_partition_policy",
            primary_usv_ids=(secondary_id,),
            secondary_usv_ids=(primary_id,),
            partition_reason="soft_partition_promote_secondary_when_primary_unavailable",
        )

    if secondary_distance + SOFT_PARTITION_DISTANCE_ADVANTAGE_M < primary_distance:
        return TaskPartitionView(
            policy_name="soft_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(secondary_id,),
            partition_reason="soft_partition_enable_secondary_for_distance_advantage",
        )

    if task_age >= SOFT_PARTITION_AGE_THRESHOLD_STEPS:
        return TaskPartitionView(
            policy_name="soft_partition_policy",
            primary_usv_ids=(primary_id,),
            secondary_usv_ids=(secondary_id,),
            partition_reason="soft_partition_enable_secondary_for_task_age",
        )

    return TaskPartitionView(
        policy_name="soft_partition_policy",
        primary_usv_ids=(primary_id,),
        secondary_usv_ids=(),
        partition_reason="soft_partition_keep_primary_only",
    )


def _default_secondary_usv_id(task: TaskRecord, *, primary_id: str) -> str | None:
    if primary_id == "USV-1":
        return "USV-2" if task.target_y < OFFSHORE_Y_SPLIT_M else "USV-3"
    if primary_id == "USV-2":
        return "USV-3"
    if primary_id == "USV-3":
        return "USV-2"
    return None


def _distance_to_task(agent: AgentState, task: TaskRecord) -> float:
    return round(hypot(agent.x - task.target_x, agent.y - task.target_y), 3)


def _is_available_for_new_assignment(execution_state: AgentExecutionState | None) -> bool:
    if execution_state is None:
        return False
    return execution_state.active_task_id is None and execution_state.stage == ExecutionStage.PATROL
