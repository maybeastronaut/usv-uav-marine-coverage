"""Partitioning layer for task-to-USV responsibility policies."""

from __future__ import annotations

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState

from ..task_types import TaskRecord
from .backlog_aware import build_backlog_aware_task_partition
from .baseline_fixed import (
    NEARSHORE_X_END_M,
    OFFSHORE_Y_SPLIT_M,
    baseline_primary_usv_ids_for_task,
    baseline_secondary_usv_ids_for_task,
    build_baseline_task_partition,
)
from .partition_types import TaskPartitionView
from .soft_partition import build_soft_task_partition
from .weighted_voronoi import build_weighted_voronoi_task_partition


def build_task_partition(
    task: TaskRecord,
    *,
    policy_name: str = "baseline_fixed_partition",
    tasks: tuple[TaskRecord, ...] = (),
    agents: tuple[AgentState, ...] = (),
    execution_states: dict[str, AgentExecutionState] | None = None,
    step: int | None = None,
) -> TaskPartitionView:
    """Return one explicit partition view for the requested policy."""

    if policy_name == "baseline_fixed_partition":
        return build_baseline_task_partition(task)
    if policy_name == "soft_partition_policy":
        return build_soft_task_partition(
            task,
            agents=agents,
            execution_states=execution_states or {},
            step=step,
        )
    if policy_name == "backlog_aware_partition_policy":
        return build_backlog_aware_task_partition(
            task,
            tasks=tasks,
            agents=agents,
            execution_states=execution_states or {},
            step=step,
        )
    if policy_name == "weighted_voronoi_partition_policy":
        return build_weighted_voronoi_task_partition(
            task,
            agents=agents,
            execution_states=execution_states or {},
        )
    raise ValueError(f"Unsupported zone partition policy {policy_name!r}")


__all__ = [
    "NEARSHORE_X_END_M",
    "OFFSHORE_Y_SPLIT_M",
    "TaskPartitionView",
    "baseline_primary_usv_ids_for_task",
    "baseline_secondary_usv_ids_for_task",
    "build_backlog_aware_task_partition",
    "build_baseline_task_partition",
    "build_soft_task_partition",
    "build_task_partition",
    "build_weighted_voronoi_task_partition",
]
