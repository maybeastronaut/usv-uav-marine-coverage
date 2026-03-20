"""Backlog-aware task partition policy."""

from __future__ import annotations

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import AgentExecutionState

from ..task_types import TaskRecord, TaskType
from .baseline_fixed import OFFSHORE_Y_SPLIT_M
from .partition_types import TaskPartitionView
from .soft_partition import build_soft_task_partition

BASELINE_BACKLOG_AGE_STEPS = 120
BASELINE_BACKLOG_TASK_THRESHOLD = 2
HOTSPOT_BACKLOG_TASK_THRESHOLD = 4


def build_backlog_aware_task_partition(
    task: TaskRecord,
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    step: int | None,
) -> TaskPartitionView:
    """Return one soft partition adjusted by current backlog pressure."""

    soft_partition = build_soft_task_partition(
        task,
        agents=agents,
        execution_states=execution_states,
        step=step,
    )
    if task.task_type == TaskType.UAV_RESUPPLY:
        return soft_partition

    baseline_backlog = _count_aged_baseline_backlog(tasks, step=step)
    hotspot_backlog = sum(
        candidate.task_type == TaskType.HOTSPOT_CONFIRMATION for candidate in tasks
    )

    if task.task_type == TaskType.BASELINE_SERVICE:
        if baseline_backlog >= BASELINE_BACKLOG_TASK_THRESHOLD:
            secondary_ids = (
                soft_partition.secondary_usv_ids
                if soft_partition.secondary_usv_ids
                else _baseline_backlog_secondary_ids(
                    task, primary_ids=soft_partition.primary_usv_ids
                )
            )
            if soft_partition.secondary_usv_ids:
                return TaskPartitionView(
                    policy_name="backlog_aware_partition_policy",
                    primary_usv_ids=soft_partition.primary_usv_ids,
                    secondary_usv_ids=secondary_ids,
                    partition_reason="backlog_aware_enable_secondary_for_baseline_backlog",
                )
            return TaskPartitionView(
                policy_name="backlog_aware_partition_policy",
                primary_usv_ids=soft_partition.primary_usv_ids,
                secondary_usv_ids=secondary_ids,
                partition_reason="backlog_aware_enable_secondary_for_baseline_backlog",
            )
        return TaskPartitionView(
            policy_name="backlog_aware_partition_policy",
            primary_usv_ids=soft_partition.primary_usv_ids,
            secondary_usv_ids=soft_partition.secondary_usv_ids,
            partition_reason=f"backlog_aware_{soft_partition.partition_reason}",
        )

    if baseline_backlog >= BASELINE_BACKLOG_TASK_THRESHOLD:
        return TaskPartitionView(
            policy_name="backlog_aware_partition_policy",
            primary_usv_ids=soft_partition.primary_usv_ids,
            secondary_usv_ids=(),
            partition_reason="backlog_aware_hold_secondary_for_baseline_backlog",
        )

    if hotspot_backlog >= HOTSPOT_BACKLOG_TASK_THRESHOLD and soft_partition.secondary_usv_ids:
        return TaskPartitionView(
            policy_name="backlog_aware_partition_policy",
            primary_usv_ids=soft_partition.primary_usv_ids,
            secondary_usv_ids=soft_partition.secondary_usv_ids,
            partition_reason="backlog_aware_enable_secondary_for_hotspot_backlog",
        )

    return TaskPartitionView(
        policy_name="backlog_aware_partition_policy",
        primary_usv_ids=soft_partition.primary_usv_ids,
        secondary_usv_ids=soft_partition.secondary_usv_ids,
        partition_reason=f"backlog_aware_{soft_partition.partition_reason}",
    )


def _count_aged_baseline_backlog(tasks: tuple[TaskRecord, ...], *, step: int | None) -> int:
    if step is None:
        return sum(candidate.task_type == TaskType.BASELINE_SERVICE for candidate in tasks)
    count = 0
    for candidate in tasks:
        if candidate.task_type != TaskType.BASELINE_SERVICE:
            continue
        if step - candidate.created_step >= BASELINE_BACKLOG_AGE_STEPS:
            count += 1
    return count


def _baseline_backlog_secondary_ids(
    task: TaskRecord,
    *,
    primary_ids: tuple[str, ...],
) -> tuple[str, ...]:
    if not primary_ids:
        return ()
    primary_id = primary_ids[0]
    if primary_id == "USV-1":
        return ("USV-2",) if task.target_y < OFFSHORE_Y_SPLIT_M else ("USV-3",)
    if primary_id == "USV-2":
        return ("USV-3",)
    if primary_id == "USV-3":
        return ("USV-2",)
    return ()
