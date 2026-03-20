"""Baseline fixed-zone partition policy."""

from __future__ import annotations

from ..task_types import TaskRecord, TaskType
from .partition_types import TaskPartitionView

NEARSHORE_X_END_M = 250.0
OFFSHORE_Y_SPLIT_M = 500.0


def build_baseline_task_partition(task: TaskRecord) -> TaskPartitionView:
    """Return the historical fixed hard-zone partition."""

    if task.task_type == TaskType.UAV_RESUPPLY:
        return TaskPartitionView(
            policy_name="baseline_fixed_partition",
            primary_usv_ids=("USV-1", "USV-2", "USV-3"),
            secondary_usv_ids=(),
            partition_reason="all_usvs_can_support_uav_resupply",
        )
    if task.target_x < NEARSHORE_X_END_M:
        return TaskPartitionView(
            policy_name="baseline_fixed_partition",
            primary_usv_ids=("USV-1",),
            secondary_usv_ids=(),
            partition_reason="nearshore_zone_primary_usv_1",
        )
    if task.target_y < OFFSHORE_Y_SPLIT_M:
        return TaskPartitionView(
            policy_name="baseline_fixed_partition",
            primary_usv_ids=("USV-2",),
            secondary_usv_ids=(),
            partition_reason="upper_offshore_zone_primary_usv_2",
        )
    return TaskPartitionView(
        policy_name="baseline_fixed_partition",
        primary_usv_ids=("USV-3",),
        secondary_usv_ids=(),
        partition_reason="lower_offshore_zone_primary_usv_3",
    )


def baseline_primary_usv_ids_for_task(task: TaskRecord) -> set[str]:
    """Return the baseline fixed-partition primary USV set for one task."""

    return set(build_baseline_task_partition(task).primary_usv_ids)


def baseline_secondary_usv_ids_for_task(task: TaskRecord) -> set[str]:
    """Return the baseline fixed-partition secondary USV set for one task."""

    return set(build_baseline_task_partition(task).secondary_usv_ids)
