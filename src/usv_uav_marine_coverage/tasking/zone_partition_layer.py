"""Backward-compatible wrapper for the task partitioning layer."""

from .partitioning import (
    NEARSHORE_X_END_M,
    OFFSHORE_Y_SPLIT_M,
    TaskPartitionView,
    baseline_primary_usv_ids_for_task,
    baseline_secondary_usv_ids_for_task,
    build_backlog_aware_task_partition,
    build_baseline_task_partition,
    build_soft_task_partition,
    build_task_partition,
    build_weighted_voronoi_task_partition,
)

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
