"""Baseline task generation helpers for future extensions."""

from __future__ import annotations

from usv_uav_marine_coverage.information_map import InformationMap

from .task_types import TaskRecord


def build_baseline_tasks(
    info_map: InformationMap,
    *,
    step: int,
    existing_tasks: tuple[TaskRecord, ...] = (),
) -> tuple[TaskRecord, ...]:
    """Return baseline tasks for the current step.

    The first minimal closed-loop implementation is hotspot-only, so this helper
    currently keeps the baseline branch explicit while returning no active tasks.
    """

    del info_map, step, existing_tasks
    return ()
