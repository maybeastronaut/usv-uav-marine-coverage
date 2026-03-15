"""Baseline task generation helpers."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.information_map import InformationMap

from .task_types import TaskRecord, TaskSource, TaskStatus, TaskType


def build_baseline_tasks(
    info_map: InformationMap,
    *,
    step: int,
    existing_tasks: tuple[TaskRecord, ...] = (),
) -> tuple[TaskRecord, ...]:
    """Sync baseline-service tasks from the current information map."""

    existing_by_id = {task.task_id: task for task in existing_tasks}
    next_tasks: list[TaskRecord] = []
    active_ids: set[str] = set()

    for cell in info_map.grid_map.flat_cells:
        state = info_map.state_at(cell.row, cell.col)
        if not state.has_baseline_task:
            continue

        task_id = f"baseline-service-{cell.row}-{cell.col}"
        active_ids.add(task_id)
        existing = existing_by_id.get(task_id)
        if existing is None or existing.status in {
            TaskStatus.COMPLETED,
            TaskStatus.CANCELLED,
        }:
            next_tasks.append(
                TaskRecord(
                    task_id=task_id,
                    task_type=TaskType.BASELINE_SERVICE,
                    source=TaskSource.SYSTEM_BASELINE_TIMEOUT,
                    status=TaskStatus.PENDING,
                    priority=max(1, state.baseline_task_priority),
                    target_x=cell.center_x,
                    target_y=cell.center_y,
                    target_row=cell.row,
                    target_col=cell.col,
                    created_step=step,
                )
            )
            continue

        next_tasks.append(existing)

    for task_id, task in sorted(existing_by_id.items()):
        if task.task_type != TaskType.BASELINE_SERVICE:
            next_tasks.append(task)
            continue
        if task_id in active_ids:
            continue
        if any(existing.task_id == task_id for existing in next_tasks):
            continue
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            next_tasks.append(task)
            continue
        next_tasks.append(replace(task, status=TaskStatus.COMPLETED, completed_step=step))

    return tuple(sorted(next_tasks, key=lambda item: item.task_id))
