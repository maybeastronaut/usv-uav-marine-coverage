"""Hotspot-confirmation task generation helpers."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.information_map import HotspotKnowledgeState, InformationMap

from .task_types import TaskRecord, TaskSource, TaskStatus, TaskType


def sync_hotspot_confirmation_tasks(
    info_map: InformationMap,
    *,
    step: int,
    existing_tasks: tuple[TaskRecord, ...] = (),
) -> tuple[TaskRecord, ...]:
    """Sync hotspot-confirmation tasks from the current information map."""

    existing_by_id = {task.task_id: task for task in existing_tasks}
    next_tasks: list[TaskRecord] = []
    active_ids: set[str] = set()

    for cell in info_map.grid_map.flat_cells:
        state = info_map.state_at(cell.row, cell.col)
        task_id = f"hotspot-confirmation-{cell.row}-{cell.col}"
        existing = existing_by_id.get(task_id)

        if state.known_hotspot_state == HotspotKnowledgeState.SUSPECTED:
            active_ids.add(task_id)
            if existing is None:
                next_tasks.append(
                    TaskRecord(
                        task_id=task_id,
                        task_type=TaskType.HOTSPOT_CONFIRMATION,
                        source=_task_source_for_observer(state.suspected_by),
                        status=TaskStatus.PENDING,
                        priority=10,
                        target_x=cell.center_x,
                        target_y=cell.center_y,
                        target_row=cell.row,
                        target_col=cell.col,
                        created_step=step,
                    )
                )
                continue

            next_tasks.append(existing)
            continue

        if existing is None:
            continue

        if state.known_hotspot_state in {
            HotspotKnowledgeState.CONFIRMED,
            HotspotKnowledgeState.FALSE_ALARM,
        }:
            next_tasks.append(
                replace(
                    existing,
                    status=TaskStatus.COMPLETED,
                    completed_step=step,
                )
            )
            continue

        if existing.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            next_tasks.append(existing)
            continue

        next_tasks.append(
            replace(
                existing,
                status=TaskStatus.CANCELLED,
                completed_step=step,
            )
        )

    for task_id, task in sorted(existing_by_id.items()):
        if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
            next_tasks.append(task)
            continue
        if task_id in active_ids:
            continue
        if any(existing.task_id == task_id for existing in next_tasks):
            continue
        next_tasks.append(task)

    return tuple(sorted(next_tasks, key=lambda item: item.task_id))


def _task_source_for_observer(observer_id: str | None) -> TaskSource:
    if observer_id is not None and observer_id.startswith("USV"):
        return TaskSource.USV_ANOMALY
    return TaskSource.UAV_SUSPECTED
