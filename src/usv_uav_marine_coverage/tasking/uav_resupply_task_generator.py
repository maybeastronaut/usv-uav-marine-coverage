"""UAV low-energy rendezvous task generation helpers."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    can_support_uav_resupply,
    estimate_uav_energy_to_point,
    needs_uav_resupply,
)

from .task_types import TaskRecord, TaskSource, TaskStatus, TaskType
from .uav_support_policy import (
    is_uav_usv_meeting_enabled,
    should_delay_uav_resupply_for_initial_escort,
)


def build_uav_resupply_tasks(
    agents: tuple[AgentState, ...],
    *,
    step: int,
    existing_tasks: tuple[TaskRecord, ...] = (),
) -> tuple[TaskRecord, ...]:
    """Sync one low-energy resupply task per UAV when needed."""

    if not is_uav_usv_meeting_enabled():
        return tuple(
            task for task in existing_tasks if task.task_type != TaskType.UAV_RESUPPLY
        )

    existing_by_id = {task.task_id: task for task in existing_tasks}
    usv_agents = tuple(agent for agent in agents if can_support_uav_resupply(agent))
    next_tasks: list[TaskRecord] = []
    active_ids: set[str] = set()

    for agent in agents:
        if agent.kind != "UAV":
            continue
        task_id = f"uav-resupply-{agent.agent_id}"
        existing = existing_by_id.get(task_id)
        should_trigger = needs_uav_resupply(agent)
        support_usv = _nearest_support_usv(agent, usv_agents)
        if support_usv is not None:
            should_trigger = should_trigger or agent.energy_level <= estimate_uav_energy_to_point(
                agent, support_usv.x, support_usv.y
            )
        if should_trigger and should_delay_uav_resupply_for_initial_escort(
            agent,
            usvs=usv_agents,
            step=step,
        ):
            should_trigger = False
        if not should_trigger:
            # Only keep the task record if it has reached a terminal state so
            # that the history is visible to downstream consumers.  Active tasks
            # (PENDING / ASSIGNED / IN_PROGRESS) must be discarded so the UAV
            # is released back to patrol instead of looping endlessly.
            if existing is not None and existing.status in {
                TaskStatus.COMPLETED,
                TaskStatus.CANCELLED,
            }:
                next_tasks.append(existing)
            continue

        active_ids.add(task_id)
        anchor_x, anchor_y = _resolve_rendezvous_anchor(agent, existing=existing)
        if existing is None or existing.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            next_tasks.append(
                TaskRecord(
                    task_id=task_id,
                    task_type=TaskType.UAV_RESUPPLY,
                    source=TaskSource.SYSTEM_LOW_BATTERY,
                    status=TaskStatus.PENDING,
                    priority=20,
                    target_x=anchor_x,
                    target_y=anchor_y,
                    target_row=None,
                    target_col=None,
                    created_step=step,
                    assigned_agent_id=agent.agent_id,
                    rendezvous_anchor_x=anchor_x,
                    rendezvous_anchor_y=anchor_y,
                )
            )
            continue

        if existing.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
            next_tasks.append(
                replace(
                    existing,
                    assigned_agent_id=agent.agent_id,
                    target_x=anchor_x,
                    target_y=anchor_y,
                    rendezvous_anchor_x=anchor_x,
                    rendezvous_anchor_y=anchor_y,
                )
            )
            continue

        next_tasks.append(
            replace(
                existing,
                status=TaskStatus.PENDING,
                assigned_agent_id=agent.agent_id,
                support_agent_id=None,
                completed_step=None,
                target_x=anchor_x,
                target_y=anchor_y,
                rendezvous_anchor_x=anchor_x,
                rendezvous_anchor_y=anchor_y,
            )
        )

    for task_id, task in sorted(existing_by_id.items()):
        if task.task_type != TaskType.UAV_RESUPPLY:
            next_tasks.append(task)
            continue
        if task_id in active_ids:
            continue
        if any(existing.task_id == task_id for existing in next_tasks):
            continue
        next_tasks.append(task)

    return tuple(sorted(next_tasks, key=lambda item: item.task_id))


def _nearest_support_usv(
    uav: AgentState,
    usv_agents: tuple[AgentState, ...],
) -> AgentState | None:
    if not usv_agents:
        return None
    return min(
        usv_agents,
        key=lambda usv: hypot(usv.x - uav.x, usv.y - uav.y),
    )


def _resolve_rendezvous_anchor(
    uav: AgentState,
    *,
    existing: TaskRecord | None,
) -> tuple[float, float]:
    """Return the stable rendezvous anchor for the current low-energy episode."""

    if existing is None:
        return (uav.x, uav.y)
    if existing.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
        if existing.rendezvous_anchor_x is not None and existing.rendezvous_anchor_y is not None:
            return (existing.rendezvous_anchor_x, existing.rendezvous_anchor_y)
        return (existing.target_x, existing.target_y)
    return (uav.x, uav.y)
