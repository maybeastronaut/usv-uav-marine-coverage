"""Event runtime helpers for damage-triggered replanning."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.agent_model import AgentState, HealthStatus
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionStage,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskStatus, TaskType

from .event_types import AgentDamageEvent, AppliedEvent, EventType


def apply_scheduled_events(
    *,
    step: int,
    scheduled_events: tuple[AgentDamageEvent, ...],
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    task_records: tuple[TaskRecord, ...],
) -> tuple[
    tuple[AgentState, ...],
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
    tuple[TaskRecord, ...],
    tuple[AppliedEvent, ...],
    tuple[dict[str, object], ...],
]:
    """Apply all events scheduled for the current step."""

    triggered_events = tuple(event for event in scheduled_events if event.step == step)
    if not triggered_events:
        return (
            agents,
            execution_states,
            progress_states,
            task_records,
            (),
            (),
        )

    next_agents = {agent.agent_id: agent for agent in agents}
    next_execution_states = dict(execution_states)
    next_progress_states = dict(progress_states)
    next_task_records = task_records
    applied: list[AppliedEvent] = []
    reassignments: list[dict[str, object]] = []

    for event in triggered_events:
        agent = next_agents.get(event.agent_id)
        if agent is None or agent.kind != "USV":
            continue
        if agent.health_status == HealthStatus.FAILED:
            continue

        if event.event_type == EventType.AGENT_FAILURE:
            failed_agent = replace(
                agent,
                health_status=HealthStatus.FAILED,
                is_operational=False,
                speed_multiplier=0.0,
                turn_rate_multiplier=0.0,
                speed_mps=0.0,
                turn_rate_degps=0.0,
                cruise_speed_mps=0.0,
                max_speed_mps=0.0,
                max_turn_rate_degps=0.0,
            )
            next_agents[event.agent_id] = failed_agent
            execution_state = next_execution_states.get(event.agent_id)
            if execution_state is not None:
                next_execution_states[event.agent_id] = replace(
                    execution_state,
                    stage=ExecutionStage.FAILED,
                    active_plan=None,
                    active_task_id=None,
                    current_waypoint_index=0,
                )
            progress_state = next_progress_states.get(event.agent_id)
            if progress_state is not None:
                next_progress_states[event.agent_id] = replace(
                    progress_state,
                    stalled_steps=0,
                    recovery_attempts=0,
                    recovery_step_index=0,
                )
            next_task_records, released = release_agent_tasks(
                task_records=next_task_records,
                agent_id=event.agent_id,
                step=step,
            )
            reassignments.extend(released)
            applied.append(
                AppliedEvent(
                    step=step,
                    event_type=event.event_type,
                    agent_id=event.agent_id,
                    summary=f"{event.agent_id} failed and was removed from assignment candidates",
                )
            )
            continue

        profile = agent.platform_profile
        if event.event_type == EventType.SPEED_DEGRADATION:
            speed_multiplier = max(0.05, min(event.speed_multiplier, 1.0))
            degraded_agent = replace(
                agent,
                health_status=HealthStatus.DEGRADED,
                speed_multiplier=speed_multiplier,
                cruise_speed_mps=profile.cruise_speed_mps * speed_multiplier,
                max_speed_mps=profile.max_speed_mps * speed_multiplier,
                speed_mps=min(agent.speed_mps, profile.max_speed_mps * speed_multiplier),
            )
            next_agents[event.agent_id] = degraded_agent
            if event.agent_id in next_execution_states:
                next_execution_states[event.agent_id] = replace(
                    next_execution_states[event.agent_id],
                    active_plan=None,
                    current_waypoint_index=0,
                )
            next_task_records, released = release_uav_resupply_support_tasks(
                task_records=next_task_records,
                support_agent_id=event.agent_id,
                step=step,
            )
            reassignments.extend(released)
            applied.append(
                AppliedEvent(
                    step=step,
                    event_type=event.event_type,
                    agent_id=event.agent_id,
                    summary=(
                        f"{event.agent_id} speed degraded to {speed_multiplier:.2f}x "
                        "and its local/global path execution was refreshed"
                    ),
                )
            )
            continue

        if event.event_type == EventType.TURN_RATE_DEGRADATION:
            turn_multiplier = max(0.05, min(event.turn_rate_multiplier, 1.0))
            degraded_agent = replace(
                agent,
                health_status=HealthStatus.DEGRADED,
                turn_rate_multiplier=turn_multiplier,
                max_turn_rate_degps=profile.max_turn_rate_degps * turn_multiplier,
                turn_rate_degps=min(
                    agent.turn_rate_degps,
                    profile.max_turn_rate_degps * turn_multiplier,
                ),
            )
            next_agents[event.agent_id] = degraded_agent
            if event.agent_id in next_execution_states:
                next_execution_states[event.agent_id] = replace(
                    next_execution_states[event.agent_id],
                    active_plan=None,
                    current_waypoint_index=0,
                )
            next_task_records, released = release_uav_resupply_support_tasks(
                task_records=next_task_records,
                support_agent_id=event.agent_id,
                step=step,
            )
            reassignments.extend(released)
            applied.append(
                AppliedEvent(
                    step=step,
                    event_type=event.event_type,
                    agent_id=event.agent_id,
                    summary=(
                        f"{event.agent_id} turn-rate degraded to {turn_multiplier:.2f}x "
                        "and its local/global path execution was refreshed"
                    ),
                )
            )

    updated_agents = tuple(next_agents[agent.agent_id] for agent in agents)
    return (
        updated_agents,
        next_execution_states,
        next_progress_states,
        next_task_records,
        tuple(applied),
        tuple(reassignments),
    )


def release_agent_tasks(
    *,
    task_records: tuple[TaskRecord, ...],
    agent_id: str,
    step: int,
) -> tuple[tuple[TaskRecord, ...], tuple[dict[str, object], ...]]:
    """Release all in-flight tasks currently owned by one failed agent."""

    released_records: list[TaskRecord] = []
    released_events: list[dict[str, object]] = []
    for task in task_records:
        if task.assigned_agent_id != agent_id and task.support_agent_id != agent_id:
            released_records.append(task)
            continue
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            released_records.append(task)
            continue

        released_records.append(
            replace(
                task,
                status=TaskStatus.REQUEUED,
                assigned_agent_id=(
                    None if task.assigned_agent_id == agent_id else task.assigned_agent_id
                ),
                support_agent_id=None,
                retry_after_step=step,
            )
        )
        released_events.append(
            {
                "step": step,
                "agent_id": agent_id,
                "task_id": task.task_id,
                "task_type": task.task_type.value,
                "reason": "agent_failure_requeue",
            }
        )
    return (tuple(released_records), tuple(released_events))


def release_uav_resupply_support_tasks(
    *,
    task_records: tuple[TaskRecord, ...],
    support_agent_id: str,
    step: int,
) -> tuple[tuple[TaskRecord, ...], tuple[dict[str, object], ...]]:
    """Release UAV-resupply tasks whose support USV is no longer safe to recharge on."""

    released_records: list[TaskRecord] = []
    released_events: list[dict[str, object]] = []
    for task in task_records:
        if task.task_type != TaskType.UAV_RESUPPLY or task.support_agent_id != support_agent_id:
            released_records.append(task)
            continue
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            released_records.append(task)
            continue

        released_records.append(
            replace(
                task,
                status=TaskStatus.REQUEUED,
                assigned_agent_id=None,
                support_agent_id=None,
                retry_after_step=step,
            )
        )
        released_events.append(
            {
                "step": step,
                "agent_id": support_agent_id,
                "task_id": task.task_id,
                "task_type": task.task_type.value,
                "reason": "degraded_support_usv_requeue",
            }
        )
    return (tuple(released_records), tuple(released_events))
