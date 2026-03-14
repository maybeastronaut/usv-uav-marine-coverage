"""Waypoint and path-following helpers."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
)

from .execution_types import AgentExecutionState, ExecutionOutcome, ExecutionStage


def follow_path_step(
    agent: AgentState,
    execution_state: AgentExecutionState,
    *,
    dt_seconds: float,
) -> tuple[AgentState, AgentExecutionState, ExecutionOutcome]:
    """Advance one agent by one step along the currently active plan."""

    plan = execution_state.active_plan
    if plan is None or not plan.waypoints:
        if execution_state.stage == ExecutionStage.ON_TASK:
            return (
                assign_agent_task(agent, _mode_for_stage(agent.kind, execution_state.stage)),
                execution_state,
                ExecutionOutcome.ADVANCING,
            )
        return (agent, execution_state, ExecutionOutcome.FAILED)

    waypoint_index = _skip_reached_waypoints(agent, execution_state)
    skipped_waypoint = waypoint_index > execution_state.current_waypoint_index
    if waypoint_index >= len(plan.waypoints):
        return (
            assign_agent_task(agent, _mode_for_stage(agent.kind, execution_state.stage)),
            replace(execution_state, current_waypoint_index=waypoint_index),
            _final_outcome_for_stage(execution_state.stage),
        )

    waypoint = plan.waypoints[waypoint_index]
    tracked_agent = assign_agent_task(
        agent,
        _mode_for_stage(agent.kind, execution_state.stage),
        waypoint.x,
        waypoint.y,
    )
    advanced_agent = advance_agent_towards_task(tracked_agent, dt_seconds)
    next_waypoint_index = waypoint_index
    if not advanced_agent.task.has_target:
        next_waypoint_index += 1

    updated_state = replace(execution_state, current_waypoint_index=next_waypoint_index)
    if next_waypoint_index >= len(plan.waypoints):
        return (
            advanced_agent,
            updated_state,
            _final_outcome_for_stage(execution_state.stage),
        )
    if next_waypoint_index > waypoint_index:
        return (advanced_agent, updated_state, ExecutionOutcome.WAYPOINT_REACHED)
    if skipped_waypoint:
        return (advanced_agent, updated_state, ExecutionOutcome.WAYPOINT_REACHED)
    return (advanced_agent, updated_state, ExecutionOutcome.ADVANCING)


def _skip_reached_waypoints(agent: AgentState, execution_state: AgentExecutionState) -> int:
    assert execution_state.active_plan is not None
    waypoint_index = execution_state.current_waypoint_index
    while waypoint_index < len(execution_state.active_plan.waypoints):
        waypoint = execution_state.active_plan.waypoints[waypoint_index]
        if hypot(waypoint.x - agent.x, waypoint.y - agent.y) > agent.arrival_tolerance_m:
            break
        waypoint_index += 1
    return waypoint_index


def _mode_for_stage(agent_kind: str, stage: ExecutionStage) -> TaskMode:
    if stage == ExecutionStage.ON_TASK:
        return TaskMode.CONFIRM if agent_kind == "USV" else TaskMode.INVESTIGATE
    if stage == ExecutionStage.GO_TO_TASK:
        return TaskMode.CONFIRM if agent_kind == "USV" else TaskMode.INVESTIGATE
    if agent_kind == "UAV":
        return TaskMode.INVESTIGATE
    return TaskMode.PATROL


def _final_outcome_for_stage(stage: ExecutionStage) -> ExecutionOutcome:
    if stage == ExecutionStage.GO_TO_TASK:
        return ExecutionOutcome.TASK_SITE_REACHED
    if stage == ExecutionStage.RETURN_TO_PATROL:
        return ExecutionOutcome.PATROL_REJOINED
    if stage == ExecutionStage.PATROL:
        return ExecutionOutcome.WAYPOINT_REACHED
    if stage == ExecutionStage.ON_TASK:
        return ExecutionOutcome.TASK_FINISHED
    return ExecutionOutcome.FAILED
