"""Basic execution-state transitions for the first closed loop."""

from __future__ import annotations

from dataclasses import replace

from .execution_types import AgentExecutionState, ExecutionStage


def transition_to_task(
    state: AgentExecutionState,
    *,
    task_id: str,
) -> AgentExecutionState:
    """Transition one agent from patrol into one task-approach stage."""

    return replace(
        state,
        stage=ExecutionStage.GO_TO_TASK,
        active_task_id=task_id,
        active_plan=None,
        current_waypoint_index=0,
    )


def transition_to_on_task(state: AgentExecutionState) -> AgentExecutionState:
    """Transition one agent from transit to on-task handling."""

    return replace(
        state,
        stage=ExecutionStage.ON_TASK,
        active_plan=None,
        current_waypoint_index=0,
    )


def transition_to_return_to_patrol(
    state: AgentExecutionState,
    *,
    return_target_x: float,
    return_target_y: float,
    patrol_waypoint_index: int,
) -> AgentExecutionState:
    """Transition one agent from a completed task back to patrol recovery."""

    return replace(
        state,
        stage=ExecutionStage.RETURN_TO_PATROL,
        active_task_id=None,
        active_plan=None,
        current_waypoint_index=0,
        patrol_waypoint_index=patrol_waypoint_index,
        return_target_x=return_target_x,
        return_target_y=return_target_y,
    )


def transition_to_patrol(state: AgentExecutionState) -> AgentExecutionState:
    """Transition one agent from return-to-patrol back into patrol."""

    return replace(
        state,
        stage=ExecutionStage.PATROL,
        active_task_id=None,
        active_plan=None,
        current_waypoint_index=0,
        return_target_x=None,
        return_target_y=None,
    )
