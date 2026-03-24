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
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )


def transition_to_on_task(state: AgentExecutionState) -> AgentExecutionState:
    """Transition one agent from transit to on-task handling."""

    return replace(
        state,
        stage=ExecutionStage.ON_TASK,
        active_plan=None,
        current_waypoint_index=0,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )


def transition_to_rendezvous(
    state: AgentExecutionState,
    *,
    task_id: str,
) -> AgentExecutionState:
    """Transition one UAV from patrol into rendezvous transit."""

    return replace(
        state,
        stage=ExecutionStage.GO_TO_RENDEZVOUS,
        active_task_id=task_id,
        active_plan=None,
        current_waypoint_index=0,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )


def transition_to_recharge(state: AgentExecutionState) -> AgentExecutionState:
    """Transition one UAV from rendezvous transit into recharge handling."""

    return replace(
        state,
        stage=ExecutionStage.ON_RECHARGE,
        active_plan=None,
        current_waypoint_index=0,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )


def transition_to_return_to_patrol(
    state: AgentExecutionState,
    *,
    return_target_x: float,
    return_target_y: float,
    patrol_waypoint_index: int,
    return_target_source: str | None = None,
    rejoin_to_segment: bool = False,
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
        return_target_source=return_target_source,
        rejoin_to_segment=rejoin_to_segment,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
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
        return_target_source=None,
        rejoin_to_segment=False,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )


def transition_to_recovery(state: AgentExecutionState) -> AgentExecutionState:
    """Transition one USV into local recovery handling."""

    return replace(
        state,
        stage=ExecutionStage.RECOVERY,
        active_plan=None,
        current_waypoint_index=0,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )


def transition_to_yield(
    state: AgentExecutionState,
    *,
    yield_target_x: float,
    yield_target_y: float,
    yield_reason: str,
    reserved_corridor_name: str | None = None,
    corridor_owner_agent_id: str | None = None,
    reserved_bottleneck_zone_id: str | None = None,
    bottleneck_owner_agent_id: str | None = None,
) -> AgentExecutionState:
    """Transition one USV into corridor-yield holding behavior."""

    return replace(
        state,
        stage=ExecutionStage.YIELD,
        active_plan=None,
        current_waypoint_index=0,
        yield_target_x=yield_target_x,
        yield_target_y=yield_target_y,
        yield_reason=yield_reason,
        reserved_corridor_name=reserved_corridor_name,
        corridor_owner_agent_id=corridor_owner_agent_id,
        corridor_reservation_until_step=state.corridor_reservation_until_step,
        reserved_bottleneck_zone_id=reserved_bottleneck_zone_id,
        bottleneck_owner_agent_id=bottleneck_owner_agent_id,
        bottleneck_reservation_until_step=state.bottleneck_reservation_until_step,
        pre_yield_stage=state.pre_yield_stage or state.stage,
    )


def transition_from_yield(state: AgentExecutionState) -> AgentExecutionState:
    """Resume the pre-yield stage once the corridor is free."""

    resumed_stage = state.pre_yield_stage or ExecutionStage.PATROL
    return replace(
        state,
        stage=resumed_stage,
        active_plan=None,
        current_waypoint_index=0,
        yield_target_x=None,
        yield_target_y=None,
        yield_reason=None,
        reserved_corridor_name=None,
        corridor_owner_agent_id=None,
        corridor_reservation_until_step=-1,
        reserved_bottleneck_zone_id=None,
        bottleneck_owner_agent_id=None,
        bottleneck_reservation_until_step=-1,
        pre_yield_stage=None,
    )
