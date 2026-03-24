"""Execution-time UAV rendezvous and recharge helpers."""

from __future__ import annotations

from dataclasses import replace

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    TaskMode,
    assign_agent_task,
    can_support_uav_resupply,
    distance_to_point,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord

from .execution_types import AgentExecutionState, ExecutionStage

# Maximum distance at which a recharging UAV may be snapped to its support USV
# each step.  During normal charging the USV drifts at most a few metres per
# step, so the UAV stays well inside this bound.  The threshold must be large
# enough to cover the initial gap when a test places the UAV in ON_RECHARGE
# state without first running the approach phase (~90 m in demo scenarios), but
# small enough to catch mid-charge support-agent switches that would otherwise
# teleport the UAV by hundreds of metres.
_DOCK_MAX_SNAP_DISTANCE_M = 100.0


def _has_reached_rendezvous(uav: AgentState, support_agent: AgentState) -> bool:
    rendezvous_tolerance = max(
        uav.arrival_tolerance_m,
        support_agent.arrival_tolerance_m,
        12.0,
    )
    return distance_to_point(uav, support_agent.x, support_agent.y) <= rendezvous_tolerance


def _dock_to_support_agent(uav: AgentState, support_agent: AgentState) -> AgentState:
    attached_agent = replace(
        uav,
        x=support_agent.x,
        y=support_agent.y,
        heading_deg=support_agent.heading_deg,
        speed_mps=support_agent.speed_mps,
        turn_rate_degps=support_agent.turn_rate_degps,
    )
    return assign_agent_task(attached_agent, TaskMode.IDLE)


def _sync_recharging_uavs_to_support_agents(
    agents: tuple[AgentState, ...],
    *,
    execution_states: dict[str, AgentExecutionState],
    task_records: tuple[TaskRecord, ...],
) -> tuple[AgentState, ...]:
    task_by_id = {task.task_id: task for task in task_records}
    agent_by_id = {agent.agent_id: agent for agent in agents}
    synced_agents: list[AgentState] = []

    for agent in agents:
        execution_state = execution_states.get(agent.agent_id)
        if execution_state is None or execution_state.stage != ExecutionStage.ON_RECHARGE:
            synced_agents.append(agent)
            continue
        task_id = execution_state.active_task_id
        task = None if task_id is None else task_by_id.get(task_id)
        if task is None or task.support_agent_id is None:
            synced_agents.append(agent)
            continue
        support_agent = agent_by_id.get(task.support_agent_id)
        if support_agent is None or not can_support_uav_resupply(support_agent):
            synced_agents.append(agent)
            continue
        # Guard against large-distance snaps that arise when support_agent_id
        # changes mid-charge (e.g. the original USV fails and a different one
        # is assigned).  A snap of more than _DOCK_MAX_SNAP_DISTANCE_M would
        # appear as an instantaneous teleport in the replay visualisation.
        if distance_to_point(agent, support_agent.x, support_agent.y) > _DOCK_MAX_SNAP_DISTANCE_M:
            synced_agents.append(agent)
            continue
        synced_agents.append(_dock_to_support_agent(agent, support_agent))

    return tuple(synced_agents)
