"""Agent-step orchestration helpers for the replay simulation."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    AgentTaskState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
    clamp,
    distance_to_point,
    is_operational_agent,
)
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_on_task,
    transition_to_recovery,
    transition_to_rendezvous,
    transition_to_task,
)
from usv_uav_marine_coverage.execution.collision_guard import (
    _apply_collision_guard_with_optional_wrecks,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionOutcome,
    ExecutionStage,
    UavCoverageState,
    WreckZone,
)
from usv_uav_marine_coverage.execution.path_follower import (
    follow_path_step,
    follow_path_step_with_local_mpc,
)
from usv_uav_marine_coverage.execution.progress_feedback import (
    evaluate_usv_progress,
    record_task_claim_feedback,
    reset_progress_state,
)
from usv_uav_marine_coverage.execution.recharge_runtime import (
    _dock_to_support_agent,
    _has_reached_rendezvous,
    _sync_recharging_uavs_to_support_agents,
)
from usv_uav_marine_coverage.execution.stage_runtime import (
    StageRuntimeContext,
)
from usv_uav_marine_coverage.execution.stage_runtime import (
    run_agent_stage as _run_execution_stage,
)
from usv_uav_marine_coverage.execution.task_claim_runtime import (
    claim_task_for_execution,
)
from usv_uav_marine_coverage.execution.traffic_runtime import (
    apply_bottleneck_directive as _apply_bottleneck_directive,
)
from usv_uav_marine_coverage.execution.traffic_runtime import (
    apply_corridor_directive as _apply_corridor_directive,
)
from usv_uav_marine_coverage.execution.traffic_runtime import (
    build_corridor_directives as _build_corridor_directives,
)
from usv_uav_marine_coverage.execution.traffic_runtime import (
    build_dynamic_bottleneck_directives as _build_dynamic_bottleneck_directives,
)
from usv_uav_marine_coverage.execution.traffic_runtime import (
    progress_target as _progress_target,
)
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskStatus, TaskType

USV_STALL_DISTANCE_EPS_M = 1e-3
WRECK_KEEP_OUT_RADIUS_M = USV_COLLISION_CLEARANCE_M * 1.5

__all__ = [
    "_evaluate_agent_progress",
    "advance_agents_one_step",
    "build_initial_execution_states",
    "build_wreck_zones",
]


def build_wreck_zones(agents: tuple[AgentState, ...]) -> tuple[WreckZone, ...]:
    """Build persistent wreck keepout zones from failed USVs."""

    return tuple(
        WreckZone(
            source_agent_id=agent.agent_id,
            x=agent.x,
            y=agent.y,
            radius=WRECK_KEEP_OUT_RADIUS_M,
        )
        for agent in agents
        if agent.kind == "USV" and not is_operational_agent(agent)
    )


def build_initial_execution_states(
    agents: tuple[AgentState, ...],
) -> dict[str, AgentExecutionState]:
    """Build the default patrol execution state for each agent."""

    return {
        agent.agent_id: AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )
        for agent in agents
    }


def advance_agents_one_step(
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    task_records: tuple[TaskRecord, ...],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    grid_map: GridMap,
    info_map: InformationMap | None = None,
    obstacle_layout: ObstacleLayout | None = None,
    dt_seconds: float,
    step: int = 0,
    usv_path_planner: str = "astar_path_planner",
    uav_search_planner: str = "uav_lawnmower_planner",
    execution_policy: str = "phase_one_execution",
    enable_uav_usv_meeting: bool = True,
    uav_coverage_states: dict[str, UavCoverageState] | None = None,
) -> tuple[
    tuple[AgentState, ...],
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    """Advance all agents by one simulation step."""

    next_agents: list[AgentState] = []
    next_execution_states: dict[str, AgentExecutionState] = {}
    next_progress_states: dict[str, AgentProgressState] = {}
    agent_by_id = {agent.agent_id: agent for agent in agents}

    staged_execution_states: dict[str, AgentExecutionState] = {}
    staged_progress_states: dict[str, AgentProgressState] = {}
    active_tasks_by_agent: dict[str, TaskRecord | None] = {}
    wreck_zones = build_wreck_zones(agents)

    for agent in agents:
        execution_state = execution_states[agent.agent_id]
        progress_state = progress_states[agent.agent_id]
        if not is_operational_agent(agent):
            staged_execution_states[agent.agent_id] = replace(
                execution_state,
                stage=ExecutionStage.FAILED,
                active_task_id=None,
                active_plan=None,
                current_waypoint_index=0,
            )
            staged_progress_states[agent.agent_id] = reset_progress_state(progress_state)
            active_tasks_by_agent[agent.agent_id] = None
            continue
        claim_decision = claim_task_for_execution(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            task_records=task_records,
            grid_map=grid_map,
            info_map=info_map,
            patrol_routes=patrol_routes,
            step=step,
        )
        transitioned_execution_state = claim_decision.execution_state
        next_progress_state = (
            reset_progress_state(progress_state)
            if transitioned_execution_state != execution_state
            else progress_state
        )
        staged_execution_states[agent.agent_id] = transitioned_execution_state
        staged_progress_states[agent.agent_id] = record_task_claim_feedback(
            next_progress_state,
            pending_assigned_task_id=claim_decision.pending_assigned_task_id,
            claimed_task_id=claim_decision.claimed_task_id,
            claim_transition_reason=claim_decision.claim_transition_reason,
        )
        active_tasks_by_agent[agent.agent_id] = claim_decision.active_task

    corridor_directives = _build_corridor_directives(
        agents=agents,
        execution_states=staged_execution_states,
        progress_states=staged_progress_states,
        obstacle_layout=obstacle_layout,
    )
    bottleneck_directives = _build_dynamic_bottleneck_directives(
        agents=agents,
        execution_states=staged_execution_states,
        active_tasks_by_agent=active_tasks_by_agent,
        patrol_routes=patrol_routes,
        obstacle_layout=obstacle_layout,
        corridor_directives=corridor_directives,
        step=step,
    )

    for agent in agents:
        execution_state = staged_execution_states[agent.agent_id]
        progress_state = staged_progress_states[agent.agent_id]
        if not is_operational_agent(agent):
            next_agents.append(
                replace(
                    _stop_agent_motion(agent),
                    task=AgentTaskState(mode=TaskMode.IDLE),
                )
            )
            next_execution_states[agent.agent_id] = execution_state
            next_progress_states[agent.agent_id] = progress_state
            continue
        active_task = active_tasks_by_agent[agent.agent_id]
        corridor_owner_agent = None
        corridor_owner_execution_state = None
        corridor_owner_progress_state = None
        corridor_owner_agent_id = execution_state.corridor_owner_agent_id
        if corridor_owner_agent_id is not None:
            corridor_owner_agent = agent_by_id.get(corridor_owner_agent_id)
            corridor_owner_execution_state = staged_execution_states.get(corridor_owner_agent_id)
            corridor_owner_progress_state = staged_progress_states.get(corridor_owner_agent_id)
        execution_state = _apply_corridor_directive(
            execution_state,
            directive=corridor_directives.get(agent.agent_id),
            step=step,
            owner_agent=corridor_owner_agent,
            owner_execution_state=corridor_owner_execution_state,
            owner_progress_state=corridor_owner_progress_state,
        )
        execution_state = _apply_bottleneck_directive(
            execution_state,
            directive=bottleneck_directives.get(agent.agent_id),
            step=step,
        )
        updated_agent, updated_execution_state, updated_progress_state = _run_agent_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            execution_states=staged_execution_states,
            agent_by_id=agent_by_id,
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            wreck_zones=wreck_zones,
            dt_seconds=dt_seconds,
            step=step,
            usv_path_planner=usv_path_planner,
            uav_search_planner=uav_search_planner,
            execution_policy=execution_policy,
            enable_uav_usv_meeting=enable_uav_usv_meeting,
            uav_coverage_states=uav_coverage_states,
            task_records=task_records,
        )
        updated_execution_state, updated_progress_state = _evaluate_agent_progress(
            agent,
            updated_agent=updated_agent,
            execution_state=updated_execution_state,
            progress_state=updated_progress_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
            step=step,
            enable_uav_usv_meeting=enable_uav_usv_meeting,
        )
        next_agents.append(updated_agent)
        next_execution_states[agent.agent_id] = updated_execution_state
        next_progress_states[agent.agent_id] = updated_progress_state

    synced_agents = _sync_recharging_uavs_to_support_agents(
        tuple(next_agents),
        execution_states=next_execution_states,
        task_records=task_records,
    )
    (
        stabilized_agents,
        stabilized_execution_states,
        stabilized_progress_states,
    ) = _stabilize_support_usvs_after_rendezvous_sync(
        synced_agents,
        execution_states=next_execution_states,
        progress_states=next_progress_states,
        task_records=task_records,
    )
    return (stabilized_agents, stabilized_execution_states, stabilized_progress_states)


def _pre_step_transition(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    task_records: tuple[TaskRecord, ...],
    grid_map: GridMap,
    info_map: InformationMap | None,
    step: int,
) -> AgentExecutionState:
    claim_decision = claim_task_for_execution(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        task_records=task_records if active_task is None else task_records,
        patrol_routes=patrol_routes,
        grid_map=grid_map,
        info_map=info_map,
        step=step,
    )
    if (
        active_task is not None
        and claim_decision.active_task is None
        and claim_decision.claim_transition_reason != "final_approach_backoff"
    ):
        if active_task.task_type == TaskType.UAV_RESUPPLY:
            return transition_to_rendezvous(execution_state, task_id=active_task.task_id)
        return transition_to_task(execution_state, task_id=active_task.task_id)
    return claim_decision.execution_state


def _run_agent_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    execution_states: dict[str, AgentExecutionState],
    agent_by_id: dict[str, AgentState],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    grid_map: GridMap,
    info_map: InformationMap | None,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
    dt_seconds: float,
    step: int,
    usv_path_planner: str,
    uav_search_planner: str,
    execution_policy: str,
    enable_uav_usv_meeting: bool,
    uav_coverage_states: dict[str, UavCoverageState] | None,
    task_records: tuple[TaskRecord, ...],
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if execution_state.stage == ExecutionStage.YIELD:
        return _run_usv_yield_step(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            obstacle_layout=obstacle_layout,
            wreck_zones=wreck_zones,
            dt_seconds=dt_seconds,
        )

    context = StageRuntimeContext(
        agent_by_id=agent_by_id,
        patrol_routes=patrol_routes,
        grid_map=grid_map,
        info_map=info_map,
        obstacle_layout=obstacle_layout,
        wreck_zones=wreck_zones,
        dt_seconds=dt_seconds,
        step=step,
        usv_path_planner=usv_path_planner,
        uav_search_planner=uav_search_planner,
        execution_policy=execution_policy,
        enable_uav_usv_meeting=enable_uav_usv_meeting,
        execution_states=execution_states,
        uav_coverage_states=uav_coverage_states,
        task_records=task_records,
        advance_path_execution_step=_advance_path_execution_step,
        neighboring_agents=_neighboring_agents,
        stop_agent_motion=_stop_agent_motion,
        agent_moved=_agent_moved,
    )
    return _run_execution_stage(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        active_task=active_task,
        context=context,
    )


def _advance_path_execution_step(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    grid_map: GridMap,
    dt_seconds: float,
    obstacle_layout: ObstacleLayout | None,
    execution_policy: str,
    neighboring_agents: tuple[AgentState, ...],
    wreck_zones: tuple[WreckZone, ...] = (),
) -> tuple[AgentState, AgentExecutionState, ExecutionOutcome]:
    if execution_policy == "local_mpc_execution":
        if wreck_zones:
            advanced_agent, updated_state, outcome = follow_path_step_with_local_mpc(
                agent,
                execution_state,
                dt_seconds=dt_seconds,
                obstacle_layout=obstacle_layout,
                neighboring_agents=neighboring_agents,
                wreck_zones=wreck_zones,
                grid_width=grid_map.width,
                grid_height=grid_map.height,
            )
        else:
            advanced_agent, updated_state, outcome = follow_path_step_with_local_mpc(
                agent,
                execution_state,
                dt_seconds=dt_seconds,
                obstacle_layout=obstacle_layout,
                neighboring_agents=neighboring_agents,
                grid_width=grid_map.width,
                grid_height=grid_map.height,
            )
    else:
        if wreck_zones:
            advanced_agent, updated_state, outcome = follow_path_step(
                agent,
                execution_state,
                dt_seconds=dt_seconds,
                obstacle_layout=obstacle_layout,
                wreck_zones=wreck_zones,
            )
        else:
            advanced_agent, updated_state, outcome = follow_path_step(
                agent,
                execution_state,
                dt_seconds=dt_seconds,
                obstacle_layout=obstacle_layout,
            )
    return _clamp_agent_to_grid_bounds(
        previous_agent=agent,
        advanced_agent=advanced_agent,
        execution_state=updated_state,
        grid_map=grid_map,
        outcome=outcome,
    )


def _clamp_agent_to_grid_bounds(
    *,
    previous_agent: AgentState,
    advanced_agent: AgentState,
    execution_state: AgentExecutionState,
    grid_map: GridMap,
    outcome: ExecutionOutcome,
) -> tuple[AgentState, AgentExecutionState, ExecutionOutcome]:
    clamped_x = clamp(advanced_agent.x, 0.0, grid_map.width)
    clamped_y = clamp(advanced_agent.y, 0.0, grid_map.height)
    if clamped_x == advanced_agent.x and clamped_y == advanced_agent.y:
        return (advanced_agent, execution_state, outcome)
    bounded_agent = replace(
        advanced_agent,
        x=clamped_x,
        y=clamped_y,
        speed_mps=0.0 if previous_agent.kind == "USV" else advanced_agent.speed_mps,
    )
    bounded_state = execution_state
    if previous_agent.kind == "USV":
        bounded_state = replace(
            execution_state,
            active_plan=None,
            current_waypoint_index=0,
        )
    return (bounded_agent, bounded_state, outcome)


def _neighboring_agents(
    agent: AgentState,
    agent_by_id: dict[str, AgentState],
) -> tuple[AgentState, ...]:
    return tuple(
        other_agent
        for other_agent_id, other_agent in agent_by_id.items()
        if other_agent_id != agent.agent_id and is_operational_agent(other_agent)
    )


def _agent_moved(previous_agent: AgentState, advanced_agent: AgentState) -> bool:
    return (
        hypot(
            advanced_agent.x - previous_agent.x,
            advanced_agent.y - previous_agent.y,
        )
        > USV_STALL_DISTANCE_EPS_M
    )


def _stop_agent_motion(agent: AgentState) -> AgentState:
    return replace(agent, speed_mps=0.0, turn_rate_degps=0.0)


def _stabilize_support_usvs_after_rendezvous_sync(
    agents: tuple[AgentState, ...],
    *,
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    task_records: tuple[TaskRecord, ...],
) -> tuple[
    tuple[AgentState, ...],
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    agent_by_id = {agent.agent_id: agent for agent in agents}
    next_agents = dict(agent_by_id)
    next_execution_states = dict(execution_states)
    next_progress_states = dict(progress_states)

    for task in task_records:
        if task.task_type != TaskType.UAV_RESUPPLY or task.support_agent_id is None:
            continue
        if task.status not in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
            continue
        uav_execution_state = (
            None if task.assigned_agent_id is None else next_execution_states.get(task.assigned_agent_id)
        )
        if uav_execution_state is None or uav_execution_state.stage != ExecutionStage.ON_RECHARGE:
            continue
        uav_agent = None if task.assigned_agent_id is None else next_agents.get(task.assigned_agent_id)
        support_agent = next_agents.get(task.support_agent_id)
        support_execution_state = next_execution_states.get(task.support_agent_id)
        if uav_agent is None or support_agent is None or support_execution_state is None:
            continue
        if not _has_reached_rendezvous(uav_agent, support_agent):
            continue
        if support_execution_state.stage == ExecutionStage.ON_TASK:
            stationary_support_agent = _stop_agent_motion(assign_agent_task(support_agent, TaskMode.IDLE))
            next_agents[task.support_agent_id] = stationary_support_agent
            if task.assigned_agent_id is not None:
                next_agents[task.assigned_agent_id] = _dock_to_support_agent(
                    uav_agent,
                    stationary_support_agent,
                )
            continue
        if support_execution_state.stage not in {
            ExecutionStage.GO_TO_TASK,
            ExecutionStage.RETURN_TO_PATROL,
        }:
            continue
        stationary_support_agent = _stop_agent_motion(
            assign_agent_task(support_agent, TaskMode.IDLE)
        )
        next_agents[task.support_agent_id] = stationary_support_agent
        if task.assigned_agent_id is not None:
            next_agents[task.assigned_agent_id] = _dock_to_support_agent(
                uav_agent,
                stationary_support_agent,
            )
        next_execution_states[task.support_agent_id] = transition_to_on_task(support_execution_state)
        next_progress_states[task.support_agent_id] = reset_progress_state(
            next_progress_states[task.support_agent_id]
        )

    return (
        tuple(next_agents[agent.agent_id] for agent in agents),
        next_execution_states,
        next_progress_states,
    )


def _evaluate_agent_progress(
    previous_agent: AgentState,
    *,
    updated_agent: AgentState,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    step: int,
    enable_uav_usv_meeting: bool = True,
) -> tuple[AgentExecutionState, AgentProgressState]:
    if updated_agent.kind != "USV":
        return (execution_state, progress_state)
    if execution_state.stage == ExecutionStage.RECOVERY:
        return (execution_state, progress_state)
    if execution_state.stage == ExecutionStage.YIELD:
        return (execution_state, reset_progress_state(progress_state))
    if (
        execution_state.stage == ExecutionStage.PATROL
        and execution_state.active_plan is None
        and active_task is None
    ):
        # Let idle patrol recover its local plan before classifying the agent as stalled.
        return (execution_state, reset_progress_state(progress_state))
    if execution_state.stage not in {
        ExecutionStage.PATROL,
        ExecutionStage.GO_TO_TASK,
        ExecutionStage.RETURN_TO_PATROL,
    }:
        return (execution_state, reset_progress_state(progress_state))

    target_x, target_y = _progress_target(
        updated_agent,
        execution_state=execution_state,
        active_task=active_task,
        patrol_route=patrol_routes[updated_agent.agent_id],
    )
    if target_x is None or target_y is None:
        return (execution_state, progress_state)

    evaluation = evaluate_usv_progress(
        previous_agent,
        updated_agent=updated_agent,
        execution_state=execution_state,
        progress_state=progress_state,
        active_task=active_task,
        target_x=target_x,
        target_y=target_y,
        path_cleared=execution_state.active_plan is None,
        step=step,
        enable_uav_usv_meeting=enable_uav_usv_meeting,
    )
    if not evaluation.should_enter_recovery:
        return (execution_state, evaluation.progress_state)
    recovery_execution_state = transition_to_recovery(execution_state)
    recovery_progress_state = replace(
        evaluation.progress_state,
        pre_recovery_stage=execution_state.stage,
        pre_recovery_task_id=execution_state.active_task_id,
        blocked_goal_signature=evaluation.blocked_goal_signature,
        recovery_step_index=0,
    )
    return (recovery_execution_state, recovery_progress_state)


def _run_usv_yield_step(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
    dt_seconds: float,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if (
        execution_state.yield_target_x is None
        or execution_state.yield_target_y is None
        or agent.kind != "USV"
    ):
        return (
            _stop_agent_motion(assign_agent_task(agent, TaskMode.IDLE)),
            execution_state,
            reset_progress_state(progress_state),
        )

    hold_agent = assign_agent_task(
        agent,
        TaskMode.PATROL,
        execution_state.yield_target_x,
        execution_state.yield_target_y,
    )
    advanced_agent = advance_agent_towards_task(hold_agent, dt_seconds)
    advanced_agent, execution_state = _apply_collision_guard_with_optional_wrecks(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=obstacle_layout,
        wreck_zones=wreck_zones,
    )
    if (
        distance_to_point(
            advanced_agent,
            execution_state.yield_target_x,
            execution_state.yield_target_y,
        )
        <= advanced_agent.arrival_tolerance_m
    ):
        advanced_agent = _stop_agent_motion(assign_agent_task(advanced_agent, TaskMode.IDLE))
    return (advanced_agent, execution_state, reset_progress_state(progress_state))
