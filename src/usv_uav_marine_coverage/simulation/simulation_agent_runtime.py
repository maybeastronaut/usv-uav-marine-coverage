"""Agent-step orchestration helpers for the replay simulation."""

from __future__ import annotations

from dataclasses import replace
from math import cos, hypot, radians, sin

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
    distance_to_point,
    recharge_agent_energy,
    shortest_heading_delta_deg,
)
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_on_task,
    transition_to_patrol,
    transition_to_recharge,
    transition_to_recovery,
    transition_to_rendezvous,
    transition_to_return_to_patrol,
    transition_to_task,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionOutcome,
    ExecutionStage,
    UavCoverageState,
)
from usv_uav_marine_coverage.execution.path_follower import follow_path_step
from usv_uav_marine_coverage.execution.progress_feedback import (
    build_goal_signature,
    evaluate_usv_progress,
    reset_progress_state,
    should_replan_patrol,
    should_replan_return,
    should_replan_task,
)
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.planning.astar_path_planner import build_astar_path_plan
from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus, Waypoint
from usv_uav_marine_coverage.planning.uav_lawnmower_planner import (
    build_uav_lawnmower_plan,
)
from usv_uav_marine_coverage.planning.uav_multi_region_coverage_planner import (
    UAV_MULTI_REGION_LANE_SPACING_M,
    UAV_MULTI_REGION_MAX_Y,
    UAV_MULTI_REGION_MIN_Y,
    UAV_MULTI_REGION_X_MARGIN_LEFT,
    UAV_MULTI_REGION_X_MARGIN_RIGHT,
    build_uav_multi_region_plan,
    build_uav_multi_region_route,
    select_uav_multi_region_waypoint_index,
)
from usv_uav_marine_coverage.planning.uav_persistent_multi_region_coverage_planner import (
    advance_uav_region_waypoint,
    build_empty_uav_coverage_state,
    build_uav_persistent_multi_region_plan,
)
from usv_uav_marine_coverage.planning.usv_patrol_planner import (
    PatrolSegmentAccess,
    find_local_patrol_segment_access,
    find_progressive_patrol_segment_access,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType

from .uav_coverage_runtime import resolve_persistent_uav_coverage_state, uav_offshore_x_bounds

USV_COLLISION_SAMPLE_SPACING_M = 2.0
USV_STALL_DISTANCE_EPS_M = 1e-3
USV_MAX_RECOVERY_ATTEMPTS = 2
USV_RECOVERY_REVERSE_DISTANCE_M = 18.0
USV_RECOVERY_FORWARD_DISTANCE_M = 26.0
USV_RECOVERY_CLEARANCE_IMPROVEMENT_M = 0.5


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
    uav_search_planner: str = "uav_lawnmower_planner",
    uav_coverage_states: dict[str, UavCoverageState] | None = None,
) -> tuple[
    tuple[AgentState, ...],
    dict[str, AgentExecutionState],
    dict[str, AgentProgressState],
]:
    """Advance all agents by one simulation step."""

    from .simulation_task_runtime import select_assigned_task_for_agent

    next_agents: list[AgentState] = []
    next_execution_states: dict[str, AgentExecutionState] = {}
    next_progress_states: dict[str, AgentProgressState] = {}
    agent_by_id = {agent.agent_id: agent for agent in agents}

    for agent in agents:
        execution_state = execution_states[agent.agent_id]
        progress_state = progress_states[agent.agent_id]
        active_task = select_assigned_task_for_agent(task_records, agent.agent_id)
        transitioned_execution_state = _pre_step_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
        )
        if transitioned_execution_state != execution_state:
            progress_state = reset_progress_state(progress_state)
        execution_state = transitioned_execution_state
        updated_agent, updated_execution_state, updated_progress_state = _run_agent_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            agent_by_id=agent_by_id,
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=obstacle_layout,
            dt_seconds=dt_seconds,
            step=step,
            uav_search_planner=uav_search_planner,
            uav_coverage_states=uav_coverage_states,
        )
        updated_execution_state, updated_progress_state = _evaluate_agent_progress(
            agent,
            updated_agent=updated_agent,
            execution_state=updated_execution_state,
            progress_state=updated_progress_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
            step=step,
        )
        next_agents.append(updated_agent)
        next_execution_states[agent.agent_id] = updated_execution_state
        next_progress_states[agent.agent_id] = updated_progress_state

    synced_agents = _sync_recharging_uavs_to_support_agents(
        tuple(next_agents),
        execution_states=next_execution_states,
        task_records=task_records,
    )
    return (synced_agents, next_execution_states, next_progress_states)


def find_nearest_patrol_rejoin_point(
    agent: AgentState,
    patrol_route: tuple[tuple[float, float], ...],
) -> tuple[int, float, float]:
    """Return the nearest local patrol-segment access for the current agent position."""

    access = find_local_patrol_segment_access(
        agent_x=agent.x,
        agent_y=agent.y,
        patrol_route=patrol_route,
    )
    if access is None:
        indexed_points = [(index, point[0], point[1]) for index, point in enumerate(patrol_route)]
        index, x, y = min(
            indexed_points,
            key=lambda item: (agent.x - item[1]) ** 2 + (agent.y - item[2]) ** 2,
        )
        return (index, x, y)
    return (access.segment_end_index, access.access_x, access.access_y)


def _pre_step_transition(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
) -> AgentExecutionState:
    if execution_state.stage == ExecutionStage.PATROL and active_task is not None:
        if active_task.task_type == TaskType.UAV_RESUPPLY:
            return transition_to_rendezvous(execution_state, task_id=active_task.task_id)
        return transition_to_task(execution_state, task_id=active_task.task_id)

    if (
        execution_state.stage
        in {
            ExecutionStage.GO_TO_TASK,
            ExecutionStage.ON_TASK,
            ExecutionStage.GO_TO_RENDEZVOUS,
            ExecutionStage.ON_RECHARGE,
        }
        and active_task is None
    ):
        return _build_local_patrol_return_transition(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_routes[agent.agent_id],
        )
    return execution_state


def _run_agent_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    agent_by_id: dict[str, AgentState],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    grid_map: GridMap,
    info_map: InformationMap | None,
    obstacle_layout: ObstacleLayout | None,
    dt_seconds: float,
    step: int,
    uav_search_planner: str,
    uav_coverage_states: dict[str, UavCoverageState] | None,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if execution_state.stage == ExecutionStage.RECOVERY:
        return _run_usv_recovery_step(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            patrol_route=patrol_routes[agent.agent_id],
            obstacle_layout=obstacle_layout,
            dt_seconds=dt_seconds,
            step=step,
        )

    if execution_state.stage == ExecutionStage.ON_TASK:
        confirm_agent = assign_agent_task(
            agent,
            TaskMode.CONFIRM if agent.kind == "USV" else TaskMode.INVESTIGATE,
        )
        return (
            advance_agent_towards_task(confirm_agent, dt_seconds),
            execution_state,
            progress_state,
        )

    if execution_state.stage == ExecutionStage.ON_RECHARGE:
        idle_agent = assign_agent_task(agent, TaskMode.IDLE)
        return (recharge_agent_energy(idle_agent, dt_seconds), execution_state, progress_state)

    if execution_state.stage == ExecutionStage.GO_TO_TASK and active_task is not None:
        if should_replan_task(
            agent,
            execution_state=execution_state,
            active_task=active_task,
        ):
            if agent.kind == "USV":
                plan = build_astar_path_plan(
                    agent,
                    grid_map=grid_map,
                    goal_x=active_task.target_x,
                    goal_y=active_task.target_y,
                    planner_name="astar_path_planner",
                    task_id=active_task.task_id,
                    stats_context="runtime_go_to_task",
                )
            else:
                plan = build_direct_line_plan(
                    agent,
                    goal_x=active_task.target_x,
                    goal_y=active_task.target_y,
                    planner_name="direct_line_planner",
                    task_id=active_task.task_id,
                )
            execution_state = replace(
                execution_state,
                active_plan=plan,
                current_waypoint_index=0,
            )
        plan = execution_state.active_plan
        if plan is None:
            if agent.kind != "USV":
                return (agent, execution_state, progress_state)
            return _release_blocked_usv_task(
                _stop_agent_motion(assign_agent_task(agent, TaskMode.IDLE)),
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_routes=patrol_routes,
            )
        if plan.status != PathPlanStatus.PLANNED:
            if agent.kind != "USV":
                return (agent, execution_state, progress_state)
            return _release_blocked_usv_task(
                agent,
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_routes=patrol_routes,
            )
        advanced_agent, execution_state, outcome = follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
            obstacle_layout=obstacle_layout,
        )
        advanced_agent, execution_state = _apply_usv_collision_guard(
            agent,
            advanced_agent=advanced_agent,
            execution_state=execution_state,
            obstacle_layout=obstacle_layout,
        )
        if (
            agent.kind == "USV"
            and execution_state.active_plan is None
            and not _agent_moved(agent, advanced_agent)
        ):
            return _release_blocked_usv_task(
                _stop_agent_motion(assign_agent_task(advanced_agent, TaskMode.IDLE)),
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_routes=patrol_routes,
            )
        if outcome == ExecutionOutcome.TASK_SITE_REACHED:
            return (advanced_agent, transition_to_on_task(execution_state), progress_state)
        return (advanced_agent, execution_state, progress_state)

    if execution_state.stage == ExecutionStage.GO_TO_RENDEZVOUS and active_task is not None:
        support_agent = (
            None
            if active_task.support_agent_id is None
            else agent_by_id.get(active_task.support_agent_id)
        )
        if support_agent is None:
            return (agent, execution_state, progress_state)
        if _has_reached_rendezvous(agent, support_agent):
            docked_agent = _dock_to_support_agent(agent, support_agent)
            return (docked_agent, transition_to_recharge(execution_state), progress_state)
        plan = build_direct_line_plan(
            agent,
            goal_x=support_agent.x,
            goal_y=support_agent.y,
            planner_name="uav_rendezvous_planner",
            task_id=active_task.task_id,
        )
        execution_state = replace(execution_state, active_plan=plan, current_waypoint_index=0)
        advanced_agent, execution_state, outcome = follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
            obstacle_layout=obstacle_layout,
        )
        if outcome == ExecutionOutcome.TASK_SITE_REACHED or _has_reached_rendezvous(
            advanced_agent,
            support_agent,
        ):
            docked_agent = _dock_to_support_agent(advanced_agent, support_agent)
            return (docked_agent, transition_to_recharge(execution_state), progress_state)
        return (advanced_agent, execution_state, progress_state)

    if execution_state.stage == ExecutionStage.RETURN_TO_PATROL:
        assert (
            execution_state.return_target_x is not None
            and execution_state.return_target_y is not None
        )
        if should_replan_return(agent, execution_state=execution_state):
            # runtime decides when to call the planner; feedback owns the predicate.
            execution_state = _plan_return_to_patrol_path(
                agent,
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_route=patrol_routes[agent.agent_id],
                grid_map=grid_map,
                allow_patrol_index_advance=False,
                step=step,
            )
        plan = execution_state.active_plan
        if plan is None:
            return (agent, execution_state, progress_state)
        if plan.status != PathPlanStatus.PLANNED:
            execution_state = _plan_return_to_patrol_path(
                agent,
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_route=patrol_routes[agent.agent_id],
                grid_map=grid_map,
                allow_patrol_index_advance=True,
                step=step,
            )
            plan = execution_state.active_plan
            if plan is None or plan.status != PathPlanStatus.PLANNED:
                return (assign_agent_task(agent, TaskMode.PATROL), execution_state, progress_state)
        advanced_agent, execution_state, outcome = follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
            obstacle_layout=obstacle_layout,
        )
        advanced_agent, execution_state = _apply_usv_collision_guard(
            agent,
            advanced_agent=advanced_agent,
            execution_state=execution_state,
            obstacle_layout=obstacle_layout,
        )
        if (
            agent.kind == "USV"
            and execution_state.active_plan is None
            and not _agent_moved(agent, advanced_agent)
        ):
            execution_state = _plan_return_to_patrol_path(
                agent,
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_route=patrol_routes[agent.agent_id],
                grid_map=grid_map,
                allow_patrol_index_advance=True,
                step=step,
            )
        if outcome == ExecutionOutcome.PATROL_REJOINED:
            next_index = execution_state.patrol_waypoint_index
            if not execution_state.rejoin_to_segment:
                next_index = (next_index + 1) % len(patrol_routes[agent.agent_id])
            return (
                advanced_agent,
                replace(
                    transition_to_patrol(execution_state),
                    patrol_waypoint_index=next_index,
                ),
                progress_state,
            )
        return (advanced_agent, execution_state, progress_state)

    if agent.kind == "UAV":
        route_id = execution_state.patrol_route_id or agent.agent_id
        if uav_search_planner == "uav_persistent_multi_region_coverage_planner":
            persistent_state_store = {} if uav_coverage_states is None else uav_coverage_states
            persistent_coverage_state = persistent_state_store.get(agent.agent_id)
            if persistent_coverage_state is None:
                persistent_coverage_state = build_empty_uav_coverage_state(agent.agent_id)
            persistent_coverage_state = resolve_persistent_uav_coverage_state(
                agent=agent,
                coverage_state=persistent_coverage_state,
                grid_map=grid_map,
                info_map=info_map,
                step=step,
                all_coverage_states=persistent_state_store,
            )
            persistent_state_store[agent.agent_id] = persistent_coverage_state
            patrol_routes[route_id] = persistent_coverage_state.region_route
            execution_state = replace(
                execution_state,
                patrol_waypoint_index=persistent_coverage_state.region_waypoint_index,
            )
            plan = build_uav_persistent_multi_region_plan(
                agent,
                coverage_state=persistent_coverage_state,
            )
        elif uav_search_planner == "uav_multi_region_coverage_planner":
            offshore_min_x, offshore_max_x = uav_offshore_x_bounds(grid_map)
            patrol_route = build_uav_multi_region_route(
                agent,
                info_map=info_map,
                min_x=offshore_min_x + UAV_MULTI_REGION_X_MARGIN_LEFT,
                max_x=offshore_max_x - UAV_MULTI_REGION_X_MARGIN_RIGHT,
                min_y=UAV_MULTI_REGION_MIN_Y,
                max_y=UAV_MULTI_REGION_MAX_Y,
                lane_spacing=UAV_MULTI_REGION_LANE_SPACING_M,
            )
            patrol_routes[route_id] = patrol_route
            patrol_waypoint_index = select_uav_multi_region_waypoint_index(
                agent,
                patrol_route=patrol_route,
                patrol_waypoint_index=execution_state.patrol_waypoint_index,
            )
            execution_state = replace(
                execution_state,
                patrol_waypoint_index=patrol_waypoint_index,
            )
            plan = build_uav_multi_region_plan(
                agent,
                patrol_route_id=route_id,
                patrol_waypoint_index=patrol_waypoint_index,
                patrol_routes=patrol_routes,
            )
        else:
            plan = build_uav_lawnmower_plan(
                agent,
                patrol_route_id=route_id,
                patrol_waypoint_index=execution_state.patrol_waypoint_index,
                patrol_routes=patrol_routes,
            )
    else:
        patrol_route = patrol_routes[execution_state.patrol_route_id or agent.agent_id]
        patrol_goal_x, patrol_goal_y = patrol_route[execution_state.patrol_waypoint_index]
        if should_replan_patrol(
            agent,
            execution_state=execution_state,
            goal_x=patrol_goal_x,
            goal_y=patrol_goal_y,
        ):
            execution_state = _plan_patrol_path(
                agent,
                execution_state=execution_state,
                patrol_route=patrol_route,
                grid_map=grid_map,
                allow_patrol_index_advance=False,
            )
        plan = execution_state.active_plan
        if plan is None or plan.status != PathPlanStatus.PLANNED:
            stalled_agent = _stop_agent_motion(assign_agent_task(agent, TaskMode.PATROL))
            execution_state = _plan_patrol_path(
                stalled_agent,
                execution_state=replace(
                    execution_state,
                    active_plan=None,
                    current_waypoint_index=0,
                ),
                patrol_route=patrol_route,
                grid_map=grid_map,
                allow_patrol_index_advance=True,
            )
            plan = execution_state.active_plan
            if plan is None or plan.status != PathPlanStatus.PLANNED:
                return (stalled_agent, execution_state, progress_state)
    if agent.kind == "UAV":
        execution_state = replace(execution_state, active_plan=plan, current_waypoint_index=0)
    advanced_agent, execution_state, outcome = follow_path_step(
        agent,
        execution_state,
        dt_seconds=dt_seconds,
        obstacle_layout=obstacle_layout,
    )
    advanced_agent, execution_state = _apply_usv_collision_guard(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=obstacle_layout,
    )
    if (
        agent.kind == "USV"
        and execution_state.stage == ExecutionStage.PATROL
        and execution_state.active_plan is None
        and not _agent_moved(agent, advanced_agent)
    ):
        advanced_agent = _stop_agent_motion(assign_agent_task(advanced_agent, TaskMode.PATROL))
        execution_state = _plan_patrol_path(
            advanced_agent,
            execution_state=replace(
                execution_state,
                active_plan=None,
                current_waypoint_index=0,
            ),
            patrol_route=patrol_routes[agent.agent_id],
            grid_map=grid_map,
            allow_patrol_index_advance=True,
        )
        return (advanced_agent, execution_state, progress_state)
    if outcome == ExecutionOutcome.WAYPOINT_REACHED:
        if (
            agent.kind == "UAV"
            and uav_search_planner == "uav_persistent_multi_region_coverage_planner"
        ):
            persistent_state_store = {} if uav_coverage_states is None else uav_coverage_states
            persistent_coverage_state = persistent_state_store.get(agent.agent_id)
            if persistent_coverage_state is None:
                persistent_coverage_state = build_empty_uav_coverage_state(agent.agent_id)
            persistent_coverage_state = advance_uav_region_waypoint(persistent_coverage_state)
            persistent_state_store[agent.agent_id] = persistent_coverage_state
            if persistent_coverage_state.region_route:
                patrol_routes[agent.agent_id] = persistent_coverage_state.region_route
                next_index = persistent_coverage_state.region_waypoint_index
            else:
                patrol_routes[agent.agent_id] = ()
                next_index = 0
        elif agent.kind == "UAV" and uav_search_planner == "uav_multi_region_coverage_planner":
            next_index = 0
        else:
            next_index = (execution_state.patrol_waypoint_index + 1) % len(
                patrol_routes[agent.agent_id]
            )
        execution_state = replace(
            execution_state,
            active_plan=None,
            current_waypoint_index=0,
            patrol_waypoint_index=next_index,
        )
    return (advanced_agent, execution_state, progress_state)


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
        if support_agent is None:
            synced_agents.append(agent)
            continue
        synced_agents.append(_dock_to_support_agent(agent, support_agent))

    return tuple(synced_agents)


def _build_local_patrol_return_transition(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_route: tuple[tuple[float, float], ...],
    skip_blocked_goal: bool = False,
) -> AgentExecutionState:
    access = _find_local_patrol_access(
        agent,
        patrol_route=patrol_route,
        execution_state=execution_state,
        progress_state=progress_state,
        skip_blocked_goal=skip_blocked_goal,
    )
    if access is None:
        patrol_index, return_x, return_y = find_nearest_patrol_rejoin_point(agent, patrol_route)
        return transition_to_return_to_patrol(
            execution_state,
            return_target_x=return_x,
            return_target_y=return_y,
            patrol_waypoint_index=patrol_index,
        )
    return replace(
        transition_to_return_to_patrol(
            execution_state,
            return_target_x=access.access_x,
            return_target_y=access.access_y,
            patrol_waypoint_index=access.segment_end_index,
            rejoin_to_segment=True,
        ),
    )


def _find_local_patrol_access(
    agent: AgentState,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    skip_blocked_goal: bool,
) -> PatrolSegmentAccess | None:
    blocked_goal_signature = None
    if skip_blocked_goal:
        blocked_goal_signature = progress_state.blocked_goal_signature
    if len(patrol_route) > 4:
        access = find_progressive_patrol_segment_access(
            agent_x=agent.x,
            agent_y=agent.y,
            patrol_route=patrol_route,
            preferred_end_index=execution_state.patrol_waypoint_index,
            blocked_goal_signature=blocked_goal_signature,
        )
        if access is not None:
            return access
    return find_local_patrol_segment_access(
        agent_x=agent.x,
        agent_y=agent.y,
        patrol_route=patrol_route,
        preferred_end_index=execution_state.patrol_waypoint_index,
        blocked_goal_signature=blocked_goal_signature,
    )


def _release_blocked_usv_task(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    next_execution_state = _build_local_patrol_return_transition(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        patrol_route=patrol_routes[agent.agent_id],
        skip_blocked_goal=True,
    )
    return (
        assign_agent_task(agent, TaskMode.IDLE),
        next_execution_state,
        reset_progress_state(progress_state),
    )


def _plan_patrol_path(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    allow_patrol_index_advance: bool,
) -> AgentExecutionState:
    route_length = len(patrol_route)
    if route_length == 0:
        return replace(execution_state, active_plan=None, current_waypoint_index=0)

    start_index = execution_state.patrol_waypoint_index % route_length
    if allow_patrol_index_advance:
        start_index = (start_index + 1) % route_length

    blocked_state: AgentExecutionState | None = None
    for offset in range(route_length):
        patrol_index = (start_index + offset) % route_length
        goal_x, goal_y = patrol_route[patrol_index]
        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name="astar_path_planner",
            task_id=None,
            stats_context="runtime_patrol",
        )
        candidate_state = replace(
            execution_state,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_waypoint_index=patrol_index,
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate_state
        if blocked_state is None:
            blocked_state = candidate_state

    return blocked_state or replace(execution_state, active_plan=None, current_waypoint_index=0)


def _plan_return_to_patrol_path(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    allow_patrol_index_advance: bool,
    step: int,
) -> AgentExecutionState:
    if agent.kind != "USV":
        assert (
            execution_state.return_target_x is not None
            and execution_state.return_target_y is not None
        )
        return replace(
            execution_state,
            active_plan=build_direct_line_plan(
                agent,
                goal_x=execution_state.return_target_x,
                goal_y=execution_state.return_target_y,
                planner_name="direct_line_planner",
                task_id=None,
            ),
            current_waypoint_index=0,
        )

    route_length = len(patrol_route)
    if route_length == 0:
        return replace(execution_state, active_plan=None, current_waypoint_index=0)

    if (
        progress_state.cooldown_until_step > step
        and progress_state.blocked_goal_signature is not None
        and build_goal_signature(
            stage=ExecutionStage.RETURN_TO_PATROL,
            active_task=None,
            execution_state=execution_state,
        )
        == progress_state.blocked_goal_signature
    ):
        access = _find_local_patrol_access(
            agent,
            patrol_route=patrol_route,
            execution_state=execution_state,
            progress_state=progress_state,
            skip_blocked_goal=True,
        )
        if access is not None:
            execution_state = replace(
                execution_state,
                patrol_waypoint_index=access.segment_end_index,
                return_target_x=access.access_x,
                return_target_y=access.access_y,
                rejoin_to_segment=True,
            )

    if (
        not allow_patrol_index_advance
        and execution_state.return_target_x is not None
        and execution_state.return_target_y is not None
    ):
        current_target_plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=execution_state.return_target_x,
            goal_y=execution_state.return_target_y,
            planner_name="astar_path_planner",
            task_id=None,
            stats_context="runtime_return_to_patrol",
        )
        if current_target_plan.status == PathPlanStatus.PLANNED:
            return replace(
                execution_state,
                active_plan=current_target_plan,
                current_waypoint_index=0,
            )

    start_index = execution_state.patrol_waypoint_index % route_length
    if allow_patrol_index_advance:
        start_index = (start_index + 1) % route_length

    blocked_state: AgentExecutionState | None = None
    for offset in range(route_length):
        patrol_index = (start_index + offset) % route_length
        goal_x, goal_y = patrol_route[patrol_index]
        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name="astar_path_planner",
            task_id=None,
            stats_context="runtime_return_to_patrol",
        )
        candidate_state = replace(
            execution_state,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_waypoint_index=patrol_index,
            return_target_x=goal_x,
            return_target_y=goal_y,
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate_state
        if blocked_state is None:
            blocked_state = candidate_state

    access = _find_local_patrol_access(
        agent,
        patrol_route=patrol_route,
        execution_state=execution_state,
        progress_state=progress_state,
        skip_blocked_goal=allow_patrol_index_advance,
    )
    if access is not None:
        plan = build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=access.access_x,
            goal_y=access.access_y,
            planner_name="astar_path_planner",
            task_id=None,
            stats_context="runtime_return_to_patrol",
        )
        candidate_state = replace(
            execution_state,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_waypoint_index=access.segment_end_index,
            return_target_x=access.access_x,
            return_target_y=access.access_y,
            rejoin_to_segment=True,
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate_state
        if blocked_state is None:
            blocked_state = candidate_state

    return blocked_state or replace(execution_state, active_plan=None, current_waypoint_index=0)


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


def _evaluate_agent_progress(
    previous_agent: AgentState,
    *,
    updated_agent: AgentState,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    step: int,
) -> tuple[AgentExecutionState, AgentProgressState]:
    if updated_agent.kind != "USV":
        return (execution_state, progress_state)
    if execution_state.stage == ExecutionStage.RECOVERY:
        return (execution_state, progress_state)
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


def _progress_target(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord | None,
    patrol_route: tuple[tuple[float, float], ...],
) -> tuple[float | None, float | None]:
    plan = execution_state.active_plan
    if plan is not None and execution_state.current_waypoint_index < len(plan.waypoints):
        waypoint = plan.waypoints[execution_state.current_waypoint_index]
        return (waypoint.x, waypoint.y)
    if execution_state.stage == ExecutionStage.GO_TO_TASK and active_task is not None:
        return (active_task.target_x, active_task.target_y)
    if execution_state.stage == ExecutionStage.RETURN_TO_PATROL:
        return (execution_state.return_target_x, execution_state.return_target_y)
    if execution_state.stage == ExecutionStage.PATROL and patrol_route:
        target_x, target_y = patrol_route[execution_state.patrol_waypoint_index]
        return (target_x, target_y)
    return (None, None)


def _run_usv_recovery_step(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    patrol_route: tuple[tuple[float, float], ...],
    obstacle_layout: ObstacleLayout | None,
    dt_seconds: float,
    step: int,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if agent.kind != "USV":
        return (agent, transition_to_patrol(execution_state), reset_progress_state(progress_state))

    if progress_state.recovery_step_index == 0:
        return (
            _stop_agent_motion(assign_agent_task(agent, TaskMode.IDLE)),
            execution_state,
            replace(progress_state, recovery_step_index=1),
        )

    clearance_before = _clearance_to_hazards(agent, obstacle_layout)
    recovered_agent, moved = _execute_recovery_motion(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        obstacle_layout=obstacle_layout,
        dt_seconds=dt_seconds,
    )
    clearance_after = _clearance_to_hazards(recovered_agent, obstacle_layout)
    local_access = _find_local_patrol_access(
        recovered_agent,
        patrol_route=patrol_route,
        execution_state=execution_state,
        progress_state=progress_state,
        skip_blocked_goal=True,
    )
    if (
        moved
        and _clearance_improved(clearance_before, clearance_after)
        and local_access is not None
    ):
        if (
            progress_state.pre_recovery_stage == ExecutionStage.GO_TO_TASK
            and active_task is not None
        ):
            return (
                assign_agent_task(recovered_agent, TaskMode.CONFIRM),
                replace(
                    execution_state,
                    stage=ExecutionStage.GO_TO_TASK,
                    active_task_id=active_task.task_id,
                    active_plan=None,
                    current_waypoint_index=0,
                ),
                reset_progress_state(progress_state),
            )
        return (
            recovered_agent,
            _build_local_patrol_return_transition(
                recovered_agent,
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_route=patrol_route,
                skip_blocked_goal=True,
            ),
            reset_progress_state(progress_state),
        )

    next_recovery_step_index = progress_state.recovery_step_index + 1
    max_recovery_step_index = 5
    if next_recovery_step_index <= max_recovery_step_index:
        return (
            _stop_agent_motion(recovered_agent),
            execution_state,
            replace(progress_state, recovery_step_index=next_recovery_step_index),
        )

    recovery_attempts = progress_state.recovery_attempts + 1
    if (
        progress_state.pre_recovery_stage == ExecutionStage.GO_TO_TASK
        and recovery_attempts >= USV_MAX_RECOVERY_ATTEMPTS
    ):
        return _release_blocked_usv_task(
            _stop_agent_motion(assign_agent_task(recovered_agent, TaskMode.IDLE)),
            execution_state=execution_state,
            progress_state=replace(progress_state, recovery_attempts=recovery_attempts),
            patrol_routes={agent.agent_id: patrol_route},
        )
    return (
        _stop_agent_motion(recovered_agent),
        _build_local_patrol_return_transition(
            recovered_agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_route,
            skip_blocked_goal=True,
        ),
        replace(
            reset_progress_state(progress_state),
            recovery_attempts=recovery_attempts,
        ),
    )


def _execute_recovery_motion(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    obstacle_layout: ObstacleLayout | None,
    dt_seconds: float,
) -> tuple[AgentState, bool]:
    target = _recovery_waypoint(agent, progress_state.recovery_step_index)
    recovery_agent = assign_agent_task(agent, TaskMode.PATROL, target.x, target.y)
    advanced_agent = advance_agent_towards_task(recovery_agent, dt_seconds)
    advanced_agent, _ = _apply_usv_collision_guard(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=obstacle_layout,
    )
    moved = _agent_moved(agent, advanced_agent)
    return (advanced_agent, moved)


def _recovery_waypoint(agent: AgentState, recovery_step_index: int) -> Waypoint:
    if recovery_step_index == 1:
        angle_deg = agent.heading_deg + 180.0
        distance = USV_RECOVERY_REVERSE_DISTANCE_M
    elif recovery_step_index == 2:
        angle_deg = agent.heading_deg + 25.0
        distance = USV_RECOVERY_FORWARD_DISTANCE_M
    elif recovery_step_index == 3:
        angle_deg = agent.heading_deg - 25.0
        distance = USV_RECOVERY_FORWARD_DISTANCE_M
    elif recovery_step_index == 4:
        angle_deg = agent.heading_deg + 45.0
        distance = USV_RECOVERY_FORWARD_DISTANCE_M
    else:
        angle_deg = agent.heading_deg - 45.0
        distance = USV_RECOVERY_FORWARD_DISTANCE_M
    return replace(
        Waypoint(x=0.0, y=0.0),
        x=agent.x + cos(radians(angle_deg)) * distance,
        y=agent.y + sin(radians(angle_deg)) * distance,
    )


def _clearance_to_hazards(agent: AgentState, obstacle_layout: ObstacleLayout | None) -> float:
    if obstacle_layout is None:
        return float("inf")
    best_distance = float("inf")
    for obstacle in obstacle_layout.risk_zone_obstacles:
        if _point_in_polygon(agent.x, agent.y, obstacle.points):
            return 0.0
        best_distance = min(
            best_distance,
            _distance_to_polygon_edges(agent.x, agent.y, obstacle.points),
        )
    for feature in obstacle_layout.offshore_features:
        if feature.feature_type != "islet":
            continue
        best_distance = min(
            best_distance,
            hypot(agent.x - feature.x, agent.y - feature.y) - feature.radius,
        )
    return best_distance


def _clearance_improved(clearance_before: float, clearance_after: float) -> bool:
    if clearance_before == float("inf") or clearance_before >= USV_COLLISION_CLEARANCE_M * 1.5:
        return True
    return clearance_after >= clearance_before + USV_RECOVERY_CLEARANCE_IMPROVEMENT_M


def _apply_usv_collision_guard(
    previous_agent: AgentState,
    *,
    advanced_agent: AgentState,
    execution_state: AgentExecutionState,
    obstacle_layout: ObstacleLayout | None,
) -> tuple[AgentState, AgentExecutionState]:
    if previous_agent.kind != "USV" or obstacle_layout is None:
        return (advanced_agent, execution_state)
    safe_ratio = _safe_motion_ratio(previous_agent, advanced_agent, obstacle_layout)
    if safe_ratio >= 1.0:
        return (advanced_agent, execution_state)
    guarded_agent = _interpolate_agent_motion(previous_agent, advanced_agent, safe_ratio)
    guarded_agent = replace(
        guarded_agent,
        heading_deg=advanced_agent.heading_deg,
        speed_mps=0.0,
        turn_rate_degps=advanced_agent.turn_rate_degps,
    )
    guarded_state = replace(execution_state, active_plan=None, current_waypoint_index=0)
    return (guarded_agent, guarded_state)


def _safe_motion_ratio(
    previous_agent: AgentState,
    advanced_agent: AgentState,
    obstacle_layout: ObstacleLayout,
) -> float:
    travel_distance = hypot(
        advanced_agent.x - previous_agent.x,
        advanced_agent.y - previous_agent.y,
    )
    if travel_distance <= 1e-9:
        return 1.0
    sample_count = max(2, int(travel_distance / USV_COLLISION_SAMPLE_SPACING_M) + 1)
    last_safe_ratio = 0.0
    for sample_index in range(1, sample_count + 1):
        ratio = sample_index / sample_count
        sample_x = previous_agent.x + (advanced_agent.x - previous_agent.x) * ratio
        sample_y = previous_agent.y + (advanced_agent.y - previous_agent.y) * ratio
        if _hits_any_obstacle(sample_x, sample_y, obstacle_layout):
            return last_safe_ratio
        last_safe_ratio = ratio
    return 1.0


def _hits_any_obstacle(x: float, y: float, obstacle_layout: ObstacleLayout) -> bool:
    for obstacle in obstacle_layout.risk_zone_obstacles:
        if _point_in_polygon(x, y, obstacle.points):
            return True
        if _distance_to_polygon_edges(x, y, obstacle.points) <= USV_COLLISION_CLEARANCE_M:
            return True
    for feature in obstacle_layout.offshore_features:
        if feature.feature_type != "islet":
            continue
        if hypot(x - feature.x, y - feature.y) <= feature.radius + USV_COLLISION_CLEARANCE_M:
            return True
    return False


def _point_in_polygon(
    x: float,
    y: float,
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            x_intersection = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_intersection:
                inside = not inside
        j = i
    return inside


def _distance_to_polygon_edges(
    x: float,
    y: float,
    polygon: tuple[tuple[float, float], ...],
) -> float:
    best_distance = float("inf")
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        best_distance = min(best_distance, _distance_to_segment(x, y, start, end))
    return best_distance


def _distance_to_segment(
    x: float,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    start_x, start_y = start
    end_x, end_y = end
    segment_dx = end_x - start_x
    segment_dy = end_y - start_y
    segment_length_sq = segment_dx**2 + segment_dy**2
    if segment_length_sq <= 1e-9:
        return hypot(x - start_x, y - start_y)
    projection = ((x - start_x) * segment_dx + (y - start_y) * segment_dy) / segment_length_sq
    ratio = min(max(projection, 0.0), 1.0)
    closest_x = start_x + segment_dx * ratio
    closest_y = start_y + segment_dy * ratio
    return hypot(x - closest_x, y - closest_y)


def _interpolate_agent_motion(
    start_agent: AgentState,
    end_agent: AgentState,
    ratio: float,
) -> AgentState:
    heading_delta = shortest_heading_delta_deg(start_agent.heading_deg, end_agent.heading_deg)
    return replace(
        start_agent,
        x=start_agent.x + (end_agent.x - start_agent.x) * ratio,
        y=start_agent.y + (end_agent.y - start_agent.y) * ratio,
        heading_deg=start_agent.heading_deg + heading_delta * ratio,
        speed_mps=start_agent.speed_mps + (end_agent.speed_mps - start_agent.speed_mps) * ratio,
        turn_rate_degps=start_agent.turn_rate_degps
        + (end_agent.turn_rate_degps - start_agent.turn_rate_degps) * ratio,
    )
