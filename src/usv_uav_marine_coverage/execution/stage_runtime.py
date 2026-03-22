"""Execution-stage handlers extracted from the simulation runtime."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    AgentTaskState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
    can_support_uav_resupply,
    recharge_agent_energy,
)
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_on_task,
    transition_to_patrol,
    transition_to_recharge,
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
from usv_uav_marine_coverage.execution.progress_feedback import should_replan_task
from usv_uav_marine_coverage.execution.recharge_runtime import (
    _dock_to_support_agent,
    _has_reached_rendezvous,
)
from usv_uav_marine_coverage.execution.recovery_runtime import (
    _release_blocked_usv_task,
    _run_usv_recovery_step,
)
from usv_uav_marine_coverage.execution.return_to_patrol_runtime import (
    _plan_patrol_path,
    _plan_return_to_patrol_path,
    _should_refresh_patrol_plan,
    _should_refresh_return_plan,
)
from usv_uav_marine_coverage.execution.task_final_approach_runtime import (
    apply_task_final_approach_selection,
    select_task_final_approach_candidate,
    task_final_approach_satisfied,
)
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.traffic_cost import build_traffic_cost_context
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
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan
from usv_uav_marine_coverage.simulation.uav_coverage_runtime import (
    resolve_persistent_uav_coverage_state,
    uav_offshore_x_bounds,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType


@dataclass(frozen=True)
class StageRuntimeContext:
    """Shared per-step execution context for one stage handler."""

    agent_by_id: dict[str, AgentState]
    patrol_routes: dict[str, tuple[tuple[float, float], ...]]
    grid_map: GridMap
    info_map: InformationMap | None
    obstacle_layout: ObstacleLayout | None
    wreck_zones: tuple[WreckZone, ...]
    dt_seconds: float
    step: int
    usv_path_planner: str
    uav_search_planner: str
    execution_policy: str
    execution_states: dict[str, AgentExecutionState]
    uav_coverage_states: dict[str, UavCoverageState] | None
    task_records: tuple[TaskRecord, ...]
    advance_path_execution_step: Callable[
        ...,
        tuple[AgentState, AgentExecutionState, ExecutionOutcome],
    ]
    neighboring_agents: Callable[[AgentState, dict[str, AgentState]], tuple[AgentState, ...]]
    stop_agent_motion: Callable[[AgentState], AgentState]
    agent_moved: Callable[[AgentState, AgentState], bool]


def _build_usv_traffic_cost_context(
    *,
    agent_id: str,
    context: StageRuntimeContext,
):
    """Build one lightweight traffic-aware planning snapshot for a single USV."""

    reference_plans = tuple(
        execution_state.active_plan
        for other_agent_id, execution_state in context.execution_states.items()
        if other_agent_id != agent_id
        and execution_state.active_plan is not None
        and execution_state.active_plan.status == PathPlanStatus.PLANNED
    )
    return build_traffic_cost_context(
        agent_id=agent_id,
        reference_plans=reference_plans,
        grid_map=context.grid_map,
        obstacle_layout=context.obstacle_layout,
    )


def run_agent_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    """Advance one non-yield execution stage by one simulation step."""

    if execution_state.stage == ExecutionStage.RECOVERY:
        return _run_recovery_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            context=context,
        )
    if execution_state.stage == ExecutionStage.FAILED:
        return _run_failed_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            context=context,
        )
    if execution_state.stage == ExecutionStage.ON_TASK:
        return _run_on_task_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            context=context,
        )
    if execution_state.stage == ExecutionStage.ON_RECHARGE:
        return _run_on_recharge_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            context=context,
        )
    if execution_state.stage == ExecutionStage.GO_TO_TASK and active_task is not None:
        return _run_go_to_task_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            context=context,
        )
    if execution_state.stage == ExecutionStage.GO_TO_RENDEZVOUS and active_task is not None:
        return _run_go_to_rendezvous_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            active_task=active_task,
            context=context,
        )
    if execution_state.stage == ExecutionStage.RETURN_TO_PATROL:
        return _run_return_to_patrol_stage(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            context=context,
        )
    return _run_patrol_stage(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        context=context,
    )


def _run_recovery_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    return _run_usv_recovery_step(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        active_task=active_task,
        patrol_route=context.patrol_routes[agent.agent_id],
        grid_map=context.grid_map,
        info_map=context.info_map,
        obstacle_layout=context.obstacle_layout,
        wreck_zones=context.wreck_zones,
        dt_seconds=context.dt_seconds,
        step=context.step,
        task_records=context.task_records,
        usv_path_planner=context.usv_path_planner,
        traffic_cost_context=_build_usv_traffic_cost_context(
            agent_id=agent.agent_id,
            context=context,
        ),
    )


def _run_failed_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    from usv_uav_marine_coverage.execution.progress_feedback import reset_progress_state

    return (
        replace(
            context.stop_agent_motion(agent),
            task=AgentTaskState(mode=TaskMode.IDLE),
        ),
        execution_state,
        reset_progress_state(progress_state),
    )


def _run_on_task_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    confirm_agent = assign_agent_task(
        agent,
        TaskMode.CONFIRM if agent.kind == "USV" else TaskMode.INVESTIGATE,
    )
    return (
        advance_agent_towards_task(confirm_agent, context.dt_seconds),
        execution_state,
        progress_state,
    )


def _run_on_recharge_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    from usv_uav_marine_coverage.execution.progress_feedback import reset_progress_state

    if active_task is not None and active_task.task_type == TaskType.UAV_RESUPPLY:
        support_agent = (
            None
            if active_task.support_agent_id is None
            else context.agent_by_id.get(active_task.support_agent_id)
        )
        if support_agent is None or not can_support_uav_resupply(support_agent):
            return (
                assign_agent_task(agent, TaskMode.IDLE),
                transition_to_patrol(execution_state),
                reset_progress_state(progress_state),
            )
    idle_agent = assign_agent_task(agent, TaskMode.IDLE)
    return (
        recharge_agent_energy(idle_agent, context.dt_seconds),
        execution_state,
        progress_state,
    )


def _run_go_to_task_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    selection = (
        select_task_final_approach_candidate(
            agent,
            task=active_task,
            grid_map=context.grid_map,
            progress_state=progress_state,
        )
        if agent.kind == "USV"
        else None
    )
    if selection is not None:
        progress_state = apply_task_final_approach_selection(
            progress_state,
            selection=selection,
        )
        goal_x = selection.candidate_x
        goal_y = selection.candidate_y
    else:
        goal_x = active_task.target_x
        goal_y = active_task.target_y
    if should_replan_task(
        agent,
        execution_state=execution_state,
        active_task=active_task,
        usv_path_planner=context.usv_path_planner,
        goal_x=goal_x,
        goal_y=goal_y,
    ):
        traffic_cost_context = _build_usv_traffic_cost_context(
            agent_id=agent.agent_id,
            context=context,
        )
        if agent.kind == "USV":
            plan = build_usv_path_plan(
                agent,
                grid_map=context.grid_map,
                obstacle_layout=context.obstacle_layout,
                goal_x=goal_x,
                goal_y=goal_y,
                planner_name=context.usv_path_planner,
                task_id=active_task.task_id,
                stats_context="runtime_go_to_task",
                traffic_cost_context=traffic_cost_context,
            )
        else:
            plan = build_direct_line_plan(
                agent,
                goal_x=goal_x,
                goal_y=goal_y,
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
            context.stop_agent_motion(assign_agent_task(agent, TaskMode.IDLE)),
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_routes=context.patrol_routes,
            grid_map=context.grid_map,
            info_map=context.info_map,
            task_records=context.task_records,
        )
    if plan.status != PathPlanStatus.PLANNED:
        if agent.kind != "USV":
            return (agent, execution_state, progress_state)
        return _release_blocked_usv_task(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_routes=context.patrol_routes,
            grid_map=context.grid_map,
            info_map=context.info_map,
            task_records=context.task_records,
        )

    advanced_agent, execution_state, outcome = context.advance_path_execution_step(
        agent,
        execution_state=execution_state,
        grid_map=context.grid_map,
        dt_seconds=context.dt_seconds,
        obstacle_layout=context.obstacle_layout,
        execution_policy=context.execution_policy,
        neighboring_agents=context.neighboring_agents(agent, context.agent_by_id),
        wreck_zones=context.wreck_zones,
    )
    advanced_agent, execution_state = _apply_collision_guard_with_optional_wrecks(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=context.obstacle_layout,
        wreck_zones=context.wreck_zones,
    )
    if (
        agent.kind == "USV"
        and execution_state.active_plan is None
        and not context.agent_moved(agent, advanced_agent)
    ):
        return _release_blocked_usv_task(
            context.stop_agent_motion(assign_agent_task(advanced_agent, TaskMode.IDLE)),
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_routes=context.patrol_routes,
            grid_map=context.grid_map,
            info_map=context.info_map,
            task_records=context.task_records,
        )
    if _should_enter_on_task(
        advanced_agent,
        active_task=active_task,
        outcome=outcome,
    ):
        return (advanced_agent, transition_to_on_task(execution_state), progress_state)
    return (advanced_agent, execution_state, progress_state)


def _should_enter_on_task(
    agent: AgentState,
    *,
    active_task: TaskRecord,
    outcome: ExecutionOutcome,
) -> bool:
    """Return whether one task-approach step should transition into ON_TASK."""

    if active_task.task_type in {
        TaskType.BASELINE_SERVICE,
        TaskType.HOTSPOT_CONFIRMATION,
    }:
        return task_final_approach_satisfied(agent, task=active_task)
    if outcome == ExecutionOutcome.TASK_SITE_REACHED:
        return True
    return False


def _run_go_to_rendezvous_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    from usv_uav_marine_coverage.execution.progress_feedback import reset_progress_state

    support_agent = (
        None
        if active_task.support_agent_id is None
        else context.agent_by_id.get(active_task.support_agent_id)
    )
    if support_agent is None or not can_support_uav_resupply(support_agent):
        return (
            assign_agent_task(agent, TaskMode.IDLE),
            transition_to_patrol(execution_state),
            reset_progress_state(progress_state),
        )
    if _has_reached_rendezvous(agent, support_agent):
        docked_agent = _dock_to_support_agent(agent, support_agent)
        return (docked_agent, transition_to_recharge(execution_state), progress_state)

    execution_state = replace(
        execution_state,
        active_plan=build_direct_line_plan(
            agent,
            goal_x=support_agent.x,
            goal_y=support_agent.y,
            planner_name="uav_rendezvous_planner",
            task_id=active_task.task_id,
        ),
        current_waypoint_index=0,
    )
    advanced_agent, execution_state, outcome = context.advance_path_execution_step(
        agent,
        execution_state=execution_state,
        grid_map=context.grid_map,
        dt_seconds=context.dt_seconds,
        obstacle_layout=context.obstacle_layout,
        execution_policy=context.execution_policy,
        neighboring_agents=context.neighboring_agents(agent, context.agent_by_id),
        wreck_zones=context.wreck_zones,
    )
    if outcome == ExecutionOutcome.TASK_SITE_REACHED or _has_reached_rendezvous(
        advanced_agent,
        support_agent,
    ):
        docked_agent = _dock_to_support_agent(advanced_agent, support_agent)
        return (docked_agent, transition_to_recharge(execution_state), progress_state)
    return (advanced_agent, execution_state, progress_state)


def _run_return_to_patrol_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    assert (
        execution_state.return_target_x is not None and execution_state.return_target_y is not None
    )
    if _should_refresh_return_plan(
        agent,
        execution_state=execution_state,
        step=context.step,
        usv_path_planner=context.usv_path_planner,
    ):
        traffic_cost_context = _build_usv_traffic_cost_context(
            agent_id=agent.agent_id,
            context=context,
        )
        execution_state = _plan_return_to_patrol_path(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=context.patrol_routes[agent.agent_id],
            grid_map=context.grid_map,
            obstacle_layout=context.obstacle_layout,
            info_map=context.info_map,
            allow_patrol_index_advance=False,
            step=context.step,
            task_records=context.task_records,
            usv_path_planner=context.usv_path_planner,
            traffic_cost_context=traffic_cost_context,
        )
    plan = execution_state.active_plan
    if plan is None:
        return (agent, execution_state, progress_state)
    if plan.status != PathPlanStatus.PLANNED:
        traffic_cost_context = _build_usv_traffic_cost_context(
            agent_id=agent.agent_id,
            context=context,
        )
        execution_state = _plan_return_to_patrol_path(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=context.patrol_routes[agent.agent_id],
            grid_map=context.grid_map,
            obstacle_layout=context.obstacle_layout,
            info_map=context.info_map,
            allow_patrol_index_advance=True,
            step=context.step,
            task_records=context.task_records,
            usv_path_planner=context.usv_path_planner,
            traffic_cost_context=traffic_cost_context,
        )
        plan = execution_state.active_plan
        if plan is None or plan.status != PathPlanStatus.PLANNED:
            return (
                assign_agent_task(agent, TaskMode.PATROL),
                execution_state,
                progress_state,
            )

    advanced_agent, execution_state, outcome = context.advance_path_execution_step(
        agent,
        execution_state=execution_state,
        grid_map=context.grid_map,
        dt_seconds=context.dt_seconds,
        obstacle_layout=context.obstacle_layout,
        execution_policy=context.execution_policy,
        neighboring_agents=context.neighboring_agents(agent, context.agent_by_id),
        wreck_zones=context.wreck_zones,
    )
    advanced_agent, execution_state = _apply_collision_guard_with_optional_wrecks(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=context.obstacle_layout,
        wreck_zones=context.wreck_zones,
    )
    if (
        agent.kind == "USV"
        and execution_state.active_plan is None
        and not context.agent_moved(agent, advanced_agent)
    ):
        traffic_cost_context = _build_usv_traffic_cost_context(
            agent_id=agent.agent_id,
            context=context,
        )
        execution_state = _plan_return_to_patrol_path(
            agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=context.patrol_routes[agent.agent_id],
            grid_map=context.grid_map,
            obstacle_layout=context.obstacle_layout,
            info_map=context.info_map,
            allow_patrol_index_advance=True,
            step=context.step,
            task_records=context.task_records,
            usv_path_planner=context.usv_path_planner,
            traffic_cost_context=traffic_cost_context,
        )
    if outcome == ExecutionOutcome.PATROL_REJOINED:
        next_index = execution_state.patrol_waypoint_index
        if not execution_state.rejoin_to_segment:
            next_index = (next_index + 1) % len(context.patrol_routes[agent.agent_id])
        return (
            advanced_agent,
            replace(
                transition_to_patrol(execution_state),
                patrol_waypoint_index=next_index,
            ),
            progress_state,
        )
    return (advanced_agent, execution_state, progress_state)


def _run_patrol_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if agent.kind == "UAV":
        execution_state = _build_uav_patrol_state(
            agent,
            execution_state=execution_state,
            context=context,
        )
    else:
        patrol_route = context.patrol_routes[execution_state.patrol_route_id or agent.agent_id]
        patrol_goal_x, patrol_goal_y = patrol_route[execution_state.patrol_waypoint_index]
        if _should_refresh_patrol_plan(
            agent,
            execution_state=execution_state,
            goal_x=patrol_goal_x,
            goal_y=patrol_goal_y,
            step=context.step,
            usv_path_planner=context.usv_path_planner,
        ):
            execution_state = _plan_patrol_path(
                agent,
                execution_state=execution_state,
                patrol_route=patrol_route,
                grid_map=context.grid_map,
                obstacle_layout=context.obstacle_layout,
                allow_patrol_index_advance=False,
                usv_path_planner=context.usv_path_planner,
                step=context.step,
                traffic_cost_context=_build_usv_traffic_cost_context(
                    agent_id=agent.agent_id,
                    context=context,
                ),
            )
        plan = execution_state.active_plan
        if plan is None or plan.status != PathPlanStatus.PLANNED:
            stalled_agent = context.stop_agent_motion(
                assign_agent_task(agent, TaskMode.PATROL),
            )
            execution_state = _plan_patrol_path(
                stalled_agent,
                execution_state=replace(
                    execution_state,
                    active_plan=None,
                    current_waypoint_index=0,
                ),
                patrol_route=patrol_route,
                grid_map=context.grid_map,
                obstacle_layout=context.obstacle_layout,
                allow_patrol_index_advance=True,
                usv_path_planner=context.usv_path_planner,
                step=context.step,
                traffic_cost_context=_build_usv_traffic_cost_context(
                    agent_id=agent.agent_id,
                    context=context,
                ),
            )
            plan = execution_state.active_plan
            if plan is None or plan.status != PathPlanStatus.PLANNED:
                return (stalled_agent, execution_state, progress_state)

    advanced_agent, execution_state, outcome = context.advance_path_execution_step(
        agent,
        execution_state=execution_state,
        grid_map=context.grid_map,
        dt_seconds=context.dt_seconds,
        obstacle_layout=context.obstacle_layout,
        execution_policy=context.execution_policy,
        neighboring_agents=context.neighboring_agents(agent, context.agent_by_id),
        wreck_zones=context.wreck_zones,
    )
    advanced_agent, execution_state = _apply_collision_guard_with_optional_wrecks(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=context.obstacle_layout,
        wreck_zones=context.wreck_zones,
    )
    if (
        agent.kind == "USV"
        and execution_state.stage == ExecutionStage.PATROL
        and execution_state.active_plan is None
        and not context.agent_moved(agent, advanced_agent)
    ):
        advanced_agent = context.stop_agent_motion(
            assign_agent_task(advanced_agent, TaskMode.PATROL),
        )
        execution_state = _plan_patrol_path(
            advanced_agent,
            execution_state=replace(
                execution_state,
                active_plan=None,
                current_waypoint_index=0,
            ),
            patrol_route=context.patrol_routes[agent.agent_id],
            grid_map=context.grid_map,
            obstacle_layout=context.obstacle_layout,
            allow_patrol_index_advance=True,
            usv_path_planner=context.usv_path_planner,
            step=context.step,
            traffic_cost_context=_build_usv_traffic_cost_context(
                agent_id=agent.agent_id,
                context=context,
            ),
        )
        return (advanced_agent, execution_state, progress_state)
    if outcome == ExecutionOutcome.WAYPOINT_REACHED:
        return _advance_patrol_waypoint(
            advanced_agent,
            execution_state=execution_state,
            progress_state=progress_state,
            context=context,
        )
    return (advanced_agent, execution_state, progress_state)


def _build_uav_patrol_state(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    context: StageRuntimeContext,
) -> AgentExecutionState:
    route_id = execution_state.patrol_route_id or agent.agent_id
    if context.uav_search_planner == "uav_persistent_multi_region_coverage_planner":
        persistent_state_store = (
            {} if context.uav_coverage_states is None else context.uav_coverage_states
        )
        persistent_coverage_state = persistent_state_store.get(agent.agent_id)
        if persistent_coverage_state is None:
            persistent_coverage_state = build_empty_uav_coverage_state(agent.agent_id)
        persistent_coverage_state = resolve_persistent_uav_coverage_state(
            agent=agent,
            coverage_state=persistent_coverage_state,
            grid_map=context.grid_map,
            info_map=context.info_map,
            step=context.step,
            all_coverage_states=persistent_state_store,
        )
        persistent_state_store[agent.agent_id] = persistent_coverage_state
        context.patrol_routes[route_id] = persistent_coverage_state.region_route
        execution_state = replace(
            execution_state,
            patrol_waypoint_index=persistent_coverage_state.region_waypoint_index,
            active_plan=build_uav_persistent_multi_region_plan(
                agent,
                coverage_state=persistent_coverage_state,
            ),
            current_waypoint_index=0,
        )
        return execution_state

    if context.uav_search_planner == "uav_multi_region_coverage_planner":
        offshore_min_x, offshore_max_x = uav_offshore_x_bounds(context.grid_map)
        patrol_route = build_uav_multi_region_route(
            agent,
            info_map=context.info_map,
            min_x=offshore_min_x + UAV_MULTI_REGION_X_MARGIN_LEFT,
            max_x=offshore_max_x - UAV_MULTI_REGION_X_MARGIN_RIGHT,
            min_y=UAV_MULTI_REGION_MIN_Y,
            max_y=UAV_MULTI_REGION_MAX_Y,
            lane_spacing=UAV_MULTI_REGION_LANE_SPACING_M,
        )
        context.patrol_routes[route_id] = patrol_route
        patrol_waypoint_index = select_uav_multi_region_waypoint_index(
            agent,
            patrol_route=patrol_route,
            patrol_waypoint_index=execution_state.patrol_waypoint_index,
        )
        return replace(
            execution_state,
            patrol_waypoint_index=patrol_waypoint_index,
            active_plan=build_uav_multi_region_plan(
                agent,
                patrol_route_id=route_id,
                patrol_waypoint_index=patrol_waypoint_index,
                patrol_routes=context.patrol_routes,
            ),
            current_waypoint_index=0,
        )

    return replace(
        execution_state,
        active_plan=build_uav_lawnmower_plan(
            agent,
            patrol_route_id=route_id,
            patrol_waypoint_index=execution_state.patrol_waypoint_index,
            patrol_routes=context.patrol_routes,
        ),
        current_waypoint_index=0,
    )


def _advance_patrol_waypoint(
    advanced_agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    context: StageRuntimeContext,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if (
        advanced_agent.kind == "UAV"
        and context.uav_search_planner == "uav_persistent_multi_region_coverage_planner"
    ):
        persistent_state_store = (
            {} if context.uav_coverage_states is None else context.uav_coverage_states
        )
        persistent_coverage_state = persistent_state_store.get(advanced_agent.agent_id)
        if persistent_coverage_state is None:
            persistent_coverage_state = build_empty_uav_coverage_state(
                advanced_agent.agent_id,
            )
        persistent_coverage_state = advance_uav_region_waypoint(persistent_coverage_state)
        persistent_state_store[advanced_agent.agent_id] = persistent_coverage_state
        if persistent_coverage_state.region_route:
            context.patrol_routes[advanced_agent.agent_id] = persistent_coverage_state.region_route
            next_index = persistent_coverage_state.region_waypoint_index
        else:
            context.patrol_routes[advanced_agent.agent_id] = ()
            next_index = 0
    elif (
        advanced_agent.kind == "UAV"
        and context.uav_search_planner == "uav_multi_region_coverage_planner"
    ):
        next_index = 0
    else:
        next_index = (execution_state.patrol_waypoint_index + 1) % len(
            context.patrol_routes[advanced_agent.agent_id]
        )
    execution_state = replace(
        execution_state,
        active_plan=None,
        current_waypoint_index=0,
        patrol_waypoint_index=next_index,
    )
    return (advanced_agent, execution_state, progress_state)


__all__ = ["StageRuntimeContext", "run_agent_stage"]
