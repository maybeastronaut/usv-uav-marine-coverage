"""Execution-time USV recovery helpers."""

from __future__ import annotations

from dataclasses import replace
from math import cos, hypot, radians, sin

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
)
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.planning.path_types import Waypoint
from usv_uav_marine_coverage.planning.traffic_cost import TrafficCostContext
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType

from .collision_guard import _apply_collision_guard_with_optional_wrecks
from .execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionStage,
    WreckZone,
)
from .progress_feedback import record_released_task_feedback, reset_progress_state
from .return_to_patrol_runtime import (
    _build_local_patrol_return_transition,
    _build_planned_local_patrol_return_transition,
    _find_local_patrol_access,
)
from .task_final_approach_runtime import (
    advance_task_final_approach_after_failure,
    apply_task_final_approach_selection,
    task_final_approach_release_cooldown_steps,
)

USV_MAX_RECOVERY_ATTEMPTS = 2
USV_RECOVERY_REVERSE_DISTANCE_M = 18.0
USV_RECOVERY_FORWARD_DISTANCE_M = 26.0
USV_RECOVERY_CLEARANCE_IMPROVEMENT_M = 0.5
BLOCKED_TASK_RELEASE_COOLDOWN_STEPS = 12


def _release_blocked_usv_task(
    agent: AgentState,
    *,
    step: int,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    grid_map: GridMap,
    info_map: InformationMap | None,
    task_records: tuple[TaskRecord, ...],
    released_task: TaskRecord | None = None,
    release_retry_until_step: int = 0,
    release_reason: str | None = None,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    next_execution_state = _build_local_patrol_return_transition(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        patrol_route=patrol_routes[agent.agent_id],
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
        skip_blocked_goal=True,
    )
    next_progress_state = reset_progress_state(progress_state)
    if released_task is not None and release_reason is not None and release_retry_until_step > 0:
        next_progress_state = record_released_task_feedback(
            progress_state,
            task_id=released_task.task_id,
            task_created_step=released_task.created_step,
            task_step=step,
            retry_until_step=release_retry_until_step,
            reason=release_reason,
        )
    return (
        _stop_agent_motion(assign_agent_task(agent, TaskMode.IDLE)),
        next_execution_state,
        next_progress_state,
    )


def blocked_task_release_retry_until_step(step: int) -> int:
    """Return the next step until which one blocked task should stay off this USV."""

    return step + BLOCKED_TASK_RELEASE_COOLDOWN_STEPS


def _run_usv_recovery_step(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    info_map: InformationMap | None,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
    dt_seconds: float,
    step: int,
    task_records: tuple[TaskRecord, ...],
    usv_path_planner: str,
    traffic_cost_context: TrafficCostContext | None = None,
) -> tuple[AgentState, AgentExecutionState, AgentProgressState]:
    if agent.kind != "USV":
        from .basic_state_machine import transition_to_patrol

        return (agent, transition_to_patrol(execution_state), reset_progress_state(progress_state))

    if progress_state.recovery_step_index == 0:
        return (
            _stop_agent_motion(assign_agent_task(agent, TaskMode.IDLE)),
            execution_state,
            replace(progress_state, recovery_step_index=1),
        )

    clearance_before = _clearance_to_hazards(agent, obstacle_layout, wreck_zones)
    recovered_agent, moved = _execute_recovery_motion(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        obstacle_layout=obstacle_layout,
        wreck_zones=wreck_zones,
        dt_seconds=dt_seconds,
    )
    clearance_after = _clearance_to_hazards(recovered_agent, obstacle_layout, wreck_zones)
    local_access = _find_local_patrol_access(
        recovered_agent,
        patrol_route=patrol_route,
        execution_state=execution_state,
        progress_state=progress_state,
        skip_blocked_goal=True,
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
    )
    if (
        moved
        and _clearance_improved(clearance_before, clearance_after)
        and local_access is not None
    ):
        if (
            progress_state.pre_recovery_stage == ExecutionStage.GO_TO_TASK
            and active_task is not None
            and active_task.task_type in {
                TaskType.BASELINE_SERVICE,
                TaskType.HOTSPOT_CONFIRMATION,
            }
        ):
            next_selection, exhausted = advance_task_final_approach_after_failure(
                recovered_agent,
                task=active_task,
                grid_map=grid_map,
                progress_state=progress_state,
            )
            if exhausted:
                return _release_blocked_usv_task(
                    _stop_agent_motion(assign_agent_task(recovered_agent, TaskMode.IDLE)),
                    step=step,
                    execution_state=execution_state,
                    progress_state=progress_state,
                    patrol_routes={agent.agent_id: patrol_route},
                    grid_map=grid_map,
                    info_map=info_map,
                    task_records=task_records,
                    released_task=active_task,
                    release_retry_until_step=(
                        step + task_final_approach_release_cooldown_steps(active_task)
                    ),
                    release_reason="task_final_approach_exhausted",
                )
            assert next_selection is not None
            return (
                assign_agent_task(
                    recovered_agent,
                    TaskMode.CONFIRM,
                    next_selection.candidate_x,
                    next_selection.candidate_y,
                ),
                replace(
                    execution_state,
                    stage=ExecutionStage.GO_TO_TASK,
                    active_task_id=active_task.task_id,
                    active_plan=None,
                    current_waypoint_index=0,
                ),
                apply_task_final_approach_selection(
                    reset_progress_state(progress_state),
                    selection=next_selection,
                ),
            )
        if (
            progress_state.pre_recovery_stage == ExecutionStage.GO_TO_TASK
            and active_task is not None
        ):
            return (
                assign_agent_task(
                    recovered_agent,
                    TaskMode.CONFIRM if recovered_agent.kind == "USV" else TaskMode.INVESTIGATE,
                    active_task.target_x,
                    active_task.target_y,
                ),
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
            _build_planned_local_patrol_return_transition(
                recovered_agent,
                execution_state=execution_state,
                progress_state=progress_state,
                patrol_route=patrol_route,
                grid_map=grid_map,
                obstacle_layout=obstacle_layout,
                info_map=info_map,
                task_records=task_records,
                skip_blocked_goal=True,
                step=step,
                usv_path_planner=usv_path_planner,
                traffic_cost_context=traffic_cost_context,
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
            step=step,
            execution_state=execution_state,
            progress_state=replace(progress_state, recovery_attempts=recovery_attempts),
            patrol_routes={agent.agent_id: patrol_route},
            grid_map=grid_map,
            info_map=info_map,
            task_records=task_records,
            released_task=active_task,
            release_retry_until_step=(
                0 if active_task is None else blocked_task_release_retry_until_step(step)
            ),
            release_reason="recovery_exhausted",
        )
    return (
        _stop_agent_motion(recovered_agent),
        _build_planned_local_patrol_return_transition(
            recovered_agent,
            execution_state=execution_state,
            progress_state=progress_state,
            patrol_route=patrol_route,
            grid_map=grid_map,
            obstacle_layout=obstacle_layout,
            info_map=info_map,
            task_records=task_records,
            skip_blocked_goal=True,
            step=step,
            usv_path_planner=usv_path_planner,
            traffic_cost_context=traffic_cost_context,
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
    wreck_zones: tuple[WreckZone, ...],
    dt_seconds: float,
) -> tuple[AgentState, bool]:
    target = _recovery_waypoint(agent, progress_state.recovery_step_index)
    recovery_agent = assign_agent_task(agent, TaskMode.PATROL, target.x, target.y)
    advanced_agent = advance_agent_towards_task(recovery_agent, dt_seconds)
    advanced_agent, _ = _apply_collision_guard_with_optional_wrecks(
        agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=obstacle_layout,
        wreck_zones=wreck_zones,
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


def _clearance_to_hazards(
    agent: AgentState,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
) -> float:
    if obstacle_layout is None and not wreck_zones:
        return float("inf")
    best_distance = float("inf")
    if obstacle_layout is not None:
        from .collision_guard import _distance_to_polygon_edges, _point_in_polygon

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
    for wreck in wreck_zones:
        best_distance = min(
            best_distance,
            hypot(agent.x - wreck.x, agent.y - wreck.y) - wreck.radius,
        )
    return best_distance


def _clearance_improved(clearance_before: float, clearance_after: float) -> bool:
    if clearance_before == float("inf") or clearance_before >= USV_COLLISION_CLEARANCE_M * 1.5:
        return True
    return clearance_after >= clearance_before + USV_RECOVERY_CLEARANCE_IMPROVEMENT_M


def _agent_moved(previous_agent: AgentState, advanced_agent: AgentState) -> bool:
    return (
        hypot(
            advanced_agent.x - previous_agent.x,
            advanced_agent.y - previous_agent.y,
        )
        > 1e-3
    )


def _stop_agent_motion(agent: AgentState) -> AgentState:
    return replace(agent, speed_mps=0.0, turn_rate_degps=0.0)
