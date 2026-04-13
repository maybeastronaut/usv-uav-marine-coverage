"""Waypoint and path-following helpers."""

from __future__ import annotations

from dataclasses import replace
from math import atan2, cos, degrees, hypot, radians, sin
from typing import TYPE_CHECKING

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    TaskMode,
    advance_agent_towards_task,
    advance_agent_with_control,
    assign_agent_task,
    shortest_heading_delta_deg,
)
from usv_uav_marine_coverage.planning.path_types import Waypoint

from .execution_types import (
    AgentExecutionState,
    ExecutionOutcome,
    ExecutionStage,
    WreckZone,
)
from .local_mpc import compute_local_mpc_decision

if TYPE_CHECKING:
    from usv_uav_marine_coverage.environment import ObstacleLayout

USV_PATH_LOOKAHEAD_M = 90.0
UAV_PATH_LOOKAHEAD_M = 28.0
USV_LOCAL_AVOIDANCE_ANGLES_DEG = (20.0, 35.0, 55.0, 75.0)
USV_LOCAL_AVOIDANCE_MAX_DISTANCE_M = 55.0
USV_LOCAL_AVOIDANCE_SAMPLE_SPACING_M = 4.0
USV_LOCAL_AVOIDANCE_CLEARANCE_M = USV_COLLISION_CLEARANCE_M
USV_LOCAL_MPC_NEIGHBOR_TRIGGER_M = USV_COLLISION_CLEARANCE_M * 4.0
USV_LOCAL_MPC_EDGE_TRIGGER_M = USV_COLLISION_CLEARANCE_M * 2.5
USV_GO_TO_TASK_FINAL_APPROACH_MPC_TRIGGER_M = 55.0


def follow_path_step(
    agent: AgentState,
    execution_state: AgentExecutionState,
    *,
    dt_seconds: float,
    obstacle_layout: ObstacleLayout | None = None,
    wreck_zones: tuple[WreckZone, ...] = (),
) -> tuple[AgentState, AgentExecutionState, ExecutionOutcome]:
    """Advance one agent by one step along the currently active plan."""

    execution_state = _clear_local_mpc_metrics(execution_state)
    plan = execution_state.active_plan
    if plan is None or not plan.waypoints:
        if execution_state.stage in {ExecutionStage.ON_TASK, ExecutionStage.ON_RECHARGE}:
            return (
                _stop_and_clear_target(
                    agent,
                    _mode_for_stage(agent.kind, execution_state.stage),
                ),
                execution_state,
                ExecutionOutcome.ADVANCING,
            )
        return (agent, execution_state, ExecutionOutcome.FAILED)

    waypoint_index = _skip_reached_waypoints(agent, execution_state)
    if waypoint_index >= len(plan.waypoints):
        mode = _mode_for_stage(agent.kind, execution_state.stage)
        return _finish_path_step(
            agent,
            mode=mode,
            updated_state=replace(execution_state, current_waypoint_index=waypoint_index),
            outcome=_final_outcome_for_stage(execution_state.stage),
        )

    tracking_target = _build_tracking_target(
        agent,
        execution_state=execution_state,
        waypoint_index=waypoint_index,
    )
    tracking_target = _apply_local_avoidance(
        agent,
        tracking_target=tracking_target,
        obstacle_layout=obstacle_layout,
        wreck_zones=wreck_zones,
    )
    mode = _mode_for_stage(agent.kind, execution_state.stage)
    tracked_agent = assign_agent_task(
        agent,
        mode,
        tracking_target.x,
        tracking_target.y,
    )
    advanced_agent = advance_agent_towards_task(tracked_agent, dt_seconds)
    progressed_state = replace(execution_state, current_waypoint_index=waypoint_index)
    next_waypoint_index = _skip_reached_waypoints(advanced_agent, progressed_state)
    if next_waypoint_index > waypoint_index:
        advanced_agent = assign_agent_task(advanced_agent, mode)

    updated_state = replace(execution_state, current_waypoint_index=next_waypoint_index)
    if next_waypoint_index >= len(plan.waypoints):
        return _finish_path_step(
            advanced_agent,
            mode=mode,
            updated_state=updated_state,
            outcome=_final_outcome_for_stage(execution_state.stage),
        )
    if next_waypoint_index > waypoint_index:
        return (advanced_agent, updated_state, ExecutionOutcome.WAYPOINT_REACHED)
    return (advanced_agent, updated_state, ExecutionOutcome.ADVANCING)


def follow_path_step_with_local_mpc(
    agent: AgentState,
    execution_state: AgentExecutionState,
    *,
    dt_seconds: float,
    obstacle_layout: ObstacleLayout | None = None,
    neighboring_agents: tuple[AgentState, ...] = (),
    wreck_zones: tuple[WreckZone, ...] = (),
    grid_width: float | None = None,
    grid_height: float | None = None,
) -> tuple[AgentState, AgentExecutionState, ExecutionOutcome]:
    """Advance one agent with local MPC enabled for USVs and default tracking for UAVs."""

    if agent.kind != "USV":
        return follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
            obstacle_layout=obstacle_layout,
            wreck_zones=wreck_zones,
        )

    plan = execution_state.active_plan
    if plan is None or not plan.waypoints:
        if execution_state.stage in {ExecutionStage.ON_TASK, ExecutionStage.ON_RECHARGE}:
            return (
                _stop_and_clear_target(
                    agent,
                    _mode_for_stage(agent.kind, execution_state.stage),
                ),
                execution_state,
                ExecutionOutcome.ADVANCING,
            )
        return (agent, execution_state, ExecutionOutcome.FAILED)

    waypoint_index = _skip_reached_waypoints(agent, execution_state)
    if waypoint_index >= len(plan.waypoints):
        mode = _mode_for_stage(agent.kind, execution_state.stage)
        return _finish_path_step(
            agent,
            mode=mode,
            updated_state=replace(execution_state, current_waypoint_index=waypoint_index),
            outcome=_final_outcome_for_stage(execution_state.stage),
        )

    tracking_target = _build_tracking_target(
        agent,
        execution_state=execution_state,
        waypoint_index=waypoint_index,
    )
    tracking_target = _apply_local_avoidance(
        agent,
        tracking_target=tracking_target,
        obstacle_layout=obstacle_layout,
        wreck_zones=wreck_zones,
    )
    if not _should_activate_local_mpc(
        agent,
        execution_state=execution_state,
        tracking_target=tracking_target,
        obstacle_layout=obstacle_layout,
        neighboring_agents=neighboring_agents,
        wreck_zones=wreck_zones,
        grid_width=grid_width,
        grid_height=grid_height,
    ):
        return follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
            obstacle_layout=obstacle_layout,
            wreck_zones=wreck_zones,
        )
    mode = _mode_for_stage(agent.kind, execution_state.stage)
    tracked_agent = assign_agent_task(
        agent,
        mode,
        tracking_target.x,
        tracking_target.y,
    )
    mpc_decision = compute_local_mpc_decision(
        tracked_agent,
        tracking_target=tracking_target,
        dt_seconds=dt_seconds,
        obstacle_layout=obstacle_layout,
        neighboring_agents=neighboring_agents,
        wreck_zones=wreck_zones,
        grid_width=grid_width,
        grid_height=grid_height,
    )
    advanced_agent = advance_agent_with_control(tracked_agent, mpc_decision.command, dt_seconds)
    if (
        hypot(tracking_target.x - advanced_agent.x, tracking_target.y - advanced_agent.y)
        <= advanced_agent.arrival_tolerance_m
    ):
        advanced_agent = assign_agent_task(advanced_agent, mode)

    progressed_state = replace(execution_state, current_waypoint_index=waypoint_index)
    next_waypoint_index = _skip_reached_waypoints(advanced_agent, progressed_state)
    if next_waypoint_index > waypoint_index:
        advanced_agent = assign_agent_task(advanced_agent, mode)

    updated_state = replace(execution_state, current_waypoint_index=next_waypoint_index)
    updated_state = replace(
        updated_state,
        last_local_mpc_predicted_terminal_distance_m=mpc_decision.predicted_terminal_distance_m,
        last_local_mpc_predicted_min_clearance_m=mpc_decision.predicted_min_clearance_m,
        last_local_mpc_candidate_count=mpc_decision.candidate_count,
    )
    if next_waypoint_index >= len(plan.waypoints):
        return _finish_path_step(
            advanced_agent,
            mode=mode,
            updated_state=updated_state,
            outcome=_final_outcome_for_stage(execution_state.stage),
        )
    if next_waypoint_index > waypoint_index:
        return (advanced_agent, updated_state, ExecutionOutcome.WAYPOINT_REACHED)
    return (advanced_agent, updated_state, ExecutionOutcome.ADVANCING)


def _finish_path_step(
    agent: AgentState,
    *,
    mode: TaskMode,
    updated_state: AgentExecutionState,
    outcome: ExecutionOutcome,
) -> tuple[AgentState, AgentExecutionState, ExecutionOutcome]:
    return (_stop_and_clear_target(agent, mode), updated_state, outcome)


def _stop_and_clear_target(agent: AgentState, mode: TaskMode) -> AgentState:
    return replace(
        assign_agent_task(agent, mode),
        speed_mps=0.0,
        turn_rate_degps=0.0,
    )


def _should_activate_local_mpc(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    tracking_target: Waypoint,
    obstacle_layout: ObstacleLayout | None,
    neighboring_agents: tuple[AgentState, ...],
    wreck_zones: tuple[WreckZone, ...],
    grid_width: float | None,
    grid_height: float | None,
) -> bool:
    plan = execution_state.active_plan
    is_far_go_to_task = (
        execution_state.stage == ExecutionStage.GO_TO_TASK
        and execution_state.active_task_id is not None
        and plan is not None
        and hypot(tracking_target.x - plan.goal_x, tracking_target.y - plan.goal_y)
        > USV_GO_TO_TASK_FINAL_APPROACH_MPC_TRIGGER_M
    )
    if is_far_go_to_task:
        if obstacle_layout is not None and not _segment_is_clear(
            agent.x,
            agent.y,
            tracking_target.x,
            tracking_target.y,
            obstacle_layout=obstacle_layout,
            clearance_m=USV_LOCAL_AVOIDANCE_CLEARANCE_M,
            wreck_zones=wreck_zones,
        ):
            return True
    if grid_width is not None and grid_height is not None:
        edge_clearance = min(agent.x, grid_width - agent.x, agent.y, grid_height - agent.y)
        if edge_clearance <= USV_LOCAL_MPC_EDGE_TRIGGER_M:
            return True
    if not is_far_go_to_task and obstacle_layout is not None and not _segment_is_clear(
        agent.x,
        agent.y,
        tracking_target.x,
        tracking_target.y,
        obstacle_layout=obstacle_layout,
        clearance_m=USV_LOCAL_AVOIDANCE_CLEARANCE_M,
        wreck_zones=wreck_zones,
    ):
        return True
    if not is_far_go_to_task and obstacle_layout is not None:
        for feature in obstacle_layout.offshore_features:
            if feature.feature_type != "risk_area":
                continue
            if (
                _distance_to_segment(
                    feature.x,
                    feature.y,
                    (agent.x, agent.y),
                    (tracking_target.x, tracking_target.y),
                )
                <= feature.radius + USV_LOCAL_AVOIDANCE_CLEARANCE_M
            ):
                return True
    for neighboring_agent in neighboring_agents:
        if neighboring_agent.kind != "USV":
            continue
        if hypot(agent.x - neighboring_agent.x, agent.y - neighboring_agent.y) <= (
            USV_LOCAL_MPC_NEIGHBOR_TRIGGER_M
        ):
            return True
    for wreck in wreck_zones:
        if hypot(agent.x - wreck.x, agent.y - wreck.y) <= (
            wreck.radius + USV_LOCAL_MPC_NEIGHBOR_TRIGGER_M
        ):
            return True
    return False


def _clear_local_mpc_metrics(execution_state: AgentExecutionState) -> AgentExecutionState:
    return replace(
        execution_state,
        last_local_mpc_predicted_terminal_distance_m=None,
        last_local_mpc_predicted_min_clearance_m=None,
        last_local_mpc_candidate_count=0,
    )


def _skip_reached_waypoints(agent: AgentState, execution_state: AgentExecutionState) -> int:
    assert execution_state.active_plan is not None
    waypoint_index = execution_state.current_waypoint_index
    while waypoint_index < len(execution_state.active_plan.waypoints):
        waypoint = execution_state.active_plan.waypoints[waypoint_index]
        if not _has_reached_waypoint(agent, waypoint):
            if waypoint_index == 0:
                break
            previous_waypoint = execution_state.active_plan.waypoints[waypoint_index - 1]
            if not _has_passed_waypoint(agent, previous_waypoint, waypoint):
                break
        waypoint_index += 1
    return waypoint_index


def _build_tracking_target(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    waypoint_index: int,
) -> Waypoint:
    assert execution_state.active_plan is not None
    plan = execution_state.active_plan
    waypoint = plan.waypoints[waypoint_index]
    if waypoint_index == 0:
        return waypoint

    lookahead_remaining = _lookahead_distance(agent)
    segment_start = plan.waypoints[waypoint_index - 1]
    projected_ratio = _project_ratio_on_segment(agent, segment_start, waypoint)
    projected_x = segment_start.x + (waypoint.x - segment_start.x) * projected_ratio
    projected_y = segment_start.y + (waypoint.y - segment_start.y) * projected_ratio

    for segment_end_index in range(waypoint_index, len(plan.waypoints)):
        segment_end = plan.waypoints[segment_end_index]
        if segment_end_index == waypoint_index:
            start_x = projected_x
            start_y = projected_y
        else:
            previous_waypoint = plan.waypoints[segment_end_index - 1]
            start_x = previous_waypoint.x
            start_y = previous_waypoint.y

        segment_dx = segment_end.x - start_x
        segment_dy = segment_end.y - start_y
        segment_length = hypot(segment_dx, segment_dy)
        if segment_length <= 1e-9:
            continue
        if lookahead_remaining <= segment_length:
            ratio = lookahead_remaining / segment_length
            return Waypoint(
                x=start_x + segment_dx * ratio,
                y=start_y + segment_dy * ratio,
            )
        lookahead_remaining -= segment_length

    return plan.waypoints[-1]


def _lookahead_distance(agent: AgentState) -> float:
    if agent.kind == "UAV":
        return max(UAV_PATH_LOOKAHEAD_M, agent.speed_mps * 1.5, agent.arrival_tolerance_m * 2.0)
    return max(USV_PATH_LOOKAHEAD_M, agent.speed_mps * 2.0, agent.arrival_tolerance_m * 1.5)


def _apply_local_avoidance(
    agent: AgentState,
    *,
    tracking_target: Waypoint,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...] = (),
) -> Waypoint:
    if agent.kind != "USV" or obstacle_layout is None:
        return tracking_target
    if _segment_is_clear(
        agent.x,
        agent.y,
        tracking_target.x,
        tracking_target.y,
        obstacle_layout=obstacle_layout,
        clearance_m=USV_LOCAL_AVOIDANCE_CLEARANCE_M,
        wreck_zones=wreck_zones,
    ):
        return tracking_target

    desired_dx = tracking_target.x - agent.x
    desired_dy = tracking_target.y - agent.y
    desired_distance = hypot(desired_dx, desired_dy)
    if desired_distance <= 1e-9:
        return tracking_target
    candidate_distance = min(
        desired_distance,
        max(agent.arrival_tolerance_m * 2.0, USV_LOCAL_AVOIDANCE_MAX_DISTANCE_M),
    )
    base_heading_deg = degrees(atan2(desired_dy, desired_dx))

    for distance_scale in (1.0, 0.75, 0.5):
        scaled_distance = max(
            agent.arrival_tolerance_m * 1.5,
            candidate_distance * distance_scale,
        )
        for offset_deg in USV_LOCAL_AVOIDANCE_ANGLES_DEG:
            err_pos = abs(shortest_heading_delta_deg(agent.heading_deg, base_heading_deg + offset_deg))
            err_neg = abs(shortest_heading_delta_deg(agent.heading_deg, base_heading_deg - offset_deg))
            ordered_offsets = (offset_deg, -offset_deg) if err_pos <= err_neg else (-offset_deg, offset_deg)
            for signed_offset_deg in ordered_offsets:
                candidate_heading_deg = base_heading_deg + signed_offset_deg
                candidate_target = Waypoint(
                    x=agent.x + cos(radians(candidate_heading_deg)) * scaled_distance,
                    y=agent.y + sin(radians(candidate_heading_deg)) * scaled_distance,
                )
                if _segment_is_clear(
                    agent.x,
                    agent.y,
                    candidate_target.x,
                    candidate_target.y,
                    obstacle_layout=obstacle_layout,
                    clearance_m=USV_LOCAL_AVOIDANCE_CLEARANCE_M,
                    wreck_zones=wreck_zones,
                ):
                    return candidate_target
    return tracking_target


def _has_reached_waypoint(agent: AgentState, waypoint: Waypoint) -> bool:
    return hypot(waypoint.x - agent.x, waypoint.y - agent.y) <= agent.arrival_tolerance_m


def _segment_is_clear(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    *,
    obstacle_layout: ObstacleLayout,
    clearance_m: float,
    wreck_zones: tuple[WreckZone, ...] = (),
) -> bool:
    travel_distance = hypot(end_x - start_x, end_y - start_y)
    if travel_distance <= 1e-9:
        return not _sample_hits_hazard(
            start_x,
            start_y,
            obstacle_layout=obstacle_layout,
            clearance_m=clearance_m,
        )
    sample_count = max(2, int(travel_distance / USV_LOCAL_AVOIDANCE_SAMPLE_SPACING_M) + 1)
    for sample_index in range(1, sample_count + 1):
        ratio = sample_index / sample_count
        sample_x = start_x + (end_x - start_x) * ratio
        sample_y = start_y + (end_y - start_y) * ratio
        if _sample_hits_hazard(
            sample_x,
            sample_y,
            obstacle_layout=obstacle_layout,
            clearance_m=clearance_m,
            wreck_zones=wreck_zones,
        ):
            return False
    return True


def _sample_hits_hazard(
    x: float,
    y: float,
    *,
    obstacle_layout: ObstacleLayout,
    clearance_m: float,
    wreck_zones: tuple[WreckZone, ...] = (),
) -> bool:
    for obstacle in obstacle_layout.risk_zone_obstacles:
        if _point_in_polygon(x, y, obstacle.points):
            return True
        if _distance_to_polygon_edges(x, y, obstacle.points) <= clearance_m:
            return True
    for feature in obstacle_layout.offshore_features:
        if feature.feature_type != "islet":
            continue
        if hypot(x - feature.x, y - feature.y) <= feature.radius + clearance_m:
            return True
    for wreck in wreck_zones:
        if hypot(x - wreck.x, y - wreck.y) <= wreck.radius + clearance_m:
            return True
    return False


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


def _point_in_polygon(
    x: float,
    y: float,
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    inside = False
    previous_index = len(polygon) - 1
    for index, (current_x, current_y) in enumerate(polygon):
        previous_x, previous_y = polygon[previous_index]
        if (current_y > y) != (previous_y > y):
            x_intersection = (previous_x - current_x) * (y - current_y) / (
                (previous_y - current_y) or 1e-12
            ) + current_x
            if x < x_intersection:
                inside = not inside
        previous_index = index
    return inside


def _project_ratio_on_segment(agent: AgentState, start: Waypoint, end: Waypoint) -> float:
    segment_dx = end.x - start.x
    segment_dy = end.y - start.y
    segment_length_sq = segment_dx**2 + segment_dy**2
    if segment_length_sq <= 1e-9:
        return 1.0
    projection = (
        (agent.x - start.x) * segment_dx + (agent.y - start.y) * segment_dy
    ) / segment_length_sq
    return min(max(projection, 0.0), 1.0)


def _has_passed_waypoint(agent: AgentState, start: Waypoint, end: Waypoint) -> bool:
    segment_dx = end.x - start.x
    segment_dy = end.y - start.y
    segment_length_sq = segment_dx**2 + segment_dy**2
    if segment_length_sq <= 1e-9:
        return True
    projection = (
        (agent.x - start.x) * segment_dx + (agent.y - start.y) * segment_dy
    ) / segment_length_sq
    return projection >= 1.0


def _mode_for_stage(agent_kind: str, stage: ExecutionStage) -> TaskMode:
    if stage in {
        ExecutionStage.ON_TASK,
        ExecutionStage.GO_TO_TASK,
        ExecutionStage.GO_TO_RENDEZVOUS,
        ExecutionStage.RECOVERY,
        ExecutionStage.YIELD,
    }:
        return TaskMode.CONFIRM if agent_kind == "USV" else TaskMode.INVESTIGATE
    if stage == ExecutionStage.ON_RECHARGE:
        return TaskMode.IDLE
    if agent_kind == "UAV":
        return TaskMode.INVESTIGATE
    return TaskMode.PATROL


def _final_outcome_for_stage(stage: ExecutionStage) -> ExecutionOutcome:
    if stage in {ExecutionStage.GO_TO_TASK, ExecutionStage.GO_TO_RENDEZVOUS}:
        return ExecutionOutcome.TASK_SITE_REACHED
    if stage == ExecutionStage.RETURN_TO_PATROL:
        return ExecutionOutcome.PATROL_REJOINED
    if stage == ExecutionStage.PATROL:
        return ExecutionOutcome.WAYPOINT_REACHED
    if stage in {ExecutionStage.RECOVERY, ExecutionStage.YIELD}:
        return ExecutionOutcome.ADVANCING
    if stage in {ExecutionStage.ON_TASK, ExecutionStage.ON_RECHARGE}:
        return ExecutionOutcome.TASK_FINISHED
    return ExecutionOutcome.FAILED
