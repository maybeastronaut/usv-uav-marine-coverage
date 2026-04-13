"""Lightweight local MPC-style safety controller for execution-time avoidance."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, radians, sin
from typing import TYPE_CHECKING

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    ControlCommand,
    TrackingReference,
    advance_agent_with_control,
    compute_control_command,
    shortest_heading_delta_deg,
)
from usv_uav_marine_coverage.execution.execution_types import WreckZone
from usv_uav_marine_coverage.planning.path_types import Waypoint

if TYPE_CHECKING:
    from usv_uav_marine_coverage.environment import ObstacleLayout

LOCAL_MPC_HORIZON_STEPS = 4
LOCAL_MPC_HEADING_OFFSETS_DEG = (0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0)
LOCAL_MPC_SPEED_SCALES = (1.0, 0.8, 0.6, 0.35)
LOCAL_MPC_NEIGHBOR_CLEARANCE_M = USV_COLLISION_CLEARANCE_M * 1.4
LOCAL_MPC_HAZARD_CLEARANCE_M = USV_COLLISION_CLEARANCE_M * 1.1
LOCAL_MPC_TRACKING_WEIGHT = 1.0
LOCAL_MPC_PROGRESS_WEIGHT = 1.35
LOCAL_MPC_CLEARANCE_WEIGHT = 18.0
LOCAL_MPC_HEADING_WEIGHT = 0.12
LOCAL_MPC_SPEED_WEIGHT = 1.2
LOCAL_MPC_MIN_PROGRESS_RATIO = 0.18
LOCAL_MPC_LOW_PROGRESS_PENALTY = 40.0
LOCAL_MPC_LATERAL_DRIFT_PENALTY = 12.0
LOCAL_MPC_STOPPING_SPEED_MPS = 0.0
# Keep the historic constant for compatibility with other execution modules.
LOCAL_MPC_STOP_PROGRESS_MARGIN_M = 0.25
LOCAL_MPC_STOP_CLEARANCE_RATIO = 0.85
LOCAL_MPC_EDGE_CLEARANCE_M = USV_COLLISION_CLEARANCE_M * 2.0


@dataclass(frozen=True)
class LocalMpcDecision:
    """The selected control command and diagnostics for one local MPC update."""

    command: ControlCommand
    predicted_min_clearance_m: float
    predicted_terminal_distance_m: float
    candidate_count: int
    predicted_lateral_drift_m: float


def compute_local_mpc_decision(
    agent: AgentState,
    *,
    tracking_target: Waypoint,
    dt_seconds: float,
    obstacle_layout: ObstacleLayout | None,
    neighboring_agents: tuple[AgentState, ...] = (),
    wreck_zones: tuple[WreckZone, ...] = (),
    grid_width: float | None = None,
    grid_height: float | None = None,
) -> LocalMpcDecision:
    """Return one short-horizon control command that tracks the path while avoiding hazards."""

    reference = TrackingReference(
        target_x=tracking_target.x,
        target_y=tracking_target.y,
        desired_speed_mps=min(agent.cruise_speed_mps, agent.max_speed_mps),
    )
    nominal_command = compute_control_command(agent, reference)
    initial_distance = hypot(tracking_target.x - agent.x, tracking_target.y - agent.y)

    best_decision: tuple[float, ControlCommand, float, float, float] | None = None
    best_moving_decision: tuple[float, ControlCommand, float, float, float] | None = None
    candidate_count = 0
    for command in _candidate_commands(agent, nominal_command):
        candidate_count += 1
        rollout = _simulate_rollout(
            agent,
            command=command,
            tracking_target=tracking_target,
            dt_seconds=dt_seconds,
            obstacle_layout=obstacle_layout,
            neighboring_agents=neighboring_agents,
            wreck_zones=wreck_zones,
            grid_width=grid_width,
            grid_height=grid_height,
        )
        if rollout is None:
            continue
        terminal_distance, min_clearance_m, lateral_drift_m = rollout
        progress_m = max(0.0, initial_distance - terminal_distance)
        heading_error = abs(
            shortest_heading_delta_deg(
                agent.heading_deg,
                command.target_heading_deg,
            )
        )
        speed_delta = abs(nominal_command.target_speed_mps - command.target_speed_mps)
        low_progress_penalty = 0.0
        if progress_m < initial_distance * LOCAL_MPC_MIN_PROGRESS_RATIO:
            low_progress_penalty = LOCAL_MPC_LOW_PROGRESS_PENALTY
        score = (
            terminal_distance * LOCAL_MPC_TRACKING_WEIGHT
            - progress_m * LOCAL_MPC_PROGRESS_WEIGHT
            - min_clearance_m * LOCAL_MPC_CLEARANCE_WEIGHT
            + heading_error * LOCAL_MPC_HEADING_WEIGHT
            + speed_delta * LOCAL_MPC_SPEED_WEIGHT
            + low_progress_penalty
            + lateral_drift_m * LOCAL_MPC_LATERAL_DRIFT_PENALTY
        )
        if best_decision is None or score < best_decision[0]:
            best_decision = (
                score,
                command,
                min_clearance_m,
                terminal_distance,
                lateral_drift_m,
            )
        if command.target_speed_mps > 0.0 and (
            best_moving_decision is None or score < best_moving_decision[0]
        ):
            best_moving_decision = (
                score,
                command,
                min_clearance_m,
                terminal_distance,
                lateral_drift_m,
            )

    if best_decision is None:
        fallback_command = ControlCommand(
            target_speed_mps=LOCAL_MPC_STOPPING_SPEED_MPS,
            target_heading_deg=nominal_command.target_heading_deg,
        )
        return LocalMpcDecision(
            command=fallback_command,
            predicted_min_clearance_m=0.0,
            predicted_terminal_distance_m=initial_distance,
            candidate_count=candidate_count,
            predicted_lateral_drift_m=0.0,
        )

    best_decision = _prefer_progressing_command_over_stop(
        best_decision,
        best_moving_decision=best_moving_decision,
    )
    _, command, min_clearance_m, terminal_distance, lateral_drift_m = best_decision
    return LocalMpcDecision(
        command=command,
        predicted_min_clearance_m=min_clearance_m,
        predicted_terminal_distance_m=terminal_distance,
        candidate_count=candidate_count,
        predicted_lateral_drift_m=lateral_drift_m,
    )


def _prefer_progressing_command_over_stop(
    best_decision: tuple[float, ControlCommand, float, float, float],
    *,
    best_moving_decision: tuple[float, ControlCommand, float, float, float] | None,
) -> tuple[float, ControlCommand, float, float, float]:
    score, command, min_clearance_m, terminal_distance, lateral_drift_m = best_decision
    if command.target_speed_mps > 0.0 or best_moving_decision is None:
        return best_decision

    (
        moving_score,
        moving_command,
        moving_clearance_m,
        moving_terminal_distance_m,
        moving_lateral_drift_m,
    ) = best_moving_decision
    if moving_terminal_distance_m >= terminal_distance - LOCAL_MPC_STOP_PROGRESS_MARGIN_M:
        return best_decision
    if moving_clearance_m < min_clearance_m * LOCAL_MPC_STOP_CLEARANCE_RATIO:
        return best_decision
    return (
        moving_score,
        moving_command,
        moving_clearance_m,
        moving_terminal_distance_m,
        moving_lateral_drift_m,
    )


def _candidate_commands(
    agent: AgentState,
    nominal_command: ControlCommand,
) -> tuple[ControlCommand, ...]:
    candidates: list[ControlCommand] = []
    for speed_scale in LOCAL_MPC_SPEED_SCALES:
        target_speed = min(
            agent.max_speed_mps,
            nominal_command.target_speed_mps * speed_scale,
        )
        for heading_offset_deg in LOCAL_MPC_HEADING_OFFSETS_DEG:
            candidates.append(
                ControlCommand(
                    target_speed_mps=target_speed,
                    target_heading_deg=nominal_command.target_heading_deg + heading_offset_deg,
                )
            )
    candidates.append(
        ControlCommand(
            target_speed_mps=LOCAL_MPC_STOPPING_SPEED_MPS,
            target_heading_deg=nominal_command.target_heading_deg,
        )
    )
    return tuple(candidates)


def _simulate_rollout(
    agent: AgentState,
    *,
    command: ControlCommand,
    tracking_target: Waypoint,
    dt_seconds: float,
    obstacle_layout: ObstacleLayout | None,
    neighboring_agents: tuple[AgentState, ...],
    wreck_zones: tuple[WreckZone, ...],
    grid_width: float | None,
    grid_height: float | None,
) -> tuple[float, float, float] | None:
    simulated_agent = agent
    predicted_neighbors = neighboring_agents
    min_clearance_m = float("inf")
    for _ in range(LOCAL_MPC_HORIZON_STEPS):
        simulated_agent = advance_agent_with_control(simulated_agent, command, dt_seconds)
        predicted_neighbors = tuple(
            _predict_neighbor_step(neighbor, dt_seconds) for neighbor in predicted_neighbors
        )
        clearance_m = _minimum_clearance(
            simulated_agent,
            obstacle_layout=obstacle_layout,
            neighboring_agents=predicted_neighbors,
            wreck_zones=wreck_zones,
            grid_width=grid_width,
            grid_height=grid_height,
        )
        min_clearance_m = min(min_clearance_m, clearance_m)
        if clearance_m <= 0.0:
            return None
    terminal_distance = hypot(
        tracking_target.x - simulated_agent.x,
        tracking_target.y - simulated_agent.y,
    )
    lateral_drift_m = _lateral_drift_from_tracking_line(
        agent,
        tracking_target=tracking_target,
        simulated_agent=simulated_agent,
    )
    return (terminal_distance, min_clearance_m, lateral_drift_m)


def _lateral_drift_from_tracking_line(
    agent: AgentState,
    *,
    tracking_target: Waypoint,
    simulated_agent: AgentState,
) -> float:
    line_dx = tracking_target.x - agent.x
    line_dy = tracking_target.y - agent.y
    line_length = hypot(line_dx, line_dy)
    if line_length <= 1e-9:
        return 0.0
    point_dx = simulated_agent.x - agent.x
    point_dy = simulated_agent.y - agent.y
    area_twice = abs(line_dx * point_dy - line_dy * point_dx)
    return area_twice / line_length


def _predict_neighbor_step(agent: AgentState, dt_seconds: float) -> AgentState:
    travel_distance = agent.speed_mps * dt_seconds
    return AgentState(
        agent_id=agent.agent_id,
        kind=agent.kind,
        x=agent.x + cos(radians(agent.heading_deg)) * travel_distance,
        y=agent.y + sin(radians(agent.heading_deg)) * travel_distance,
        heading_deg=agent.heading_deg,
        speed_mps=agent.speed_mps,
        max_speed_mps=agent.max_speed_mps,
        detection_radius=agent.detection_radius,
        coverage_radius=agent.coverage_radius,
        task=agent.task,
        energy_capacity=agent.energy_capacity,
        energy_level=agent.energy_level,
        energy_burn_per_second=agent.energy_burn_per_second,
        energy_burn_per_meter=agent.energy_burn_per_meter,
        energy_burn_per_turn_deg=agent.energy_burn_per_turn_deg,
        recharge_rate_per_second=agent.recharge_rate_per_second,
        minimum_reserve_ratio=agent.minimum_reserve_ratio,
        turn_rate_degps=agent.turn_rate_degps,
        max_acceleration_mps2=agent.max_acceleration_mps2,
        max_deceleration_mps2=agent.max_deceleration_mps2,
        max_turn_rate_degps=agent.max_turn_rate_degps,
        cruise_speed_mps=agent.cruise_speed_mps,
        arrival_tolerance_m=agent.arrival_tolerance_m,
        platform_profile=agent.platform_profile,
    )


def _minimum_clearance(
    agent: AgentState,
    *,
    obstacle_layout: ObstacleLayout | None,
    neighboring_agents: tuple[AgentState, ...],
    wreck_zones: tuple[WreckZone, ...],
    grid_width: float | None,
    grid_height: float | None,
) -> float:
    best_clearance_m = float("inf")
    if grid_width is not None and grid_height is not None:
        edge_clearance_m = min(
            agent.x,
            grid_width - agent.x,
            agent.y,
            grid_height - agent.y,
        )
        if edge_clearance_m <= 0.0:
            return 0.0
        # Treat map edges as a soft clearance term. The activation gate already
        # decides when edge-aware local MPC should engage; here we only need to
        # reject trajectories that actually leave the map, not trajectories that
        # legitimately operate near a shoreline corridor in the scaled RL world.
        best_clearance_m = min(best_clearance_m, edge_clearance_m)
    if obstacle_layout is not None:
        for obstacle in obstacle_layout.risk_zone_obstacles:
            if _point_in_polygon(agent.x, agent.y, obstacle.points):
                return 0.0
            best_clearance_m = min(
                best_clearance_m,
                _distance_to_polygon_edges(agent.x, agent.y, obstacle.points)
                - LOCAL_MPC_HAZARD_CLEARANCE_M,
            )
        for feature in obstacle_layout.offshore_features:
            best_clearance_m = min(
                best_clearance_m,
                hypot(agent.x - feature.x, agent.y - feature.y)
                - feature.radius
                - LOCAL_MPC_HAZARD_CLEARANCE_M,
            )
    for neighbor in neighboring_agents:
        if neighbor.kind != "USV":
            continue
        best_clearance_m = min(
            best_clearance_m,
            hypot(agent.x - neighbor.x, agent.y - neighbor.y) - LOCAL_MPC_NEIGHBOR_CLEARANCE_M,
        )
    for wreck in wreck_zones:
        best_clearance_m = min(
            best_clearance_m,
            hypot(agent.x - wreck.x, agent.y - wreck.y) - wreck.radius,
        )
    return best_clearance_m


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
