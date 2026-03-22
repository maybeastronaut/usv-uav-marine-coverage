"""Close-range collision and wreck keepout guards for execution-time motion."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    shortest_heading_delta_deg,
)
from usv_uav_marine_coverage.environment import ObstacleLayout

from .execution_types import AgentExecutionState, WreckZone

USV_COLLISION_SAMPLE_SPACING_M = 2.0


def _apply_collision_guard_with_optional_wrecks(
    previous_agent: AgentState,
    *,
    advanced_agent: AgentState,
    execution_state: AgentExecutionState,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
) -> tuple[AgentState, AgentExecutionState]:
    if wreck_zones:
        return _apply_usv_collision_guard(
            previous_agent,
            advanced_agent=advanced_agent,
            execution_state=execution_state,
            obstacle_layout=obstacle_layout,
            wreck_zones=wreck_zones,
        )
    return _apply_usv_collision_guard(
        previous_agent,
        advanced_agent=advanced_agent,
        execution_state=execution_state,
        obstacle_layout=obstacle_layout,
    )


def _apply_usv_collision_guard(
    previous_agent: AgentState,
    *,
    advanced_agent: AgentState,
    execution_state: AgentExecutionState,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...] = (),
) -> tuple[AgentState, AgentExecutionState]:
    if previous_agent.kind != "USV":
        return (advanced_agent, execution_state)
    if obstacle_layout is None and not wreck_zones:
        return (advanced_agent, execution_state)
    safe_ratio = _safe_motion_ratio(previous_agent, advanced_agent, obstacle_layout, wreck_zones)
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
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
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
        if _hits_any_obstacle(sample_x, sample_y, obstacle_layout, wreck_zones):
            return last_safe_ratio
        last_safe_ratio = ratio
    return 1.0


def _hits_any_obstacle(
    x: float,
    y: float,
    obstacle_layout: ObstacleLayout | None,
    wreck_zones: tuple[WreckZone, ...],
) -> bool:
    if obstacle_layout is not None:
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
    for wreck in wreck_zones:
        if hypot(x - wreck.x, y - wreck.y) <= wreck.radius:
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
