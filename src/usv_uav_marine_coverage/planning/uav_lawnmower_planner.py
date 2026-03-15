"""Lawnmower search planning for UAV patrol coverage."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState

from .path_types import PathPlan, PathPlanStatus, Waypoint


def build_lawnmower_route(
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    lane_spacing: float,
) -> tuple[tuple[float, float], ...]:
    """Build a simple boustrophedon route inside one rectangular search band."""

    if max_x <= min_x:
        raise ValueError("max_x must be greater than min_x.")
    if max_y <= min_y:
        raise ValueError("max_y must be greater than min_y.")
    if lane_spacing <= 0.0:
        raise ValueError("lane_spacing must be positive.")

    sweep_ys: list[float] = []
    current_y = min_y
    while current_y < max_y:
        sweep_ys.append(round(current_y, 3))
        current_y += lane_spacing
    if not sweep_ys or sweep_ys[-1] != round(max_y, 3):
        sweep_ys.append(round(max_y, 3))

    route: list[tuple[float, float]] = []
    left_to_right = True
    for sweep_y in sweep_ys:
        if left_to_right:
            route.append((min_x, sweep_y))
            route.append((max_x, sweep_y))
        else:
            route.append((max_x, sweep_y))
            route.append((min_x, sweep_y))
        left_to_right = not left_to_right
    return tuple(route)


def build_uav_lawnmower_plan(
    agent: AgentState,
    *,
    patrol_route_id: str,
    patrol_waypoint_index: int,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
) -> PathPlan:
    """Build a one-segment patrol plan to the current UAV lawnmower waypoint."""

    route = patrol_routes[patrol_route_id]
    target_x, target_y = route[patrol_waypoint_index]
    estimated_cost = round(hypot(target_x - agent.x, target_y - agent.y), 3)
    return PathPlan(
        plan_id=f"uav-lawnmower-{agent.agent_id}-{patrol_waypoint_index}",
        planner_name="uav_lawnmower_planner",
        agent_id=agent.agent_id,
        task_id=None,
        status=PathPlanStatus.PLANNED,
        waypoints=(
            Waypoint(x=agent.x, y=agent.y),
            Waypoint(x=target_x, y=target_y),
        ),
        goal_x=target_x,
        goal_y=target_y,
        estimated_cost=estimated_cost,
    )
