"""Fixed patrol-route planning helpers."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState

from .path_types import PathPlan, PathPlanStatus, Waypoint


def build_fixed_patrol_plan(
    agent: AgentState,
    *,
    patrol_route_id: str,
    patrol_waypoint_index: int,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
) -> PathPlan:
    """Build a minimal patrol plan to the current patrol waypoint."""

    route = patrol_routes[patrol_route_id]
    target_x, target_y = route[patrol_waypoint_index]
    estimated_cost = round(hypot(target_x - agent.x, target_y - agent.y), 3)
    return PathPlan(
        plan_id=f"patrol-{agent.agent_id}-{patrol_waypoint_index}",
        planner_name="fixed_patrol_planner",
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
