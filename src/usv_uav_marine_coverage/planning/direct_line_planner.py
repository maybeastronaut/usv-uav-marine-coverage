"""Direct-line planner for minimal task and return paths."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState

from .path_types import PathPlan, PathPlanStatus, Waypoint


def build_direct_line_plan(
    agent: AgentState,
    *,
    goal_x: float,
    goal_y: float,
    planner_name: str,
    task_id: str | None,
) -> PathPlan:
    """Build a two-waypoint direct path from the current pose to one goal."""

    estimated_cost = round(hypot(goal_x - agent.x, goal_y - agent.y), 3)
    return PathPlan(
        plan_id=f"{planner_name}-{agent.agent_id}-{int(goal_x)}-{int(goal_y)}",
        planner_name=planner_name,
        agent_id=agent.agent_id,
        task_id=task_id,
        status=PathPlanStatus.PLANNED,
        waypoints=(
            Waypoint(x=agent.x, y=agent.y),
            Waypoint(x=goal_x, y=goal_y),
        ),
        goal_x=goal_x,
        goal_y=goal_y,
        estimated_cost=estimated_cost,
    )
