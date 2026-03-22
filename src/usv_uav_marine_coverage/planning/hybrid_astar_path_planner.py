"""Improved hybrid A* path planning with post-search smoothing for USVs."""

from __future__ import annotations

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.grid import GridMap

from .astar_path_planner import HYBRID_ASTAR_TUNING, build_heading_aware_path_plan
from .path_types import PathPlan
from .traffic_cost import TrafficCostContext


def build_hybrid_astar_path_plan(
    agent: AgentState,
    *,
    grid_map: GridMap,
    obstacle_layout: ObstacleLayout | None = None,
    goal_x: float,
    goal_y: float,
    planner_name: str,
    task_id: str | None,
    stats_context: str = "unspecified",
    traffic_cost_context: TrafficCostContext | None = None,
) -> PathPlan:
    """Build one improved hybrid A* path with richer motion primitives and smoothing."""

    return build_heading_aware_path_plan(
        agent,
        grid_map=grid_map,
        obstacle_layout=obstacle_layout,
        goal_x=goal_x,
        goal_y=goal_y,
        planner_name=planner_name,
        task_id=task_id,
        stats_context=stats_context,
        tuning=HYBRID_ASTAR_TUNING,
        traffic_cost_context=traffic_cost_context,
    )
