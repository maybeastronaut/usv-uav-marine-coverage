"""Unified USV path-planner dispatch for baseline and comparative planners."""

from __future__ import annotations

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.grid import GridMap

from .astar_path_planner import build_astar_path_plan
from .astar_smoother_path_planner import build_astar_smoother_path_plan
from .hybrid_astar_path_planner import build_hybrid_astar_path_plan
from .path_types import PathPlan

SUPPORTED_USV_PATH_PLANNERS = {
    "astar_path_planner",
    "astar_smoother_path_planner",
    "hybrid_astar_path_planner",
}


def build_usv_path_plan(
    agent: AgentState,
    *,
    grid_map: GridMap,
    goal_x: float,
    goal_y: float,
    planner_name: str,
    task_id: str | None,
    stats_context: str = "unspecified",
) -> PathPlan:
    """Dispatch one USV path-planning request to the configured planner."""

    if planner_name == "astar_path_planner":
        return build_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name=planner_name,
            task_id=task_id,
            stats_context=stats_context,
        )
    if planner_name == "astar_smoother_path_planner":
        return build_astar_smoother_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name=planner_name,
            task_id=task_id,
            stats_context=stats_context,
        )
    if planner_name == "hybrid_astar_path_planner":
        return build_hybrid_astar_path_plan(
            agent,
            grid_map=grid_map,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name=planner_name,
            task_id=task_id,
            stats_context=stats_context,
        )
    raise ValueError(f"Unsupported USV path planner {planner_name!r}")


def is_supported_usv_path_planner(planner_name: str) -> bool:
    """Return whether the current planner name is a supported USV path planner."""

    return planner_name in SUPPORTED_USV_PATH_PLANNERS
