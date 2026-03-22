"""Traffic-aware path-cost helpers shared by USV planners."""

from __future__ import annotations

from dataclasses import dataclass

from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.grid import GridMap

from .path_types import PathPlan, PathPlanStatus

TRAFFIC_CELL_COST_PER_PLAN = 24.0
TRAFFIC_CORRIDOR_ENTRY_COST_PER_PLAN = 90.0


@dataclass(frozen=True)
class TrafficCostContext:
    """One lightweight congestion snapshot for a single planning request."""

    cell_penalties: dict[tuple[int, int], float]
    corridor_penalties: dict[str, float]


def build_traffic_cost_context(
    *,
    agent_id: str,
    reference_plans: tuple[PathPlan, ...],
    grid_map: GridMap,
    obstacle_layout: ObstacleLayout | None,
) -> TrafficCostContext | None:
    """Build one traffic-aware cost context from other active USV plans."""

    cell_penalties: dict[tuple[int, int], float] = {}
    corridor_penalties: dict[str, float] = {}

    for plan in reference_plans:
        if (
            plan.agent_id == agent_id
            or plan.status != PathPlanStatus.PLANNED
            or len(plan.waypoints) < 2
        ):
            continue
        plan_cells: set[tuple[int, int]] = set()
        touched_corridors: set[str] = set()
        for waypoint in plan.waypoints[1:]:
            cell = (
                grid_map.locate_cell(waypoint.x, waypoint.y)
                if waypoint.row is None or waypoint.col is None
                else grid_map.cell_at(waypoint.row, waypoint.col)
            )
            plan_cells.add((cell.row, cell.col))
            corridor_name = _corridor_name_for_point(
                waypoint.x,
                waypoint.y,
                obstacle_layout=obstacle_layout,
            )
            if corridor_name is not None:
                touched_corridors.add(corridor_name)

        for cell_key in plan_cells:
            cell_penalties[cell_key] = (
                cell_penalties.get(cell_key, 0.0) + TRAFFIC_CELL_COST_PER_PLAN
            )
        for corridor_name in touched_corridors:
            corridor_penalties[corridor_name] = (
                corridor_penalties.get(corridor_name, 0.0) + TRAFFIC_CORRIDOR_ENTRY_COST_PER_PLAN
            )

    if not cell_penalties and not corridor_penalties:
        return None
    return TrafficCostContext(
        cell_penalties=cell_penalties,
        corridor_penalties=corridor_penalties,
    )


def traffic_transition_cost(
    *,
    grid_map: GridMap,
    current: tuple[int, int, int],
    next_state: tuple[int, int, int],
    obstacle_layout: ObstacleLayout | None,
    traffic_cost_context: TrafficCostContext | None,
) -> float:
    """Return one additive traffic-aware penalty for the next transition."""

    if traffic_cost_context is None:
        return 0.0

    penalty = traffic_cost_context.cell_penalties.get(next_state[:2], 0.0)
    next_corridor = _corridor_name_for_state(
        grid_map,
        next_state,
        obstacle_layout=obstacle_layout,
    )
    if next_corridor is None:
        return penalty

    current_corridor = _corridor_name_for_state(
        grid_map,
        current,
        obstacle_layout=obstacle_layout,
    )
    if next_corridor != current_corridor:
        penalty += traffic_cost_context.corridor_penalties.get(next_corridor, 0.0)
    return penalty


def _corridor_name_for_state(
    grid_map: GridMap,
    state: tuple[int, int, int],
    *,
    obstacle_layout: ObstacleLayout | None,
) -> str | None:
    cell = grid_map.cell_at(state[0], state[1])
    return _corridor_name_for_point(
        cell.center_x,
        cell.center_y,
        obstacle_layout=obstacle_layout,
    )


def _corridor_name_for_point(
    x: float,
    y: float,
    *,
    obstacle_layout: ObstacleLayout | None,
) -> str | None:
    if obstacle_layout is None:
        return None
    for corridor in obstacle_layout.traversable_corridors:
        if _distance_to_corridor(x, y, corridor.control_points) <= corridor.width / 2.0:
            return corridor.name
    return None


def _distance_to_corridor(
    x: float,
    y: float,
    control_points: tuple[tuple[float, float], ...],
) -> float:
    best_distance = float("inf")
    for start, end in zip(control_points, control_points[1:], strict=False):
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
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    segment_length_sq = delta_x * delta_x + delta_y * delta_y
    if segment_length_sq <= 1e-9:
        return ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5
    projection = ((x - start_x) * delta_x + (y - start_y) * delta_y) / segment_length_sq
    clamped_projection = min(max(projection, 0.0), 1.0)
    closest_x = start_x + delta_x * clamped_projection
    closest_y = start_y + delta_y * clamped_projection
    return ((x - closest_x) ** 2 + (y - closest_y) ** 2) ** 0.5
