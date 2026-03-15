"""Heading-aware risk-weighted A* path planning for USV task execution."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from heapq import heappop, heappush
from math import atan2, cos, degrees, hypot, radians, sin

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    normalize_heading_deg,
)
from usv_uav_marine_coverage.grid import GridMap

from .path_types import PathPlan, PathPlanStatus, Waypoint

HEADING_BIN_COUNT = 16
HEADING_STEP_DEG = 360.0 / HEADING_BIN_COUNT
MOTION_PRIMITIVES = (-1, 0, 1)
RISK_ZONE_COST_MULTIPLIER = 1.8
RISK_AREA_COST_MULTIPLIER = 3.5
TURN_ACTION_COST = 6.0
HEADING_CHANGE_COST = 2.0
PRIMITIVE_STEP_SCALE = 0.9
SEGMENT_SAMPLE_COUNT = 4


@dataclass(frozen=True)
class PlannerMetricsSnapshot:
    """Per-run snapshot of A* invocation counts grouped by usage context."""

    total_calls: int
    planned_calls: int
    blocked_calls: int
    expanded_nodes: int
    max_expanded_nodes: int
    by_context: dict[str, dict[str, int]]


_planner_metrics: dict[str, dict[str, int]] = defaultdict(
    lambda: {
        "calls": 0,
        "planned": 0,
        "blocked": 0,
        "expanded_nodes": 0,
        "max_expanded_nodes": 0,
    }
)


def build_astar_path_plan(
    agent: AgentState,
    *,
    grid_map: GridMap,
    goal_x: float,
    goal_y: float,
    planner_name: str,
    task_id: str | None,
    stats_context: str = "unspecified",
) -> PathPlan:
    """Build one heading-aware risk-weighted A* plan for the current USV task target."""

    start_cell = grid_map.locate_cell(agent.x, agent.y)
    goal_cell = grid_map.locate_cell(goal_x, goal_y)
    if start_cell.has_obstacle or goal_cell.has_obstacle:
        return _blocked_plan(
            agent,
            planner_name=planner_name,
            task_id=task_id,
            goal_x=goal_x,
            goal_y=goal_y,
            stats_context=stats_context,
        )
    if not _point_has_clearance(grid_map, agent.x, agent.y):
        return _blocked_plan(
            agent,
            planner_name=planner_name,
            task_id=task_id,
            goal_x=goal_x,
            goal_y=goal_y,
            stats_context=stats_context,
        )
    if not _point_has_clearance(grid_map, goal_x, goal_y):
        return _blocked_plan(
            agent,
            planner_name=planner_name,
            task_id=task_id,
            goal_x=goal_x,
            goal_y=goal_y,
            stats_context=stats_context,
        )

    start_heading_bin = _heading_to_bin(agent.heading_deg)
    path_states, expanded_nodes = _search_heading_aware_path(
        grid_map,
        start=(start_cell.row, start_cell.col, start_heading_bin),
        goal=(goal_cell.row, goal_cell.col),
        start_pose=(agent.x, agent.y),
    )
    if path_states is None:
        path_states, expanded_nodes = _search_with_start_pose_fallback(
            grid_map,
            start_cell=start_cell,
            goal_cell=goal_cell,
        )
    if path_states is None:
        return _blocked_plan(
            agent,
            planner_name=planner_name,
            task_id=task_id,
            goal_x=goal_x,
            goal_y=goal_y,
            stats_context=stats_context,
        )

    waypoints = [Waypoint(x=agent.x, y=agent.y, row=start_cell.row, col=start_cell.col)]
    for row, col, _ in path_states[1:]:
        cell = grid_map.cell_at(row, col)
        waypoints.append(
            Waypoint(
                x=cell.center_x,
                y=cell.center_y,
                row=row,
                col=col,
            )
        )

    estimated_cost = round(_path_cost(grid_map, path_states), 3)
    plan = PathPlan(
        plan_id=f"{planner_name}-{agent.agent_id}-{goal_cell.row}-{goal_cell.col}",
        planner_name=planner_name,
        agent_id=agent.agent_id,
        task_id=task_id,
        status=PathPlanStatus.PLANNED,
        waypoints=tuple(_deduplicate_waypoints(waypoints)),
        goal_x=goal_x,
        goal_y=goal_y,
        estimated_cost=estimated_cost,
    )
    _record_planner_metric(stats_context, plan.status, expanded_nodes)
    return plan


def reset_planner_metrics() -> None:
    """Clear the in-memory A* invocation counters for one replay step."""

    _planner_metrics.clear()


def snapshot_planner_metrics() -> PlannerMetricsSnapshot:
    """Return the current grouped A* invocation counters."""

    by_context = {key: dict(value) for key, value in sorted(_planner_metrics.items())}
    total_calls = sum(item["calls"] for item in by_context.values())
    planned_calls = sum(item["planned"] for item in by_context.values())
    blocked_calls = sum(item["blocked"] for item in by_context.values())
    expanded_nodes = sum(item["expanded_nodes"] for item in by_context.values())
    max_expanded_nodes = max(
        (item["max_expanded_nodes"] for item in by_context.values()),
        default=0,
    )
    return PlannerMetricsSnapshot(
        total_calls=total_calls,
        planned_calls=planned_calls,
        blocked_calls=blocked_calls,
        expanded_nodes=expanded_nodes,
        max_expanded_nodes=max_expanded_nodes,
        by_context=by_context,
    )


def _search_with_start_pose_fallback(
    grid_map: GridMap,
    *,
    start_cell,
    goal_cell,
) -> tuple[tuple[tuple[int, int, int], ...] | None, int]:
    fallback_pose = (start_cell.center_x, start_cell.center_y)
    fallback_bins = _fallback_start_heading_bins(
        start_x=start_cell.center_x,
        start_y=start_cell.center_y,
        goal_x=goal_cell.center_x,
        goal_y=goal_cell.center_y,
    )
    best_expanded_nodes = 0
    for heading_bin in fallback_bins:
        path_states, expanded_nodes = _search_heading_aware_path(
            grid_map,
            start=(start_cell.row, start_cell.col, heading_bin),
            goal=(goal_cell.row, goal_cell.col),
            start_pose=fallback_pose,
        )
        if path_states is not None:
            return (path_states, expanded_nodes)
        best_expanded_nodes = max(best_expanded_nodes, expanded_nodes)
    return (None, best_expanded_nodes)


def _fallback_start_heading_bins(
    *,
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
) -> tuple[int, ...]:
    goal_heading_deg = normalize_heading_deg(degrees(atan2(goal_y - start_y, goal_x - start_x)))
    goal_heading_bin = _heading_to_bin(goal_heading_deg)
    ranked_bins = sorted(
        range(HEADING_BIN_COUNT),
        key=lambda heading_bin: min(
            (heading_bin - goal_heading_bin) % HEADING_BIN_COUNT,
            (goal_heading_bin - heading_bin) % HEADING_BIN_COUNT,
        ),
    )
    return tuple(ranked_bins)


def _search_heading_aware_path(
    grid_map: GridMap,
    *,
    start: tuple[int, int, int],
    goal: tuple[int, int],
    start_pose: tuple[float, float],
) -> tuple[tuple[tuple[int, int, int], ...] | None, int]:
    open_heap: list[tuple[float, int, tuple[int, int, int]]] = []
    heappush(open_heap, (_heuristic(start[:2], goal, grid_map.cell_size), 0, start))
    came_from: dict[tuple[int, int, int], tuple[int, int, int]] = {}
    cost_so_far: dict[tuple[int, int, int], float] = {start: 0.0}
    visit_order = 0

    while open_heap:
        _, _, current = heappop(open_heap)
        if current[:2] == goal:
            return (_reconstruct_path(came_from, current), visit_order)

        for next_state, transition_cost in _expand_motion_primitives(
            grid_map,
            current=current,
            start=start,
            start_pose=start_pose,
        ):
            next_cost = cost_so_far[current] + transition_cost
            if next_cost >= cost_so_far.get(next_state, float("inf")):
                continue
            cost_so_far[next_state] = next_cost
            came_from[next_state] = current
            visit_order += 1
            priority = next_cost + _heuristic(next_state[:2], goal, grid_map.cell_size)
            heappush(open_heap, (priority, visit_order, next_state))

    return (None, visit_order)


def _expand_motion_primitives(
    grid_map: GridMap,
    *,
    current: tuple[int, int, int],
    start: tuple[int, int, int],
    start_pose: tuple[float, float],
) -> tuple[tuple[tuple[int, int, int], float], ...]:
    row, col, heading_bin = current
    current_x, current_y = _state_pose(
        grid_map,
        state=current,
        start=start,
        start_pose=start_pose,
    )
    step_distance = grid_map.cell_size * PRIMITIVE_STEP_SCALE
    transitions: list[tuple[tuple[int, int, int], float]] = []

    for steering_action in MOTION_PRIMITIVES:
        next_heading_bin = (heading_bin + steering_action) % HEADING_BIN_COUNT
        heading_deg = _bin_to_heading(next_heading_bin)
        next_x = current_x + cos(radians(heading_deg)) * step_distance
        next_y = current_y + sin(radians(heading_deg)) * step_distance
        if not (0.0 <= next_x < grid_map.width and 0.0 <= next_y < grid_map.height):
            continue
        next_cell = grid_map.locate_cell(next_x, next_y)
        if next_cell.has_obstacle:
            continue
        if not _point_has_clearance(grid_map, next_x, next_y):
            continue
        if not _segment_is_collision_free(
            grid_map,
            current_x=current_x,
            current_y=current_y,
            next_x=next_x,
            next_y=next_y,
        ):
            continue
        next_state = (next_cell.row, next_cell.col, next_heading_bin)
        if next_state[:2] == current[:2]:
            continue
        transitions.append(
            (
                next_state,
                _transition_cost(
                    grid_map,
                    current=current,
                    next_state=next_state,
                    steering_action=steering_action,
                ),
            )
        )

    return tuple(transitions)


def _transition_cost(
    grid_map: GridMap,
    *,
    current: tuple[int, int, int],
    next_state: tuple[int, int, int],
    steering_action: int,
) -> float:
    current_cell = grid_map.cell_at(current[0], current[1])
    next_cell = grid_map.cell_at(next_state[0], next_state[1])
    base_cost = hypot(next_state[0] - current[0], next_state[1] - current[1]) * grid_map.cell_size
    if "Risk" in next_cell.zone_name:
        base_cost *= RISK_ZONE_COST_MULTIPLIER
    if next_cell.has_risk_area:
        base_cost *= RISK_AREA_COST_MULTIPLIER
    if steering_action != 0:
        base_cost += TURN_ACTION_COST
    heading_delta_bins = min(
        (next_state[2] - current[2]) % HEADING_BIN_COUNT,
        (current[2] - next_state[2]) % HEADING_BIN_COUNT,
    )
    base_cost += heading_delta_bins * HEADING_CHANGE_COST
    if next_cell.zone_name != current_cell.zone_name and "Risk" in next_cell.zone_name:
        base_cost += grid_map.cell_size * 0.5
    return base_cost


def _state_pose(
    grid_map: GridMap,
    *,
    state: tuple[int, int, int],
    start: tuple[int, int, int],
    start_pose: tuple[float, float],
) -> tuple[float, float]:
    if state == start:
        return start_pose
    cell = grid_map.cell_at(state[0], state[1])
    return (cell.center_x, cell.center_y)


def _segment_is_collision_free(
    grid_map: GridMap,
    *,
    current_x: float,
    current_y: float,
    next_x: float,
    next_y: float,
) -> bool:
    for sample_index in range(1, SEGMENT_SAMPLE_COUNT + 1):
        ratio = sample_index / SEGMENT_SAMPLE_COUNT
        sample_x = current_x + (next_x - current_x) * ratio
        sample_y = current_y + (next_y - current_y) * ratio
        if not (0.0 <= sample_x < grid_map.width and 0.0 <= sample_y < grid_map.height):
            return False
        if not _point_has_clearance(grid_map, sample_x, sample_y):
            return False
    return True


def _point_has_clearance(
    grid_map: GridMap,
    x: float,
    y: float,
    clearance_m: float = USV_COLLISION_CLEARANCE_M,
) -> bool:
    sample_cell = grid_map.locate_cell(x, y)
    if sample_cell.has_obstacle:
        return False
    for cell in grid_map.flat_cells:
        if not cell.has_obstacle:
            continue
        if _distance_to_cell_bounds(x, y, cell) <= clearance_m:
            return False
    return True


def _distance_to_cell_bounds(x: float, y: float, cell) -> float:
    dx = max(cell.x_min - x, 0.0, x - cell.x_max)
    dy = max(cell.y_min - y, 0.0, y - cell.y_max)
    return hypot(dx, dy)


def _heading_to_bin(heading_deg: float) -> int:
    normalized = normalize_heading_deg(heading_deg)
    return int(round((normalized % 360.0) / HEADING_STEP_DEG)) % HEADING_BIN_COUNT


def _bin_to_heading(heading_bin: int) -> float:
    return (heading_bin % HEADING_BIN_COUNT) * HEADING_STEP_DEG


def _heuristic(
    current: tuple[int, int],
    goal: tuple[int, int],
    cell_size: float,
) -> float:
    return hypot(goal[0] - current[0], goal[1] - current[1]) * cell_size


def _reconstruct_path(
    came_from: dict[tuple[int, int, int], tuple[int, int, int]],
    current: tuple[int, int, int],
) -> tuple[tuple[int, int, int], ...]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return tuple(path)


def _path_cost(grid_map: GridMap, path: tuple[tuple[int, int, int], ...]) -> float:
    return sum(
        _transition_cost(
            grid_map,
            current=current,
            next_state=next_state,
            steering_action=_steering_action(current[2], next_state[2]),
        )
        for current, next_state in zip(path, path[1:], strict=False)
    )


def _steering_action(current_heading_bin: int, next_heading_bin: int) -> int:
    delta = (next_heading_bin - current_heading_bin) % HEADING_BIN_COUNT
    if delta == 0:
        return 0
    if delta == 1 or delta == HEADING_BIN_COUNT - 1:
        return 1 if delta == 1 else -1
    return 0


def _deduplicate_waypoints(
    waypoints: list[Waypoint],
) -> tuple[Waypoint, ...]:
    deduplicated: list[Waypoint] = []
    for waypoint in waypoints:
        if deduplicated and (waypoint.row, waypoint.col) == (
            deduplicated[-1].row,
            deduplicated[-1].col,
        ):
            continue
        deduplicated.append(waypoint)
    return tuple(deduplicated)


def _blocked_plan(
    agent: AgentState,
    *,
    planner_name: str,
    task_id: str | None,
    goal_x: float,
    goal_y: float,
    stats_context: str,
) -> PathPlan:
    plan = PathPlan(
        plan_id=f"{planner_name}-{agent.agent_id}-blocked",
        planner_name=planner_name,
        agent_id=agent.agent_id,
        task_id=task_id,
        status=PathPlanStatus.BLOCKED,
        waypoints=(Waypoint(x=agent.x, y=agent.y),),
        goal_x=goal_x,
        goal_y=goal_y,
        estimated_cost=0.0,
    )
    _record_planner_metric(stats_context, plan.status, 0)
    return plan


def _record_planner_metric(
    context: str,
    status: PathPlanStatus,
    expanded_nodes: int,
) -> None:
    metrics = _planner_metrics[context]
    metrics["calls"] += 1
    if status == PathPlanStatus.PLANNED:
        metrics["planned"] += 1
    else:
        metrics["blocked"] += 1
    metrics["expanded_nodes"] += expanded_nodes
    metrics["max_expanded_nodes"] = max(metrics["max_expanded_nodes"], expanded_nodes)
