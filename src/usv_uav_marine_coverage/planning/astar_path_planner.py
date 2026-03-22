"""Heading-aware grid path planning for USV task execution."""

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
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.grid import GridMap

from .path_types import PathPlan, PathPlanStatus, Waypoint
from .traffic_cost import TrafficCostContext, traffic_transition_cost

DEFAULT_HEADING_BIN_COUNT = 16


@dataclass(frozen=True)
class PlannerTuning:
    """Planner tuning knobs shared by the baseline and hybrid variants."""

    heading_bin_count: int
    motion_primitives: tuple[int, ...]
    risk_zone_cost_multiplier: float
    risk_area_cost_multiplier: float
    turn_action_cost: float
    heading_change_cost: float
    primitive_step_scale: float
    segment_sample_count: int
    smoothing_enabled: bool = False
    smoothing_cost_tolerance: float = 0.0
    max_smoothed_segment_length_m: float | None = None

    @property
    def heading_step_deg(self) -> float:
        return 360.0 / self.heading_bin_count


ASTAR_TUNING = PlannerTuning(
    heading_bin_count=DEFAULT_HEADING_BIN_COUNT,
    motion_primitives=(-1, 0, 1),
    risk_zone_cost_multiplier=1.8,
    risk_area_cost_multiplier=3.5,
    turn_action_cost=6.0,
    heading_change_cost=2.0,
    primitive_step_scale=0.9,
    segment_sample_count=4,
)

ASTAR_SMOOTHER_TUNING = PlannerTuning(
    heading_bin_count=DEFAULT_HEADING_BIN_COUNT,
    motion_primitives=(-1, 0, 1),
    risk_zone_cost_multiplier=1.8,
    risk_area_cost_multiplier=3.5,
    turn_action_cost=6.0,
    heading_change_cost=2.0,
    primitive_step_scale=0.9,
    segment_sample_count=4,
    smoothing_enabled=True,
    smoothing_cost_tolerance=8.0,
    max_smoothed_segment_length_m=55.0,
)

HYBRID_ASTAR_TUNING = PlannerTuning(
    heading_bin_count=24,
    motion_primitives=(-2, -1, 0, 1, 2),
    risk_zone_cost_multiplier=1.7,
    risk_area_cost_multiplier=3.2,
    turn_action_cost=4.0,
    heading_change_cost=1.4,
    primitive_step_scale=1.0,
    segment_sample_count=6,
    smoothing_enabled=True,
    smoothing_cost_tolerance=12.0,
    max_smoothed_segment_length_m=70.0,
)


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
    obstacle_layout: ObstacleLayout | None = None,
    goal_x: float,
    goal_y: float,
    planner_name: str,
    task_id: str | None,
    stats_context: str = "unspecified",
    traffic_cost_context: TrafficCostContext | None = None,
) -> PathPlan:
    """Build one baseline heading-aware A* plan for the current USV task target."""

    return build_heading_aware_path_plan(
        agent,
        grid_map=grid_map,
        goal_x=goal_x,
        goal_y=goal_y,
        planner_name=planner_name,
        task_id=task_id,
        stats_context=stats_context,
        tuning=ASTAR_TUNING,
        obstacle_layout=obstacle_layout,
        traffic_cost_context=traffic_cost_context,
    )


def build_heading_aware_path_plan(
    agent: AgentState,
    *,
    grid_map: GridMap,
    obstacle_layout: ObstacleLayout | None = None,
    goal_x: float,
    goal_y: float,
    planner_name: str,
    task_id: str | None,
    stats_context: str = "unspecified",
    tuning: PlannerTuning = ASTAR_TUNING,
    traffic_cost_context: TrafficCostContext | None = None,
) -> PathPlan:
    """Build one configured heading-aware grid path plan."""

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
    if not _point_has_clearance(grid_map, goal_x, goal_y):
        return _blocked_plan(
            agent,
            planner_name=planner_name,
            task_id=task_id,
            goal_x=goal_x,
            goal_y=goal_y,
            stats_context=stats_context,
        )

    start_heading_bin = _heading_to_bin(agent.heading_deg, tuning=tuning)
    path_states, expanded_nodes = _search_heading_aware_path(
        grid_map,
        start=(start_cell.row, start_cell.col, start_heading_bin),
        goal=(goal_cell.row, goal_cell.col),
        start_pose=(agent.x, agent.y),
        tuning=tuning,
        obstacle_layout=obstacle_layout,
        traffic_cost_context=traffic_cost_context,
    )
    if path_states is None:
        path_states, expanded_nodes = _search_with_start_pose_fallback(
            grid_map,
            start_cell=start_cell,
            goal_cell=goal_cell,
            tuning=tuning,
            obstacle_layout=obstacle_layout,
            traffic_cost_context=traffic_cost_context,
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

    waypoints = list(_deduplicate_waypoints(waypoints))
    if tuning.smoothing_enabled:
        waypoints = list(_smooth_waypoints(grid_map, waypoints, tuning=tuning))
        estimated_cost = round(
            _waypoint_path_cost(
                grid_map,
                tuple(waypoints),
                start_heading_deg=agent.heading_deg,
                tuning=tuning,
            ),
            3,
        )
    else:
        estimated_cost = round(_path_cost(grid_map, path_states, tuning=tuning), 3)
    plan = PathPlan(
        plan_id=f"{planner_name}-{agent.agent_id}-{goal_cell.row}-{goal_cell.col}",
        planner_name=planner_name,
        agent_id=agent.agent_id,
        task_id=task_id,
        status=PathPlanStatus.PLANNED,
        waypoints=tuple(waypoints),
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
    tuning: PlannerTuning,
    obstacle_layout: ObstacleLayout | None = None,
    traffic_cost_context: TrafficCostContext | None = None,
) -> tuple[tuple[tuple[int, int, int], ...] | None, int]:
    fallback_pose = (start_cell.center_x, start_cell.center_y)
    fallback_bins = _fallback_start_heading_bins(
        start_x=start_cell.center_x,
        start_y=start_cell.center_y,
        goal_x=goal_cell.center_x,
        goal_y=goal_cell.center_y,
        tuning=tuning,
    )
    best_expanded_nodes = 0
    for heading_bin in fallback_bins:
        path_states, expanded_nodes = _search_heading_aware_path(
            grid_map,
            start=(start_cell.row, start_cell.col, heading_bin),
            goal=(goal_cell.row, goal_cell.col),
            start_pose=fallback_pose,
            tuning=tuning,
            obstacle_layout=obstacle_layout,
            traffic_cost_context=traffic_cost_context,
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
    tuning: PlannerTuning,
) -> tuple[int, ...]:
    goal_heading_deg = normalize_heading_deg(degrees(atan2(goal_y - start_y, goal_x - start_x)))
    goal_heading_bin = _heading_to_bin(goal_heading_deg, tuning=tuning)
    ranked_bins = sorted(
        range(tuning.heading_bin_count),
        key=lambda heading_bin: min(
            (heading_bin - goal_heading_bin) % tuning.heading_bin_count,
            (goal_heading_bin - heading_bin) % tuning.heading_bin_count,
        ),
    )
    return tuple(ranked_bins)


def _search_heading_aware_path(
    grid_map: GridMap,
    *,
    start: tuple[int, int, int],
    goal: tuple[int, int],
    start_pose: tuple[float, float],
    tuning: PlannerTuning,
    obstacle_layout: ObstacleLayout | None = None,
    traffic_cost_context: TrafficCostContext | None = None,
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
            tuning=tuning,
            obstacle_layout=obstacle_layout,
            traffic_cost_context=traffic_cost_context,
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
    tuning: PlannerTuning,
    obstacle_layout: ObstacleLayout | None = None,
    traffic_cost_context: TrafficCostContext | None = None,
) -> tuple[tuple[tuple[int, int, int], float], ...]:
    row, col, heading_bin = current
    current_x, current_y = _state_pose(
        grid_map,
        state=current,
        start=start,
        start_pose=start_pose,
    )
    step_distance = grid_map.cell_size * tuning.primitive_step_scale
    transitions: list[tuple[tuple[int, int, int], float]] = []

    for steering_action in tuning.motion_primitives:
        next_heading_bin = (heading_bin + steering_action) % tuning.heading_bin_count
        heading_deg = _bin_to_heading(next_heading_bin, tuning=tuning)
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
            segment_sample_count=tuning.segment_sample_count,
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
                    tuning=tuning,
                    obstacle_layout=obstacle_layout,
                    traffic_cost_context=traffic_cost_context,
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
    tuning: PlannerTuning,
    obstacle_layout: ObstacleLayout | None = None,
    traffic_cost_context: TrafficCostContext | None = None,
) -> float:
    current_cell = grid_map.cell_at(current[0], current[1])
    next_cell = grid_map.cell_at(next_state[0], next_state[1])
    base_cost = hypot(next_state[0] - current[0], next_state[1] - current[1]) * grid_map.cell_size
    if "Risk" in next_cell.zone_name:
        base_cost *= tuning.risk_zone_cost_multiplier
    if next_cell.has_risk_area:
        base_cost *= tuning.risk_area_cost_multiplier
    if steering_action != 0:
        base_cost += tuning.turn_action_cost
    heading_delta_bins = min(
        (next_state[2] - current[2]) % tuning.heading_bin_count,
        (current[2] - next_state[2]) % tuning.heading_bin_count,
    )
    base_cost += heading_delta_bins * tuning.heading_change_cost
    if next_cell.zone_name != current_cell.zone_name and "Risk" in next_cell.zone_name:
        base_cost += grid_map.cell_size * 0.5
    base_cost += traffic_transition_cost(
        grid_map=grid_map,
        current=current,
        next_state=next_state,
        obstacle_layout=obstacle_layout,
        traffic_cost_context=traffic_cost_context,
    )
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
    segment_sample_count: int,
) -> bool:
    for sample_index in range(1, segment_sample_count + 1):
        ratio = sample_index / segment_sample_count
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


def _heading_to_bin(heading_deg: float, *, tuning: PlannerTuning) -> int:
    normalized = normalize_heading_deg(heading_deg)
    return int(round((normalized % 360.0) / tuning.heading_step_deg)) % tuning.heading_bin_count


def _bin_to_heading(heading_bin: int, *, tuning: PlannerTuning) -> float:
    return (heading_bin % tuning.heading_bin_count) * tuning.heading_step_deg


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


def _path_cost(
    grid_map: GridMap,
    path: tuple[tuple[int, int, int], ...],
    *,
    tuning: PlannerTuning,
) -> float:
    return sum(
        _transition_cost(
            grid_map,
            current=current,
            next_state=next_state,
            steering_action=_steering_action(
                current[2],
                next_state[2],
                heading_bin_count=tuning.heading_bin_count,
            ),
            tuning=tuning,
        )
        for current, next_state in zip(path, path[1:], strict=False)
    )


def _steering_action(
    current_heading_bin: int,
    next_heading_bin: int,
    *,
    heading_bin_count: int,
) -> int:
    delta = (next_heading_bin - current_heading_bin) % heading_bin_count
    if delta == 0:
        return 0
    if delta == 1 or delta == heading_bin_count - 1:
        return 1 if delta == 1 else -1
    return 0


def _smooth_waypoints(
    grid_map: GridMap,
    waypoints: list[Waypoint],
    *,
    tuning: PlannerTuning,
) -> tuple[Waypoint, ...]:
    if len(waypoints) <= 2:
        return tuple(waypoints)

    smoothed = [waypoints[0]]
    current_index = 0
    while current_index < len(waypoints) - 1:
        next_index = current_index + 1
        for candidate_index in range(len(waypoints) - 1, current_index + 1, -1):
            if _exceeds_smoothed_segment_limit(
                waypoints[current_index],
                waypoints[candidate_index],
                tuning=tuning,
            ):
                continue
            if not _segment_is_collision_free(
                grid_map,
                current_x=waypoints[current_index].x,
                current_y=waypoints[current_index].y,
                next_x=waypoints[candidate_index].x,
                next_y=waypoints[candidate_index].y,
                segment_sample_count=tuning.segment_sample_count,
            ):
                continue
            direct_cost = _segment_path_cost(
                grid_map,
                start=waypoints[current_index],
                end=waypoints[candidate_index],
                tuning=tuning,
            )
            original_cost = _waypoint_subpath_cost(
                grid_map,
                waypoints=waypoints,
                start_index=current_index,
                end_index=candidate_index,
                tuning=tuning,
            )
            if direct_cost <= original_cost + tuning.smoothing_cost_tolerance:
                next_index = candidate_index
                break
        smoothed.append(waypoints[next_index])
        current_index = next_index
    return tuple(smoothed)


def _exceeds_smoothed_segment_limit(
    start: Waypoint,
    end: Waypoint,
    *,
    tuning: PlannerTuning,
) -> bool:
    if tuning.max_smoothed_segment_length_m is None:
        return False
    return hypot(end.x - start.x, end.y - start.y) > tuning.max_smoothed_segment_length_m


def _waypoint_path_cost(
    grid_map: GridMap,
    waypoints: tuple[Waypoint, ...],
    *,
    start_heading_deg: float,
    tuning: PlannerTuning,
) -> float:
    if len(waypoints) <= 1:
        return 0.0

    total_cost = 0.0
    previous_heading_bin = _heading_to_bin(start_heading_deg, tuning=tuning)
    for current, next_waypoint in zip(waypoints, waypoints[1:], strict=False):
        segment_cost = _segment_path_cost(
            grid_map,
            start=current,
            end=next_waypoint,
            tuning=tuning,
        )
        segment_heading_deg = normalize_heading_deg(
            degrees(atan2(next_waypoint.y - current.y, next_waypoint.x - current.x))
        )
        segment_heading_bin = _heading_to_bin(segment_heading_deg, tuning=tuning)
        heading_delta_bins = min(
            (segment_heading_bin - previous_heading_bin) % tuning.heading_bin_count,
            (previous_heading_bin - segment_heading_bin) % tuning.heading_bin_count,
        )
        total_cost += segment_cost + heading_delta_bins * tuning.heading_change_cost
        previous_heading_bin = segment_heading_bin
    return total_cost


def _waypoint_subpath_cost(
    grid_map: GridMap,
    *,
    waypoints: list[Waypoint],
    start_index: int,
    end_index: int,
    tuning: PlannerTuning,
) -> float:
    return sum(
        _segment_path_cost(grid_map, start=current, end=next_waypoint, tuning=tuning)
        for current, next_waypoint in zip(
            waypoints[start_index:end_index],
            waypoints[start_index + 1 : end_index + 1],
            strict=False,
        )
    )


def _segment_path_cost(
    grid_map: GridMap,
    *,
    start: Waypoint,
    end: Waypoint,
    tuning: PlannerTuning,
) -> float:
    distance = hypot(end.x - start.x, end.y - start.y)
    if distance <= 0.0:
        return 0.0

    sample_count = max(
        tuning.segment_sample_count,
        int(distance / max(grid_map.cell_size * 0.5, 1.0)),
    )
    total_multiplier = 0.0
    zone_transition_penalty = 0.0
    previous_zone_name: str | None = None
    for sample_index in range(1, sample_count + 1):
        ratio = sample_index / sample_count
        sample_x = start.x + (end.x - start.x) * ratio
        sample_y = start.y + (end.y - start.y) * ratio
        sample_cell = grid_map.locate_cell(sample_x, sample_y)
        multiplier = 1.0
        if "Risk" in sample_cell.zone_name:
            multiplier *= tuning.risk_zone_cost_multiplier
        if sample_cell.has_risk_area:
            multiplier *= tuning.risk_area_cost_multiplier
        total_multiplier += multiplier
        if previous_zone_name is not None and previous_zone_name != sample_cell.zone_name:
            if "Risk" in sample_cell.zone_name:
                zone_transition_penalty += grid_map.cell_size * 0.5
        previous_zone_name = sample_cell.zone_name
    return distance * (total_multiplier / sample_count) + zone_transition_penalty


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
