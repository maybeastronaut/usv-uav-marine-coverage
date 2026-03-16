"""Multi-region coverage planning for UAV offshore search."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.information_map import (
    InformationMap,
    InformationValidity,
)

from .path_types import PathPlan, PathPlanStatus, Waypoint
from .uav_lawnmower_planner import build_lawnmower_route

UAV_MULTI_REGION_LANE_SPACING_M = 170.0
UAV_MULTI_REGION_MIN_Y = 120.0
UAV_MULTI_REGION_MAX_Y = 880.0
UAV_MULTI_REGION_X_MARGIN_LEFT = 35.0
UAV_MULTI_REGION_X_MARGIN_RIGHT = 60.0


@dataclass(frozen=True)
class UavSearchRegion:
    """One fixed offshore search AOI used by the multi-region planner."""

    region_id: str
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class UavSearchRegionScore:
    """One comparable freshness score for one offshore AOI."""

    region_id: str
    stale_ratio: float
    mean_information_age: float
    travel_cost: float


def build_fixed_uav_search_regions(
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> tuple[UavSearchRegion, ...]:
    """Return the fixed four offshore AOIs for the first multi-region planner."""

    if max_x <= min_x:
        raise ValueError("max_x must be greater than min_x.")
    if max_y <= min_y:
        raise ValueError("max_y must be greater than min_y.")

    mid_x = round((min_x + max_x) / 2.0, 3)
    mid_y = round((min_y + max_y) / 2.0, 3)
    return (
        UavSearchRegion("upper_left", min_x, mid_x, min_y, mid_y),
        UavSearchRegion("upper_right", mid_x, max_x, min_y, mid_y),
        UavSearchRegion("lower_left", min_x, mid_x, mid_y, max_y),
        UavSearchRegion("lower_right", mid_x, max_x, mid_y, max_y),
    )


def rank_uav_search_regions(
    *,
    agent: AgentState,
    info_map: InformationMap | None,
    regions: tuple[UavSearchRegion, ...],
    current_x: float | None = None,
    current_y: float | None = None,
    lane_spacing: float = UAV_MULTI_REGION_LANE_SPACING_M,
) -> tuple[UavSearchRegionScore, ...]:
    """Rank offshore AOIs by freshness first, travel cost second."""

    base_x = agent.x if current_x is None else current_x
    base_y = agent.y if current_y is None else current_y
    scores = [
        _score_region(
            region,
            info_map=info_map,
            current_x=base_x,
            current_y=base_y,
            lane_spacing=lane_spacing,
        )
        for region in regions
    ]
    scores.sort(
        key=lambda score: (
            -score.stale_ratio,
            -score.mean_information_age,
            score.travel_cost,
            score.region_id,
        )
    )
    return tuple(scores)


def build_uav_multi_region_route(
    agent: AgentState,
    *,
    info_map: InformationMap | None,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    lane_spacing: float = UAV_MULTI_REGION_LANE_SPACING_M,
) -> tuple[tuple[float, float], ...]:
    """Build one offshore patrol route by ranking and stitching four fixed AOIs."""

    regions = list(
        build_fixed_uav_search_regions(
            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
        )
    )
    if not regions:
        return ()

    route: list[tuple[float, float]] = []
    current_x = agent.x
    current_y = agent.y
    while regions:
        ranked_scores = rank_uav_search_regions(
            agent=agent,
            info_map=info_map,
            regions=tuple(regions),
            current_x=current_x,
            current_y=current_y,
            lane_spacing=lane_spacing,
        )
        next_region_id = ranked_scores[0].region_id
        next_region = next(region for region in regions if region.region_id == next_region_id)
        region_route = list(
            build_lawnmower_route(
                min_x=next_region.min_x,
                max_x=next_region.max_x,
                min_y=next_region.min_y,
                max_y=next_region.max_y,
                lane_spacing=lane_spacing,
            )
        )
        if not region_route:
            regions.remove(next_region)
            continue
        distance_to_end = _distance_to_route_endpoint(
            current_x,
            current_y,
            region_route[-1],
        )
        distance_to_start = _distance_to_route_endpoint(
            current_x,
            current_y,
            region_route[0],
        )
        if distance_to_end < distance_to_start:
            region_route.reverse()
        route.extend(region_route)
        current_x, current_y = region_route[-1]
        regions.remove(next_region)
    return tuple(route)


def build_uav_multi_region_plan(
    agent: AgentState,
    *,
    patrol_route_id: str,
    patrol_waypoint_index: int,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
) -> PathPlan:
    """Build a one-segment patrol plan to the current multi-region waypoint."""

    route = patrol_routes[patrol_route_id]
    target_x, target_y = route[patrol_waypoint_index]
    estimated_cost = round(hypot(target_x - agent.x, target_y - agent.y), 3)
    return PathPlan(
        plan_id=f"uav-multi-region-{agent.agent_id}-{patrol_waypoint_index}",
        planner_name="uav_multi_region_coverage_planner",
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


def select_uav_multi_region_waypoint_index(
    agent: AgentState,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    patrol_waypoint_index: int,
) -> int:
    """Return the first waypoint index that is meaningfully ahead of the UAV."""

    if not patrol_route:
        return 0
    route_length = len(patrol_route)
    start_index = patrol_waypoint_index % route_length
    for offset in range(route_length):
        candidate_index = (start_index + offset) % route_length
        waypoint_x, waypoint_y = patrol_route[candidate_index]
        if hypot(waypoint_x - agent.x, waypoint_y - agent.y) > agent.arrival_tolerance_m:
            return candidate_index
    return start_index


def _score_region(
    region: UavSearchRegion,
    *,
    info_map: InformationMap | None,
    current_x: float,
    current_y: float,
    lane_spacing: float,
) -> UavSearchRegionScore:
    stale_count = 0
    total_count = 0
    total_age = 0
    if info_map is not None:
        for row in info_map.grid_map.cells:
            for cell in row:
                if cell.zone_name != "Offshore Zone":
                    continue
                if cell.has_obstacle:
                    continue
                if not _cell_in_region(cell.center_x, cell.center_y, region):
                    continue
                total_count += 1
                state = info_map.state_at(cell.row, cell.col)
                total_age += state.information_age
                if state.validity == InformationValidity.STALE_KNOWN:
                    stale_count += 1
    if total_count == 0:
        stale_ratio = 1.0
        mean_age = 0.0
    else:
        stale_ratio = stale_count / total_count
        mean_age = total_age / total_count

    entry_route = build_lawnmower_route(
        min_x=region.min_x,
        max_x=region.max_x,
        min_y=region.min_y,
        max_y=region.max_y,
        lane_spacing=lane_spacing,
    )
    travel_cost = min(
        hypot(current_x - waypoint_x, current_y - waypoint_y)
        for waypoint_x, waypoint_y in entry_route
    )
    return UavSearchRegionScore(
        region_id=region.region_id,
        stale_ratio=round(stale_ratio, 6),
        mean_information_age=round(mean_age, 6),
        travel_cost=round(travel_cost, 6),
    )


def _cell_in_region(x: float, y: float, region: UavSearchRegion) -> bool:
    within_x = region.min_x <= x <= region.max_x
    within_y = region.min_y <= y <= region.max_y
    return within_x and within_y


def _distance_to_route_endpoint(
    current_x: float,
    current_y: float,
    endpoint: tuple[float, float],
) -> float:
    return hypot(current_x - endpoint[0], current_y - endpoint[1])
