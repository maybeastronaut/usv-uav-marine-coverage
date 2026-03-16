"""Persistent multi-region coverage planning for UAV offshore search."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import UavCoverageState
from usv_uav_marine_coverage.information_map import InformationMap, InformationValidity

from .path_types import PathPlan, PathPlanStatus, Waypoint
from .uav_lawnmower_planner import build_lawnmower_route
from .uav_multi_region_coverage_planner import (
    UAV_MULTI_REGION_LANE_SPACING_M,
    UAV_MULTI_REGION_MAX_Y,
    UAV_MULTI_REGION_MIN_Y,
    UAV_MULTI_REGION_X_MARGIN_LEFT,
    UAV_MULTI_REGION_X_MARGIN_RIGHT,
    UavSearchRegion,
    build_fixed_uav_search_regions,
)

UAV_REGION_COMMIT_MIN_WAYPOINTS = 6
UAV_REGION_COMMIT_MIN_STEPS = 60
UAV_REGION_SWITCH_STALE_RATIO_GAP = 0.15
UAV_REGION_SWITCH_MEAN_AGE_GAP = 100.0


@dataclass(frozen=True)
class PersistentUavRegionScore:
    """Comparable freshness-debt score for one offshore AOI."""

    region_id: str
    stale_ratio: float
    mean_information_age: float
    time_since_last_visit: int
    travel_cost: float


def build_uav_persistent_search_regions(
    *,
    min_x: float,
    max_x: float,
    min_y: float = UAV_MULTI_REGION_MIN_Y,
    max_y: float = UAV_MULTI_REGION_MAX_Y,
) -> tuple[UavSearchRegion, ...]:
    """Return the fixed offshore AOIs used by the persistent planner."""

    return build_fixed_uav_search_regions(
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )


def build_uav_region_coverage_route(
    region: UavSearchRegion,
    *,
    lane_spacing: float = UAV_MULTI_REGION_LANE_SPACING_M,
) -> tuple[tuple[float, float], ...]:
    """Return the boustrophedon route for one specific AOI."""

    return build_lawnmower_route(
        min_x=region.min_x,
        max_x=region.max_x,
        min_y=region.min_y,
        max_y=region.max_y,
        lane_spacing=lane_spacing,
    )


def rank_persistent_uav_search_regions(
    *,
    agent: AgentState,
    info_map: InformationMap | None,
    regions: tuple[UavSearchRegion, ...],
    coverage_state: UavCoverageState,
    step: int,
    current_x: float | None = None,
    current_y: float | None = None,
    lane_spacing: float = UAV_MULTI_REGION_LANE_SPACING_M,
) -> tuple[PersistentUavRegionScore, ...]:
    """Rank offshore AOIs by freshness debt, then travel cost."""

    base_x = agent.x if current_x is None else current_x
    base_y = agent.y if current_y is None else current_y
    scores = [
        _score_region(
            region,
            info_map=info_map,
            coverage_state=coverage_state,
            step=step,
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
            -score.time_since_last_visit,
            score.travel_cost,
            score.region_id,
        )
    )
    return tuple(scores)


def should_replan_persistent_region(
    *,
    coverage_state: UavCoverageState,
    step: int,
    ranked_scores: tuple[PersistentUavRegionScore, ...],
) -> tuple[bool, str]:
    """Return whether the UAV should switch to another AOI at this step."""

    if not coverage_state.current_region_id or not coverage_state.region_route:
        return (True, "missing_region_route")
    if coverage_state.region_waypoint_index >= len(coverage_state.region_route):
        return (True, "region_route_complete")
    if not ranked_scores:
        return (False, "no_candidate_regions")
    current_score = next(
        (score for score in ranked_scores if score.region_id == coverage_state.current_region_id),
        None,
    )
    if current_score is None:
        return (True, "current_region_missing")
    if not _commitment_released(coverage_state, step):
        return (False, "commitment_active")
    best_score = ranked_scores[0]
    if best_score.region_id == coverage_state.current_region_id:
        return (False, "current_region_still_best")
    stale_gap = best_score.stale_ratio - current_score.stale_ratio
    age_gap = best_score.mean_information_age - current_score.mean_information_age
    if stale_gap >= UAV_REGION_SWITCH_STALE_RATIO_GAP:
        return (True, "higher_stale_ratio_region")
    if stale_gap > 0.0 and age_gap >= UAV_REGION_SWITCH_MEAN_AGE_GAP:
        return (True, "higher_mean_age_region")
    return (False, "debt_gap_not_large_enough")


def select_persistent_uav_region(
    *,
    agent: AgentState,
    info_map: InformationMap | None,
    regions: tuple[UavSearchRegion, ...],
    coverage_state: UavCoverageState,
    step: int,
    lane_spacing: float = UAV_MULTI_REGION_LANE_SPACING_M,
    force_replan: bool = False,
    occupied_region_ids: frozenset[str] = frozenset(),
) -> tuple[UavCoverageState, PersistentUavRegionScore]:
    """Select or keep one AOI according to freshness debt and commitment rules."""

    ranked_scores = rank_persistent_uav_search_regions(
        agent=agent,
        info_map=info_map,
        regions=regions,
        coverage_state=coverage_state,
        step=step,
        lane_spacing=lane_spacing,
    )
    if not ranked_scores:
        raise ValueError("Persistent UAV planner requires at least one offshore region.")

    should_replan, reason = should_replan_persistent_region(
        coverage_state=coverage_state,
        step=step,
        ranked_scores=ranked_scores,
    )
    if not force_replan and not should_replan:
        current_score = next(
            score for score in ranked_scores if score.region_id == coverage_state.current_region_id
        )
        return (
            replace_region_visit_step(
                coverage_state,
                region_id=current_score.region_id,
                step=step,
            ),
            current_score,
        )

    selected_score = _select_region_candidate(
        ranked_scores=ranked_scores,
        current_region_id=coverage_state.current_region_id,
        occupied_region_ids=occupied_region_ids,
    )
    selected_region = next(
        region for region in regions if region.region_id == selected_score.region_id
    )
    base_region_route = build_uav_region_coverage_route(
        selected_region,
        lane_spacing=lane_spacing,
    )
    region_route, waypoint_index = _orient_region_route_for_entry(
        agent=agent,
        region_route=base_region_route,
    )
    committed_waypoints = min(
        UAV_REGION_COMMIT_MIN_WAYPOINTS,
        max(1, len(region_route) - waypoint_index),
    )
    updated_state = replace_region_visit_step(
        coverage_state,
        region_id=selected_region.region_id,
        step=step,
    )
    updated_state = UavCoverageState(
        agent_id=coverage_state.agent_id,
        current_region_id=selected_region.region_id,
        region_route=region_route,
        region_waypoint_index=waypoint_index,
        region_entry_step=step,
        committed_waypoints_remaining=committed_waypoints,
        last_replan_step=step,
        last_replan_reason=reason if not force_replan else "forced_region_replan",
        region_last_visit_steps=updated_state.region_last_visit_steps,
    )
    return (updated_state, selected_score)


def build_uav_persistent_multi_region_plan(
    agent: AgentState,
    *,
    coverage_state: UavCoverageState,
) -> PathPlan:
    """Build a one-segment patrol plan to the current persistent-region waypoint."""

    if not coverage_state.region_route:
        raise ValueError("Persistent UAV plan requires a non-empty region route.")
    target_x, target_y = coverage_state.region_route[coverage_state.region_waypoint_index]
    estimated_cost = round(hypot(target_x - agent.x, target_y - agent.y), 3)
    return PathPlan(
        plan_id=(
            f"uav-persistent-multi-region-{agent.agent_id}-"
            f"{coverage_state.current_region_id}-{coverage_state.region_waypoint_index}"
        ),
        planner_name="uav_persistent_multi_region_coverage_planner",
        agent_id=agent.agent_id,
        task_id=None,
        status=PathPlanStatus.PLANNED,
        waypoints=(Waypoint(x=agent.x, y=agent.y), Waypoint(x=target_x, y=target_y)),
        goal_x=target_x,
        goal_y=target_y,
        estimated_cost=estimated_cost,
    )


def advance_uav_region_waypoint(
    coverage_state: UavCoverageState,
) -> UavCoverageState:
    """Advance the persistent-region waypoint index after one waypoint is reached."""

    if not coverage_state.region_route:
        return coverage_state
    next_index = coverage_state.region_waypoint_index + 1
    if next_index >= len(coverage_state.region_route):
        return UavCoverageState(
            agent_id=coverage_state.agent_id,
            current_region_id=None,
            region_route=(),
            region_waypoint_index=0,
            region_entry_step=coverage_state.region_entry_step,
            committed_waypoints_remaining=0,
            last_replan_step=coverage_state.last_replan_step,
            last_replan_reason="region_route_complete",
            region_last_visit_steps=coverage_state.region_last_visit_steps,
        )
    return UavCoverageState(
        agent_id=coverage_state.agent_id,
        current_region_id=coverage_state.current_region_id,
        region_route=coverage_state.region_route,
        region_waypoint_index=next_index,
        region_entry_step=coverage_state.region_entry_step,
        committed_waypoints_remaining=max(0, coverage_state.committed_waypoints_remaining - 1),
        last_replan_step=coverage_state.last_replan_step,
        last_replan_reason=coverage_state.last_replan_reason,
        region_last_visit_steps=coverage_state.region_last_visit_steps,
    )


def build_empty_uav_coverage_state(agent_id: str) -> UavCoverageState:
    """Return the empty persistent patrol state for one UAV."""

    return UavCoverageState(agent_id=agent_id)


def _select_region_candidate(
    *,
    ranked_scores: tuple[PersistentUavRegionScore, ...],
    current_region_id: str | None,
    occupied_region_ids: frozenset[str],
) -> PersistentUavRegionScore:
    if not occupied_region_ids:
        return ranked_scores[0]
    preferred_scores = tuple(
        score
        for score in ranked_scores
        if score.region_id not in occupied_region_ids or score.region_id == current_region_id
    )
    if preferred_scores:
        return preferred_scores[0]
    return ranked_scores[0]


def _orient_region_route_for_entry(
    *,
    agent: AgentState,
    region_route: tuple[tuple[float, float], ...],
) -> tuple[tuple[tuple[float, float], ...], int]:
    """Enter one AOI from the nearer sweep endpoint to preserve route continuity."""

    if not region_route:
        raise ValueError("Persistent UAV region route cannot be empty.")
    start_x, start_y = region_route[0]
    end_x, end_y = region_route[-1]
    start_distance = hypot(agent.x - start_x, agent.y - start_y)
    end_distance = hypot(agent.x - end_x, agent.y - end_y)
    if end_distance < start_distance:
        return (tuple(reversed(region_route)), 0)
    return (region_route, 0)


def replace_region_visit_step(
    coverage_state: UavCoverageState,
    *,
    region_id: str,
    step: int,
) -> UavCoverageState:
    """Return a copy with one region's last-visit step updated."""

    visit_map = dict(coverage_state.region_last_visit_steps)
    visit_map[region_id] = step
    return UavCoverageState(
        agent_id=coverage_state.agent_id,
        current_region_id=coverage_state.current_region_id,
        region_route=coverage_state.region_route,
        region_waypoint_index=coverage_state.region_waypoint_index,
        region_entry_step=coverage_state.region_entry_step,
        committed_waypoints_remaining=coverage_state.committed_waypoints_remaining,
        last_replan_step=coverage_state.last_replan_step,
        last_replan_reason=coverage_state.last_replan_reason,
        region_last_visit_steps=tuple(sorted(visit_map.items())),
    )


def _score_region(
    region: UavSearchRegion,
    *,
    info_map: InformationMap | None,
    coverage_state: UavCoverageState,
    step: int,
    current_x: float,
    current_y: float,
    lane_spacing: float,
) -> PersistentUavRegionScore:
    stale_count = 0
    total_count = 0
    total_age = 0
    if info_map is not None:
        for row in info_map.grid_map.cells:
            for cell in row:
                if cell.zone_name != "Offshore Zone" or cell.has_obstacle:
                    continue
                if not _cell_in_region(cell.center_x, cell.center_y, region):
                    continue
                total_count += 1
                state = info_map.state_at(cell.row, cell.col)
                total_age += state.information_age
                if state.validity == InformationValidity.STALE_KNOWN:
                    stale_count += 1
    stale_ratio = 1.0 if total_count == 0 else stale_count / total_count
    mean_age = 0.0 if total_count == 0 else total_age / total_count
    entry_route = build_uav_region_coverage_route(region, lane_spacing=lane_spacing)
    travel_cost = min(
        hypot(current_x - waypoint_x, current_y - waypoint_y)
        for waypoint_x, waypoint_y in entry_route
    )
    last_visit_step = _last_region_visit_step(coverage_state, region.region_id)
    time_since_last_visit = step + 1 if last_visit_step < 0 else step - last_visit_step
    return PersistentUavRegionScore(
        region_id=region.region_id,
        stale_ratio=round(stale_ratio, 6),
        mean_information_age=round(mean_age, 6),
        time_since_last_visit=time_since_last_visit,
        travel_cost=round(travel_cost, 6),
    )


def _last_region_visit_step(coverage_state: UavCoverageState, region_id: str) -> int:
    visit_map = dict(coverage_state.region_last_visit_steps)
    return int(visit_map.get(region_id, -1))


def _commitment_released(coverage_state: UavCoverageState, step: int) -> bool:
    if coverage_state.region_entry_step < 0:
        return True
    if coverage_state.committed_waypoints_remaining <= 0:
        return True
    return step - coverage_state.region_entry_step >= UAV_REGION_COMMIT_MIN_STEPS


def _cell_in_region(x: float, y: float, region: UavSearchRegion) -> bool:
    return region.min_x <= x <= region.max_x and region.min_y <= y <= region.max_y


__all__ = [
    "PersistentUavRegionScore",
    "UAV_MULTI_REGION_LANE_SPACING_M",
    "UAV_MULTI_REGION_MAX_Y",
    "UAV_MULTI_REGION_MIN_Y",
    "UAV_MULTI_REGION_X_MARGIN_LEFT",
    "UAV_MULTI_REGION_X_MARGIN_RIGHT",
    "advance_uav_region_waypoint",
    "build_empty_uav_coverage_state",
    "build_uav_persistent_multi_region_plan",
    "build_uav_persistent_search_regions",
    "build_uav_region_coverage_route",
    "rank_persistent_uav_search_regions",
    "replace_region_visit_step",
    "select_persistent_uav_region",
]
