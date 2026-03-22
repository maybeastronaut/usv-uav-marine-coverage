"""USV patrol-route planners for preview partition scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from usv_uav_marine_coverage.environment import SeaMap
from usv_uav_marine_coverage.grid import GridMap


@dataclass(frozen=True)
class PatrolSegmentAccess:
    """One local access point back onto a patrol segment."""

    access_x: float
    access_y: float
    segment_start_index: int
    segment_end_index: int
    access_distance: float

    @property
    def goal_signature(self) -> str:
        """Return a stable short signature for cooldown and failure memory."""

        return (
            f"segment:{self.segment_start_index}->{self.segment_end_index}:"
            f"{round(self.access_x, 1)}:{round(self.access_y, 1)}"
        )


def build_default_usv_patrol_routes(
    sea_map: SeaMap,
) -> dict[str, tuple[tuple[float, float], ...]]:
    """Build the default one-nearshore/two-offshore patrol loops for preview USVs."""

    nearshore_left = sea_map.nearshore.x_start + 45.0
    nearshore_right = sea_map.nearshore.x_end - 40.0
    offshore_left = sea_map.offshore.x_start + 70.0
    offshore_right = sea_map.offshore.x_end - 80.0
    return {
        "USV-1": build_usv_serpentine_patrol_route(
            min_x=nearshore_left,
            max_x=nearshore_right,
            min_y=140.0,
            max_y=860.0,
            lane_spacing=90.0,
            sweep_axis="horizontal",
        ),
        "USV-2": build_usv_patrol_loop(
            min_x=offshore_left,
            max_x=offshore_right,
            min_y=120.0,
            max_y=420.0,
        ),
        "USV-3": build_usv_patrol_loop(
            min_x=offshore_left,
            max_x=offshore_right,
            min_y=580.0,
            max_y=880.0,
        ),
    }


def build_usv_patrol_loop(
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> tuple[tuple[float, float], ...]:
    """Build a rectangular nearshore patrol loop for one USV."""

    return (
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    )


def build_usv_serpentine_patrol_route(
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    lane_spacing: float,
    sweep_axis: str = "vertical",
) -> tuple[tuple[float, float], ...]:
    """Build a serpentine patrol route to revisit most of one zone."""

    if max_x <= min_x:
        raise ValueError("max_x must be greater than min_x.")
    if max_y <= min_y:
        raise ValueError("max_y must be greater than min_y.")
    if lane_spacing <= 0.0:
        raise ValueError("lane_spacing must be positive.")

    if sweep_axis not in {"vertical", "horizontal"}:
        raise ValueError("sweep_axis must be 'vertical' or 'horizontal'.")

    route: list[tuple[float, float]] = []
    if sweep_axis == "vertical":
        sweep_xs: list[float] = []
        current_x = min_x
        while current_x < max_x:
            sweep_xs.append(round(current_x, 3))
            current_x += lane_spacing
        if not sweep_xs or sweep_xs[-1] != round(max_x, 3):
            sweep_xs.append(round(max_x, 3))

        bottom_to_top = True
        for sweep_x in sweep_xs:
            if bottom_to_top:
                route.append((sweep_x, min_y))
                route.append((sweep_x, max_y))
            else:
                route.append((sweep_x, max_y))
                route.append((sweep_x, min_y))
            bottom_to_top = not bottom_to_top
        return tuple(route)

    sweep_ys: list[float] = []
    current_y = min_y
    while current_y < max_y:
        sweep_ys.append(round(current_y, 3))
        current_y += lane_spacing
    if not sweep_ys or sweep_ys[-1] != round(max_y, 3):
        sweep_ys.append(round(max_y, 3))

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


def find_progressive_patrol_segment_access(
    *,
    agent_x: float,
    agent_y: float,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int,
    blocked_goal_signature: str | None = None,
    lookahead_segments: int = 3,
) -> PatrolSegmentAccess | None:
    """Return a local patrol access that prefers forward patrol progression."""

    candidate_accesses = build_progressive_patrol_segment_accesses(
        agent_x=agent_x,
        agent_y=agent_y,
        patrol_route=patrol_route,
        preferred_end_index=preferred_end_index,
        blocked_goal_signature=blocked_goal_signature,
        lookahead_segments=lookahead_segments,
    )
    if not candidate_accesses:
        return None
    return candidate_accesses[0]


def build_progressive_patrol_segment_accesses(
    *,
    agent_x: float,
    agent_y: float,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int,
    blocked_goal_signature: str | None = None,
    lookahead_segments: int = 3,
) -> tuple[PatrolSegmentAccess, ...]:
    """Return forward-preferring segment accesses ordered by local preference."""

    if len(patrol_route) < 2:
        return ()

    route_length = len(patrol_route)
    candidate_accesses: list[tuple[int, PatrolSegmentAccess]] = []
    for offset in range(max(1, lookahead_segments)):
        segment_end_index = (preferred_end_index + offset) % route_length
        segment_start_index = (segment_end_index - 1) % route_length
        start_x, start_y = patrol_route[segment_start_index]
        end_x, end_y = patrol_route[segment_end_index]
        access_x, access_y = _project_point_to_segment(
            x=agent_x,
            y=agent_y,
            start_x=start_x,
            start_y=start_y,
            end_x=end_x,
            end_y=end_y,
        )
        access = PatrolSegmentAccess(
            access_x=access_x,
            access_y=access_y,
            segment_start_index=segment_start_index,
            segment_end_index=segment_end_index,
            access_distance=hypot(access_x - agent_x, access_y - agent_y),
        )
        if access.goal_signature == blocked_goal_signature:
            continue
        candidate_accesses.append((offset, access))

    if not candidate_accesses:
        return ()
    sorted_accesses = sorted(
        candidate_accesses,
        key=lambda item: (item[0], item[1].access_distance, item[1].segment_end_index),
    )
    return tuple(access for _, access in sorted_accesses)


def find_local_patrol_segment_access(
    *,
    agent_x: float,
    agent_y: float,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int = 0,
    blocked_goal_signature: str | None = None,
) -> PatrolSegmentAccess | None:
    """Return the nearest local patrol-segment access point for one USV."""

    candidate_accesses = build_local_patrol_segment_accesses(
        agent_x=agent_x,
        agent_y=agent_y,
        patrol_route=patrol_route,
        preferred_end_index=preferred_end_index,
        blocked_goal_signature=blocked_goal_signature,
    )
    if not candidate_accesses:
        return None
    return candidate_accesses[0]


def build_local_patrol_segment_accesses(
    *,
    agent_x: float,
    agent_y: float,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int = 0,
    blocked_goal_signature: str | None = None,
) -> tuple[PatrolSegmentAccess, ...]:
    """Return all local patrol-segment accesses ordered by local preference."""

    if len(patrol_route) < 2:
        return ()

    candidate_accesses: list[tuple[int, PatrolSegmentAccess]] = []
    route_length = len(patrol_route)
    for segment_end_index in range(route_length):
        segment_start_index = (segment_end_index - 1) % route_length
        start_x, start_y = patrol_route[segment_start_index]
        end_x, end_y = patrol_route[segment_end_index]
        access_x, access_y = _project_point_to_segment(
            x=agent_x,
            y=agent_y,
            start_x=start_x,
            start_y=start_y,
            end_x=end_x,
            end_y=end_y,
        )
        access = PatrolSegmentAccess(
            access_x=access_x,
            access_y=access_y,
            segment_start_index=segment_start_index,
            segment_end_index=segment_end_index,
            access_distance=hypot(access_x - agent_x, access_y - agent_y),
        )
        if access.goal_signature == blocked_goal_signature:
            continue
        cyclic_offset = (segment_end_index - preferred_end_index) % route_length
        candidate_accesses.append((cyclic_offset, access))

    if not candidate_accesses:
        return ()
    sorted_accesses = sorted(
        candidate_accesses,
        key=lambda item: (item[1].access_distance, item[0], item[1].segment_end_index),
    )
    return tuple(access for _, access in sorted_accesses)


def distance_from_patrol_access_to_map_edge(
    access: PatrolSegmentAccess,
    *,
    width: float,
    height: float,
) -> float:
    """Return the minimum clearance from one access point to the map boundary."""

    return min(access.access_x, width - access.access_x, access.access_y, height - access.access_y)


def distance_from_patrol_access_to_segment_endpoints(
    access: PatrolSegmentAccess,
    *,
    patrol_route: tuple[tuple[float, float], ...],
) -> float:
    """Return the minimum distance from one access point to the owning segment endpoints."""

    start_x, start_y = patrol_route[access.segment_start_index]
    end_x, end_y = patrol_route[access.segment_end_index]
    return min(
        hypot(access.access_x - start_x, access.access_y - start_y),
        hypot(access.access_x - end_x, access.access_y - end_y),
    )


def distance_from_patrol_access_to_hazards(
    access: PatrolSegmentAccess,
    *,
    grid_map: GridMap,
) -> float:
    """Return the minimum distance from one access point to obstacle or risk cells."""

    min_distance: float | None = None
    for cell in grid_map.flat_cells:
        if not cell.has_obstacle and not cell.has_risk_area:
            continue
        distance = hypot(access.access_x - cell.center_x, access.access_y - cell.center_y)
        if min_distance is None or distance < min_distance:
            min_distance = distance
    return float("inf") if min_distance is None else min_distance


def _project_point_to_segment(
    *,
    x: float,
    y: float,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
) -> tuple[float, float]:
    segment_dx = end_x - start_x
    segment_dy = end_y - start_y
    segment_length_sq = segment_dx**2 + segment_dy**2
    if segment_length_sq <= 1e-9:
        return (start_x, start_y)
    projection = ((x - start_x) * segment_dx + (y - start_y) * segment_dy) / segment_length_sq
    ratio = min(max(projection, 0.0), 1.0)
    return (start_x + segment_dx * ratio, start_y + segment_dy * ratio)
