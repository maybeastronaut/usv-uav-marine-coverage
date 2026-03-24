"""Return-to-patrol target selection and replanning helpers."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, degrees, hypot

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    shortest_heading_delta_deg,
)
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
)
from usv_uav_marine_coverage.execution.progress_feedback import (
    build_goal_signature,
    should_replan_patrol,
    should_replan_return,
)
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.planning.traffic_cost import TrafficCostContext
from usv_uav_marine_coverage.planning.usv_path_planner import build_usv_path_plan
from usv_uav_marine_coverage.planning.usv_patrol_planner import (
    PatrolSegmentAccess,
    build_local_patrol_segment_accesses,
    build_progressive_patrol_segment_accesses,
    distance_from_patrol_access_to_hazards,
    distance_from_patrol_access_to_map_edge,
    distance_from_patrol_access_to_segment_endpoints,
    find_local_patrol_segment_access,
    find_progressive_patrol_segment_access,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskStatus, TaskType

RETURN_PLAN_REUSE_WINDOW_STEPS = 3
PATROL_MIN_REPLAN_INTERVAL_STEPS = 2
RETURN_ACCESS_EDGE_CLEARANCE_M = 60.0
RETURN_ACCESS_ENDPOINT_CLEARANCE_M = 35.0
RETURN_ACCESS_STALE_RADIUS_M = 120.0
RETURN_ACCESS_HOTSPOT_RADIUS_M = 150.0
RETURN_ACCESS_HAZARD_CLEARANCE_M = USV_COLLISION_CLEARANCE_M * 1.5
RETURN_ACCESS_MIN_DISTANCE_M = 24.0
RETURN_ACCESS_APPROACH_SAMPLE_SPACING_M = 12.0
RETURN_ACCESS_APPROACH_CLEARANCE_M = USV_COLLISION_CLEARANCE_M * 1.25
RETURN_ACCESS_STALE_WEIGHT = 0.35
RETURN_ACCESS_HOTSPOT_WEIGHT = 0.20
RETURN_ACCESS_APPROACH_WEIGHT = 0.30
RETURN_ACCESS_HEADING_WEIGHT = 0.20
RETURN_ACCESS_TRAVEL_WEIGHT = 0.15


@dataclass(frozen=True)
class ReturnPatrolAccessSelection:
    """One selected return-to-patrol access with its selection source."""

    access: PatrolSegmentAccess
    source: str


def find_nearest_patrol_rejoin_point(
    agent: AgentState,
    patrol_route: tuple[tuple[float, float], ...],
) -> tuple[int, float, float]:
    """Return the nearest local patrol-segment access for the current agent position."""

    access = find_local_patrol_segment_access(
        agent_x=agent.x,
        agent_y=agent.y,
        patrol_route=patrol_route,
    )
    if access is None:
        indexed_points = [(index, point[0], point[1]) for index, point in enumerate(patrol_route)]
        index, x, y = min(
            indexed_points,
            key=lambda item: (agent.x - item[1]) ** 2 + (agent.y - item[2]) ** 2,
        )
        return (index, x, y)
    return (access.segment_end_index, access.access_x, access.access_y)


def _build_local_patrol_return_transition(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    info_map: InformationMap | None,
    task_records: tuple[TaskRecord, ...],
    skip_blocked_goal: bool = False,
) -> AgentExecutionState:
    from .basic_state_machine import transition_to_return_to_patrol

    selection = _find_local_patrol_access(
        agent,
        patrol_route=patrol_route,
        execution_state=execution_state,
        progress_state=progress_state,
        skip_blocked_goal=skip_blocked_goal,
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
    )
    if selection is None:
        patrol_index, return_x, return_y = find_nearest_patrol_rejoin_point(agent, patrol_route)
        return transition_to_return_to_patrol(
            execution_state,
            return_target_x=return_x,
            return_target_y=return_y,
            patrol_waypoint_index=patrol_index,
            return_target_source="fallback_patrol_waypoint",
        )
    return replace(
        transition_to_return_to_patrol(
            execution_state,
            return_target_x=selection.access.access_x,
            return_target_y=selection.access.access_y,
            patrol_waypoint_index=selection.access.segment_end_index,
            return_target_source=selection.source,
            rejoin_to_segment=True,
        ),
    )


def _find_local_patrol_access(
    agent: AgentState,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    skip_blocked_goal: bool,
    grid_map: GridMap,
    info_map: InformationMap | None,
    task_records: tuple[TaskRecord, ...],
) -> ReturnPatrolAccessSelection | None:
    blocked_goal_signature = None
    if skip_blocked_goal:
        blocked_goal_signature = progress_state.blocked_goal_signature
    selection = _select_safe_value_patrol_access(
        agent,
        patrol_route=patrol_route,
        preferred_end_index=execution_state.patrol_waypoint_index,
        blocked_goal_signature=blocked_goal_signature,
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
        relax_endpoint_filter=False,
    )
    if selection is not None:
        return selection
    selection = _select_safe_value_patrol_access(
        agent,
        patrol_route=patrol_route,
        preferred_end_index=execution_state.patrol_waypoint_index,
        blocked_goal_signature=blocked_goal_signature,
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
        relax_endpoint_filter=True,
    )
    if selection is not None:
        return selection
    legacy_access = _find_legacy_local_patrol_access(
        agent,
        patrol_route=patrol_route,
        preferred_end_index=execution_state.patrol_waypoint_index,
        blocked_goal_signature=blocked_goal_signature,
    )
    if legacy_access is None:
        return None
    return ReturnPatrolAccessSelection(
        access=legacy_access,
        source="fallback_legacy_local",
    )


def _find_legacy_local_patrol_access(
    agent: AgentState,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int,
    blocked_goal_signature: str | None,
) -> PatrolSegmentAccess | None:
    if len(patrol_route) > 4:
        access = find_progressive_patrol_segment_access(
            agent_x=agent.x,
            agent_y=agent.y,
            patrol_route=patrol_route,
            preferred_end_index=preferred_end_index,
            blocked_goal_signature=blocked_goal_signature,
        )
        if access is not None:
            return access
    return find_local_patrol_segment_access(
        agent_x=agent.x,
        agent_y=agent.y,
        patrol_route=patrol_route,
        preferred_end_index=preferred_end_index,
        blocked_goal_signature=blocked_goal_signature,
    )


def _select_safe_value_patrol_access(
    agent: AgentState,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int,
    blocked_goal_signature: str | None,
    grid_map: GridMap,
    info_map: InformationMap | None,
    task_records: tuple[TaskRecord, ...],
    relax_endpoint_filter: bool,
) -> ReturnPatrolAccessSelection | None:
    candidates = _collect_patrol_access_candidates(
        agent,
        patrol_route=patrol_route,
        preferred_end_index=preferred_end_index,
        blocked_goal_signature=blocked_goal_signature,
    )
    if not candidates:
        return None

    scored_candidates: list[tuple[PatrolSegmentAccess, float, float, float, float, float]] = []
    for access in candidates:
        if not _is_safe_patrol_access_candidate(
            agent,
            access,
            patrol_route=patrol_route,
            blocked_goal_signature=blocked_goal_signature,
            grid_map=grid_map,
            relax_endpoint_filter=relax_endpoint_filter,
        ):
            continue
        approach_clearance = _approach_clearance_near_access(
            agent,
            access=access,
            grid_map=grid_map,
        )
        heading_alignment = _approach_heading_alignment(agent, access=access)
        scored_candidates.append(
            (
                access,
                _stale_pressure_near_access(access, info_map=info_map),
                _hotspot_pressure_near_access(access, task_records=task_records),
                approach_clearance,
                heading_alignment,
                access.access_distance,
            )
        )
    if not scored_candidates:
        return None

    max_stale = max(stale for _, stale, _, _, _, _ in scored_candidates)
    max_hotspot = max(hotspot for _, _, hotspot, _, _, _ in scored_candidates)
    max_approach = max(approach for _, _, _, approach, _, _ in scored_candidates)
    max_heading = max(heading for _, _, _, _, heading, _ in scored_candidates)
    max_travel = max(travel for _, _, _, _, _, travel in scored_candidates)

    def _normalized(value: float, maximum: float) -> float:
        if maximum <= 1e-9:
            return 0.0
        return value / maximum

    selected_access = max(
        scored_candidates,
        key=lambda item: (
            (
                RETURN_ACCESS_STALE_WEIGHT * _normalized(item[1], max_stale)
                + RETURN_ACCESS_HOTSPOT_WEIGHT * _normalized(item[2], max_hotspot)
                + RETURN_ACCESS_APPROACH_WEIGHT * _normalized(item[3], max_approach)
                + RETURN_ACCESS_HEADING_WEIGHT * _normalized(item[4], max_heading)
                - RETURN_ACCESS_TRAVEL_WEIGHT * _normalized(item[5], max_travel)
            ),
            -item[5],
            -item[0].segment_end_index,
        ),
    )[0]
    return ReturnPatrolAccessSelection(
        access=selected_access,
        source="fallback_relaxed_endpoint" if relax_endpoint_filter else "safe_value_access",
    )


def _collect_patrol_access_candidates(
    agent: AgentState,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    preferred_end_index: int,
    blocked_goal_signature: str | None,
) -> tuple[PatrolSegmentAccess, ...]:
    candidates_by_signature: dict[str, PatrolSegmentAccess] = {}
    if len(patrol_route) > 4:
        for access in build_progressive_patrol_segment_accesses(
            agent_x=agent.x,
            agent_y=agent.y,
            patrol_route=patrol_route,
            preferred_end_index=preferred_end_index,
            blocked_goal_signature=blocked_goal_signature,
        ):
            candidates_by_signature.setdefault(access.goal_signature, access)
    for access in build_local_patrol_segment_accesses(
        agent_x=agent.x,
        agent_y=agent.y,
        patrol_route=patrol_route,
        preferred_end_index=preferred_end_index,
        blocked_goal_signature=blocked_goal_signature,
    ):
        candidates_by_signature.setdefault(access.goal_signature, access)
    return tuple(candidates_by_signature.values())


def _is_safe_patrol_access_candidate(
    agent: AgentState,
    access: PatrolSegmentAccess,
    *,
    patrol_route: tuple[tuple[float, float], ...],
    blocked_goal_signature: str | None,
    grid_map: GridMap,
    relax_endpoint_filter: bool,
) -> bool:
    if blocked_goal_signature is not None and access.goal_signature == blocked_goal_signature:
        return False
    if (
        distance_from_patrol_access_to_map_edge(
            access,
            width=grid_map.width,
            height=grid_map.height,
        )
        < RETURN_ACCESS_EDGE_CLEARANCE_M
    ):
        return False
    if (
        not relax_endpoint_filter
        and distance_from_patrol_access_to_segment_endpoints(
            access,
            patrol_route=patrol_route,
        )
        < RETURN_ACCESS_ENDPOINT_CLEARANCE_M
    ):
        return False
    if (
        distance_from_patrol_access_to_hazards(
            access,
            grid_map=grid_map,
        )
        < RETURN_ACCESS_HAZARD_CLEARANCE_M
    ):
        return False
    if access.access_distance < max(
        RETURN_ACCESS_MIN_DISTANCE_M,
        agent.arrival_tolerance_m * 1.25,
    ):
        return False
    return True


def _approach_clearance_near_access(
    agent: AgentState,
    *,
    access: PatrolSegmentAccess,
    grid_map: GridMap,
) -> float:
    sample_count = max(
        2,
        int(access.access_distance / RETURN_ACCESS_APPROACH_SAMPLE_SPACING_M) + 1,
    )
    best_clearance = float("inf")
    for sample_index in range(1, sample_count + 1):
        ratio = sample_index / sample_count
        sample_x = agent.x + (access.access_x - agent.x) * ratio
        sample_y = agent.y + (access.access_y - agent.y) * ratio
        edge_clearance = min(
            sample_x,
            grid_map.width - sample_x,
            sample_y,
            grid_map.height - sample_y,
        )
        hazard_clearance = _distance_to_hazard_cells(
            sample_x,
            sample_y,
            grid_map=grid_map,
        )
        best_clearance = min(best_clearance, edge_clearance, hazard_clearance)
    return max(0.0, best_clearance - RETURN_ACCESS_APPROACH_CLEARANCE_M)


def _approach_heading_alignment(
    agent: AgentState,
    *,
    access: PatrolSegmentAccess,
) -> float:
    approach_dx = access.access_x - agent.x
    approach_dy = access.access_y - agent.y
    if hypot(approach_dx, approach_dy) <= 1e-9:
        return 0.0
    desired_heading_deg = degrees(atan2(approach_dy, approach_dx))
    heading_delta = abs(shortest_heading_delta_deg(agent.heading_deg, desired_heading_deg))
    return max(0.0, 180.0 - heading_delta)


def _distance_to_hazard_cells(
    x: float,
    y: float,
    *,
    grid_map: GridMap,
) -> float:
    min_distance: float | None = None
    for cell in grid_map.flat_cells:
        if not cell.has_obstacle and not cell.has_risk_area:
            continue
        distance = hypot(x - cell.center_x, y - cell.center_y)
        if min_distance is None or distance < min_distance:
            min_distance = distance
    return float("inf") if min_distance is None else min_distance


def _stale_pressure_near_access(
    access: PatrolSegmentAccess,
    *,
    info_map: InformationMap | None,
) -> float:
    if info_map is None:
        return 0.0
    stale_pressure = 0.0
    for cell in info_map.grid_map.flat_cells:
        if cell.has_obstacle:
            continue
        if hypot(access.access_x - cell.center_x, access.access_y - cell.center_y) > (
            RETURN_ACCESS_STALE_RADIUS_M
        ):
            continue
        state = info_map.state_at(cell.row, cell.col)
        if state.validity.value != "stale_known":
            continue
        stale_pressure += 1.0
    return stale_pressure


def _hotspot_pressure_near_access(
    access: PatrolSegmentAccess,
    *,
    task_records: tuple[TaskRecord, ...],
) -> float:
    hotspot_pressure = 0.0
    for task in task_records:
        if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
            continue
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            continue
        if hypot(access.access_x - task.target_x, access.access_y - task.target_y) > (
            RETURN_ACCESS_HOTSPOT_RADIUS_M
        ):
            continue
        hotspot_pressure += 1.0
    return hotspot_pressure


def _should_refresh_return_plan(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    step: int,
    usv_path_planner: str,
) -> bool:
    if not should_replan_return(
        agent,
        execution_state=execution_state,
        usv_path_planner=usv_path_planner,
    ):
        return False
    plan = execution_state.active_plan
    if plan is None or plan.status != PathPlanStatus.PLANNED:
        return True
    if plan.planner_name != usv_path_planner:
        return True
    if execution_state.current_waypoint_index >= len(plan.waypoints):
        return True
    if execution_state.return_target_x is None or execution_state.return_target_y is None:
        return True
    if (
        abs(plan.goal_x - execution_state.return_target_x) > 1.0
        or abs(plan.goal_y - execution_state.return_target_y) > 1.0
    ):
        return True
    if step - execution_state.last_return_plan_step > RETURN_PLAN_REUSE_WINDOW_STEPS:
        return True
    return False


def _should_refresh_patrol_plan(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    goal_x: float,
    goal_y: float,
    step: int,
    usv_path_planner: str,
) -> bool:
    if not should_replan_patrol(
        agent,
        execution_state=execution_state,
        goal_x=goal_x,
        goal_y=goal_y,
        usv_path_planner=usv_path_planner,
    ):
        return False
    plan = execution_state.active_plan
    if plan is None or plan.status != PathPlanStatus.PLANNED:
        return True
    if plan.planner_name != usv_path_planner:
        return True
    if execution_state.current_waypoint_index >= len(plan.waypoints):
        return True
    if abs(plan.goal_x - goal_x) > 1.0 or abs(plan.goal_y - goal_y) > 1.0:
        return True
    if step - execution_state.last_patrol_plan_step >= PATROL_MIN_REPLAN_INTERVAL_STEPS:
        return True
    return False


def _plan_patrol_path(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    obstacle_layout: ObstacleLayout | None,
    allow_patrol_index_advance: bool,
    usv_path_planner: str,
    step: int,
    traffic_cost_context: TrafficCostContext | None = None,
) -> AgentExecutionState:
    route_length = len(patrol_route)
    if route_length == 0:
        return replace(execution_state, active_plan=None, current_waypoint_index=0)

    start_index = execution_state.patrol_waypoint_index % route_length
    if allow_patrol_index_advance:
        start_index = (start_index + 1) % route_length

    blocked_state: AgentExecutionState | None = None
    for offset in range(route_length):
        patrol_index = (start_index + offset) % route_length
        goal_x, goal_y = patrol_route[patrol_index]
        plan = build_usv_path_plan(
            agent,
            grid_map=grid_map,
            obstacle_layout=obstacle_layout,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name=usv_path_planner,
            task_id=None,
            stats_context="runtime_patrol",
            traffic_cost_context=traffic_cost_context,
        )
        candidate_state = replace(
            execution_state,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_waypoint_index=patrol_index,
            last_patrol_plan_step=step,
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate_state
        if blocked_state is None and not allow_patrol_index_advance:
            blocked_state = candidate_state

    return blocked_state or replace(
        execution_state,
        active_plan=None,
        current_waypoint_index=0,
        last_patrol_plan_step=step,
    )


def _plan_return_to_patrol_path(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    obstacle_layout: ObstacleLayout | None,
    info_map: InformationMap | None,
    allow_patrol_index_advance: bool,
    step: int,
    task_records: tuple[TaskRecord, ...],
    usv_path_planner: str,
    traffic_cost_context: TrafficCostContext | None = None,
) -> AgentExecutionState:
    if agent.kind != "USV":
        assert (
            execution_state.return_target_x is not None
            and execution_state.return_target_y is not None
        )
        return replace(
            execution_state,
            active_plan=build_direct_line_plan(
                agent,
                goal_x=execution_state.return_target_x,
                goal_y=execution_state.return_target_y,
                planner_name="direct_line_planner",
                task_id=None,
            ),
            current_waypoint_index=0,
        )

    route_length = len(patrol_route)
    if route_length == 0:
        return replace(execution_state, active_plan=None, current_waypoint_index=0)

    if (
        progress_state.cooldown_until_step > step
        and progress_state.blocked_goal_signature is not None
        and build_goal_signature(
            stage=execution_state.stage,
            active_task=None,
            execution_state=execution_state,
        )
        == progress_state.blocked_goal_signature
    ):
        access = _find_local_patrol_access(
            agent,
            patrol_route=patrol_route,
            execution_state=execution_state,
            progress_state=progress_state,
            skip_blocked_goal=True,
            grid_map=grid_map,
            info_map=info_map,
            task_records=task_records,
        )
        if access is not None:
            execution_state = replace(
                execution_state,
                patrol_waypoint_index=access.access.segment_end_index,
                return_target_x=access.access.access_x,
                return_target_y=access.access.access_y,
                return_target_source=access.source,
                rejoin_to_segment=True,
            )

    if (
        not allow_patrol_index_advance
        and execution_state.return_target_x is not None
        and execution_state.return_target_y is not None
    ):
        current_target_plan = build_usv_path_plan(
            agent,
            grid_map=grid_map,
            obstacle_layout=obstacle_layout,
            goal_x=execution_state.return_target_x,
            goal_y=execution_state.return_target_y,
            planner_name=usv_path_planner,
            task_id=None,
            stats_context="runtime_return_to_patrol",
            traffic_cost_context=traffic_cost_context,
        )
        if current_target_plan.status == PathPlanStatus.PLANNED:
            return replace(
                execution_state,
                active_plan=current_target_plan,
                current_waypoint_index=0,
                last_return_plan_step=step,
            )

    start_index = execution_state.patrol_waypoint_index % route_length
    if allow_patrol_index_advance:
        start_index = (start_index + 1) % route_length

    blocked_state: AgentExecutionState | None = None
    for offset in range(route_length):
        patrol_index = (start_index + offset) % route_length
        goal_x, goal_y = patrol_route[patrol_index]
        plan = build_usv_path_plan(
            agent,
            grid_map=grid_map,
            obstacle_layout=obstacle_layout,
            goal_x=goal_x,
            goal_y=goal_y,
            planner_name=usv_path_planner,
            task_id=None,
            stats_context="runtime_return_to_patrol",
            traffic_cost_context=traffic_cost_context,
        )
        candidate_state = replace(
            execution_state,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_waypoint_index=patrol_index,
            return_target_x=goal_x,
            return_target_y=goal_y,
            return_target_source="fallback_patrol_waypoint",
            last_return_plan_step=step,
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate_state
        if blocked_state is None and not allow_patrol_index_advance:
            blocked_state = candidate_state

    access = _find_local_patrol_access(
        agent,
        patrol_route=patrol_route,
        execution_state=execution_state,
        progress_state=progress_state,
        skip_blocked_goal=allow_patrol_index_advance,
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
    )
    if access is not None:
        plan = build_usv_path_plan(
            agent,
            grid_map=grid_map,
            obstacle_layout=obstacle_layout,
            goal_x=access.access.access_x,
            goal_y=access.access.access_y,
            planner_name=usv_path_planner,
            task_id=None,
            stats_context="runtime_return_to_patrol",
            traffic_cost_context=traffic_cost_context,
        )
        candidate_state = replace(
            execution_state,
            active_plan=plan,
            current_waypoint_index=0,
            patrol_waypoint_index=access.access.segment_end_index,
            return_target_x=access.access.access_x,
            return_target_y=access.access.access_y,
            return_target_source=access.source,
            rejoin_to_segment=True,
            last_return_plan_step=step,
        )
        if plan.status == PathPlanStatus.PLANNED:
            return candidate_state
        if blocked_state is None and not allow_patrol_index_advance:
            blocked_state = candidate_state

    return blocked_state or replace(
        execution_state,
        active_plan=None,
        current_waypoint_index=0,
        last_return_plan_step=step,
    )


def _build_planned_local_patrol_return_transition(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    patrol_route: tuple[tuple[float, float], ...],
    grid_map: GridMap,
    obstacle_layout: ObstacleLayout | None,
    info_map: InformationMap | None,
    task_records: tuple[TaskRecord, ...],
    skip_blocked_goal: bool,
    step: int,
    usv_path_planner: str,
    traffic_cost_context: TrafficCostContext | None = None,
) -> AgentExecutionState:
    transitioned_state = _build_local_patrol_return_transition(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
        patrol_route=patrol_route,
        grid_map=grid_map,
        info_map=info_map,
        task_records=task_records,
        skip_blocked_goal=skip_blocked_goal,
    )
    return _plan_return_to_patrol_path(
        agent,
        execution_state=transitioned_state,
        progress_state=progress_state,
        patrol_route=patrol_route,
        grid_map=grid_map,
        obstacle_layout=obstacle_layout,
        info_map=info_map,
        allow_patrol_index_advance=False,
        step=step,
        task_records=task_records,
        usv_path_planner=usv_path_planner,
        traffic_cost_context=traffic_cost_context,
    )
