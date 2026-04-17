"""Traffic-coordination helpers for execution-time USV movement."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import hypot

from usv_uav_marine_coverage.agent_model import (
    USV_COLLISION_CLEARANCE_M,
    AgentState,
    distance_to_point,
    is_operational_agent,
)
from usv_uav_marine_coverage.environment import ObstacleLayout
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType

from .basic_state_machine import transition_from_yield, transition_to_yield
from .execution_types import AgentExecutionState, AgentProgressState, ExecutionStage

CORRIDOR_APPROACH_DISTANCE_M = 85.0
CORRIDOR_HOLD_OFFSET_M = 28.0
CORRIDOR_CONVOY_MIN_SPACING_M = 72.0
CORRIDOR_CONVOY_MIN_OWNER_SPEED_MPS = 2.0
CORRIDOR_OWNER_STALL_RELEASE_STEPS = 2
CORRIDOR_OWNER_MIN_PROGRESS_SPEED_MPS = 0.5
DYNAMIC_BOTTLENECK_LOOKAHEAD_POINTS = 3
DYNAMIC_BOTTLENECK_SAMPLE_SPACING_M = 22.0
DYNAMIC_BOTTLENECK_MAX_SAMPLES = 8
DYNAMIC_BOTTLENECK_CONFLICT_DISTANCE_M = 52.0
DYNAMIC_BOTTLENECK_PAIR_DISTANCE_M = 130.0
DYNAMIC_BOTTLENECK_EDGE_CLEARANCE_M = 180.0
DYNAMIC_BOTTLENECK_HAZARD_CLEARANCE_M = 95.0
DYNAMIC_BOTTLENECK_HOLD_OFFSET_M = 26.0
DYNAMIC_BOTTLENECK_DIRECTIVE_TTL_STEPS = 6
CORRIDOR_DIRECTIVE_STICKY_STEPS = 4


@dataclass(frozen=True)
class CorridorDirective:
    """One per-agent corridor coordination decision for the current step."""

    corridor_name: str
    owner_agent_id: str
    should_yield: bool
    hold_x: float
    hold_y: float
    reason: str


@dataclass(frozen=True)
class BottleneckDirective:
    """One per-agent dynamic bottleneck coordination decision for the current step."""

    zone_id: str
    owner_agent_id: str
    should_yield: bool
    hold_x: float
    hold_y: float
    reason: str
    expires_after_step: int


def apply_corridor_directive(
    execution_state: AgentExecutionState,
    *,
    directive: CorridorDirective | None,
    step: int,
    owner_agent: AgentState | None = None,
    owner_execution_state: AgentExecutionState | None = None,
    owner_progress_state: AgentProgressState | None = None,
) -> AgentExecutionState:
    """Apply the corridor reservation decision for one execution state."""

    if directive is None:
        if (
            execution_state.reserved_corridor_name is not None
            and not _should_release_sticky_corridor_reservation(
                execution_state,
                owner_agent=owner_agent,
                owner_execution_state=owner_execution_state,
                owner_progress_state=owner_progress_state,
            )
            and (
                step <= execution_state.corridor_reservation_until_step
                or _should_extend_corridor_wait(
                    execution_state,
                    owner_execution_state=owner_execution_state,
                )
            )
        ):
            return execution_state
        if is_corridor_yield_state(execution_state):
            return transition_from_yield(execution_state)
        if (
            execution_state.reserved_corridor_name is None
            and execution_state.corridor_owner_agent_id is None
        ):
            return execution_state
        return replace(
            execution_state,
            reserved_corridor_name=None,
            corridor_owner_agent_id=None,
            corridor_reservation_until_step=-1,
        )

    if directive.should_yield:
        return replace(
            transition_to_yield(
                execution_state,
                yield_target_x=directive.hold_x,
                yield_target_y=directive.hold_y,
                yield_reason=directive.reason,
                reserved_corridor_name=directive.corridor_name,
                corridor_owner_agent_id=directive.owner_agent_id,
            ),
            corridor_reservation_until_step=step + CORRIDOR_DIRECTIVE_STICKY_STEPS,
        )

    base_state = (
        transition_from_yield(execution_state)
        if execution_state.stage == ExecutionStage.YIELD
        else execution_state
    )
    return replace(
        base_state,
        reserved_corridor_name=directive.corridor_name,
        corridor_owner_agent_id=directive.owner_agent_id,
        corridor_reservation_until_step=step + CORRIDOR_DIRECTIVE_STICKY_STEPS,
    )


def apply_bottleneck_directive(
    execution_state: AgentExecutionState,
    *,
    directive: BottleneckDirective | None,
    step: int,
) -> AgentExecutionState:
    """Apply one dynamic bottleneck coordination decision."""

    if directive is None:
        if (
            execution_state.reserved_bottleneck_zone_id is not None
            and step <= execution_state.bottleneck_reservation_until_step
        ):
            return execution_state
        if is_bottleneck_yield_state(execution_state):
            return transition_from_yield(execution_state)
        if (
            execution_state.reserved_bottleneck_zone_id is None
            and execution_state.bottleneck_owner_agent_id is None
        ):
            return execution_state
        return replace(
            execution_state,
            reserved_bottleneck_zone_id=None,
            bottleneck_owner_agent_id=None,
            bottleneck_reservation_until_step=-1,
        )

    if directive.should_yield:
        return replace(
            transition_to_yield(
                execution_state,
                yield_target_x=directive.hold_x,
                yield_target_y=directive.hold_y,
                yield_reason=directive.reason,
                reserved_bottleneck_zone_id=directive.zone_id,
                bottleneck_owner_agent_id=directive.owner_agent_id,
            ),
            bottleneck_reservation_until_step=directive.expires_after_step,
        )

    base_state = (
        transition_from_yield(execution_state)
        if is_bottleneck_yield_state(execution_state)
        else execution_state
    )
    return replace(
        base_state,
        reserved_bottleneck_zone_id=directive.zone_id,
        bottleneck_owner_agent_id=directive.owner_agent_id,
        bottleneck_reservation_until_step=directive.expires_after_step,
    )


def build_corridor_directives(
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState] | None = None,
    obstacle_layout: ObstacleLayout | None,
) -> dict[str, CorridorDirective]:
    """Build fixed-corridor traffic directives for the current step."""

    if obstacle_layout is None or not obstacle_layout.traversable_corridors:
        return {}

    agent_by_id = {agent.agent_id: agent for agent in agents}
    directives: dict[str, CorridorDirective] = {}
    for corridor in obstacle_layout.traversable_corridors:
        candidates: list[tuple[int, int, bool, float, str, float, float, int]] = []
        for agent in agents:
            if agent.kind != "USV" or not is_operational_agent(agent):
                continue
            execution_state = execution_states[agent.agent_id]
            if execution_state.stage not in {
                ExecutionStage.GO_TO_TASK,
                ExecutionStage.RETURN_TO_PATROL,
                ExecutionStage.PATROL,
                ExecutionStage.YIELD,
            }:
                continue
            plan = execution_state.active_plan
            if plan is None or not plan.waypoints:
                continue
            if not _plan_uses_corridor(plan, corridor):
                continue
            if not _agent_is_near_corridor(agent, corridor):
                continue
            entry_x, entry_y, exit_x, exit_y = _corridor_entry_and_exit(agent, corridor)
            hold_x, hold_y = _corridor_hold_point(entry_x, entry_y, exit_x)
            direction = _corridor_travel_direction(agent, execution_state, corridor)
            owner_penalty = _corridor_owner_penalty(
                agent,
                execution_state=execution_state,
                progress_state=(
                    None if progress_states is None else progress_states.get(agent.agent_id)
                ),
            )
            task_priority_rank = _traffic_task_priority_rank(
                execution_state=execution_state,
                active_task=None,
            )
            candidates.append(
                (
                    task_priority_rank,
                    owner_penalty,
                    _point_is_inside_corridor(agent.x, agent.y, corridor),
                    hypot(agent.x - exit_x, agent.y - exit_y),
                    agent.agent_id,
                    hold_x,
                    hold_y,
                    direction,
                )
            )

        if len(candidates) <= 1:
            if len(candidates) == 1:
                _, _, _, _, owner_agent_id, _, _, _ = sorted(
                    candidates,
                    key=lambda item: (item[0], item[1], -int(item[2]), item[3], item[4]),
                )[0]
                directives[owner_agent_id] = CorridorDirective(
                    corridor_name=corridor.name,
                    owner_agent_id=owner_agent_id,
                    should_yield=False,
                    hold_x=0.0,
                    hold_y=0.0,
                    reason="corridor_owner",
                )
            continue

        ordered_candidates = sorted(
            candidates,
            key=lambda item: (item[0], item[1], -int(item[2]), item[3], item[4]),
        )
        _, _, _, _, owner_agent_id, _, _, owner_direction = ordered_candidates[0]
        owner_agent = agent_by_id[owner_agent_id]
        directives[owner_agent_id] = CorridorDirective(
            corridor_name=corridor.name,
            owner_agent_id=owner_agent_id,
            should_yield=False,
            hold_x=0.0,
            hold_y=0.0,
            reason="corridor_owner",
        )
        for _, _, _, _, agent_id, hold_x, hold_y, direction in ordered_candidates[1:]:
            follower_agent = agent_by_id[agent_id]
            follower_inside_corridor = _point_is_inside_corridor(
                follower_agent.x,
                follower_agent.y,
                corridor,
            )
            if _can_follow_corridor_owner(
                follower_agent=follower_agent,
                owner_agent=owner_agent,
                follower_direction=direction,
                owner_direction=owner_direction,
                follower_inside_corridor=follower_inside_corridor,
            ):
                directives[agent_id] = CorridorDirective(
                    corridor_name=corridor.name,
                    owner_agent_id=owner_agent_id,
                    should_yield=False,
                    hold_x=0.0,
                    hold_y=0.0,
                    reason="corridor_follow_owner",
                )
                continue
            directives[agent_id] = CorridorDirective(
                corridor_name=corridor.name,
                owner_agent_id=owner_agent_id,
                should_yield=True,
                hold_x=hold_x,
                hold_y=hold_y,
                reason="corridor_wait_for_owner",
            )
    return directives


def build_dynamic_bottleneck_directives(
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    active_tasks_by_agent: dict[str, TaskRecord | None],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    obstacle_layout: ObstacleLayout | None,
    corridor_directives: dict[str, CorridorDirective],
    step: int,
) -> dict[str, BottleneckDirective]:
    """Build dynamic narrow-area traffic directives for the current step."""

    eligible_agents = [
        agent
        for agent in agents
        if (
            agent.kind == "USV"
            and is_operational_agent(agent)
            and agent.agent_id not in corridor_directives
            and execution_states[agent.agent_id].stage
            in {
                ExecutionStage.GO_TO_TASK,
                ExecutionStage.RECOVERY,
                ExecutionStage.RETURN_TO_PATROL,
                ExecutionStage.PATROL,
                ExecutionStage.YIELD,
            }
        )
    ]
    if len(eligible_agents) < 2:
        return {}

    directives: dict[str, BottleneckDirective] = {}
    reserved_agent_ids: set[str] = set()

    for index, first_agent in enumerate(eligible_agents[:-1]):
        if first_agent.agent_id in reserved_agent_ids:
            continue
        first_state = execution_states[first_agent.agent_id]
        first_route = build_progress_target_route(
            first_agent,
            execution_state=first_state,
            active_task=active_tasks_by_agent[first_agent.agent_id],
            patrol_route=patrol_routes[first_agent.agent_id],
        )
        if len(first_route) < 2:
            continue
        for second_agent in eligible_agents[index + 1 :]:
            if second_agent.agent_id in reserved_agent_ids:
                continue
            if (
                distance_to_point(first_agent, second_agent.x, second_agent.y)
                > DYNAMIC_BOTTLENECK_PAIR_DISTANCE_M
            ):
                continue
            second_state = execution_states[second_agent.agent_id]
            second_route = build_progress_target_route(
                second_agent,
                execution_state=second_state,
                active_task=active_tasks_by_agent[second_agent.agent_id],
                patrol_route=patrol_routes[second_agent.agent_id],
            )
            if len(second_route) < 2:
                continue
            conflict = _detect_dynamic_bottleneck_conflict(
                first_route=first_route,
                second_route=second_route,
                obstacle_layout=obstacle_layout,
            )
            if conflict is None:
                continue

            owner_agent_id, yield_agent_id = _choose_dynamic_bottleneck_owner(
                first_agent,
                second_agent,
                first_state=first_state,
                second_state=second_state,
                first_active_task=active_tasks_by_agent[first_agent.agent_id],
                second_active_task=active_tasks_by_agent[second_agent.agent_id],
                first_patrol_route=patrol_routes[first_agent.agent_id],
                second_patrol_route=patrol_routes[second_agent.agent_id],
                center_x=conflict[0],
                center_y=conflict[1],
            )
            yield_agent = first_agent if yield_agent_id == first_agent.agent_id else second_agent
            hold_x, hold_y = _dynamic_bottleneck_hold_point(
                yield_agent,
                center_x=conflict[0],
                center_y=conflict[1],
                obstacle_layout=obstacle_layout,
            )
            zone_id = (
                f"bottleneck-{int(conflict[0] // 25)}-{int(conflict[1] // 25)}-"
                f"{'-'.join(sorted((first_agent.agent_id, second_agent.agent_id)))}"
            )
            directives[owner_agent_id] = BottleneckDirective(
                zone_id=zone_id,
                owner_agent_id=owner_agent_id,
                should_yield=False,
                hold_x=0.0,
                hold_y=0.0,
                reason="dynamic_bottleneck_owner",
                expires_after_step=step + DYNAMIC_BOTTLENECK_DIRECTIVE_TTL_STEPS,
            )
            directives[yield_agent_id] = BottleneckDirective(
                zone_id=zone_id,
                owner_agent_id=owner_agent_id,
                should_yield=True,
                hold_x=hold_x,
                hold_y=hold_y,
                reason="dynamic_bottleneck_wait_for_owner",
                expires_after_step=step + DYNAMIC_BOTTLENECK_DIRECTIVE_TTL_STEPS,
            )
            reserved_agent_ids.add(owner_agent_id)
            reserved_agent_ids.add(yield_agent_id)
            break

    return directives


def build_progress_target_route(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord | None,
    patrol_route: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    """Build a short future route used by traffic-coordination logic."""

    route: list[tuple[float, float]] = [(agent.x, agent.y)]
    plan = execution_state.active_plan
    if plan is not None and plan.waypoints:
        for waypoint in plan.waypoints[
            execution_state.current_waypoint_index : execution_state.current_waypoint_index
            + DYNAMIC_BOTTLENECK_LOOKAHEAD_POINTS
        ]:
            route.append((waypoint.x, waypoint.y))

    target_x, target_y = progress_target(
        agent,
        execution_state=execution_state,
        active_task=active_task,
        patrol_route=patrol_route,
    )
    if (
        target_x is not None
        and target_y is not None
        and (not route or hypot(route[-1][0] - target_x, route[-1][1] - target_y) > 1e-6)
    ):
        route.append((target_x, target_y))
    return tuple(route)


def progress_target(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord | None,
    patrol_route: tuple[tuple[float, float], ...],
) -> tuple[float | None, float | None]:
    """Resolve the current execution-time progress target for one agent."""
    plan = execution_state.active_plan
    if plan is not None and execution_state.current_waypoint_index < len(plan.waypoints):
        waypoint = plan.waypoints[execution_state.current_waypoint_index]
        return (waypoint.x, waypoint.y)
    if execution_state.stage in {ExecutionStage.GO_TO_TASK, ExecutionStage.RECOVERY}:
        if agent.task.target_x is not None and agent.task.target_y is not None:
            return (agent.task.target_x, agent.task.target_y)
        if active_task is not None:
            return (active_task.target_x, active_task.target_y)
    if execution_state.stage == ExecutionStage.RETURN_TO_PATROL:
        return (execution_state.return_target_x, execution_state.return_target_y)
    if execution_state.stage == ExecutionStage.YIELD:
        return (execution_state.yield_target_x, execution_state.yield_target_y)
    if execution_state.stage == ExecutionStage.PATROL and patrol_route:
        target_x, target_y = patrol_route[execution_state.patrol_waypoint_index]
        return (target_x, target_y)
    return (None, None)


def is_corridor_yield_state(execution_state: AgentExecutionState) -> bool:
    """Return whether this execution state is a corridor-yield hold."""

    return (
        execution_state.stage == ExecutionStage.YIELD
        and execution_state.reserved_corridor_name is not None
    )


def is_bottleneck_yield_state(execution_state: AgentExecutionState) -> bool:
    """Return whether this execution state is a bottleneck-yield hold."""

    return (
        execution_state.stage == ExecutionStage.YIELD
        and execution_state.reserved_bottleneck_zone_id is not None
    )


def _plan_uses_corridor(plan, corridor) -> bool:
    for waypoint in plan.waypoints:
        if _point_is_inside_corridor(waypoint.x, waypoint.y, corridor):
            return True
    return False


def _agent_is_near_corridor(agent: AgentState, corridor) -> bool:
    return _distance_to_corridor(agent.x, agent.y, corridor) <= CORRIDOR_APPROACH_DISTANCE_M


def _point_is_inside_corridor(x: float, y: float, corridor) -> bool:
    return _distance_to_corridor(x, y, corridor) <= corridor.width / 2.0


def _distance_to_corridor(x: float, y: float, corridor) -> float:
    best_distance = float("inf")
    control_points = corridor.control_points
    for start, end in zip(control_points, control_points[1:], strict=False):
        best_distance = min(best_distance, _distance_to_segment(x, y, start, end))
    return best_distance


def _corridor_entry_and_exit(agent: AgentState, corridor) -> tuple[float, float, float, float]:
    first_x, first_y = corridor.control_points[0]
    last_x, last_y = corridor.control_points[-1]
    first_distance = hypot(agent.x - first_x, agent.y - first_y)
    last_distance = hypot(agent.x - last_x, agent.y - last_y)
    if first_distance <= last_distance:
        return (first_x, first_y, last_x, last_y)
    return (last_x, last_y, first_x, first_y)


def _corridor_hold_point(
    entry_x: float,
    entry_y: float,
    exit_x: float,
) -> tuple[float, float]:
    offset = -CORRIDOR_HOLD_OFFSET_M if entry_x < exit_x else CORRIDOR_HOLD_OFFSET_M
    return (entry_x + offset, entry_y)


def _corridor_travel_direction(
    agent: AgentState,
    execution_state: AgentExecutionState,
    corridor,
) -> int:
    plan = execution_state.active_plan
    if plan is not None and plan.waypoints:
        corridor_waypoints = [
            waypoint
            for waypoint in plan.waypoints[execution_state.current_waypoint_index :]
            if _point_is_inside_corridor(waypoint.x, waypoint.y, corridor)
        ]
        if len(corridor_waypoints) >= 2:
            delta_x = corridor_waypoints[-1].x - corridor_waypoints[0].x
            if abs(delta_x) > 1e-6:
                return 1 if delta_x > 0.0 else -1
        if corridor_waypoints:
            delta_x = plan.goal_x - corridor_waypoints[0].x
            if abs(delta_x) > 1e-6:
                return 1 if delta_x > 0.0 else -1
    return 1 if corridor.control_points[-1][0] >= agent.x else -1


def _can_follow_corridor_owner(
    *,
    follower_agent: AgentState,
    owner_agent: AgentState,
    follower_direction: int,
    owner_direction: int,
    follower_inside_corridor: bool,
) -> bool:
    if follower_direction == 0 or owner_direction == 0:
        return False
    if follower_direction != owner_direction:
        return False
    if not follower_inside_corridor:
        return False
    if owner_agent.speed_mps < CORRIDOR_CONVOY_MIN_OWNER_SPEED_MPS:
        return False
    longitudinal_gap = (owner_agent.x - follower_agent.x) * owner_direction
    return longitudinal_gap >= CORRIDOR_CONVOY_MIN_SPACING_M


def _should_release_sticky_corridor_reservation(
    execution_state: AgentExecutionState,
    *,
    owner_agent: AgentState | None,
    owner_execution_state: AgentExecutionState | None,
    owner_progress_state: AgentProgressState | None,
) -> bool:
    owner_agent_id = execution_state.corridor_owner_agent_id
    if owner_agent_id is None:
        return False
    if owner_agent is None or not is_operational_agent(owner_agent):
        return True
    if owner_execution_state is None:
        return True
    if owner_execution_state.stage in {
        ExecutionStage.RECOVERY,
        ExecutionStage.FAILED,
    }:
        return True
    if _is_low_progress_corridor_owner(
        owner_agent,
        execution_state=owner_execution_state,
        progress_state=owner_progress_state,
    ):
        return True
    return False


def _should_extend_corridor_wait(
    execution_state: AgentExecutionState,
    *,
    owner_execution_state: AgentExecutionState | None,
) -> bool:
    if not is_corridor_yield_state(execution_state):
        return False
    if owner_execution_state is None:
        return False
    if execution_state.reserved_corridor_name is None:
        return False
    return owner_execution_state.reserved_corridor_name == execution_state.reserved_corridor_name


def _corridor_owner_penalty(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState | None,
) -> int:
    if _is_low_progress_corridor_owner(
        agent,
        execution_state=execution_state,
        progress_state=progress_state,
    ):
        return 1
    return 0


def _traffic_task_priority_rank(
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord | None,
) -> int:
    if active_task is not None and active_task.task_type == TaskType.UAV_RESUPPLY:
        return 0
    active_task_id = execution_state.active_task_id
    if active_task_id is not None and active_task_id.startswith("uav-resupply-"):
        return 0
    if active_task is not None or active_task_id is not None:
        return 1
    return 2


def _is_low_progress_corridor_owner(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState | None,
) -> bool:
    if execution_state.stage in {ExecutionStage.RECOVERY, ExecutionStage.FAILED}:
        return True
    if (
        progress_state is not None
        and progress_state.stalled_steps >= CORRIDOR_OWNER_STALL_RELEASE_STEPS
    ):
        return True
    return (
        agent.speed_mps < CORRIDOR_OWNER_MIN_PROGRESS_SPEED_MPS
        and progress_state is not None
        and progress_state.stalled_steps > 0
    )


def _detect_dynamic_bottleneck_conflict(
    *,
    first_route: tuple[tuple[float, float], ...],
    second_route: tuple[tuple[float, float], ...],
    obstacle_layout: ObstacleLayout | None,
) -> tuple[float, float] | None:
    first_samples = _sample_reference_route(first_route)
    second_samples = _sample_reference_route(second_route)
    if not first_samples or not second_samples:
        return None

    best_distance = float("inf")
    conflict_center: tuple[float, float] | None = None
    for first_sample in first_samples:
        for second_sample in second_samples:
            distance = hypot(
                first_sample[0] - second_sample[0],
                first_sample[1] - second_sample[1],
            )
            if distance >= best_distance:
                continue
            best_distance = distance
            conflict_center = (
                (first_sample[0] + second_sample[0]) / 2.0,
                (first_sample[1] + second_sample[1]) / 2.0,
            )

    if (
        conflict_center is None
        or best_distance > DYNAMIC_BOTTLENECK_CONFLICT_DISTANCE_M
        or not _conflict_center_is_narrow(conflict_center[0], conflict_center[1], obstacle_layout)
    ):
        return None
    return conflict_center


def _sample_reference_route(
    route: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    if len(route) < 2:
        return route
    samples: list[tuple[float, float]] = [route[0]]
    for start, end in zip(route, route[1:], strict=False):
        segment_length = hypot(end[0] - start[0], end[1] - start[1])
        if segment_length <= 1e-6:
            continue
        sample_count = max(
            1,
            min(
                DYNAMIC_BOTTLENECK_MAX_SAMPLES,
                int(segment_length / DYNAMIC_BOTTLENECK_SAMPLE_SPACING_M),
            ),
        )
        for index in range(1, sample_count + 1):
            ratio = index / sample_count
            samples.append(
                (
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                )
            )
            if len(samples) >= DYNAMIC_BOTTLENECK_MAX_SAMPLES:
                return tuple(samples)
    return tuple(samples)


def _conflict_center_is_narrow(
    x: float,
    y: float,
    obstacle_layout: ObstacleLayout | None,
) -> bool:
    edge_clearance = min(x, y, 1000.0 - x, 1000.0 - y)
    if edge_clearance < DYNAMIC_BOTTLENECK_EDGE_CLEARANCE_M:
        return True
    if obstacle_layout is None:
        return False
    return (
        _clearance_from_point_to_static_hazards(x, y, obstacle_layout)
        < DYNAMIC_BOTTLENECK_HAZARD_CLEARANCE_M
    )


def _clearance_from_point_to_static_hazards(
    x: float,
    y: float,
    obstacle_layout: ObstacleLayout,
) -> float:
    best_clearance = float("inf")
    for obstacle in obstacle_layout.risk_zone_obstacles:
        if _point_in_polygon(x, y, obstacle.points):
            return 0.0
        best_clearance = min(best_clearance, _distance_to_polygon_edges(x, y, obstacle.points))
    for feature in obstacle_layout.offshore_features:
        clearance = hypot(x - feature.x, y - feature.y) - feature.radius
        best_clearance = min(best_clearance, clearance)
    return best_clearance


def _choose_dynamic_bottleneck_owner(
    first_agent: AgentState,
    second_agent: AgentState,
    *,
    first_state: AgentExecutionState,
    second_state: AgentExecutionState,
    first_active_task: TaskRecord | None,
    second_active_task: TaskRecord | None,
    first_patrol_route: tuple[tuple[float, float], ...],
    second_patrol_route: tuple[tuple[float, float], ...],
    center_x: float,
    center_y: float,
) -> tuple[str, str]:
    existing_owner_agent_id = _existing_bottleneck_owner_agent_id(
        first_state,
        second_state,
        candidate_agent_ids=(first_agent.agent_id, second_agent.agent_id),
    )
    if existing_owner_agent_id is not None:
        if existing_owner_agent_id == first_agent.agent_id:
            return (first_agent.agent_id, second_agent.agent_id)
        return (second_agent.agent_id, first_agent.agent_id)

    first_target_x, first_target_y = progress_target(
        first_agent,
        execution_state=first_state,
        active_task=first_active_task,
        patrol_route=first_patrol_route,
    )
    second_target_x, second_target_y = progress_target(
        second_agent,
        execution_state=second_state,
        active_task=second_active_task,
        patrol_route=second_patrol_route,
    )

    def _priority(
        agent: AgentState,
        execution_state: AgentExecutionState,
        active_task: TaskRecord | None,
        target_x: float | None,
        target_y: float | None,
    ) -> tuple[int, float, float, str]:
        target_distance = float("inf")
        if target_x is not None and target_y is not None:
            target_distance = hypot(agent.x - target_x, agent.y - target_y)
        return (
            _traffic_task_priority_rank(
                execution_state=execution_state,
                active_task=active_task,
            ),
            hypot(agent.x - center_x, agent.y - center_y),
            target_distance,
            agent.agent_id,
        )

    first_priority = _priority(
        first_agent,
        first_state,
        first_active_task,
        first_target_x,
        first_target_y,
    )
    second_priority = _priority(
        second_agent,
        second_state,
        second_active_task,
        second_target_x,
        second_target_y,
    )
    if first_priority <= second_priority:
        return (first_agent.agent_id, second_agent.agent_id)
    return (second_agent.agent_id, first_agent.agent_id)


def _existing_bottleneck_owner_agent_id(
    first_state: AgentExecutionState,
    second_state: AgentExecutionState,
    *,
    candidate_agent_ids: tuple[str, str],
) -> str | None:
    for state in (first_state, second_state):
        owner_agent_id = state.bottleneck_owner_agent_id
        if owner_agent_id in candidate_agent_ids and state.reserved_bottleneck_zone_id is not None:
            return owner_agent_id
    return None


def _dynamic_bottleneck_hold_point(
    agent: AgentState,
    *,
    center_x: float,
    center_y: float,
    obstacle_layout: ObstacleLayout | None,
) -> tuple[float, float]:
    offset_x = agent.x - center_x
    offset_y = agent.y - center_y
    offset_norm = hypot(offset_x, offset_y)
    if offset_norm <= 1e-6:
        return (agent.x, agent.y)
    hold_x = agent.x + (offset_x / offset_norm) * DYNAMIC_BOTTLENECK_HOLD_OFFSET_M
    hold_y = agent.y + (offset_y / offset_norm) * DYNAMIC_BOTTLENECK_HOLD_OFFSET_M
    if hold_x < 0.0 or hold_x > 1000.0 or hold_y < 0.0 or hold_y > 1000.0:
        return (agent.x, agent.y)
    if (
        obstacle_layout is not None
        and _clearance_from_point_to_static_hazards(hold_x, hold_y, obstacle_layout)
        < USV_COLLISION_CLEARANCE_M
    ):
        return (agent.x, agent.y)
    return (hold_x, hold_y)


def _point_in_polygon(
    x: float,
    y: float,
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    inside = False
    last_index = len(polygon) - 1
    for index, current in enumerate(polygon):
        previous = polygon[last_index if index == 0 else index - 1]
        x1, y1 = current
        x2, y2 = previous
        intersects = ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-9) + x1
        )
        if intersects:
            inside = not inside
    return inside


def _distance_to_polygon_edges(
    x: float,
    y: float,
    polygon: tuple[tuple[float, float], ...],
) -> float:
    best_distance = float("inf")
    for start, end in zip(polygon, polygon[1:] + polygon[:1], strict=False):
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
        return hypot(x - start_x, y - start_y)
    projection = ((x - start_x) * delta_x + (y - start_y) * delta_y) / segment_length_sq
    projection = min(1.0, max(0.0, projection))
    closest_x = start_x + projection * delta_x
    closest_y = start_y + projection * delta_y
    return hypot(x - closest_x, y - closest_y)
