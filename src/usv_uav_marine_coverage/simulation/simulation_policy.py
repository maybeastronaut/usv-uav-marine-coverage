"""Preview-policy helpers for the replay-style simulation stage."""

from __future__ import annotations

from math import hypot

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    AgentTaskState,
    TaskMode,
    default_coverage_radius,
)
from usv_uav_marine_coverage.agent_overlay import build_demo_agents
from usv_uav_marine_coverage.environment import build_default_sea_map
from usv_uav_marine_coverage.information_map import HotspotKnowledgeState, InformationMap
from usv_uav_marine_coverage.planning.uav_lawnmower_planner import build_lawnmower_route
from usv_uav_marine_coverage.planning.usv_patrol_planner import (
    build_default_usv_patrol_routes,
)


def build_demo_agent_states() -> tuple[AgentState, ...]:
    """Build the fixed preview agents used by the replay simulation."""

    states: list[AgentState] = []
    for visual in build_demo_agents():
        states.append(
            AgentState(
                agent_id=visual.agent_id,
                kind=visual.kind,
                x=visual.x,
                y=visual.y,
                heading_deg=visual.heading_deg,
                speed_mps=0.0,
                max_speed_mps=0.0,
                detection_radius=120.0 if visual.kind == "UAV" else 70.0,
                coverage_radius=default_coverage_radius(visual.kind),
                task=AgentTaskState(),
            )
        )
    return tuple(states)


def build_patrol_routes() -> dict[str, tuple[tuple[float, float], ...]]:
    """Return the preview patrol routes assembled from planning-layer builders."""

    sea_map = build_default_sea_map()
    offshore_left = sea_map.offshore.x_start + 35.0
    offshore_right = sea_map.offshore.x_end - 60.0
    upper_route = build_lawnmower_route(
        min_x=offshore_left,
        max_x=offshore_right,
        min_y=120.0,
        max_y=460.0,
        lane_spacing=170.0,
    )
    lower_route = build_lawnmower_route(
        min_x=offshore_left,
        max_x=offshore_right,
        min_y=540.0,
        max_y=880.0,
        lane_spacing=170.0,
    )
    patrol_routes = dict(build_default_usv_patrol_routes(sea_map))
    patrol_routes.update(
        {
            "UAV-1": upper_route,
            "UAV-2": lower_route,
        }
    )
    return patrol_routes


def assign_demo_tasks(
    *,
    agents: tuple[AgentState, ...],
    grid_map,
    info_map: InformationMap,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    patrol_indices: dict[str, int],
) -> tuple[tuple[AgentState, ...], tuple[dict[str, object], ...]]:
    """Assign preview tasks and emit the matching decision summary."""

    assigned_agents: list[AgentState] = []
    decisions: list[dict[str, object]] = []
    suspected_targets = _suspected_target_points(grid_map, info_map)
    usv_agents = tuple(agent for agent in agents if agent.kind == "USV")

    for agent in agents:
        if agent.kind == "USV":
            suspected_target = _nearest_target_point(agent, suspected_targets)
            if suspected_target is not None:
                updated_agent = ensure_task(
                    agent,
                    TaskMode.CONFIRM,
                    suspected_target[0],
                    suspected_target[1],
                )
                assigned_agents.append(updated_agent)
                decisions.append(
                    _build_usv_task_decision(
                        agent=agent,
                        selected_agent=updated_agent,
                        target=suspected_target,
                        candidates=usv_agents,
                    )
                )
                continue

        waypoint = patrol_routes[agent.agent_id][patrol_indices[agent.agent_id]]
        mode = TaskMode.INVESTIGATE if agent.kind == "UAV" else TaskMode.PATROL
        updated_agent = ensure_task(agent, mode, waypoint[0], waypoint[1])
        assigned_agents.append(updated_agent)
        decisions.append(
            _build_patrol_task_decision(
                agent=agent,
                selected_agent=updated_agent,
                waypoint=waypoint,
                patrol_index=patrol_indices[agent.agent_id],
            )
        )

    return (tuple(assigned_agents), tuple(decisions))


def ensure_task(agent: AgentState, mode: TaskMode, target_x: float, target_y: float) -> AgentState:
    """Keep the existing task if unchanged, otherwise return an updated agent state."""

    if (
        agent.task.mode == mode
        and agent.task.target_x == target_x
        and agent.task.target_y == target_y
    ):
        return agent
    return AgentState(
        agent_id=agent.agent_id,
        kind=agent.kind,
        x=agent.x,
        y=agent.y,
        heading_deg=agent.heading_deg,
        speed_mps=agent.speed_mps,
        max_speed_mps=agent.max_speed_mps,
        detection_radius=agent.detection_radius,
        coverage_radius=agent.coverage_radius,
        task=AgentTaskState(mode=mode, target_x=target_x, target_y=target_y),
        turn_rate_degps=agent.turn_rate_degps,
        max_acceleration_mps2=agent.max_acceleration_mps2,
        max_deceleration_mps2=agent.max_deceleration_mps2,
        max_turn_rate_degps=agent.max_turn_rate_degps,
        cruise_speed_mps=agent.cruise_speed_mps,
        arrival_tolerance_m=agent.arrival_tolerance_m,
        platform_profile=agent.platform_profile,
    )


def _build_usv_task_decision(
    *,
    agent: AgentState,
    selected_agent: AgentState,
    target: tuple[float, float],
    candidates: tuple[AgentState, ...],
) -> dict[str, object]:
    candidate_distances = sorted(
        (
            {
                "agent_id": candidate.agent_id,
                "distance_to_target": round(
                    hypot(candidate.x - target[0], candidate.y - target[1]),
                    3,
                ),
            }
            for candidate in candidates
        ),
        key=lambda item: item["distance_to_target"],
    )
    selected_distance = round(hypot(agent.x - target[0], agent.y - target[1]), 3)
    return {
        "task_id": f"confirm-{int(target[0])}-{int(target[1])}",
        "task_type": "hotspot_confirmation",
        "selected_agent": selected_agent.agent_id,
        "candidate_agents": candidate_distances,
        "selection_reason": "nearest_suspected_hotspot_under_usv_confirmation_heuristic",
        "selection_score": selected_distance,
        "target_x": round(target[0], 3),
        "target_y": round(target[1], 3),
    }


def _build_patrol_task_decision(
    *,
    agent: AgentState,
    selected_agent: AgentState,
    waypoint: tuple[float, float],
    patrol_index: int,
) -> dict[str, object]:
    return {
        "task_id": f"{selected_agent.agent_id}-patrol-{patrol_index}",
        "task_type": (
            "patrol_waypoint_tracking" if agent.kind == "USV" else "uav_investigation_patrol"
        ),
        "selected_agent": selected_agent.agent_id,
        "candidate_agents": [{"agent_id": selected_agent.agent_id}],
        "selection_reason": "continue_assigned_patrol_route",
        "selection_score": 1.0,
        "target_x": round(waypoint[0], 3),
        "target_y": round(waypoint[1], 3),
    }


def _suspected_target_points(grid_map, info_map: InformationMap) -> tuple[tuple[float, float], ...]:
    targets: list[tuple[float, float]] = []
    for cell in grid_map.flat_cells:
        state = info_map.state_at(cell.row, cell.col)
        if state.known_hotspot_state == HotspotKnowledgeState.SUSPECTED:
            targets.append((cell.center_x, cell.center_y))
    return tuple(targets)


def _nearest_target_point(
    agent: AgentState,
    targets: tuple[tuple[float, float], ...],
) -> tuple[float, float] | None:
    if not targets:
        return None
    return min(targets, key=lambda point: (agent.x - point[0]) ** 2 + (agent.y - point[1]) ** 2)
