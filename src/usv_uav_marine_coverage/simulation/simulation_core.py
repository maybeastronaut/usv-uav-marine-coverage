"""Core replay-simulation flow for the current preview stage."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random

from usv_uav_marine_coverage.agent_model import AgentState, advance_agent_towards_task
from usv_uav_marine_coverage.agent_overlay import VisualAgent
from usv_uav_marine_coverage.environment import ObstacleLayout, SeaMap, build_default_sea_map, build_obstacle_layout
from usv_uav_marine_coverage.grid import (
    CoverageState,
    GridCoverageMap,
    apply_agent_coverage,
    build_grid_coverage_map,
    build_grid_map,
)
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    InformationMap,
    InformationMapConfig,
    advance_information_age,
    apply_uav_detection,
    apply_usv_confirmation,
    build_information_map,
    observe_cells,
    spawn_hotspots,
)
from .simulation_policy import (
    assign_demo_tasks,
    build_demo_agent_states,
    build_patrol_routes,
)


@dataclass(frozen=True)
class SimulationFrame:
    """One replay frame for the visualization preview."""

    step: int
    agents: tuple[VisualAgent, ...]
    valid_cells: tuple[tuple[int, int], ...]
    suspected_cells: tuple[tuple[int, int], ...]
    confirmed_cells: tuple[tuple[int, int], ...]
    false_alarm_cells: tuple[tuple[int, int], ...]
    covered_cells: tuple[tuple[int, int], ...]
    coverage_ratio: float
    events: tuple[str, ...]
    trajectories: tuple[tuple[str, tuple[tuple[float, float], ...]], ...]


@dataclass(frozen=True)
class SimulationReplay:
    """Replay data and static environment for the preview page."""

    sea_map: SeaMap
    obstacle_layout: ObstacleLayout
    frames: tuple[SimulationFrame, ...]
    seed: int
    dt_seconds: float
    initial_agents: tuple[AgentState, ...]
    step_logs: tuple[dict[str, object], ...]


def build_simulation_replay(
    sea_map: SeaMap | None = None,
    obstacle_layout: ObstacleLayout | None = None,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationReplay:
    """Build a simple replay preview on top of the current simulation baseline."""

    from .simulation_logging import build_step_log

    if steps <= 0:
        raise ValueError("Simulation steps must be positive.")
    if dt_seconds <= 0:
        raise ValueError("Simulation time step must be positive.")

    target_map = sea_map or build_default_sea_map()
    layout = obstacle_layout or build_obstacle_layout(target_map, seed=seed)
    grid_map = build_grid_map(target_map, layout)
    coverage_map = build_grid_coverage_map(grid_map)
    info_map = build_information_map(grid_map, InformationMapConfig())
    rng = Random(layout.seed + 17)

    agents = build_demo_agent_states()
    initial_agents = agents
    patrol_routes = build_patrol_routes()
    patrol_indices = {agent.agent_id: 0 for agent in agents}
    trajectories = {
        agent.agent_id: [(agent.x, agent.y)]
        for agent in agents
    }
    step_logs: list[dict[str, object]] = []

    frames = [
        _capture_frame(
            step=0,
            agents=agents,
            info_map=info_map,
            coverage_map=coverage_map,
            trajectories=trajectories,
            events=(),
        )
    ]
    step_logs.append(
        build_step_log(
            step=0,
            agents=agents,
            info_map=info_map,
            coverage_map=coverage_map,
            events=(),
            observed_by_agent={},
            spawned_hotspots=(),
            newly_stale_cells=tuple(_collect_stale_cells(info_map)),
            detected_by_agent={},
            confirmations_by_agent={},
            false_alarms_by_agent={},
            task_decisions=(),
            prior_agents=agents,
            planned_agents=agents,
        )
    )

    for step in range(1, steps + 1):
        events: list[str] = []
        stale_before = set(_collect_stale_cells(info_map))
        advance_information_age(info_map, step)
        stale_after = set(_collect_stale_cells(info_map))
        newly_stale_cells = tuple(sorted(stale_after - stale_before))
        spawned = spawn_hotspots(info_map, step, rng)
        if spawned:
            events.append(f"Spawned {len(spawned)} hotspot(s)")

        previous_agents = agents
        agents, task_decisions = assign_demo_tasks(
            agents=agents,
            grid_map=grid_map,
            info_map=info_map,
            patrol_routes=patrol_routes,
            patrol_indices=patrol_indices,
        )

        next_agents: list[AgentState] = []
        observed_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        detected_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        confirmations_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        false_alarms_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        for agent in agents:
            advanced = advance_agent_towards_task(agent, dt_seconds)
            if not advanced.task.has_target:
                patrol_indices[advanced.agent_id] = (
                    patrol_indices[advanced.agent_id] + 1
                ) % len(patrol_routes[advanced.agent_id])
            next_agents.append(advanced)
            trajectories[advanced.agent_id].append((advanced.x, advanced.y))
        agents = tuple(next_agents)

        for agent in agents:
            observed_indices = apply_agent_coverage(coverage_map, agent, step=step)
            observed_by_agent[agent.agent_id] = observed_indices
            observe_cells(info_map, observed_indices, observer_id=agent.agent_id, step=step)
            if agent.kind == "UAV":
                detected = apply_uav_detection(info_map, agent, observed_indices, step=step, rng=rng)
                detected_by_agent[agent.agent_id] = detected
                if detected:
                    events.append(f"{agent.agent_id} marked {len(detected)} suspected hotspot(s)")
            else:
                resolved = apply_usv_confirmation(info_map, agent, observed_indices, step=step)
                confirmed_count = 0
                false_alarm_count = 0
                confirmed_indices: list[tuple[int, int]] = []
                false_alarm_indices: list[tuple[int, int]] = []
                for row, col in resolved:
                    state = info_map.state_at(row, col)
                    if state.known_hotspot_state == HotspotKnowledgeState.CONFIRMED:
                        confirmed_count += 1
                        confirmed_indices.append((row, col))
                    elif state.known_hotspot_state == HotspotKnowledgeState.FALSE_ALARM:
                        false_alarm_count += 1
                        false_alarm_indices.append((row, col))
                confirmations_by_agent[agent.agent_id] = tuple(confirmed_indices)
                false_alarms_by_agent[agent.agent_id] = tuple(false_alarm_indices)
                if confirmed_count:
                    events.append(f"{agent.agent_id} confirmed {confirmed_count} hotspot(s)")
                if false_alarm_count:
                    events.append(f"{agent.agent_id} cleared {false_alarm_count} false alarm(s)")

        frames.append(
            _capture_frame(
                step=step,
                agents=agents,
                info_map=info_map,
                coverage_map=coverage_map,
                trajectories=trajectories,
                events=tuple(events),
            )
        )
        step_logs.append(
            build_step_log(
                step=step,
                agents=agents,
                info_map=info_map,
                coverage_map=coverage_map,
                events=tuple(events),
                observed_by_agent=observed_by_agent,
                spawned_hotspots=spawned,
                newly_stale_cells=newly_stale_cells,
                detected_by_agent=detected_by_agent,
                confirmations_by_agent=confirmations_by_agent,
                false_alarms_by_agent=false_alarms_by_agent,
                task_decisions=task_decisions,
                prior_agents=previous_agents,
                planned_agents=agents,
            )
        )

    return SimulationReplay(
        sea_map=target_map,
        obstacle_layout=layout,
        frames=tuple(frames),
        seed=layout.seed,
        dt_seconds=dt_seconds,
        initial_agents=initial_agents,
        step_logs=tuple(step_logs),
    )


def _collect_stale_cells(info_map: InformationMap) -> tuple[tuple[int, int], ...]:
    stale_cells: list[tuple[int, int]] = []
    for cell in info_map.grid_map.flat_cells:
        if info_map.state_at(cell.row, cell.col).validity.value == "stale_known":
            stale_cells.append((cell.row, cell.col))
    return tuple(stale_cells)


def _capture_frame(
    *,
    step: int,
    agents: tuple[AgentState, ...],
    info_map: InformationMap,
    coverage_map: GridCoverageMap,
    trajectories: dict[str, list[tuple[float, float]]],
    events: tuple[str, ...],
) -> SimulationFrame:
    valid_cells: list[tuple[int, int]] = []
    suspected_cells: list[tuple[int, int]] = []
    confirmed_cells: list[tuple[int, int]] = []
    false_alarm_cells: list[tuple[int, int]] = []
    covered_cells: list[tuple[int, int]] = []

    for cell in info_map.grid_map.flat_cells:
        info_state = info_map.state_at(cell.row, cell.col)
        coverage_state = coverage_map.state_at(cell.row, cell.col)
        if info_state.validity.value == "valid":
            valid_cells.append((cell.row, cell.col))
        if info_state.known_hotspot_state == HotspotKnowledgeState.SUSPECTED:
            suspected_cells.append((cell.row, cell.col))
        elif info_state.known_hotspot_state == HotspotKnowledgeState.CONFIRMED:
            confirmed_cells.append((cell.row, cell.col))
        elif info_state.known_hotspot_state == HotspotKnowledgeState.FALSE_ALARM:
            false_alarm_cells.append((cell.row, cell.col))
        if coverage_state.coverage_state == CoverageState.COVERED:
            covered_cells.append((cell.row, cell.col))

    return SimulationFrame(
        step=step,
        agents=tuple(
            VisualAgent(
                agent_id=agent.agent_id,
                kind=agent.kind,
                x=agent.x,
                y=agent.y,
                heading_deg=agent.heading_deg,
            )
            for agent in agents
        ),
        valid_cells=tuple(valid_cells),
        suspected_cells=tuple(suspected_cells),
        confirmed_cells=tuple(confirmed_cells),
        false_alarm_cells=tuple(false_alarm_cells),
        covered_cells=tuple(covered_cells),
        coverage_ratio=coverage_map.covered_ratio(),
        events=events,
        trajectories=tuple(
            (agent_id, tuple(points))
            for agent_id, points in sorted(trajectories.items())
        ),
    )
