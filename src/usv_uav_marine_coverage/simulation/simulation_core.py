"""Core replay-simulation flow for the current preview stage."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random

from usv_uav_marine_coverage.agent_model import AgentState, is_operational_agent
from usv_uav_marine_coverage.agent_overlay import VisualAgent
from usv_uav_marine_coverage.environment import (
    ObstacleLayout,
    SeaMap,
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.events import apply_scheduled_events
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    ExecutionStage,
)
from usv_uav_marine_coverage.execution.progress_feedback import (
    build_initial_progress_states,
)
from usv_uav_marine_coverage.execution.task_claim_runtime import find_task_by_id
from usv_uav_marine_coverage.grid import (
    CoverageState,
    GridCoverageMap,
    GridMap,
    apply_agent_coverage,
    build_grid_coverage_map,
    build_grid_map,
)
from usv_uav_marine_coverage.information_map import (
    HotspotKnowledgeState,
    InformationMap,
    advance_information_age,
    apply_uav_detection,
    apply_usv_confirmation,
    build_information_map,
    observe_cells,
    spawn_baseline_tasks,
    spawn_hotspots,
)
from usv_uav_marine_coverage.planning.astar_path_planner import (
    reset_planner_metrics,
    snapshot_planner_metrics,
)
from usv_uav_marine_coverage.tasking.aoi_energy_auction_allocator import (
    allocate_tasks_with_aoi_energy_auction_policy,
)
from usv_uav_marine_coverage.tasking.baseline_task_generator import build_baseline_tasks
from usv_uav_marine_coverage.tasking.basic_task_allocator import (
    allocate_tasks_with_basic_policy,
)
from usv_uav_marine_coverage.tasking.cost_aware_task_allocator import (
    allocate_tasks_with_cost_aware_policy,
)
from usv_uav_marine_coverage.tasking.distributed_cbba_allocator import (
    allocate_tasks_with_distributed_cbba_policy,
)
from usv_uav_marine_coverage.tasking.hotspot_task_generator import sync_hotspot_confirmation_tasks
from usv_uav_marine_coverage.tasking.rho_task_allocator import (
    allocate_tasks_with_rho_policy,
)
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType
from usv_uav_marine_coverage.tasking.uav_resupply_task_generator import (
    build_uav_resupply_tasks,
)

from .experiment_config import ExperimentConfig, serialize_experiment_config
from .simulation_agent_runtime import (
    advance_agents_one_step,
    build_initial_execution_states,
    build_wreck_zones,
)
from .simulation_policy import build_demo_agent_states, build_patrol_routes
from .simulation_task_runtime import (
    finalize_task_resolutions,
    requeue_tasks_blocked_by_wrecks,
    serialize_task_assignment,
    sync_task_statuses,
)
from .uav_coverage_runtime import build_initial_uav_coverage_states


@dataclass(frozen=True)
class SimulationFrame:
    """One replay frame for the visualization preview."""

    step: int
    agents: tuple[VisualAgent, ...]
    valid_cells: tuple[tuple[int, int], ...]
    stale_cells: tuple[tuple[int, int], ...]
    baseline_cells: tuple[tuple[int, int], ...]
    pending_hotspot_cells: tuple[tuple[int, int], ...]
    uav_checked_cells: tuple[tuple[int, int], ...]
    pending_hotspots: tuple[dict[str, object], ...]
    uav_checked_hotspots: tuple[dict[str, object], ...]
    covered_cells: tuple[tuple[int, int], ...]
    coverage_ratio: float
    events: tuple[str, ...]
    planned_paths: tuple[tuple[str, tuple[tuple[float, float], ...]], ...]
    trajectories: tuple[tuple[str, tuple[tuple[float, float], ...]], ...]


@dataclass(frozen=True)
class SimulationReplay:
    """Replay data and static environment for the preview page."""

    sea_map: SeaMap
    obstacle_layout: ObstacleLayout
    frames: tuple[SimulationFrame, ...]
    seed: int
    dt_seconds: float
    experiment_config: dict[str, object]
    initial_agents: tuple[AgentState, ...]
    step_logs: tuple[dict[str, object], ...]


def build_simulation_replay(
    sea_map: SeaMap | None = None,
    obstacle_layout: ObstacleLayout | None = None,
    experiment_config: ExperimentConfig | None = None,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationReplay:
    """Build a simple replay preview on top of the current simulation baseline."""

    from .simulation_logging import build_step_log

    effective_config = experiment_config
    if effective_config is None:
        from .experiment_config import build_default_experiment_config

        effective_config = build_default_experiment_config(
            seed=seed,
            steps=steps,
            dt_seconds=dt_seconds,
        )

    effective_steps = effective_config.simulation.steps
    effective_dt_seconds = effective_config.simulation.dt_seconds

    if effective_steps <= 0:
        raise ValueError("Simulation steps must be positive.")
    if effective_dt_seconds <= 0:
        raise ValueError("Simulation time step must be positive.")

    target_map = sea_map or build_default_sea_map()
    layout = obstacle_layout or build_obstacle_layout(
        target_map,
        seed=effective_config.simulation.seed,
    )
    grid_map = build_grid_map(target_map, layout)
    coverage_map = build_grid_coverage_map(grid_map)
    info_map = build_information_map(grid_map, effective_config.information_map)
    rng = Random(layout.seed + 17)
    seeded_hotspots = spawn_hotspots(info_map, step=0, rng=rng)

    agents = build_demo_agent_states()
    initial_agents = agents
    uav_coverage_states = (
        build_initial_uav_coverage_states(
            agents=agents,
            grid_map=grid_map,
            info_map=info_map,
            step=0,
        )
        if effective_config.algorithms.uav_search_planner
        == "uav_persistent_multi_region_coverage_planner"
        else {}
    )
    patrol_routes = build_patrol_routes(
        uav_search_planner=effective_config.algorithms.uav_search_planner,
        agents=agents,
        info_map=info_map,
        uav_coverage_states=uav_coverage_states,
    )
    execution_states = build_initial_execution_states(agents)
    progress_states = build_initial_progress_states(agents)
    task_records: tuple[TaskRecord, ...] = ()
    trajectories = {agent.agent_id: [(agent.x, agent.y)] for agent in agents}
    step_logs: list[dict[str, object]] = []

    frames = [
        _capture_frame(
            step=0,
            agents=agents,
            info_map=info_map,
            coverage_map=coverage_map,
            execution_states=execution_states,
            trajectories=trajectories,
            events=(f"Seeded {len(seeded_hotspots)} hotspot(s)",) if seeded_hotspots else (),
        )
    ]
    step_logs.append(
        build_step_log(
            step=0,
            agents=agents,
            info_map=info_map,
            coverage_map=coverage_map,
            events=(f"Seeded {len(seeded_hotspots)} hotspot(s)",) if seeded_hotspots else (),
            observed_by_agent={},
            spawned_hotspots=seeded_hotspots,
            newly_stale_cells=tuple(_collect_stale_cells(info_map)),
            detected_by_agent={},
            confirmations_by_agent={},
            task_decisions=(),
            prior_agents=agents,
            planned_agents=agents,
            task_records=task_records,
            execution_states=execution_states,
            progress_states=progress_states,
            uav_coverage_states=uav_coverage_states,
            planner_metrics=snapshot_planner_metrics(),
            failure_events=(),
            reassignments=(),
            wreck_zones=(),
            failed_agent_ids=(),
        )
    )

    for step in range(1, effective_steps + 1):
        reset_planner_metrics()
        events: list[str] = []
        stale_before = set(_collect_stale_cells(info_map))
        advance_information_age(info_map, step)
        stale_after = set(_collect_stale_cells(info_map))
        newly_stale_cells = tuple(sorted(stale_after - stale_before))
        spawned = spawn_hotspots(info_map, step, rng)
        if spawned:
            events.append(f"Spawned {len(spawned)} hotspot(s)")
        spawned_baselines = spawn_baseline_tasks(info_map, step, rng)
        if spawned_baselines:
            events.append(f"Spawned {len(spawned_baselines)} baseline task point(s)")

        task_records = sync_hotspot_confirmation_tasks(
            info_map,
            step=step,
            existing_tasks=task_records,
        )
        task_records = build_baseline_tasks(
            info_map,
            step=step,
            existing_tasks=task_records,
        )
        task_records = build_uav_resupply_tasks(
            agents,
            step=step,
            existing_tasks=task_records,
        )
        (
            agents,
            execution_states,
            progress_states,
            task_records,
            applied_events,
            released_assignments,
        ) = apply_scheduled_events(
            step=step,
            scheduled_events=effective_config.events,
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=task_records,
        )
        if applied_events:
            events.extend(applied_event.summary for applied_event in applied_events)
        wreck_zones = build_wreck_zones(agents)
        task_records, wreck_reassignments = requeue_tasks_blocked_by_wrecks(
            task_records=task_records,
            agents=agents,
            wreck_zones=wreck_zones,
            step=step,
        )
        task_records, task_assignments = _allocate_task_records(
            task_records,
            step=step,
            task_allocator=effective_config.algorithms.task_allocator,
            zone_partition_policy=effective_config.algorithms.zone_partition_policy,
            distributed_sync_interval_steps=(
                effective_config.algorithms.distributed_sync_interval_steps
            ),
            distributed_broadcast_range_m=(
                effective_config.algorithms.distributed_broadcast_range_m
            ),
            distributed_winner_memory_ttl_steps=(
                effective_config.algorithms.distributed_winner_memory_ttl_steps
            ),
            distributed_bundle_length=(effective_config.algorithms.distributed_bundle_length),
            usv_path_planner=effective_config.algorithms.usv_path_planner,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
        )
        if any(task.task_type == TaskType.UAV_RESUPPLY for task in task_records):
            events.append("Low-energy UAV rendezvous task active")

        previous_agents = agents
        task_decisions = tuple(
            serialize_task_assignment(assignment) for assignment in task_assignments
        )
        agents, execution_states, progress_states = advance_agents_one_step(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            uav_coverage_states=uav_coverage_states,
            task_records=task_records,
            patrol_routes=patrol_routes,
            grid_map=grid_map,
            info_map=info_map,
            obstacle_layout=layout,
            dt_seconds=effective_dt_seconds,
            step=step,
            usv_path_planner=effective_config.algorithms.usv_path_planner,
            uav_search_planner=effective_config.algorithms.uav_search_planner,
            execution_policy=effective_config.algorithms.execution_policy,
        )
        task_records = sync_task_statuses(task_records, execution_states, progress_states)

        observed_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        uav_checked_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        confirmations_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        for agent in agents:
            trajectories[agent.agent_id].append((agent.x, agent.y))

        for agent in agents:
            if not is_operational_agent(agent):
                observed_by_agent[agent.agent_id] = ()
                continue
            observed_indices = apply_agent_coverage(coverage_map, agent, step=step)
            observed_by_agent[agent.agent_id] = observed_indices
            observe_cells(info_map, observed_indices, observer_id=agent.agent_id, step=step)
            if agent.kind == "UAV":
                detected = apply_uav_detection(
                    info_map, agent, observed_indices, step=step, rng=rng
                )
                uav_checked_by_agent[agent.agent_id] = detected
                if detected:
                    events.append(
                        f"{agent.agent_id} completed {len(detected)} hotspot pre-check(s)"
                    )
            else:
                confirmation_indices = _confirmation_indices_for_usv(
                    agent_id=agent.agent_id,
                    observed_indices=observed_indices,
                    task_records=task_records,
                    execution_states=execution_states,
                )
                resolved = apply_usv_confirmation(
                    info_map,
                    agent,
                    confirmation_indices,
                    step=step,
                    rng=rng,
                )
                confirmed_count = 0
                confirmed_indices: list[tuple[int, int]] = []
                for row, col in resolved:
                    state = info_map.state_at(row, col)
                    if state.known_hotspot_state == HotspotKnowledgeState.CONFIRMED:
                        confirmed_count += 1
                        confirmed_indices.append((row, col))
                confirmations_by_agent[agent.agent_id] = tuple(confirmed_indices)
                if confirmed_count:
                    events.append(f"{agent.agent_id} confirmed {confirmed_count} hotspot(s)")

        task_records, execution_states, progress_states = finalize_task_resolutions(
            agents=agents,
            execution_states=execution_states,
            progress_states=progress_states,
            task_records=task_records,
            patrol_routes=patrol_routes,
            info_map=info_map,
            step=step,
        )
        task_records = sync_task_statuses(task_records, execution_states, progress_states)

        frames.append(
            _capture_frame(
                step=step,
                agents=agents,
                info_map=info_map,
                coverage_map=coverage_map,
                execution_states=execution_states,
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
                detected_by_agent=uav_checked_by_agent,
                confirmations_by_agent=confirmations_by_agent,
                task_decisions=task_decisions,
                prior_agents=previous_agents,
                planned_agents=agents,
                task_records=task_records,
                execution_states=execution_states,
                progress_states=progress_states,
                uav_coverage_states=uav_coverage_states,
                planner_metrics=snapshot_planner_metrics(),
                failure_events=tuple(
                    {
                        "step": applied_event.step,
                        "type": applied_event.event_type.value,
                        "agent_id": applied_event.agent_id,
                        "summary": applied_event.summary,
                    }
                    for applied_event in applied_events
                ),
                reassignments=tuple(released_assignments) + tuple(wreck_reassignments),
                wreck_zones=tuple(
                    {
                        "source_agent_id": wreck.source_agent_id,
                        "x": round(wreck.x, 3),
                        "y": round(wreck.y, 3),
                        "radius": round(wreck.radius, 3),
                    }
                    for wreck in wreck_zones
                ),
                failed_agent_ids=tuple(
                    agent.agent_id for agent in agents if not is_operational_agent(agent)
                ),
            )
        )

    return SimulationReplay(
        sea_map=target_map,
        obstacle_layout=layout,
        frames=tuple(frames),
        seed=layout.seed,
        dt_seconds=effective_dt_seconds,
        experiment_config=serialize_experiment_config(effective_config),
        initial_agents=initial_agents,
        step_logs=tuple(step_logs),
    )


def _confirmation_indices_for_usv(
    *,
    agent_id: str,
    observed_indices: tuple[tuple[int, int], ...],
    task_records: tuple[TaskRecord, ...],
    execution_states: dict[str, AgentExecutionState],
) -> tuple[tuple[int, int], ...]:
    """Only allow hotspot confirmation while a USV is stopped on its assigned task."""

    execution_state = execution_states.get(agent_id)
    if execution_state is None or execution_state.stage != ExecutionStage.ON_TASK:
        return ()

    active_task = find_task_by_id(task_records, execution_state.active_task_id)
    if active_task is None or active_task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return ()
    if active_task.target_row is None or active_task.target_col is None:
        return ()

    target_index = (active_task.target_row, active_task.target_col)
    if target_index not in observed_indices:
        return ()
    return (target_index,)


def _collect_stale_cells(info_map: InformationMap) -> tuple[tuple[int, int], ...]:
    stale_cells: list[tuple[int, int]] = []
    for cell in info_map.grid_map.flat_cells:
        if cell.has_obstacle:
            continue
        if info_map.state_at(cell.row, cell.col).validity.value == "stale_known":
            stale_cells.append((cell.row, cell.col))
    return tuple(stale_cells)


def _allocate_task_records(
    task_records: tuple[TaskRecord, ...],
    *,
    step: int,
    task_allocator: str,
    zone_partition_policy: str,
    distributed_sync_interval_steps: int,
    distributed_broadcast_range_m: float,
    distributed_winner_memory_ttl_steps: int,
    distributed_bundle_length: int,
    usv_path_planner: str,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    grid_map: GridMap,
    info_map: InformationMap,
) -> tuple[tuple[TaskRecord, ...], tuple]:
    if task_allocator == "basic_task_allocator":
        return allocate_tasks_with_basic_policy(
            task_records,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            usv_path_planner=usv_path_planner,
        )
    if task_allocator == "cost_aware_centralized_allocator":
        return allocate_tasks_with_cost_aware_policy(
            task_records,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
        )
    if task_allocator == "aoi_energy_auction_allocator":
        return allocate_tasks_with_aoi_energy_auction_policy(
            task_records,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
        )
    if task_allocator == "rho_task_allocator":
        return allocate_tasks_with_rho_policy(
            task_records,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
        )
    if task_allocator == "distributed_cbba_allocator":
        return allocate_tasks_with_distributed_cbba_policy(
            task_records,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            sync_interval_steps=distributed_sync_interval_steps,
            broadcast_range_m=distributed_broadcast_range_m,
            winner_memory_ttl_steps=distributed_winner_memory_ttl_steps,
            bundle_length=distributed_bundle_length,
        )
    raise ValueError(f"Unsupported task allocator {task_allocator!r}")


def _capture_frame(
    *,
    step: int,
    agents: tuple[AgentState, ...],
    info_map: InformationMap,
    coverage_map: GridCoverageMap,
    execution_states: dict[str, AgentExecutionState],
    trajectories: dict[str, list[tuple[float, float]]],
    events: tuple[str, ...],
) -> SimulationFrame:
    valid_cells: list[tuple[int, int]] = []
    stale_cells: list[tuple[int, int]] = []
    baseline_cells: list[tuple[int, int]] = []
    pending_hotspot_cells: list[tuple[int, int]] = []
    uav_checked_cells: list[tuple[int, int]] = []
    pending_hotspots: list[dict[str, object]] = []
    uav_checked_hotspots: list[dict[str, object]] = []
    covered_cells: list[tuple[int, int]] = []

    for cell in info_map.grid_map.flat_cells:
        info_state = info_map.state_at(cell.row, cell.col)
        coverage_state = coverage_map.state_at(cell.row, cell.col)
        if not cell.has_obstacle:
            if info_state.validity.value == "valid":
                valid_cells.append((cell.row, cell.col))
            else:
                stale_cells.append((cell.row, cell.col))
        if info_state.has_baseline_task:
            baseline_cells.append((cell.row, cell.col))
        if (
            info_state.ground_truth_hotspot
            and info_state.known_hotspot_state == HotspotKnowledgeState.NONE
        ):
            pending_hotspot_cells.append((cell.row, cell.col))
            pending_hotspots.append(
                {
                    "hotspot_id": info_state.ground_truth_hotspot_id,
                    "row": cell.row,
                    "col": cell.col,
                    "x": round(cell.center_x, 3),
                    "y": round(cell.center_y, 3),
                }
            )
        elif info_state.known_hotspot_state == HotspotKnowledgeState.UAV_CHECKED:
            uav_checked_cells.append((cell.row, cell.col))
            uav_checked_hotspots.append(
                {
                    "hotspot_id": (
                        info_state.known_hotspot_id
                        if info_state.known_hotspot_id is not None
                        else info_state.ground_truth_hotspot_id
                    ),
                    "row": cell.row,
                    "col": cell.col,
                    "x": round(cell.center_x, 3),
                    "y": round(cell.center_y, 3),
                }
            )
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
        stale_cells=tuple(stale_cells),
        baseline_cells=tuple(baseline_cells),
        pending_hotspot_cells=tuple(pending_hotspot_cells),
        uav_checked_cells=tuple(uav_checked_cells),
        pending_hotspots=tuple(pending_hotspots),
        uav_checked_hotspots=tuple(uav_checked_hotspots),
        covered_cells=tuple(covered_cells),
        coverage_ratio=coverage_map.covered_ratio(),
        events=events,
        planned_paths=tuple(
            (
                agent.agent_id,
                _extract_active_plan_points(
                    agent,
                    execution_states.get(agent.agent_id),
                ),
            )
            for agent in agents
        ),
        trajectories=tuple(
            (agent_id, tuple(points)) for agent_id, points in sorted(trajectories.items())
        ),
    )


def _extract_active_plan_points(
    agent: AgentState,
    execution_state: AgentExecutionState | None,
) -> tuple[tuple[float, float], ...]:
    if execution_state is None or execution_state.active_plan is None:
        return ()

    plan = execution_state.active_plan
    if len(plan.waypoints) < 1:
        return ()

    start_index = min(execution_state.current_waypoint_index, len(plan.waypoints) - 1)
    remaining_waypoints = plan.waypoints[start_index:]
    if not remaining_waypoints:
        return ()

    points: list[tuple[float, float]] = [(agent.x, agent.y)]
    for waypoint in remaining_waypoints:
        point = (waypoint.x, waypoint.y)
        if point == points[-1]:
            continue
        points.append(point)
    if len(points) < 2:
        return ()
    return tuple(points)
