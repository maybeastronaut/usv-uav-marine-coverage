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
    AgentProgressState,
    ExecutionStage,
    UavCoverageState,
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
from usv_uav_marine_coverage.tasking.task_types import (
    TaskAssignment,
    TaskRecord,
    TaskStatus,
    TaskType,
)
from usv_uav_marine_coverage.tasking.uav_resupply_task_generator import (
    build_uav_resupply_tasks,
)
from usv_uav_marine_coverage.tasking.uav_support_policy import (
    set_uav_usv_meeting_enabled,
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
    current_observed_cells: tuple[tuple[int, int], ...] = ()


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


@dataclass
class PendingSimulationStep:
    """Prepared pre-allocation state for one simulation step."""

    step: int
    events: list[str]
    prior_task_records: tuple[TaskRecord, ...]
    newly_stale_cells: tuple[tuple[int, int], ...]
    spawned_hotspots: tuple[tuple[int, int], ...]
    applied_events: tuple[object, ...]
    released_assignments: tuple[dict[str, object], ...]
    wreck_reassignments: tuple[dict[str, object], ...]
    wreck_zones: tuple[object, ...]
    previous_agents: tuple[AgentState, ...]


@dataclass
class SimulationStepResult:
    """One finalized simulation step result."""

    step: int
    frame: SimulationFrame
    step_log: dict[str, object]
    task_assignments: tuple[TaskAssignment, ...]


@dataclass
class SimulationRuntime:
    """Mutable simulation runtime shared by replay and RL environments."""

    effective_config: ExperimentConfig
    target_map: SeaMap
    layout: ObstacleLayout
    grid_map: GridMap
    coverage_map: GridCoverageMap
    info_map: InformationMap
    rng: Random
    agents: tuple[AgentState, ...]
    initial_agents: tuple[AgentState, ...]
    uav_coverage_states: dict[str, UavCoverageState]
    patrol_routes: dict[str, tuple[tuple[float, float], ...]]
    execution_states: dict[str, AgentExecutionState]
    progress_states: dict[str, AgentProgressState]
    task_records: tuple[TaskRecord, ...]
    trajectories: dict[str, list[tuple[float, float]]]
    frames: list[SimulationFrame]
    step_logs: list[dict[str, object]]
    current_step: int = 0
    pending_step: PendingSimulationStep | None = None


def build_simulation_replay(
    sea_map: SeaMap | None = None,
    obstacle_layout: ObstacleLayout | None = None,
    experiment_config: ExperimentConfig | None = None,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationReplay:
    """Build a simple replay preview on top of the current simulation baseline."""

    runtime = initialize_simulation_runtime(
        experiment_config=experiment_config,
        sea_map=sea_map,
        obstacle_layout=obstacle_layout,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    total_steps = runtime.effective_config.simulation.steps

    while runtime.current_step < total_steps:
        step = runtime.current_step + 1
        if step % 10 == 0 or step == total_steps:
            print(
                f"\r[INFO] Simulating... Step {step}/{total_steps} ({step/total_steps:.1%})",
                end="",
                flush=True,
            )
            if step == total_steps:
                print("", flush=True)
        prepare_simulation_runtime_step(runtime)
        advance_simulation_runtime_step(runtime)

    return build_replay_from_runtime(runtime)


def initialize_simulation_runtime(
    *,
    experiment_config: ExperimentConfig | None = None,
    sea_map: SeaMap | None = None,
    obstacle_layout: ObstacleLayout | None = None,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationRuntime:
    """Build the shared mutable runtime used by replay and RL environments."""

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

    set_uav_usv_meeting_enabled(effective_config.coordination.enable_uav_usv_meeting)
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
    step_logs = [
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
            prior_task_records=(),
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
    ]
    return SimulationRuntime(
        effective_config=effective_config,
        target_map=target_map,
        layout=layout,
        grid_map=grid_map,
        coverage_map=coverage_map,
        info_map=info_map,
        rng=rng,
        agents=agents,
        initial_agents=initial_agents,
        uav_coverage_states=uav_coverage_states,
        patrol_routes=patrol_routes,
        execution_states=execution_states,
        progress_states=progress_states,
        task_records=task_records,
        trajectories=trajectories,
        frames=frames,
        step_logs=step_logs,
    )


def prepare_simulation_runtime_step(runtime: SimulationRuntime) -> PendingSimulationStep:
    """Advance the runtime to the pre-allocation state for the next step."""

    if runtime.pending_step is not None:
        raise ValueError("Simulation runtime already has one pending prepared step.")
    if runtime.current_step >= runtime.effective_config.simulation.steps:
        raise ValueError("Simulation runtime is already complete.")

    step = runtime.current_step + 1
    reset_planner_metrics()
    events: list[str] = []
    prior_task_records = runtime.task_records
    stale_before = set(_collect_stale_cells(runtime.info_map))
    advance_information_age(runtime.info_map, step)
    stale_after = set(_collect_stale_cells(runtime.info_map))
    newly_stale_cells = tuple(sorted(stale_after - stale_before))
    spawned_hotspots = spawn_hotspots(runtime.info_map, step, runtime.rng)
    if spawned_hotspots:
        events.append(f"Spawned {len(spawned_hotspots)} hotspot(s)")
    spawned_baselines = spawn_baseline_tasks(runtime.info_map, step, runtime.rng)
    if spawned_baselines:
        events.append(f"Spawned {len(spawned_baselines)} baseline task point(s)")

    runtime.task_records = sync_hotspot_confirmation_tasks(
        runtime.info_map,
        step=step,
        existing_tasks=runtime.task_records,
    )
    runtime.task_records = build_baseline_tasks(
        runtime.info_map,
        step=step,
        existing_tasks=runtime.task_records,
    )
    if runtime.effective_config.coordination.enable_uav_usv_meeting:
        runtime.task_records = build_uav_resupply_tasks(
            runtime.agents,
            step=step,
            existing_tasks=runtime.task_records,
        )
    (
        runtime.agents,
        runtime.execution_states,
        runtime.progress_states,
        runtime.task_records,
        applied_events,
        released_assignments,
    ) = apply_scheduled_events(
        step=step,
        scheduled_events=runtime.effective_config.events,
        agents=runtime.agents,
        execution_states=runtime.execution_states,
        progress_states=runtime.progress_states,
        task_records=runtime.task_records,
    )
    if applied_events:
        events.extend(applied_event.summary for applied_event in applied_events)
    wreck_zones = build_wreck_zones(runtime.agents)
    runtime.task_records, wreck_reassignments = requeue_tasks_blocked_by_wrecks(
        task_records=runtime.task_records,
        agents=runtime.agents,
        wreck_zones=wreck_zones,
        step=step,
    )
    runtime.pending_step = PendingSimulationStep(
        step=step,
        events=events,
        prior_task_records=prior_task_records,
        newly_stale_cells=newly_stale_cells,
        spawned_hotspots=spawned_hotspots,
        applied_events=tuple(applied_events),
        released_assignments=tuple(released_assignments),
        wreck_reassignments=tuple(wreck_reassignments),
        wreck_zones=tuple(wreck_zones),
        previous_agents=runtime.agents,
    )
    return runtime.pending_step


def advance_simulation_runtime_step(
    runtime: SimulationRuntime,
    *,
    allocation_override: tuple[tuple[TaskRecord, ...], tuple[TaskAssignment, ...]] | None = None,
) -> SimulationStepResult:
    """Finalize the current prepared step and advance the runtime by one tick."""

    from .simulation_logging import build_step_log

    pending = runtime.pending_step
    if pending is None:
        raise ValueError("Simulation runtime must be prepared before advancing one step.")

    if allocation_override is None:
        runtime.task_records, task_assignments = _allocate_task_records(
            runtime.task_records,
            step=pending.step,
            task_allocator=runtime.effective_config.algorithms.task_allocator,
            zone_partition_policy=runtime.effective_config.algorithms.zone_partition_policy,
            distributed_sync_interval_steps=(
                runtime.effective_config.algorithms.distributed_sync_interval_steps
            ),
            distributed_broadcast_range_m=(
                runtime.effective_config.algorithms.distributed_broadcast_range_m
            ),
            distributed_winner_memory_ttl_steps=(
                runtime.effective_config.algorithms.distributed_winner_memory_ttl_steps
            ),
            distributed_bundle_length=runtime.effective_config.algorithms.distributed_bundle_length,
            usv_path_planner=runtime.effective_config.algorithms.usv_path_planner,
            agents=runtime.agents,
            execution_states=runtime.execution_states,
            grid_map=runtime.grid_map,
            info_map=runtime.info_map,
            rl_checkpoint_path=runtime.effective_config.rl.checkpoint_path,
            rl_policy_id=runtime.effective_config.rl.policy_id,
        )
    else:
        runtime.task_records, task_assignments = allocation_override

    active_task_statuses = {TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
    if runtime.effective_config.coordination.enable_uav_usv_meeting and any(
        task.task_type == TaskType.UAV_RESUPPLY and task.status in active_task_statuses
        for task in runtime.task_records
    ):
        pending.events.append("Low-energy UAV rendezvous task active")

    task_decisions = tuple(
        serialize_task_assignment(assignment) for assignment in task_assignments
    )
    runtime.agents, runtime.execution_states, runtime.progress_states = advance_agents_one_step(
        agents=runtime.agents,
        execution_states=runtime.execution_states,
        progress_states=runtime.progress_states,
        uav_coverage_states=runtime.uav_coverage_states,
        task_records=runtime.task_records,
        patrol_routes=runtime.patrol_routes,
        grid_map=runtime.grid_map,
        info_map=runtime.info_map,
        obstacle_layout=runtime.layout,
        dt_seconds=runtime.effective_config.simulation.dt_seconds,
        step=pending.step,
        usv_path_planner=runtime.effective_config.algorithms.usv_path_planner,
        uav_search_planner=runtime.effective_config.algorithms.uav_search_planner,
        execution_policy=runtime.effective_config.algorithms.execution_policy,
        enable_uav_usv_meeting=runtime.effective_config.coordination.enable_uav_usv_meeting,
    )
    runtime.task_records = sync_task_statuses(
        runtime.task_records,
        runtime.execution_states,
        runtime.progress_states,
        step=pending.step,
    )

    observed_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
    uav_checked_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
    confirmations_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
    for agent in runtime.agents:
        runtime.trajectories[agent.agent_id].append((agent.x, agent.y))

    for agent in runtime.agents:
        if not is_operational_agent(agent):
            observed_by_agent[agent.agent_id] = ()
            continue
        observed_indices = apply_agent_coverage(runtime.coverage_map, agent, step=pending.step)
        observed_by_agent[agent.agent_id] = observed_indices
        observe_cells(
            runtime.info_map,
            observed_indices,
            observer_id=agent.agent_id,
            step=pending.step,
        )
        if agent.kind == "UAV":
            detected = apply_uav_detection(
                runtime.info_map,
                agent,
                observed_indices,
                step=pending.step,
                rng=runtime.rng,
            )
            uav_checked_by_agent[agent.agent_id] = detected
            if detected:
                pending.events.append(
                    f"{agent.agent_id} completed {len(detected)} hotspot pre-check(s)"
                )
        else:
            confirmation_indices = _confirmation_indices_for_usv(
                agent_id=agent.agent_id,
                observed_indices=observed_indices,
                task_records=runtime.task_records,
                execution_states=runtime.execution_states,
                info_map=runtime.info_map,
            )
            resolved = apply_usv_confirmation(
                runtime.info_map,
                agent,
                confirmation_indices,
                step=pending.step,
                rng=runtime.rng,
            )
            confirmed_indices: list[tuple[int, int]] = []
            for row, col in resolved:
                state = runtime.info_map.state_at(row, col)
                if state.known_hotspot_state == HotspotKnowledgeState.CONFIRMED:
                    confirmed_indices.append((row, col))
            confirmations_by_agent[agent.agent_id] = tuple(confirmed_indices)
            if confirmed_indices:
                pending.events.append(
                    f"{agent.agent_id} confirmed {len(confirmed_indices)} hotspot(s)"
                )

    runtime.task_records, runtime.execution_states, runtime.progress_states = finalize_task_resolutions(
        agents=runtime.agents,
        execution_states=runtime.execution_states,
        progress_states=runtime.progress_states,
        task_records=runtime.task_records,
        patrol_routes=runtime.patrol_routes,
        info_map=runtime.info_map,
        step=pending.step,
    )
    runtime.task_records = sync_task_statuses(
        runtime.task_records,
        runtime.execution_states,
        runtime.progress_states,
        step=pending.step,
    )

    frame = _capture_frame(
        step=pending.step,
        agents=runtime.agents,
        info_map=runtime.info_map,
        coverage_map=runtime.coverage_map,
        execution_states=runtime.execution_states,
        trajectories=runtime.trajectories,
        events=tuple(pending.events),
    )
    runtime.frames.append(frame)
    step_log = build_step_log(
        step=pending.step,
        agents=runtime.agents,
        info_map=runtime.info_map,
        coverage_map=runtime.coverage_map,
        events=tuple(pending.events),
        observed_by_agent=observed_by_agent,
        spawned_hotspots=pending.spawned_hotspots,
        newly_stale_cells=pending.newly_stale_cells,
        detected_by_agent=uav_checked_by_agent,
        confirmations_by_agent=confirmations_by_agent,
        task_decisions=task_decisions,
        prior_task_records=pending.prior_task_records,
        prior_agents=pending.previous_agents,
        planned_agents=runtime.agents,
        task_records=runtime.task_records,
        execution_states=runtime.execution_states,
        progress_states=runtime.progress_states,
        uav_coverage_states=runtime.uav_coverage_states,
        planner_metrics=snapshot_planner_metrics(),
        failure_events=tuple(
            {
                "step": applied_event.step,
                "type": applied_event.event_type.value,
                "agent_id": applied_event.agent_id,
                "summary": applied_event.summary,
            }
            for applied_event in pending.applied_events
        ),
        reassignments=pending.released_assignments + pending.wreck_reassignments,
        wreck_zones=tuple(
            {
                "source_agent_id": wreck.source_agent_id,
                "x": round(wreck.x, 3),
                "y": round(wreck.y, 3),
                "radius": round(wreck.radius, 3),
            }
            for wreck in pending.wreck_zones
        ),
        failed_agent_ids=tuple(
            agent.agent_id for agent in runtime.agents if not is_operational_agent(agent)
        ),
    )
    runtime.step_logs.append(step_log)
    runtime.current_step = pending.step
    runtime.pending_step = None
    return SimulationStepResult(
        step=runtime.current_step,
        frame=frame,
        step_log=step_log,
        task_assignments=task_assignments,
    )


def build_replay_from_runtime(runtime: SimulationRuntime) -> SimulationReplay:
    """Materialize a replay snapshot from the current runtime state."""

    return SimulationReplay(
        sea_map=runtime.target_map,
        obstacle_layout=runtime.layout,
        frames=tuple(runtime.frames),
        seed=runtime.layout.seed,
        dt_seconds=runtime.effective_config.simulation.dt_seconds,
        experiment_config=serialize_experiment_config(runtime.effective_config),
        initial_agents=runtime.initial_agents,
        step_logs=tuple(runtime.step_logs),
    )


def _confirmation_indices_for_usv(
    *,
    agent_id: str,
    observed_indices: tuple[tuple[int, int], ...],
    task_records: tuple[TaskRecord, ...],
    execution_states: dict[str, AgentExecutionState],
    info_map: InformationMap,
) -> tuple[tuple[int, int], ...]:
    """Return hotspot cells that this USV can confirm on this step.

    Assigned ON_TASK hotspot confirmation remains the primary path. In addition,
    a healthy USV may opportunistically accumulate confirmation progress when it
    physically observes a UAV-checked hotspot while already navigating the sea
    surface for patrol, return, baseline, or another hotspot task.
    """

    execution_state = execution_states.get(agent_id)
    if execution_state is None:
        return ()
    if execution_state.stage not in {
        ExecutionStage.PATROL,
        ExecutionStage.RETURN_TO_PATROL,
        ExecutionStage.GO_TO_TASK,
        ExecutionStage.ON_TASK,
    }:
        return ()

    observed_set = set(observed_indices)
    if not observed_set:
        return ()

    active_task = find_task_by_id(task_records, execution_state.active_task_id)
    confirmation_indices: list[tuple[int, int]] = []
    if (
        execution_state.stage == ExecutionStage.ON_TASK
        and active_task is not None
        and active_task.task_type == TaskType.HOTSPOT_CONFIRMATION
        and active_task.target_row is not None
        and active_task.target_col is not None
    ):
        target_index = (active_task.target_row, active_task.target_col)
        if target_index in observed_set:
            confirmation_indices.append(target_index)

    if active_task is not None and active_task.task_type == TaskType.UAV_RESUPPLY:
        return tuple(confirmation_indices)

    hotspot_indices = {
        (task.target_row, task.target_col)
        for task in task_records
        if (
            task.task_type == TaskType.HOTSPOT_CONFIRMATION
            and task.target_row is not None
            and task.target_col is not None
            and task.status not in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}
        )
    }
    for index in observed_indices:
        if index in confirmation_indices or index not in hotspot_indices:
            continue
        row, col = index
        if info_map.state_at(row, col).known_hotspot_state != HotspotKnowledgeState.UAV_CHECKED:
            continue
        confirmation_indices.append(index)

    return tuple(confirmation_indices)


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
    rl_checkpoint_path: str | None = None,
    rl_policy_id: str = "default_policy",
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
    if task_allocator == "rllib_ppo_usv_allocator":
        from usv_uav_marine_coverage.rl.policy_allocator import (
            allocate_tasks_with_rllib_ppo_policy,
        )

        return allocate_tasks_with_rllib_ppo_policy(
            task_records,
            agents=agents,
            execution_states=execution_states,
            grid_map=grid_map,
            info_map=info_map,
            step=step,
            usv_path_planner=usv_path_planner,
            zone_partition_policy=zone_partition_policy,
            checkpoint_path=rl_checkpoint_path,
            policy_id=rl_policy_id,
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
        current_observed_cells=(),
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
