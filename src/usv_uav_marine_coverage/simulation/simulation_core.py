"""Core replay-simulation flow for the current preview stage."""

from __future__ import annotations

from dataclasses import dataclass, replace
from random import Random

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    TaskMode,
    advance_agent_towards_task,
    assign_agent_task,
)
from usv_uav_marine_coverage.agent_overlay import VisualAgent
from usv_uav_marine_coverage.environment import (
    ObstacleLayout,
    SeaMap,
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.execution.basic_state_machine import (
    transition_to_on_task,
    transition_to_patrol,
    transition_to_return_to_patrol,
    transition_to_task,
)
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    ExecutionOutcome,
    ExecutionStage,
)
from usv_uav_marine_coverage.execution.path_follower import follow_path_step
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
from usv_uav_marine_coverage.planning.direct_line_planner import build_direct_line_plan
from usv_uav_marine_coverage.planning.fixed_patrol_planner import build_fixed_patrol_plan
from usv_uav_marine_coverage.tasking.baseline_task_generator import build_baseline_tasks
from usv_uav_marine_coverage.tasking.hotspot_task_generator import sync_hotspot_confirmation_tasks
from usv_uav_marine_coverage.tasking.nearest_agent_allocator import allocate_nearest_agents
from usv_uav_marine_coverage.tasking.task_types import (
    TaskAssignment,
    TaskRecord,
    TaskStatus,
    TaskType,
)

from .simulation_policy import build_demo_agent_states, build_patrol_routes


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
    execution_states = _build_initial_execution_states(agents)
    task_records: tuple[TaskRecord, ...] = ()
    trajectories = {agent.agent_id: [(agent.x, agent.y)] for agent in agents}
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
            task_records=task_records,
            execution_states=execution_states,
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

        task_records = sync_hotspot_confirmation_tasks(
            info_map,
            step=step,
            existing_tasks=task_records,
        )
        task_records = tuple(task_records) + build_baseline_tasks(
            info_map,
            step=step,
            existing_tasks=task_records,
        )
        task_records, task_assignments = allocate_nearest_agents(
            task_records,
            agents=agents,
            execution_states=execution_states,
        )

        previous_agents = agents
        task_decisions = tuple(
            _serialize_task_assignment(assignment) for assignment in task_assignments
        )
        agents, execution_states, task_records = _advance_simulation_step(
            agents=agents,
            execution_states=execution_states,
            task_records=task_records,
            patrol_routes=patrol_routes,
            dt_seconds=dt_seconds,
        )

        observed_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        detected_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        confirmations_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        false_alarms_by_agent: dict[str, tuple[tuple[int, int], ...]] = {}
        for agent in agents:
            trajectories[agent.agent_id].append((agent.x, agent.y))

        for agent in agents:
            observed_indices = apply_agent_coverage(coverage_map, agent, step=step)
            observed_by_agent[agent.agent_id] = observed_indices
            observe_cells(info_map, observed_indices, observer_id=agent.agent_id, step=step)
            if agent.kind == "UAV":
                detected = apply_uav_detection(
                    info_map, agent, observed_indices, step=step, rng=rng
                )
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

        task_records, execution_states = _finalize_task_resolutions(
            agents=agents,
            execution_states=execution_states,
            task_records=task_records,
            patrol_routes=patrol_routes,
            info_map=info_map,
            step=step,
        )
        task_records = _sync_task_statuses(task_records, execution_states)

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
                task_records=task_records,
                execution_states=execution_states,
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


def _build_initial_execution_states(
    agents: tuple[AgentState, ...],
) -> dict[str, AgentExecutionState]:
    return {
        agent.agent_id: AgentExecutionState(
            agent_id=agent.agent_id,
            stage=ExecutionStage.PATROL,
            active_task_id=None,
            active_plan=None,
            current_waypoint_index=0,
            patrol_route_id=agent.agent_id,
            patrol_waypoint_index=0,
        )
        for agent in agents
    }


def _advance_simulation_step(
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    task_records: tuple[TaskRecord, ...],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    dt_seconds: float,
) -> tuple[tuple[AgentState, ...], dict[str, AgentExecutionState], tuple[TaskRecord, ...]]:
    next_agents: list[AgentState] = []
    next_execution_states: dict[str, AgentExecutionState] = {}

    for agent in agents:
        execution_state = execution_states[agent.agent_id]
        active_task = _assigned_task_for_agent(task_records, agent.agent_id)
        execution_state = _pre_step_transition(
            agent,
            execution_state=execution_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
        )

        updated_agent, updated_execution_state = _run_agent_stage(
            agent,
            execution_state=execution_state,
            active_task=active_task,
            patrol_routes=patrol_routes,
            dt_seconds=dt_seconds,
        )
        next_agents.append(updated_agent)
        next_execution_states[agent.agent_id] = updated_execution_state

    synced_tasks = _sync_task_statuses(task_records, next_execution_states)
    return (tuple(next_agents), next_execution_states, synced_tasks)


def _pre_step_transition(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord | None,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
) -> AgentExecutionState:
    if execution_state.stage == ExecutionStage.PATROL and active_task is not None:
        return transition_to_task(execution_state, task_id=active_task.task_id)

    if (
        execution_state.stage in {ExecutionStage.GO_TO_TASK, ExecutionStage.ON_TASK}
        and active_task is None
    ):
        patrol_index, return_x, return_y = _nearest_patrol_rejoin_point(
            agent,
            patrol_routes[agent.agent_id],
        )
        return transition_to_return_to_patrol(
            execution_state,
            return_target_x=return_x,
            return_target_y=return_y,
            patrol_waypoint_index=patrol_index,
        )
    return execution_state


def _run_agent_stage(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord | None,
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    dt_seconds: float,
) -> tuple[AgentState, AgentExecutionState]:
    if execution_state.stage == ExecutionStage.ON_TASK:
        confirm_agent = assign_agent_task(
            agent, TaskMode.CONFIRM if agent.kind == "USV" else TaskMode.INVESTIGATE
        )
        return (advance_agent_towards_task(confirm_agent, dt_seconds), execution_state)

    if execution_state.stage == ExecutionStage.GO_TO_TASK and active_task is not None:
        plan = build_direct_line_plan(
            agent,
            goal_x=active_task.target_x,
            goal_y=active_task.target_y,
            planner_name="direct_line_planner",
            task_id=active_task.task_id,
        )
        execution_state = replace(execution_state, active_plan=plan, current_waypoint_index=0)
        advanced_agent, execution_state, outcome = follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
        )
        if outcome == ExecutionOutcome.TASK_SITE_REACHED:
            return (advanced_agent, transition_to_on_task(execution_state))
        return (advanced_agent, execution_state)

    if execution_state.stage == ExecutionStage.RETURN_TO_PATROL:
        assert (
            execution_state.return_target_x is not None
            and execution_state.return_target_y is not None
        )
        plan = build_direct_line_plan(
            agent,
            goal_x=execution_state.return_target_x,
            goal_y=execution_state.return_target_y,
            planner_name="direct_line_planner",
            task_id=None,
        )
        execution_state = replace(execution_state, active_plan=plan, current_waypoint_index=0)
        advanced_agent, execution_state, outcome = follow_path_step(
            agent,
            execution_state,
            dt_seconds=dt_seconds,
        )
        if outcome == ExecutionOutcome.PATROL_REJOINED:
            next_index = (execution_state.patrol_waypoint_index + 1) % len(
                patrol_routes[agent.agent_id]
            )
            return (
                advanced_agent,
                replace(
                    transition_to_patrol(execution_state),
                    patrol_waypoint_index=next_index,
                ),
            )
        return (advanced_agent, execution_state)

    plan = build_fixed_patrol_plan(
        agent,
        patrol_route_id=execution_state.patrol_route_id or agent.agent_id,
        patrol_waypoint_index=execution_state.patrol_waypoint_index,
        patrol_routes=patrol_routes,
    )
    execution_state = replace(execution_state, active_plan=plan, current_waypoint_index=0)
    advanced_agent, execution_state, outcome = follow_path_step(
        agent,
        execution_state,
        dt_seconds=dt_seconds,
    )
    if outcome == ExecutionOutcome.WAYPOINT_REACHED:
        next_index = (execution_state.patrol_waypoint_index + 1) % len(
            patrol_routes[agent.agent_id]
        )
        execution_state = replace(
            execution_state,
            active_plan=None,
            current_waypoint_index=0,
            patrol_waypoint_index=next_index,
        )
    return (advanced_agent, execution_state)


def _assigned_task_for_agent(
    task_records: tuple[TaskRecord, ...],
    agent_id: str,
) -> TaskRecord | None:
    for task in task_records:
        if task.assigned_agent_id != agent_id:
            continue
        if task.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
            return task
    return None


def _sync_task_statuses(
    task_records: tuple[TaskRecord, ...],
    execution_states: dict[str, AgentExecutionState],
) -> tuple[TaskRecord, ...]:
    synced_tasks: list[TaskRecord] = []
    for task in task_records:
        if task.status in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
            synced_tasks.append(task)
            continue
        if task.assigned_agent_id is None:
            synced_tasks.append(task)
            continue

        execution_state = execution_states.get(task.assigned_agent_id)
        if execution_state is None:
            synced_tasks.append(replace(task, status=TaskStatus.REQUEUED, assigned_agent_id=None))
            continue

        if (
            execution_state.active_task_id == task.task_id
            and execution_state.stage == ExecutionStage.ON_TASK
        ):
            synced_tasks.append(replace(task, status=TaskStatus.IN_PROGRESS))
            continue
        if (
            execution_state.active_task_id == task.task_id
            and execution_state.stage == ExecutionStage.GO_TO_TASK
        ):
            synced_tasks.append(replace(task, status=TaskStatus.ASSIGNED))
            continue
        if execution_state.active_task_id is None and execution_state.stage in {
            ExecutionStage.PATROL,
            ExecutionStage.RETURN_TO_PATROL,
        }:
            synced_tasks.append(replace(task, status=TaskStatus.REQUEUED, assigned_agent_id=None))
            continue
        synced_tasks.append(task)
    return tuple(synced_tasks)


def _finalize_task_resolutions(
    *,
    agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    task_records: tuple[TaskRecord, ...],
    patrol_routes: dict[str, tuple[tuple[float, float], ...]],
    info_map: InformationMap,
    step: int,
) -> tuple[tuple[TaskRecord, ...], dict[str, AgentExecutionState]]:
    agent_by_id = {agent.agent_id: agent for agent in agents}
    next_execution_states = dict(execution_states)
    resolved_tasks: list[TaskRecord] = []

    for task in task_records:
        if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
            resolved_tasks.append(task)
            continue
        if task.status == TaskStatus.COMPLETED:
            resolved_tasks.append(task)
            continue

        assert task.target_row is not None and task.target_col is not None
        state = info_map.state_at(task.target_row, task.target_col)
        if state.known_hotspot_state not in {
            HotspotKnowledgeState.CONFIRMED,
            HotspotKnowledgeState.FALSE_ALARM,
        }:
            resolved_tasks.append(task)
            continue

        resolved_tasks.append(
            replace(
                task,
                status=TaskStatus.COMPLETED,
                completed_step=step,
            )
        )
        if task.assigned_agent_id is None:
            continue

        agent = agent_by_id.get(task.assigned_agent_id)
        execution_state = next_execution_states.get(task.assigned_agent_id)
        if agent is None or execution_state is None:
            continue
        if execution_state.stage != ExecutionStage.ON_TASK:
            continue
        patrol_index, return_x, return_y = _nearest_patrol_rejoin_point(
            agent,
            patrol_routes[agent.agent_id],
        )
        next_execution_states[task.assigned_agent_id] = transition_to_return_to_patrol(
            execution_state,
            return_target_x=return_x,
            return_target_y=return_y,
            patrol_waypoint_index=patrol_index,
        )

    return (tuple(resolved_tasks), next_execution_states)


def _nearest_patrol_rejoin_point(
    agent: AgentState,
    patrol_route: tuple[tuple[float, float], ...],
) -> tuple[int, float, float]:
    indexed_points = [(index, point[0], point[1]) for index, point in enumerate(patrol_route)]
    index, x, y = min(
        indexed_points,
        key=lambda item: (agent.x - item[1]) ** 2 + (agent.y - item[2]) ** 2,
    )
    return (index, x, y)


def _serialize_task_assignment(assignment: TaskAssignment) -> dict[str, object]:
    return {
        "task_id": assignment.task_id,
        "task_type": "hotspot_confirmation",
        "selected_agent": assignment.agent_id,
        "candidate_agents": [{"agent_id": assignment.agent_id}],
        "selection_reason": assignment.selection_reason,
        "selection_score": round(assignment.selection_score, 3),
    }


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
            (agent_id, tuple(points)) for agent_id, points in sorted(trajectories.items())
        ),
    )
