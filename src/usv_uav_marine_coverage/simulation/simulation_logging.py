"""Structured logging helpers for the replay-style simulation stage."""

from __future__ import annotations

import json
from dataclasses import dataclass
from math import atan2, degrees, hypot
from pathlib import Path
from typing import TYPE_CHECKING

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    TaskMode,
    shortest_heading_delta_deg,
)
from usv_uav_marine_coverage.environment import ObstacleLayout, SeaMap
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionStage,
    UavCoverageState,
)
from usv_uav_marine_coverage.execution.task_claim_runtime import (
    find_task_by_id,
    select_claimable_task_for_agent,
)
from usv_uav_marine_coverage.grid import GridCoverageMap
from usv_uav_marine_coverage.information_map import HotspotKnowledgeState, InformationMap
from usv_uav_marine_coverage.planning.astar_path_planner import PlannerMetricsSnapshot
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskStatus, TaskType
from usv_uav_marine_coverage.simulation.replay_validation import (
    build_step_validation_layer,
    summarize_replay_validation,
)

if TYPE_CHECKING:
    from .simulation_core import SimulationFrame, SimulationReplay


@dataclass(frozen=True)
class SimulationArtifacts:
    """Written simulation outputs for one replay run."""

    html_path: Path | None
    events_path: Path
    summary_path: Path


def derive_log_paths(output_path: Path) -> tuple[Path, Path]:
    """Return the companion log paths for one replay HTML output."""

    events_path = output_path.with_name(f"{output_path.stem}_events.jsonl")
    summary_path = output_path.with_name(f"{output_path.stem}_summary.json")
    return (events_path, summary_path)


def write_events_jsonl(replay: SimulationReplay, output_path: Path) -> None:
    """Write machine-readable event records for one replay run."""

    records: list[dict[str, object]] = [
        {
            "record_type": "simulation_metadata",
            "seed": replay.seed,
            "steps": replay.frames[-1].step,
            "dt_seconds": replay.dt_seconds,
            "algorithm_stage": "replay_preview",
            "environment_model": "three_zone_continuous_sea",
            "grid_model": "25m_rectangular_grid",
            "experiment_config": replay.experiment_config,
        },
        {
            "record_type": "environment_snapshot",
            "sea_map": build_sea_map_snapshot(replay.sea_map),
            "obstacle_layout": build_obstacle_layout_snapshot(replay.obstacle_layout),
        },
        {
            "record_type": "initial_agents",
            "agents": [serialize_initial_agent(agent) for agent in replay.initial_agents],
        },
    ]
    records.extend(replay.step_logs)
    records.append(
        {
            "record_type": "final_summary",
            **build_summary_payload(replay),
        }
    )
    with output_path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def write_summary_json(replay: SimulationReplay, output_path: Path) -> None:
    """Write the final summary payload for one replay run."""

    output_path.write_text(
        json.dumps(build_summary_payload(replay), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def build_summary_payload(replay: SimulationReplay) -> dict[str, object]:
    """Build the final summary payload for one replay run."""

    final_frame = replay.frames[-1]
    final_step_log = replay.step_logs[-1]
    final_coverage = final_step_log["coverage"]
    validation_summary = summarize_replay_validation(replay.step_logs)
    return {
        "simulation": {
            "seed": replay.seed,
            "steps": replay.frames[-1].step,
            "dt_seconds": replay.dt_seconds,
            "algorithm_stage": "replay_preview",
            "experiment_config": replay.experiment_config,
        },
        "environment": {
            "sea_map": build_sea_map_snapshot(replay.sea_map),
            "obstacle_layout": build_obstacle_layout_snapshot(replay.obstacle_layout),
        },
        "initial_agents": [serialize_initial_agent(agent) for agent in replay.initial_agents],
        "final_metrics": {
            "coverage_ratio": round(final_frame.coverage_ratio, 6),
            "covered_cells": len(final_frame.covered_cells),
            "total_coverable_cells": int(final_coverage["total_coverable_cells"]),
            "valid_cells": len(final_frame.valid_cells),
            "stale_cells": len(final_frame.stale_cells),
            "active_hotspot_cells": len(final_frame.pending_hotspot_cells)
            + len(final_frame.uav_checked_cells),
            "uav_checked_cells": len(final_frame.uav_checked_cells),
            "active_ground_truth_hotspots": count_active_ground_truth_hotspots(replay.step_logs),
            "path_lengths_by_agent": build_path_lengths(final_frame),
            "event_totals": build_event_totals(replay.step_logs),
            "last_step_events": list(final_step_log["events"]),
        },
        "validation": validation_summary,
    }


def build_sea_map_snapshot(sea_map: SeaMap) -> dict[str, object]:
    """Serialize the current sea-map geometry for logs."""

    return {
        "width": sea_map.width,
        "height": sea_map.height,
        "zones": [
            {"name": zone.name, "x_start": zone.x_start, "x_end": zone.x_end}
            for zone in sea_map.zones
        ],
    }


def build_obstacle_layout_snapshot(layout: ObstacleLayout) -> dict[str, object]:
    """Serialize the current obstacle layout for logs."""

    return {
        "seed": layout.seed,
        "generation_attempts": layout.generation_attempts,
        "risk_zone_obstacles": [
            {
                "name": obstacle.name,
                "zone_name": obstacle.zone_name,
                "points": [[x, y] for x, y in obstacle.points],
            }
            for obstacle in layout.risk_zone_obstacles
        ],
        "offshore_features": [
            {
                "name": feature.name,
                "feature_type": feature.feature_type,
                "x": feature.x,
                "y": feature.y,
                "radius": feature.radius,
            }
            for feature in layout.offshore_features
        ],
        "nearshore_monitor_points": [
            {
                "name": target.name,
                "x": target.x,
                "y": target.y,
            }
            for target in layout.nearshore_monitor_points
        ],
        "offshore_hotspots": [
            {
                "name": target.name,
                "x": target.x,
                "y": target.y,
            }
            for target in layout.offshore_hotspots
        ],
    }


def serialize_initial_agent(agent: AgentState) -> dict[str, object]:
    """Serialize one initial agent state for logs."""

    return {
        "agent_id": agent.agent_id,
        "kind": agent.kind,
        "x": round(agent.x, 3),
        "y": round(agent.y, 3),
        "heading_deg": round(agent.heading_deg, 3),
        "speed_mps": round(agent.speed_mps, 3),
        "detection_radius": agent.detection_radius,
        "coverage_radius": agent.coverage_radius,
        "cruise_speed_mps": agent.cruise_speed_mps,
        "max_speed_mps": agent.max_speed_mps,
        "max_turn_rate_degps": agent.max_turn_rate_degps,
        "energy_capacity": round(agent.energy_capacity, 3),
        "energy_level": round(agent.energy_level, 3),
        "energy_ratio": round(agent.energy_level / agent.energy_capacity, 6)
        if agent.energy_capacity > 0.0
        else None,
        "task_mode": agent.task.mode.value,
        "available": agent.is_operational,
        "health_status": agent.health_status.value,
        "speed_multiplier": round(agent.speed_multiplier, 3),
        "turn_rate_multiplier": round(agent.turn_rate_multiplier, 3),
    }


def serialize_agent_step(agent: AgentState) -> dict[str, object]:
    """Serialize one per-step agent state for logs."""

    return {
        "agent_id": agent.agent_id,
        "kind": agent.kind,
        "x": round(agent.x, 3),
        "y": round(agent.y, 3),
        "heading_deg": round(agent.heading_deg, 3),
        "speed_mps": round(agent.speed_mps, 3),
        "turn_rate_degps": round(agent.turn_rate_degps, 3),
        "energy_level": round(agent.energy_level, 3),
        "energy_ratio": round(agent.energy_level / agent.energy_capacity, 6)
        if agent.energy_capacity > 0.0
        else None,
        "health_status": agent.health_status.value,
        "speed_multiplier": round(agent.speed_multiplier, 3),
        "turn_rate_multiplier": round(agent.turn_rate_multiplier, 3),
        "is_operational": agent.is_operational,
        "task_mode": agent.task.mode.value,
        "target_x": None if agent.task.target_x is None else round(agent.task.target_x, 3),
        "target_y": None if agent.task.target_y is None else round(agent.task.target_y, 3),
        "has_target": agent.task.has_target,
    }


def serialize_index_mapping(
    mapping: dict[str, tuple[tuple[int, int], ...]],
) -> dict[str, list[list[int]]]:
    """Serialize an agent -> grid indices mapping for logs."""

    return {
        agent_id: [[row, col] for row, col in indices]
        for agent_id, indices in sorted(mapping.items())
        if indices
    }


def build_step_log(
    *,
    step: int,
    agents: tuple[AgentState, ...],
    info_map: InformationMap,
    coverage_map: GridCoverageMap,
    events: tuple[str, ...],
    observed_by_agent: dict[str, tuple[tuple[int, int], ...]],
    spawned_hotspots: tuple[tuple[int, int], ...],
    newly_stale_cells: tuple[tuple[int, int], ...],
    detected_by_agent: dict[str, tuple[tuple[int, int], ...]],
    confirmations_by_agent: dict[str, tuple[tuple[int, int], ...]],
    task_decisions: tuple[dict[str, object], ...],
    prior_agents: tuple[AgentState, ...],
    planned_agents: tuple[AgentState, ...],
    task_records: tuple[TaskRecord, ...],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    uav_coverage_states: dict[str, UavCoverageState] | None,
    planner_metrics: PlannerMetricsSnapshot,
    failure_events: tuple[dict[str, object], ...] = (),
    reassignments: tuple[dict[str, object], ...] = (),
    wreck_zones: tuple[dict[str, object], ...] = (),
    failed_agent_ids: tuple[str, ...] = (),
) -> dict[str, object]:
    """Build one structured per-step simulation log record."""

    valid_cells = 0
    stale_known_cells = 0
    active_hotspot_cells = 0
    uav_checked_cells = 0
    active_ground_truth_hotspots = 0
    for cell in info_map.grid_map.flat_cells:
        state = info_map.state_at(cell.row, cell.col)
        if state.validity.value == "valid":
            valid_cells += 1
        else:
            stale_known_cells += 1
        if state.ground_truth_hotspot:
            active_ground_truth_hotspots += 1
        if state.ground_truth_hotspot and state.known_hotspot_state == HotspotKnowledgeState.NONE:
            active_hotspot_cells += 1
        if state.known_hotspot_state == HotspotKnowledgeState.UAV_CHECKED:
            uav_checked_cells += 1
            active_hotspot_cells += 1

    task_assignments = build_task_assignment_logs(
        agents,
        task_records=task_records,
        execution_states=execution_states,
        progress_states=progress_states,
        info_map=info_map,
    )

    observed_union = sorted({index for indices in observed_by_agent.values() for index in indices})
    path_plans = build_path_plan_logs(
        info_map.grid_map,
        prior_agents=prior_agents,
        planned_agents=planned_agents,
        execution_states=execution_states,
        uav_coverage_states=uav_coverage_states,
        step=step,
    )
    execution_updates = build_execution_updates(
        agents,
        step=step,
        execution_states=execution_states,
        progress_states=progress_states,
        uav_coverage_states=uav_coverage_states,
    )
    hotspot_chain = build_hotspot_chain_updates(
        info_map,
        detected_by_agent=detected_by_agent,
        confirmations_by_agent=confirmations_by_agent,
    )
    step_log = {
        "record_type": "step_snapshot",
        "step": step,
        "agent_states": [serialize_agent_step(agent) for agent in agents],
        "observation_updates": {
            "observed_cells": [[row, col] for row, col in observed_union],
            "observed_by_agent": serialize_index_mapping(observed_by_agent),
            "newly_stale_cells": [[row, col] for row, col in newly_stale_cells],
            "spawned_hotspots": [[row, col] for row, col in spawned_hotspots],
        },
        "information_map": {
            "valid_cells": valid_cells,
            "stale_known_cells": stale_known_cells,
            "active_hotspot_cells": active_hotspot_cells,
            "active_ground_truth_hotspots": active_ground_truth_hotspots,
            "uav_checked_cells": uav_checked_cells,
            "pending_hotspots": _serialize_active_hotspots(
                info_map, state_filter=HotspotKnowledgeState.NONE
            ),
            "uav_checked_hotspots": _serialize_active_hotspots(
                info_map, state_filter=HotspotKnowledgeState.UAV_CHECKED
            ),
        },
        "task_layer": {
            "task_assignments": task_assignments,
            "task_decisions": list(task_decisions),
            "tasks": [
                serialize_task_record(
                    task,
                    step=step,
                    info_map=info_map,
                    execution_states=execution_states,
                    progress_states=progress_states,
                )
                for task in task_records
            ],
            "task_rejections": [],
            "newly_uav_checked_by_agent": serialize_index_mapping(detected_by_agent),
            "confirmed_by_agent": serialize_index_mapping(confirmations_by_agent),
        },
        "path_layer": {
            "path_plans": path_plans,
            "planner_metrics": {
                "total_calls": planner_metrics.total_calls,
                "planned_calls": planner_metrics.planned_calls,
                "blocked_calls": planner_metrics.blocked_calls,
                "expanded_nodes": planner_metrics.expanded_nodes,
                "max_expanded_nodes": planner_metrics.max_expanded_nodes,
                "by_context": planner_metrics.by_context,
            },
        },
        "execution_layer": execution_updates,
        "hotspot_chain": hotspot_chain,
        "failure_recovery": {
            "failure_events": list(failure_events),
            "reassignments": list(reassignments),
            "recovery_steps": None if not reassignments else step,
            "wreck_zones": list(wreck_zones),
            "failed_agent_ids": list(failed_agent_ids),
        },
        "coverage": {
            "covered_cells": coverage_map.covered_cells,
            "total_coverable_cells": coverage_map.total_coverable_cells,
            "coverage_ratio": round(coverage_map.covered_ratio(), 6),
        },
        "events": list(events),
    }
    step_log["validation_layer"] = build_step_validation_layer(step_log)
    return step_log


def count_active_ground_truth_hotspots(step_logs: tuple[dict[str, object], ...]) -> int:
    """Count active ground-truth hotspots from the last step log."""

    if not step_logs:
        return 0
    info_map_snapshot = step_logs[-1]["information_map"]
    return int(info_map_snapshot["active_ground_truth_hotspots"])


def build_event_totals(step_logs: tuple[dict[str, object], ...]) -> dict[str, int]:
    """Aggregate event totals across all step logs."""

    totals = {
        "spawned_hotspots": 0,
        "uav_checked_marks": 0,
        "confirmed_hotspots": 0,
        "astar_total_calls": 0,
        "astar_planned_calls": 0,
        "astar_blocked_calls": 0,
        "astar_expanded_nodes": 0,
        "astar_max_expanded_nodes": 0,
        "failure_events": 0,
        "task_reassignments": 0,
    }
    for record in step_logs:
        observation_updates = record["observation_updates"]
        task_layer = record["task_layer"]
        path_layer = record["path_layer"]
        totals["spawned_hotspots"] += len(observation_updates["spawned_hotspots"])
        totals["uav_checked_marks"] += sum(
            len(indices) for indices in task_layer["newly_uav_checked_by_agent"].values()
        )
        totals["confirmed_hotspots"] += sum(
            len(indices) for indices in task_layer["confirmed_by_agent"].values()
        )
        planner_metrics = path_layer.get("planner_metrics", {})
        failure_recovery = record.get("failure_recovery", {})
        totals["astar_total_calls"] += int(planner_metrics.get("total_calls", 0))
        totals["astar_planned_calls"] += int(planner_metrics.get("planned_calls", 0))
        totals["astar_blocked_calls"] += int(planner_metrics.get("blocked_calls", 0))
        totals["astar_expanded_nodes"] += int(planner_metrics.get("expanded_nodes", 0))
        totals["astar_max_expanded_nodes"] = max(
            totals["astar_max_expanded_nodes"],
            int(planner_metrics.get("max_expanded_nodes", 0)),
        )
        totals["failure_events"] += len(failure_recovery.get("failure_events", []))
        totals["task_reassignments"] += len(failure_recovery.get("reassignments", []))
    return totals


def build_path_plan_logs(
    grid_map,
    *,
    prior_agents: tuple[AgentState, ...],
    planned_agents: tuple[AgentState, ...],
    execution_states: dict[str, AgentExecutionState],
    uav_coverage_states: dict[str, UavCoverageState] | None,
    step: int,
) -> list[dict[str, object]]:
    """Build path-plan summary records for the current step."""

    prior_by_id = {agent.agent_id: agent for agent in prior_agents}
    plan_logs: list[dict[str, object]] = []
    for agent in planned_agents:
        prior_agent = prior_by_id.get(agent.agent_id, agent)
        execution_state = execution_states.get(agent.agent_id)
        replan_reason = "continue_current_target"
        active_task_id = None if execution_state is None else execution_state.active_task_id
        if (
            prior_agent.task.target_x != agent.task.target_x
            or prior_agent.task.target_y != agent.task.target_y
        ):
            if active_task_id is not None and active_task_id.startswith("uav-resupply-"):
                replan_reason = "uav_resupply_rendezvous"
            elif agent.task.mode == TaskMode.CONFIRM and active_task_id is not None:
                if active_task_id.startswith("baseline-service-"):
                    replan_reason = "baseline_service_response"
                else:
                    replan_reason = "suspected_hotspot_response"
            elif agent.task.has_target:
                replan_reason = "patrol_waypoint_update"
            else:
                replan_reason = "task_cleared"
        start_cell = grid_map.locate_cell(agent.x, agent.y)
        goal_cell = None
        estimated_cost = None
        planner_name = None
        plan_status = None
        task_id = None
        coverage_state = (
            None if uav_coverage_states is None else uav_coverage_states.get(agent.agent_id)
        )
        if execution_state is not None and execution_state.active_plan is not None:
            goal = grid_map.locate_cell(
                execution_state.active_plan.goal_x,
                execution_state.active_plan.goal_y,
            )
            goal_cell = [goal.row, goal.col]
            estimated_cost = execution_state.active_plan.estimated_cost
            planner_name = execution_state.active_plan.planner_name
            plan_status = execution_state.active_plan.status.value
            task_id = execution_state.active_plan.task_id
        plan_logs.append(
            {
                "plan_id": f"step-{step}-{agent.agent_id}",
                "agent_id": agent.agent_id,
                "task_id": task_id,
                "planner_name": planner_name,
                "start_cell": [start_cell.row, start_cell.col],
                "goal_cell": goal_cell,
                "path_mode": agent.task.mode.value,
                "planning_success": execution_state is not None
                and execution_state.active_plan is not None,
                "estimated_cost": estimated_cost,
                "path_length_estimate": estimated_cost,
                "replan_reason": replan_reason,
                "plan_status": plan_status,
                "patrol_region_id": None
                if coverage_state is None
                else coverage_state.current_region_id,
                "region_replan_reason": None
                if coverage_state is None
                else coverage_state.last_replan_reason,
            }
        )
    return plan_logs


def build_execution_updates(
    agents: tuple[AgentState, ...],
    *,
    step: int,
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    uav_coverage_states: dict[str, UavCoverageState] | None,
) -> dict[str, object]:
    """Build execution-deviation snapshots for the current step."""

    execution_updates = []
    for agent in agents:
        execution_state = execution_states.get(agent.agent_id)
        progress_state = progress_states.get(agent.agent_id)
        coverage_state = (
            None if uav_coverage_states is None else uav_coverage_states.get(agent.agent_id)
        )
        tracking_error = None
        heading_error = None
        if (
            agent.task.has_target
            and agent.task.target_x is not None
            and agent.task.target_y is not None
        ):
            tracking_error = round(
                hypot(agent.task.target_x - agent.x, agent.task.target_y - agent.y), 3
            )
            target_heading_deg = degrees(
                atan2(agent.task.target_y - agent.y, agent.task.target_x - agent.x)
            )
            heading_error = round(
                abs(shortest_heading_delta_deg(agent.heading_deg, target_heading_deg)),
                3,
            )
        execution_updates.append(
            {
                "agent_id": agent.agent_id,
                "execution_stage": None if execution_state is None else execution_state.stage.value,
                "active_task_id": None
                if execution_state is None
                else execution_state.active_task_id,
                "reference_target": None
                if not agent.task.has_target
                else {
                    "x": round(agent.task.target_x, 3),
                    "y": round(agent.task.target_y, 3),
                },
                "actual_position": {
                    "x": round(agent.x, 3),
                    "y": round(agent.y, 3),
                },
                "tracking_error": tracking_error,
                "speed_error": None
                if not agent.task.has_target
                else round(abs(agent.cruise_speed_mps - agent.speed_mps), 3),
                "heading_error": heading_error,
                "stalled_steps": None if progress_state is None else progress_state.stalled_steps,
                "recovery_attempts": None
                if progress_state is None
                else progress_state.recovery_attempts,
                "cooldown_until_step": None
                if progress_state is None
                else progress_state.cooldown_until_step,
                "blocked_goal_signature": None
                if progress_state is None
                else progress_state.blocked_goal_signature,
                "task_final_approach_task_id": None
                if progress_state is None
                else progress_state.task_final_approach_task_id,
                "task_final_approach_candidate_index": None
                if progress_state is None or progress_state.task_final_approach_candidate_index < 0
                else progress_state.task_final_approach_candidate_index,
                "task_final_approach_candidate_x": None
                if progress_state is None or progress_state.task_final_approach_candidate_x is None
                else round(progress_state.task_final_approach_candidate_x, 3),
                "task_final_approach_candidate_y": None
                if progress_state is None or progress_state.task_final_approach_candidate_y is None
                else round(progress_state.task_final_approach_candidate_y, 3),
                "task_final_approach_attempt_count": None
                if progress_state is None
                else progress_state.task_final_approach_attempt_count,
                "task_final_approach_status": None
                if progress_state is None
                else progress_state.task_final_approach_status,
                "released_task_id": None
                if progress_state is None or progress_state.released_task_step != step
                else progress_state.released_task_id,
                "released_task_created_step": None
                if progress_state is None or progress_state.released_task_step != step
                else progress_state.released_task_created_step,
                "released_task_retry_until_step": None
                if (
                    progress_state is None
                    or progress_state.released_task_step != step
                    or progress_state.released_task_retry_until_step <= 0
                )
                else progress_state.released_task_retry_until_step,
                "released_task_reason": None
                if progress_state is None or progress_state.released_task_step != step
                else progress_state.released_task_reason,
                "pending_assigned_task_id": None
                if progress_state is None
                else progress_state.pending_assigned_task_id,
                "claimed_task_id": None
                if progress_state is None
                else progress_state.claimed_task_id,
                "claim_transition_reason": None
                if progress_state is None
                else progress_state.claim_transition_reason,
                "last_return_plan_step": None
                if execution_state is None
                else execution_state.last_return_plan_step,
                "return_target_source": None
                if execution_state is None
                else execution_state.return_target_source,
                "yield_reason": None if execution_state is None else execution_state.yield_reason,
                "reserved_corridor_name": None
                if execution_state is None
                else execution_state.reserved_corridor_name,
                "corridor_owner_agent_id": None
                if execution_state is None
                else execution_state.corridor_owner_agent_id,
                "reserved_bottleneck_zone_id": None
                if execution_state is None
                else execution_state.reserved_bottleneck_zone_id,
                "bottleneck_owner_agent_id": None
                if execution_state is None
                else execution_state.bottleneck_owner_agent_id,
                "last_patrol_plan_step": None
                if execution_state is None
                else execution_state.last_patrol_plan_step,
                "patrol_region_id": None
                if coverage_state is None
                else coverage_state.current_region_id,
                "region_replan_reason": None
                if coverage_state is None
                else coverage_state.last_replan_reason,
                "entered_recovery": False
                if execution_state is None or progress_state is None
                else (
                    execution_state.stage.value == "recovery"
                    and progress_state.recovery_step_index == 0
                ),
            }
        )
    return {"tracking_updates": execution_updates}


def build_task_assignment_logs(
    agents: tuple[AgentState, ...],
    *,
    task_records: tuple[TaskRecord, ...],
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
    info_map: InformationMap,
) -> list[dict[str, object]]:
    """Build agent-centric task assignment snapshots for logs."""

    assignment_logs = []
    for agent in agents:
        execution_state = execution_states.get(agent.agent_id)
        progress_state = progress_states.get(agent.agent_id)
        assigned_task = None
        if execution_state is not None:
            assigned_task = find_task_by_id(task_records, execution_state.active_task_id)
        if assigned_task is None:
            assigned_task = select_claimable_task_for_agent(
                task_records,
                agent_id=agent.agent_id,
                execution_state=execution_state,
            )
        assignment_logs.append(
            {
                "agent_id": agent.agent_id,
                "mode": agent.task.mode.value,
                "execution_stage": None if execution_state is None else execution_state.stage.value,
                "active_task_id": None if assigned_task is None else assigned_task.task_id,
                "hotspot_id": (
                    None
                    if assigned_task is None
                    else _task_hotspot_id(assigned_task, info_map=info_map)
                ),
                "support_agent_id": None
                if assigned_task is None
                else assigned_task.support_agent_id,
                "task_status": None if assigned_task is None else assigned_task.status.value,
                "pending_assigned_task_id": None
                if progress_state is None
                else progress_state.pending_assigned_task_id,
                "claimed_task_id": None
                if progress_state is None
                else progress_state.claimed_task_id,
                "claim_transition_reason": None
                if progress_state is None
                else progress_state.claim_transition_reason,
                "target_x": None if agent.task.target_x is None else round(agent.task.target_x, 3),
                "target_y": None if agent.task.target_y is None else round(agent.task.target_y, 3),
                "energy_level": round(agent.energy_level, 3),
                "health_status": agent.health_status.value,
                "is_operational": agent.is_operational,
            }
        )
    return assignment_logs


def serialize_task_record(
    task: TaskRecord,
    *,
    step: int,
    info_map: InformationMap,
    execution_states: dict[str, AgentExecutionState],
    progress_states: dict[str, AgentProgressState],
) -> dict[str, object]:
    """Serialize one task record for logs."""

    claim_status = "released"
    release_reason = None
    owner_agent_id = task.assigned_agent_id
    if task.status not in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}:
        if owner_agent_id is None:
            claim_status = "unclaimed"
        else:
            execution_state = execution_states.get(owner_agent_id)
            progress_state = progress_states.get(owner_agent_id)
            if execution_state is not None and execution_state.active_task_id == task.task_id:
                if _task_claim_is_executing(
                    task,
                    execution_state=execution_state,
                    execution_states=execution_states,
                ):
                    claim_status = "executing"
                else:
                    claim_status = "claimed"
            else:
                claim_status = "pending_claim"
            if (
                progress_state is not None
                and progress_state.released_task_id == task.task_id
                and progress_state.released_task_step == step
                and (
                    progress_state.released_task_created_step is None
                    or progress_state.released_task_created_step == task.created_step
                )
            ):
                release_reason = progress_state.released_task_reason
    else:
        release_reason = "completed" if task.status == TaskStatus.COMPLETED else "cancelled"

    return {
        "task_id": task.task_id,
        "hotspot_id": _task_hotspot_id(task, info_map=info_map),
        "task_type": task.task_type.value,
        "source": task.source.value,
        "status": task.status.value,
        "owner_agent_id": owner_agent_id,
        "claim_status": claim_status,
        "release_reason": release_reason,
        "priority": task.priority,
        "target_x": round(task.target_x, 3),
        "target_y": round(task.target_y, 3),
        "target_row": task.target_row,
        "target_col": task.target_col,
        "created_step": task.created_step,
        "assigned_agent_id": task.assigned_agent_id,
        "support_agent_id": task.support_agent_id,
        "completed_step": task.completed_step,
        "retry_after_step": task.retry_after_step,
        "suppression_grace_until_step": task.suppression_grace_until_step,
        "agent_retry_after_steps": [
            {"agent_id": agent_id, "retry_after_step": retry_after_step}
            for agent_id, retry_after_step in task.agent_retry_after_steps
        ],
        "distributed_winner_memories": [
            {
                "observer_agent_id": observer_agent_id,
                "winner_agent_id": winner_agent_id,
                "known_step": known_step,
            }
            for observer_agent_id, winner_agent_id, known_step in task.distributed_winner_memories
        ],
    }


def _task_claim_is_executing(
    task: TaskRecord,
    *,
    execution_state: AgentExecutionState,
    execution_states: dict[str, AgentExecutionState],
) -> bool:
    if task.task_type != TaskType.UAV_RESUPPLY:
        return execution_state.stage in {ExecutionStage.ON_TASK, ExecutionStage.ON_RECHARGE}
    if execution_state.stage != ExecutionStage.ON_RECHARGE or task.support_agent_id is None:
        return False
    support_execution_state = execution_states.get(task.support_agent_id)
    if support_execution_state is None:
        return False
    return (
        support_execution_state.active_task_id == task.task_id
        and support_execution_state.stage == ExecutionStage.ON_TASK
    )


def _task_hotspot_id(task: TaskRecord, *, info_map: InformationMap) -> int | None:
    if task.task_type != TaskType.HOTSPOT_CONFIRMATION:
        return None
    state = info_map.state_at(task.target_row, task.target_col)
    if state.known_hotspot_id is not None:
        return state.known_hotspot_id
    return state.ground_truth_hotspot_id


def _serialize_active_hotspots(
    info_map: InformationMap,
    *,
    state_filter: HotspotKnowledgeState,
) -> list[dict[str, object]]:
    serialized: list[dict[str, object]] = []
    for cell in info_map.grid_map.flat_cells:
        state = info_map.state_at(cell.row, cell.col)
        if state_filter == HotspotKnowledgeState.NONE:
            if not (
                state.ground_truth_hotspot
                and state.known_hotspot_state == HotspotKnowledgeState.NONE
            ):
                continue
            hotspot_id = state.ground_truth_hotspot_id
        else:
            if state.known_hotspot_state != state_filter:
                continue
            hotspot_id = (
                state.known_hotspot_id
                if state.known_hotspot_id is not None
                else state.ground_truth_hotspot_id
            )
        serialized.append(
            {
                "hotspot_id": hotspot_id,
                "cell": [cell.row, cell.col],
                "x": round(cell.center_x, 3),
                "y": round(cell.center_y, 3),
            }
        )
    return serialized


def build_hotspot_chain_updates(
    info_map: InformationMap,
    *,
    detected_by_agent: dict[str, tuple[tuple[int, int], ...]],
    confirmations_by_agent: dict[str, tuple[tuple[int, int], ...]],
) -> list[dict[str, object]]:
    """Build hotspot state-transition records for the current step."""

    updates: list[dict[str, object]] = []
    for agent_id, indices in sorted(detected_by_agent.items()):
        for row, col in indices:
            state = info_map.state_at(row, col)
            updates.append(
                {
                    "hotspot_id": state.ground_truth_hotspot_id,
                    "cell": [row, col],
                    "uav_checked_by": agent_id,
                    "known_state_before": "none",
                    "known_state_after": state.known_hotspot_state.value,
                    "ground_truth_state": state.ground_truth_hotspot,
                    "final_resolution": None,
                }
            )
    for agent_id, indices in sorted(confirmations_by_agent.items()):
        for row, col in indices:
            state = info_map.state_at(row, col)
            updates.append(
                {
                    "hotspot_id": state.ground_truth_hotspot_id,
                    "cell": [row, col],
                    "confirmed_by": agent_id,
                    "known_state_before": "uav_checked",
                    "known_state_after": "confirmed",
                    "ground_truth_state": state.ground_truth_hotspot,
                    "final_resolution": "confirmed",
                }
            )
    return updates


def build_path_lengths(frame: SimulationFrame) -> dict[str, float]:
    """Compute the cumulative path length for each agent trajectory."""

    path_lengths: dict[str, float] = {}
    for agent_id, points in frame.trajectories:
        length = 0.0
        for start, end in zip(points, points[1:], strict=False):
            length += ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
        path_lengths[agent_id] = round(length, 3)
    return path_lengths
