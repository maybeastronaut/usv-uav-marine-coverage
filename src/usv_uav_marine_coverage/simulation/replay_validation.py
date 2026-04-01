"""Replay validation helpers for step invariants and flip-flop detection."""

from __future__ import annotations

import json
from math import hypot
from pathlib import Path
from typing import Any

_ACTIVE_TASK_STATUSES = {"assigned", "in_progress"}
_HOTSPOT_FLIPFLOP_ACTIVE_STATUSES = {"assigned", "in_progress"}
_HOTSPOT_FLIPFLOP_RELEASE_STATUSES = {"pending", "requeued"}
_RESUPPLY_OVERLAP_EPSILON = 1e-6
_FLIP_FLOP_WINDOW_STEPS = 6
_RETURN_RECOVERY_LOOP_WINDOW_STEPS = 48
_RETURN_RECOVERY_LOOP_MIN_ENTRIES = 4
_APPROACH_SIDE_LOOP_WINDOW_STEPS = 80
_APPROACH_SIDE_LOOP_MIN_ENTRIES = 3
_PSEUDO_PROGRESS_WINDOW_STEPS = 20
_PSEUDO_PROGRESS_MAX_BBOX_DIAGONAL_M = 12.0
_PSEUDO_PROGRESS_MIN_DISTANCE_GAIN_M = 6.0


def load_step_snapshots_from_events_jsonl(path: Path) -> list[dict[str, Any]]:
    """Load all step-snapshot records from one replay events log."""

    step_records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get("record_type") == "step_snapshot":
            step_records.append(record)
    return step_records


def build_step_validation_layer(step_snapshot: dict[str, Any]) -> dict[str, Any]:
    """Build one machine-readable validation section for a step snapshot."""

    violations = validate_step_snapshot(step_snapshot)
    return {
        "violation_count": len(violations),
        "violations": violations,
    }


def summarize_replay_validation(step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize replay-level validation findings across all step snapshots."""

    validations = [
        record.get("validation_layer") or build_step_validation_layer(record)
        for record in step_snapshots
    ]
    steps_with_violations = [
        int(record["step"])
        for record, validation in zip(step_snapshots, validations, strict=False)
        if int(validation.get("violation_count", 0)) > 0
    ]
    flip_flops = detect_task_status_flip_flops(step_snapshots)
    silent_requeues = detect_requeued_without_release_reason(step_snapshots)
    return_recovery_loops = detect_taskless_return_recovery_loops(step_snapshots)
    approach_side_loops = detect_task_approach_same_side_recovery_loops(step_snapshots)
    approach_side_switches = detect_task_approach_side_switches_without_recovery(step_snapshots)
    pseudo_progress_loops = detect_pseudo_progress_loops(step_snapshots)
    return {
        "step_violation_count": sum(int(validation.get("violation_count", 0)) for validation in validations),
        "steps_with_violations": steps_with_violations,
        "flip_flop_count": len(flip_flops),
        "flip_flops": flip_flops,
        "silent_requeue_count": len(silent_requeues),
        "silent_requeues": silent_requeues,
        "taskless_return_recovery_loop_count": len(return_recovery_loops),
        "taskless_return_recovery_loops": return_recovery_loops,
        "task_approach_same_side_recovery_loop_count": len(approach_side_loops),
        "task_approach_same_side_recovery_loops": approach_side_loops,
        "task_approach_side_switch_without_recovery_count": len(approach_side_switches),
        "task_approach_side_switches_without_recovery": approach_side_switches,
        "pseudo_progress_loop_count": len(pseudo_progress_loops),
        "pseudo_progress_loops": pseudo_progress_loops,
        "has_errors": bool(
            steps_with_violations
            or flip_flops
            or silent_requeues
            or return_recovery_loops
            or approach_side_loops
            or approach_side_switches
            or pseudo_progress_loops
        ),
    }


def validate_step_snapshot(step_snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    """Validate one serialized step snapshot against key execution invariants."""

    step = int(step_snapshot["step"])
    agent_states = {
        str(agent["agent_id"]): agent for agent in step_snapshot.get("agent_states", [])
    }
    execution_updates = {
        str(update["agent_id"]): update
        for update in step_snapshot.get("execution_layer", {}).get("tracking_updates", [])
    }
    task_assignments = {
        str(row["agent_id"]): row
        for row in step_snapshot.get("task_layer", {}).get("task_assignments", [])
    }
    tasks = step_snapshot.get("task_layer", {}).get("tasks", [])
    task_type_by_id = {
        str(task["task_id"]): str(task["task_type"]) for task in tasks
    }
    violations: list[dict[str, Any]] = []

    for task in tasks:
        task_id = str(task["task_id"])
        task_type = str(task["task_type"])
        status = str(task["status"])
        assigned_agent_id = task.get("assigned_agent_id")
        support_agent_id = task.get("support_agent_id")
        claim_status = task.get("claim_status")

        if status in _ACTIVE_TASK_STATUSES and assigned_agent_id is None:
            violations.append(
                _build_violation(
                    step,
                    code="active_task_missing_owner",
                    task_id=task_id,
                    detail=f"{task_type} task is {status} without assigned_agent_id",
                )
            )

        if claim_status == "executing" and status != "in_progress":
            violations.append(
                _build_violation(
                    step,
                    code="executing_task_not_in_progress",
                    task_id=task_id,
                    detail=f"{task_type} task has claim_status=executing but status={status}",
                )
            )

        if task_type == "uav_resupply" and status in _ACTIVE_TASK_STATUSES and support_agent_id is None:
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_missing_support",
                    task_id=task_id,
                    detail="uav_resupply task is active without support_agent_id",
                )
            )
        if (
            task_type == "uav_resupply"
            and support_agent_id is None
            and claim_status in {"claimed", "executing"}
        ):
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_supportless_claim",
                    task_id=task_id,
                    detail=(
                        "uav_resupply task has no support_agent_id, but claim_status still "
                        f"reports {claim_status}"
                    ),
                )
            )

        owner_execution = (
            None
            if assigned_agent_id is None
            else execution_updates.get(str(assigned_agent_id))
        )
        owner_assignment = (
            None
            if assigned_agent_id is None
            else task_assignments.get(str(assigned_agent_id))
        )
        if task_type in {"hotspot_confirmation", "baseline_service"} and status == "assigned":
            if not _execution_or_claim_holds_task(owner_execution, owner_assignment, task_id):
                violations.append(
                    _build_violation(
                        step,
                        code="assigned_task_owner_not_holding",
                        task_id=task_id,
                        detail=(
                            f"{task_type} task is assigned to {assigned_agent_id}, but execution "
                            "and claim state do not hold the task"
                        ),
                    )
                )
            elif claim_status == "pending_claim" and _is_stale_pending_claim(owner_execution, owner_assignment):
                violations.append(
                    _build_violation(
                        step,
                        code="assigned_task_execution_desync",
                        task_id=task_id,
                        detail=(
                            f"{task_type} task is still assigned to {assigned_agent_id}, but the "
                            "owner is already back in patrol flow without a task target"
                        ),
                    )
                )

        if task_type == "uav_resupply" and status == "assigned" and support_agent_id is not None:
            support_execution = execution_updates.get(str(support_agent_id))
            support_assignment = task_assignments.get(str(support_agent_id))
            if not _execution_or_claim_holds_task(support_execution, support_assignment, task_id):
                violations.append(
                    _build_violation(
                        step,
                        code="uav_resupply_support_not_holding_assignment",
                        task_id=task_id,
                        detail=(
                            "uav_resupply task is assigned, but the support USV execution and "
                            "claim state do not hold the rendezvous task"
                        ),
                    )
                )
            elif claim_status == "claimed" and _is_stale_pending_claim(
                support_execution,
                support_assignment,
            ):
                violations.append(
                    _build_violation(
                        step,
                        code="uav_resupply_support_assignment_desync",
                        task_id=task_id,
                        detail=(
                            "uav_resupply task is assigned, but the support USV has already "
                            "fallen back to patrol flow without a rendezvous target"
                        ),
                    )
                )

        if status != "in_progress":
            continue

        if task_type in {"hotspot_confirmation", "baseline_service"}:
            if (
                owner_execution is None
                or owner_execution.get("execution_stage") != "on_task"
                or owner_execution.get("active_task_id") != task_id
            ):
                violations.append(
                    _build_violation(
                        step,
                        code="task_in_progress_without_executor",
                        task_id=task_id,
                        detail=(
                            f"{task_type} task is in_progress but owner execution stage is "
                            f"{None if owner_execution is None else owner_execution.get('execution_stage')}"
                        ),
                    )
                )
            continue

        if task_type != "uav_resupply":
            continue

        if assigned_agent_id is None or support_agent_id is None:
            continue
        uav_execution = execution_updates.get(str(assigned_agent_id))
        support_execution = execution_updates.get(str(support_agent_id))
        uav_state = agent_states.get(str(assigned_agent_id))
        support_state = agent_states.get(str(support_agent_id))
        if (
            uav_execution is None
            or uav_execution.get("execution_stage") != "on_recharge"
            or uav_execution.get("active_task_id") != task_id
        ):
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_without_recharging_uav",
                    task_id=task_id,
                    detail="uav_resupply task is in_progress but UAV is not in on_recharge",
                )
            )
            continue
        if support_execution is None or support_state is None or uav_state is None:
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_missing_support_state",
                    task_id=task_id,
                    detail="uav_resupply task is in_progress but support execution state is missing",
                )
            )
            continue

        if not _same_position(uav_state, support_state):
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_not_docked",
                    task_id=task_id,
                    detail="uav_resupply task is in_progress but UAV and support USV are not co-located",
                )
            )
            continue

        if support_execution.get("execution_stage") != "on_task":
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_support_not_holding",
                    task_id=task_id,
                    detail=(
                        "uav_resupply task is in_progress and docked, but support USV stage is "
                        f"{support_execution.get('execution_stage')}"
                    ),
                )
            )

        if not _is_zero_motion(support_state):
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_support_motion_mismatch",
                    task_id=task_id,
                    detail="uav_resupply task is in_progress and docked, but support USV is still moving",
                )
            )

        if not _same_motion(uav_state, support_state):
            violations.append(
                _build_violation(
                    step,
                    code="uav_resupply_docked_motion_desync",
                    task_id=task_id,
                    detail="docked UAV motion does not match support USV motion",
                )
            )

    for agent_id, assignment in task_assignments.items():
        execution_update = execution_updates.get(agent_id)
        if execution_update is None:
            continue
        stage = str(execution_update.get("execution_stage"))
        active_task_id = execution_update.get("active_task_id")
        if stage not in {"go_to_task", "go_to_rendezvous"} or active_task_id is None:
            continue
        if assignment.get("target_x") is not None and assignment.get("target_y") is not None:
            continue
        violations.append(
            _build_violation(
                step,
                code="active_transit_missing_target",
                task_id=str(active_task_id),
                detail=(
                    f"agent {agent_id} is still in {stage}, but its execution target is missing"
                ),
                )
            )
    for execution_update in execution_updates.values():
        active_task_id = execution_update.get("active_task_id")
        active_side = execution_update.get("task_approach_active_side")
        if (
            active_task_id is not None
            and active_side in {"left", "right"}
            and task_type_by_id.get(str(active_task_id)) == "baseline_service"
        ):
            violations.append(
                _build_violation(
                    step,
                    code="baseline_task_should_not_use_macro_approach",
                    task_id=str(active_task_id),
                    detail="baseline task entered macro left/right approach mode",
                )
            )

    return violations


def detect_task_status_flip_flops(
    step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Detect short-window task-status oscillations that indicate policy jitter."""

    change_history_by_task: dict[str, list[tuple[int, str, str | None]]] = {}
    previous_by_task: dict[str, tuple[str, str | None]] = {}

    for record in step_snapshots:
        step = int(record["step"])
        for task in record.get("task_layer", {}).get("tasks", []):
            task_id = str(task["task_id"])
            current = (str(task["status"]), task.get("assigned_agent_id"))
            if previous_by_task.get(task_id) == current:
                continue
            previous_by_task[task_id] = current
            change_history_by_task.setdefault(task_id, []).append((step, current[0], current[1]))

    findings: list[dict[str, Any]] = []
    seen_keys: set[tuple[str, int, int]] = set()
    for task_id, change_history in change_history_by_task.items():
        for index in range(len(change_history) - 2):
            start_step, first_status, first_owner = change_history[index]
            middle_step, middle_status, _ = change_history[index + 1]
            end_step, third_status, third_owner = change_history[index + 2]
            if end_step - start_step > _FLIP_FLOP_WINDOW_STEPS:
                continue
            if first_status != third_status or first_owner != third_owner:
                continue
            if first_status not in _HOTSPOT_FLIPFLOP_ACTIVE_STATUSES:
                continue
            if middle_status not in _HOTSPOT_FLIPFLOP_RELEASE_STATUSES:
                continue
            finding_key = (task_id, start_step, end_step)
            if finding_key in seen_keys:
                continue
            seen_keys.add(finding_key)
            findings.append(
                {
                    "task_id": task_id,
                    "step_start": start_step,
                    "step_end": end_step,
                    "pattern": [first_status, middle_status, third_status],
                    "owner_agent_id": first_owner,
                }
            )
    return findings


def detect_requeued_without_release_reason(
    step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Detect assigned/in-progress tasks that become requeued without a same-step reason."""

    findings: list[dict[str, Any]] = []
    previous_by_task: dict[tuple[str, int], dict[str, Any]] = {}
    for record in step_snapshots:
        step = int(record["step"])
        current_by_task: dict[tuple[str, int], dict[str, Any]] = {}
        for task in record.get("task_layer", {}).get("tasks", []):
            key = _task_episode_key(task)
            current_by_task[key] = task
            previous = previous_by_task.get(key)
            if previous is None:
                continue
            if str(previous.get("status")) not in _ACTIVE_TASK_STATUSES:
                continue
            if str(task.get("status")) != "requeued":
                continue
            if task.get("release_reason") is not None:
                continue
            findings.append(
                {
                    "task_id": str(task["task_id"]),
                    "created_step": key[1],
                    "step": step,
                }
            )
        previous_by_task = current_by_task
    return findings


def _task_episode_key(task: dict[str, Any]) -> tuple[str, int]:
    created_step = task.get("created_step")
    if created_step is None:
        return (str(task["task_id"]), -1)
    return (str(task["task_id"]), int(created_step))


def detect_taskless_return_recovery_loops(
    step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Detect no-task return-to-patrol agents repeatedly entering recovery on one goal."""

    entries_by_agent: dict[str, list[tuple[int, str]]] = {}
    previous_stage_by_agent: dict[str, str | None] = {}
    findings: list[dict[str, Any]] = []
    seen: set[tuple[str, int, str]] = set()

    for record in step_snapshots:
        step = int(record["step"])
        for update in record.get("execution_layer", {}).get("tracking_updates", []):
            agent_id = str(update["agent_id"])
            stage = update.get("execution_stage")
            active_task_id = update.get("active_task_id")
            blocked_goal_signature = update.get("return_blocked_goal_signature") or update.get(
                "blocked_goal_signature"
            )
            if stage == "recovery" and active_task_id is None:
                previous_stage = previous_stage_by_agent.get(agent_id)
                if previous_stage != "recovery" and blocked_goal_signature is not None:
                    entries = entries_by_agent.setdefault(agent_id, [])
                    entries.append((step, str(blocked_goal_signature)))
                    entries[:] = [
                        entry
                        for entry in entries
                        if step - entry[0] <= _RETURN_RECOVERY_LOOP_WINDOW_STEPS
                    ]
                    matching_entries = [
                        entry for entry in entries if entry[1] == blocked_goal_signature
                    ]
                    if len(matching_entries) >= _RETURN_RECOVERY_LOOP_MIN_ENTRIES:
                        finding_key = (
                            agent_id,
                            matching_entries[0][0],
                            str(blocked_goal_signature),
                        )
                        if finding_key not in seen:
                            seen.add(finding_key)
                            findings.append(
                                {
                                    "agent_id": agent_id,
                                    "blocked_goal_signature": str(blocked_goal_signature),
                                    "step_start": matching_entries[0][0],
                                    "step_end": step,
                                    "entry_count": len(matching_entries),
                                }
                            )
            previous_stage_by_agent[agent_id] = None if stage is None else str(stage)
    return findings


def detect_task_approach_same_side_recovery_loops(
    step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Detect task recovery loops that keep retrying the same approach side."""

    entries_by_agent_task_side: dict[tuple[str, str, str], list[int]] = {}
    previous_stage_by_agent: dict[str, str | None] = {}
    findings: list[dict[str, Any]] = []
    seen: set[tuple[str, str, str, int]] = set()

    for record in step_snapshots:
        step = int(record["step"])
        for update in record.get("execution_layer", {}).get("tracking_updates", []):
            agent_id = str(update["agent_id"])
            stage = None if update.get("execution_stage") is None else str(update["execution_stage"])
            task_id = update.get("active_task_id")
            approach_task_id = update.get("task_approach_task_id")
            active_side = update.get("task_approach_active_side")
            anchor_status = update.get("task_approach_anchor_status")
            if (
                stage == "recovery"
                and task_id is not None
                and approach_task_id == task_id
                and active_side in {"left", "right"}
                and anchor_status == "enroute_anchor"
            ):
                previous_stage = previous_stage_by_agent.get(agent_id)
                if previous_stage != "recovery":
                    key = (agent_id, str(task_id), str(active_side))
                    entries = entries_by_agent_task_side.setdefault(key, [])
                    entries.append(step)
                    entries[:] = [
                        entry_step
                        for entry_step in entries
                        if step - entry_step <= _APPROACH_SIDE_LOOP_WINDOW_STEPS
                    ]
                    if len(entries) >= _APPROACH_SIDE_LOOP_MIN_ENTRIES:
                        finding_key = (agent_id, str(task_id), str(active_side), entries[0])
                        if finding_key not in seen:
                            seen.add(finding_key)
                            findings.append(
                                {
                                    "agent_id": agent_id,
                                    "task_id": str(task_id),
                                    "approach_side": str(active_side),
                                    "step_start": entries[0],
                                    "step_end": step,
                                    "entry_count": len(entries),
                                }
                            )
            previous_stage_by_agent[agent_id] = stage
    return findings


def detect_task_approach_side_switches_without_recovery(
    step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Detect macro approach side switches that happen without a recovery transition."""

    last_side_by_agent_task: dict[tuple[str, str], str] = {}
    last_recovery_step_by_agent_task: dict[tuple[str, str], int] = {}
    findings: list[dict[str, Any]] = []

    for record in step_snapshots:
        step = int(record["step"])
        for update in record.get("execution_layer", {}).get("tracking_updates", []):
            agent_id = str(update["agent_id"])
            task_id = update.get("active_task_id")
            approach_task_id = update.get("task_approach_task_id")
            active_side = update.get("task_approach_active_side")
            stage = update.get("execution_stage")
            if task_id is None:
                continue
            key = (agent_id, str(task_id))
            if approach_task_id != task_id:
                last_side_by_agent_task.pop(key, None)
                continue
            if stage == "recovery":
                last_recovery_step_by_agent_task[key] = step
            if active_side not in {"left", "right"}:
                last_side_by_agent_task.pop(key, None)
                continue
            previous_side = last_side_by_agent_task.get(key)
            if previous_side is not None and previous_side != active_side:
                last_recovery_step = last_recovery_step_by_agent_task.get(key, -1)
                if last_recovery_step < step - 1:
                    findings.append(
                        {
                            "agent_id": agent_id,
                            "task_id": str(task_id),
                            "from_side": previous_side,
                            "to_side": str(active_side),
                            "step": step,
                        }
                    )
            last_side_by_agent_task[key] = str(active_side)
    return findings


def detect_pseudo_progress_loops(
    step_snapshots: tuple[dict[str, Any], ...] | list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Detect moving-but-not-converging loops for one task/candidate pair."""

    findings: list[dict[str, Any]] = []
    active_windows: dict[tuple[str, str, int], list[tuple[int, float, float, float]]] = {}
    emitted_keys: set[tuple[str, str, int, int]] = set()

    for record in step_snapshots:
        step = int(record["step"])
        agent_states = {
            str(agent["agent_id"]): agent for agent in record.get("agent_states", [])
        }
        for update in record.get("execution_layer", {}).get("tracking_updates", []):
            agent_id = str(update["agent_id"])
            task_id = update.get("active_task_id")
            candidate_index = update.get("task_final_approach_candidate_index")
            candidate_x = update.get("task_final_approach_candidate_x")
            candidate_y = update.get("task_final_approach_candidate_y")
            if (
                update.get("execution_stage") != "go_to_task"
                or task_id is None
                or candidate_index is None
                or candidate_x is None
                or candidate_y is None
            ):
                continue
            if int(update.get("stalled_steps") or 0) != 0:
                continue
            agent_state = agent_states.get(agent_id)
            if agent_state is None or float(agent_state.get("speed_mps", 0.0)) <= 0.0:
                continue
            x = float(agent_state["x"])
            y = float(agent_state["y"])
            target_distance = hypot(float(candidate_x) - x, float(candidate_y) - y)
            key = (agent_id, str(task_id), int(candidate_index))
            window = active_windows.setdefault(key, [])
            window.append((step, x, y, target_distance))
            window[:] = [
                entry
                for entry in window
                if step - entry[0] < _PSEUDO_PROGRESS_WINDOW_STEPS
            ]
            if len(window) < _PSEUDO_PROGRESS_WINDOW_STEPS:
                continue
            xs = [entry[1] for entry in window]
            ys = [entry[2] for entry in window]
            distances = [entry[3] for entry in window]
            bbox_diagonal = hypot(max(xs) - min(xs), max(ys) - min(ys))
            distance_gain = max(distances[0] - distances[-1], 0.0)
            if (
                bbox_diagonal <= _PSEUDO_PROGRESS_MAX_BBOX_DIAGONAL_M
                and distance_gain < _PSEUDO_PROGRESS_MIN_DISTANCE_GAIN_M
            ):
                finding_key = (agent_id, str(task_id), int(candidate_index), window[0][0])
                if finding_key in emitted_keys:
                    continue
                emitted_keys.add(finding_key)
                findings.append(
                    {
                        "agent_id": agent_id,
                        "task_id": str(task_id),
                        "candidate_index": int(candidate_index),
                        "step_start": window[0][0],
                        "step_end": window[-1][0],
                        "bbox_diagonal_m": round(bbox_diagonal, 3),
                        "distance_gain_m": round(distance_gain, 3),
                    }
                )
    return findings


def _build_violation(step: int, *, code: str, task_id: str, detail: str) -> dict[str, Any]:
    return {
        "step": step,
        "code": code,
        "task_id": task_id,
        "detail": detail,
    }


def _execution_or_claim_holds_task(
    execution_update: dict[str, Any] | None,
    task_assignment: dict[str, Any] | None,
    task_id: str,
) -> bool:
    if execution_update is not None and execution_update.get("active_task_id") == task_id:
        return True
    if task_assignment is None:
        return False
    return task_assignment.get("pending_assigned_task_id") == task_id or task_assignment.get(
        "claimed_task_id"
    ) == task_id


def _is_stale_pending_claim(
    execution_update: dict[str, Any] | None,
    task_assignment: dict[str, Any] | None,
) -> bool:
    if execution_update is None or task_assignment is None:
        return False
    if str(execution_update.get("execution_stage")) not in {"patrol", "return_to_patrol", "failed"}:
        return False
    if execution_update.get("active_task_id") is not None:
        return False
    return (
        task_assignment.get("pending_assigned_task_id") is None
        and task_assignment.get("claimed_task_id") is None
        and task_assignment.get("target_x") is None
        and task_assignment.get("target_y") is None
    )


def _is_zero_motion(agent_state: dict[str, Any]) -> bool:
    return abs(float(agent_state.get("speed_mps", 0.0))) <= _RESUPPLY_OVERLAP_EPSILON and abs(
        float(agent_state.get("turn_rate_degps", 0.0))
    ) <= _RESUPPLY_OVERLAP_EPSILON


def _same_motion(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return abs(float(left.get("speed_mps", 0.0)) - float(right.get("speed_mps", 0.0))) <= _RESUPPLY_OVERLAP_EPSILON and abs(
        float(left.get("turn_rate_degps", 0.0)) - float(right.get("turn_rate_degps", 0.0))
    ) <= _RESUPPLY_OVERLAP_EPSILON


def _same_position(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return abs(float(left.get("x", 0.0)) - float(right.get("x", 0.0))) <= _RESUPPLY_OVERLAP_EPSILON and abs(
        float(left.get("y", 0.0)) - float(right.get("y", 0.0))
    ) <= _RESUPPLY_OVERLAP_EPSILON
