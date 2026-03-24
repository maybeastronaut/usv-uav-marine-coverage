"""Replay validation helpers for step invariants and flip-flop detection."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

_ACTIVE_TASK_STATUSES = {"assigned", "in_progress"}
_HOTSPOT_FLIPFLOP_ACTIVE_STATUSES = {"assigned", "in_progress"}
_HOTSPOT_FLIPFLOP_RELEASE_STATUSES = {"pending", "requeued"}
_RESUPPLY_OVERLAP_EPSILON = 1e-6
_FLIP_FLOP_WINDOW_STEPS = 6


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
    return {
        "step_violation_count": sum(int(validation.get("violation_count", 0)) for validation in validations),
        "steps_with_violations": steps_with_violations,
        "flip_flop_count": len(flip_flops),
        "flip_flops": flip_flops,
        "has_errors": bool(steps_with_violations or flip_flops),
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
