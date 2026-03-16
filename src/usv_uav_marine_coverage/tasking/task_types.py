"""Shared task data structures for the tasking layer."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class TaskType(StrEnum):
    """Supported task categories for the current simulation stage."""

    BASELINE_SERVICE = "baseline_service"
    HOTSPOT_CONFIRMATION = "hotspot_confirmation"
    UAV_RESUPPLY = "uav_resupply"


class TaskSource(StrEnum):
    """Origin of one task record."""

    UAV_INSPECTED = "uav_inspected"
    UAV_SUSPECTED = "uav_inspected"
    USV_ANOMALY = "usv_anomaly"
    SYSTEM_BASELINE_TIMEOUT = "system_baseline_timeout"
    SYSTEM_LOW_BATTERY = "system_low_battery"


class TaskStatus(StrEnum):
    """Lifecycle state of one task record."""

    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    REQUEUED = "requeued"


@dataclass(frozen=True)
class TaskRecord:
    """A normalized task object shared across tasking, planning, and execution."""

    task_id: str
    task_type: TaskType
    source: TaskSource
    status: TaskStatus
    priority: int
    target_x: float
    target_y: float
    target_row: int | None
    target_col: int | None
    created_step: int
    assigned_agent_id: str | None = None
    support_agent_id: str | None = None
    completed_step: int | None = None
    retry_after_step: int | None = None
    agent_retry_after_steps: tuple[tuple[str, int], ...] = ()


@dataclass(frozen=True)
class TaskAssignment:
    """One agent-task assignment decision."""

    task_id: str
    task_type: TaskType
    agent_id: str
    support_agent_id: str | None
    selection_reason: str
    selection_score: float
