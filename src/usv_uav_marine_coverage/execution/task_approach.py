"""Compatibility wrappers for the unified task-final-approach runtime."""

from __future__ import annotations

from .task_final_approach_runtime import (
    USV_TASK_EDGE_APPROACH_BUFFER_M,
    can_start_hotspot_confirmation,
    resolve_usv_task_approach_target,
)

__all__ = [
    "USV_TASK_EDGE_APPROACH_BUFFER_M",
    "can_start_hotspot_confirmation",
    "resolve_usv_task_approach_target",
]
