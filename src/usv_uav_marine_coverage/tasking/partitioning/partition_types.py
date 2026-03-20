"""Shared types for task-zone partition policies."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskPartitionView:
    """One explicit task-partition decision for the current scheduling step."""

    policy_name: str
    primary_usv_ids: tuple[str, ...]
    secondary_usv_ids: tuple[str, ...]
    partition_reason: str
