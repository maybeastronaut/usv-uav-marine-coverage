"""Shared path-planning data structures."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


@dataclass(frozen=True)
class Waypoint:
    """One waypoint in continuous space, with optional grid coordinates."""

    x: float
    y: float
    row: int | None = None
    col: int | None = None


class PathPlanStatus(StrEnum):
    """Lifecycle state of one path plan."""

    PLANNED = "planned"
    BLOCKED = "blocked"
    COMPLETED = "completed"


@dataclass(frozen=True)
class PathPlan:
    """A normalized path plan returned by one planner."""

    plan_id: str
    planner_name: str
    agent_id: str
    task_id: str | None
    status: PathPlanStatus
    waypoints: tuple[Waypoint, ...]
    goal_x: float
    goal_y: float
    estimated_cost: float
