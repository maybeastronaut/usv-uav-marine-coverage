"""Shared execution-layer data structures."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from usv_uav_marine_coverage.planning.path_types import PathPlan


class ExecutionStage(StrEnum):
    """High-level operating stage for one agent."""

    PATROL = "patrol"
    GO_TO_TASK = "go_to_task"
    ON_TASK = "on_task"
    RETURN_TO_PATROL = "return_to_patrol"


class ExecutionOutcome(StrEnum):
    """Per-step execution result for one agent."""

    ADVANCING = "advancing"
    WAYPOINT_REACHED = "waypoint_reached"
    TASK_SITE_REACHED = "task_site_reached"
    TASK_FINISHED = "task_finished"
    PATROL_REJOINED = "patrol_rejoined"
    FAILED = "failed"


@dataclass(frozen=True)
class AgentExecutionState:
    """Execution-layer state kept alongside the physical agent state."""

    agent_id: str
    stage: ExecutionStage
    active_task_id: str | None
    active_plan: PathPlan | None
    current_waypoint_index: int
    patrol_route_id: str | None
    patrol_waypoint_index: int
    return_target_x: float | None = None
    return_target_y: float | None = None
