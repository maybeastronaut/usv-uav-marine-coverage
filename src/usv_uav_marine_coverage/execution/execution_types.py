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
    GO_TO_RENDEZVOUS = "go_to_rendezvous"
    ON_RECHARGE = "on_recharge"
    RETURN_TO_PATROL = "return_to_patrol"
    RECOVERY = "recovery"


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
    rejoin_to_segment: bool = False


@dataclass(frozen=True)
class AgentProgressState:
    """Runtime feedback state used to detect stalls and drive recovery."""

    agent_id: str
    stalled_steps: int = 0
    last_progress_distance: float = 0.0
    last_target_distance: float | None = None
    recovery_attempts: int = 0
    recovery_step_index: int = 0
    cooldown_until_step: int = 0
    blocked_goal_signature: str | None = None
    pre_recovery_stage: ExecutionStage | None = None
    pre_recovery_task_id: str | None = None
