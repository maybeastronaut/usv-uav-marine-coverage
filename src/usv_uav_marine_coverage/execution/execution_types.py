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
    YIELD = "yield"
    RECOVERY = "recovery"
    FAILED = "failed"


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
    return_target_source: str | None = None
    rejoin_to_segment: bool = False
    yield_target_x: float | None = None
    yield_target_y: float | None = None
    yield_reason: str | None = None
    reserved_corridor_name: str | None = None
    corridor_owner_agent_id: str | None = None
    corridor_reservation_until_step: int = -1
    reserved_bottleneck_zone_id: str | None = None
    bottleneck_owner_agent_id: str | None = None
    bottleneck_reservation_until_step: int = -1
    pre_yield_stage: ExecutionStage | None = None
    last_return_plan_step: int = -1
    last_patrol_plan_step: int = -1


@dataclass(frozen=True)
class WreckZone:
    """One static keepout zone created by a failed USV."""

    source_agent_id: str
    x: float
    y: float
    radius: float


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
    task_final_approach_task_id: str | None = None
    task_final_approach_frozen_candidates: tuple[tuple[float, float], ...] = ()
    task_final_approach_candidate_index: int = -1
    task_final_approach_candidate_x: float | None = None
    task_final_approach_candidate_y: float | None = None
    task_final_approach_failed_candidate_indexes: tuple[int, ...] = ()
    task_final_approach_attempt_count: int = 0
    task_final_approach_status: str | None = None
    released_task_id: str | None = None
    released_task_created_step: int | None = None
    released_task_step: int = -1
    released_task_retry_until_step: int = 0
    released_task_reason: str | None = None
    pending_assigned_task_id: str | None = None
    claimed_task_id: str | None = None
    claim_transition_reason: str | None = None


@dataclass(frozen=True)
class UavCoverageState:
    """Persistent UAV patrol-coverage state kept outside execution state."""

    agent_id: str
    current_region_id: str | None = None
    region_route: tuple[tuple[float, float], ...] = ()
    region_waypoint_index: int = 0
    region_entry_step: int = -1
    committed_waypoints_remaining: int = 0
    last_replan_step: int = -1
    last_replan_reason: str | None = None
    region_last_visit_steps: tuple[tuple[str, int], ...] = ()
