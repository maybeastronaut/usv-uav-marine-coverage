"""Feedback-layer helpers for USV progress monitoring and replan decisions."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import (
    AgentExecutionState,
    AgentProgressState,
    ExecutionStage,
)
from usv_uav_marine_coverage.planning.path_types import PathPlanStatus
from usv_uav_marine_coverage.tasking.task_types import TaskRecord, TaskType

USV_TASK_REPLAN_GOAL_TOLERANCE_M = 1.0
USV_TASK_REPLAN_DEVIATION_M = 60.0
UAV_RESUPPLY_SUPPORT_REPLAN_GOAL_TOLERANCE_M = 40.0
UAV_RESUPPLY_SUPPORT_REPLAN_DEVIATION_M = 100.0
USV_STALL_DISTANCE_EPS_M = 1e-3
USV_MIN_PROGRESS_DISTANCE_M = 0.5
USV_STALL_TRIGGER_STEPS = 3
USV_COLLISION_CLEAR_TRIGGER_STEPS = 2
USV_RECOVERY_COOLDOWN_STEPS = 8


@dataclass(frozen=True)
class ProgressEvaluation:
    """Progress-evaluation result for one USV step."""

    progress_state: AgentProgressState
    should_enter_recovery: bool
    blocked_goal_signature: str | None


def build_initial_progress_states(
    agents: tuple[AgentState, ...],
) -> dict[str, AgentProgressState]:
    """Build the default feedback state for each agent."""

    return {agent.agent_id: AgentProgressState(agent_id=agent.agent_id) for agent in agents}


def reset_progress_state(progress_state: AgentProgressState) -> AgentProgressState:
    """Reset progress-monitor bookkeeping when normal execution resumes."""

    return replace(
        progress_state,
        stalled_steps=0,
        last_progress_distance=0.0,
        last_target_distance=None,
        recovery_step_index=0,
        cooldown_until_step=0,
        blocked_goal_signature=None,
        pre_recovery_stage=None,
        pre_recovery_task_id=None,
        task_final_approach_task_id=None,
        task_final_approach_frozen_candidates=(),
        task_final_approach_candidate_index=-1,
        task_final_approach_candidate_x=None,
        task_final_approach_candidate_y=None,
        task_final_approach_failed_candidate_indexes=(),
        task_final_approach_attempt_count=0,
        task_final_approach_status=None,
        released_task_id=None,
        released_task_created_step=None,
        released_task_step=-1,
        released_task_retry_until_step=0,
        released_task_reason=None,
        pending_assigned_task_id=None,
        claimed_task_id=None,
        claim_transition_reason=None,
    )


def record_released_task_feedback(
    progress_state: AgentProgressState,
    *,
    task_id: str,
    task_created_step: int,
    task_step: int,
    retry_until_step: int,
    reason: str,
) -> AgentProgressState:
    """Reset runtime stall state while preserving one release signal for tasking."""

    cleared_state = reset_progress_state(progress_state)
    return replace(
        cleared_state,
        released_task_id=task_id,
        released_task_created_step=task_created_step,
        released_task_step=task_step,
        released_task_retry_until_step=retry_until_step,
        released_task_reason=reason,
    )


def record_task_claim_feedback(
    progress_state: AgentProgressState,
    *,
    pending_assigned_task_id: str | None,
    claimed_task_id: str | None,
    claim_transition_reason: str | None,
) -> AgentProgressState:
    """Record task-ownership metadata for logging and sync projection."""

    return replace(
        progress_state,
        pending_assigned_task_id=pending_assigned_task_id,
        claimed_task_id=claimed_task_id,
        claim_transition_reason=claim_transition_reason,
    )


def build_goal_signature(
    *,
    stage: ExecutionStage,
    active_task: TaskRecord | None,
    execution_state: AgentExecutionState,
) -> str | None:
    """Build a stable goal signature for cooldown and failure memory."""

    if stage == ExecutionStage.GO_TO_TASK:
        task_id = active_task.task_id if active_task is not None else execution_state.active_task_id
        if task_id is not None:
            return f"task:{task_id}"
    if stage == ExecutionStage.RETURN_TO_PATROL:
        if execution_state.return_target_x is None or execution_state.return_target_y is None:
            return None
        return (
            f"return:{execution_state.patrol_waypoint_index}:"
            f"{round(execution_state.return_target_x, 1)}:"
            f"{round(execution_state.return_target_y, 1)}"
        )
    if stage == ExecutionStage.PATROL:
        return f"patrol:{execution_state.patrol_waypoint_index}"
    return None


def evaluate_usv_progress(
    previous_agent: AgentState,
    *,
    updated_agent: AgentState,
    execution_state: AgentExecutionState,
    progress_state: AgentProgressState,
    active_task: TaskRecord | None,
    target_x: float | None,
    target_y: float | None,
    path_cleared: bool,
    step: int,
) -> ProgressEvaluation:
    """Evaluate one USV step and decide whether recovery should start."""

    if target_x is None or target_y is None:
        return ProgressEvaluation(
            progress_state=progress_state,
            should_enter_recovery=False,
            blocked_goal_signature=None,
        )

    previous_distance = (
        hypot(target_x - previous_agent.x, target_y - previous_agent.y)
        if progress_state.last_target_distance is None
        else progress_state.last_target_distance
    )
    current_distance = hypot(target_x - updated_agent.x, target_y - updated_agent.y)
    movement_distance = hypot(
        updated_agent.x - previous_agent.x,
        updated_agent.y - previous_agent.y,
    )
    progress_distance = max(previous_distance - current_distance, 0.0)
    plan_cleared_without_motion = path_cleared and movement_distance < USV_STALL_DISTANCE_EPS_M

    stalled_steps = 0
    if (
        movement_distance < USV_STALL_DISTANCE_EPS_M
        and progress_distance < USV_MIN_PROGRESS_DISTANCE_M
    ) or plan_cleared_without_motion:
        stalled_steps = progress_state.stalled_steps + 1

    next_progress_state = replace(
        progress_state,
        stalled_steps=stalled_steps,
        last_progress_distance=progress_distance,
        last_target_distance=current_distance,
    )

    should_enter_recovery = stalled_steps >= USV_STALL_TRIGGER_STEPS or (
        plan_cleared_without_motion and stalled_steps >= USV_COLLISION_CLEAR_TRIGGER_STEPS
    )
    if not should_enter_recovery:
        if stalled_steps == 0:
            next_progress_state = replace(
                next_progress_state,
                recovery_attempts=0,
                blocked_goal_signature=None,
                cooldown_until_step=0,
            )
        return ProgressEvaluation(
            progress_state=next_progress_state,
            should_enter_recovery=False,
            blocked_goal_signature=None,
        )

    blocked_goal_signature = build_goal_signature(
        stage=execution_state.stage,
        active_task=active_task,
        execution_state=execution_state,
    )
    next_progress_state = replace(
        next_progress_state,
        stalled_steps=0,
        recovery_step_index=0,
        cooldown_until_step=step + USV_RECOVERY_COOLDOWN_STEPS,
        blocked_goal_signature=blocked_goal_signature,
        pre_recovery_stage=execution_state.stage,
        pre_recovery_task_id=execution_state.active_task_id,
    )
    return ProgressEvaluation(
        progress_state=next_progress_state,
        should_enter_recovery=True,
        blocked_goal_signature=blocked_goal_signature,
    )


def should_replan_task(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    active_task: TaskRecord,
    usv_path_planner: str = "astar_path_planner",
    goal_x: float | None = None,
    goal_y: float | None = None,
) -> bool:
    """Return whether the current task path should be replanned."""

    plan = execution_state.active_plan
    expected_goal_x = active_task.target_x if goal_x is None else goal_x
    expected_goal_y = active_task.target_y if goal_y is None else goal_y
    goal_tolerance_m = USV_TASK_REPLAN_GOAL_TOLERANCE_M
    deviation_tolerance_m = USV_TASK_REPLAN_DEVIATION_M
    if agent.kind == "USV" and active_task.task_type == TaskType.UAV_RESUPPLY:
        goal_tolerance_m = UAV_RESUPPLY_SUPPORT_REPLAN_GOAL_TOLERANCE_M
        deviation_tolerance_m = UAV_RESUPPLY_SUPPORT_REPLAN_DEVIATION_M
    if plan is None:
        return True
    if plan.status != PathPlanStatus.PLANNED:
        return True
    if plan.task_id != active_task.task_id:
        return True
    expected_planner = usv_path_planner if agent.kind == "USV" else "direct_line_planner"
    if plan.planner_name != expected_planner:
        return True
    if execution_state.current_waypoint_index >= len(plan.waypoints):
        return True
    if agent.kind == "USV" and active_task.task_type in {
        TaskType.BASELINE_SERVICE,
        TaskType.HOTSPOT_CONFIRMATION,
    }:
        allowed_goal_offset = max(
            agent.coverage_radius - agent.arrival_tolerance_m,
            goal_tolerance_m,
        )
        goal_offset = hypot(plan.goal_x - expected_goal_x, plan.goal_y - expected_goal_y)
        if goal_offset > allowed_goal_offset:
            return True
    elif agent.kind == "USV" and active_task.task_type == TaskType.UAV_RESUPPLY:
        goal_offset = hypot(plan.goal_x - expected_goal_x, plan.goal_y - expected_goal_y)
        if goal_offset > goal_tolerance_m:
            return True
    elif (
        abs(plan.goal_x - expected_goal_x) > goal_tolerance_m
        or abs(plan.goal_y - expected_goal_y) > goal_tolerance_m
    ):
        return True
    if agent.kind != "USV":
        return False
    current_waypoint = plan.waypoints[execution_state.current_waypoint_index]
    return (
        hypot(current_waypoint.x - agent.x, current_waypoint.y - agent.y)
        > deviation_tolerance_m
    )


def should_replan_patrol(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    goal_x: float,
    goal_y: float,
    usv_path_planner: str = "astar_path_planner",
) -> bool:
    """Return whether the current patrol path should be replanned."""

    plan = execution_state.active_plan
    if plan is None:
        return True
    if plan.status != PathPlanStatus.PLANNED:
        return True
    if plan.planner_name != usv_path_planner:
        return True
    if execution_state.current_waypoint_index >= len(plan.waypoints):
        return True
    if (
        abs(plan.goal_x - goal_x) > USV_TASK_REPLAN_GOAL_TOLERANCE_M
        or abs(plan.goal_y - goal_y) > USV_TASK_REPLAN_GOAL_TOLERANCE_M
    ):
        return True
    if agent.kind != "USV":
        return False
    current_waypoint = plan.waypoints[execution_state.current_waypoint_index]
    return (
        hypot(current_waypoint.x - agent.x, current_waypoint.y - agent.y)
        > USV_TASK_REPLAN_DEVIATION_M
    )


def should_replan_return(
    agent: AgentState,
    *,
    execution_state: AgentExecutionState,
    usv_path_planner: str = "astar_path_planner",
) -> bool:
    """Return whether the current return-to-patrol path should be replanned."""

    plan = execution_state.active_plan
    if execution_state.return_target_x is None or execution_state.return_target_y is None:
        return True
    if plan is None:
        return True
    if plan.status != PathPlanStatus.PLANNED:
        return True
    expected_planner = usv_path_planner if agent.kind == "USV" else "direct_line_planner"
    if plan.planner_name != expected_planner:
        return True
    if execution_state.current_waypoint_index >= len(plan.waypoints):
        return True
    if (
        abs(plan.goal_x - execution_state.return_target_x) > USV_TASK_REPLAN_GOAL_TOLERANCE_M
        or abs(plan.goal_y - execution_state.return_target_y) > USV_TASK_REPLAN_GOAL_TOLERANCE_M
    ):
        return True
    if agent.kind != "USV":
        return False
    current_waypoint = plan.waypoints[execution_state.current_waypoint_index]
    return (
        hypot(current_waypoint.x - agent.x, current_waypoint.y - agent.y)
        > USV_TASK_REPLAN_DEVIATION_M
    )
