"""Regional recovery debt scoring for failure-mode hotspot recovery."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from usv_uav_marine_coverage.agent_model import AgentState, is_operational_agent
from usv_uav_marine_coverage.information_map import InformationMap, InformationValidity

from ..task_types import TaskRecord, TaskStatus, TaskType

REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS = 2
REGIONAL_RECOVERY_STALE_CELL_WEIGHT = 6.0
REGIONAL_RECOVERY_BACKLOG_WEIGHT = 18.0
REGIONAL_RECOVERY_MEAN_AGE_WEIGHT = 0.05
REGIONAL_RECOVERY_WAITING_STEPS_WEIGHT = 0.10
REGIONAL_RECOVERY_DISTANCE_WEIGHT = 0.04
REGIONAL_RECOVERY_MAX_MEAN_AGE = 400.0
REGIONAL_RECOVERY_MAX_WAITING_STEPS = 240
REGIONAL_RECOVERY_MAX_DISTANCE_M = 400.0
REGIONAL_RECOVERY_HIGH_DEBT_THRESHOLD = 90.0
REGIONAL_RECOVERY_MIN_BACKLOG_FOR_HIGH_DEBT = 2
REGIONAL_RECOVERY_MIN_STALE_CELLS_FOR_HIGH_DEBT = 8
REGIONAL_RECOVERY_AGED_REMOTE_ISOLATED_WAITING_STEPS = 360
REGIONAL_RECOVERY_AGED_REMOTE_ISOLATED_DISTANCE_M = 350.0
REGIONAL_RECOVERY_AGED_REMOTE_ISOLATED_SCORE_BONUS = 24.0


@dataclass(frozen=True)
class RegionalRecoveryDebt:
    """One per-task regional recovery debt estimate."""

    score: float
    stale_coverable_cell_count: int
    backlog_hotspot_count: int
    mean_information_age: float
    waiting_steps: int
    nearest_operational_usv_distance_m: float
    is_high_debt: bool


def build_regional_recovery_debt(
    task: TaskRecord,
    *,
    tasks: tuple[TaskRecord, ...],
    agents: tuple[AgentState, ...],
    info_map: InformationMap | None,
    step: int | None,
) -> RegionalRecoveryDebt:
    """Build a local failure-recovery debt score for one hotspot task."""

    if (
        task.task_type != TaskType.HOTSPOT_CONFIRMATION
        or info_map is None
        or step is None
        or task.target_row is None
        or task.target_col is None
    ):
        return RegionalRecoveryDebt(
            score=0.0,
            stale_coverable_cell_count=0,
            backlog_hotspot_count=0,
            mean_information_age=0.0,
            waiting_steps=0,
            nearest_operational_usv_distance_m=0.0,
            is_high_debt=False,
        )

    row_start = max(0, task.target_row - REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS)
    row_end = min(
        info_map.grid_map.rows,
        task.target_row + REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS + 1,
    )
    col_start = max(0, task.target_col - REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS)
    col_end = min(
        info_map.grid_map.cols,
        task.target_col + REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS + 1,
    )

    coverable_cell_ages: list[int] = []
    stale_coverable_cell_count = 0
    for row in range(row_start, row_end):
        for col in range(col_start, col_end):
            cell = info_map.grid_map.cell_at(row, col)
            if cell.has_obstacle:
                continue
            state = info_map.state_at(row, col)
            coverable_cell_ages.append(state.information_age)
            if state.validity == InformationValidity.STALE_KNOWN:
                stale_coverable_cell_count += 1

    backlog_hotspot_count = sum(
        1
        for candidate in tasks
        if candidate.task_type == TaskType.HOTSPOT_CONFIRMATION
        and candidate.status not in {TaskStatus.COMPLETED, TaskStatus.CANCELLED}
        and candidate.target_row is not None
        and candidate.target_col is not None
        and abs(candidate.target_row - task.target_row) <= REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS
        and abs(candidate.target_col - task.target_col) <= REGIONAL_RECOVERY_NEIGHBOR_RADIUS_CELLS
    )

    mean_information_age = (
        sum(coverable_cell_ages) / len(coverable_cell_ages) if coverable_cell_ages else 0.0
    )
    waiting_anchor_step = (
        task.retry_after_step
        if task.status == TaskStatus.REQUEUED and task.retry_after_step is not None
        else task.created_step
    )
    waiting_steps = max(step - waiting_anchor_step, 0)

    operational_usvs = tuple(
        agent for agent in agents if agent.kind == "USV" and is_operational_agent(agent)
    )
    nearest_operational_usv_distance_m = (
        min(hypot(agent.x - task.target_x, agent.y - task.target_y) for agent in operational_usvs)
        if operational_usvs
        else 0.0
    )

    score = (
        stale_coverable_cell_count * REGIONAL_RECOVERY_STALE_CELL_WEIGHT
        + max(backlog_hotspot_count - 1, 0) * REGIONAL_RECOVERY_BACKLOG_WEIGHT
        + min(mean_information_age, REGIONAL_RECOVERY_MAX_MEAN_AGE)
        * REGIONAL_RECOVERY_MEAN_AGE_WEIGHT
        + min(waiting_steps, REGIONAL_RECOVERY_MAX_WAITING_STEPS)
        * REGIONAL_RECOVERY_WAITING_STEPS_WEIGHT
        + min(nearest_operational_usv_distance_m, REGIONAL_RECOVERY_MAX_DISTANCE_M)
        * REGIONAL_RECOVERY_DISTANCE_WEIGHT
    )
    score += _aged_remote_isolated_score_bonus(
        backlog_hotspot_count=backlog_hotspot_count,
        waiting_steps=waiting_steps,
        nearest_operational_usv_distance_m=nearest_operational_usv_distance_m,
    )
    is_high_debt = score >= REGIONAL_RECOVERY_HIGH_DEBT_THRESHOLD and (
        backlog_hotspot_count >= REGIONAL_RECOVERY_MIN_BACKLOG_FOR_HIGH_DEBT
        or stale_coverable_cell_count >= REGIONAL_RECOVERY_MIN_STALE_CELLS_FOR_HIGH_DEBT
    )
    return RegionalRecoveryDebt(
        score=round(score, 3),
        stale_coverable_cell_count=stale_coverable_cell_count,
        backlog_hotspot_count=backlog_hotspot_count,
        mean_information_age=round(mean_information_age, 3),
        waiting_steps=waiting_steps,
        nearest_operational_usv_distance_m=round(nearest_operational_usv_distance_m, 3),
        is_high_debt=is_high_debt,
    )


def _aged_remote_isolated_score_bonus(
    *,
    backlog_hotspot_count: int,
    waiting_steps: int,
    nearest_operational_usv_distance_m: float,
) -> float:
    """Lift only truly old and remote isolated hotspots.

    This keeps the first-version debt focused on tail recovery without
    broadly promoting freshly spawned isolated tasks.
    """

    if backlog_hotspot_count != 1:
        return 0.0
    if waiting_steps < REGIONAL_RECOVERY_AGED_REMOTE_ISOLATED_WAITING_STEPS:
        return 0.0
    if nearest_operational_usv_distance_m < REGIONAL_RECOVERY_AGED_REMOTE_ISOLATED_DISTANCE_M:
        return 0.0
    return REGIONAL_RECOVERY_AGED_REMOTE_ISOLATED_SCORE_BONUS
