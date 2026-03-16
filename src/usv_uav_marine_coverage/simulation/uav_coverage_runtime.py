"""Helpers for persistent UAV multi-region coverage state management."""

from __future__ import annotations

from usv_uav_marine_coverage.agent_model import AgentState
from usv_uav_marine_coverage.execution.execution_types import UavCoverageState
from usv_uav_marine_coverage.grid import GridMap
from usv_uav_marine_coverage.information_map import InformationMap
from usv_uav_marine_coverage.planning.uav_persistent_multi_region_coverage_planner import (
    build_empty_uav_coverage_state,
    build_uav_persistent_search_regions,
    select_persistent_uav_region,
)


def build_initial_uav_coverage_states(
    *,
    agents: tuple[AgentState, ...],
    grid_map: GridMap,
    info_map: InformationMap | None,
    step: int = 0,
) -> dict[str, UavCoverageState]:
    """Build initial persistent patrol states for all UAVs."""

    offshore_min_x, offshore_max_x = uav_offshore_x_bounds(grid_map)
    regions = build_uav_persistent_search_regions(
        min_x=offshore_min_x,
        max_x=offshore_max_x,
    )
    coverage_states: dict[str, UavCoverageState] = {}
    for agent in agents:
        if agent.kind != "UAV":
            continue
        occupied_regions = frozenset(
            state.current_region_id
            for state in coverage_states.values()
            if state.current_region_id is not None
        )
        state, _ = select_persistent_uav_region(
            agent=agent,
            info_map=info_map,
            regions=regions,
            coverage_state=build_empty_uav_coverage_state(agent.agent_id),
            step=step,
            force_replan=True,
            occupied_region_ids=occupied_regions,
        )
        coverage_states[agent.agent_id] = state
    return coverage_states


def resolve_persistent_uav_coverage_state(
    *,
    agent: AgentState,
    coverage_state: UavCoverageState,
    grid_map: GridMap,
    info_map: InformationMap | None,
    step: int,
    force_replan: bool = False,
    all_coverage_states: dict[str, UavCoverageState] | None = None,
) -> UavCoverageState:
    """Return the current persistent AOI state for one UAV patrol step."""

    offshore_min_x, offshore_max_x = uav_offshore_x_bounds(grid_map)
    regions = build_uav_persistent_search_regions(
        min_x=offshore_min_x,
        max_x=offshore_max_x,
    )
    occupied_regions = frozenset(
        state.current_region_id
        for agent_id, state in (all_coverage_states or {}).items()
        if agent_id != agent.agent_id and state.current_region_id is not None
    )
    updated_state, _ = select_persistent_uav_region(
        agent=agent,
        info_map=info_map,
        regions=regions,
        coverage_state=coverage_state,
        step=step,
        force_replan=force_replan,
        occupied_region_ids=occupied_regions,
    )
    return updated_state


def uav_offshore_x_bounds(grid_map: GridMap) -> tuple[float, float]:
    """Return the offshore x extents needed by UAV planners."""

    offshore_cells = tuple(
        cell for cell in grid_map.flat_cells if cell.zone_name == "Offshore Zone"
    )
    if not offshore_cells:
        raise ValueError("Grid map does not contain any offshore cells for UAV search.")
    return (
        min(cell.x_min for cell in offshore_cells),
        max(cell.x_max for cell in offshore_cells),
    )


__all__ = [
    "build_initial_uav_coverage_states",
    "resolve_persistent_uav_coverage_state",
    "uav_offshore_x_bounds",
]
