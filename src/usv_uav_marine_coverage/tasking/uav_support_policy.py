"""Stable UAV-to-USV support pairing preferences."""

from __future__ import annotations

from usv_uav_marine_coverage.agent_model import AgentState, is_operational_agent

_DEFAULT_USV_SUPPORT_ORDER = ("USV-1", "USV-2", "USV-3")
_PREFERRED_USV_SUPPORT_BY_UAV: dict[str, tuple[str, ...]] = {
    "UAV-1": ("USV-1", "USV-3", "USV-2"),
    "UAV-2": ("USV-3", "USV-1", "USV-2"),
}
_FIXED_INITIAL_ESCORT_SUPPORT_BY_UAV: dict[str, str] = {
    "UAV-1": "USV-1",
    "UAV-2": "USV-3",
}
_FIXED_INITIAL_ESCORT_UAV_BY_USV: dict[str, str] = {
    "USV-1": "UAV-1",
    "USV-3": "UAV-2",
}
_UAV_USV_MEETING_ENABLED = True
INITIAL_FIXED_ESCORT_MAX_PHASE_STEPS = 360
INITIAL_FIXED_ESCORT_USV_IDS = frozenset({"USV-1", "USV-3"})
INITIAL_FIXED_BASE_USV_ID = "USV-2"
INITIAL_FIXED_ESCORT_RELEASE_X_M = 450.0


def set_uav_usv_meeting_enabled(enabled: bool) -> None:
    """Set whether UAV-USV meeting logic is enabled for the current run."""

    global _UAV_USV_MEETING_ENABLED
    _UAV_USV_MEETING_ENABLED = enabled


def is_uav_usv_meeting_enabled() -> bool:
    """Return whether UAV-USV meeting logic is currently enabled."""

    return _UAV_USV_MEETING_ENABLED


def preferred_support_usv_order_for_uav(
    uav_id: str,
    *,
    available_usv_ids: tuple[str, ...] | list[str] | None = None,
) -> tuple[str, ...]:
    """Return support-USV preference order for one UAV.

    The default policy keeps `USV-2` free as long as one of the two offshore
    support pairings is still available.
    """

    preferred_ids = _PREFERRED_USV_SUPPORT_BY_UAV.get(uav_id, _DEFAULT_USV_SUPPORT_ORDER)
    if available_usv_ids is None:
        return preferred_ids
    available_set = set(available_usv_ids)
    ordered = tuple(agent_id for agent_id in preferred_ids if agent_id in available_set)
    remaining = tuple(agent_id for agent_id in available_usv_ids if agent_id not in ordered)
    return ordered + remaining


def select_preferred_operational_support_usv_id(
    uav: AgentState,
    *,
    usvs: tuple[AgentState, ...],
) -> str | None:
    """Return the preferred operational support USV id for one UAV."""

    operational_usv_ids = tuple(
        agent.agent_id
        for agent in usvs
        if agent.kind == "USV" and is_operational_agent(agent)
    )
    for support_usv_id in preferred_support_usv_order_for_uav(
        uav.agent_id,
        available_usv_ids=operational_usv_ids,
    ):
        return support_usv_id
    return None


def _is_fixed_initial_escort_pair_active(
    uav_id: str,
    support_usv_id: str,
    *,
    step: int | None,
    agent_by_id: dict[str, AgentState] | None = None,
) -> bool:
    """Return whether one hard-bound escort pair should stay reserved."""

    if not _UAV_USV_MEETING_ENABLED:
        return False
    if step is None or step >= INITIAL_FIXED_ESCORT_MAX_PHASE_STEPS:
        return False
    if agent_by_id is None:
        return True
    escort_usv = agent_by_id.get(support_usv_id)
    if escort_usv is None:
        return False
    if not is_operational_agent(escort_usv):
        return False
    escort_uav = agent_by_id.get(uav_id)
    if escort_uav is not None and not is_operational_agent(escort_uav):
        return False
    release_threshold_x = INITIAL_FIXED_ESCORT_RELEASE_X_M - escort_usv.arrival_tolerance_m
    return escort_usv.x < release_threshold_x


def is_initial_fixed_escort_phase(
    step: int | None,
    *,
    agent_by_id: dict[str, AgentState] | None = None,
) -> bool:
    """Return whether the simulation is still inside the initial escort phase."""

    return bool(
        reserved_usv_ids_for_initial_escort(
            step,
            agent_by_id=agent_by_id,
        )
    )


def reserved_usv_ids_for_initial_escort(
    step: int | None,
    *,
    agent_by_id: dict[str, AgentState] | None = None,
) -> frozenset[str]:
    """Return the USVs reserved for fixed UAV escort during the initial phase."""

    active_ids = [
        support_usv_id
        for uav_id, support_usv_id in _FIXED_INITIAL_ESCORT_SUPPORT_BY_UAV.items()
        if _is_fixed_initial_escort_pair_active(
            uav_id,
            support_usv_id,
            step=step,
            agent_by_id=agent_by_id,
        )
    ]
    return frozenset(active_ids)


def is_reserved_for_initial_escort(
    agent_id: str,
    *,
    step: int | None,
    agent_by_id: dict[str, AgentState] | None = None,
) -> bool:
    """Return whether one USV is reserved for fixed escort at this step."""

    return agent_id in reserved_usv_ids_for_initial_escort(step, agent_by_id=agent_by_id)


def fixed_initial_escort_support_usv_id_for_uav(
    uav_id: str,
    *,
    step: int | None,
    agent_by_id: dict[str, AgentState] | None = None,
) -> str | None:
    """Return the hard-bound support USV id during the initial escort phase."""

    support_usv_id = _FIXED_INITIAL_ESCORT_SUPPORT_BY_UAV.get(uav_id)
    if support_usv_id is None:
        return None
    if not _is_fixed_initial_escort_pair_active(
        uav_id,
        support_usv_id,
        step=step,
        agent_by_id=agent_by_id,
    ):
        return None
    return support_usv_id


def fixed_initial_escort_uav_id_for_usv(
    usv_id: str,
    *,
    step: int | None,
    agent_by_id: dict[str, AgentState] | None = None,
) -> str | None:
    """Return the hard-bound escort UAV id during the initial escort phase."""

    uav_id = _FIXED_INITIAL_ESCORT_UAV_BY_USV.get(usv_id)
    if uav_id is None:
        return None
    if not _is_fixed_initial_escort_pair_active(
        uav_id,
        usv_id,
        step=step,
        agent_by_id=agent_by_id,
    ):
        return None
    return uav_id


def should_delay_uav_resupply_for_initial_escort(
    uav: AgentState,
    *,
    usvs: tuple[AgentState, ...],
    step: int | None,
) -> bool:
    """Return whether low-energy resupply should wait until fixed escort completes.

    During the initial fixed escort window we want the paired `USV-1/3` to finish
    escorting `UAV-1/2` through their corridor instead of getting pulled into an
    early rendezvous episode.  If the hard-bound escort USV is no longer
    operational, resupply should proceed normally.
    """

    if not _UAV_USV_MEETING_ENABLED:
        return False
    agent_by_id = {uav.agent_id: uav, **{agent.agent_id: agent for agent in usvs}}
    support_usv_id = fixed_initial_escort_support_usv_id_for_uav(
        uav.agent_id,
        step=step,
        agent_by_id=agent_by_id,
    )
    if support_usv_id is None:
        return False
    support_usv = next((agent for agent in usvs if agent.agent_id == support_usv_id), None)
    return support_usv is not None and is_operational_agent(support_usv)
