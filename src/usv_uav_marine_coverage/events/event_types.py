"""Scheduled dynamic-event types for replay simulation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class EventType(StrEnum):
    """Supported event types for the first dynamic-event stage."""

    AGENT_FAILURE = "agent_failure"
    SPEED_DEGRADATION = "speed_degradation"
    TURN_RATE_DEGRADATION = "turn_rate_degradation"


@dataclass(frozen=True)
class AgentDamageEvent:
    """One scheduled damage/failure event applied to an agent."""

    step: int
    event_type: EventType
    agent_id: str
    speed_multiplier: float = 1.0
    turn_rate_multiplier: float = 1.0


@dataclass(frozen=True)
class AppliedEvent:
    """One event application result emitted into logs and summaries."""

    step: int
    event_type: EventType
    agent_id: str
    summary: str
