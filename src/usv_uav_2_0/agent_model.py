"""Minimal mathematical agent model for follow-up simulation stages."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from math import atan2, degrees, hypot

USV_COVERAGE_RADIUS_M = 50.0
UAV_COVERAGE_RADIUS_M = 100.0


class TaskMode(str, Enum):
    """Basic task modes for the first simulation stage."""

    IDLE = "idle"
    PATROL = "patrol"
    INVESTIGATE = "investigate"
    CONFIRM = "confirm"


@dataclass(frozen=True)
class AgentTaskState:
    """The basic task state carried by an agent."""

    mode: TaskMode = TaskMode.IDLE
    target_x: float | None = None
    target_y: float | None = None

    @property
    def has_target(self) -> bool:
        return self.target_x is not None and self.target_y is not None


@dataclass(frozen=True)
class AgentState:
    """A minimal mathematical state for one USV or UAV."""

    agent_id: str
    kind: str
    x: float
    y: float
    heading_deg: float
    speed_mps: float
    max_speed_mps: float
    detection_radius: float
    coverage_radius: float
    task: AgentTaskState


def default_coverage_radius(kind: str) -> float:
    """Return the confirmed first-stage coverage radius for an agent kind."""

    if kind == "USV":
        return USV_COVERAGE_RADIUS_M
    if kind == "UAV":
        return UAV_COVERAGE_RADIUS_M
    raise ValueError(f"Unsupported agent kind: {kind}")


def assign_agent_task(
    agent: AgentState,
    mode: TaskMode,
    target_x: float | None = None,
    target_y: float | None = None,
) -> AgentState:
    """Return a new agent state with an updated basic task."""

    return replace(
        agent,
        task=AgentTaskState(mode=mode, target_x=target_x, target_y=target_y),
    )


def advance_agent_towards_task(
    agent: AgentState,
    dt_seconds: float,
) -> AgentState:
    """Advance an agent with a simple point-to-point kinematic model."""

    if dt_seconds <= 0:
        raise ValueError("Time step must be positive.")

    if not agent.task.has_target:
        return replace(agent, speed_mps=0.0)

    target_x = agent.task.target_x
    target_y = agent.task.target_y
    assert target_x is not None and target_y is not None

    dx = target_x - agent.x
    dy = target_y - agent.y
    distance = hypot(dx, dy)

    if distance == 0.0:
        return replace(
            agent,
            speed_mps=0.0,
            task=AgentTaskState(mode=agent.task.mode),
        )

    heading_deg = normalize_heading_deg(degrees(atan2(dy, dx)))
    travel_distance = min(agent.max_speed_mps * dt_seconds, distance)
    ratio = travel_distance / distance
    next_x = agent.x + dx * ratio
    next_y = agent.y + dy * ratio
    next_speed = travel_distance / dt_seconds

    next_task = agent.task
    if travel_distance == distance:
        next_task = AgentTaskState(mode=agent.task.mode)

    return replace(
        agent,
        x=next_x,
        y=next_y,
        heading_deg=heading_deg,
        speed_mps=next_speed,
        task=next_task,
    )


def can_detect_point(agent: AgentState, x: float, y: float) -> bool:
    """Return whether a point lies inside the agent's detection radius."""

    return distance_to_point(agent, x, y) <= agent.detection_radius


def can_cover_point(agent: AgentState, x: float, y: float) -> bool:
    """Return whether a point lies inside the agent's coverage radius."""

    return distance_to_point(agent, x, y) <= agent.coverage_radius


def distance_to_point(agent: AgentState, x: float, y: float) -> float:
    """Measure Euclidean distance from an agent to a point."""

    return hypot(agent.x - x, agent.y - y)


def normalize_heading_deg(heading_deg: float) -> float:
    """Normalize a heading angle into the [-180, 180) interval."""

    normalized = (heading_deg + 180.0) % 360.0 - 180.0
    if normalized == -180.0 and heading_deg > 0:
        return 180.0
    return normalized
