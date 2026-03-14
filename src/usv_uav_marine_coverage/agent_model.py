"""First-stage closed-loop agent model for follow-up simulation stages."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from math import atan2, cos, degrees, hypot, radians, sin

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
class PlatformProfile:
    """Platform-specific first-stage control and kinematic limits."""

    kind: str
    cruise_speed_mps: float
    max_speed_mps: float
    max_acceleration_mps2: float
    max_deceleration_mps2: float
    max_turn_rate_degps: float
    arrival_tolerance_m: float


@dataclass(frozen=True)
class TrackingReference:
    """A lightweight target generated from the current task state."""

    target_x: float
    target_y: float
    desired_speed_mps: float


@dataclass(frozen=True)
class ControlCommand:
    """A lightweight control command for first-stage closed-loop updates."""

    target_speed_mps: float
    target_heading_deg: float


@dataclass(frozen=True)
class AgentState:
    """A first-stage closed-loop mathematical state for one USV or UAV."""

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
    turn_rate_degps: float = 0.0
    max_acceleration_mps2: float = 0.0
    max_deceleration_mps2: float = 0.0
    max_turn_rate_degps: float = 0.0
    cruise_speed_mps: float = 0.0
    arrival_tolerance_m: float = 0.0
    platform_profile: PlatformProfile | None = None

    def __post_init__(self) -> None:
        profile = self.platform_profile or default_platform_profile(self.kind)
        object.__setattr__(self, "platform_profile", profile)

        if self.max_speed_mps <= 0.0:
            object.__setattr__(self, "max_speed_mps", profile.max_speed_mps)
        if self.max_acceleration_mps2 <= 0.0:
            object.__setattr__(self, "max_acceleration_mps2", profile.max_acceleration_mps2)
        if self.max_deceleration_mps2 <= 0.0:
            object.__setattr__(self, "max_deceleration_mps2", profile.max_deceleration_mps2)
        if self.max_turn_rate_degps <= 0.0:
            object.__setattr__(self, "max_turn_rate_degps", profile.max_turn_rate_degps)
        if self.cruise_speed_mps <= 0.0:
            object.__setattr__(self, "cruise_speed_mps", profile.cruise_speed_mps)
        if self.arrival_tolerance_m <= 0.0:
            object.__setattr__(self, "arrival_tolerance_m", profile.arrival_tolerance_m)


def default_platform_profile(kind: str) -> PlatformProfile:
    """Return the first-stage medium-difference profile for one platform kind."""

    if kind == "USV":
        return PlatformProfile(
            kind="USV",
            cruise_speed_mps=5.5,
            max_speed_mps=8.0,
            max_acceleration_mps2=1.8,
            max_deceleration_mps2=2.4,
            max_turn_rate_degps=28.0,
            arrival_tolerance_m=12.0,
        )
    if kind == "UAV":
        return PlatformProfile(
            kind="UAV",
            cruise_speed_mps=16.0,
            max_speed_mps=22.0,
            max_acceleration_mps2=8.0,
            max_deceleration_mps2=10.0,
            max_turn_rate_degps=120.0,
            arrival_tolerance_m=8.0,
        )
    raise ValueError(f"Unsupported agent kind: {kind}")


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


def build_tracking_reference(agent: AgentState) -> TrackingReference | None:
    """Build a lightweight tracking reference from the current task state."""

    if not agent.task.has_target:
        return None

    target_x = agent.task.target_x
    target_y = agent.task.target_y
    assert target_x is not None and target_y is not None
    return TrackingReference(
        target_x=target_x,
        target_y=target_y,
        desired_speed_mps=min(agent.cruise_speed_mps, agent.max_speed_mps),
    )


def compute_control_command(
    agent: AgentState,
    reference: TrackingReference | None,
) -> ControlCommand:
    """Compute a lightweight heading/speed command from one tracking reference."""

    if reference is None:
        return ControlCommand(
            target_speed_mps=0.0,
            target_heading_deg=agent.heading_deg,
        )

    dx = reference.target_x - agent.x
    dy = reference.target_y - agent.y
    distance = hypot(dx, dy)
    target_heading_deg = normalize_heading_deg(degrees(atan2(dy, dx)))

    if distance <= agent.arrival_tolerance_m:
        return ControlCommand(target_speed_mps=0.0, target_heading_deg=target_heading_deg)

    desired_speed = min(reference.desired_speed_mps, agent.max_speed_mps)
    slowdown_radius = max(agent.arrival_tolerance_m * 6.0, desired_speed * 2.0)
    distance_scale = min(distance / slowdown_radius, 1.0)
    target_speed = desired_speed * distance_scale

    heading_error = abs(shortest_heading_delta_deg(agent.heading_deg, target_heading_deg))
    if agent.kind == "USV":
        target_speed *= _usv_heading_alignment_factor(heading_error)
    else:
        target_speed *= _uav_heading_alignment_factor(heading_error)

    return ControlCommand(
        target_speed_mps=clamp(target_speed, 0.0, agent.max_speed_mps),
        target_heading_deg=target_heading_deg,
    )


def advance_agent_with_control(
    agent: AgentState,
    command: ControlCommand,
    dt_seconds: float,
) -> AgentState:
    """Advance one agent with constrained heading and speed dynamics."""

    if dt_seconds <= 0:
        raise ValueError("Time step must be positive.")

    target_heading_deg = normalize_heading_deg(command.target_heading_deg)
    heading_error = shortest_heading_delta_deg(agent.heading_deg, target_heading_deg)
    max_heading_step = agent.max_turn_rate_degps * dt_seconds
    applied_heading_step = clamp(heading_error, -max_heading_step, max_heading_step)
    next_heading_deg = normalize_heading_deg(agent.heading_deg + applied_heading_step)
    next_turn_rate_degps = applied_heading_step / dt_seconds

    target_speed = clamp(command.target_speed_mps, 0.0, agent.max_speed_mps)
    speed_error = target_speed - agent.speed_mps
    speed_step_limit = (
        agent.max_acceleration_mps2 * dt_seconds
        if speed_error >= 0.0
        else agent.max_deceleration_mps2 * dt_seconds
    )
    applied_speed_step = clamp(speed_error, -speed_step_limit, speed_step_limit)
    next_speed_mps = clamp(agent.speed_mps + applied_speed_step, 0.0, agent.max_speed_mps)

    travel_distance = next_speed_mps * dt_seconds
    next_x = agent.x + cos(radians(next_heading_deg)) * travel_distance
    next_y = agent.y + sin(radians(next_heading_deg)) * travel_distance

    return replace(
        agent,
        x=next_x,
        y=next_y,
        heading_deg=next_heading_deg,
        speed_mps=next_speed_mps,
        turn_rate_degps=next_turn_rate_degps,
    )


def advance_agent_towards_task(
    agent: AgentState,
    dt_seconds: float,
) -> AgentState:
    """Advance an agent with the first-stage tracking -> control -> dynamics loop."""

    if dt_seconds <= 0:
        raise ValueError("Time step must be positive.")

    reference = build_tracking_reference(agent)
    if reference is None:
        idle_command = compute_control_command(agent, None)
        return advance_agent_with_control(agent, idle_command, dt_seconds)

    distance = hypot(reference.target_x - agent.x, reference.target_y - agent.y)
    if distance <= agent.arrival_tolerance_m:
        cleared_agent = replace(agent, task=AgentTaskState(mode=agent.task.mode))
        idle_command = compute_control_command(cleared_agent, None)
        return advance_agent_with_control(cleared_agent, idle_command, dt_seconds)

    command = compute_control_command(agent, reference)
    advanced_agent = advance_agent_with_control(agent, command, dt_seconds)

    remaining_distance = hypot(
        reference.target_x - advanced_agent.x,
        reference.target_y - advanced_agent.y,
    )
    if remaining_distance <= advanced_agent.arrival_tolerance_m:
        return replace(
            advanced_agent,
            task=AgentTaskState(mode=advanced_agent.task.mode),
        )
    return advanced_agent


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


def shortest_heading_delta_deg(current_heading_deg: float, target_heading_deg: float) -> float:
    """Return the shortest signed heading delta from current to target."""

    return normalize_heading_deg(target_heading_deg - current_heading_deg)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric value into one inclusive interval."""

    return max(lower, min(upper, value))


def _usv_heading_alignment_factor(heading_error_deg: float) -> float:
    """Reduce USV speed more aggressively when the heading error is large."""

    return clamp(1.0 - heading_error_deg / 110.0, 0.2, 1.0)


def _uav_heading_alignment_factor(heading_error_deg: float) -> float:
    """Reduce UAV speed more gently when the heading error is large."""

    return clamp(1.0 - heading_error_deg / 240.0, 0.55, 1.0)
