"""Static agent overlay for visual preview on the sea map."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin


@dataclass(frozen=True)
class VisualAgent:
    """A static agent used only for HTML appearance preview."""

    agent_id: str
    kind: str
    x: float
    y: float
    heading_deg: float


def build_demo_agents() -> tuple[VisualAgent, ...]:
    """Return a fixed agent layout for visual inspection."""

    return (
        VisualAgent(agent_id="USV-1", kind="USV", x=120.0, y=180.0, heading_deg=8.0),
        VisualAgent(agent_id="USV-2", kind="USV", x=165.0, y=520.0, heading_deg=-12.0),
        VisualAgent(agent_id="USV-3", kind="USV", x=215.0, y=810.0, heading_deg=5.0),
        VisualAgent(agent_id="UAV-1", kind="UAV", x=610.0, y=250.0, heading_deg=-28.0),
        VisualAgent(agent_id="UAV-2", kind="UAV", x=760.0, y=670.0, heading_deg=35.0),
    )


def rotate_points(
    points: tuple[tuple[float, float], ...],
    center_x: float,
    center_y: float,
    heading_deg: float,
) -> tuple[tuple[float, float], ...]:
    """Rotate relative points around a center for SVG rendering."""

    angle = radians(heading_deg)
    cos_angle = cos(angle)
    sin_angle = sin(angle)
    rotated: list[tuple[float, float]] = []

    for rel_x, rel_y in points:
        rot_x = rel_x * cos_angle - rel_y * sin_angle
        rot_y = rel_x * sin_angle + rel_y * cos_angle
        rotated.append((center_x + rot_x, center_y + rot_y))

    return tuple(rotated)
