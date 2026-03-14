"""HTML/SVG viewer for the three-zone sea map."""

from __future__ import annotations

import tempfile
import webbrowser
from pathlib import Path

from usv_uav_2_0.agent_overlay import VisualAgent, build_demo_agents, rotate_points
from usv_uav_2_0.environment import (
    CircularFeature,
    MonitoringTarget,
    ObstacleLayout,
    PolygonObstacle,
    SeaMap,
    build_default_sea_map,
    build_obstacle_layout,
)

SVG_WIDTH = 940
SVG_HEIGHT = 860
MAP_LEFT = 120
MAP_TOP = 70
MAP_WIDTH = 700
MAP_HEIGHT = 700


def build_map_html(
    sea_map: SeaMap,
    obstacle_layout: ObstacleLayout | None = None,
) -> str:
    """Build a standalone HTML page that only shows the sea map."""

    layout = obstacle_layout or build_obstacle_layout(sea_map)
    zone_styles = {
        "Nearshore Zone": {
            "fill": "#BFD7EA",
            "accent": "#2F6B99",
            "pattern": "nearshorePattern",
            "note": "deployment and staging band",
        },
        "Middle Risk Zone": {
            "fill": "#F6D6AE",
            "accent": "#C56A1A",
            "pattern": "riskPattern",
            "note": "risk-transition band",
        },
        "Offshore Zone": {
            "fill": "#C9E7CF",
            "accent": "#2F8450",
            "pattern": "offshorePattern",
            "note": "main monitoring band",
        },
    }
    zone_rectangles: list[str] = []
    tick_marks: list[str] = []
    agent_markup = _build_agent_markup(sea_map, build_demo_agents())
    obstacle_markup = _build_obstacle_markup(sea_map, layout)
    monitoring_markup = _build_monitoring_markup(sea_map, layout)

    for zone in sea_map.zones:
        style = zone_styles[zone.name]
        zone_left = MAP_LEFT + (zone.x_start / sea_map.width) * MAP_WIDTH
        zone_width = (zone.width / sea_map.width) * MAP_WIDTH
        zone_rectangles.append(
            f"""
            <g>
              <rect x="{zone_left}" y="{MAP_TOP}" width="{zone_width}" height="{MAP_HEIGHT}"
                    fill="{style['fill']}" stroke="{style['accent']}" stroke-width="1.25" />
              <rect x="{zone_left}" y="{MAP_TOP}" width="{zone_width}" height="{MAP_HEIGHT}"
                    fill="url(#{style['pattern']})" opacity="0.22" />
            </g>
            """
        )

    for value in (0, 250, 450, 1000):
        x = MAP_LEFT + (value / sea_map.width) * MAP_WIDTH
        tick_marks.append(
            f"""
            <g>
              <line x1="{x}" y1="{MAP_TOP + MAP_HEIGHT}" x2="{x}" y2="{MAP_TOP + MAP_HEIGHT + 8}"
                    stroke="#334155" stroke-width="1.5" />
              <text x="{x}" y="{MAP_TOP + MAP_HEIGHT + 28}" text-anchor="middle"
                    font-size="13" fill="#334155">
                {value}
              </text>
            </g>
            """
        )

    return f"""<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>USV-UAV Sea Map</title>
    <style>
      body {{
        margin: 0;
        background:
          radial-gradient(circle at top left, rgba(110, 168, 214, 0.12), transparent 28%),
          linear-gradient(180deg, #eef5fb 0%, #f8fbfd 100%);
        font-family: "Avenir Next", "Segoe UI", Helvetica, Arial, sans-serif;
        color: #0f172a;
      }}
      .page {{
        max-width: 980px;
        margin: 0 auto;
        padding: 20px 18px 28px;
      }}
      .card {{
        background: rgba(255, 255, 255, 0.72);
        border: 1px solid rgba(148, 163, 184, 0.18);
        border-radius: 28px;
        box-shadow: 0 18px 48px rgba(15, 23, 42, 0.06);
        overflow: hidden;
        backdrop-filter: blur(12px);
      }}
      svg {{
        display: block;
        width: 100%;
        height: auto;
      }}
    </style>
  </head>
  <body>
    <div class="page">
      <div class="card">
        <svg viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}" role="img" aria-label="Three-zone marine environment map">
          <defs>
            <pattern id="nearshorePattern" width="24" height="24" patternUnits="userSpaceOnUse">
              <path d="M0 18 Q6 10 12 18 T24 18" fill="none" stroke="#2F6B99" stroke-width="0.9" />
            </pattern>
            <pattern id="riskPattern" width="22" height="22" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
              <line x1="0" y1="0" x2="0" y2="22" stroke="#C56A1A" stroke-width="2.2" />
            </pattern>
            <pattern id="offshorePattern" width="28" height="28" patternUnits="userSpaceOnUse">
              <circle cx="7" cy="7" r="1.8" fill="#2F8450" />
              <circle cx="20" cy="16" r="1.8" fill="#2F8450" />
            </pattern>
            <pattern id="gridPattern" width="50" height="50" patternUnits="userSpaceOnUse">
              <path d="M50 0 L0 0 0 50" fill="none" stroke="rgba(71, 85, 105, 0.08)" stroke-width="1" />
            </pattern>
            <pattern id="rockPattern" width="18" height="18" patternUnits="userSpaceOnUse">
              <circle cx="5" cy="5" r="1.6" fill="rgba(255,255,255,0.28)" />
              <circle cx="13" cy="10" r="1.4" fill="rgba(255,255,255,0.18)" />
            </pattern>
            <linearGradient id="oceanGlow" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" stop-color="#F8FBFD" />
              <stop offset="100%" stop-color="#E0EEF8" />
            </linearGradient>
            <clipPath id="mapClip">
              <rect x="{MAP_LEFT}" y="{MAP_TOP}" width="{MAP_WIDTH}" height="{MAP_HEIGHT}" rx="26" ry="26" />
            </clipPath>
          </defs>
          <rect x="0" y="0" width="{SVG_WIDTH}" height="{SVG_HEIGHT}" fill="url(#oceanGlow)" />
          <circle cx="140" cy="120" r="110" fill="rgba(255,255,255,0.38)" />
          <circle cx="810" cy="120" r="120" fill="rgba(255,255,255,0.24)" />
          <circle cx="820" cy="760" r="130" fill="rgba(160, 196, 224, 0.10)" />
          <rect x="{MAP_LEFT}" y="{MAP_TOP}" width="{MAP_WIDTH}" height="{MAP_HEIGHT}"
                rx="26" ry="26" fill="#ECF5FB" stroke="#4B647D" stroke-width="1.6" />
          <rect x="{MAP_LEFT}" y="{MAP_TOP}" width="{MAP_WIDTH}" height="{MAP_HEIGHT}"
                rx="26" ry="26" fill="url(#gridPattern)" />
          <g clip-path="url(#mapClip)">
            {''.join(zone_rectangles)}
            {obstacle_markup}
            {monitoring_markup}
            {agent_markup}
          </g>
          <line x1="{MAP_LEFT + (250 / sea_map.width) * MAP_WIDTH}" y1="{MAP_TOP}" x2="{MAP_LEFT + (250 / sea_map.width) * MAP_WIDTH}" y2="{MAP_TOP + MAP_HEIGHT}"
                stroke="#8A5A2B" stroke-width="1.4" stroke-dasharray="8 10" opacity="0.52" />
          <line x1="{MAP_LEFT + (450 / sea_map.width) * MAP_WIDTH}" y1="{MAP_TOP}" x2="{MAP_LEFT + (450 / sea_map.width) * MAP_WIDTH}" y2="{MAP_TOP + MAP_HEIGHT}"
                stroke="#6B7280" stroke-width="1.4" stroke-dasharray="8 10" opacity="0.52" />
          {''.join(tick_marks)}
          <line x1="{MAP_LEFT}" y1="{MAP_TOP}" x2="{MAP_LEFT - 8}" y2="{MAP_TOP}"
                stroke="#64748B" stroke-width="1.2" />
          <line x1="{MAP_LEFT}" y1="{MAP_TOP + MAP_HEIGHT / 2}" x2="{MAP_LEFT - 8}" y2="{MAP_TOP + MAP_HEIGHT / 2}"
                stroke="#64748B" stroke-width="1.2" />
          <line x1="{MAP_LEFT}" y1="{MAP_TOP + MAP_HEIGHT}" x2="{MAP_LEFT - 8}" y2="{MAP_TOP + MAP_HEIGHT}"
                stroke="#64748B" stroke-width="1.2" />
          <text x="{MAP_LEFT - 14}" y="{MAP_TOP + 4}" text-anchor="end"
                font-size="11" fill="#64748B">
            1000
          </text>
          <text x="{MAP_LEFT - 14}" y="{MAP_TOP + MAP_HEIGHT / 2 + 4}" text-anchor="end"
                font-size="11" fill="#64748B">
            500
          </text>
          <text x="{MAP_LEFT - 14}" y="{MAP_TOP + MAP_HEIGHT + 4}" text-anchor="end"
                font-size="11" fill="#64748B">
            0
          </text>
          <text x="{MAP_LEFT + MAP_WIDTH / 2}" y="{MAP_TOP + MAP_HEIGHT + 56}"
                text-anchor="middle" font-size="14" fill="#334155">
            X Axis (0 m to {int(sea_map.width)} m)
          </text>
          <text x="22" y="{MAP_TOP + MAP_HEIGHT / 2}" text-anchor="middle"
                font-size="13.5" fill="#334155"
                transform="rotate(-90 22 {MAP_TOP + MAP_HEIGHT / 2})">Y Axis (m)</text>
        </svg>
      </div>
    </div>
  </body>
</html>
"""


def write_map_html(
    output_path: Path,
    sea_map: SeaMap | None = None,
    seed: int | None = None,
) -> Path:
    """Write the sea map HTML page to disk."""

    target_map = sea_map or build_default_sea_map()
    output_path.write_text(
        build_map_html(target_map, build_obstacle_layout(target_map, seed=seed)),
        encoding="utf-8",
    )
    return output_path


def run_map_viewer(
    output_path: Path | None = None,
    open_browser: bool = True,
    seed: int | None = None,
) -> Path:
    """Create the sea map page and optionally open it in the browser."""

    if output_path is None:
        output_path = Path(tempfile.gettempdir()) / "usv_uav_sea_map.html"

    html_path = write_map_html(output_path, build_default_sea_map(), seed=seed)
    if open_browser:
        webbrowser.open(html_path.resolve().as_uri())
    return html_path


def _build_agent_markup(
    sea_map: SeaMap,
    agents: tuple[VisualAgent, ...],
) -> str:
    agent_svgs: list[str] = []
    for agent in agents:
        center_x, center_y = _map_point_to_svg(sea_map, agent.x, agent.y)
        if agent.kind == "USV":
            agent_svgs.append(_build_usv_svg(agent, center_x, center_y))
        else:
            agent_svgs.append(_build_uav_svg(agent, center_x, center_y))
    return "".join(agent_svgs)


def _build_obstacle_markup(sea_map: SeaMap, layout: ObstacleLayout) -> str:
    polygon_svgs = "".join(_build_polygon_obstacle_svg(sea_map, obstacle) for obstacle in layout.risk_zone_obstacles)
    feature_svgs = "".join(_build_offshore_feature_svg(sea_map, feature) for feature in layout.offshore_features)
    return polygon_svgs + feature_svgs


def _build_monitoring_markup(sea_map: SeaMap, layout: ObstacleLayout) -> str:
    baseline_svgs = "".join(
        _build_monitoring_target_svg(sea_map, target)
        for target in layout.nearshore_monitor_points
    )
    hotspot_svgs = "".join(
        _build_monitoring_target_svg(sea_map, target)
        for target in layout.offshore_hotspots
    )
    return baseline_svgs + hotspot_svgs


def _build_polygon_obstacle_svg(sea_map: SeaMap, obstacle: PolygonObstacle) -> str:
    svg_points = " ".join(
        f"{svg_x:.2f},{svg_y:.2f}"
        for svg_x, svg_y in (_map_point_to_svg(sea_map, x, y) for x, y in obstacle.points)
    )
    return f"""
    <g aria-label="{obstacle.name}">
      <polygon points="{svg_points}" fill="#374151" fill-opacity="0.78" stroke="#E2E8F0" stroke-width="1.3" />
      <polygon points="{svg_points}" fill="url(#rockPattern)" opacity="0.22" />
    </g>
    """


def _build_offshore_feature_svg(sea_map: SeaMap, feature: CircularFeature) -> str:
    svg_x, svg_y = _map_point_to_svg(sea_map, feature.x, feature.y)
    svg_radius = (feature.radius / sea_map.width) * MAP_WIDTH
    if feature.feature_type == "islet":
        fill = "#475569"
        stroke = "#E2E8F0"
        overlay = f'<circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="{svg_radius:.2f}" fill="url(#rockPattern)" opacity="0.22" />'
    else:
        fill = "rgba(190, 24, 93, 0.12)"
        stroke = "#9D174D"
        overlay = (
            f'<circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="{svg_radius * 0.55:.2f}" '
            'fill="rgba(244, 114, 182, 0.26)" />'
        )
    return f"""
    <g aria-label="{feature.name}">
      <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="{svg_radius:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="1.3"
              stroke-dasharray="{'none' if feature.feature_type == 'islet' else '7 6'}" />
      {overlay}
    </g>
    """


def _build_monitoring_target_svg(sea_map: SeaMap, target: MonitoringTarget) -> str:
    svg_x, svg_y = _map_point_to_svg(sea_map, target.x, target.y)
    if target.target_type == "baseline_point":
        return f"""
        <g aria-label="{target.name}">
          <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="10.0" fill="rgba(37, 99, 235, 0.14)" />
          <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="5.2" fill="#2563EB" stroke="#EFF6FF" stroke-width="1.8" />
          <path d="M {svg_x - 8:.2f} {svg_y:.2f} L {svg_x + 8:.2f} {svg_y:.2f} M {svg_x:.2f} {svg_y - 8:.2f} L {svg_x:.2f} {svg_y + 8:.2f}"
                stroke="#DBEAFE" stroke-width="1.8" stroke-linecap="round" />
        </g>
        """

    diamond = (
        f"{svg_x:.2f},{svg_y - 9:.2f} "
        f"{svg_x + 9:.2f},{svg_y:.2f} "
        f"{svg_x:.2f},{svg_y + 9:.2f} "
        f"{svg_x - 9:.2f},{svg_y:.2f}"
    )
    return f"""
    <g aria-label="{target.name}">
      <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="13.0" fill="rgba(245, 158, 11, 0.18)" stroke="rgba(120, 53, 15, 0.36)"
              stroke-width="1.1" stroke-dasharray="4 4" />
      <polygon points="{diamond}" fill="#F59E0B" stroke="#7C2D12" stroke-width="1.7" />
      <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="2.2" fill="#FFF7ED" />
    </g>
    """


def _map_point_to_svg(sea_map: SeaMap, x: float, y: float) -> tuple[float, float]:
    svg_x = MAP_LEFT + (x / sea_map.width) * MAP_WIDTH
    svg_y = MAP_TOP + MAP_HEIGHT - (y / sea_map.height) * MAP_HEIGHT
    return svg_x, svg_y


def _build_usv_svg(agent: VisualAgent, center_x: float, center_y: float) -> str:
    hull_points = rotate_points(
        (
            (-9.0, -4.5),
            (6.0, -4.5),
            (10.5, 0.0),
            (6.0, 4.5),
            (-9.0, 4.5),
        ),
        center_x,
        center_y,
        agent.heading_deg,
    )
    wake_points = rotate_points(
        (
            (-13.5, 0.0),
            (-21.0, -3.8),
            (-18.0, 0.0),
            (-21.0, 3.8),
        ),
        center_x,
        center_y,
        agent.heading_deg,
    )
    label_x = center_x
    label_y = center_y - 17
    hull = " ".join(f"{x:.2f},{y:.2f}" for x, y in hull_points)
    wake = " ".join(f"{x:.2f},{y:.2f}" for x, y in wake_points)
    return f"""
    <g aria-label="{agent.agent_id}">
      <polygon points="{wake}" fill="rgba(125, 211, 252, 0.45)" />
      <polygon points="{hull}" fill="#0F172A" stroke="#E2E8F0" stroke-width="1.8" />
      <circle cx="{center_x:.2f}" cy="{center_y:.2f}" r="2.2" fill="#7DD3FC" />
      <rect x="{label_x - 22:.2f}" y="{label_y - 8:.2f}" width="44" height="15" rx="7.5" ry="7.5"
            fill="rgba(255,255,255,0.86)" stroke="rgba(15,23,42,0.10)" />
      <text x="{label_x:.2f}" y="{label_y + 3:.2f}" text-anchor="middle"
            font-size="8.8" font-weight="700" fill="#0F172A">{agent.agent_id}</text>
    </g>
    """


def _build_uav_svg(agent: VisualAgent, center_x: float, center_y: float) -> str:
    frame_points = rotate_points(
        (
            (0.0, -7.0),
            (7.0, 0.0),
            (0.0, 7.0),
            (-7.0, 0.0),
        ),
        center_x,
        center_y,
        agent.heading_deg,
    )
    rotor_offsets = rotate_points(
        (
            (-8.0, -8.0),
            (8.0, -8.0),
            (8.0, 8.0),
            (-8.0, 8.0),
        ),
        center_x,
        center_y,
        agent.heading_deg,
    )
    nose_points = rotate_points(
        (
            (0.0, -9.5),
            (3.0, -5.5),
            (-3.0, -5.5),
        ),
        center_x,
        center_y,
        agent.heading_deg,
    )
    frame = " ".join(f"{x:.2f},{y:.2f}" for x, y in frame_points)
    nose = " ".join(f"{x:.2f},{y:.2f}" for x, y in nose_points)
    rotor_circles = "".join(
        f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3.4" fill="#FFF1F2" stroke="#9F1239" stroke-width="1.2" />'
        for x, y in rotor_offsets
    )
    label_x = center_x
    label_y = center_y - 18
    return f"""
    <g aria-label="{agent.agent_id}">
      {rotor_circles}
      <polygon points="{frame}" fill="#9F1239" stroke="#FFF1F2" stroke-width="1.4" />
      <polygon points="{nose}" fill="#FCA5A5" />
      <circle cx="{center_x:.2f}" cy="{center_y:.2f}" r="2.2" fill="#FFF7ED" />
      <rect x="{label_x - 22:.2f}" y="{label_y - 8:.2f}" width="44" height="15" rx="7.5" ry="7.5"
            fill="rgba(255,255,255,0.88)" stroke="rgba(15,23,42,0.10)" />
      <text x="{label_x:.2f}" y="{label_y + 3:.2f}" text-anchor="middle"
            font-size="8.8" font-weight="700" fill="#7F1D1D">{agent.agent_id}</text>
    </g>
    """
