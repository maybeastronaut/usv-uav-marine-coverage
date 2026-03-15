"""HTML/SVG viewer for the three-zone sea map."""

from __future__ import annotations

import webbrowser
from pathlib import Path
from typing import Literal

from usv_uav_marine_coverage.agent_model import (
    AgentState,
    AgentTaskState,
    default_coverage_radius,
)
from usv_uav_marine_coverage.agent_overlay import VisualAgent, build_demo_agents, rotate_points
from usv_uav_marine_coverage.environment import (
    CircularFeature,
    MonitoringTarget,
    ObstacleLayout,
    PolygonObstacle,
    SeaMap,
    build_default_sea_map,
    build_obstacle_layout,
)
from usv_uav_marine_coverage.grid import apply_agent_coverage, build_grid_coverage_map, build_grid_map

SVG_WIDTH = 940
SVG_HEIGHT = 860
MAP_LEFT = 120
MAP_TOP = 70
MAP_WIDTH = 700
MAP_HEIGHT = 700
ViewMode = Literal["clean", "debug"]


def build_map_html(
    sea_map: SeaMap,
    obstacle_layout: ObstacleLayout | None = None,
    mode: ViewMode = "clean",
    show_coverage_preview: bool | None = None,
    show_footprints: bool | None = None,
) -> str:
    """Build a standalone HTML page that only shows the sea map."""

    layout = obstacle_layout or build_obstacle_layout(sea_map)
    zone_rectangles: list[str] = []
    zone_labels: list[str] = []
    tick_marks: list[str] = []
    demo_agents = build_demo_agents()
    resolved_show_coverage = show_coverage_preview is not False
    resolved_show_footprints = show_footprints is not False
    coverage_markup = _build_coverage_markup(sea_map, layout, demo_agents) if resolved_show_coverage else ""
    footprint_markup = _build_footprint_markup(sea_map, demo_agents) if resolved_show_footprints else ""
    trajectory_markup = _build_trajectory_markup()
    agent_markup = _build_agent_markup(sea_map, demo_agents)
    obstacle_markup = _build_obstacle_markup(sea_map, layout)
    monitoring_markup = _build_monitoring_markup(sea_map, layout)

    for zone in sea_map.zones:
        zone_left = MAP_LEFT + (zone.x_start / sea_map.width) * MAP_WIDTH
        zone_width = (zone.width / sea_map.width) * MAP_WIDTH
        zone_rectangles.append(
            f"""
            <g>
              <rect x="{zone_left}" y="{MAP_TOP}" width="{zone_width}" height="{MAP_HEIGHT}"
                    fill="transparent" stroke="rgba(148, 163, 184, 0.26)" stroke-width="1.0" />
            </g>
            """
        )
        zone_labels.append(
            f"""
            <text x="{zone_left + zone_width / 2:.2f}" y="{MAP_TOP + 22:.2f}" text-anchor="middle"
                  font-size="12" font-weight="600" fill="rgba(100, 116, 139, 0.88)">
              {zone.name}
            </text>
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
        background: #ffffff;
        font-family: "Avenir Next", "Segoe UI", Helvetica, Arial, sans-serif;
        color: #0f172a;
      }}
      .page {{
        max-width: 980px;
        margin: 0 auto;
        padding: 20px 18px 28px;
      }}
      .card {{
        background: #ffffff;
        border: 1px solid rgba(148, 163, 184, 0.16);
        border-radius: 16px;
        box-shadow: 0 10px 24px rgba(15, 23, 42, 0.04);
        overflow: hidden;
      }}
      .toolbar {{
        display: flex;
        align-items: center;
        justify-content: space-between;
        flex-wrap: wrap;
        gap: 12px;
        padding: 14px 16px 0;
      }}
      .toolbar-section {{
        display: inline-flex;
        align-items: center;
        gap: 10px;
      }}
      .toolbar-label {{
        font-size: 12px;
        font-weight: 600;
        letter-spacing: 0.03em;
        color: #64748b;
        text-transform: uppercase;
      }}
      .toolbar-pill {{
        display: inline-flex;
        gap: 8px;
        padding: 4px;
        border: 1px solid rgba(148, 163, 184, 0.26);
        border-radius: 999px;
        background: rgba(248, 250, 252, 0.92);
      }}
      .toolbar-pill button {{
        border: 0;
        border-radius: 999px;
        padding: 7px 12px;
        background: transparent;
        color: #475569;
        font-size: 12px;
        font-weight: 700;
        cursor: pointer;
        transition: background 0.2s ease, color 0.2s ease;
      }}
      body[data-view-mode="clean"] .toolbar-pill.view-toggle button[data-mode="clean"],
      body[data-view-mode="debug"] .toolbar-pill.view-toggle button[data-mode="debug"] {{
        background: #0f172a;
        color: #f8fafc;
      }}
      body[data-show-labels="true"] .toolbar-pill.label-toggle button[data-toggle="labels-on"],
      body[data-show-labels="false"] .toolbar-pill.label-toggle button[data-toggle="labels-off"] {{
        background: #0f172a;
        color: #f8fafc;
      }}
      body[data-view-mode="clean"] .debug-layer {{
        display: none;
      }}
      body[data-show-labels="false"] .agent-label {{
        display: none;
      }}
      svg {{
        display: block;
        width: 100%;
        height: auto;
      }}
    </style>
  </head>
  <body data-view-mode="{mode}" data-show-labels="true">
    <div class="page">
      <div class="card">
        <div class="toolbar">
          <div class="toolbar-section">
            <div class="toolbar-label">Sea Map View</div>
            <div class="toolbar-pill view-toggle" role="group" aria-label="Sea map mode switch">
              <button type="button" data-mode="clean" aria-pressed="{str(mode == 'clean').lower()}" onclick="setViewMode('clean')">
                Clean
              </button>
              <button type="button" data-mode="debug" aria-pressed="{str(mode == 'debug').lower()}" onclick="setViewMode('debug')">
                Debug
              </button>
            </div>
          </div>
          <div class="toolbar-section">
            <div class="toolbar-label">Agent Labels</div>
            <div class="toolbar-pill label-toggle" role="group" aria-label="Agent label switch">
              <button type="button" data-toggle="labels-on" aria-pressed="true" onclick="setLabelVisibility(true)">
                On
              </button>
              <button type="button" data-toggle="labels-off" aria-pressed="false" onclick="setLabelVisibility(false)">
                Off
              </button>
            </div>
          </div>
        </div>
        <svg viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}" role="img" aria-label="Three-zone marine environment map">
          <defs>
            <pattern id="gridPattern" width="50" height="50" patternUnits="userSpaceOnUse">
              <path d="M50 0 L0 0 0 50" fill="none" stroke="rgba(148, 163, 184, 0.10)" stroke-width="1" />
            </pattern>
            <pattern id="rockPattern" width="18" height="18" patternUnits="userSpaceOnUse">
              <circle cx="5" cy="5" r="1.6" fill="rgba(255,255,255,0.28)" />
              <circle cx="13" cy="10" r="1.4" fill="rgba(255,255,255,0.18)" />
            </pattern>
            <clipPath id="mapClip">
              <rect x="{MAP_LEFT}" y="{MAP_TOP}" width="{MAP_WIDTH}" height="{MAP_HEIGHT}" rx="26" ry="26" />
            </clipPath>
          </defs>
          <rect x="0" y="0" width="{SVG_WIDTH}" height="{SVG_HEIGHT}" fill="#FFFFFF" />
          <rect x="{MAP_LEFT}" y="{MAP_TOP}" width="{MAP_WIDTH}" height="{MAP_HEIGHT}"
                rx="26" ry="26" fill="#FFFFFF" stroke="rgba(100, 116, 139, 0.42)" stroke-width="1.2" />
          <rect x="{MAP_LEFT}" y="{MAP_TOP}" width="{MAP_WIDTH}" height="{MAP_HEIGHT}"
                rx="26" ry="26" fill="url(#gridPattern)" />
          <g clip-path="url(#mapClip)">
            {''.join(zone_rectangles)}
            {obstacle_markup}
            {monitoring_markup}
            {coverage_markup}
            {footprint_markup}
            {trajectory_markup}
            {agent_markup}
          </g>
          {''.join(zone_labels)}
          <line x1="{MAP_LEFT + (250 / sea_map.width) * MAP_WIDTH}" y1="{MAP_TOP}" x2="{MAP_LEFT + (250 / sea_map.width) * MAP_WIDTH}" y2="{MAP_TOP + MAP_HEIGHT}"
                stroke="rgba(148, 163, 184, 0.72)" stroke-width="1.1" stroke-dasharray="6 8" />
          <line x1="{MAP_LEFT + (450 / sea_map.width) * MAP_WIDTH}" y1="{MAP_TOP}" x2="{MAP_LEFT + (450 / sea_map.width) * MAP_WIDTH}" y2="{MAP_TOP + MAP_HEIGHT}"
                stroke="rgba(148, 163, 184, 0.72)" stroke-width="1.1" stroke-dasharray="6 8" />
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
    <script>
      function setViewMode(mode) {{
        document.body.setAttribute("data-view-mode", mode);
        document.querySelectorAll(".view-toggle button").forEach((button) => {{
          button.setAttribute("aria-pressed", String(button.dataset.mode === mode));
        }});
      }}
      function setLabelVisibility(isVisible) {{
        document.body.setAttribute("data-show-labels", String(isVisible));
        document.querySelectorAll(".label-toggle button").forEach((button) => {{
          const isOnButton = button.dataset.toggle === "labels-on";
          button.setAttribute("aria-pressed", String(isVisible === isOnButton));
        }});
      }}
    </script>
  </body>
</html>
"""


def write_map_html(
    output_path: Path,
    sea_map: SeaMap | None = None,
    seed: int | None = None,
    mode: ViewMode = "clean",
    show_coverage_preview: bool | None = None,
    show_footprints: bool | None = None,
) -> Path:
    """Write the sea map HTML page to disk."""

    target_map = sea_map or build_default_sea_map()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        build_map_html(
            target_map,
            build_obstacle_layout(target_map, seed=seed),
            mode=mode,
            show_coverage_preview=show_coverage_preview,
            show_footprints=show_footprints,
        ),
        encoding="utf-8",
    )
    return output_path


def run_map_viewer(
    output_path: Path | None = None,
    open_browser: bool = True,
    seed: int | None = None,
    mode: ViewMode = "clean",
    show_coverage_preview: bool | None = None,
    show_footprints: bool | None = None,
) -> Path:
    """Create the sea map page and optionally open it in the browser."""

    if output_path is None:
        output_path = Path.cwd() / "outputs" / "usv_uav_sea_map.html"

    html_path = write_map_html(
        output_path,
        build_default_sea_map(),
        seed=seed,
        mode=mode,
        show_coverage_preview=show_coverage_preview,
        show_footprints=show_footprints,
    )
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


def _build_footprint_markup(
    sea_map: SeaMap,
    agents: tuple[VisualAgent, ...],
) -> str:
    footprint_svgs = "".join(_build_footprint_svg(sea_map, agent) for agent in agents)
    return f'<g class="debug-layer" aria-label="Static Footprint Preview">{footprint_svgs}</g>'


def _build_trajectory_markup() -> str:
    return '<g aria-label="Reserved Trajectory Layer"></g>'


def _build_obstacle_markup(sea_map: SeaMap, layout: ObstacleLayout) -> str:
    polygon_svgs = "".join(_build_polygon_obstacle_svg(sea_map, obstacle) for obstacle in layout.risk_zone_obstacles)
    feature_svgs = "".join(_build_offshore_feature_svg(sea_map, feature) for feature in layout.offshore_features)
    return polygon_svgs + feature_svgs


def _build_coverage_markup(
    sea_map: SeaMap,
    layout: ObstacleLayout,
    agents: tuple[VisualAgent, ...],
) -> str:
    grid_map = build_grid_map(sea_map, layout)
    coverage_map = build_grid_coverage_map(grid_map)

    for step, agent in enumerate(_build_demo_agent_states(agents), start=1):
        apply_agent_coverage(coverage_map, agent, step=step)

    cell_width = (grid_map.cell_size / sea_map.width) * MAP_WIDTH
    cell_height = (grid_map.cell_size / sea_map.height) * MAP_HEIGHT
    covered_cells: list[str] = []

    for row_index, row in enumerate(coverage_map.states):
        for col_index, state in enumerate(row):
            if state.coverage_count == 0:
                continue
            cell = grid_map.cell_at(row_index, col_index)
            svg_x, svg_y = _map_point_to_svg(sea_map, cell.x_min, cell.y_max)
            fill = _coverage_fill_for_state(state.covered_by_usv, state.covered_by_uav)
            covered_cells.append(
                f"""
                <rect x="{svg_x:.2f}" y="{svg_y:.2f}" width="{cell_width:.2f}" height="{cell_height:.2f}"
                      fill="{fill}" stroke="rgba(255,255,255,0.28)" stroke-width="0.5"
                      aria-label="Coverage Cell {row_index}-{col_index}" />
                """
            )

    return f'<g class="debug-layer" aria-label="Static Coverage Preview">{"".join(covered_cells)}</g>'


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
          <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="10.0" fill="rgba(71, 85, 105, 0.10)" />
          <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="5.2" fill="#475569" stroke="#F8FAFC" stroke-width="1.8" />
          <path d="M {svg_x - 8:.2f} {svg_y:.2f} L {svg_x + 8:.2f} {svg_y:.2f} M {svg_x:.2f} {svg_y - 8:.2f} L {svg_x:.2f} {svg_y + 8:.2f}"
                stroke="#CBD5E1" stroke-width="1.8" stroke-linecap="round" />
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


def _build_footprint_svg(sea_map: SeaMap, agent: VisualAgent) -> str:
    svg_x, svg_y = _map_point_to_svg(sea_map, agent.x, agent.y)
    radius_m = default_coverage_radius(agent.kind)
    svg_radius = (radius_m / sea_map.width) * MAP_WIDTH
    if agent.kind == "USV":
        stroke = "rgba(37, 99, 235, 0.72)"
        fill = "rgba(37, 99, 235, 0.08)"
    else:
        stroke = "rgba(13, 148, 136, 0.72)"
        fill = "rgba(13, 148, 136, 0.08)"
    return f"""
    <g aria-label="{agent.agent_id} Footprint">
      <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="{svg_radius:.2f}" fill="{fill}" stroke="{stroke}"
              stroke-width="1.2" stroke-dasharray="6 6" />
    </g>
    """


def _build_demo_agent_states(agents: tuple[VisualAgent, ...]) -> tuple[AgentState, ...]:
    preview_agents: list[AgentState] = []
    for agent in agents:
        coverage_radius = default_coverage_radius(agent.kind)
        preview_agents.append(
            AgentState(
                agent_id=agent.agent_id,
                kind=agent.kind,
                x=agent.x,
                y=agent.y,
                heading_deg=agent.heading_deg,
                speed_mps=0.0,
                max_speed_mps=0.0,
                detection_radius=coverage_radius,
                coverage_radius=coverage_radius,
                task=AgentTaskState(),
            )
        )
    return tuple(preview_agents)


def _coverage_fill_for_state(covered_by_usv: int, covered_by_uav: int) -> str:
    if covered_by_usv > 0 and covered_by_uav > 0:
        return "rgba(234, 179, 8, 0.30)"
    if covered_by_uav > 0:
        return "rgba(13, 148, 136, 0.24)"
    return "rgba(37, 99, 235, 0.22)"


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
      <g class="agent-label">
        <rect x="{label_x - 22:.2f}" y="{label_y - 8:.2f}" width="44" height="15" rx="7.5" ry="7.5"
              fill="rgba(255,255,255,0.86)" stroke="rgba(15,23,42,0.10)" />
        <text x="{label_x:.2f}" y="{label_y + 3:.2f}" text-anchor="middle"
              font-size="8.8" font-weight="700" fill="#0F172A">{agent.agent_id}</text>
      </g>
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
        f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3.4" fill="#ECFEFF" stroke="#0F766E" stroke-width="1.2" />'
        for x, y in rotor_offsets
    )
    label_x = center_x
    label_y = center_y - 18
    return f"""
    <g aria-label="{agent.agent_id}">
      {rotor_circles}
      <polygon points="{frame}" fill="#0F766E" stroke="#F0FDFA" stroke-width="1.4" />
      <polygon points="{nose}" fill="#99F6E4" />
      <circle cx="{center_x:.2f}" cy="{center_y:.2f}" r="2.2" fill="#F0FDFA" />
      <g class="agent-label">
        <rect x="{label_x - 22:.2f}" y="{label_y - 8:.2f}" width="44" height="15" rx="7.5" ry="7.5"
              fill="rgba(255,255,255,0.88)" stroke="rgba(15,23,42,0.10)" />
        <text x="{label_x:.2f}" y="{label_y + 3:.2f}" text-anchor="middle"
              font-size="8.8" font-weight="700" fill="#134E4A">{agent.agent_id}</text>
      </g>
    </g>
    """
