"""HTML/SVG replay view for the preview simulation stage."""

from __future__ import annotations

import json

from usv_uav_marine_coverage.grid import build_grid_map
from usv_uav_marine_coverage.viewer import (
    MAP_HEIGHT,
    MAP_LEFT,
    MAP_TOP,
    MAP_WIDTH,
    SVG_HEIGHT,
    SVG_WIDTH,
    _build_footprint_svg,
    _build_monitoring_markup,
    _build_obstacle_markup,
    _build_uav_svg,
    _build_usv_svg,
    _map_point_to_svg,
)
from .simulation_core import SimulationFrame, SimulationReplay


def build_simulation_html(replay: SimulationReplay) -> str:
    """Build one standalone HTML replay page for the current simulation preview."""

    sea_map = replay.sea_map
    layout = replay.obstacle_layout
    zone_rectangles: list[str] = []
    zone_labels: list[str] = []
    tick_marks: list[str] = []

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

    frame_groups = "".join(
        _build_frame_group(replay, frame, is_visible=index == 0)
        for index, frame in enumerate(replay.frames)
    )
    frame_summaries = _build_frame_summaries(replay.frames)
    max_step = replay.frames[-1].step

    return f"""<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>USV-UAV Simulation Replay</title>
    <style>
      body {{
        margin: 0;
        background: #ffffff;
        font-family: "Avenir Next", "Segoe UI", Helvetica, Arial, sans-serif;
        color: #0f172a;
      }}
      .page {{
        max-width: 1240px;
        margin: 0 auto;
        padding: 20px 18px 28px;
      }}
      .layout {{
        display: grid;
        grid-template-columns: minmax(0, 980px) 220px;
        gap: 16px;
        align-items: start;
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
      }}
      body[data-show-labels="true"] .toolbar-pill.label-toggle button[data-toggle="labels-on"],
      body[data-show-labels="false"] .toolbar-pill.label-toggle button[data-toggle="labels-off"] {{
        background: #0f172a;
        color: #f8fafc;
      }}
      body[data-show-footprints="true"] .toolbar-pill.footprint-toggle button[data-toggle="footprints-on"],
      body[data-show-footprints="false"] .toolbar-pill.footprint-toggle button[data-toggle="footprints-off"] {{
        background: #0f172a;
        color: #f8fafc;
      }}
      body[data-show-labels="false"] .agent-label {{
        display: none;
      }}
      body[data-show-footprints="false"] .footprint-layer {{
        display: none;
      }}
      .timeline {{
        display: flex;
        align-items: center;
        gap: 12px;
        padding: 8px 16px 14px;
      }}
      .timeline button {{
        border: 0;
        border-radius: 999px;
        background: #0f172a;
        color: #f8fafc;
        padding: 7px 12px;
        cursor: pointer;
        font-size: 12px;
        font-weight: 700;
      }}
      .timeline input[type="range"] {{
        flex: 1;
      }}
      .step-badge {{
        min-width: 80px;
        text-align: right;
        font-size: 12px;
        font-weight: 700;
        color: #334155;
      }}
      .side-card {{
        padding: 14px 14px 16px;
      }}
      .side-title {{
        margin: 0 0 10px;
        font-size: 13px;
        font-weight: 800;
        color: #0f172a;
        text-transform: uppercase;
        letter-spacing: 0.04em;
      }}
      .stat {{
        display: flex;
        justify-content: space-between;
        gap: 12px;
        padding: 8px 0;
        border-bottom: 1px solid rgba(226, 232, 240, 0.92);
        font-size: 13px;
      }}
      .events {{
        margin: 10px 0 0;
        padding-left: 18px;
        font-size: 12px;
        color: #475569;
      }}
      .events li {{
        margin: 0 0 6px;
      }}
      .empty-events {{
        margin: 10px 0 0;
        font-size: 12px;
        color: #94a3b8;
      }}
      svg {{
        display: block;
        width: 100%;
        height: auto;
      }}
      @media (max-width: 1140px) {{
        .layout {{
          grid-template-columns: 1fr;
        }}
      }}
    </style>
  </head>
  <body data-show-labels="true" data-show-footprints="true">
    <div class="page">
      <div class="layout">
        <div class="card">
          <div class="toolbar">
            <div class="toolbar-section">
              <div class="toolbar-label">Replay Controls</div>
            </div>
            <div class="toolbar-section">
              <div class="toolbar-label">Agent Labels</div>
              <div class="toolbar-pill label-toggle" role="group" aria-label="Simulation label switch">
                <button type="button" data-toggle="labels-on" aria-pressed="true" onclick="setLabelVisibility(true)">On</button>
                <button type="button" data-toggle="labels-off" aria-pressed="false" onclick="setLabelVisibility(false)">Off</button>
              </div>
            </div>
            <div class="toolbar-section">
              <div class="toolbar-label">Footprints</div>
              <div class="toolbar-pill footprint-toggle" role="group" aria-label="Simulation footprint switch">
                <button type="button" data-toggle="footprints-on" aria-pressed="true" onclick="setFootprintVisibility(true)">On</button>
                <button type="button" data-toggle="footprints-off" aria-pressed="false" onclick="setFootprintVisibility(false)">Off</button>
              </div>
            </div>
          </div>
          <div class="timeline">
            <button type="button" onclick="togglePlayback()" id="playButton">Play</button>
            <input type="range" min="0" max="{max_step}" value="0" id="timeline" oninput="setFrame(Number(this.value))" />
            <div class="step-badge" id="stepBadge">Step 0</div>
          </div>
          <svg viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}" role="img" aria-label="USV-UAV simulation replay map">
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
              {_build_obstacle_markup(sea_map, layout)}
              {_build_monitoring_markup(sea_map, layout)}
              {frame_groups}
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
            <text x="{MAP_LEFT - 14}" y="{MAP_TOP + 4}" text-anchor="end" font-size="11" fill="#64748B">1000</text>
            <text x="{MAP_LEFT - 14}" y="{MAP_TOP + MAP_HEIGHT / 2 + 4}" text-anchor="end" font-size="11" fill="#64748B">500</text>
            <text x="{MAP_LEFT - 14}" y="{MAP_TOP + MAP_HEIGHT + 4}" text-anchor="end" font-size="11" fill="#64748B">0</text>
            <text x="{MAP_LEFT + MAP_WIDTH / 2}" y="{MAP_TOP + MAP_HEIGHT + 56}" text-anchor="middle" font-size="14" fill="#334155">
              X Axis (0 m to {int(sea_map.width)} m)
            </text>
            <text x="22" y="{MAP_TOP + MAP_HEIGHT / 2}" text-anchor="middle"
                  font-size="13.5" fill="#334155"
                  transform="rotate(-90 22 {MAP_TOP + MAP_HEIGHT / 2})">Y Axis (m)</text>
          </svg>
        </div>
        <div class="card side-card">
          <h2 class="side-title">Simulation Summary</h2>
          <div class="stat"><span>Coverage</span><strong id="coverageRatio">0.0%</strong></div>
          <div class="stat"><span>Valid Cells</span><strong id="validCells">0</strong></div>
          <div class="stat"><span>Suspected</span><strong id="suspectedCells">0</strong></div>
          <div class="stat"><span>Confirmed</span><strong id="confirmedCells">0</strong></div>
          <div class="stat"><span>False Alarms</span><strong id="falseAlarmCells">0</strong></div>
          <h2 class="side-title" style="margin-top:14px;">Events</h2>
          <ul class="events" id="eventList"></ul>
          <div class="empty-events" id="emptyEvents">No notable event in this step.</div>
        </div>
      </div>
    </div>
    <script>
      const frameSummaries = {frame_summaries};
      let currentFrame = 0;
      let isPlaying = false;
      let playTimer = null;

      function setFrame(step) {{
        currentFrame = step;
        document.querySelectorAll(".simulation-frame").forEach((frame) => {{
          frame.style.display = Number(frame.dataset.step) === step ? "block" : "none";
        }});
        document.getElementById("timeline").value = String(step);
        document.getElementById("stepBadge").textContent = "Step " + String(step);

        const summary = frameSummaries[step];
        document.getElementById("coverageRatio").textContent = summary.coverage_ratio;
        document.getElementById("validCells").textContent = String(summary.valid_cells);
        document.getElementById("suspectedCells").textContent = String(summary.suspected_cells);
        document.getElementById("confirmedCells").textContent = String(summary.confirmed_cells);
        document.getElementById("falseAlarmCells").textContent = String(summary.false_alarm_cells);

        const eventList = document.getElementById("eventList");
        eventList.innerHTML = "";
        if (summary.events.length === 0) {{
          document.getElementById("emptyEvents").style.display = "block";
        }} else {{
          document.getElementById("emptyEvents").style.display = "none";
          summary.events.forEach((event) => {{
            const item = document.createElement("li");
            item.textContent = event;
            eventList.appendChild(item);
          }});
        }}
      }}

      function togglePlayback() {{
        if (isPlaying) {{
          stopPlayback();
          return;
        }}
        isPlaying = true;
        document.getElementById("playButton").textContent = "Pause";
        playTimer = window.setInterval(() => {{
          const nextFrame = currentFrame >= frameSummaries.length - 1 ? 0 : currentFrame + 1;
          setFrame(nextFrame);
        }}, 500);
      }}

      function stopPlayback() {{
        isPlaying = false;
        document.getElementById("playButton").textContent = "Play";
        if (playTimer !== null) {{
          window.clearInterval(playTimer);
          playTimer = null;
        }}
      }}

      function setLabelVisibility(isVisible) {{
        document.body.setAttribute("data-show-labels", String(isVisible));
        document.querySelectorAll(".label-toggle button").forEach((button) => {{
          const isOnButton = button.dataset.toggle === "labels-on";
          button.setAttribute("aria-pressed", String(isVisible === isOnButton));
        }});
      }}

      function setFootprintVisibility(isVisible) {{
        document.body.setAttribute("data-show-footprints", String(isVisible));
        document.querySelectorAll(".footprint-toggle button").forEach((button) => {{
          const isOnButton = button.dataset.toggle === "footprints-on";
          button.setAttribute("aria-pressed", String(isVisible === isOnButton));
        }});
      }}

      setFrame(0);
    </script>
  </body>
</html>
"""


def _build_frame_group(replay: SimulationReplay, frame: SimulationFrame, *, is_visible: bool) -> str:
    sea_map = replay.sea_map
    grid_map = build_grid_map(sea_map, replay.obstacle_layout)
    trajectory_markup = _build_trajectory_markup(sea_map, frame)
    footprint_markup = _build_frame_footprints(sea_map, frame.agents)
    agent_markup = _build_frame_agents(sea_map, frame.agents)
    return f"""
    <g class="simulation-frame" data-step="{frame.step}" style="display:{'block' if is_visible else 'none'};">
      {_build_valid_info_markup(sea_map, grid_map, frame.valid_cells)}
      {_build_covered_markup(sea_map, grid_map, frame.covered_cells)}
      {_build_hotspot_markup(sea_map, grid_map, frame.suspected_cells, frame.confirmed_cells, frame.false_alarm_cells)}
      {footprint_markup}
      {trajectory_markup}
      {agent_markup}
    </g>
    """


def _build_valid_info_markup(sea_map: SeaMap, grid_map, indices: tuple[tuple[int, int], ...]) -> str:
    cell_width = (grid_map.cell_size / sea_map.width) * MAP_WIDTH
    cell_height = (grid_map.cell_size / sea_map.height) * MAP_HEIGHT
    cells = []
    for row, col in indices:
        cell = grid_map.cell_at(row, col)
        svg_x, svg_y = _map_point_to_svg(sea_map, cell.x_min, cell.y_max)
        cells.append(
            f'<rect x="{svg_x:.2f}" y="{svg_y:.2f}" width="{cell_width:.2f}" height="{cell_height:.2f}" '
            'fill="rgba(56, 189, 248, 0.08)" stroke="none" />'
        )
    return f'<g aria-label="Valid Information Cells">{"".join(cells)}</g>'


def _build_covered_markup(sea_map: SeaMap, grid_map, indices: tuple[tuple[int, int], ...]) -> str:
    cell_width = (grid_map.cell_size / sea_map.width) * MAP_WIDTH
    cell_height = (grid_map.cell_size / sea_map.height) * MAP_HEIGHT
    cells = []
    for row, col in indices:
        cell = grid_map.cell_at(row, col)
        svg_x, svg_y = _map_point_to_svg(sea_map, cell.x_min, cell.y_max)
        cells.append(
            f'<rect x="{svg_x:.2f}" y="{svg_y:.2f}" width="{cell_width:.2f}" height="{cell_height:.2f}" '
            'fill="rgba(234, 179, 8, 0.10)" stroke="none" />'
        )
    return f'<g aria-label="Covered Cells">{"".join(cells)}</g>'


def _build_hotspot_markup(
    sea_map: SeaMap,
    grid_map,
    suspected_cells: tuple[tuple[int, int], ...],
    confirmed_cells: tuple[tuple[int, int], ...],
    false_alarm_cells: tuple[tuple[int, int], ...],
) -> str:
    elements: list[str] = []
    for row, col in suspected_cells:
        elements.append(_build_marker_svg(sea_map, grid_map.cell_at(row, col), "#F59E0B", "rgba(245, 158, 11, 0.18)", "suspected"))
    for row, col in confirmed_cells:
        elements.append(_build_marker_svg(sea_map, grid_map.cell_at(row, col), "#DC2626", "rgba(248, 113, 113, 0.18)", "confirmed"))
    for row, col in false_alarm_cells:
        elements.append(_build_false_alarm_svg(sea_map, grid_map.cell_at(row, col)))
    return f'<g aria-label="Hotspot Knowledge Layer">{"".join(elements)}</g>'


def _build_marker_svg(sea_map: SeaMap, cell, stroke: str, fill: str, label: str) -> str:
    svg_x, svg_y = _map_point_to_svg(sea_map, cell.center_x, cell.center_y)
    return f"""
    <g aria-label="{label.title()} Hotspot {cell.row}-{cell.col}">
      <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="11.0" fill="{fill}" stroke="{stroke}" stroke-width="1.5" stroke-dasharray="4 3" />
      <polygon points="{svg_x:.2f},{svg_y - 8:.2f} {svg_x + 8:.2f},{svg_y:.2f} {svg_x:.2f},{svg_y + 8:.2f} {svg_x - 8:.2f},{svg_y:.2f}"
               fill="{stroke}" opacity="0.88" />
    </g>
    """


def _build_false_alarm_svg(sea_map: SeaMap, cell) -> str:
    svg_x, svg_y = _map_point_to_svg(sea_map, cell.center_x, cell.center_y)
    return f"""
    <g aria-label="False Alarm {cell.row}-{cell.col}">
      <circle cx="{svg_x:.2f}" cy="{svg_y:.2f}" r="10.0" fill="rgba(148, 163, 184, 0.10)" stroke="#64748B" stroke-width="1.3" />
      <path d="M {svg_x - 6:.2f} {svg_y - 6:.2f} L {svg_x + 6:.2f} {svg_y + 6:.2f} M {svg_x + 6:.2f} {svg_y - 6:.2f} L {svg_x - 6:.2f} {svg_y + 6:.2f}"
            stroke="#64748B" stroke-width="1.6" stroke-linecap="round" />
    </g>
    """


def _build_frame_footprints(sea_map: SeaMap, agents) -> str:
    return f'<g class="footprint-layer" aria-label="Replay Footprints">{"".join(_build_footprint_svg(sea_map, agent) for agent in agents)}</g>'


def _build_trajectory_markup(sea_map: SeaMap, frame: SimulationFrame) -> str:
    segments: list[str] = []
    for agent_id, points in frame.trajectories:
        if len(points) < 2:
            continue
        svg_points = " ".join(
            f"{_map_point_to_svg(sea_map, x, y)[0]:.2f},{_map_point_to_svg(sea_map, x, y)[1]:.2f}"
            for x, y in points
        )
        stroke = "#0F766E" if agent_id.startswith("UAV") else "#1D4ED8"
        segments.append(
            f'<polyline points="{svg_points}" fill="none" stroke="{stroke}" stroke-width="2.0" opacity="0.55" stroke-linecap="round" stroke-linejoin="round" />'
        )
    return f'<g aria-label="Trajectory Layer">{"".join(segments)}</g>'


def _build_frame_agents(sea_map: SeaMap, agents) -> str:
    elements: list[str] = []
    for agent in agents:
        center_x, center_y = _map_point_to_svg(sea_map, agent.x, agent.y)
        if agent.kind == "USV":
            elements.append(_build_usv_svg(agent, center_x, center_y))
        else:
            elements.append(_build_uav_svg(agent, center_x, center_y))
    return "".join(elements)


def _build_frame_summaries(frames: tuple[SimulationFrame, ...]) -> str:
    summary_items = []
    for frame in frames:
        summary_items.append(
            {
                "step": frame.step,
                "coverage_ratio": f"{frame.coverage_ratio * 100:.1f}%",
                "valid_cells": len(frame.valid_cells),
                "suspected_cells": len(frame.suspected_cells),
                "confirmed_cells": len(frame.confirmed_cells),
                "false_alarm_cells": len(frame.false_alarm_cells),
                "events": list(frame.events),
            }
        )
    return json.dumps(summary_items, ensure_ascii=True)
