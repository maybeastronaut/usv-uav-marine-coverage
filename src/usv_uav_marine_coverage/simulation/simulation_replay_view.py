"""HTML/SVG replay view for the preview simulation stage."""

# ruff: noqa: E501

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
    _build_monitoring_markup,
    _build_obstacle_markup,
)

from .simulation_core import SimulationReplay


def build_simulation_html(replay: SimulationReplay) -> str:
    """Build one standalone HTML replay page for the current simulation preview."""

    sea_map = replay.sea_map
    layout = replay.obstacle_layout
    grid_map = _build_replay_grid_map(sea_map, layout)
    page_title = "USV-UAV Simulation Replay"
    summary_title = "Simulation Summary"
    replay_badge = "Task Simulation"
    zone_rectangles: list[str] = []
    zone_labels: list[str] = []
    tick_marks: list[str] = []
    y_label_values = (
        _format_distance_label(sea_map.height),
        _format_distance_label(sea_map.height / 2.0),
        _format_distance_label(0.0),
    )

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

    for value in _build_x_tick_values(sea_map):
        x = MAP_LEFT + (value / sea_map.width) * MAP_WIDTH
        tick_marks.append(
            f"""
            <g>
              <line x1="{x}" y1="{MAP_TOP + MAP_HEIGHT}" x2="{x}" y2="{MAP_TOP + MAP_HEIGHT + 8}"
                    stroke="#334155" stroke-width="1.5" />
              <text x="{x}" y="{MAP_TOP + MAP_HEIGHT + 28}" text-anchor="middle"
                    font-size="13" fill="#334155">
                {_format_distance_label(value)}
              </text>
            </g>
            """
        )

    frame_payloads = _build_frame_payloads(replay, grid_map)
    frame_summaries = _build_frame_summaries(replay)
    planned_path_payloads = _build_planned_path_payloads(replay)
    agent_static_payloads = _build_agent_static_payloads(replay)
    max_step = replay.frames[-1].step

    return f"""<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>{page_title}</title>
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
              <div class="toolbar-pill" aria-label="Replay kind">
                <button type="button" disabled>{replay_badge}</button>
              </div>
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
          <svg viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}" role="img" aria-label="{page_title}">
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
              {"".join(zone_rectangles)}
              {_build_obstacle_markup(sea_map, layout)}
              {_build_monitoring_markup(sea_map, layout)}
              <g id="validInfoLayer"></g>
              <g id="staleInfoLayer"></g>
              <g id="coveredLayer"></g>
              <g id="baselineLayer"></g>
              <g id="hotspotLayer"></g>
            <g id="footprintLayer" class="footprint-layer" aria-label="Replay Footprints"></g>
            <g id="trajectoryLayer" aria-label="Planned Path Layer"></g>
            <g id="agentLayer"></g>
            </g>
            {"".join(zone_labels)}
            {"".join(_build_zone_boundary_lines(sea_map))}
            {"".join(tick_marks)}
            <line x1="{MAP_LEFT}" y1="{MAP_TOP}" x2="{MAP_LEFT - 8}" y2="{MAP_TOP}"
                  stroke="#64748B" stroke-width="1.2" />
            <line x1="{MAP_LEFT}" y1="{MAP_TOP + MAP_HEIGHT / 2}" x2="{MAP_LEFT - 8}" y2="{MAP_TOP + MAP_HEIGHT / 2}"
                  stroke="#64748B" stroke-width="1.2" />
            <line x1="{MAP_LEFT}" y1="{MAP_TOP + MAP_HEIGHT}" x2="{MAP_LEFT - 8}" y2="{MAP_TOP + MAP_HEIGHT}"
                  stroke="#64748B" stroke-width="1.2" />
            <text x="{MAP_LEFT - 14}" y="{MAP_TOP + 4}" text-anchor="end" font-size="11" fill="#64748B">{y_label_values[0]}</text>
            <text x="{MAP_LEFT - 14}" y="{MAP_TOP + MAP_HEIGHT / 2 + 4}" text-anchor="end" font-size="11" fill="#64748B">{y_label_values[1]}</text>
            <text x="{MAP_LEFT - 14}" y="{MAP_TOP + MAP_HEIGHT + 4}" text-anchor="end" font-size="11" fill="#64748B">{y_label_values[2]}</text>
            <text x="{MAP_LEFT + MAP_WIDTH / 2}" y="{MAP_TOP + MAP_HEIGHT + 56}" text-anchor="middle" font-size="14" fill="#334155">
              X Axis (0 m to {int(sea_map.width)} m)
            </text>
            <text x="22" y="{MAP_TOP + MAP_HEIGHT / 2}" text-anchor="middle"
                  font-size="13.5" fill="#334155"
                  transform="rotate(-90 22 {MAP_TOP + MAP_HEIGHT / 2})">Y Axis (m)</text>
          </svg>
        </div>
        <div class="card side-card">
          <h2 class="side-title">{summary_title}</h2>
          <div class="stat"><span>Coverage</span><strong id="coverageRatio">0.0%</strong></div>
          <div class="stat"><span>Covered Cells</span><strong id="coveredCells">0</strong></div>
          <div class="stat"><span>Current Observed Cells</span><strong id="currentObservedCells">0</strong></div>
          <div class="stat"><span>Valid Cells</span><strong id="validCells">0</strong></div>
          <div class="stat"><span>Stale Cells</span><strong id="staleCells">0</strong></div>
          <div class="stat"><span>Active Hotspots</span><strong id="activeHotspots">0</strong></div>
          <div class="stat"><span>UAV Checked Marks</span><strong id="uavCheckedMarks">0</strong></div>
          <div class="stat"><span>Confirmed Hotspots</span><strong id="confirmedHotspots">0</strong></div>
          <h2 class="side-title" style="margin-top:14px;">Events</h2>
          <ul class="events" id="eventList"></ul>
          <div class="empty-events" id="emptyEvents">No notable event in this step.</div>
        </div>
      </div>
    </div>
    <script>
      const agentDisplayScale = 1.0;
      const frameRenderData = {frame_payloads};
      const frameSummaries = {frame_summaries};
      const plannedPathData = {planned_path_payloads};
      const agentStaticData = {agent_static_payloads};
      const gridRows = {grid_map.rows};
      const gridCols = {grid_map.cols};
      const gridCellSize = {grid_map.cell_size};
      const maxFrameIndex = frameSummaries.length - 1;
      const stepDurationMs = 180;
      let currentFrame = 0;
      let currentFrameProgress = 0;
      let isPlaying = false;
      let animationHandle = null;
      let playbackStartTime = null;
      let playbackStartFrame = 0;
      let lastStaticFrame = -1;
      const validInfoLayer = document.getElementById("validInfoLayer");
      const staleInfoLayer = document.getElementById("staleInfoLayer");
      const coveredLayer = document.getElementById("coveredLayer");
      const baselineLayer = document.getElementById("baselineLayer");
      const hotspotLayer = document.getElementById("hotspotLayer");
      const footprintLayer = document.getElementById("footprintLayer");
      const trajectoryLayer = document.getElementById("trajectoryLayer");
      const agentLayer = document.getElementById("agentLayer");

      function mapPointToSvg(x, y) {{
        const svgX = {MAP_LEFT} + (x / {replay.sea_map.width}) * {MAP_WIDTH};
        const svgY = {MAP_TOP + MAP_HEIGHT} - (y / {replay.sea_map.height}) * {MAP_HEIGHT};
        return [svgX, svgY];
      }}

      function cellIdToRowCol(cellId) {{
        const row = Math.floor(cellId / gridCols);
        const col = cellId % gridCols;
        return [row, col];
      }}

      function cellBounds(cellId) {{
        const [row, col] = cellIdToRowCol(cellId);
        const xMin = col * gridCellSize;
        const yMin = row * gridCellSize;
        const xMax = xMin + gridCellSize;
        const yMax = yMin + gridCellSize;
        return {{ xMin, yMin, xMax, yMax }};
      }}

      function cellCenter(cellId) {{
        const bounds = cellBounds(cellId);
        return {{
          x: bounds.xMin + gridCellSize / 2,
          y: bounds.yMin + gridCellSize / 2,
          row: Math.floor(cellId / gridCols),
          col: cellId % gridCols,
        }};
      }}

      function rotatePoints(points, centerX, centerY, headingDeg) {{
        const angle = (-headingDeg * Math.PI) / 180;
        const cosAngle = Math.cos(angle);
        const sinAngle = Math.sin(angle);
        return points.map(([relX, relY]) => {{
          const rotX = relX * cosAngle - relY * sinAngle;
          const rotY = relX * sinAngle + relY * cosAngle;
          return [centerX + rotX, centerY + rotY];
        }});
      }}

      function formatPoints(points) {{
        return points.map(([x, y]) => `${{x.toFixed(2)}},${{y.toFixed(2)}}`).join(" ");
      }}

      function scalePoints(points, factor) {{
        return points.map(([x, y]) => [x * factor, y * factor]);
      }}

      function interpolateHeading(startDeg, endDeg, progress) {{
        let delta = ((endDeg - startDeg + 540) % 360) - 180;
        return startDeg + delta * progress;
      }}

      function headingUnitVector(headingDeg) {{
        const angle = (headingDeg * Math.PI) / 180;
        return [Math.cos(angle), Math.sin(angle)];
      }}

      function hermitePoint(startAgent, endAgent, progress, tangentScale) {{
        const t = progress;
        const t2 = t * t;
        const t3 = t2 * t;
        const h00 = 2 * t3 - 3 * t2 + 1;
        const h10 = t3 - 2 * t2 + t;
        const h01 = -2 * t3 + 3 * t2;
        const h11 = t3 - t2;
        const [startTx, startTy] = headingUnitVector(startAgent.heading_deg);
        const [endTx, endTy] = headingUnitVector(endAgent.heading_deg);
        return {{
          x:
            h00 * startAgent.x +
            h10 * startTx * tangentScale +
            h01 * endAgent.x +
            h11 * endTx * tangentScale,
          y:
            h00 * startAgent.y +
            h10 * startTy * tangentScale +
            h01 * endAgent.y +
            h11 * endTy * tangentScale,
        }};
      }}

      function interpolateUsvPose(agent, nextAgent, progress) {{
        const dx = nextAgent.x - agent.x;
        const dy = nextAgent.y - agent.y;
        const chordLength = Math.hypot(dx, dy);
        if (chordLength <= 1e-6) {{
          return {{
            x: agent.x,
            y: agent.y,
            heading_deg: interpolateHeading(agent.heading_deg, nextAgent.heading_deg, progress),
          }};
        }}
        const tangentScale = chordLength * 0.9;
        const point = hermitePoint(agent, nextAgent, progress, tangentScale);
        return {{
          x: point.x,
          y: point.y,
          heading_deg: interpolateHeading(agent.heading_deg, nextAgent.heading_deg, progress),
        }};
      }}

      function interpolateAgents(frameIndex, progress) {{
        const currentAgents = frameRenderData[frameIndex].agents;
        const nextAgents =
          frameIndex >= maxFrameIndex ? currentAgents : frameRenderData[frameIndex + 1].agents;
        return currentAgents.map((agent, index) => {{
          const nextAgent = nextAgents[index] || agent;
          if (agent.kind === "USV") {{
            const pose = interpolateUsvPose(agent, nextAgent, progress);
            return {{
              ...agent,
              x: pose.x,
              y: pose.y,
              heading_deg: pose.heading_deg,
            }};
          }}
          return {{
            ...agent,
            x: agent.x + (nextAgent.x - agent.x) * progress,
            y: agent.y + (nextAgent.y - agent.y) * progress,
            heading_deg: interpolateHeading(agent.heading_deg, nextAgent.heading_deg, progress),
          }};
        }});
      }}

      function renderFootprints(agents) {{
        return agents
          .map((agent) => {{
            const [svgX, svgY] = mapPointToSvg(agent.x, agent.y);
            const radiusM =
              agentStaticData[agent.agent_id]?.coverage_radius ??
              (agent.kind === "USV" ? 50.0 : 100.0);
            const svgRadius = (radiusM / {replay.sea_map.width}) * {MAP_WIDTH};
            const stroke =
              agent.kind === "USV" ? "rgba(37, 99, 235, 0.72)" : "rgba(13, 148, 136, 0.72)";
            const fill =
              agent.kind === "USV" ? "rgba(37, 99, 235, 0.08)" : "rgba(13, 148, 136, 0.08)";
            return `
              <g aria-label="${{agent.agent_id}} Footprint">
                <circle cx="${{svgX.toFixed(2)}}" cy="${{svgY.toFixed(2)}}" r="${{svgRadius.toFixed(2)}}"
                        fill="${{fill}}" stroke="${{stroke}}" stroke-width="1.2" stroke-dasharray="6 6" />
              </g>
            `;
          }})
          .join("");
      }}

      function renderUSV(agent) {{
        const [centerX, centerY] = mapPointToSvg(agent.x, agent.y);
        const scale = agentDisplayScale;
        const hull = formatPoints(
          rotatePoints(
            scalePoints(
              [
                [-9.0, -4.5],
                [6.0, -4.5],
                [10.5, 0.0],
                [6.0, 4.5],
                [-9.0, 4.5],
              ],
              scale,
            ),
            centerX,
            centerY,
            agent.heading_deg,
          ),
        );
        const wake = formatPoints(
          rotatePoints(
            scalePoints(
              [
                [-13.5, 0.0],
                [-21.0, -3.8],
                [-18.0, 0.0],
                [-21.0, 3.8],
              ],
              scale,
            ),
            centerX,
            centerY,
            agent.heading_deg,
          ),
        );
        const labelX = centerX;
        const labelY = centerY - 17 * scale;
        return `
          <g aria-label="${{agent.agent_id}}">
            <polygon points="${{wake}}" fill="rgba(125, 211, 252, 0.45)" />
            <polygon points="${{hull}}" fill="#0F172A" stroke="#E2E8F0" stroke-width="1.8" />
            <circle cx="${{centerX.toFixed(2)}}" cy="${{centerY.toFixed(2)}}" r="${{(2.2 * scale).toFixed(2)}}" fill="#7DD3FC" />
            <g class="agent-label">
              <rect x="${{(labelX - 22).toFixed(2)}}" y="${{(labelY - 8).toFixed(2)}}" width="44" height="15" rx="7.5" ry="7.5"
                    fill="rgba(255,255,255,0.86)" stroke="rgba(15,23,42,0.10)" />
              <text x="${{labelX.toFixed(2)}}" y="${{(labelY + 3).toFixed(2)}}" text-anchor="middle"
                    font-size="8.8" font-weight="700" fill="#0F172A">${{agent.agent_id}}</text>
            </g>
          </g>
        `;
      }}

      function renderUAV(agent) {{
        const [centerX, centerY] = mapPointToSvg(agent.x, agent.y);
        const scale = agentDisplayScale;
        const frame = formatPoints(
          rotatePoints(
            scalePoints(
              [
                [0.0, -7.0],
                [7.0, 0.0],
                [0.0, 7.0],
                [-7.0, 0.0],
              ],
              scale,
            ),
            centerX,
            centerY,
            agent.heading_deg,
          ),
        );
        const nose = formatPoints(
          rotatePoints(
            scalePoints(
              [
                [0.0, -9.5],
                [3.0, -5.5],
                [-3.0, -5.5],
              ],
              scale,
            ),
            centerX,
            centerY,
            agent.heading_deg,
          ),
        );
        const rotorOffsets = rotatePoints(
          scalePoints(
            [
              [-8.0, -8.0],
              [8.0, -8.0],
              [8.0, 8.0],
              [-8.0, 8.0],
            ],
            scale,
          ),
          centerX,
          centerY,
          agent.heading_deg,
        );
        const rotorCircles = rotorOffsets
          .map(
            ([x, y]) =>
              `<circle cx="${{x.toFixed(2)}}" cy="${{y.toFixed(2)}}" r="${{(3.4 * scale).toFixed(2)}}" fill="#ECFEFF" stroke="#0F766E" stroke-width="1.2" />`,
          )
          .join("");
        const labelX = centerX;
        const labelY = centerY - 18 * scale;
        return `
          <g aria-label="${{agent.agent_id}}">
            ${{rotorCircles}}
            <polygon points="${{frame}}" fill="#0F766E" stroke="#F0FDFA" stroke-width="1.4" />
            <polygon points="${{nose}}" fill="#99F6E4" />
            <circle cx="${{centerX.toFixed(2)}}" cy="${{centerY.toFixed(2)}}" r="${{(2.2 * scale).toFixed(2)}}" fill="#F0FDFA" />
            <g class="agent-label">
              <rect x="${{(labelX - 22).toFixed(2)}}" y="${{(labelY - 8).toFixed(2)}}" width="44" height="15" rx="7.5" ry="7.5"
                    fill="rgba(255,255,255,0.88)" stroke="rgba(15,23,42,0.10)" />
              <text x="${{labelX.toFixed(2)}}" y="${{(labelY + 3).toFixed(2)}}" text-anchor="middle"
                    font-size="8.8" font-weight="700" fill="#134E4A">${{agent.agent_id}}</text>
            </g>
          </g>
        `;
      }}

      function renderAgents(agents) {{
        return agents.map((agent) => (agent.kind === "USV" ? renderUSV(agent) : renderUAV(agent))).join("");
      }}

      function renderInterpolatedAgents(frameIndex, progress) {{
        const agents = interpolateAgents(frameIndex, progress);
        footprintLayer.innerHTML = renderFootprints(agents);
        agentLayer.innerHTML = renderAgents(agents);
      }}

      function renderCellLayer(cellIds, fill, stroke = "none", strokeWidth = 0) {{
        if (cellIds.length === 0) {{
          return "";
        }}
        const cellWidth = (gridCellSize / {replay.sea_map.width}) * {MAP_WIDTH};
        const cellHeight = (gridCellSize / {replay.sea_map.height}) * {MAP_HEIGHT};
        return cellIds
          .map((cellId) => {{
            const bounds = cellBounds(cellId);
            const [svgX, svgY] = mapPointToSvg(bounds.xMin, bounds.yMax);
            return `<rect x="${{svgX.toFixed(2)}}" y="${{svgY.toFixed(2)}}" width="${{cellWidth.toFixed(2)}}" height="${{cellHeight.toFixed(2)}}" fill="${{fill}}" stroke="${{stroke}}" stroke-width="${{strokeWidth}}" />`;
          }})
          .join("");
      }}

      function renderValidInfo(cellIds) {{
        return `<g aria-label="Valid Information Cells">${{renderCellLayer(cellIds, "rgba(56, 189, 248, 0.08)")}}</g>`;
      }}

      function renderStaleInfo(cellIds) {{
        return `<g aria-label="Stale Information Cells">${{renderCellLayer(cellIds, "rgba(249, 115, 22, 0.16)", "rgba(249, 115, 22, 0.18)", 0.4)}}</g>`;
      }}

      function renderCoveredCells(cellIds) {{
        return `<g aria-label="Covered Cells">${{renderCellLayer(cellIds, "rgba(234, 179, 8, 0.10)")}}</g>`;
      }}

      function renderCurrentObservedCells(cellIds) {{
        return `<g aria-label="Current Observed Cells">${{renderCellLayer(cellIds, "rgba(34, 197, 94, 0.18)", "rgba(22, 163, 74, 0.28)", 0.5)}}</g>`;
      }}

      function renderCoveredHistoryCells(cellIds, currentCellIds) {{
        const currentSet = new Set(currentCellIds);
        const historyCellIds = cellIds.filter((cellId) => !currentSet.has(cellId));
        return `<g aria-label="Covered History Cells">${{renderCellLayer(historyCellIds, "rgba(59, 130, 246, 0.08)", "rgba(59, 130, 246, 0.12)", 0.4)}}</g>`;
      }}

      function renderBaselineTasks(cellIds) {{
        return `<g aria-label="Baseline Task Layer">${{cellIds
          .map((cellId) => {{
            const cell = cellCenter(cellId);
            const [svgX, svgY] = mapPointToSvg(cell.x, cell.y);
            return `
              <g aria-label="Baseline Task ${{cell.row}}-${{cell.col}}">
                <circle cx="${{svgX.toFixed(2)}}" cy="${{svgY.toFixed(2)}}" r="10.0" fill="rgba(71, 85, 105, 0.10)" />
                <circle cx="${{svgX.toFixed(2)}}" cy="${{svgY.toFixed(2)}}" r="5.2" fill="#475569" stroke="#F8FAFC" stroke-width="1.8" />
                <path d="M ${{(svgX - 8).toFixed(2)}} ${{svgY.toFixed(2)}} L ${{(svgX + 8).toFixed(2)}} ${{svgY.toFixed(2)}} M ${{svgX.toFixed(2)}} ${{(svgY - 8).toFixed(2)}} L ${{svgX.toFixed(2)}} ${{(svgY + 8).toFixed(2)}}"
                      stroke="#CBD5E1" stroke-width="1.8" stroke-linecap="round" />
              </g>
            `;
          }})
          .join("")}}</g>`;
      }}

      function renderHotspotMarker(hotspot, stroke, fill, label) {{
        const [svgX, svgY] = mapPointToSvg(hotspot.x, hotspot.y);
        const markerId = hotspot.hotspot_id === null ? "?" : String(hotspot.hotspot_id);
        return `
          <g aria-label="${{label}} Hotspot #${{markerId}} (${{hotspot.row}}-${{hotspot.col}})">
            <circle cx="${{svgX.toFixed(2)}}" cy="${{svgY.toFixed(2)}}" r="11.0" fill="${{fill}}" stroke="${{stroke}}" stroke-width="1.5" stroke-dasharray="4 3" />
            <polygon points="${{svgX.toFixed(2)}},${{(svgY - 8).toFixed(2)}} ${{(svgX + 8).toFixed(2)}},${{svgY.toFixed(2)}} ${{svgX.toFixed(2)}},${{(svgY + 8).toFixed(2)}} ${{(svgX - 8).toFixed(2)}},${{svgY.toFixed(2)}}"
                     fill="${{stroke}}" opacity="0.88" />
            <text x="${{svgX.toFixed(2)}}" y="${{(svgY + 3.6).toFixed(2)}}" fill="#F8FAFC" font-size="8.8" font-weight="700" text-anchor="middle">${{markerId}}</text>
          </g>
        `;
      }}

      function renderHotspotLayer(frame) {{
        const pendingMarkup = frame.pending_hotspots
          .map((hotspot) => renderHotspotMarker(hotspot, "#F59E0B", "rgba(245, 158, 11, 0.18)", "Pending"))
          .join("");
        const uavCheckedMarkup = frame.uav_checked_hotspots
          .map((hotspot) => renderHotspotMarker(hotspot, "#DC2626", "rgba(248, 113, 113, 0.18)", "UAV Checked"))
          .join("");
        return `<g aria-label="Hotspot Knowledge Layer">${{pendingMarkup}}${{uavCheckedMarkup}}</g>`;
      }}

      function renderPlannedPaths(step) {{
        return plannedPathData[step]
          .map((item) => {{
            const points = item.points;
            if (!points || points.length < 2) {{
              return "";
            }}
            const svgPoints = points
              .map(([x, y]) => {{
                const [svgX, svgY] = mapPointToSvg(x, y);
                return `${{svgX.toFixed(2)}},${{svgY.toFixed(2)}}`;
              }})
              .join(" ");
            const stroke = item.agent_id.startsWith("UAV") ? "#0F766E" : "#1D4ED8";
            return `<polyline points="${{svgPoints}}" fill="none" stroke="${{stroke}}" stroke-width="2.0" opacity="0.72" stroke-dasharray="8 6" stroke-linecap="round" stroke-linejoin="round" />`;
          }})
          .join("");
      }}

      function updateStaticFrame(step) {{
        if (lastStaticFrame === step) {{
          return;
        }}
        const renderData = frameRenderData[step];
        validInfoLayer.innerHTML = renderValidInfo(renderData.valid_cells);
        staleInfoLayer.innerHTML = renderStaleInfo(renderData.stale_cells);
        coveredLayer.innerHTML = renderCoveredCells(renderData.covered_cells);
        baselineLayer.innerHTML = renderBaselineTasks(renderData.baseline_cells);
        hotspotLayer.innerHTML = renderHotspotLayer(renderData);
        trajectoryLayer.innerHTML = renderPlannedPaths(step);
        lastStaticFrame = step;
      }}

      function setFrame(step) {{
        currentFrame = step;
        currentFrameProgress = 0;
        updateStaticFrame(step);
        renderInterpolatedAgents(step, 0);
        document.getElementById("timeline").value = String(step);
        document.getElementById("stepBadge").textContent = "Step " + String(step);

        const summary = frameSummaries[step];
        document.getElementById("coverageRatio").textContent = summary.coverage_ratio;
        document.getElementById("coveredCells").textContent = String(summary.covered_cells);
        document.getElementById("currentObservedCells").textContent = String(summary.current_observed_cells);
        document.getElementById("validCells").textContent = String(summary.valid_cells);
        document.getElementById("staleCells").textContent = String(summary.stale_cells);
        document.getElementById("activeHotspots").textContent = String(summary.active_hotspots);
        document.getElementById("uavCheckedMarks").textContent = String(summary.uav_checked_marks);
        document.getElementById("confirmedHotspots").textContent = String(summary.confirmed_hotspots);

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
        playbackStartTime = null;
        playbackStartFrame = currentFrame + currentFrameProgress;
        document.getElementById("playButton").textContent = "Pause";
        animationHandle = window.requestAnimationFrame(playbackTick);
      }}

      function stopPlayback() {{
        isPlaying = false;
        document.getElementById("playButton").textContent = "Play";
        if (animationHandle !== null) {{
          window.cancelAnimationFrame(animationHandle);
          animationHandle = null;
        }}
      }}

      function playbackTick(timestamp) {{
        if (!isPlaying) {{
          return;
        }}
        if (playbackStartTime === null) {{
          playbackStartTime = timestamp;
        }}
        const elapsed = timestamp - playbackStartTime;
        let playbackFrame = playbackStartFrame + elapsed / stepDurationMs;
        if (playbackFrame >= frameSummaries.length - 1) {{
          playbackFrame = 0;
          playbackStartFrame = 0;
          playbackStartTime = timestamp;
        }}

        const baseFrame = Math.floor(playbackFrame);
        const progress = playbackFrame - baseFrame;
        currentFrame = baseFrame;
        currentFrameProgress = progress;
        updateStaticFrame(baseFrame);
        renderInterpolatedAgents(baseFrame, progress);
        document.getElementById("timeline").value = String(baseFrame);
        document.getElementById("stepBadge").textContent = "Step " + String(baseFrame);

        const summary = frameSummaries[baseFrame];
        document.getElementById("coverageRatio").textContent = summary.coverage_ratio;
        document.getElementById("coveredCells").textContent = String(summary.covered_cells);
        document.getElementById("currentObservedCells").textContent = String(summary.current_observed_cells);
        document.getElementById("validCells").textContent = String(summary.valid_cells);
        document.getElementById("staleCells").textContent = String(summary.stale_cells);
        document.getElementById("activeHotspots").textContent = String(summary.active_hotspots);
        document.getElementById("uavCheckedMarks").textContent = String(summary.uav_checked_marks);
        document.getElementById("confirmedHotspots").textContent = String(summary.confirmed_hotspots);

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

        animationHandle = window.requestAnimationFrame(playbackTick);
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


def _build_replay_grid_map(sea_map, layout):
    return build_grid_map(sea_map, layout)


def _build_frame_payloads(replay: SimulationReplay, grid_map) -> str:
    payloads: list[dict[str, object]] = []
    for frame in replay.frames:
        payloads.append(
            {
                "valid_cells": _encode_cell_indices(frame.valid_cells, cols=grid_map.cols),
                "stale_cells": _encode_cell_indices(frame.stale_cells, cols=grid_map.cols),
                "covered_cells": _encode_cell_indices(frame.covered_cells, cols=grid_map.cols),
                "current_observed_cells": _encode_cell_indices(
                    frame.current_observed_cells,
                    cols=grid_map.cols,
                ),
                "baseline_cells": _encode_cell_indices(frame.baseline_cells, cols=grid_map.cols),
                "pending_hotspot_cells": _encode_cell_indices(
                    frame.pending_hotspot_cells, cols=grid_map.cols
                ),
                "uav_checked_cells": _encode_cell_indices(
                    frame.uav_checked_cells, cols=grid_map.cols
                ),
                "pending_hotspots": list(frame.pending_hotspots),
                "uav_checked_hotspots": list(frame.uav_checked_hotspots),
                "agents": [
                    {
                        "agent_id": agent.agent_id,
                        "kind": agent.kind,
                        "x": agent.x,
                        "y": agent.y,
                        "heading_deg": agent.heading_deg,
                    }
                    for agent in frame.agents
                ],
            }
        )
    return json.dumps(payloads, ensure_ascii=True)


def _build_agent_static_payloads(replay: SimulationReplay) -> str:
    payload = {
        agent.agent_id: {
            "coverage_radius": agent.coverage_radius,
            "detection_radius": agent.detection_radius,
        }
        for agent in replay.initial_agents
    }
    return json.dumps(payload, ensure_ascii=True)


def _build_planned_path_payloads(replay: SimulationReplay) -> str:
    payloads: list[list[dict[str, object]]] = []
    for frame in replay.frames:
        payloads.append(
            [
                {"agent_id": agent_id, "points": points}
                for agent_id, points in frame.planned_paths
                if len(points) >= 2
            ]
        )
    return json.dumps(payloads, ensure_ascii=True)


def _encode_cell_indices(indices: tuple[tuple[int, int], ...], *, cols: int) -> list[int]:
    return [row * cols + col for row, col in indices]


def _build_frame_summaries(replay: SimulationReplay) -> str:
    summary_items = []
    running_totals = {
        "uav_checked_marks": 0,
        "confirmed_hotspots": 0,
    }
    for frame, step_log in zip(replay.frames, replay.step_logs, strict=True):
        task_layer = step_log["task_layer"]
        running_totals["uav_checked_marks"] += sum(
            len(indices) for indices in task_layer["newly_uav_checked_by_agent"].values()
        )
        running_totals["confirmed_hotspots"] += sum(
            len(indices) for indices in task_layer["confirmed_by_agent"].values()
        )
        summary_items.append(
            {
                "step": frame.step,
                "coverage_ratio": f"{frame.coverage_ratio * 100:.1f}%",
                "covered_cells": len(frame.covered_cells),
                "current_observed_cells": len(frame.current_observed_cells),
                "valid_cells": len(frame.valid_cells),
                "stale_cells": len(frame.stale_cells),
                "active_hotspots": len(frame.pending_hotspot_cells) + len(frame.uav_checked_cells),
                "uav_checked_marks": running_totals["uav_checked_marks"],
                "confirmed_hotspots": running_totals["confirmed_hotspots"],
                "events": list(frame.events),
            }
        )
    return json.dumps(summary_items, ensure_ascii=True)


def _build_x_tick_values(sea_map) -> tuple[float, ...]:
    values = {0.0, sea_map.width}
    values.update(zone.x_end for zone in sea_map.zones[:-1])
    return tuple(sorted(values))


def _build_zone_boundary_lines(sea_map) -> tuple[str, ...]:
    return tuple(
        f"""
            <line x1="{MAP_LEFT + (zone.x_end / sea_map.width) * MAP_WIDTH}" y1="{MAP_TOP}" x2="{MAP_LEFT + (zone.x_end / sea_map.width) * MAP_WIDTH}" y2="{MAP_TOP + MAP_HEIGHT}"
                  stroke="rgba(148, 163, 184, 0.72)" stroke-width="1.1" stroke-dasharray="6 8" />
        """
        for zone in sea_map.zones[:-1]
    )


def _format_distance_label(value: float) -> str:
    rounded = round(value)
    if abs(value - rounded) < 1e-6:
        return str(int(rounded))
    return f"{value:.1f}"
