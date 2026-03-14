"""Grid-network projection for the continuous sea environment."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from usv_uav_marine_coverage.agent_model import AgentState, can_cover_point

from usv_uav_marine_coverage.environment import CircularFeature, ObstacleLayout, PolygonObstacle, SeaMap

DEFAULT_CELL_SIZE = 25.0


@dataclass(frozen=True)
class GridCell:
    """A single rectangular cell in the discrete sea grid."""

    row: int
    col: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    center_x: float
    center_y: float
    zone_name: str
    has_obstacle: bool
    has_risk_area: bool
    has_baseline_point: bool
    has_task_hotspot: bool


@dataclass(frozen=True)
class GridMap:
    """A rule-based rectangular grid projected from the continuous sea map."""

    width: float
    height: float
    cell_size: float
    rows: int
    cols: int
    cells: tuple[tuple[GridCell, ...], ...]

    @property
    def total_cells(self) -> int:
        return self.rows * self.cols

    @property
    def flat_cells(self) -> tuple[GridCell, ...]:
        return tuple(cell for row in self.cells for cell in row)

    def cell_at(self, row: int, col: int) -> GridCell:
        return self.cells[row][col]

    def locate_cell(self, x: float, y: float) -> GridCell:
        row, col = locate_grid_index(
            x=x,
            y=y,
            width=self.width,
            height=self.height,
            cell_size=self.cell_size,
        )
        return self.cell_at(row, col)


class CoverageState(str, Enum):
    """Basic coverage states for the first mapping stage."""

    UNCOVERED = "uncovered"
    COVERED = "covered"


@dataclass
class GridCoverageCell:
    """Dynamic coverage state attached to a static grid cell."""

    coverage_count: int = 0
    covered_by_usv: int = 0
    covered_by_uav: int = 0
    last_covered_step: int | None = None
    coverage_state: CoverageState = CoverageState.UNCOVERED


@dataclass
class GridCoverageMap:
    """Dynamic coverage layer that can be reset independently from the environment grid."""

    grid_map: GridMap
    states: list[list[GridCoverageCell]]

    @property
    def total_coverable_cells(self) -> int:
        return sum(1 for cell in self.grid_map.flat_cells if not cell.has_obstacle)

    @property
    def covered_cells(self) -> int:
        covered = 0
        for row_index, row in enumerate(self.states):
            for col_index, state in enumerate(row):
                if self.grid_map.cell_at(row_index, col_index).has_obstacle:
                    continue
                if state.coverage_state == CoverageState.COVERED:
                    covered += 1
        return covered

    def state_at(self, row: int, col: int) -> GridCoverageCell:
        return self.states[row][col]

    def covered_ratio(self) -> float:
        if self.total_coverable_cells == 0:
            return 0.0
        return self.covered_cells / self.total_coverable_cells


def build_grid_map(
    sea_map: SeaMap,
    obstacle_layout: ObstacleLayout,
    cell_size: float = DEFAULT_CELL_SIZE,
) -> GridMap:
    """Project the continuous sea environment onto a rectangular grid."""

    if cell_size <= 0:
        raise ValueError("Grid cell size must be positive.")
    if sea_map.width % cell_size != 0 or sea_map.height % cell_size != 0:
        raise ValueError("Grid cell size must divide the sea map width and height exactly.")

    rows = int(sea_map.height / cell_size)
    cols = int(sea_map.width / cell_size)

    baseline_indices = {
        locate_grid_index(
            x=target.x,
            y=target.y,
            width=sea_map.width,
            height=sea_map.height,
            cell_size=cell_size,
        )
        for target in obstacle_layout.nearshore_monitor_points
    }
    hotspot_indices = {
        locate_grid_index(
            x=target.x,
            y=target.y,
            width=sea_map.width,
            height=sea_map.height,
            cell_size=cell_size,
        )
        for target in obstacle_layout.offshore_hotspots
    }

    grid_rows: list[tuple[GridCell, ...]] = []
    for row in range(rows):
        row_cells: list[GridCell] = []
        y_min = row * cell_size
        y_max = y_min + cell_size
        center_y = y_min + cell_size / 2

        for col in range(cols):
            x_min = col * cell_size
            x_max = x_min + cell_size
            center_x = x_min + cell_size / 2
            zone_name = _zone_name_for_x(sea_map, center_x)
            has_obstacle = _point_hits_polygon_obstacle(
                point=(center_x, center_y),
                obstacles=obstacle_layout.risk_zone_obstacles,
            ) or _point_hits_feature_type(
                point=(center_x, center_y),
                features=obstacle_layout.offshore_features,
                feature_type="islet",
            )
            has_risk_area = _point_hits_feature_type(
                point=(center_x, center_y),
                features=obstacle_layout.offshore_features,
                feature_type="risk_area",
            )
            row_cells.append(
                GridCell(
                    row=row,
                    col=col,
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max,
                    center_x=center_x,
                    center_y=center_y,
                    zone_name=zone_name,
                    has_obstacle=has_obstacle,
                    has_risk_area=has_risk_area,
                    has_baseline_point=(row, col) in baseline_indices,
                    has_task_hotspot=(row, col) in hotspot_indices,
                )
            )
        grid_rows.append(tuple(row_cells))

    return GridMap(
        width=sea_map.width,
        height=sea_map.height,
        cell_size=cell_size,
        rows=rows,
        cols=cols,
        cells=tuple(grid_rows),
    )


def build_grid_coverage_map(grid_map: GridMap) -> GridCoverageMap:
    """Create an empty dynamic coverage layer for a static grid map."""

    states = [
        [GridCoverageCell() for _ in range(grid_map.cols)]
        for _ in range(grid_map.rows)
    ]
    return GridCoverageMap(grid_map=grid_map, states=states)


def apply_agent_coverage(
    coverage_map: GridCoverageMap,
    agent: AgentState,
    step: int,
) -> tuple[tuple[int, int], ...]:
    """Project one agent's circular footprint onto the grid using center-point coverage."""

    if step < 0:
        raise ValueError("Simulation step must be non-negative.")

    grid_map = coverage_map.grid_map
    row_min, row_max, col_min, col_max = _candidate_index_bounds(
        grid_map=grid_map,
        x=agent.x,
        y=agent.y,
        radius=agent.coverage_radius,
    )
    covered_indices: list[tuple[int, int]] = []

    for row in range(row_min, row_max + 1):
        for col in range(col_min, col_max + 1):
            cell = grid_map.cell_at(row, col)
            if cell.has_obstacle:
                continue
            if not can_cover_point(agent, cell.center_x, cell.center_y):
                continue
            state = coverage_map.state_at(row, col)
            state.coverage_count += 1
            state.last_covered_step = step
            state.coverage_state = CoverageState.COVERED
            if agent.kind == "USV":
                state.covered_by_usv += 1
            elif agent.kind == "UAV":
                state.covered_by_uav += 1
            covered_indices.append((row, col))

    return tuple(covered_indices)


def locate_grid_index(
    *,
    x: float,
    y: float,
    width: float,
    height: float,
    cell_size: float,
) -> tuple[int, int]:
    """Map a continuous point to its grid row and column."""

    if not (0.0 <= x <= width and 0.0 <= y <= height):
        raise ValueError("Point must stay inside the sea map bounds.")

    clamped_x = min(x, width - 1e-9)
    clamped_y = min(y, height - 1e-9)
    col = int(clamped_x // cell_size)
    row = int(clamped_y // cell_size)
    return (row, col)


def _candidate_index_bounds(
    *,
    grid_map: GridMap,
    x: float,
    y: float,
    radius: float,
) -> tuple[int, int, int, int]:
    x_min = max(0.0, x - radius)
    x_max = min(grid_map.width, x + radius)
    y_min = max(0.0, y - radius)
    y_max = min(grid_map.height, y + radius)
    row_min, col_min = locate_grid_index(
        x=x_min,
        y=y_min,
        width=grid_map.width,
        height=grid_map.height,
        cell_size=grid_map.cell_size,
    )
    row_max, col_max = locate_grid_index(
        x=x_max,
        y=y_max,
        width=grid_map.width,
        height=grid_map.height,
        cell_size=grid_map.cell_size,
    )
    return (row_min, row_max, col_min, col_max)


def _zone_name_for_x(sea_map: SeaMap, x: float) -> str:
    if x < sea_map.nearshore.x_end:
        return sea_map.nearshore.name
    if x < sea_map.risk.x_end:
        return sea_map.risk.name
    return sea_map.offshore.name


def _point_hits_polygon_obstacle(
    point: tuple[float, float],
    obstacles: tuple[PolygonObstacle, ...],
) -> bool:
    return any(_point_in_polygon(point, obstacle.points) for obstacle in obstacles)


def _point_hits_feature_type(
    point: tuple[float, float],
    features: tuple[CircularFeature, ...],
    feature_type: str,
) -> bool:
    return any(
        feature.feature_type == feature_type and _point_in_circle(point, feature)
        for feature in features
    )


def _point_in_circle(point: tuple[float, float], feature: CircularFeature) -> bool:
    dx = point[0] - feature.x
    dy = point[1] - feature.y
    return dx * dx + dy * dy <= feature.radius * feature.radius


def _point_in_polygon(
    point: tuple[float, float],
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    x, y = point
    is_inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        intersects = (y1 > y) != (y2 > y)
        if not intersects:
            continue
        intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        if x < intersection_x:
            is_inside = not is_inside
    return is_inside
