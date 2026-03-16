"""Environment definitions for the three-zone marine map."""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Literal

MAX_OBSTACLE_LAYOUT_ATTEMPTS = 64


@dataclass(frozen=True)
class SeaZone:
    """A continuous zone spanning a horizontal interval of the sea area."""

    name: str
    x_start: float
    x_end: float
    color: str

    @property
    def width(self) -> float:
        return self.x_end - self.x_start


@dataclass(frozen=True)
class SeaMap:
    """A left-to-right three-zone sea area."""

    width: float
    height: float
    nearshore: SeaZone
    risk: SeaZone
    offshore: SeaZone

    @property
    def zones(self) -> tuple[SeaZone, SeaZone, SeaZone]:
        return (self.nearshore, self.risk, self.offshore)


@dataclass(frozen=True)
class TraversableCorridor:
    """An internal corridor constraint used to keep the risk zone passable."""

    name: str
    control_points: tuple[tuple[float, float], ...]
    width: float

    @property
    def height(self) -> float:
        return self.width

    @property
    def y_min(self) -> float:
        return min(point[1] for point in self.control_points) - self.width / 2

    @property
    def y_max(self) -> float:
        return max(point[1] for point in self.control_points) + self.width / 2


@dataclass(frozen=True)
class PolygonObstacle:
    """A polygon obstacle representing reefs, shoals or hard barriers."""

    name: str
    zone_name: str
    points: tuple[tuple[float, float], ...]

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        xs = [point[0] for point in self.points]
        ys = [point[1] for point in self.points]
        return (min(xs), min(ys), max(xs), max(ys))


@dataclass(frozen=True)
class CircularFeature:
    """An isolated offshore obstacle or risk area."""

    name: str
    feature_type: Literal["islet", "risk_area"]
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class MonitoringTarget:
    """A nearshore baseline point or an offshore task hotspot."""

    name: str
    target_type: Literal["baseline_point", "task_hotspot"]
    zone_name: str
    x: float
    y: float


@dataclass(frozen=True)
class ObstacleLayout:
    """Generated obstacle layout for a single simulation run."""

    seed: int
    traversable_corridors: tuple[TraversableCorridor, ...]
    risk_zone_obstacles: tuple[PolygonObstacle, ...]
    offshore_features: tuple[CircularFeature, ...]
    nearshore_monitor_points: tuple[MonitoringTarget, ...]
    offshore_hotspots: tuple[MonitoringTarget, ...]
    generation_attempts: int = 1


def build_default_sea_map() -> SeaMap:
    """Build the agreed first-version sea map."""

    sea_map = SeaMap(
        width=1000.0,
        height=1000.0,
        nearshore=SeaZone(
            name="Nearshore Zone",
            x_start=0.0,
            x_end=250.0,
            color="#4C78A8",
        ),
        risk=SeaZone(
            name="Middle Risk Zone",
            x_start=250.0,
            x_end=450.0,
            color="#F58518",
        ),
        offshore=SeaZone(
            name="Offshore Zone",
            x_start=450.0,
            x_end=1000.0,
            color="#54A24B",
        ),
    )
    validate_sea_map(sea_map)
    return sea_map


def build_obstacle_layout(
    sea_map: SeaMap,
    seed: int | None = None,
) -> ObstacleLayout:
    """Build a legal obstacle layout for a simulation run.

    The first attempt keeps the historical single-sample behavior for a given
    seed. If that candidate fails validation, the generator deterministically
    resamples follow-up candidates derived from the same external seed until a
    legal layout is found.
    """

    actual_seed = seed if seed is not None else random.SystemRandom().randrange(1, 1_000_000_000)
    last_error: ValueError | None = None

    for attempt_index in range(MAX_OBSTACLE_LAYOUT_ATTEMPTS):
        candidate_seed = actual_seed + attempt_index
        candidate_layout = _build_obstacle_layout_candidate(
            sea_map=sea_map,
            actual_seed=actual_seed,
            candidate_seed=candidate_seed,
            generation_attempts=attempt_index + 1,
        )
        try:
            validate_obstacle_layout(sea_map, candidate_layout)
        except ValueError as error:
            last_error = error
            continue
        return candidate_layout

    raise ValueError(
        "Failed to construct a legal obstacle layout after "
        f"{MAX_OBSTACLE_LAYOUT_ATTEMPTS} deterministic attempts for seed {actual_seed}: "
        f"{last_error}"
    )


def _build_obstacle_layout_candidate(
    *,
    sea_map: SeaMap,
    actual_seed: int,
    candidate_seed: int,
    generation_attempts: int,
) -> ObstacleLayout:
    """Build one deterministic obstacle-layout candidate from a candidate seed."""

    generator = random.Random(candidate_seed)

    traversable_corridors = (
        TraversableCorridor(
            name="Upper Traversable Corridor",
            control_points=(
                (sea_map.risk.x_start, 272.0 + generator.uniform(-10.0, 10.0)),
                (292.0, 320.0 + generator.uniform(-10.0, 8.0)),
                (338.0, 248.0 + generator.uniform(-10.0, 10.0)),
                (392.0, 304.0 + generator.uniform(-8.0, 10.0)),
                (sea_map.risk.x_end, 278.0 + generator.uniform(-10.0, 10.0)),
            ),
            width=64.0 + generator.uniform(-4.0, 4.0),
        ),
        TraversableCorridor(
            name="Lower Traversable Corridor",
            control_points=(
                (sea_map.risk.x_start, 730.0 + generator.uniform(-10.0, 10.0)),
                (302.0, 658.0 + generator.uniform(-10.0, 10.0)),
                (354.0, 742.0 + generator.uniform(-10.0, 8.0)),
                (404.0, 676.0 + generator.uniform(-10.0, 12.0)),
                (sea_map.risk.x_end, 722.0 + generator.uniform(-10.0, 10.0)),
            ),
            width=68.0 + generator.uniform(-4.0, 4.0),
        ),
    )

    risk_zone_obstacles = (
        _build_top_risk_obstacle(generator, sea_map, traversable_corridors[0]),
        _build_middle_risk_obstacle(
            generator,
            name="Risk Obstacle 2",
            sea_map=sea_map,
            upper_corridor=traversable_corridors[0],
            lower_corridor=traversable_corridors[1],
            x_start=272.0,
            x_end=336.0,
            top_margin=8.0,
            bottom_margin=10.0,
        ),
        _build_middle_risk_obstacle(
            generator,
            name="Risk Obstacle 3",
            sea_map=sea_map,
            upper_corridor=traversable_corridors[0],
            lower_corridor=traversable_corridors[1],
            x_start=352.0,
            x_end=430.0,
            top_margin=10.0,
            bottom_margin=8.0,
        ),
        _build_bottom_risk_obstacle(generator, sea_map, traversable_corridors[1]),
    )

    offshore_features = _build_offshore_features(generator, sea_map)
    return ObstacleLayout(
        seed=actual_seed,
        traversable_corridors=traversable_corridors,
        risk_zone_obstacles=risk_zone_obstacles,
        offshore_features=offshore_features,
        nearshore_monitor_points=(),
        offshore_hotspots=(),
        generation_attempts=generation_attempts,
    )


def validate_sea_map(sea_map: SeaMap) -> None:
    """Validate that the three zones partition the sea map continuously."""

    nearshore, risk, offshore = sea_map.zones

    if sea_map.width <= 0 or sea_map.height <= 0:
        raise ValueError("Sea map width and height must be positive.")

    if nearshore.x_start != 0.0:
        raise ValueError("Nearshore zone must start at x = 0.")

    if nearshore.x_end != risk.x_start:
        raise ValueError("Nearshore and risk zones must be continuous.")

    if risk.x_end != offshore.x_start:
        raise ValueError("Risk and offshore zones must be continuous.")

    if offshore.x_end != sea_map.width:
        raise ValueError("Offshore zone must end at the map width.")

    for zone in sea_map.zones:
        if zone.width <= 0:
            raise ValueError(f"{zone.name} must have positive width.")


def validate_obstacle_layout(sea_map: SeaMap, layout: ObstacleLayout) -> None:
    """Validate obstacle counts and their spatial constraints."""

    if len(layout.traversable_corridors) != 2:
        raise ValueError("The risk zone must guarantee exactly two traversable corridors.")

    if len(layout.risk_zone_obstacles) != 4:
        raise ValueError("The risk zone must contain exactly four irregular obstacles.")
    if len(layout.offshore_features) != 5:
        raise ValueError("The offshore zone must contain three islets and two risk areas.")
    if layout.nearshore_monitor_points:
        raise ValueError("Initial baseline monitoring points must be empty.")
    if layout.offshore_hotspots:
        raise ValueError("Initial offshore hotspots must be empty.")

    for corridor in layout.traversable_corridors:
        if corridor.height <= 0:
            raise ValueError(f"{corridor.name} must have positive height.")
        if len(corridor.control_points) < 2:
            raise ValueError(f"{corridor.name} must define at least two control points.")

    obstacle_bounds: list[tuple[float, float, float, float]] = []
    for obstacle in layout.risk_zone_obstacles:
        x_min, y_min, x_max, y_max = obstacle.bounds
        if x_min < sea_map.risk.x_start or x_max > sea_map.risk.x_end:
            raise ValueError(f"{obstacle.name} must stay inside the middle risk zone.")
        if not (0.0 <= y_min < y_max <= sea_map.height):
            raise ValueError(f"{obstacle.name} must stay inside the sea map height.")
        for existing_bounds in obstacle_bounds:
            if _boxes_overlap((x_min, y_min, x_max, y_max), existing_bounds):
                raise ValueError(f"{obstacle.name} cannot overlap another risk-zone obstacle.")
        obstacle_bounds.append((x_min, y_min, x_max, y_max))

    for corridor in layout.traversable_corridors:
        _validate_corridor_path(sea_map, corridor, layout.risk_zone_obstacles)

    feature_circles: list[tuple[float, float, float]] = []
    feature_type_counts: dict[str, int] = {"islet": 0, "risk_area": 0}
    for feature in layout.offshore_features:
        if feature.x <= sea_map.offshore.x_start or feature.x >= sea_map.offshore.x_end:
            raise ValueError(f"{feature.name} must stay inside the offshore zone.")
        if feature.y - feature.radius < 0.0 or feature.y + feature.radius > sea_map.height:
            raise ValueError(f"{feature.name} must stay inside the sea map height.")
        for existing_circle in feature_circles:
            if _circles_overlap((feature.x, feature.y, feature.radius), existing_circle):
                raise ValueError(f"{feature.name} cannot overlap another offshore feature.")
        feature_circles.append((feature.x, feature.y, feature.radius))
        feature_type_counts[feature.feature_type] += 1

    if feature_type_counts["islet"] != 3 or feature_type_counts["risk_area"] != 2:
        raise ValueError("The offshore zone must contain three islets and two risk areas.")

    for target in layout.nearshore_monitor_points:
        if target.zone_name != sea_map.nearshore.name:
            raise ValueError(f"{target.name} must belong to the nearshore zone.")
        if not (sea_map.nearshore.x_start < target.x < sea_map.nearshore.x_end):
            raise ValueError(f"{target.name} must stay inside the nearshore zone.")
        if not (0.0 < target.y < sea_map.height):
            raise ValueError(f"{target.name} must stay inside the sea map height.")

    baseline_positions: list[tuple[float, float]] = []
    for target in layout.nearshore_monitor_points:
        for existing_position in baseline_positions:
            if _point_distance((target.x, target.y), existing_position) < 120.0:
                raise ValueError(
                    f"{target.name} is too close to another baseline monitoring point."
                )
        baseline_positions.append((target.x, target.y))

    hotspot_positions: list[tuple[float, float]] = []
    for target in layout.offshore_hotspots:
        if target.zone_name != sea_map.offshore.name:
            raise ValueError(f"{target.name} must belong to the offshore zone.")
        if not (sea_map.offshore.x_start < target.x < sea_map.offshore.x_end):
            raise ValueError(f"{target.name} must stay inside the offshore zone.")
        if not (0.0 < target.y < sea_map.height):
            raise ValueError(f"{target.name} must stay inside the sea map height.")
        for feature in layout.offshore_features:
            if _point_within_feature((target.x, target.y), feature, padding=24.0):
                raise ValueError(f"{target.name} must avoid offshore obstacles and risk areas.")
        for existing_position in hotspot_positions:
            if _point_distance((target.x, target.y), existing_position) < 95.0:
                raise ValueError(f"{target.name} is too close to another offshore hotspot.")
        hotspot_positions.append((target.x, target.y))


def _build_top_risk_obstacle(
    generator: random.Random,
    sea_map: SeaMap,
    corridor: TraversableCorridor,
) -> PolygonObstacle:
    x_values = _risk_zone_x_values(sea_map.risk, point_count=6)
    lower_edge = tuple(
        (
            x,
            _corridor_y(corridor, x) - corridor.width / 2 - generator.uniform(4.0, 8.0),
        )
        for x in x_values
    )
    points = (
        (sea_map.risk.x_start, 0.0),
        (sea_map.risk.x_end, 0.0),
        *reversed(lower_edge),
    )
    return PolygonObstacle(name="Risk Obstacle 1", zone_name=sea_map.risk.name, points=points)


def _build_middle_risk_obstacle(
    generator: random.Random,
    name: str,
    sea_map: SeaMap,
    upper_corridor: TraversableCorridor,
    lower_corridor: TraversableCorridor,
    x_start: float,
    x_end: float,
    top_margin: float,
    bottom_margin: float,
) -> PolygonObstacle:
    x_values = _segment_x_values(x_start, x_end, point_count=5)
    top_edge = tuple(
        (
            x,
            _corridor_y(upper_corridor, x)
            + upper_corridor.width / 2
            + top_margin
            + generator.uniform(-3.0, 3.0),
        )
        for x in x_values
    )
    bottom_edge = tuple(
        (
            x,
            _corridor_y(lower_corridor, x)
            - lower_corridor.width / 2
            - bottom_margin
            + generator.uniform(-3.0, 3.0),
        )
        for x in reversed(x_values)
    )
    points = top_edge + bottom_edge
    return PolygonObstacle(name=name, zone_name=sea_map.risk.name, points=points)


def _build_bottom_risk_obstacle(
    generator: random.Random,
    sea_map: SeaMap,
    corridor: TraversableCorridor,
) -> PolygonObstacle:
    x_values = _risk_zone_x_values(sea_map.risk, point_count=6)
    upper_edge = tuple(
        (
            x,
            _corridor_y(corridor, x) + corridor.width / 2 + generator.uniform(18.0, 24.0),
        )
        for x in x_values
    )
    points = (
        *upper_edge,
        (sea_map.risk.x_end, sea_map.height),
        (sea_map.risk.x_start, sea_map.height),
    )
    return PolygonObstacle(name="Risk Obstacle 4", zone_name=sea_map.risk.name, points=points)


def _build_offshore_features(
    generator: random.Random,
    sea_map: SeaMap,
) -> tuple[CircularFeature, ...]:
    feature_specs = (
        ("Offshore Islet 1", "islet", (530.0, 640.0), (170.0, 320.0), (20.0, 28.0)),
        ("Offshore Islet 2", "islet", (660.0, 800.0), (600.0, 760.0), (22.0, 32.0)),
        ("Offshore Islet 3", "islet", (820.0, 930.0), (220.0, 360.0), (18.0, 26.0)),
        ("Offshore Risk Area 1", "risk_area", (610.0, 760.0), (410.0, 560.0), (40.0, 58.0)),
        ("Offshore Risk Area 2", "risk_area", (820.0, 940.0), (700.0, 860.0), (36.0, 52.0)),
    )
    features: list[CircularFeature] = []
    for name, feature_type, x_range, y_range, radius_range in feature_specs:
        features.append(
            _sample_offshore_feature(
                generator=generator,
                name=name,
                feature_type=feature_type,
                x_range=x_range,
                y_range=y_range,
                radius_range=radius_range,
                existing_features=tuple(features),
                sea_map=sea_map,
            )
        )
    return tuple(features)


def _build_nearshore_monitor_points(
    generator: random.Random,
    sea_map: SeaMap,
) -> tuple[MonitoringTarget, ...]:
    points: list[MonitoringTarget] = []
    for index in range(2):
        x, y = _sample_monitoring_location(
            generator=generator,
            zone=sea_map.nearshore,
            sea_height=sea_map.height,
            forbidden_features=(),
            existing_targets=tuple(points),
            min_spacing=120.0,
            x_margin=42.0,
            y_margin=120.0,
        )
        points.append(
            MonitoringTarget(
                name=f"Nearshore Baseline Point {index + 1}",
                target_type="baseline_point",
                zone_name=sea_map.nearshore.name,
                x=x,
                y=y,
            )
        )
    return tuple(points)


def _build_offshore_hotspots(
    generator: random.Random,
    sea_map: SeaMap,
    offshore_features: tuple[CircularFeature, ...],
) -> tuple[MonitoringTarget, ...]:
    hotspots: list[MonitoringTarget] = []
    for index in range(5):
        x, y = _sample_monitoring_location(
            generator=generator,
            zone=sea_map.offshore,
            sea_height=sea_map.height,
            forbidden_features=offshore_features,
            existing_targets=tuple(hotspots),
            min_spacing=95.0,
            x_margin=48.0,
            y_margin=88.0,
        )
        hotspots.append(
            MonitoringTarget(
                name=f"Offshore Hotspot {index + 1}",
                target_type="task_hotspot",
                zone_name=sea_map.offshore.name,
                x=x,
                y=y,
            )
        )
    return tuple(hotspots)


def _sample_offshore_feature(
    *,
    generator: random.Random,
    name: str,
    feature_type: Literal["islet", "risk_area"],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    radius_range: tuple[float, float],
    existing_features: tuple[CircularFeature, ...],
    sea_map: SeaMap,
) -> CircularFeature:
    for _ in range(200):
        radius = generator.uniform(*radius_range)
        x = generator.uniform(
            max(sea_map.offshore.x_start + radius + 10.0, x_range[0]),
            x_range[1],
        )
        y = generator.uniform(
            max(radius + 10.0, y_range[0]),
            min(sea_map.height - radius - 10.0, y_range[1]),
        )
        candidate = CircularFeature(
            name=name,
            feature_type=feature_type,
            x=x,
            y=y,
            radius=radius,
        )
        if any(
            _circles_overlap(
                (candidate.x, candidate.y, candidate.radius),
                (feature.x, feature.y, feature.radius),
            )
            for feature in existing_features
        ):
            continue
        return candidate
    raise ValueError(f"Failed to sample {name} without overlapping another offshore feature.")


def _sample_monitoring_location(
    *,
    generator: random.Random,
    zone: SeaZone,
    sea_height: float,
    forbidden_features: tuple[CircularFeature, ...],
    existing_targets: tuple[MonitoringTarget, ...],
    min_spacing: float,
    x_margin: float,
    y_margin: float,
) -> tuple[float, float]:
    for _ in range(300):
        x = generator.uniform(zone.x_start + x_margin, zone.x_end - x_margin)
        y = generator.uniform(y_margin, sea_height - y_margin)
        if any(
            _point_within_feature((x, y), feature, padding=24.0) for feature in forbidden_features
        ):
            continue
        if any(
            _point_distance((x, y), (target.x, target.y)) < min_spacing
            for target in existing_targets
        ):
            continue
        return (x, y)
    raise ValueError(f"Failed to sample a monitoring location inside {zone.name}.")


def _validate_corridor_path(
    sea_map: SeaMap,
    corridor: TraversableCorridor,
    obstacles: tuple[PolygonObstacle, ...],
) -> None:
    sample_x_values = _risk_zone_x_values(sea_map.risk, point_count=21)
    inner_offsets = (-corridor.width * 0.35, 0.0, corridor.width * 0.35)

    for x in sample_x_values:
        center_y = _corridor_y(corridor, x)
        for offset in inner_offsets:
            sample_y = center_y + offset
            if not (0.0 <= sample_y <= sea_map.height):
                raise ValueError(f"{corridor.name} must stay inside the sea map height.")
            if any(_point_in_polygon((x, sample_y), obstacle.points) for obstacle in obstacles):
                raise ValueError(f"{corridor.name} must remain passable through the risk zone.")


def _risk_zone_x_values(zone: SeaZone, point_count: int) -> tuple[float, ...]:
    return _segment_x_values(zone.x_start, zone.x_end, point_count)


def _segment_x_values(x_start: float, x_end: float, point_count: int) -> tuple[float, ...]:
    step = (x_end - x_start) / (point_count - 1)
    return tuple(x_start + index * step for index in range(point_count))


def _corridor_y(corridor: TraversableCorridor, x: float) -> float:
    points = corridor.control_points
    for start, end in zip(points, points[1:], strict=False):
        start_x, start_y = start
        end_x, end_y = end
        if start_x <= x <= end_x:
            ratio = 0.0 if end_x == start_x else (x - start_x) / (end_x - start_x)
            return start_y + ratio * (end_y - start_y)
    if x < points[0][0]:
        return points[0][1]
    return points[-1][1]


def _point_in_polygon(point: tuple[float, float], polygon: tuple[tuple[float, float], ...]) -> bool:
    x, y = point
    is_inside = False
    for (x1, y1), (x2, y2) in zip(
        polygon,
        polygon[1:] + polygon[:1],
        strict=False,
    ):
        intersects = (y1 > y) != (y2 > y)
        if not intersects:
            continue
        intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        if x < intersection_x:
            is_inside = not is_inside
    return is_inside


def _point_within_feature(
    point: tuple[float, float],
    feature: CircularFeature,
    padding: float = 0.0,
) -> bool:
    return _point_distance(point, (feature.x, feature.y)) <= feature.radius + padding


def _point_distance(point_a: tuple[float, float], point_b: tuple[float, float]) -> float:
    return ((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2) ** 0.5


def _circles_overlap(
    circle_a: tuple[float, float, float],
    circle_b: tuple[float, float, float],
) -> bool:
    ax, ay, ar = circle_a
    bx, by, br = circle_b
    return _point_distance((ax, ay), (bx, by)) < ar + br + 10.0


def _boxes_overlap(
    box_a: tuple[float, float, float, float],
    box_b: tuple[float, float, float, float],
) -> bool:
    ax_min, ay_min, ax_max, ay_max = box_a
    bx_min, by_min, bx_max, by_max = box_b
    return ax_min < bx_max and ax_max > bx_min and ay_min < by_max and ay_max > by_min
