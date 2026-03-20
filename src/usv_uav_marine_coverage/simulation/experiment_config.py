"""Experiment configuration for reproducible simulation runs."""

from __future__ import annotations

import tomllib
from dataclasses import asdict, dataclass, fields
from pathlib import Path

from usv_uav_marine_coverage.information_map import InformationMapConfig
from usv_uav_marine_coverage.planning.usv_path_planner import SUPPORTED_USV_PATH_PLANNERS

from .scenario_catalog import get_scenario_preset, list_scenario_names


@dataclass(frozen=True)
class SimulationConfig:
    """Runtime settings for one simulation run."""

    seed: int | None = None
    steps: int = 40
    dt_seconds: float = 1.0


@dataclass(frozen=True)
class AlgorithmSelection:
    """Named baseline algorithms selected for one experiment."""

    task_allocator: str = "basic_task_allocator"
    zone_partition_policy: str = "baseline_fixed_partition"
    distributed_sync_interval_steps: int = 1
    distributed_broadcast_range_m: float = 0.0
    distributed_winner_memory_ttl_steps: int = 0
    distributed_bundle_length: int = 1
    usv_path_planner: str = "astar_path_planner"
    uav_search_planner: str = "uav_lawnmower_planner"
    execution_policy: str = "phase_one_execution"


@dataclass(frozen=True)
class ScenarioSelection:
    """Named reusable scenario preset selected for one experiment."""

    name: str = "baseline_patrol"


@dataclass(frozen=True)
class ExperimentConfig:
    """Top-level experiment configuration."""

    simulation: SimulationConfig
    scenario: ScenarioSelection
    algorithms: AlgorithmSelection
    information_map: InformationMapConfig


def build_default_experiment_config(
    *,
    scenario_name: str | None = None,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> ExperimentConfig:
    """Return the current baseline experiment configuration."""

    default_scenario_name = ScenarioSelection().name
    scenario = ScenarioSelection(
        name=default_scenario_name if scenario_name is None else scenario_name
    )
    preset = get_scenario_preset(scenario.name)
    return ExperimentConfig(
        simulation=SimulationConfig(seed=seed, steps=steps, dt_seconds=dt_seconds),
        scenario=scenario,
        algorithms=AlgorithmSelection(),
        information_map=preset.information_map,
    )


def load_experiment_config(
    path: Path,
    *,
    scenario_override: str | None = None,
    seed_override: int | None = None,
    steps_override: int | None = None,
    dt_override: float | None = None,
) -> ExperimentConfig:
    """Load one experiment configuration from TOML."""

    raw = tomllib.loads(path.read_text(encoding="utf-8"))
    simulation_raw = _expect_table(raw, "simulation")
    scenario_name = _read_scenario_name(raw.get("scenario"))
    if scenario_override is not None:
        scenario_name = scenario_override
    scenario = ScenarioSelection(name=scenario_name)
    preset = get_scenario_preset(scenario_name)
    algorithms_raw = _expect_table(raw, "algorithms")
    info_map_raw = _read_optional_table(raw, "information_map")

    config = ExperimentConfig(
        simulation=SimulationConfig(
            seed=_read_optional_int(simulation_raw, "seed"),
            steps=_read_int(simulation_raw, "steps", minimum=1),
            dt_seconds=_read_float(simulation_raw, "dt_seconds", minimum=0.0, strict_minimum=True),
        ),
        scenario=scenario,
        algorithms=AlgorithmSelection(
            task_allocator=_read_str(algorithms_raw, "task_allocator"),
            zone_partition_policy=_read_optional_str(
                algorithms_raw,
                "zone_partition_policy",
                default=AlgorithmSelection().zone_partition_policy,
            ),
            distributed_sync_interval_steps=_read_optional_int(
                algorithms_raw,
                "distributed_sync_interval_steps",
                default=AlgorithmSelection().distributed_sync_interval_steps,
                minimum=1,
            ),
            distributed_broadcast_range_m=_read_optional_float(
                algorithms_raw,
                "distributed_broadcast_range_m",
                default=AlgorithmSelection().distributed_broadcast_range_m,
                minimum=0.0,
            ),
            distributed_winner_memory_ttl_steps=_read_optional_int(
                algorithms_raw,
                "distributed_winner_memory_ttl_steps",
                default=AlgorithmSelection().distributed_winner_memory_ttl_steps,
                minimum=0,
            ),
            distributed_bundle_length=_read_optional_int(
                algorithms_raw,
                "distributed_bundle_length",
                default=AlgorithmSelection().distributed_bundle_length,
                minimum=1,
            ),
            usv_path_planner=_read_str(algorithms_raw, "usv_path_planner"),
            uav_search_planner=_read_str(algorithms_raw, "uav_search_planner"),
            execution_policy=_read_str(algorithms_raw, "execution_policy"),
        ),
        information_map=_build_information_map_config(
            info_map_raw,
            base=preset.information_map,
        ),
    )
    return apply_experiment_overrides(
        config,
        seed=seed_override,
        steps=steps_override,
        dt_seconds=dt_override,
    )


def apply_experiment_overrides(
    config: ExperimentConfig,
    *,
    seed: int | None = None,
    steps: int | None = None,
    dt_seconds: float | None = None,
) -> ExperimentConfig:
    """Return a copy of one config with CLI/runtime overrides applied."""

    simulation = config.simulation
    return ExperimentConfig(
        simulation=SimulationConfig(
            seed=simulation.seed if seed is None else seed,
            steps=simulation.steps if steps is None else steps,
            dt_seconds=simulation.dt_seconds if dt_seconds is None else dt_seconds,
        ),
        scenario=config.scenario,
        algorithms=config.algorithms,
        information_map=config.information_map,
    )


def serialize_experiment_config(config: ExperimentConfig) -> dict[str, object]:
    """Serialize one experiment configuration for logs and summaries."""

    payload = asdict(config)
    payload["algorithms"] = dict(payload["algorithms"])
    payload["scenario"] = dict(payload["scenario"])
    payload["simulation"] = dict(payload["simulation"])
    payload["information_map"] = dict(payload["information_map"])
    return payload


def validate_experiment_config(config: ExperimentConfig) -> None:
    """Validate that the current baseline can execute the requested config."""

    supported_allocators = {
        "basic_task_allocator",
        "cost_aware_centralized_allocator",
        "aoi_energy_auction_allocator",
        "rho_task_allocator",
        "distributed_cbba_allocator",
    }
    supported_partition_policies = {
        "baseline_fixed_partition",
        "soft_partition_policy",
        "backlog_aware_partition_policy",
        "weighted_voronoi_partition_policy",
    }
    supported_usv_planners = set(SUPPORTED_USV_PATH_PLANNERS)
    supported_uav_planners = {
        "uav_lawnmower_planner",
        "uav_multi_region_coverage_planner",
        "uav_persistent_multi_region_coverage_planner",
    }
    supported_execution = {"phase_one_execution", "local_mpc_execution"}
    supported_scenarios = set(list_scenario_names())

    if config.scenario.name not in supported_scenarios:
        raise ValueError(
            "Unsupported scenario "
            f"{config.scenario.name!r}; supported: {sorted(supported_scenarios)}"
        )

    if config.algorithms.task_allocator not in supported_allocators:
        raise ValueError(
            "Unsupported task allocator "
            f"{config.algorithms.task_allocator!r}; supported: {sorted(supported_allocators)}"
        )
    if config.algorithms.zone_partition_policy not in supported_partition_policies:
        raise ValueError(
            "Unsupported zone partition policy "
            f"{config.algorithms.zone_partition_policy!r}; "
            f"supported: {sorted(supported_partition_policies)}"
        )
    if config.algorithms.distributed_bundle_length not in {1, 2}:
        raise ValueError(
            "Unsupported distributed CBBA bundle length "
            f"{config.algorithms.distributed_bundle_length!r}; supported: [1, 2]"
        )
    if config.algorithms.usv_path_planner not in supported_usv_planners:
        raise ValueError(
            "Unsupported USV path planner "
            f"{config.algorithms.usv_path_planner!r}; supported: {sorted(supported_usv_planners)}"
        )
    if config.algorithms.uav_search_planner not in supported_uav_planners:
        raise ValueError(
            "Unsupported UAV search planner "
            f"{config.algorithms.uav_search_planner!r}; supported: {sorted(supported_uav_planners)}"
        )
    if config.algorithms.execution_policy not in supported_execution:
        raise ValueError(
            "Unsupported execution policy "
            f"{config.algorithms.execution_policy!r}; supported: {sorted(supported_execution)}"
        )


def _build_information_map_config(
    raw: dict[str, object],
    *,
    base: InformationMapConfig,
) -> InformationMapConfig:
    defaults = base
    values = asdict(base)
    for field in fields(InformationMapConfig):
        if field.name not in raw:
            continue
        value = raw[field.name]
        default_value = getattr(defaults, field.name)
        if isinstance(default_value, bool):
            if not isinstance(value, bool):
                raise ValueError(f"information_map.{field.name} must be a boolean")
        elif isinstance(default_value, int):
            if not isinstance(value, int):
                raise ValueError(f"information_map.{field.name} must be an integer")
        elif isinstance(default_value, float):
            if not isinstance(value, (int, float)):
                raise ValueError(f"information_map.{field.name} must be a float")
            value = float(value)
        values[field.name] = value
    return InformationMapConfig(**values)


def _expect_table(raw: dict[str, object], key: str) -> dict[str, object]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Config table [{key}] is required")
    return value


def _read_optional_table(raw: dict[str, object], key: str) -> dict[str, object]:
    value = raw.get(key)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"Config table [{key}] must be a table when provided")
    return value


def _read_scenario_name(raw: object) -> str:
    if raw is None:
        return ScenarioSelection().name
    if not isinstance(raw, dict):
        raise ValueError("Config table [scenario] must be a table when provided")
    return _read_str(raw, "name")


def _read_str(raw: dict[str, object], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _read_optional_str(raw: dict[str, object], key: str, *, default: str) -> str:
    value = raw.get(key)
    if value is None:
        return default
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string when provided")
    return value


def _read_optional_int(
    raw: dict[str, object],
    key: str,
    *,
    default: int | None = None,
    minimum: int | None = None,
) -> int | None:
    value = raw.get(key)
    if value is None:
        return default
    if not isinstance(value, int):
        raise ValueError(f"{key} must be an integer or null")
    if minimum is not None and value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return value


def _read_optional_float(
    raw: dict[str, object],
    key: str,
    *,
    default: float,
    minimum: float,
) -> float:
    value = raw.get(key)
    if value is None:
        return default
    if not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a float when provided")
    float_value = float(value)
    if float_value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return float_value


def _read_int(raw: dict[str, object], key: str, *, minimum: int) -> int:
    value = raw.get(key)
    if not isinstance(value, int):
        raise ValueError(f"{key} must be an integer")
    if value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return value


def _read_float(
    raw: dict[str, object],
    key: str,
    *,
    minimum: float,
    strict_minimum: bool = False,
) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a float")
    float_value = float(value)
    if strict_minimum:
        if float_value <= minimum:
            raise ValueError(f"{key} must be > {minimum}")
    elif float_value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return float_value
