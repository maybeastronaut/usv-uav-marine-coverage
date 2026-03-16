"""Reusable experiment-scenario presets for algorithm comparison."""

from __future__ import annotations

from dataclasses import dataclass, replace

from usv_uav_marine_coverage.information_map import InformationMapConfig


@dataclass(frozen=True)
class ScenarioPreset:
    """One reusable simulation scenario preset."""

    name: str
    description: str
    information_map: InformationMapConfig


_BASELINE_INFORMATION_MAP = InformationMapConfig()

SCENARIO_CATALOG: dict[str, ScenarioPreset] = {
    "baseline_patrol": ScenarioPreset(
        name="baseline_patrol",
        description="当前第一阶段 baseline 场景，任务密度适中，适合作为统一对照基线。",
        information_map=_BASELINE_INFORMATION_MAP,
    ),
    "offshore_hotspot_pressure": ScenarioPreset(
        name="offshore_hotspot_pressure",
        description="远海热点压力场景，提升远海热点生成频率，适合放大热点响应与确认算法差异。",
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            offshore_hotspot_spawn_probability=0.08,
            nearshore_baseline_spawn_probability=0.0001,
        ),
    ),
    "nearshore_baseline_pressure": ScenarioPreset(
        name="nearshore_baseline_pressure",
        description="近海基础巡检压力场景，提升近海基础任务密度，适合放大驻区巡航与基础任务调度差异。",
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            max_active_baseline_tasks=3,
            nearshore_baseline_spawn_probability=0.0008,
            baseline_respawn_cooldown_steps=120,
        ),
    ),
    "mixed_task_pressure": ScenarioPreset(
        name="mixed_task_pressure",
        description="混合任务压力场景，同时提高远海热点与近海基础任务压力，适合做多任务冲突对比。",
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            max_active_baseline_tasks=2,
            nearshore_baseline_spawn_probability=0.0005,
            baseline_respawn_cooldown_steps=150,
            offshore_hotspot_spawn_probability=0.06,
        ),
    ),
}


def get_scenario_preset(name: str) -> ScenarioPreset:
    """Return one named scenario preset."""

    try:
        return SCENARIO_CATALOG[name]
    except KeyError as exc:
        raise ValueError(
            f"Unknown scenario {name!r}; supported: {sorted(SCENARIO_CATALOG)}"
        ) from exc


def list_scenario_names() -> tuple[str, ...]:
    """Return supported scenario names in stable order."""

    return tuple(SCENARIO_CATALOG)
