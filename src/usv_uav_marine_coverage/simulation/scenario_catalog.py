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
    "planner_path_stress": ScenarioPreset(
        name="planner_path_stress",
        description=(
            "USV 路径规划对比场景，保持中等远海热点压力并进一步压低近海基础任务干扰，"
            "适合放大跨风险区往返、任务接入与回巡航路径差异。"
        ),
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            max_active_hotspots=10,
            offshore_hotspot_spawn_probability=0.055,
            nearshore_baseline_spawn_probability=0.00005,
            baseline_respawn_cooldown_steps=360,
        ),
    ),
    "return_to_patrol_stress": ScenarioPreset(
        name="return_to_patrol_stress",
        description=(
            "USV 回巡航接入压力场景，保持中等远海热点负载并进一步降低近海任务噪声，"
            "适合放大任务完成后长距离回巡航、跨风险区往返与路径折返差异。"
        ),
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            max_active_hotspots=8,
            offshore_hotspot_spawn_probability=0.05,
            nearshore_baseline_spawn_probability=0.00002,
            baseline_respawn_cooldown_steps=420,
            information_timeout_steps=360,
        ),
    ),
    "aoi_revisit_pressure": ScenarioPreset(
        name="aoi_revisit_pressure",
        description=(
            "AoI 优势放大场景，缩短信息超时并保持中等热点压力，同时保留少量近海基础任务干扰，"
            "适合放大“近但不急”和“远但更 stale”之间的任务价值冲突。"
        ),
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            information_timeout_steps=300,
            nearshore_information_timeout_steps=520,
            max_active_hotspots=10,
            max_active_baseline_tasks=2,
            nearshore_baseline_spawn_probability=0.00018,
            baseline_respawn_cooldown_steps=260,
            offshore_hotspot_spawn_probability=0.05,
        ),
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
    "distributed_overlap_pressure": ScenarioPreset(
        name="distributed_overlap_pressure",
        description=(
            "分布式协商放大场景，同时提高近海基础任务、近海热点和远海热点活跃度，并缩短信息超时，"
            "适合放大 weighted Voronoi 下的多艇候选重叠、跨区重分配与局部通信约束影响。"
        ),
        information_map=replace(
            _BASELINE_INFORMATION_MAP,
            information_timeout_steps=340,
            nearshore_information_timeout_steps=560,
            max_active_baseline_tasks=2,
            max_active_hotspots=14,
            nearshore_baseline_spawn_probability=0.00035,
            baseline_respawn_cooldown_steps=180,
            nearshore_hotspot_spawn_probability=0.008,
            offshore_hotspot_spawn_probability=0.075,
            hotspot_clearance_cells=0,
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
