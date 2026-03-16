"""Batch experiment runner for reproducible algorithm comparison."""

from __future__ import annotations

import json
import tomllib
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

from .experiment_config import load_experiment_config, serialize_experiment_config


@dataclass(frozen=True)
class BatchRunSpec:
    """One run inside a batch experiment."""

    label: str
    config_path: Path
    seed_override: int | None = None
    steps_override: int | None = None
    dt_override: float | None = None


@dataclass(frozen=True)
class BatchExperimentSpec:
    """A batch experiment definition loaded from TOML."""

    name: str
    output_dir: Path
    generate_html: bool
    runs: tuple[BatchRunSpec, ...]


@dataclass(frozen=True)
class BatchExperimentArtifacts:
    """Written outputs for one batch experiment run."""

    output_dir: Path
    runs_path: Path
    summary_path: Path


def load_batch_experiment_spec(path: Path) -> BatchExperimentSpec:
    """Load one batch experiment definition from TOML."""

    raw = tomllib.loads(path.read_text(encoding="utf-8"))
    batch_raw = _expect_table(raw, "batch")
    runs_raw = raw.get("runs")
    if not isinstance(runs_raw, list) or not runs_raw:
        raise ValueError("Batch config requires at least one [[runs]] entry")

    base_config_value = batch_raw.get("base_config")
    base_config_path = None
    if base_config_value is not None:
        if not isinstance(base_config_value, str) or not base_config_value:
            raise ValueError("batch.base_config must be a non-empty string when provided")
        base_config_path = _resolve_relative(path, Path(base_config_value))

    output_dir_value = batch_raw.get("output_dir")
    if not isinstance(output_dir_value, str) or not output_dir_value:
        raise ValueError("batch.output_dir must be a non-empty string")

    generate_html = batch_raw.get("generate_html", False)
    if not isinstance(generate_html, bool):
        raise ValueError("batch.generate_html must be a boolean")

    runs: list[BatchRunSpec] = []
    for index, run_raw in enumerate(runs_raw):
        if not isinstance(run_raw, dict):
            raise ValueError(f"runs[{index}] must be a table")
        label = _read_non_empty_str(run_raw, "label")
        config_value = run_raw.get("config")
        if config_value is None:
            if base_config_path is None:
                raise ValueError(
                    f"runs[{index}] must define config when batch.base_config is not set"
                )
            config_path = base_config_path
        else:
            if not isinstance(config_value, str) or not config_value:
                raise ValueError(f"runs[{index}].config must be a non-empty string")
            config_path = _resolve_relative(path, Path(config_value))
        seed_override = _optional_int(run_raw, "seed")
        steps_override = _optional_int(run_raw, "steps")
        dt_override = _optional_float(run_raw, "dt_seconds")
        runs.append(
            BatchRunSpec(
                label=label,
                config_path=config_path,
                seed_override=seed_override,
                steps_override=steps_override,
                dt_override=dt_override,
            )
        )

    name = _read_non_empty_str(batch_raw, "name")
    output_dir = _resolve_relative(path, Path(output_dir_value))
    return BatchExperimentSpec(
        name=name,
        output_dir=output_dir,
        generate_html=generate_html,
        runs=tuple(runs),
    )


def run_batch_experiment(
    spec: BatchExperimentSpec,
) -> BatchExperimentArtifacts:
    """Run one batch experiment and write aggregate outputs."""

    from . import write_simulation_artifacts

    spec.output_dir.mkdir(parents=True, exist_ok=True)
    run_records: list[dict[str, object]] = []
    for run in spec.runs:
        html_path = spec.output_dir / f"{run.label}.html"
        resolved_config = load_experiment_config(
            run.config_path,
            seed_override=run.seed_override,
            steps_override=run.steps_override,
            dt_override=run.dt_override,
        )
        try:
            artifacts = write_simulation_artifacts(
                output_path=html_path,
                generate_html=spec.generate_html,
                config_path=run.config_path,
                seed=run.seed_override,
                steps=run.steps_override,
                dt_seconds=run.dt_override,
            )
        except Exception as exc:
            run_records.append(
                {
                    "label": run.label,
                    "config_path": str(run.config_path),
                    "experiment_config": serialize_experiment_config(resolved_config),
                    "status": "failed",
                    "error_type": type(exc).__name__,
                    "error_message": str(exc),
                    "html_path": None,
                    "events_path": None,
                    "summary_path": None,
                    "final_metrics": None,
                }
            )
            continue

        summary_payload = json.loads(artifacts.summary_path.read_text(encoding="utf-8"))
        run_records.append(
            {
                "label": run.label,
                "config_path": str(run.config_path),
                "experiment_config": serialize_experiment_config(resolved_config),
                "status": "completed",
                "error_type": None,
                "error_message": None,
                "html_path": None if artifacts.html_path is None else str(artifacts.html_path),
                "events_path": str(artifacts.events_path),
                "summary_path": str(artifacts.summary_path),
                "final_metrics": summary_payload["final_metrics"],
            }
        )

    runs_path = spec.output_dir / "batch_results.jsonl"
    with runs_path.open("w", encoding="utf-8") as handle:
        for record in run_records:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")

    summary_path = spec.output_dir / "batch_summary.json"
    summary_payload = build_batch_summary(spec, run_records)
    summary_path.write_text(
        json.dumps(summary_payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return BatchExperimentArtifacts(
        output_dir=spec.output_dir,
        runs_path=runs_path,
        summary_path=summary_path,
    )


def build_batch_summary(
    spec: BatchExperimentSpec,
    run_records: list[dict[str, object]],
) -> dict[str, object]:
    """Build aggregate summary payload for one batch experiment."""

    successful_runs = [
        record
        for record in run_records
        if record["status"] == "completed" and record["final_metrics"]
    ]
    coverage_ratios = [
        float(record["final_metrics"]["coverage_ratio"]) for record in successful_runs
    ]
    valid_cells = [int(record["final_metrics"]["valid_cells"]) for record in successful_runs]
    stale_cells = [int(record["final_metrics"]["stale_cells"]) for record in successful_runs]
    confirmed_cells = [
        int(record["final_metrics"]["confirmed_cells"]) for record in successful_runs
    ]
    false_alarm_cells = [
        int(record["final_metrics"]["false_alarm_cells"]) for record in successful_runs
    ]
    event_totals = [record["final_metrics"]["event_totals"] for record in successful_runs]

    return {
        "batch": {
            "name": spec.name,
            "output_dir": str(spec.output_dir),
            "run_count": len(run_records),
            "successful_runs": len(successful_runs),
            "failed_runs": len(run_records) - len(successful_runs),
            "generate_html": spec.generate_html,
        },
        "runs": run_records,
        "aggregates": {
            "coverage_ratio": _aggregate_numeric_series(coverage_ratios),
            "valid_cells": _aggregate_numeric_series(valid_cells),
            "stale_cells": _aggregate_numeric_series(stale_cells),
            "confirmed_cells": _aggregate_numeric_series(confirmed_cells),
            "false_alarm_cells": _aggregate_numeric_series(false_alarm_cells),
            "event_totals": _aggregate_event_totals(event_totals),
        },
    }


def _aggregate_numeric_series(values: list[int | float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": round(mean(float(value) for value in values), 6),
        "min": round(float(min(values)), 6),
        "max": round(float(max(values)), 6),
    }


def _aggregate_event_totals(event_totals: list[dict[str, object]]) -> dict[str, float]:
    if not event_totals:
        return {
            "spawned_hotspots_mean": 0.0,
            "confirmed_hotspots_mean": 0.0,
            "false_alarms_mean": 0.0,
            "astar_expanded_nodes_mean": 0.0,
        }
    return {
        "spawned_hotspots_mean": round(
            mean(float(item["spawned_hotspots"]) for item in event_totals), 6
        ),
        "confirmed_hotspots_mean": round(
            mean(float(item["confirmed_hotspots"]) for item in event_totals), 6
        ),
        "false_alarms_mean": round(mean(float(item["false_alarms"]) for item in event_totals), 6),
        "astar_expanded_nodes_mean": round(
            mean(float(item["astar_expanded_nodes"]) for item in event_totals), 6
        ),
    }


def _expect_table(raw: dict[str, object], key: str) -> dict[str, object]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Config table [{key}] is required")
    return value


def _read_non_empty_str(raw: dict[str, object], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _optional_int(raw: dict[str, object], key: str) -> int | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, int):
        raise ValueError(f"{key} must be an integer when provided")
    return value


def _optional_float(raw: dict[str, object], key: str) -> float | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a float when provided")
    return float(value)


def _resolve_relative(base_file: Path, candidate: Path) -> Path:
    if candidate.is_absolute():
        return candidate
    return (base_file.parent / candidate).resolve()
