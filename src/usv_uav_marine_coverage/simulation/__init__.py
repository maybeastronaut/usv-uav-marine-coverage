"""Public facade for the replay-style simulation stage."""

from __future__ import annotations

import webbrowser
from pathlib import Path

from .experiment_batch import (
    BatchExperimentArtifacts,
    BatchExperimentSpec,
    load_batch_experiment_spec,
    run_batch_experiment,
)
from .experiment_config import (
    ExperimentConfig,
    build_default_experiment_config,
    load_experiment_config,
    validate_experiment_config,
)
from .simulation_core import SimulationFrame, SimulationReplay, build_simulation_replay
from .simulation_logging import (
    SimulationArtifacts,
    derive_log_paths,
    write_events_jsonl,
    write_summary_json,
)
from .simulation_replay_view import build_simulation_html


def write_simulation_html(
    output_path: Path,
    config_path: Path | None = None,
    seed: int | None = None,
    steps: int | None = None,
    dt_seconds: float | None = None,
) -> Path:
    """Write the simulation replay page to disk."""

    artifacts = write_simulation_artifacts(
        output_path,
        generate_html=True,
        config_path=config_path,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    assert artifacts.html_path is not None
    return artifacts.html_path


def write_simulation_artifacts(
    output_path: Path,
    generate_html: bool = True,
    config_path: Path | None = None,
    seed: int | None = None,
    steps: int | None = None,
    dt_seconds: float | None = None,
) -> SimulationArtifacts:
    """Write replay logs and optionally the HTML page to disk."""

    experiment_config = _resolve_experiment_config(
        config_path=config_path,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    replay = build_simulation_replay(experiment_config=experiment_config)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    events_path, summary_path = derive_log_paths(output_path)
    html_path: Path | None = None
    if generate_html:
        output_path.write_text(build_simulation_html(replay), encoding="utf-8")
        html_path = output_path
    write_events_jsonl(replay, events_path)
    write_summary_json(replay, summary_path)
    return SimulationArtifacts(
        html_path=html_path,
        events_path=events_path,
        summary_path=summary_path,
    )


def run_simulation_viewer(
    output_path: Path | None = None,
    open_browser: bool = True,
    generate_html: bool = True,
    config_path: Path | None = None,
    seed: int | None = None,
    steps: int | None = None,
    dt_seconds: float | None = None,
) -> SimulationArtifacts:
    """Create simulation artifacts and optionally open the replay page."""

    if output_path is None:
        output_path = Path.cwd() / "outputs" / "usv_uav_simulation_replay.html"

    artifacts = write_simulation_artifacts(
        output_path,
        generate_html=generate_html,
        config_path=config_path,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    if open_browser and artifacts.html_path is not None:
        webbrowser.open(artifacts.html_path.resolve().as_uri())
    return artifacts


def _resolve_experiment_config(
    *,
    config_path: Path | None,
    seed: int | None,
    steps: int | None,
    dt_seconds: float | None,
) -> ExperimentConfig:
    if config_path is None:
        config = build_default_experiment_config(
            seed=seed,
            steps=40 if steps is None else steps,
            dt_seconds=1.0 if dt_seconds is None else dt_seconds,
        )
    else:
        config = load_experiment_config(
            config_path,
            seed_override=seed,
            steps_override=steps,
            dt_override=dt_seconds,
        )
    validate_experiment_config(config)
    return config


__all__ = [
    "BatchExperimentArtifacts",
    "BatchExperimentSpec",
    "SimulationArtifacts",
    "ExperimentConfig",
    "SimulationFrame",
    "SimulationReplay",
    "build_default_experiment_config",
    "load_batch_experiment_spec",
    "build_simulation_html",
    "build_simulation_replay",
    "load_experiment_config",
    "run_batch_experiment",
    "run_simulation_viewer",
    "write_simulation_artifacts",
    "write_simulation_html",
]
