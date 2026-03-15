"""Public facade for the replay-style simulation stage."""

from __future__ import annotations

import webbrowser
from pathlib import Path

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
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> Path:
    """Write the simulation replay page to disk."""

    artifacts = write_simulation_artifacts(
        output_path,
        generate_html=True,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    assert artifacts.html_path is not None
    return artifacts.html_path


def write_simulation_artifacts(
    output_path: Path,
    generate_html: bool = True,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationArtifacts:
    """Write replay logs and optionally the HTML page to disk."""

    replay = build_simulation_replay(seed=seed, steps=steps, dt_seconds=dt_seconds)
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
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationArtifacts:
    """Create simulation artifacts and optionally open the replay page."""

    if output_path is None:
        output_path = Path.cwd() / "outputs" / "usv_uav_simulation_replay.html"

    artifacts = write_simulation_artifacts(
        output_path,
        generate_html=generate_html,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    if open_browser and artifacts.html_path is not None:
        webbrowser.open(artifacts.html_path.resolve().as_uri())
    return artifacts


__all__ = [
    "SimulationArtifacts",
    "SimulationFrame",
    "SimulationReplay",
    "build_simulation_html",
    "build_simulation_replay",
    "run_simulation_viewer",
    "write_simulation_artifacts",
    "write_simulation_html",
]
