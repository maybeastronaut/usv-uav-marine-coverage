"""Public facade for the replay-style simulation stage."""

from __future__ import annotations

import tempfile
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
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    return artifacts.html_path


def write_simulation_artifacts(
    output_path: Path,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> SimulationArtifacts:
    """Write the replay HTML together with machine-readable simulation logs."""

    replay = build_simulation_replay(seed=seed, steps=steps, dt_seconds=dt_seconds)
    output_path.write_text(build_simulation_html(replay), encoding="utf-8")
    events_path, summary_path = derive_log_paths(output_path)
    write_events_jsonl(replay, events_path)
    write_summary_json(replay, summary_path)
    return SimulationArtifacts(
        html_path=output_path,
        events_path=events_path,
        summary_path=summary_path,
    )


def run_simulation_viewer(
    output_path: Path | None = None,
    open_browser: bool = True,
    seed: int | None = None,
    steps: int = 40,
    dt_seconds: float = 1.0,
) -> Path:
    """Create the simulation replay page and optionally open it in the browser."""

    if output_path is None:
        output_path = Path(tempfile.gettempdir()) / "usv_uav_simulation_replay.html"

    artifacts = write_simulation_artifacts(
        output_path,
        seed=seed,
        steps=steps,
        dt_seconds=dt_seconds,
    )
    if open_browser:
        webbrowser.open(artifacts.html_path.resolve().as_uri())
    return artifacts.html_path


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
