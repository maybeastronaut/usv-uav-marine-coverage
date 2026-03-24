#!/usr/bin/env python3
"""Scan replay event logs for invariant violations and task flip-flops."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from usv_uav_marine_coverage.simulation.replay_validation import (
    load_step_snapshots_from_events_jsonl,
    summarize_replay_validation,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate one replay events log.")
    parser.add_argument(
        "events_path",
        nargs="?",
        type=Path,
        default=Path("outputs/usv_uav_simulation_replay_events.jsonl"),
        help="Path to one *_events.jsonl replay log. Defaults to outputs/usv_uav_simulation_replay_events.jsonl.",
    )
    args = parser.parse_args()

    step_snapshots = load_step_snapshots_from_events_jsonl(args.events_path)
    if not step_snapshots:
        raise SystemExit(f"No step_snapshot records found in {args.events_path}")

    summary = summarize_replay_validation(step_snapshots)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 1 if summary["has_errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
