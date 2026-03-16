"""Package entry point."""

from __future__ import annotations

import argparse
from pathlib import Path

from usv_uav_marine_coverage.simulation import (
    load_batch_experiment_spec,
    run_batch_experiment,
    run_simulation_viewer,
)
from usv_uav_marine_coverage.viewer import run_map_viewer


def main() -> None:
    parser = argparse.ArgumentParser(description="Display the three-zone sea map.")
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional base output path. Defaults to the outputs/ directory.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        help="Optional TOML experiment config for simulation runs.",
    )
    parser.add_argument(
        "--batch-config",
        type=Path,
        help="Optional TOML batch experiment config for simulation runs.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Write outputs without opening a browser.",
    )
    parser.add_argument(
        "--no-html",
        action="store_true",
        help="For simulation runs, write machine-readable logs only and skip HTML output.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="Optional random seed for reproducible obstacle generation.",
    )
    parser.add_argument(
        "--mode",
        choices=("clean", "debug"),
        default="clean",
        help="Initial sea map view mode inside the generated HTML. Defaults to clean.",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Generate the replay-style simulation preview instead of the static sea map.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        help="Optional number of simulation steps. Defaults to 40 unless provided by --config.",
    )
    args = parser.parse_args()
    if args.simulate:
        if args.batch_config is not None:
            conflicting_args = (
                args.config,
                args.output,
                args.seed,
                args.steps,
            )
            if any(value is not None for value in conflicting_args):
                parser.error(
                    "--batch-config cannot be combined with --config, --output, --seed, or --steps"
                )
            run_batch_experiment(load_batch_experiment_spec(args.batch_config))
            return
        run_simulation_viewer(
            output_path=args.output,
            open_browser=not args.no_open and not args.no_html,
            generate_html=not args.no_html,
            config_path=args.config,
            seed=args.seed,
            steps=args.steps,
        )
        return

    run_map_viewer(
        output_path=args.output,
        open_browser=not args.no_open,
        seed=args.seed,
        mode=args.mode,
    )


if __name__ == "__main__":
    main()
