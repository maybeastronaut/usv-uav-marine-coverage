"""Package entry point."""

from __future__ import annotations

import argparse
from pathlib import Path

from usv_uav_marine_coverage.simulation import run_simulation_viewer
from usv_uav_marine_coverage.viewer import run_map_viewer


def main() -> None:
    parser = argparse.ArgumentParser(description="Display the three-zone sea map.")
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional base output path. Defaults to the outputs/ directory.",
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
        default=40,
        help="Number of simulation steps for the replay preview. Defaults to 40.",
    )
    args = parser.parse_args()
    if args.simulate:
        run_simulation_viewer(
            output_path=args.output,
            open_browser=not args.no_open and not args.no_html,
            generate_html=not args.no_html,
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
