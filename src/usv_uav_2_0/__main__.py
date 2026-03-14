"""Package entry point."""

from __future__ import annotations

import argparse
from pathlib import Path

from usv_uav_2_0.viewer import run_map_viewer


def main() -> None:
    parser = argparse.ArgumentParser(description="Display the three-zone sea map.")
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional HTML output path for the generated sea map.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Generate the HTML sea map without opening a browser.",
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
    args = parser.parse_args()
    run_map_viewer(
        output_path=args.output,
        open_browser=not args.no_open,
        seed=args.seed,
        mode=args.mode,
    )


if __name__ == "__main__":
    main()
