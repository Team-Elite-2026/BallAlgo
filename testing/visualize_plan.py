#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

from matplotlib import pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ballalgo_testing import load_artifact
from ballalgo_testing.plots import (
    plot_error_timeseries,
    plot_field_overview,
    plot_heading_panel,
    plot_velocity_panels,
    save_or_show,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize a BallAlgo planner bench artifact.")
    parser.add_argument("artifact", help="Path to a planner bench JSON artifact.")
    parser.add_argument("--output", help="Optional path for a saved image instead of an interactive window.")
    parser.add_argument(
        "--quiver-stride",
        type=int,
        default=5,
        help="Show every Nth action direction arrow on the field plot.",
    )
    parser.add_argument(
        "--hide-action-directions",
        action="store_true",
        help="Skip the field-direction quiver overlay.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    artifact = load_artifact(args.artifact)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    plot_field_overview(
        axes[0, 0],
        artifact,
        quiver_stride=max(1, args.quiver_stride),
        show_action_directions=not args.hide_action_directions,
    )
    plot_error_timeseries(axes[0, 1], artifact)
    plot_velocity_panels(axes[1, 0], artifact)
    plot_heading_panel(axes[1, 1], artifact)
    fig.suptitle(f"BallAlgo Planner Overview: {artifact.raw['label']}", fontsize=14)
    save_or_show(fig, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
