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
    plot_acceleration_panels,
    plot_angular_command_panel,
    plot_velocity_panels,
    save_or_show,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot action-chunk command signals from a planner artifact.")
    parser.add_argument("artifact", help="Path to a planner bench JSON artifact.")
    parser.add_argument("--output", help="Optional path for a saved image instead of an interactive window.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    artifact = load_artifact(args.artifact)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    plot_velocity_panels(axes[0], artifact)
    plot_acceleration_panels(axes[1], artifact)
    plot_angular_command_panel(axes[2], artifact)
    fig.suptitle(f"BallAlgo Chunk Signals: {artifact.raw['label']}", fontsize=14)
    save_or_show(fig, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
