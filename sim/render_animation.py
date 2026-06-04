#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ballalgo_sim import load_artifact
from ballalgo_sim.animation import save_animation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render a saveable BallAlgo simulation animation.")
    parser.add_argument("artifact", help="Path to a BallAlgo simulation JSON artifact.")
    parser.add_argument("--output", required=True, help="Animation output path (.gif or .mp4).")
    parser.add_argument("--fps", type=int, default=24, help="Rendered frames per second.")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--tail-seconds", type=float, default=1.0, help="Robot trail duration.")
    parser.add_argument(
        "--freeze-final-seconds",
        type=float,
        default=1.0,
        help="How long to hold the final frame at the end of the animation.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    artifact = load_artifact(args.artifact)
    output = save_animation(
        artifact,
        args.output,
        fps=max(1, args.fps),
        speed=max(1e-6, args.speed),
        tail_seconds=max(0.05, args.tail_seconds),
        freeze_final_seconds=max(0.0, args.freeze_final_seconds),
    )
    print(f"wrote animation: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
