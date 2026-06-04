from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the BallAlgo Foxglove sidecar and fake runtime demo.")
    parser.add_argument("--config", default="foxglove_sim/foxglove.conf", help="Shared Foxglove config path.")
    parser.add_argument(
        "--scenario",
        default="replan_demo",
        choices=["replan_demo", "orbit_ball", "straight_line", "idle"],
        help="Fake runtime scenario to stream.",
    )
    parser.add_argument("--hz", type=float, default=30.0, help="Fake runtime publish rate.")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Optional finite fake runtime duration.")
    parser.add_argument("--label", default="fake-runtime", help="Scenario label.")
    parser.add_argument(
        "--sidecar-python",
        default=sys.executable,
        help="Python interpreter to use for the Foxglove sidecar. This interpreter must have foxglove-sdk installed.",
    )
    parser.add_argument(
        "--runtime-python",
        default=sys.executable,
        help="Python interpreter to use for the fake runtime publisher.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = (repo_root / config_path).resolve()

    sidecar_cmd = [
        args.sidecar_python,
        str(repo_root / "foxglove_sim" / "sidecar.py"),
        "--config",
        str(config_path),
        "--session-label",
        f"{args.label}-{args.scenario}",
        "--note",
        "local fake foxglove demo",
    ]
    fake_cmd = [
        args.runtime_python,
        str(repo_root / "foxglove_sim" / "fake_runtime.py"),
        "--config",
        str(config_path),
        "--scenario",
        args.scenario,
        "--hz",
        str(args.hz),
        "--duration-s",
        str(args.duration_s),
        "--label",
        args.label,
    ]

    sidecar = subprocess.Popen(sidecar_cmd, cwd=repo_root)
    try:
        time.sleep(0.75)
        sidecar_status = sidecar.poll()
        if sidecar_status is not None:
            return sidecar_status

        fake = subprocess.run(fake_cmd, cwd=repo_root, check=False)
        return fake.returncode
    finally:
        sidecar.terminate()
        try:
            sidecar.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            sidecar.kill()
            sidecar.wait()


if __name__ == "__main__":
    raise SystemExit(main())
