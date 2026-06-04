from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

from config import load_config


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


def _wait_for_sidecar(sidecar: subprocess.Popen, socket_path: str, timeout_s: float = 3.0) -> bool:
    """Poll until the sidecar's Unix socket file appears, or the process exits."""
    sock = Path(socket_path)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if sidecar.poll() is not None:
            return False  # sidecar crashed
        if sock.exists():
            return True
        time.sleep(0.05)
    return sock.exists()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = (repo_root / config_path).resolve()

    cfg = load_config(config_path)

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
        if not _wait_for_sidecar(sidecar, cfg.socket_path):
            print("Sidecar failed to start or timed out waiting for socket.")
            return sidecar.returncode or 1

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
