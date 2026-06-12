from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_DIR = REPO_ROOT / "build"
DEFAULT_ARTIFACT_DIR = REPO_ROOT / "tests" / "trajectory_cases" / "generated"
SIDECAR_PATH = REPO_ROOT / "foxglove_sim" / "sidecar.py"


def main() -> int:
    parser = argparse.ArgumentParser(description="Run BallAlgo in trajectory replay mode")
    parser.add_argument("artifact", help="Artifact name or path to .traj replay file")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=DEFAULT_BUILD_DIR,
        help="BallAlgo build directory containing the runtime binary",
    )
    parser.add_argument(
        "--with-sidecar",
        action="store_true",
        help="Launch foxglove_sim/sidecar.py before starting the runtime",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python interpreter to use for the optional Foxglove sidecar",
    )
    args = parser.parse_args()

    artifact_path = Path(args.artifact)
    if not artifact_path.exists():
        candidate = DEFAULT_ARTIFACT_DIR / args.artifact
        if candidate.suffix != ".traj":
            candidate = candidate.with_suffix(".traj")
        artifact_path = candidate
    if not artifact_path.exists():
        raise SystemExit(f"artifact not found: {args.artifact}")

    runtime = args.build_dir / "ballalgo"
    if not runtime.exists():
        raise SystemExit(f"runtime binary not found: {runtime}")

    sidecar_proc: subprocess.Popen[str] | None = None
    runtime_proc: subprocess.Popen[str] | None = None

    def stop_children(*_args: object) -> None:
        for proc in (runtime_proc, sidecar_proc):
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop_children)
    signal.signal(signal.SIGTERM, stop_children)

    try:
        if args.with_sidecar:
            sidecar_proc = subprocess.Popen(
                [args.python, str(SIDECAR_PATH)],
                cwd=str(REPO_ROOT),
                text=True,
            )
            time.sleep(0.5)

        runtime_proc = subprocess.Popen(
            [str(runtime), "--trajectory-replay-artifact", str(artifact_path)],
            cwd=str(REPO_ROOT),
            text=True,
            env=os.environ.copy(),
        )
        return runtime_proc.wait()
    finally:
        stop_children()
        if runtime_proc is not None and runtime_proc.poll() is None:
            runtime_proc.wait(timeout=5)
        if sidecar_proc is not None and sidecar_proc.poll() is None:
            sidecar_proc.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
