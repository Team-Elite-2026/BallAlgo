#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parents[1] / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from run_pose_sweep import main


if __name__ == "__main__":
    raise SystemExit(main())
