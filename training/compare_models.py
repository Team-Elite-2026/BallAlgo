#!/usr/bin/env python3
"""Compare the candidate ball-distance models on the held-out eval set and pick
the most accurate one.

Candidates:
  * parametric  -- log-poly-Fourier coefficients JSON (torch-free).
  * mlp         -- a training run's best checkpoint (needs torch; skipped if the
                   checkpoint or torch is unavailable).

Usage:
  python3 compare_models.py \
      --parametric parametric_model_cleaned.json \
      --mlp-run run_cleaned_xy_radius_300 \
      --mlp-run run_cleaned_polar_300 \
      --eval-csv distance_eval_data.csv

Reports MAE / RMSE / MAPE / max-error per model and declares the winner by MAPE
(relative error matters most for a distance sensor that spans 13..230 cm).
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent


def read_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(newline="") as handle:
        for i, row in enumerate(csv.reader(handle)):
            if i == 0 or not row or not row[0].strip():
                continue
            rows.append([float(row[0]), float(row[1]), float(row[2])])
    data = np.asarray(rows, dtype=np.float64)
    return data[:, :2], data[:, 2]


def metrics(pred: np.ndarray, true: np.ndarray) -> dict[str, float]:
    err = pred - true
    abs_err = np.abs(err)
    nz = np.abs(true) > 1e-6
    return {
        "mae_cm": float(abs_err.mean()),
        "rmse_cm": float(np.sqrt((err * err).mean())),
        "mape_percent": float((abs_err[nz] / np.abs(true[nz])).mean() * 100.0),
        "max_abs_error_cm": float(abs_err.max()),
    }


# --- parametric ------------------------------------------------------------
def parametric_predict(coords: np.ndarray, model: dict) -> np.ndarray:
    P = int(model["radial_degree"])
    M = int(model["angular_order"])
    weights = np.asarray(model["weights"], dtype=np.float64)
    x, y = coords[:, 0], coords[:, 1]
    r = np.clip(np.hypot(x, y), 1e-6, None)
    logr = np.log(r)
    theta = np.arctan2(y, x)
    base = [np.ones_like(theta)]
    for k in range(1, M + 1):
        base += [np.cos(k * theta), np.sin(k * theta)]
    design = np.column_stack([(logr**p) * b for p in range(P + 1) for b in base])
    return np.exp(design @ weights)


# --- numpy mlp (torch-free reference, weights saved as JSON) ---------------
def numpy_mlp_predict(coords: np.ndarray, model: dict) -> np.ndarray:
    mode = model["feature_mode"]
    x, y = coords[:, 0:1], coords[:, 1:2]
    r = np.hypot(x, y)
    if mode == "xy_radius":
        feats = np.concatenate([x, y, r], axis=1)
    else:  # polar
        th = np.arctan2(y, x)
        feats = np.concatenate([r, np.sin(th), np.cos(th)], axis=1)
    fm = np.asarray(model["feature_mean"]); fs = np.asarray(model["feature_std"])
    tm = np.asarray(model["target_mean"]); ts = np.asarray(model["target_std"])
    h = (feats - fm) / fs
    weights = [np.asarray(w) for w in model["weights"]]
    biases = [np.asarray(b) for b in model["biases"]]
    for i in range(len(weights) - 1):
        h = np.maximum(h @ weights[i] + biases[i], 0.0)
    out = h @ weights[-1] + biases[-1]
    return (out * ts + tm).reshape(-1)


# --- mlp (torch checkpoint) ------------------------------------------------
def mlp_best_checkpoint_predict(run_dir: Path, coords: np.ndarray, true: np.ndarray):
    """Return (metrics, best_checkpoint_name) for the run's best checkpoint by MAE,
    or None if torch / checkpoints are unavailable."""
    import sys
    if str(TRAINING_DIR) not in sys.path:
        sys.path.insert(0, str(TRAINING_DIR))
    try:
        from distance_model import load_checkpoint_model, predict_model  # noqa: WPS433
    except SystemExit:
        return None  # torch missing

    ckpt_dir = run_dir / "checkpoints"
    ckpts = sorted(ckpt_dir.glob("epoch_*.pt"))
    if not ckpts:
        return None
    best = None
    for ckpt in ckpts:
        model, norm = load_checkpoint_model(ckpt)
        pred = predict_model(model, norm, coords).reshape(-1)
        m = metrics(pred, true)
        if best is None or m["mae_cm"] < best[0]["mae_cm"]:
            best = (m, ckpt.name)
    return best


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--eval-csv", type=Path, default=TRAINING_DIR / "distance_eval_data.csv")
    parser.add_argument("--parametric", type=Path, action="append", default=[],
                        help="parametric model JSON (repeatable)")
    parser.add_argument("--mlp-run", type=str, action="append", default=[],
                        help="run folder name under training/ (repeatable)")
    parser.add_argument("--numpy-mlp", type=Path, action="append", default=[],
                        help="numpy-MLP weights JSON from train_distance_model_numpy.py (repeatable)")
    args = parser.parse_args()

    coords, true = read_csv(args.eval_csv)
    print(f"Held-out evaluation on {args.eval_csv.name} ({len(true)} points)\n")

    table: list[tuple[str, dict[str, float]]] = []

    for pj in args.parametric:
        model = json.loads(Path(pj).read_text())
        pred = parametric_predict(coords, model)
        table.append((f"parametric:{Path(pj).stem}", metrics(pred, true)))

    for nj in args.numpy_mlp:
        model = json.loads(Path(nj).read_text())
        pred = numpy_mlp_predict(coords, model)
        table.append((f"numpy_mlp:{model.get('feature_mode', Path(nj).stem)}", metrics(pred, true)))

    for run in args.mlp_run:
        run_dir = TRAINING_DIR / run
        result = mlp_best_checkpoint_predict(run_dir, coords, true)
        if result is None:
            print(f"  (skipped MLP run {run}: torch or checkpoints unavailable)")
            continue
        m, ckpt = result
        table.append((f"mlp:{run}({ckpt})", m))

    if not table:
        raise SystemExit("No models evaluated.")

    print(f"{'model':42s} {'MAE':>8} {'RMSE':>8} {'MAPE':>8} {'max':>8}")
    print("-" * 80)
    for name, m in table:
        print(f"{name:42s} {m['mae_cm']:>7.2f} {m['rmse_cm']:>7.2f} "
              f"{m['mape_percent']:>7.2f}% {m['max_abs_error_cm']:>7.2f}")

    winner = min(table, key=lambda t: t[1]["mape_percent"])
    print(f"\nMost accurate (by held-out MAPE): {winner[0]}  "
          f"-> MAPE {winner[1]['mape_percent']:.2f}%, MAE {winner[1]['mae_cm']:.2f}cm")


if __name__ == "__main__":
    main()
