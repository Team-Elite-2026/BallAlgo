#!/usr/bin/env python3
"""Evaluate every checkpoint in a run's checkpoints/ folder on a fixed test set.

Test rows match training_data.csv: centered x, y, measured_cm.
Writes metrics to the console and saves plots under evaluation_plots/.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent
if str(TRAINING_DIR) not in sys.path:
    sys.path.insert(0, str(TRAINING_DIR))

from distance_model import load_checkpoint_model, predict_model

# x, y, measured_cm — same coordinate system as training_data.csv
TEST_ROWS = [
    (135, -42, 90.28),
    (51, 34, 16.62),
    (147, 128, 136.0),
    (-27, 127, 64.88),
    (-57, -156, 95.36),
    (129, -149, 148.7),
    (54, -16, 14.08),
    (-66, 39, 26.78),
    (33, -68, 25.51),
    (-100, -18, 43.29),
]


def resolve_run_dir(run_name: str) -> Path:
    if run_name in ("", ".", "..") or "/" in run_name or "\\" in run_name:
        raise SystemExit("--run-dir must be a single folder name (e.g. run_10), not a path.")
    return TRAINING_DIR / run_name


def checkpoint_epoch(path: Path) -> int:
    return int(path.stem.split("_", 1)[1])


def metric_summary(pred_cm: np.ndarray, measured_cm: np.ndarray) -> dict[str, float]:
    error = pred_cm.reshape(-1) - measured_cm.reshape(-1)
    abs_error = np.abs(error)
    measured = measured_cm.reshape(-1)
    summary = {
        "mae_cm": float(np.mean(abs_error)),
        "rmse_cm": float(np.sqrt(np.mean(error * error))),
        "max_abs_error_cm": float(np.max(abs_error)),
    }
    if np.any(np.abs(measured) > 1e-6):
        summary["mape_percent"] = float(np.mean(abs_error / np.abs(measured)) * 100.0)
    return summary


def evaluate_checkpoint(
    checkpoint_path: Path,
    coords_np: np.ndarray,
    measured_np: np.ndarray,
) -> tuple[dict[str, float], np.ndarray, np.ndarray]:
    model, normalization = load_checkpoint_model(checkpoint_path)
    pred = predict_model(model, normalization, coords_np).reshape(-1)
    error = pred - measured_np
    metrics = metric_summary(pred, measured_np)
    return metrics, pred, error


def save_evaluation_plots(
    plots_dir: Path,
    checkpoint_rows: list[dict[str, float]],
    coords_np: np.ndarray,
    measured_np: np.ndarray,
    best_pred: np.ndarray,
    best_error: np.ndarray,
    best_checkpoint_name: str,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError("matplotlib is required for plots. Install it with: python3 -m pip install matplotlib") from exc

    plots_dir.mkdir(parents=True, exist_ok=True)
    measured = measured_np.reshape(-1)
    predicted = best_pred.reshape(-1)
    residuals = best_error.reshape(-1)
    abs_error = np.abs(residuals)

    fieldnames = [
        "epoch",
        "checkpoint",
        "mae_cm",
        "rmse_cm",
        "max_abs_error_cm",
        "mape_percent",
    ]
    with (plots_dir / "evaluation_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(checkpoint_rows)

    epochs = np.array([int(row["epoch"]) for row in checkpoint_rows])
    mae = np.array([row["mae_cm"] for row in checkpoint_rows])
    rmse = np.array([row["rmse_cm"] for row in checkpoint_rows])

    figure, axis = plt.subplots(figsize=(9, 5))
    axis.plot(epochs, mae, label="MAE", color="#dc2626", linewidth=2, marker="o")
    axis.plot(epochs, rmse, label="RMSE", color="#0f766e", linewidth=2, marker="o")
    axis.set_title("Test-Set Error vs Training Epoch")
    axis.set_xlabel("Epoch")
    axis.set_ylabel("Error (cm)")
    axis.legend()
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "metrics_vs_epoch.png", dpi=170)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(6.5, 6))
    axis.scatter(measured, predicted, color="#2563eb", alpha=0.85, s=60)
    limit = max(float(np.max(measured)), float(np.max(predicted))) * 1.04
    axis.plot([0, limit], [0, limit], color="#111827", linestyle="--", linewidth=1.5)
    axis.set_title(f"Predicted vs Measured ({best_checkpoint_name})")
    axis.set_xlabel("Measured Distance (cm)")
    axis.set_ylabel("Predicted Distance (cm)")
    axis.set_xlim(0, limit)
    axis.set_ylim(0, limit)
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "predicted_vs_measured.png", dpi=170)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 5))
    axis.scatter(measured, residuals, color="#7c3aed", alpha=0.85, s=60)
    axis.axhline(0, color="#111827", linewidth=1.3)
    axis.set_title(f"Residuals by Distance ({best_checkpoint_name})")
    axis.set_xlabel("Measured Distance (cm)")
    axis.set_ylabel("Prediction - Measurement (cm)")
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "residuals_by_distance.png", dpi=170)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    measured_plot = axes[0].scatter(coords_np[:, 0], coords_np[:, 1], c=measured, cmap="viridis", s=70)
    axes[0].set_title("Test Samples (Measured Distance)")
    axes[0].set_xlabel("Centered x (px)")
    axes[0].set_ylabel("Centered y (px)")
    figure.colorbar(measured_plot, ax=axes[0], label="Distance (cm)")
    error_plot = axes[1].scatter(coords_np[:, 0], coords_np[:, 1], c=abs_error, cmap="magma", s=70)
    axes[1].set_title(f"Absolute Error ({best_checkpoint_name})")
    axes[1].set_xlabel("Centered x (px)")
    axes[1].set_ylabel("Centered y (px)")
    figure.colorbar(error_plot, ax=axes[1], label="Absolute error (cm)")
    figure.tight_layout()
    figure.savefig(plots_dir / "spatial_error_map.png", dpi=170)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 5))
    sample_labels = [f"({int(x)}, {int(y)})" for x, y, _ in TEST_ROWS]
    x_positions = np.arange(len(sample_labels))
    axis.bar(x_positions, abs_error, color="#2563eb", alpha=0.85)
    axis.set_xticks(x_positions)
    axis.set_xticklabels(sample_labels, rotation=35, ha="right")
    axis.set_title(f"Per-Sample Absolute Error ({best_checkpoint_name})")
    axis.set_xlabel("Test point (centered x, y)")
    axis.set_ylabel("Absolute error (cm)")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "per_sample_abs_error.png", dpi=170)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-dir",
        type=str,
        required=True,
        metavar="NAME",
        help="Run folder name under training/ (e.g. run_10).",
    )
    parser.add_argument(
        "--plots-dir",
        type=Path,
        default=None,
        help="Output directory for evaluation plots (default: training/<run>/evaluation_plots).",
    )
    parser.add_argument("--skip-plots", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = resolve_run_dir(args.run_dir)
    checkpoint_dir = run_dir / "checkpoints"
    plots_dir = args.plots_dir if args.plots_dir is not None else run_dir / "evaluation_plots"

    if not checkpoint_dir.is_dir():
        raise SystemExit(f"Checkpoint folder not found: {checkpoint_dir}")

    checkpoint_paths = sorted(checkpoint_dir.glob("epoch_*.pt"), key=checkpoint_epoch)
    if not checkpoint_paths:
        raise SystemExit(f"No epoch_*.pt checkpoints found in {checkpoint_dir}")

    coords_np = np.asarray([[x, y] for x, y, _ in TEST_ROWS], dtype=np.float32)
    measured_np = np.asarray([measured_cm for _, _, measured_cm in TEST_ROWS], dtype=np.float32)

    print(f"Evaluating {len(checkpoint_paths)} checkpoints from {run_dir.name} on {len(TEST_ROWS)} test samples.")
    print(f"Checkpoint directory: {checkpoint_dir}")
    print()

    checkpoint_rows: list[dict[str, float]] = []
    best_name: str | None = None
    best_mae = float("inf")
    best_pred: np.ndarray | None = None
    best_error: np.ndarray | None = None

    for checkpoint_path in checkpoint_paths:
        metrics, pred, error = evaluate_checkpoint(checkpoint_path, coords_np, measured_np)
        row = {
            "epoch": float(checkpoint_epoch(checkpoint_path)),
            "checkpoint": checkpoint_path.name,
            **metrics,
        }
        checkpoint_rows.append(row)
        print(
            f"{checkpoint_path.name}: MAE {metrics['mae_cm']:.3f} cm | "
            f"RMSE {metrics['rmse_cm']:.3f} cm | "
            f"Max {metrics['max_abs_error_cm']:.3f} cm | "
            f"MAPE {metrics.get('mape_percent', float('nan')):.2f}%"
        )
        if metrics["mae_cm"] < best_mae:
            best_mae = metrics["mae_cm"]
            best_name = checkpoint_path.name
            best_pred = pred
            best_error = error

    print()
    if best_name is None or best_pred is None or best_error is None:
        raise SystemExit("No checkpoint produced valid predictions.")

    print(f"Best checkpoint: {best_name} (MAE {best_mae:.3f} cm)")

    if not args.skip_plots:
        save_evaluation_plots(
            plots_dir,
            checkpoint_rows,
            coords_np,
            measured_np,
            best_pred,
            best_error,
            best_name,
        )
        print(f"Saved evaluation plots to: {plots_dir}")


if __name__ == "__main__":
    main()
