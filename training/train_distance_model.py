#!/usr/bin/env python3
"""Train a ball pixel-coordinate to distance neural network.

CSV format:
    dx,dy,measured_cm

All rows are used for training. The MLP consumes three features:
``dx``, ``dy``, and the pixel-radius ``sqrt(dx^2 + dy^2)``.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

try:
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset
except ModuleNotFoundError as exc:
    torch = None
    nn = None
    DataLoader = None
    TensorDataset = None
    TORCH_IMPORT_ERROR = exc
else:
    TORCH_IMPORT_ERROR = None

from distance_model import DistanceMLP, build_input_features


TRAINING_DIR = Path(__file__).resolve().parent
DEFAULT_TRAIN_CSV = TRAINING_DIR / "training_data.csv"
DEFAULT_PLOTS_DIR = TRAINING_DIR / "training_plots"
CHECKPOINT_INTERVAL_EPOCHS = 50


def require_torch() -> None:
    if TORCH_IMPORT_ERROR is not None:
        raise SystemExit(
            "Missing dependency: torch. Install the training packages with:\n"
            "  python3 -m pip install torch matplotlib\n"
        ) from TORCH_IMPORT_ERROR


def read_distance_csv(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows: list[list[float]] = []
    with csv_path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for line_number, row in enumerate(reader, start=1):
            if not row or all(not cell.strip() for cell in row):
                continue
            if len(row) < 3:
                raise ValueError(f"{csv_path}:{line_number} needs at least 3 columns")
            try:
                rows.append([float(row[0]), float(row[1]), float(row[2])])
            except ValueError:
                if line_number == 1:
                    continue
                raise ValueError(f"{csv_path}:{line_number} has non-numeric x/y/measured_cm values") from None

    if not rows:
        raise ValueError(f"{csv_path} did not contain any data rows")

    data = np.asarray(rows, dtype=np.float32)
    return data[:, :2], data[:, 2:3]


def metric_summary(pred_cm: np.ndarray, measured_cm: np.ndarray) -> dict[str, float]:
    errors = pred_cm.reshape(-1) - measured_cm.reshape(-1)
    abs_errors = np.abs(errors)
    measured = measured_cm.reshape(-1)
    nonzero = np.abs(measured) > 1e-6
    summary = {
        "mse_cm2": float(np.mean(errors * errors)),
        "mae_cm": float(np.mean(abs_errors)),
        "rmse_cm": float(np.sqrt(np.mean(errors * errors))),
        "max_abs_error_cm": float(np.max(abs_errors)),
    }
    if np.any(nonzero):
        summary["mape_percent"] = float(np.mean(abs_errors[nonzero] / np.abs(measured[nonzero])) * 100.0)
    return summary


def far_distance_weighted_mse(
    predictions: torch.Tensor,
    targets: torch.Tensor,
    distance_mean: torch.Tensor,
    distance_std: torch.Tensor,
    max_distance_cm: float,
) -> torch.Tensor:
    """MSE weighted by true distance in cm (farther ball -> larger penalty)."""
    targets_cm = targets * distance_std + distance_mean
    distance_fraction = torch.clamp(targets_cm / max_distance_cm, min=0.0, max=1.0)
    distance_weights = 1.0 + (distance_fraction **2 ) * 1.5
    squared_errors = (predictions - targets) ** 2
    return torch.mean(squared_errors * distance_weights)


def print_metrics(name: str, pred_cm: np.ndarray, measured_cm: np.ndarray) -> None:
    metrics = metric_summary(pred_cm, measured_cm)
    print(f"\n{name}")
    print("-" * len(name))
    print(f"MSE loss:      {metrics['mse_cm2']:.3f} cm^2")
    print(f"MAE:           {metrics['mae_cm']:.3f} cm")
    print(f"RMSE:          {metrics['rmse_cm']:.3f} cm")
    print(f"Max abs error: {metrics['max_abs_error_cm']:.3f} cm")
    if "mape_percent" in metrics:
        print(f"MAPE:          {metrics['mape_percent']:.2f}%")


def train_model(
    args: argparse.Namespace,
) -> tuple[DistanceMLP, dict[str, torch.Tensor], list[dict[str, float]], np.ndarray, np.ndarray, np.ndarray]:
    require_torch()
    coords, measured_cm = read_distance_csv(args.train_csv)
    features = build_input_features(coords)

    feature_mean_np = features.mean(axis=0, keepdims=True)
    feature_std_np = features.std(axis=0, keepdims=True)
    feature_std_np[feature_std_np < 1e-6] = 1.0
    features_norm = (features - feature_mean_np) / feature_std_np
    distance_mean_np = measured_cm.mean(axis=0, keepdims=True)
    distance_std_np = measured_cm.std(axis=0, keepdims=True)
    distance_std_np[distance_std_np < 1e-6] = 1.0
    measured_scaled = (measured_cm - distance_mean_np) / distance_std_np

    torch.manual_seed(args.seed)
    model = DistanceMLP(input_dim=features.shape[1], hidden_sizes=(32, 32))
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate, weight_decay=1e-4)
    dataset = TensorDataset(
        torch.from_numpy(features_norm.astype(np.float32)),
        torch.from_numpy(measured_scaled.astype(np.float32)),
    )
    loader = DataLoader(dataset, batch_size=min(args.batch_size, len(dataset)), shuffle=True)
    all_x = torch.from_numpy(features_norm.astype(np.float32))
    normalization = {
        "feature_mean": torch.from_numpy(feature_mean_np.astype(np.float32)),
        "feature_std": torch.from_numpy(feature_std_np.astype(np.float32)),
        "distance_mean": torch.from_numpy(distance_mean_np.astype(np.float32)),
        "distance_std": torch.from_numpy(distance_std_np.astype(np.float32)),
    }
    distance_mean_t = normalization["distance_mean"]
    distance_std_t = normalization["distance_std"]
    max_distance_cm = float(measured_cm.max())

    history: list[dict[str, float]] = []
    epoch_limit = args.max_epochs
    checkpoint_dir = args.checkpoint_dir
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    print(f"Training on all {len(coords)} CSV rows; no train/test split.")
    print(f"Learning rate: {args.learning_rate:g}")
    print(f"Max epochs: {epoch_limit}")
    print(f"Loss: far-distance weighted MSE (weight = 1.0 + 1.5 * distance_cm / {max_distance_cm:.1f})")
    print(f"Checkpoints every {CHECKPOINT_INTERVAL_EPOCHS} epochs -> {checkpoint_dir}")

    for epoch in range(1, epoch_limit + 1):
        model.train()
        for batch_x, batch_y in loader:
            optimizer.zero_grad()
            predictions = model(batch_x)
            loss = far_distance_weighted_mse(
                predictions,
                batch_y,
                distance_mean_t,
                distance_std_t,
                max_distance_cm,
            )
            loss.backward()
            optimizer.step()

        model.eval()
        with torch.no_grad():
            pred_cm = model(all_x).numpy() * distance_std_np + distance_mean_np
        metrics = metric_summary(pred_cm, measured_cm)
        history.append({"epoch": float(epoch), **metrics})

        if epoch % CHECKPOINT_INTERVAL_EPOCHS == 0:
            checkpoint_path = checkpoint_dir / f"epoch_{epoch:04d}.pt"
            save_checkpoint(model, normalization, checkpoint_path)
            print(f"Saved checkpoint: {checkpoint_path}")

        if epoch == 1 or epoch % args.print_every == 0 or epoch == epoch_limit:
            print(
                f"epoch {epoch:3d} | train MSE {metrics['mse_cm2']:.3f} cm^2 "
                f"| train MAE {metrics['mae_cm']:.3f} cm | RMSE {metrics['rmse_cm']:.3f} cm"
            )

    if epoch_limit % CHECKPOINT_INTERVAL_EPOCHS != 0:
        checkpoint_path = checkpoint_dir / f"epoch_{epoch_limit:04d}.pt"
        save_checkpoint(model, normalization, checkpoint_path)
        print(f"Saved checkpoint: {checkpoint_path}")

    model.eval()
    with torch.no_grad():
        pred_cm = model(all_x).numpy() * distance_std_np + distance_mean_np

    return model, normalization, history, coords, measured_cm, pred_cm.astype(np.float32)


def save_checkpoint(
    model: DistanceMLP,
    normalization: dict[str, torch.Tensor],
    checkpoint_path: Path,
) -> None:
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "normalization": normalization,
            "input_dim": model.input_dim,
            "hidden_sizes": list(model.hidden_sizes),
        },
        checkpoint_path,
    )


def save_training_plots(
    model: DistanceMLP,
    normalization: dict[str, torch.Tensor],
    coords: np.ndarray,
    measured_cm: np.ndarray,
    pred_cm: np.ndarray,
    history: list[dict[str, float]],
    plots_dir: Path,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError("matplotlib is required for plots. Install it with: python3 -m pip install matplotlib") from exc

    plots_dir.mkdir(parents=True, exist_ok=True)
    epochs = np.array([row["epoch"] for row in history])
    mse = np.array([row["mse_cm2"] for row in history])
    mae = np.array([row["mae_cm"] for row in history])
    rmse = np.array([row["rmse_cm"] for row in history])
    measured = measured_cm.reshape(-1)
    predicted = pred_cm.reshape(-1)
    residuals = predicted - measured

    with (plots_dir / "training_history.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["epoch", "mse_cm2", "mae_cm", "rmse_cm", "max_abs_error_cm", "mape_percent"])
        writer.writeheader()
        writer.writerows(history)

    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(epochs, mse, color="#2563eb", linewidth=2)
    axes[0].set_title("Training Loss Over Time")
    axes[0].set_xlabel("Epoch")
    axes[0].set_ylabel("MSE Loss (cm squared)")
    axes[0].grid(alpha=0.25)
    axes[1].plot(epochs, mae, label="MAE", color="#dc2626", linewidth=2)
    axes[1].plot(epochs, rmse, label="RMSE", color="#0f766e", linewidth=2)
    axes[1].set_title("Centimeter Error Over Time")
    axes[1].set_xlabel("Epoch")
    axes[1].set_ylabel("Error (cm)")
    axes[1].legend()
    axes[1].grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "training_loss_history.png", dpi=170)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(6.5, 6))
    axis.scatter(measured, predicted, color="#2563eb", alpha=0.8)
    limit = max(float(np.max(measured)), float(np.max(predicted))) * 1.04
    axis.plot([0, limit], [0, limit], color="#111827", linestyle="--", linewidth=1.5)
    axis.set_title("Predicted vs Measured Distance")
    axis.set_xlabel("Measured Distance (cm)")
    axis.set_ylabel("Predicted Distance (cm)")
    axis.set_xlim(0, limit)
    axis.set_ylim(0, limit)
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "predicted_vs_measured.png", dpi=170)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 5))
    axis.scatter(measured, residuals, color="#7c3aed", alpha=0.8)
    axis.axhline(0, color="#111827", linewidth=1.3)
    axis.set_title("Residual Error by Measured Distance")
    axis.set_xlabel("Measured Distance (cm)")
    axis.set_ylabel("Prediction - Measurement (cm)")
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(plots_dir / "residuals_by_distance.png", dpi=170)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    measured_plot = axes[0].scatter(coords[:, 0], coords[:, 1], c=measured, cmap="viridis", s=35)
    axes[0].set_title("Measured Distance Samples")
    axes[0].set_xlabel("Centered x (px)")
    axes[0].set_ylabel("Centered y (px)")
    figure.colorbar(measured_plot, ax=axes[0], label="Distance (cm)")
    error_plot = axes[1].scatter(coords[:, 0], coords[:, 1], c=np.abs(residuals), cmap="magma", s=35)
    axes[1].set_title("Absolute Training Error")
    axes[1].set_xlabel("Centered x (px)")
    axes[1].set_ylabel("Centered y (px)")
    figure.colorbar(error_plot, ax=axes[1], label="Absolute error (cm)")
    figure.tight_layout()
    figure.savefig(plots_dir / "spatial_error_map.png", dpi=170)
    plt.close(figure)

    x_grid = np.linspace(float(coords[:, 0].min()), float(coords[:, 0].max()), 120)
    y_grid = np.linspace(float(coords[:, 1].min()), float(coords[:, 1].max()), 120)
    xx, yy = np.meshgrid(x_grid, y_grid)
    grid_coords = np.column_stack((xx.reshape(-1), yy.reshape(-1))).astype(np.float32)
    grid_features = build_input_features(grid_coords)
    feature_mean = normalization["feature_mean"].numpy()
    feature_std = normalization["feature_std"].numpy()
    distance_mean = normalization["distance_mean"].numpy()
    distance_std = normalization["distance_std"].numpy()
    grid_norm = (grid_features - feature_mean) / feature_std
    model.eval()
    with torch.no_grad():
        grid_pred = (
            model(torch.from_numpy(grid_norm.astype(np.float32))).numpy() * distance_std + distance_mean
        ).reshape(xx.shape)
    figure, axis = plt.subplots(figsize=(7, 6))
    surface = axis.contourf(xx, yy, grid_pred, levels=22, cmap="viridis")
    axis.scatter(coords[:, 0], coords[:, 1], c=measured, cmap="viridis", edgecolors="white", linewidths=0.4, s=24)
    axis.set_title("Learned Distance Surface")
    axis.set_xlabel("Centered x (px)")
    axis.set_ylabel("Centered y (px)")
    figure.colorbar(surface, ax=axis, label="Predicted distance (cm)")
    figure.tight_layout()
    figure.savefig(plots_dir / "learned_distance_surface.png", dpi=170)
    plt.close(figure)


def resolve_run_dir(run_name: str) -> Path:
    """Place run output at training/<run_name>/ (e.g. training/run_10/)."""
    if run_name in ("", ".", "..") or "/" in run_name or "\\" in run_name:
        raise SystemExit("--run-dir must be a single folder name (e.g. run_10), not a path.")
    return TRAINING_DIR / run_name


def resolve_output_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.run_dir is None:
        return TRAINING_DIR / "checkpoints", args.plots_dir

    run_dir = resolve_run_dir(args.run_dir)
    return run_dir / "checkpoints", run_dir / "training_plots"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train a 3-input dx/dy/radius ball distance model using all CSV rows.",
    )
    parser.add_argument("train_csv", nargs="?", type=Path, default=DEFAULT_TRAIN_CSV, help="Training CSV with x,y,measured_cm columns")
    parser.add_argument("--plots-dir", type=Path, default=DEFAULT_PLOTS_DIR, help="Directory for training plots (without --run-dir)")
    parser.add_argument(
        "--run-dir",
        type=str,
        default=None,
        metavar="NAME",
        help="Run folder name under training/ (e.g. run_10 -> training/run_10/).",
    )
    parser.add_argument("--max-epochs", type=int, required=True, help="Maximum training epochs")
    parser.add_argument("--learning-rate", type=float, required=True, help="Adam learning rate")
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--skip-plots", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.max_epochs < 1 or args.batch_size < 1 or args.learning_rate <= 0:
        raise SystemExit("--max-epochs, --batch-size, and --learning-rate must be positive")

    checkpoint_dir, plots_dir = resolve_output_paths(args)
    args.checkpoint_dir = checkpoint_dir
    if args.run_dir is not None:
        print(f"Run output directory: {resolve_run_dir(args.run_dir)}")
    model, normalization, history, coords, measured_cm, pred_cm = train_model(args)
    print_metrics("Neural network fit on training data (final epoch weights)", pred_cm, measured_cm)

    if not args.skip_plots:
        save_training_plots(model, normalization, coords, measured_cm, pred_cm, history, plots_dir)
        print(f"Saved training plots to: {plots_dir}")


if __name__ == "__main__":
    main()
