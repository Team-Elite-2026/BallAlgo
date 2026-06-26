#!/usr/bin/env python3
"""Fit and cross-validate a *parametric* ball-distance model and compare it to
the neural net.

Model family (log-linear, so it solves in closed form via least squares):

    log(cm) = sum_{p=0..P} logr^p * A_p(theta)

where each ``A_p`` is a low-order Fourier series in the bearing ``theta``:

    A_p(theta) = a_p0 + sum_{k=1..M} [ a_pk cos(k th) + b_pk sin(k th) ]

Intuition:
  * ``P`` (radial degree) is a polynomial in log(r): the catadioptric mirror's
    distance-vs-radius curve is NOT a pure power law (P=1) -- it bends sharply
    near the rim, so P>=3 is needed to avoid a large systematic radial bias.
  * ``M`` (angular order) captures the mirror's angular asymmetry (tilt /
    decentering); the radial exponent genuinely varies with direction.
  * Fitting in log space optimises *relative* (percentage) error and keeps the
    prediction ``cm = exp(...)`` strictly positive.

P=1, M=0 reduces to a single power law ``cm = a * r^b`` (your old C++ baseline).

Outputs K-fold cross-validated metrics (honest generalisation) so the result is
directly comparable to ``train_distance_model.py --cv-folds`` for the MLP. Saves
coefficients to JSON for torch-free deployment, and checks that the chosen model
is monotonic in r across the sampled region.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent
DEFAULT_CSV = TRAINING_DIR / "distance_training_data.csv"


def read_csv(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows: list[list[float]] = []
    with csv_path.open(newline="") as handle:
        for line_no, row in enumerate(csv.reader(handle), start=1):
            if not row or all(not c.strip() for c in row):
                continue
            try:
                rows.append([float(row[0]), float(row[1]), float(row[2])])
            except ValueError:
                if line_no == 1:
                    continue  # header
                raise
    data = np.asarray(rows, dtype=np.float64)
    return data[:, :2], data[:, 2]


def _angular_block(theta: np.ndarray, order: int) -> list[np.ndarray]:
    cols = [np.ones_like(theta)]
    for k in range(1, order + 1):
        cols.append(np.cos(k * theta))
        cols.append(np.sin(k * theta))
    return cols


def design_matrix(coords: np.ndarray, radial_degree: int, angular_order: int) -> np.ndarray:
    """Columns: for each radial power p=0..P, an angular Fourier block * logr^p."""
    x = coords[:, 0]
    y = coords[:, 1]
    r = np.clip(np.hypot(x, y), 1e-6, None)
    logr = np.log(r)
    theta = np.arctan2(y, x)
    base = _angular_block(theta, angular_order)
    cols: list[np.ndarray] = []
    for power in range(radial_degree + 1):
        scale = logr**power
        cols.extend(scale * col for col in base)
    return np.column_stack(cols)


def fit(coords: np.ndarray, cm: np.ndarray, radial_degree: int, angular_order: int) -> np.ndarray:
    matrix = design_matrix(coords, radial_degree, angular_order)
    target = np.log(np.clip(cm, 1e-6, None))
    weights, *_ = np.linalg.lstsq(matrix, target, rcond=None)
    return weights


def predict(coords: np.ndarray, weights: np.ndarray, radial_degree: int, angular_order: int) -> np.ndarray:
    return np.exp(design_matrix(coords, radial_degree, angular_order) @ weights)


def metrics(pred: np.ndarray, true: np.ndarray) -> dict[str, float]:
    err = pred - true
    abs_err = np.abs(err)
    nonzero = np.abs(true) > 1e-6
    return {
        "mae_cm": float(abs_err.mean()),
        "rmse_cm": float(np.sqrt((err * err).mean())),
        "max_abs_error_cm": float(abs_err.max()),
        "mape_percent": float((abs_err[nonzero] / np.abs(true[nonzero])).mean() * 100.0),
    }


def cross_validate(
    coords: np.ndarray, cm: np.ndarray, radial_degree: int, angular_order: int, folds: int, seed: int
) -> dict[str, float]:
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(cm))
    oof = np.empty_like(cm)
    for fold in range(folds):
        test_idx = idx[fold::folds]
        train_idx = np.setdiff1d(idx, test_idx, assume_unique=False)
        weights = fit(coords[train_idx], cm[train_idx], radial_degree, angular_order)
        oof[test_idx] = predict(coords[test_idx], weights, radial_degree, angular_order)
    return metrics(oof, cm)


def monotonic_fraction(coords: np.ndarray, weights: np.ndarray, radial_degree: int, angular_order: int) -> float:
    """Fraction of (r, theta) grid points where d(cm)/dr >= 0, within the sampled
    radius range. 1.0 means strictly monotonic increasing everywhere checked."""
    r = np.hypot(coords[:, 0], coords[:, 1])
    r_lo, r_hi = float(np.percentile(r, 1)), float(np.percentile(r, 99))
    r_grid = np.linspace(r_lo, r_hi, 80)
    th_grid = np.linspace(-np.pi, np.pi, 72, endpoint=False)
    rr, tt = np.meshgrid(r_grid, th_grid)
    pts = np.column_stack((rr.ravel() * np.cos(tt.ravel()), rr.ravel() * np.sin(tt.ravel())))
    pred = predict(pts, weights, radial_degree, angular_order).reshape(rr.shape)
    d_pred = np.diff(pred, axis=1)  # along increasing r
    return float((d_pred >= -1e-9).mean())


def save_plots(coords, cm, weights, radial_degree, angular_order, plots_dir: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("matplotlib not installed; skipping plots.")
        return
    plots_dir.mkdir(parents=True, exist_ok=True)
    pred = predict(coords, weights, radial_degree, angular_order)
    theta_deg = np.degrees(np.arctan2(coords[:, 1], coords[:, 0]))
    resid = pred - cm

    fig, ax = plt.subplots(figsize=(6.5, 6))
    ax.scatter(cm, pred, alpha=0.7, s=25, color="#2563eb")
    lim = max(cm.max(), pred.max()) * 1.04
    ax.plot([0, lim], [0, lim], "--", color="#111827")
    ax.set(xlabel="Measured (cm)", ylabel="Predicted (cm)",
           title=f"Parametric fit (radial deg {radial_degree}, angular order {angular_order})")
    fig.tight_layout(); fig.savefig(plots_dir / "parametric_pred_vs_measured.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].scatter(cm, resid, alpha=0.7, s=22, color="#7c3aed"); axes[0].axhline(0, color="#111827")
    axes[0].set(xlabel="Measured (cm)", ylabel="Pred - Meas (cm)", title="Residual vs distance (should be flat)")
    axes[1].scatter(theta_deg, resid, alpha=0.7, s=22, color="#0f766e"); axes[1].axhline(0, color="#111827")
    axes[1].set(xlabel="Bearing theta (deg)", ylabel="Pred - Meas (cm)", title="Residual vs angle (should be flat)")
    fig.tight_layout(); fig.savefig(plots_dir / "parametric_residuals.png", dpi=160); plt.close(fig)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="?", type=Path, default=DEFAULT_CSV)
    p.add_argument("--max-radial-degree", type=int, default=4, help="Highest log(r) polynomial degree to sweep (1..).")
    p.add_argument("--max-angular-order", type=int, default=3, help="Highest Fourier order in theta to sweep (0..).")
    p.add_argument("--cv-folds", type=int, default=5)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--save-coeffs", type=Path, default=TRAINING_DIR / "parametric_model.json")
    p.add_argument("--plots-dir", type=Path, default=TRAINING_DIR / "parametric_plots")
    p.add_argument("--skip-plots", action="store_true")
    p.add_argument("--eval-csv", type=Path, default=None, help="Optional held-out CSV (x,y,measured_cm) to report metrics on.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    coords, cm = read_csv(args.csv)
    print(f"Loaded {len(cm)} rows from {args.csv.name}")
    print(f"{args.cv_folds}-fold CV (seed {args.seed}); out-of-fold metrics = honest generalisation.\n")
    print(f"{'r_deg':>5} {'a_ord':>5} {'params':>6} {'CV MAE':>8} {'CV RMSE':>9} {'CV MAPE':>9} {'maxerr':>8} {'mono%':>7}")

    results = []
    for radial_degree in range(1, args.max_radial_degree + 1):
        for angular_order in range(args.max_angular_order + 1):
            n_params = (1 + 2 * angular_order) * (1 + radial_degree)
            if n_params >= len(cm):
                continue
            cv = cross_validate(coords, cm, radial_degree, angular_order, args.cv_folds, args.seed)
            full_w = fit(coords, cm, radial_degree, angular_order)
            mono = monotonic_fraction(coords, full_w, radial_degree, angular_order)
            results.append((radial_degree, angular_order, n_params, cv, full_w, mono))
            print(f"{radial_degree:>5} {angular_order:>5} {n_params:>6} {cv['mae_cm']:>7.3f} "
                  f"{cv['rmse_cm']:>8.3f} {cv['mape_percent']:>8.2f}% {cv['max_abs_error_cm']:>7.2f} "
                  f"{mono * 100:>6.1f}%")

    # Prefer fully-monotonic fits; among those pick best CV RMSE.
    monotonic = [r for r in results if r[5] >= 0.999]
    best = min(monotonic or results, key=lambda r: r[3]["rmse_cm"])
    radial_degree, angular_order, n_params, cv, weights, mono = best
    print(f"\nBest: radial_degree={radial_degree}, angular_order={angular_order} ({n_params} params)")
    print(f"  CV MAE {cv['mae_cm']:.3f} cm | RMSE {cv['rmse_cm']:.3f} cm | MAPE {cv['mape_percent']:.2f}% "
          f"| max {cv['max_abs_error_cm']:.1f} cm | monotonic over {mono * 100:.1f}% of grid")

    coeffs = {
        "model": "log_poly_fourier",
        "radial_degree": radial_degree,
        "angular_order": angular_order,
        "weights": weights.tolist(),
        "cv_folds": args.cv_folds,
        "cv_metrics": cv,
        "note": (
            "cm = exp(D @ weights). Build D per row: logr=log(hypot(x,y)), th=atan2(y,x); "
            "base=[1,cos th,sin th,...,cos(M th),sin(M th)] with M=angular_order; "
            "D = concat over p=0..radial_degree of (logr**p)*base."
        ),
    }
    args.save_coeffs.write_text(json.dumps(coeffs, indent=2))
    print(f"Saved coefficients -> {args.save_coeffs}")

    if args.eval_csv is not None:
        eval_coords, eval_cm = read_csv(args.eval_csv)
        eval_pred = predict(eval_coords, weights, radial_degree, angular_order)
        em = metrics(eval_pred, eval_cm)
        print(f"\nHeld-out evaluation on {args.eval_csv.name} ({len(eval_cm)} points):")
        print(f"  MAE {em['mae_cm']:.3f} cm | RMSE {em['rmse_cm']:.3f} cm | "
              f"MAPE {em['mape_percent']:.2f}% | max {em['max_abs_error_cm']:.2f} cm")
        print(f"  {'x':>8}{'y':>8}{'meas_cm':>9}{'pred_cm':>9}{'err_cm':>8}")
        for (ex, ey), m_cm, p_cm in zip(eval_coords, eval_cm, eval_pred):
            print(f"  {ex:>8.1f}{ey:>8.1f}{m_cm:>9.1f}{p_cm:>9.1f}{p_cm - m_cm:>8.1f}")

    if not args.skip_plots:
        save_plots(coords, cm, weights, radial_degree, angular_order, args.plots_dir)
        print(f"Saved plots -> {args.plots_dir}")


if __name__ == "__main__":
    main()
