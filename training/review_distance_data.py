#!/usr/bin/env python3
"""Review the ball-distance training data and fix the center-offset shift.

Two things happened during collection:

  * The first ``N_OLD`` rows were recorded with the camera origin at
    ``OLD_CENTER`` = (308, 325).
  * Every later row used the current origin ``NEW_CENTER`` = (330, 318)
    (this is what ``thresholds.json`` stores today and what ``main.py`` uses
    at inference time).

Because the stored ``x``/``y`` are *centered* coordinates (``cx - origin_x``,
``cy - origin_y``), the two batches are in different frames. Mixing them puts
the same physical ball position at two different (dx, dy), which corrupts the
contour map and any r/theta model.

This script:
  1. Plots a contour map of the RAW (uncorrected) data so the seam is visible.
  2. Rewrites the old rows into the NEW frame and saves a fixed CSV.
  3. Plots the contour map of the FIXED data.
  4. Flags outliers via robust residuals of a smooth r/theta fit.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

TRAINING_DIR = Path(__file__).resolve().parent
SRC_CSV = TRAINING_DIR / "distance_training_data.csv"
FIXED_CSV = TRAINING_DIR / "distance_training_data_fixed_330_318.csv"
NEWONLY_CSV = TRAINING_DIR / "distance_training_data_newframe_only.csv"
OUT_DIR = TRAINING_DIR / "data_review"
PLOTS = OUT_DIR / "plots"

# Number of leading data rows recorded with the OLD origin.
N_OLD = 337
OLD_CENTER = (308.0, 325.0)
NEW_CENTER = (330.0, 318.0)
# Shift to add to old-frame (dx, dy) to express them in the new frame.
DX_SHIFT = OLD_CENTER[0] - NEW_CENTER[0]   # -22
DY_SHIFT = OLD_CENTER[1] - NEW_CENTER[1]   # +7


def read_rows(path: Path):
    rows = []
    with path.open(newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        for r in reader:
            if not r or all(not c.strip() for c in r):
                continue
            rows.append([float(r[0]), float(r[1]), float(r[2])])
    return header, np.asarray(rows, dtype=np.float64)


def apply_fix(data: np.ndarray) -> np.ndarray:
    fixed = data.copy()
    fixed[:N_OLD, 0] += DX_SHIFT
    fixed[:N_OLD, 1] += DY_SHIFT
    return fixed


def write_fixed(header, fixed: np.ndarray, path: Path) -> None:
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for x, y, cm in fixed:
            # keep original-ish formatting: ints for coords if whole
            xs = int(x) if x == int(x) else round(x, 2)
            ys = int(y) if y == int(y) else round(y, 2)
            w.writerow([xs, ys, cm])


def per_batch_contours(fixed: np.ndarray, path: Path):
    """Side-by-side contour of each batch in its own frame.

    If the two batches shared geometry, matching pixel locations would show
    matching colors. They don't -- the new batch is much 'hotter' (larger cm)
    at the same radius, which is why merging them corrupts the map.
    """
    batches = [("OLD batch rows 1-%d (origin 308,325)" % N_OLD, fixed[:N_OLD]),
               ("NEW batch rows %d+ (origin 330,318) = deployment" % (N_OLD + 1),
                fixed[N_OLD:])]
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for ax, (name, d) in zip(axes, batches):
        dx, dy, cm = d[:, 0], d[:, 1], d[:, 2]
        pad = 15
        gx = np.linspace(dx.min() - pad, dx.max() + pad, 250)
        gy = np.linspace(dy.min() - pad, dy.max() + pad, 250)
        GX, GY = np.meshgrid(gx, gy)
        grid = griddata((dx, dy), cm, (GX, GY), method="linear")
        cf = ax.contourf(GX, GY, grid, levels=np.linspace(0, 210, 22), cmap="viridis", extend="max")
        ax.scatter(dx, dy, s=7, c="white", edgecolors="k", linewidths=0.3, alpha=0.6)
        fig.colorbar(cf, ax=ax, label="cm")
        ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-280, 280); ax.set_ylim(-280, 280)
        ax.set_xlabel("dx"); ax.set_ylabel("dy"); ax.set_title(name, fontsize=10)
    fig.suptitle("Same color = same distance. The two batches DISAGREE at matching pixels "
                 "(different camera geometry).", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def contour_plot(data: np.ndarray, title: str, path: Path, batch_split: int | None = None):
    dx, dy, cm = data[:, 0], data[:, 1], data[:, 2]

    # Interpolate scattered samples onto a regular grid for filled contours.
    pad = 20
    gx = np.linspace(dx.min() - pad, dx.max() + pad, 300)
    gy = np.linspace(dy.min() - pad, dy.max() + pad, 300)
    GX, GY = np.meshgrid(gx, gy)
    grid = griddata((dx, dy), cm, (GX, GY), method="linear")

    fig, ax = plt.subplots(figsize=(10, 8))
    cf = ax.contourf(GX, GY, grid, levels=25, cmap="viridis")
    cs = ax.contour(GX, GY, grid, levels=12, colors="k", linewidths=0.4, alpha=0.5)
    ax.clabel(cs, inline=True, fontsize=6, fmt="%.0f")
    fig.colorbar(cf, ax=ax, label="measured distance (cm)")

    if batch_split is not None:
        ax.scatter(dx[:batch_split], dy[:batch_split], s=10, c="red",
                   edgecolors="white", linewidths=0.3, label=f"old frame (rows 1-{batch_split})")
        ax.scatter(dx[batch_split:], dy[batch_split:], s=10, c="deepskyblue",
                   edgecolors="white", linewidths=0.3, label=f"new frame (rows {batch_split+1}+)")
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.scatter(dx, dy, s=8, c="white", edgecolors="k", linewidths=0.3, alpha=0.7)

    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("dx (centered px)")
    ax.set_ylabel("dy (centered px)")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def radius_plot(raw: np.ndarray, fixed: np.ndarray, path: Path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharey=True)
    for ax, data, name in ((axes[0], raw, "RAW"), (axes[1], fixed, "FIXED")):
        r = np.hypot(data[:, 0], data[:, 1])
        cm = data[:, 2]
        ax.scatter(r[:N_OLD], cm[:N_OLD], s=12, c="red", alpha=0.6,
                   label="old frame rows")
        ax.scatter(r[N_OLD:], cm[N_OLD:], s=12, c="deepskyblue", alpha=0.6,
                   label="new frame rows")
        ax.set_xlabel("radius r = hypot(dx, dy)  [px]")
        ax.set_title(f"{name}: distance vs radius")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("measured distance (cm)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fit_logr_model(data: np.ndarray, P=4, M=2):
    """Closed-form least squares: log(cm) = sum_p logr^p * Fourier_M(theta)."""
    dx, dy, cm = data[:, 0], data[:, 1], data[:, 2]
    r = np.hypot(dx, dy)
    r = np.clip(r, 1e-6, None)
    logr = np.log(r)
    th = np.arctan2(dy, dx)
    ang = [np.ones_like(th)]
    for k in range(1, M + 1):
        ang.append(np.cos(k * th))
        ang.append(np.sin(k * th))
    cols = []
    for p in range(P + 1):
        for a in ang:
            cols.append((logr ** p) * a)
    A = np.vstack(cols).T
    y = np.log(np.clip(cm, 1e-6, None))
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = np.exp(A @ coef)
    return pred


def outlier_report(data: np.ndarray, path: Path):
    pred = fit_logr_model(data)
    cm = data[:, 2]
    r = np.hypot(data[:, 0], data[:, 1])
    th = np.degrees(np.arctan2(data[:, 1], data[:, 0]))
    # Robust residual in log space (scale-free, percentage-like).
    log_res = np.log(np.clip(cm, 1e-6, None)) - np.log(np.clip(pred, 1e-6, None))
    med = np.median(log_res)
    mad = np.median(np.abs(log_res - med)) + 1e-9
    robust_z = 0.6745 * (log_res - med) / mad
    pct_err = 100.0 * (cm - pred) / pred

    order = np.argsort(-np.abs(robust_z))
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["row_1based", "dx", "dy", "r_px", "theta_deg",
                    "measured_cm", "model_cm", "pct_err", "robust_z"])
        for i in order:
            w.writerow([i + 1, data[i, 0], data[i, 1], round(r[i], 1),
                        round(th[i], 1), cm[i], round(pred[i], 2),
                        round(pct_err[i], 1), round(robust_z[i], 2)])

    flagged = np.where(np.abs(robust_z) > 3.5)[0]
    return flagged, robust_z, pred


def outlier_plot(data: np.ndarray, flagged, path: Path):
    dx, dy, cm = data[:, 0], data[:, 1], data[:, 2]
    r = np.hypot(dx, dy)
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    sc = axes[0].scatter(dx, dy, c=cm, s=14, cmap="viridis")
    axes[0].scatter(dx[flagged], dy[flagged], s=120, facecolors="none",
                    edgecolors="red", linewidths=1.6, label="outlier")
    fig.colorbar(sc, ax=axes[0], label="cm")
    axes[0].set_xlabel("dx"); axes[0].set_ylabel("dy")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_title("Flagged outliers (xy)")
    axes[0].legend(fontsize=8)

    axes[1].scatter(r, cm, s=14, c="steelblue", alpha=0.6)
    axes[1].scatter(r[flagged], cm[flagged], s=80, facecolors="none",
                    edgecolors="red", linewidths=1.6, label="outlier")
    for i in flagged:
        axes[1].annotate(str(i + 1), (r[i], cm[i]), fontsize=7, color="red")
    axes[1].set_xlabel("radius (px)"); axes[1].set_ylabel("cm")
    axes[1].set_title("Flagged outliers (r vs cm)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)

    header, raw = read_rows(SRC_CSV)
    print(f"Loaded {len(raw)} data rows from {SRC_CSV.name}")
    print(f"Old-frame rows: 1..{N_OLD}  shift (dx{DX_SHIFT:+}, dy{DY_SHIFT:+})")

    fixed = apply_fix(raw)
    write_fixed(header, fixed, FIXED_CSV)
    print(f"Wrote fixed CSV -> {FIXED_CSV.name}")

    # Deployment-consistent set: ONLY the new batch (matches thresholds.json).
    write_fixed(header, fixed[N_OLD:], NEWONLY_CSV)
    print(f"Wrote new-frame-only CSV -> {NEWONLY_CSV.name} ({len(fixed)-N_OLD} rows)")

    contour_plot(raw, "RAW data (mixed frames) - distance contour",
                 PLOTS / "01_contour_raw.png", batch_split=N_OLD)
    contour_plot(fixed, "FIXED data (all in 330,318 frame) - distance contour",
                 PLOTS / "02_contour_fixed.png", batch_split=N_OLD)
    radius_plot(raw, fixed, PLOTS / "03_radius_vs_distance.png")
    per_batch_contours(fixed, PLOTS / "05_per_batch_contours.png")

    # Outlier report on the DEPLOYMENT set only (new batch). The old batch is a
    # different camera geometry, so flagging across both is meaningless.
    new = fixed[N_OLD:]
    flagged, rz, pred = outlier_report(new, OUT_DIR / "outlier_report_newbatch.csv")
    outlier_plot(new, flagged, PLOTS / "04_outliers_newbatch.png")
    print(f"\n[NEW batch] Outliers (|robust_z|>3.5): {len(flagged)} rows")
    cm = new[:, 2]
    fixed = new  # for the print loop below
    for i in flagged[np.argsort(-np.abs(rz[flagged]))]:
        print(f"  row {i+1+N_OLD:>3}: dx={fixed[i,0]:.0f} dy={fixed[i,1]:.0f} "
              f"r={np.hypot(fixed[i,0],fixed[i,1]):.0f} "
              f"measured={cm[i]:.1f} model={pred[i]:.1f} z={rz[i]:+.1f}")

    print(f"\nPlots in {PLOTS}")


if __name__ == "__main__":
    main()
