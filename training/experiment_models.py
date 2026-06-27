#!/usr/bin/env python3
"""Compare an MLP vs a closed-form parametric regression for f(r, theta) -> cm.

Datasets (all in the current 330,318 frame):
  A. NEW data only (rows 338+), outliers INCLUDED
  B. NEW data only, outliers REMOVED (robust-z > 3.5 of the parametric fit)

Both models use the SAME polar inputs derived from (dx, dy):
  * MLP        : features [r, sin(theta), cos(theta)], numpy MLP (32,32) ReLU
                 (identical to train_distance_model_numpy.py, feature_mode=polar)
  * regression : log(cm) = sum_{p=0..P} logr^p * Fourier_M(theta), closed-form LS

Reports honest 5-fold out-of-fold metrics (the number to compare on) plus the
all-data fit for reference. Lower is better for MAE/RMSE/MAPE.
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from train_distance_model_numpy import (
    build_features, metrics, cross_validate,
)

TRAINING_DIR = Path(__file__).resolve().parent
NEW_CSV = TRAINING_DIR / "distance_training_data_newframe_only.csv"
FULL_CSV = TRAINING_DIR / "distance_training_data_fixed_330_318.csv"

SEED = 7
FOLDS = 5
MLP_EPOCHS = 300
MLP_LR = 0.01
MLP_BATCH = 16
P_DEG = 4      # radial polynomial degree in log(r)
M_ORD = 3      # angular Fourier order
Z_THRESH = 3.5


# ---------- parametric regression: log(cm) = poly(logr) x Fourier(theta) ------
def _design(coords: np.ndarray, P: int, M: int) -> np.ndarray:
    r = np.clip(np.hypot(coords[:, 0], coords[:, 1]), 1e-6, None)
    logr = np.log(r)
    th = np.arctan2(coords[:, 1], coords[:, 0])
    ang = [np.ones_like(th)]
    for k in range(1, M + 1):
        ang += [np.cos(k * th), np.sin(k * th)]
    cols = [(logr ** p) * a for p in range(P + 1) for a in ang]
    return np.vstack(cols).T


def param_fit_predict(coords_tr, cm_tr, coords_te, P=P_DEG, M=M_ORD):
    A = _design(coords_tr, P, M)
    y = np.log(np.clip(cm_tr.reshape(-1), 1e-6, None))
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    return np.exp(_design(coords_te, P, M) @ coef).reshape(-1, 1)


def param_cv(coords, cm, folds=FOLDS, seed=SEED):
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(cm))
    oof = np.zeros_like(cm)
    for f in range(folds):
        te = idx[f::folds]
        tr = np.setdiff1d(idx, te)
        oof[te] = param_fit_predict(coords[tr], cm[tr], coords[te])
    return metrics(oof, cm), oof


# ---------- io / outliers -----------------------------------------------------
def read_csv(path: Path):
    rows = []
    with path.open(newline="") as fh:
        for i, row in enumerate(csv.reader(fh)):
            if i == 0 or not row or not row[0].strip():
                continue
            rows.append([float(row[0]), float(row[1]), float(row[2])])
    d = np.asarray(rows, dtype=np.float64)
    return d[:, :2], d[:, 2:3]


def flag_outliers(coords, cm, z_thresh=Z_THRESH):
    pred = param_fit_predict(coords, cm, coords).reshape(-1)
    lres = np.log(np.clip(cm.reshape(-1), 1e-6, None)) - np.log(np.clip(pred, 1e-6, None))
    med = np.median(lres)
    mad = np.median(np.abs(lres - med)) + 1e-9
    z = 0.6745 * (lres - med) / mad
    return np.abs(z) > z_thresh, z


def line(label, m):
    print(f"    {label:<12} MAE {m['mae_cm']:6.2f}  RMSE {m['rmse_cm']:6.2f}  "
          f"MAPE {m['mape_percent']:5.2f}%  maxerr {m['max_abs_error_cm']:6.2f}")


def run(name, coords, cm):
    print(f"\n=== {name}  (n={len(cm)}) ===")
    # MLP, polar features
    mlp_cv, _ = cross_validate(coords, cm, "polar", MLP_EPOCHS, MLP_LR, MLP_BATCH, SEED, FOLDS)
    # parametric regression
    par_cv, _ = param_cv(coords, cm)
    print("  5-fold out-of-fold (honest generalization):")
    line("MLP", mlp_cv)
    line("regression", par_cv)


def main():
    # ---- NEW data only (deployment-consistent batch) --------------------
    coords, cm = read_csv(NEW_CSV)
    print(f"Loaded {len(cm)} NEW-frame rows from {NEW_CSV.name}")
    run("A. NEW only, outliers INCLUDED", coords, cm)
    out_mask, _ = flag_outliers(coords, cm)
    keep = ~out_mask
    print(f"\n  -> flagged {out_mask.sum()} outliers, {keep.sum()} kept")
    run("B. NEW only, outliers REMOVED", coords[keep], cm[keep])

    # ---- FULL merged data (old+new, fixed to 330,318 frame) -------------
    # NOTE: the two batches are different camera geometry, so this set mixes
    # incompatible (pixel -> cm) mappings; shown only for comparison.
    fcoords, fcm = read_csv(FULL_CSV)
    print(f"\n\nLoaded {len(fcm)} FULL merged rows from {FULL_CSV.name} "
          f"(WARNING: mixed geometry)")
    run("C. FULL merged, outliers INCLUDED", fcoords, fcm)
    fmask, _ = flag_outliers(fcoords, fcm)
    fkeep = ~fmask
    print(f"\n  -> flagged {fmask.sum()} outliers, {fkeep.sum()} kept")
    run("D. FULL merged, outliers REMOVED", fcoords[fkeep], fcm[fkeep])


if __name__ == "__main__":
    main()
