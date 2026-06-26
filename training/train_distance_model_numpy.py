#!/usr/bin/env python3
"""Pure-numpy training of the ball distance MLP -- a torch-free reference.

This mirrors ``train_distance_model.py`` exactly so its results are directly
comparable, but needs no torch (useful on the Pi before torch is installed):

  * architecture  : 3 inputs -> Linear(.,32) ReLU -> Linear(32,32) ReLU -> Linear(32,1)
  * features      : ``xy_radius`` -> [x, y, r]  or  ``polar`` -> [r, sin th, cos th]
  * normalization : standardize features and target (z-score), same as torch
  * loss          : far-distance-weighted MSE in normalized target space
                    weight = 1 + 1.5 * (cm / max_cm)^2
  * optimizer     : Adam (lr, betas 0.9/0.999, eps 1e-8) with L2 weight_decay 1e-4
  * init          : Linear's Kaiming-uniform default, U(-1/sqrt(fan_in), +...)

It reports honest K-fold out-of-fold metrics and a final all-data fit, evaluates
on a held-out CSV, and saves the weights to JSON for torch-free inference.

When torch is available, ``train_distance_model.py`` is still the deployment path
(it writes .pt checkpoints + ONNX export); this script is for analysis/parity.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent


# --- features (kept identical to distance_model.build_input_features) -------
def build_features(coords: np.ndarray, mode: str) -> np.ndarray:
    x = coords[:, 0:1]
    y = coords[:, 1:2]
    r = np.hypot(x, y)
    if mode == "xy_radius":
        return np.concatenate([x, y, r], axis=1)
    if mode == "polar":
        th = np.arctan2(y, x)
        return np.concatenate([r, np.sin(th), np.cos(th)], axis=1)
    raise ValueError(f"unknown feature mode {mode!r}")


def read_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(newline="") as handle:
        for i, row in enumerate(csv.reader(handle)):
            if i == 0 or not row or not row[0].strip():
                continue
            rows.append([float(row[0]), float(row[1]), float(row[2])])
    data = np.asarray(rows, dtype=np.float64)
    return data[:, :2], data[:, 2:3]


def metrics(pred: np.ndarray, true: np.ndarray) -> dict[str, float]:
    err = pred.reshape(-1) - true.reshape(-1)
    abs_err = np.abs(err)
    t = true.reshape(-1)
    nz = np.abs(t) > 1e-6
    return {
        "mae_cm": float(abs_err.mean()),
        "rmse_cm": float(np.sqrt((err * err).mean())),
        "mape_percent": float((abs_err[nz] / np.abs(t[nz])).mean() * 100.0),
        "max_abs_error_cm": float(abs_err.max()),
    }


# --- the MLP ----------------------------------------------------------------
class NumpyMLP:
    def __init__(self, input_dim: int, hidden=(32, 32), seed: int = 7):
        rng = np.random.default_rng(seed)
        sizes = [input_dim, *hidden, 1]
        self.W: list[np.ndarray] = []
        self.b: list[np.ndarray] = []
        for fan_in, fan_out in zip(sizes[:-1], sizes[1:]):
            bound = 1.0 / np.sqrt(fan_in)  # torch nn.Linear default init
            self.W.append(rng.uniform(-bound, bound, size=(fan_in, fan_out)))
            self.b.append(rng.uniform(-bound, bound, size=(fan_out,)))

    def forward(self, x: np.ndarray):
        acts = [x]
        h = x
        for i in range(len(self.W) - 1):
            z = h @ self.W[i] + self.b[i]
            h = np.maximum(z, 0.0)  # ReLU
            acts.append(h)
        out = h @ self.W[-1] + self.b[-1]
        acts.append(out)
        return out, acts

    def predict(self, x: np.ndarray) -> np.ndarray:
        return self.forward(x)[0]


def adam_train(model: NumpyMLP, x: np.ndarray, y_norm: np.ndarray,
               sample_weight: np.ndarray, epochs: int, lr: float,
               batch_size: int, weight_decay: float, seed: int) -> None:
    """Minibatch Adam on weighted MSE in normalized target space."""
    rng = np.random.default_rng(seed + 1)
    mW = [np.zeros_like(w) for w in model.W]
    vW = [np.zeros_like(w) for w in model.W]
    mb = [np.zeros_like(b) for b in model.b]
    vb = [np.zeros_like(b) for b in model.b]
    b1, b2, eps = 0.9, 0.999, 1e-8
    t = 0
    n = len(x)
    for _ in range(epochs):
        order = rng.permutation(n)
        for start in range(0, n, batch_size):
            idx = order[start:start + batch_size]
            xb, yb, wb = x[idx], y_norm[idx], sample_weight[idx]
            out, acts = model.forward(xb)
            # dL/dout for L = mean( w * (out - y)^2 )
            d = (2.0 * wb * (out - yb)) / len(idx)
            gW: list[np.ndarray] = [None] * len(model.W)
            gb: list[np.ndarray] = [None] * len(model.b)
            grad = d
            for layer in range(len(model.W) - 1, -1, -1):
                a_in = acts[layer]
                gW[layer] = a_in.T @ grad + weight_decay * model.W[layer]
                gb[layer] = grad.sum(axis=0)
                if layer > 0:
                    grad = grad @ model.W[layer].T
                    grad = grad * (acts[layer] > 0.0)  # back through ReLU
            t += 1
            for layer in range(len(model.W)):
                for g, m, v, param in ((gW[layer], mW[layer], vW[layer], model.W[layer]),
                                       (gb[layer], mb[layer], vb[layer], model.b[layer])):
                    m[...] = b1 * m + (1 - b1) * g
                    v[...] = b2 * v + (1 - b2) * (g * g)
                    mhat = m / (1 - b1 ** t)
                    vhat = v / (1 - b2 ** t)
                    param -= lr * mhat / (np.sqrt(vhat) + eps)


def normalize_fit(features: np.ndarray, target_cm: np.ndarray):
    fm = features.mean(0, keepdims=True)
    fs = features.std(0, keepdims=True); fs[fs < 1e-6] = 1.0
    tm = target_cm.mean(0, keepdims=True)
    ts = target_cm.std(0, keepdims=True); ts[ts < 1e-6] = 1.0
    return fm, fs, tm, ts


def fit_predict(coords_tr, cm_tr, coords_te, mode, epochs, lr, batch_size, seed):
    feats_tr = build_features(coords_tr, mode)
    fm, fs, tm, ts = normalize_fit(feats_tr, cm_tr)
    xn = (feats_tr - fm) / fs
    yn = (cm_tr - tm) / ts
    max_cm = float(cm_tr.max())
    frac = np.clip(cm_tr / max_cm, 0, 1)
    w = 1.0 + 1.5 * frac ** 2  # far-distance weighting
    model = NumpyMLP(feats_tr.shape[1], seed=seed)
    adam_train(model, xn, yn, w, epochs, lr, min(batch_size, len(xn)), 1e-4, seed)
    feats_te = build_features(coords_te, mode)
    pred = model.predict((feats_te - fm) / fs) * ts + tm
    return pred, model, (fm, fs, tm, ts)


def cross_validate(coords, cm, mode, epochs, lr, batch_size, seed, folds):
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(cm))
    oof = np.zeros_like(cm)
    for fold in range(folds):
        te = idx[fold::folds]
        tr = np.setdiff1d(idx, te, assume_unique=False)
        pred, *_ = fit_predict(coords[tr], cm[tr], coords[te], mode, epochs, lr, batch_size, seed)
        oof[te] = pred
    return metrics(oof, cm), oof


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("train_csv", nargs="?", type=Path, default=TRAINING_DIR / "distance_training_data_cleaned.csv")
    p.add_argument("--feature-mode", choices=("xy_radius", "polar"), default="xy_radius")
    p.add_argument("--max-epochs", type=int, default=300)
    p.add_argument("--learning-rate", type=float, default=0.01)
    p.add_argument("--batch-size", type=int, default=16)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--cv-folds", type=int, default=5)
    p.add_argument("--eval-csv", type=Path, default=TRAINING_DIR / "distance_eval_data.csv")
    p.add_argument("--save-json", type=Path, default=None)
    args = p.parse_args()

    coords, cm = read_csv(args.train_csv)
    print(f"[numpy MLP] {args.feature_mode} | {len(cm)} rows from {args.train_csv.name} "
          f"| {args.max_epochs} epochs, lr {args.learning_rate}")

    if args.cv_folds > 0:
        cv, _ = cross_validate(coords, cm, args.feature_mode, args.max_epochs,
                               args.learning_rate, args.batch_size, args.seed, args.cv_folds)
        print(f"  CV (out-of-fold): MAE {cv['mae_cm']:.3f}  RMSE {cv['rmse_cm']:.3f}  "
              f"MAPE {cv['mape_percent']:.2f}%  max {cv['max_abs_error_cm']:.2f}")

    # final all-data fit
    pred_all, model, (fm, fs, tm, ts) = fit_predict(coords, cm, coords, args.feature_mode,
                                                     args.max_epochs, args.learning_rate,
                                                     args.batch_size, args.seed)
    fit = metrics(pred_all, cm)
    print(f"  all-data fit:     MAE {fit['mae_cm']:.3f}  RMSE {fit['rmse_cm']:.3f}  "
          f"MAPE {fit['mape_percent']:.2f}%  max {fit['max_abs_error_cm']:.2f}")

    if args.eval_csv and args.eval_csv.exists():
        ec, ecm = read_csv(args.eval_csv)
        feats = build_features(ec, args.feature_mode)
        epred = model.predict((feats - fm) / fs) * ts + tm
        em = metrics(epred, ecm)
        print(f"  held-out eval:    MAE {em['mae_cm']:.3f}  RMSE {em['rmse_cm']:.3f}  "
              f"MAPE {em['mape_percent']:.2f}%  max {em['max_abs_error_cm']:.2f}")

    if args.save_json:
        payload = {
            "model": "numpy_mlp",
            "feature_mode": args.feature_mode,
            "hidden_sizes": [32, 32],
            "weights": [w.tolist() for w in model.W],
            "biases": [b.tolist() for b in model.b],
            "feature_mean": fm.tolist(), "feature_std": fs.tolist(),
            "target_mean": tm.tolist(), "target_std": ts.tolist(),
        }
        args.save_json.write_text(json.dumps(payload))
        print(f"  saved weights -> {args.save_json}")


if __name__ == "__main__":
    main()
