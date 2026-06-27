#!/usr/bin/env python3
"""Plot a trained model's LEARNED distance surface and its SPATIAL ERROR map.

Unlike the data-review contour (which interpolates the raw samples), these plots
evaluate the actual fitted model on a dense grid, and color each training sample
by the model's residual there. Works torch-free for the parametric JSON and the
numpy-MLP JSON.

  python3 plot_model_surface.py --parametric parametric_model_cleaned.json \
      --data distance_training_data_cleaned.csv --out-dir data_review/model_surface
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent


def read_csv(path: Path):
    rows = []
    with path.open(newline="") as h:
        for i, r in enumerate(csv.reader(h)):
            if i == 0 or not r or not r[0].strip():
                continue
            rows.append([float(r[0]), float(r[1]), float(r[2])])
    a = np.asarray(rows, float)
    return a[:, :2], a[:, 2]


def parametric_predict(coords, model):
    P, M = int(model["radial_degree"]), int(model["angular_order"])
    w = np.asarray(model["weights"])
    x, y = coords[:, 0], coords[:, 1]
    r = np.clip(np.hypot(x, y), 1e-6, None); logr = np.log(r); th = np.arctan2(y, x)
    base = [np.ones_like(th)]
    for k in range(1, M + 1):
        base += [np.cos(k * th), np.sin(k * th)]
    D = np.column_stack([(logr ** p) * b for p in range(P + 1) for b in base])
    return np.exp(D @ w)


def numpy_mlp_predict(coords, model):
    x, y = coords[:, 0:1], coords[:, 1:2]; r = np.hypot(x, y)
    if model["feature_mode"] == "xy_radius":
        f = np.concatenate([x, y, r], 1)
    else:
        th = np.arctan2(y, x); f = np.concatenate([r, np.sin(th), np.cos(th)], 1)
    fm, fs = np.asarray(model["feature_mean"]), np.asarray(model["feature_std"])
    tm, ts = np.asarray(model["target_mean"]), np.asarray(model["target_std"])
    h = (f - fm) / fs
    W = [np.asarray(w) for w in model["weights"]]; b = [np.asarray(bb) for bb in model["biases"]]
    for i in range(len(W) - 1):
        h = np.maximum(h @ W[i] + b[i], 0.0)
    return ((h @ W[-1] + b[-1]) * ts + tm).reshape(-1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--parametric", type=Path)
    p.add_argument("--numpy-mlp", type=Path)
    p.add_argument("--data", type=Path, default=TRAINING_DIR / "distance_training_data_cleaned.csv")
    p.add_argument("--out-dir", type=Path, default=TRAINING_DIR / "data_review" / "model_surface")
    args = p.parse_args()

    if args.parametric:
        model = json.loads(args.parametric.read_text()); predict = parametric_predict; tag = f"parametric ({args.parametric.stem})"
    elif args.numpy_mlp:
        model = json.loads(args.numpy_mlp.read_text()); predict = numpy_mlp_predict; tag = f"MLP {model['feature_mode']}"
    else:
        raise SystemExit("pass --parametric or --numpy-mlp")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    coords, cm = read_csv(args.data)
    pred = predict(coords, model)
    resid = pred - cm
    args.out_dir.mkdir(parents=True, exist_ok=True)

    # dense grid over the sampled region, masked to the sampled radius range
    gx = np.linspace(coords[:, 0].min(), coords[:, 0].max(), 260)
    gy = np.linspace(coords[:, 1].min(), coords[:, 1].max(), 260)
    mx, my = np.meshgrid(gx, gy)
    grid = np.column_stack([mx.ravel(), my.ravel()])
    gz = predict(grid, model).reshape(mx.shape)
    rr = np.hypot(mx, my)
    gz = np.ma.masked_where((rr < np.hypot(coords[:, 0], coords[:, 1]).min()) |
                            (rr > np.hypot(coords[:, 0], coords[:, 1]).max()), gz)

    # 1) learned distance surface (model evaluated on the grid) + samples.
    # Clip the color scale to the measured range; the model can extrapolate to
    # extreme values in the unsampled corners (a known instability of the
    # log-poly-Fourier family) which would otherwise wash out the useful region.
    vmax = float(cm.max()) * 1.15
    levels = np.linspace(0, vmax, 24)
    fig, ax = plt.subplots(figsize=(7.4, 6))
    cf = ax.contourf(mx, my, np.clip(gz, 0, vmax), levels=levels, cmap="viridis", extend="max")
    ax.contour(mx, my, np.clip(gz, 0, vmax), levels=levels[::2], colors="white", linewidths=0.4, alpha=0.6)
    ax.scatter(coords[:, 0], coords[:, 1], c=cm, cmap="viridis", vmin=0, vmax=vmax,
               edgecolors="k", linewidths=0.3, s=18)
    ax.set(title=f"LEARNED distance surface — {tag}", xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal"); fig.colorbar(cf, ax=ax, label="predicted distance (cm)")
    fig.tight_layout(); fig.savefig(args.out_dir / "learned_distance_surface.png", dpi=160); plt.close(fig)

    # 2) spatial error map (signed residual at each sample)
    fig, ax = plt.subplots(figsize=(7.4, 6))
    vmax = float(np.percentile(np.abs(resid), 98)) or 1.0
    sc = ax.scatter(coords[:, 0], coords[:, 1], c=resid, cmap="coolwarm", vmin=-vmax, vmax=vmax, s=34, edgecolors="k", linewidths=0.2)
    ax.set(title=f"Spatial error (pred − meas) — {tag}", xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal"); fig.colorbar(sc, ax=ax, label="residual (cm)")
    fig.tight_layout(); fig.savefig(args.out_dir / "spatial_error_map.png", dpi=160); plt.close(fig)

    # 3) |error| map
    fig, ax = plt.subplots(figsize=(7.4, 6))
    sc = ax.scatter(coords[:, 0], coords[:, 1], c=np.abs(resid), cmap="magma", s=34, edgecolors="k", linewidths=0.2)
    ax.set(title=f"Absolute error — {tag}", xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal"); fig.colorbar(sc, ax=ax, label="|residual| (cm)")
    fig.tight_layout(); fig.savefig(args.out_dir / "abs_error_map.png", dpi=160); plt.close(fig)

    print(f"{tag}: MAE {np.abs(resid).mean():.2f}cm  max {np.abs(resid).max():.2f}cm")
    print(f"Wrote learned_distance_surface.png, spatial_error_map.png, abs_error_map.png -> {args.out_dir}")


if __name__ == "__main__":
    main()
