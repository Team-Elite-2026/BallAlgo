#!/usr/bin/env python3
"""Thorough review + outlier removal for the ball distance-calibration data.

The training data (``training/distance_training_data.csv``: ``x,y,measured_cm``)
is collected as **constant-distance arcs**: the ball is held at a fixed physical
distance from the robot and swept around it in a circle, recording the centred
pixel position ``(x, y)`` at many bearings. Each distinct ``measured_cm`` value
is therefore one arc, and -- because the camera looks at an omnidirectional
(catadioptric) mirror -- every point on a given arc should sit at a nearly
constant pixel radius ``r = hypot(x, y)``.

That structure is what makes principled cleaning possible:

1.  **Optical-centre diagnostic.** A constant physical distance maps to a circle
    in the image. If the assumed centre (``thresholds.json`` offsets, here the
    origin of the centred ``x, y``) is wrong by ``(dx, dy)``, every arc's radius
    picks up a coherent ``r(theta) ~ R + dx*cos(theta) + dy*sin(theta)`` wobble.
    We estimate the centre offset that minimises within-arc radius variance and
    report it -- this is *calibration* information, not an outlier, so we never
    silently apply it (it would have to be matched by ``thresholds.json`` at
    inference time).

2.  **Per-arc angular-residual outlier removal (primary).** Within each arc we
    fit the smooth first-order model ``r(theta) = R + a*cos(theta) + b*sin(theta)``
    robustly (iterative sigma-clipping). That absorbs the coherent decentring
    wobble; whatever is *left* is genuine scatter. A point whose radius departs
    from its own arc's smooth curve by more than ``ARC_Z`` robust sigmas (and an
    absolute floor) is a real bad detection / mislabel and is dropped.

3.  **Global model-residual cross-check (secondary).** We also fit the
    deployment model family ``log(cm) = poly_P(log r) x Fourier_M(theta)`` on the
    surviving inliers and flag any remaining points with a huge relative
    residual. This catches mislabelled *distances* that happen to land on a
    plausible radius.

4.  **Duplicate (x, y) handling.** Identical pixel coordinates with inconsistent
    ``measured_cm`` are collapsed to their median when the spread is small and
    dropped entirely when the spread is large (ambiguous ground truth).

Outputs (all written next to this script unless noted):

* ``../distance_training_data_cleaned.csv`` -- cleaned data, same coordinate
  frame as the input, a drop-in for ``train_distance_model.py`` /
  ``fit_parametric_model.py``.
* ``outlier_report.csv`` -- every dropped row with the reason.
* ``arc_summary.csv``    -- per-arc radius spread before/after detrending.
* ``plots/`` -- the contour map, gradient-magnitude map, radius-vs-distance
  structure, flagged-outlier map, and before/after comparison.

Pure numpy + matplotlib (no scipy / torch needed).
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

DATA_REVIEW_DIR = Path(__file__).resolve().parent
TRAINING_DIR = DATA_REVIEW_DIR.parent
DEFAULT_INPUT = TRAINING_DIR / "distance_training_data.csv"
DEFAULT_OUTPUT = TRAINING_DIR / "distance_training_data_cleaned.csv"

# ---- tunables (documented in the module docstring) -------------------------
DUP_SPREAD_TOL_CM = 5.0   # duplicate (x,y): collapse if cm spread <= this, else drop
ARC_MIN_POINTS = 5        # arcs smaller than this are judged by radius-MAD only
ARC_Z = 3.5               # robust-z threshold for per-arc angular residual
ARC_ABS_FLOOR_PX = 6.0    # also require this many px of deviation to drop
SMALL_ARC_Z = 4.0
SMALL_ARC_ABS_FLOOR_PX = 15.0
GLOBAL_RADIAL_DEGREE = 3  # log(r) polynomial degree for the cross-check fit
GLOBAL_ANGULAR_ORDER = 3  # Fourier order in theta for the cross-check fit
GLOBAL_REL_Z = 5.0        # robust-z on relative residual for the global check


# ---------------------------------------------------------------------------
# IO
# ---------------------------------------------------------------------------
def read_csv(path: Path) -> tuple[np.ndarray, list[int]]:
    """Return an (N,3) float array of x,y,cm and the 1-based source line of each row."""
    rows: list[list[float]] = []
    source_lines: list[int] = []
    with path.open(newline="") as handle:
        for line_no, row in enumerate(csv.reader(handle), start=1):
            if not row or all(not c.strip() for c in row):
                continue
            try:
                values = [float(row[0]), float(row[1]), float(row[2])]
            except (ValueError, IndexError):
                if line_no == 1:
                    continue  # header
                raise
            rows.append(values)
            source_lines.append(line_no)
    data = np.asarray(rows, dtype=np.float64)
    return data, source_lines


# ---------------------------------------------------------------------------
# Model helpers
# ---------------------------------------------------------------------------
def design_matrix(x: np.ndarray, y: np.ndarray, radial_degree: int, angular_order: int) -> np.ndarray:
    """log(cm) = sum_p (log r)^p * [1, cos k th, sin k th ...]. Same family as the
    deployed parametric model so the cross-check matches production."""
    r = np.clip(np.hypot(x, y), 1e-6, None)
    logr = np.log(r)
    theta = np.arctan2(y, x)
    base = [np.ones_like(theta)]
    for k in range(1, angular_order + 1):
        base += [np.cos(k * theta), np.sin(k * theta)]
    return np.column_stack([(logr**p) * b for p in range(radial_degree + 1) for b in base])


def robust_center_offset(x: np.ndarray, y: np.ndarray, arc_labels: np.ndarray) -> tuple[float, float, float, float]:
    """Grid-search the centre correction (dx,dy) that minimises within-arc radius
    variance. Returns (dx, dy, rms_spread_before, rms_spread_after)."""
    arcs = [np.where(arc_labels == v)[0] for v in np.unique(arc_labels)]
    arcs = [a for a in arcs if len(a) >= 4]

    def spread(dx: float, dy: float) -> float:
        r = np.hypot(x - dx, y - dy)
        total = 0.0
        count = 0
        for a in arcs:
            ra = r[a]
            total += np.sum((ra - np.median(ra)) ** 2)
            count += len(a)
        return total / max(count, 1)

    base = spread(0.0, 0.0)
    best = (0.0, 0.0, base)
    # coarse then fine search
    for step, lo, hi in ((2.0, -30, 31), (0.5, -4, 5)):
        cx, cy, _ = best
        for dx in np.arange(cx + lo, cx + hi, step):
            for dy in np.arange(cy + lo, cy + hi, step):
                value = spread(dx, dy)
                if value < best[2]:
                    best = (float(dx), float(dy), value)
    return best[0], best[1], float(np.sqrt(base)), float(np.sqrt(best[2]))


# ---------------------------------------------------------------------------
# Outlier detection
# ---------------------------------------------------------------------------
def collapse_duplicates(data: np.ndarray, source_lines: list[int]) -> tuple[np.ndarray, list[int], list[dict]]:
    """Collapse identical (x,y). Small cm spread -> median; large spread -> drop."""
    keys: dict[tuple[float, float], list[int]] = {}
    for i, (x, y, _cm) in enumerate(data):
        keys.setdefault((x, y), []).append(i)

    kept_rows: list[list[float]] = []
    kept_lines: list[int] = []
    dropped: list[dict] = []
    for (x, y), idxs in keys.items():
        cms = data[idxs, 2]
        spread = float(cms.max() - cms.min())
        if len(idxs) == 1:
            kept_rows.append([x, y, cms[0]])
            kept_lines.append(source_lines[idxs[0]])
        elif spread <= DUP_SPREAD_TOL_CM:
            kept_rows.append([x, y, float(np.median(cms))])
            kept_lines.append(source_lines[idxs[0]])
        else:
            for i in idxs:
                dropped.append({
                    "source_line": source_lines[i], "x": x, "y": y,
                    "measured_cm": data[i, 2], "radius_px": float(np.hypot(x, y)),
                    "reason": f"duplicate_xy_spread_{spread:.1f}cm",
                })
    out = np.asarray(kept_rows, dtype=np.float64)
    # keep a stable order by original line
    order = np.argsort(kept_lines)
    return out[order], [kept_lines[i] for i in order], dropped


def per_arc_outliers(data: np.ndarray, source_lines: list[int]) -> tuple[np.ndarray, list[dict]]:
    """Flag points that deviate from their arc's smooth r(theta) curve."""
    x, y, cm = data[:, 0], data[:, 1], data[:, 2]
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    labels = np.round(cm, 2)
    flagged = np.zeros(len(data), dtype=bool)
    reasons: list[dict] = []

    for value in np.unique(labels):
        a = np.where(labels == value)[0]
        if len(a) < ARC_MIN_POINTS:
            # too few points to fit r(theta); fall back to a radius MAD test
            ra = r[a]
            med = np.median(ra)
            mad = np.median(np.abs(ra - med)) * 1.4826 or 1.0
            for i in a:
                if abs(r[i] - med) > max(SMALL_ARC_Z * mad, SMALL_ARC_ABS_FLOOR_PX):
                    flagged[i] = True
                    reasons.append({"source_line": source_lines[i], "x": x[i], "y": y[i],
                                    "measured_cm": cm[i], "radius_px": r[i],
                                    "reason": f"small_arc_radius_outlier(arc={value})"})
            continue

        # robust fit r = R + a cos th + b sin th (first-order decentre)
        design = np.column_stack([np.ones(len(a)), np.cos(theta[a]), np.sin(theta[a])])
        keep = np.ones(len(a), dtype=bool)
        weights = np.zeros(3)
        for _ in range(4):
            weights, *_ = np.linalg.lstsq(design[keep], r[a][keep], rcond=None)
            resid = r[a] - design @ weights
            sigma = np.median(np.abs(resid[keep] - np.median(resid[keep]))) * 1.4826 or 1.0
            keep = np.abs(resid - np.median(resid[keep])) < ARC_Z * sigma
        resid = r[a] - design @ weights
        sigma = np.median(np.abs(resid)) * 1.4826 or 1.0
        z = np.abs(resid) / sigma
        for j, i in enumerate(a):
            if z[j] > ARC_Z and abs(resid[j]) > ARC_ABS_FLOOR_PX:
                flagged[i] = True
                reasons.append({"source_line": source_lines[i], "x": x[i], "y": y[i],
                                "measured_cm": cm[i], "radius_px": r[i],
                                "reason": f"arc_angular_residual(arc={value},z={z[j]:.1f},dr={resid[j]:.1f}px)"})
    return flagged, reasons


def global_residual_outliers(data: np.ndarray, source_lines: list[int]) -> tuple[np.ndarray, list[dict]]:
    """Cross-check: fit the deployment model family and flag huge relative residuals."""
    x, y, cm = data[:, 0], data[:, 1], data[:, 2]
    design = design_matrix(x, y, GLOBAL_RADIAL_DEGREE, GLOBAL_ANGULAR_ORDER)
    target = np.log(np.clip(cm, 1e-6, None))
    keep = np.ones(len(cm), dtype=bool)
    weights = np.zeros(design.shape[1])
    for _ in range(5):
        weights, *_ = np.linalg.lstsq(design[keep], target[keep], rcond=None)
        pred = np.exp(design @ weights)
        rel = (pred - cm) / cm
        center = np.median(rel[keep])
        sigma = np.median(np.abs(rel[keep] - center)) * 1.4826 or 1.0
        keep = np.abs(rel - center) < GLOBAL_REL_Z * sigma
    rel = (np.exp(design @ weights) - cm) / cm
    center = np.median(rel)
    sigma = np.median(np.abs(rel - center)) * 1.4826 or 1.0
    flagged = np.abs(rel - center) > GLOBAL_REL_Z * sigma
    reasons: list[dict] = []
    for i in np.where(flagged)[0]:
        reasons.append({"source_line": source_lines[i], "x": x[i], "y": y[i],
                        "measured_cm": cm[i], "radius_px": float(np.hypot(x[i], y[i])),
                        "reason": f"global_rel_residual({rel[i] * 100:.0f}%)"})
    return flagged, reasons


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def make_plots(raw: np.ndarray, cleaned: np.ndarray, dropped_xy: np.ndarray,
               arc_rows: list[dict], plots_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.tri as mtri

    plots_dir.mkdir(parents=True, exist_ok=True)

    def grid_interp(xs, ys, zs, n=240):
        tri = mtri.Triangulation(xs, ys)
        gx = np.linspace(xs.min(), xs.max(), n)
        gy = np.linspace(ys.min(), ys.max(), n)
        mx, my = np.meshgrid(gx, gy)
        interp = mtri.LinearTriInterpolator(tri, zs)
        gz = interp(mx, my)
        return gx, gy, mx, my, gz

    rx, ry, rcm = raw[:, 0], raw[:, 1], raw[:, 2]
    cx, cy, ccm = cleaned[:, 0], cleaned[:, 1], cleaned[:, 2]

    # 1) raw measured scatter
    fig, ax = plt.subplots(figsize=(7, 6))
    sc = ax.scatter(rx, ry, c=rcm, cmap="viridis", s=22)
    ax.set(title="Raw measured distance samples", xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal")
    fig.colorbar(sc, ax=ax, label="measured distance (cm)")
    fig.tight_layout(); fig.savefig(plots_dir / "01_raw_measured_scatter.png", dpi=160); plt.close(fig)

    # 2) cleaned contour map (interpolated distance surface)
    gx, gy, mx, my, gz = grid_interp(cx, cy, ccm)
    fig, ax = plt.subplots(figsize=(7.4, 6))
    cf = ax.contourf(mx, my, gz, levels=24, cmap="viridis")
    ax.contour(mx, my, gz, levels=12, colors="white", linewidths=0.4, alpha=0.6)
    ax.scatter(cx, cy, c="k", s=4, alpha=0.4)
    ax.set(title="Distance contour map (cleaned data)", xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal")
    fig.colorbar(cf, ax=ax, label="distance (cm)")
    fig.tight_layout(); fig.savefig(plots_dir / "02_contour_map.png", dpi=160); plt.close(fig)

    # 3) gradient magnitude |grad cm| over the surface
    dzdy, dzdx = np.gradient(np.asarray(gz.filled(np.nan)), gy, gx)
    grad_mag = np.hypot(dzdx, dzdy)
    fig, ax = plt.subplots(figsize=(7.4, 6))
    cf = ax.contourf(mx, my, grad_mag, levels=24, cmap="magma")
    ax.set(title="Distance gradient magnitude  |grad cm| (cm per px)",
           xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal")
    fig.colorbar(cf, ax=ax, label="cm change per pixel")
    fig.tight_layout(); fig.savefig(plots_dir / "03_gradient_magnitude.png", dpi=160); plt.close(fig)

    # 4) radius vs distance (the core 1-D relationship + arc structure)
    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.scatter(np.hypot(rx, ry), rcm, s=14, alpha=0.35, color="#9ca3af", label="raw")
    ax.scatter(np.hypot(cx, cy), ccm, s=16, alpha=0.8, color="#2563eb", label="cleaned (inliers)")
    ax.set(title="Pixel radius vs measured distance", xlabel="pixel radius r = hypot(x,y)",
           ylabel="measured distance (cm)")
    ax.grid(alpha=0.25); ax.legend()
    fig.tight_layout(); fig.savefig(plots_dir / "04_radius_vs_distance.png", dpi=160); plt.close(fig)

    # 5) flagged outliers on the map
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.scatter(cx, cy, s=16, color="#2563eb", alpha=0.7, label="kept")
    if len(dropped_xy):
        ax.scatter(dropped_xy[:, 0], dropped_xy[:, 1], s=55, marker="x",
                   color="#dc2626", linewidths=1.6, label="removed outlier")
    ax.set(title="Removed outliers", xlabel="centred x (px)", ylabel="centred y (px)")
    ax.set_aspect("equal"); ax.legend()
    fig.tight_layout(); fig.savefig(plots_dir / "05_removed_outliers.png", dpi=160); plt.close(fig)

    # 6) per-arc radius spread before/after detrending
    if arc_rows:
        arc_rows = sorted(arc_rows, key=lambda r: r["arc_cm"])
        d = np.array([r["arc_cm"] for r in arc_rows])
        sb = np.array([r["spread_before_px"] for r in arc_rows])
        sa = np.array([r["spread_after_px"] for r in arc_rows])
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(d, sb, "o-", color="#dc2626", label="raw radius std")
        ax.plot(d, sa, "o-", color="#0f766e", label="after removing decentre wobble")
        ax.set(title="Per-arc pixel-radius spread (decentring vs true scatter)",
               xlabel="arc distance (cm)", ylabel="radius std (px)")
        ax.grid(alpha=0.25); ax.legend()
        fig.tight_layout(); fig.savefig(plots_dir / "06_per_arc_radius_spread.png", dpi=160); plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def write_csv(path: Path, data: np.ndarray) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x", "y", "measured_cm"])
        for x, y, cm in data:
            # keep integer-looking coords tidy, full precision on cm
            writer.writerow([_fmt(x), _fmt(y), repr(float(cm))])


def _fmt(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else repr(float(v))


def write_report(path: Path, rows: list[dict]) -> None:
    fields = ["source_line", "x", "y", "measured_cm", "radius_px", "reason"]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in sorted(rows, key=lambda r: r["source_line"]):
            writer.writerow({k: row.get(k, "") for k in fields})


def arc_summary(data: np.ndarray) -> list[dict]:
    x, y, cm = data[:, 0], data[:, 1], data[:, 2]
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    labels = np.round(cm, 2)
    out = []
    for value in np.unique(labels):
        a = np.where(labels == value)[0]
        if len(a) < ARC_MIN_POINTS:
            continue
        design = np.column_stack([np.ones(len(a)), np.cos(theta[a]), np.sin(theta[a])])
        weights, *_ = np.linalg.lstsq(design, r[a], rcond=None)
        resid = r[a] - design @ weights
        out.append({"arc_cm": float(value), "n_points": int(len(a)),
                    "mean_radius_px": float(r[a].mean()),
                    "spread_before_px": float(r[a].std()),
                    "spread_after_px": float(resid.std())})
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="cleaned CSV path")
    parser.add_argument("--plots-dir", type=Path, default=DATA_REVIEW_DIR / "plots")
    parser.add_argument("--skip-plots", action="store_true")
    args = parser.parse_args()

    raw, source_lines = read_csv(args.input)
    print(f"Loaded {len(raw)} rows from {args.input.name}")
    x, y, cm = raw[:, 0], raw[:, 1], raw[:, 2]
    r = np.hypot(x, y)
    print(f"  x[{x.min():.0f},{x.max():.0f}]  y[{y.min():.0f},{y.max():.0f}]  "
          f"cm[{cm.min():.1f},{cm.max():.1f}]  radius[{r.min():.0f},{r.max():.0f}]")

    # --- centre diagnostic (reported, never applied) ---
    dx, dy, spread_before, spread_after = robust_center_offset(x, y, np.round(cm, 2))
    print(f"\n[centre diagnostic] within-arc radius RMS spread {spread_before:.2f}px @ current centre.")
    print(f"  best centre correction ({dx:+.1f},{dy:+.1f})px -> {spread_after:.2f}px "
          f"({100 * (1 - (spread_after / spread_before) ** 2):.0f}% variance removed).")
    if np.hypot(dx, dy) > 3:
        print("  NOTE: a non-trivial centre offset remains. This is a CALIBRATION fix")
        print("  (update thresholds.json offsets + retrain), not handled here to keep the")
        print("  cleaned data in the same frame as deployment.")

    all_dropped: list[dict] = []

    # 1) duplicates
    deduped, deduped_lines, dup_dropped = collapse_duplicates(raw, source_lines)
    all_dropped += dup_dropped
    print(f"\n[duplicates] {len(raw) - len(deduped)} rows removed/collapsed "
          f"({len(dup_dropped)} dropped as ambiguous).")

    # 2) per-arc angular residual (primary)
    arc_flag, arc_reasons = per_arc_outliers(deduped, deduped_lines)
    print(f"[per-arc] flagged {int(arc_flag.sum())} points off their arc's smooth radius curve.")

    survivors = deduped[~arc_flag]
    survivor_lines = [l for l, f in zip(deduped_lines, arc_flag) if not f]
    all_dropped += arc_reasons

    # 3) global residual cross-check (secondary)
    glob_flag, glob_reasons = global_residual_outliers(survivors, survivor_lines)
    print(f"[global cross-check] flagged {int(glob_flag.sum())} additional high-residual points.")
    all_dropped += glob_reasons

    cleaned = survivors[~glob_flag]

    print(f"\nCLEANED: {len(cleaned)} rows kept, {len(raw) - len(cleaned)} removed "
          f"({100 * (len(raw) - len(cleaned)) / len(raw):.1f}%).")

    write_csv(args.output, cleaned)
    print(f"Wrote cleaned data -> {args.output}")
    write_report(DATA_REVIEW_DIR / "outlier_report.csv", all_dropped)
    print(f"Wrote outlier report -> {DATA_REVIEW_DIR / 'outlier_report.csv'}")

    arcs = arc_summary(cleaned)
    with (DATA_REVIEW_DIR / "arc_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["arc_cm", "n_points", "mean_radius_px",
                                                    "spread_before_px", "spread_after_px"])
        writer.writeheader()
        writer.writerows(arcs)
    print(f"Wrote arc summary -> {DATA_REVIEW_DIR / 'arc_summary.csv'}")

    if not args.skip_plots:
        dropped_xy = np.array([[d["x"], d["y"]] for d in all_dropped], dtype=np.float64) \
            if all_dropped else np.empty((0, 2))
        make_plots(raw, cleaned, dropped_xy, arcs, args.plots_dir)
        print(f"Wrote plots -> {args.plots_dir}")


if __name__ == "__main__":
    main()
