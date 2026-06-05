#!/usr/bin/env python3
"""
Visualize the robot EKF simulation.
Runs the C++ binary in --csv mode, parses the output, and produces a
three-panel figure:
  Left  : 2D field view — true path vs EKF estimate + LiDAR snap arrows
  Right : err_x and err_y over time, showing the saw-tooth drift/correction pattern

Usage
-----
  python3 plot_ekf.py              # auto-saves ekf_visualization.png
  python3 plot_ekf.py --no-show   # save only, skip interactive window
"""

import csv
import io
import os
import subprocess
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D

# ---------------------------------------------------------------------------
# Run C++ binary
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BINARY     = os.path.join(SCRIPT_DIR, "build", "bin", "motion_planning")

result = subprocess.run([BINARY, "--csv"], capture_output=True, text=True)
if result.returncode != 0:
    print("Binary failed:\n", result.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Parse CSV
# ---------------------------------------------------------------------------
reader = csv.DictReader(io.StringIO(result.stdout))
rows   = list(reader)

t       = np.array([float(r["t"])       for r in rows])
true_x  = np.array([float(r["true_x"])  for r in rows])
true_y  = np.array([float(r["true_y"])  for r in rows])
ekf_x   = np.array([float(r["ekf_x"])   for r in rows])
ekf_y   = np.array([float(r["ekf_y"])   for r in rows])
events  = [r["event"] for r in rows]

pred_mask     = np.array([e == "pred"     for e in events])
lidar_mask    = np.array([e == "lidar"    for e in events])
pre_snap_mask = np.array([e == "pre_snap" for e in events])

# Build a continuous EKF path: pred + lidar rows in time order (skip pre_snap
# for path drawing — pre_snap is plotted separately as drift markers).
path_mask = pred_mask | lidar_mask

lidar_times = t[lidar_mask]

err_x = ekf_x[pred_mask] - true_x[pred_mask]
err_y = ekf_y[pred_mask] - true_y[pred_mask]
t_pred = t[pred_mask]

# ---------------------------------------------------------------------------
# Figure layout
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(15, 8))
fig.suptitle(
    "Robot EKF — Mouse sensor dead-reckoning (1 kHz) corrected by LiDAR snaps (15 Hz)",
    fontsize=12, fontweight="bold"
)

gs = GridSpec(2, 2, figure=fig, width_ratios=[1.3, 1], hspace=0.42, wspace=0.32)
ax_xy = fig.add_subplot(gs[:, 0])   # 2D field view, full height
ax_ex = fig.add_subplot(gs[0, 1])   # err_x
ax_ey = fig.add_subplot(gs[1, 1])   # err_y

# ---------------------------------------------------------------------------
# Left: 2D field view
# ---------------------------------------------------------------------------

# EKF predicted path (pred rows only — continuous dead-reckoning between snaps)
ax_xy.plot(ekf_x[pred_mask], ekf_y[pred_mask],
           color="#FF9800", lw=1.5, linestyle="--",
           label="EKF estimate (pred)", zorder=2)

# Pre-snap drift markers: hollow circles where the EKF was just before each snap
ax_xy.scatter(ekf_x[pre_snap_mask], ekf_y[pre_snap_mask],
              marker="o", s=70, facecolors="none", edgecolors="#E53935",
              linewidths=1.8, label="Pre-snap position (drift)", zorder=5)

# Post-snap markers: stars at corrected positions
ax_xy.scatter(ekf_x[lidar_mask], ekf_y[lidar_mask],
              marker="*", s=180, color="#43A047",
              label="Post-snap position (LiDAR fix)", zorder=6)

# Snap correction arrows: pre-snap → post-snap
for idx, row in enumerate(rows):
    if row["event"] == "pre_snap" and idx + 1 < len(rows) and rows[idx + 1]["event"] == "lidar":
        px = float(row["ekf_x"]);  py = float(row["ekf_y"])
        lx = float(rows[idx + 1]["ekf_x"]); ly = float(rows[idx + 1]["ekf_y"])
        ax_xy.annotate(
            "", xy=(lx, ly), xytext=(px, py),
            arrowprops=dict(arrowstyle="-|>", color="#E53935",
                            lw=1.3, mutation_scale=12),
            zorder=7
        )

ax_xy.set_xlabel("x  (m)  →  right", fontsize=10)
ax_xy.set_ylabel("y  (m)  →  forward", fontsize=10)
ax_xy.set_title("2D Field Position", fontsize=11)
ax_xy.legend(fontsize=8.5, loc="upper left")
ax_xy.set_aspect("equal", adjustable="datalim")
ax_xy.grid(True, alpha=0.3)
ax_xy.axhline(0, color="k", lw=0.5, alpha=0.35)
ax_xy.axvline(0, color="k", lw=0.5, alpha=0.35)

# Annotate first and last LiDAR snap to orient the reader
if lidar_mask.any():
    first_i = np.where(lidar_mask)[0][0]
    ax_xy.annotate(
        "LiDAR snap #1",
        xy=(ekf_x[first_i], ekf_y[first_i]),
        xytext=(ekf_x[first_i] + 0.04, ekf_y[first_i] - 0.05),
        arrowprops=dict(arrowstyle="->", color="grey", lw=0.8),
        fontsize=7.5, color="grey"
    )

# ---------------------------------------------------------------------------
# Right top: err_x over time
# ---------------------------------------------------------------------------
ax_ex.plot(t_pred, err_x, color="#1565C0", lw=1.2, label="err_x")
ax_ex.axhline(0, color="k", lw=0.5, alpha=0.5)
ax_ex.fill_between(t_pred, err_x, alpha=0.12, color="#1565C0")

for lt in lidar_times:
    ax_ex.axvline(lt, color="#43A047", lw=0.9, alpha=0.7, linestyle=":")

ax_ex.set_ylabel("err_x  (m)", fontsize=9)
ax_ex.set_xlabel("time  (s)", fontsize=9)
ax_ex.set_title("x position error  (ekf − true)\ngreen lines = LiDAR snaps", fontsize=9)
ax_ex.grid(True, alpha=0.3)

# ---------------------------------------------------------------------------
# Right bottom: err_y over time
# ---------------------------------------------------------------------------
ax_ey.plot(t_pred, err_y, color="#E65100", lw=1.2, label="err_y")
ax_ey.axhline(0, color="k", lw=0.5, alpha=0.5)
ax_ey.fill_between(t_pred, err_y, alpha=0.12, color="#E65100")

for lt in lidar_times:
    ax_ey.axvline(lt, color="#43A047", lw=0.9, alpha=0.7, linestyle=":")

ax_ey.set_ylabel("err_y  (m)", fontsize=9)
ax_ey.set_xlabel("time  (s)", fontsize=9)
ax_ey.set_title("y position error  (ekf − true)\ngreen lines = LiDAR snaps", fontsize=9)
ax_ey.grid(True, alpha=0.3)

# ---------------------------------------------------------------------------
# Summary stats in figure
# ---------------------------------------------------------------------------
max_drift_x = float(np.max(np.abs(err_x)))
max_drift_y = float(np.max(np.abs(err_y)))
n_snaps     = int(lidar_mask.sum())

fig.text(0.52, 0.01,
         f"n_snaps={n_snaps}  |  max |err_x|={max_drift_x:.4f} m  |  max |err_y|={max_drift_y:.4f} m",
         ha="center", fontsize=8.5, color="#555555")

# ---------------------------------------------------------------------------
# Save + show
# ---------------------------------------------------------------------------
out_path = os.path.join(SCRIPT_DIR, "ekf_visualization.png")
plt.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"Saved: {out_path}")

if "--no-show" not in sys.argv:
    plt.show()
