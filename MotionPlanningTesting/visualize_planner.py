#!/usr/bin/env python3
"""
RoboCup ball-acquisition planner visualizer.

Calls the C++ binary (motion_planning --json) to obtain pre-computed A* paths,
Hermite splines, and heading schedules, then renders them.

No algorithm logic lives here — the C++ is the single source of truth.
To test different initial velocities, edit the vel[] array in runJsonExport()
inside src/main.cpp, rebuild, then re-run this script.

Requirements:
    pip install matplotlib
    cmake --build build

Headless (no display):
    MPLBACKEND=Agg python3 visualize_planner.py
"""

import json
import math
import os
import subprocess
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.cm as cm
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection

# ── Visual constants ──────────────────────────────────────────────────────────

BALL_R    =  4.3
ROBOT_R   =  9.0
GOAL_D    = 12.0
HDG_EVERY =  6

C_FIELD  = '#2a7030'
C_ROBOT  = '#4499ff'
C_BALL   = '#ff8c00'
C_OBS    = '#ff4444'
C_SPLINE = '#ff3333'
C_ASTAR  = '#99bbff'
C_TARGET = '#00ff99'
C_ARROW  = '#eeeeee'
C_SHOT   = '#ffff44'
C_GOAL_A = '#ffee00'
C_GOAL_D = '#ff8800'
C_VEL    = '#ff44ff'   # initial-velocity arrow
BG       = '#0d1117'

# ── C++ interface ─────────────────────────────────────────────────────────────

def get_planner_data():
    """
    Invoke the C++ binary with --json and return the parsed dict.
    Looks for the binary at bin/motion_planning relative to this file.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    binary = os.path.join(script_dir, 'bin', 'motion_planning')

    if not os.path.isfile(binary):
        sys.exit(
            f'Error: binary not found at:\n  {binary}\n'
            f'Build with:  cmake --build build'
        )

    proc = subprocess.run([binary, '--json'], capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f'C++ binary failed:\n{proc.stderr}')

    return json.loads(proc.stdout)

# ── Drawing helpers ───────────────────────────────────────────────────────────

def heading_vec(h_deg, length=1.0):
    """Direction vector for heading angle.  0°=+y, 90°=+x, CW."""
    r = math.radians(h_deg)
    return math.sin(r) * length, math.cos(r) * length

def _arrow(ax, x, y, dx, dy, color, lw=1.2, zorder=8):
    ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw),
                zorder=zorder)

# ── Field drawing ─────────────────────────────────────────────────────────────

def draw_field(ax, C):
    fw, fh     = C['field_w_cm'], C['field_h_cm']
    cell       = C['cell_cm']
    cols, rows = int(C['cols']), int(C['rows'])
    goal_y     = C['goal_y_cm']
    goal_w     = C['goal_w_cm']
    hw, hh     = fw / 2, fh / 2

    ax.add_patch(mpatches.Rectangle((-hw, -hh), fw, fh,
                                    fc=C_FIELD, ec='white', lw=2, zorder=0))

    col_offset = (cols - 1) // 2
    row_offset = (rows - 1) // 2
    for c in range(cols + 1):
        ax.axvline((c - col_offset) * cell, color='white', lw=0.15, alpha=0.2, zorder=1)
    for r in range(rows + 1):
        ax.axhline((r - row_offset) * cell, color='white', lw=0.15, alpha=0.2, zorder=1)

    ax.axhline(0, color='white', lw=1.0, ls='--', alpha=0.45, zorder=2)
    ax.add_patch(Circle((0, 0), 40, fc='none', ec='white', lw=1.0, alpha=0.45, zorder=2))
    ax.plot(0, 0, '+', color='white', ms=6, alpha=0.5, zorder=2)

    ax.add_patch(mpatches.Rectangle((-goal_w/2, goal_y), goal_w, GOAL_D,
                                    fc=C_GOAL_A + '33', ec=C_GOAL_A, lw=2, zorder=2))
    ax.text(0, goal_y + GOAL_D/2, 'ATTACK', color=C_GOAL_A,
            fontsize=5.5, ha='center', va='center', fontweight='bold', zorder=3)

    ax.add_patch(mpatches.Rectangle((-goal_w/2, -goal_y - GOAL_D), goal_w, GOAL_D,
                                    fc=C_GOAL_D + '33', ec=C_GOAL_D, lw=2, zorder=2))
    ax.text(0, -goal_y - GOAL_D/2, 'DEFENCE', color=C_GOAL_D,
            fontsize=5.5, ha='center', va='center', fontweight='bold', zorder=3)

    ax.set_xlim(-hw - 12, hw + 12)
    ax.set_ylim(-hh - 14, hh + 14)
    ax.set_aspect('equal')
    ax.set_xlabel('x (cm)', color='#888888', fontsize=7)
    ax.set_ylabel('y (cm)', color='#888888', fontsize=7)
    ax.tick_params(colors='#666666', labelsize=6)
    for sp in ax.spines.values():
        sp.set_edgecolor('#333333')

# ── Scenario drawing ──────────────────────────────────────────────────────────

def draw_scenario(ax, data, C):
    """
    Render one scenario from C++ JSON output.

    The spline is exactly as computed by C++ HermiteSpline::build() with the
    boundary velocities set in the vel[] array in src/main.cpp.
    Edit those values and rebuild to change the spline shape.
    """
    title      = data['title']
    rx, ry, rh = data['robot']['x'],    data['robot']['y'],    data['robot']['heading']
    bx, by     = data['ball']['x'],     data['ball']['y']
    gx, gy     = data['goal']['x'],     data['goal']['y']
    tgt_x      = data['approach']['x']
    tgt_y      = data['approach']['y']
    tgt_h      = data['approach']['heading']
    vx0        = data['robot_vel']['vx']
    vy0        = data['robot_vel']['vy']

    astar      = data['astar']
    path_found = astar['found']
    path_pts   = astar['path_cm']
    path_len   = astar['path_len_cm']
    rot        = astar['rotation_deg']
    node_count = astar['node_count']
    xs         = [p[0] for p in path_pts]
    ys         = [p[1] for p in path_pts]

    spline_pts    = data['spline']
    profile_speeds = data.get('profile', [])
    sx = [p['x']       for p in spline_pts]
    sy = [p['y']       for p in spline_pts]
    sh = [p['heading'] for p in spline_pts]

    ball_clear = C['ball_clear_cm']

    draw_field(ax, C)
    ax.set_title(title, color='white', fontsize=8, pad=5)

    # Ball obstacle zone
    ax.add_patch(Circle((bx, by), ball_clear,
                        fc=C_OBS, ec=C_OBS, alpha=0.20, zorder=3))
    ax.add_patch(Circle((bx, by), ball_clear,
                        fc='none', ec=C_OBS, lw=1, alpha=0.7, ls='--', zorder=3))

    # Ball
    ax.add_patch(Circle((bx, by), BALL_R, fc=C_BALL, ec='#cc6600', lw=1, zorder=4))
    ax.text(bx, by - BALL_R - 5, 'ball', color=C_BALL,
            fontsize=5.5, ha='center', va='top', zorder=5)

    if not path_found:
        ax.text(0, 0, 'NO PATH FOUND', color='red',
                ha='center', va='center', fontsize=10, zorder=20)
        return

    # Shot line + alignment
    ax.plot([bx, gx], [by, gy], '--', color=C_SHOT, lw=0.9, alpha=0.45, zorder=4)
    ax.plot([tgt_x, bx], [tgt_y, by], ':', color=C_TARGET, lw=1.0, alpha=0.6, zorder=4)

    # A* waypoints
    ax.plot(xs, ys, '--', color=C_ASTAR, lw=0.9, alpha=0.55, zorder=5)
    ax.scatter(xs, ys, s=12, color=C_ASTAR, alpha=0.75, zorder=6)

    # Hermite spline — colored by instantaneous speed when profile data is present,
    # otherwise drawn as a plain line.
    if profile_speeds and len(profile_speeds) == len(sx):
        speeds = np.array(profile_speeds, dtype=float)
        v_max  = speeds.max() if speeds.max() > 0 else 1.0

        pts  = np.array([sx, sy]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        seg_speeds = (speeds[:-1] + speeds[1:]) / 2.0  # midpoint speed per segment

        lc = LineCollection(segs, cmap='plasma', linewidth=2.5, zorder=7,
                            capstyle='round', joinstyle='round')
        lc.set_array(seg_speeds)
        lc.set_clim(0, v_max)
        ax.add_collection(lc)

        cbar = ax.get_figure().colorbar(lc, ax=ax, shrink=0.55, pad=0.02,
                                        orientation='vertical')
        cbar.set_label('speed (m/s)', color='#aaaaaa', fontsize=6)
        cbar.ax.tick_params(colors='#aaaaaa', labelsize=5.5)
        cbar.ax.yaxis.set_tick_params(color='#555555')
    else:
        ax.plot(sx, sy, '-', color=C_SPLINE, lw=2.2, zorder=7,
                solid_capstyle='round')

    # Heading arrows (robot orientation, independent of travel direction)
    alen = 7.0
    for i in range(0, len(sx), HDG_EVERY):
        dx, dy = heading_vec(sh[i], alen)
        _arrow(ax, sx[i], sy[i], dx, dy, color=C_ARROW, lw=0.7, zorder=8)

    # Approach target
    ax.add_patch(Circle((tgt_x, tgt_y), 3.5,
                        fc=C_TARGET, ec='white', lw=0.6, zorder=9))
    dx, dy = heading_vec(tgt_h, 13.0)
    _arrow(ax, tgt_x, tgt_y, dx, dy, color=C_TARGET, lw=1.6, zorder=10)
    ax.text(tgt_x, tgt_y - 6, f'target\n{tgt_h:.0f}°', color=C_TARGET,
            fontsize=5.5, ha='center', va='top', zorder=10)

    # Robot (facing arrow = white)
    ax.add_patch(Circle((rx, ry), ROBOT_R,
                        fc=C_ROBOT, ec='white', lw=0.8, alpha=0.85, zorder=9))
    dx, dy = heading_vec(rh, ROBOT_R * 1.6)
    _arrow(ax, rx, ry, dx, dy, color='white', lw=1.6, zorder=10)
    ax.text(rx, ry - ROBOT_R - 5, f'robot\n{rh:.0f}°', color=C_ROBOT,
            fontsize=5.5, ha='center', va='top', zorder=10)

    # Initial velocity arrow (magenta) — length proportional to speed
    speed = math.hypot(vx0, vy0)
    if speed > 0.5:
        arrow_len = min(speed * 0.35, 50.0)
        adx = (vx0 / speed) * arrow_len
        ady = (vy0 / speed) * arrow_len
        _arrow(ax, rx, ry, adx, ady, color=C_VEL, lw=2.5, zorder=11)
        ax.text(rx + adx * 1.2, ry + ady * 1.2,
                f'v₀={speed:.0f}', color=C_VEL,
                fontsize=6.0, ha='center', va='center', zorder=11)

    # Stats overlay
    direction = 'CW' if rot >= 0 else 'CCW'
    vel_str   = f'v₀=({vx0:.0f}, {vy0:.0f})' if speed > 0.5 else 'v₀=rest'
    stats = (
        f'A* nodes : {node_count}   path : {path_len:.0f} cm\n'
        f'Rotation : {abs(rot):.1f}° {direction}\n'
        f'Strike   : {tgt_h:.1f}°   {vel_str}'
    )
    ax.text(0.01, 0.985, stats, transform=ax.transAxes,
            color='#dddddd', fontsize=6.0, va='top', ha='left',
            family='monospace',
            bbox=dict(fc='#00000077', ec='none', pad=3), zorder=15)

# ── Main ──────────────────────────────────────────────────────────────────────

planner_data = get_planner_data()
constants    = {k: planner_data[k]
                for k in ('field_w_cm', 'field_h_cm', 'cell_cm',
                          'cols', 'rows', 'ball_clear_cm',
                          'goal_y_cm', 'goal_w_cm')}
scenarios    = planner_data['scenarios']

fig, axes = plt.subplots(2, 2, figsize=(14, 20))
fig.patch.set_facecolor(BG)

fig.suptitle(
    'RoboCup Ball-Acquisition Planner\n'
    'A* Pathfinding  ·  Hermite Spline  ·  Heading Schedule',
    color='white', fontsize=13, fontweight='bold', y=0.997)

for ax, scenario in zip(axes.flat, scenarios):
    ax.set_facecolor(BG)
    draw_scenario(ax, scenario, constants)

# ── Legend ────────────────────────────────────────────────────────────────────

bcm = constants['ball_clear_cm']
legend_items = [
    mpatches.Patch(fc=C_ROBOT, ec='white', lw=0.6,
                   label='Robot  (white arrow = facing dir.)'),
    mpatches.Patch(fc=C_BALL, ec='#cc6600',
                   label='Ball'),
    mpatches.Patch(fc=C_OBS, alpha=0.35,
                   label=f'Ball obstacle zone  (r={bcm:.0f} cm)'),
    plt.Line2D([0], [0], c=C_ASTAR, ls='--', marker='o', ms=4,
               label='A* waypoints'),
    plt.Line2D([0], [0], c='#ff88ff', lw=2.2,
               label='Hermite spline  (color = speed, violet=slow → yellow=fast)'),
    plt.Line2D([0], [0], c=C_ARROW, marker='>', ms=5,
               label='Heading arrows  (robot orientation)'),
    mpatches.Patch(fc=C_TARGET, ec='white', lw=0.5,
                   label='Approach target  (arrow = strike dir.)'),
    plt.Line2D([0], [0], c=C_VEL, lw=2, marker='>', ms=5,
               label='Init velocity v₀  (set in src/main.cpp vel[])'),
]

fig.legend(
    handles=legend_items,
    loc='lower center', ncol=4,
    fontsize=8, labelcolor='white',
    facecolor='#1a1a2e', edgecolor='#333355',
    framealpha=0.8,
    bbox_to_anchor=(0.5, 0.003),
)

plt.tight_layout(rect=[0, 0.065, 1, 0.993])

out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'planner_visualization.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f'Saved: {out_path}')

try:
    plt.show()
except Exception:
    pass
