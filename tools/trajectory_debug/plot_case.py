from __future__ import annotations

import argparse
import math
from pathlib import Path

from artifact_io import read_artifact


FIELD_WIDTH_MM = 1820.0
FIELD_HEIGHT_MM = 2430.0
GRID_SPACING_MM = 200.0
PADDING_PX = 40.0
CANVAS_WIDTH_PX = 1000.0
CANVAS_HEIGHT_PX = 1300.0


def to_canvas(x_mm: float, y_mm: float) -> tuple[float, float]:
    usable_w = CANVAS_WIDTH_PX - 2 * PADDING_PX
    usable_h = CANVAS_HEIGHT_PX - 2 * PADDING_PX
    x_px = PADDING_PX + usable_w * (x_mm / FIELD_WIDTH_MM)
    y_px = CANVAS_HEIGHT_PX - (PADDING_PX + usable_h * (y_mm / FIELD_HEIGHT_MM))
    return x_px, y_px


def arrow_points(x_mm: float, y_mm: float, heading_deg: float, length_mm: float) -> tuple[tuple[float, float], tuple[float, float]]:
    heading_rad = math.radians(heading_deg)
    end_x = x_mm + math.cos(heading_rad) * length_mm
    end_y = y_mm + math.sin(heading_rad) * length_mm
    return to_canvas(x_mm, y_mm), to_canvas(end_x, end_y)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a trajectory replay artifact as SVG")
    parser.add_argument("artifact", type=Path, help="Path to a .traj artifact")
    parser.add_argument("--output", type=Path, help="Output SVG path")
    args = parser.parse_args()

    artifact = read_artifact(args.artifact)
    output_path = args.output or args.artifact.with_suffix(".svg")

    svg: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_WIDTH_PX}" height="{CANVAS_HEIGHT_PX}" viewBox="0 0 {CANVAS_WIDTH_PX} {CANVAS_HEIGHT_PX}">',
        '<rect width="100%" height="100%" fill="#0f1216"/>',
    ]

    for gx in range(0, int(FIELD_WIDTH_MM) + 1, int(GRID_SPACING_MM)):
        x0, y0 = to_canvas(gx, 0)
        x1, y1 = to_canvas(gx, FIELD_HEIGHT_MM)
        svg.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#232b33" stroke-width="1"/>')
    for gy in range(0, int(FIELD_HEIGHT_MM) + 1, int(GRID_SPACING_MM)):
        x0, y0 = to_canvas(0, gy)
        x1, y1 = to_canvas(FIELD_WIDTH_MM, gy)
        svg.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#232b33" stroke-width="1"/>')

    x0, y0 = to_canvas(0, 0)
    x1, y1 = to_canvas(FIELD_WIDTH_MM, FIELD_HEIGHT_MM)
    svg.append(
        f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{x1 - x0:.1f}" height="{y0 - y1:.1f}" '
        'fill="none" stroke="#9eb6c8" stroke-width="2"/>'
    )

    if artifact["obstacle"].get("enabled"):
        cx, cy = to_canvas(artifact["obstacle"]["x_mm"], artifact["obstacle"]["y_mm"])
        radius_px = (artifact["obstacle"]["clear_mm"] / FIELD_WIDTH_MM) * (CANVAS_WIDTH_PX - 2 * PADDING_PX)
        svg.append(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{radius_px:.1f}" '
            'fill="rgba(245,119,77,0.14)" stroke="#f5774d" stroke-width="2"/>'
        )

    path_points = artifact.get("path", [])
    if path_points:
        d = " ".join(
            ("M" if i == 0 else "L") + f"{to_canvas(point['x_mm'], point['y_mm'])[0]:.1f},{to_canvas(point['x_mm'], point['y_mm'])[1]:.1f}"
            for i, point in enumerate(path_points)
        )
        svg.append(f'<path d="{d}" fill="none" stroke="#4cf5ae" stroke-width="3"/>')

    start = artifact["start_pose"]
    start_xy = to_canvas(start["x_mm"], start["y_mm"])
    svg.append(f'<circle cx="{start_xy[0]:.1f}" cy="{start_xy[1]:.1f}" r="7" fill="#4e98e2"/>')
    arrow_start, arrow_end = arrow_points(start["x_mm"], start["y_mm"], artifact["start_heading_deg"], 140.0)
    svg.append(
        f'<line x1="{arrow_start[0]:.1f}" y1="{arrow_start[1]:.1f}" '
        f'x2="{arrow_end[0]:.1f}" y2="{arrow_end[1]:.1f}" stroke="#4e98e2" stroke-width="3"/>'
    )

    goal = artifact["goal_pose"]
    if goal.get("enabled"):
        goal_xy = to_canvas(goal["x_mm"], goal["y_mm"])
        svg.append(f'<rect x="{goal_xy[0] - 7:.1f}" y="{goal_xy[1] - 7:.1f}" width="14" height="14" fill="#f7df71"/>')
        goal_arrow_start, goal_arrow_end = arrow_points(goal["x_mm"], goal["y_mm"], goal["heading_deg"], 140.0)
        svg.append(
            f'<line x1="{goal_arrow_start[0]:.1f}" y1="{goal_arrow_start[1]:.1f}" '
            f'x2="{goal_arrow_end[0]:.1f}" y2="{goal_arrow_end[1]:.1f}" stroke="#f7df71" stroke-width="3"/>'
        )

    labels = [
        f"Case: {artifact['case_name']}",
        f"Kind: {artifact['case_kind']}",
        f"Chunks: {len(artifact['chunks'])}",
        f"Path samples: {len(path_points)}",
    ]
    if artifact["chunks"]:
        labels.append(
            f"Chunk0: traj={artifact['chunks'][0]['trajectory_id']} dt={artifact['chunks'][0]['dt_ms']}ms actions={len(artifact['chunks'][0]['actions'])}"
        )
    for idx, label in enumerate(labels):
        svg.append(
            f'<text x="{PADDING_PX:.1f}" y="{24 + idx * 22:.1f}" fill="#d9e3ea" '
            'font-family="Menlo, Monaco, monospace" font-size="15">'
            f"{label}</text>"
        )

    svg.append("</svg>")
    output_path.write_text("\n".join(svg) + "\n", encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
