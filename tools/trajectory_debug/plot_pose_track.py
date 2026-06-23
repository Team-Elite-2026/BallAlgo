"""Render a recorded LiDAR pose track from an MCAP recording as a time-gradient SVG.

The C++ runtime computes the robot pose every loop with the real LidarLocalizer
(including deskewing) and streams it to foxglove_sim/sidecar.py, which records the
typed /robot/pose channel into an MCAP file.  This tool reads that channel back and
plots the (x, y) track on the field plane, coloring each sample by elapsed time so a
driven path (e.g. the rectangle_loop trajectory) is easy to inspect.

Usage:
    python3 tools/trajectory_debug/plot_pose_track.py [recording.mcap]
        [--output track.svg] [--topic /robot/pose]
        [--artifact tests/trajectory_cases/generated/rectangle_loop.traj]

With no MCAP argument the newest file in foxglove_sim/recordings/ is used.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RECORD_DIR = REPO_ROOT / "foxglove_sim" / "recordings"
DEFAULT_ARTIFACT_DIR = REPO_ROOT / "tests" / "trajectory_cases" / "generated"

# Field geometry — keep in sync with config.hpp / py_lidar_bench.config.
FIELD_WIDTH_MM = 1820.0
FIELD_HEIGHT_MM = 2430.0
GRID_SPACING_MM = 200.0
PADDING_PX = 40.0
CANVAS_WIDTH_PX = 1000.0
CANVAS_HEIGHT_PX = 1300.0

# viridis anchor stops (perceptually uniform); linearly interpolated by time fraction.
_VIRIDIS = (
    (0.00, (68, 1, 84)),
    (0.25, (59, 82, 139)),
    (0.50, (33, 145, 140)),
    (0.75, (94, 201, 98)),
    (1.00, (253, 231, 37)),
)


def viridis(fraction: float) -> str:
    f = min(1.0, max(0.0, fraction))
    for i in range(len(_VIRIDIS) - 1):
        f0, c0 = _VIRIDIS[i]
        f1, c1 = _VIRIDIS[i + 1]
        if f <= f1:
            t = 0.0 if f1 == f0 else (f - f0) / (f1 - f0)
            r = round(c0[0] + (c1[0] - c0[0]) * t)
            g = round(c0[1] + (c1[1] - c0[1]) * t)
            b = round(c0[2] + (c1[2] - c0[2]) * t)
            return f"rgb({r},{g},{b})"
    return f"rgb{_VIRIDIS[-1][1]}"


def to_canvas(x_mm: float, y_mm: float) -> tuple[float, float]:
    usable_w = CANVAS_WIDTH_PX - 2 * PADDING_PX
    usable_h = CANVAS_HEIGHT_PX - 2 * PADDING_PX
    x_px = PADDING_PX + usable_w * (x_mm / FIELD_WIDTH_MM)
    y_px = CANVAS_HEIGHT_PX - (PADDING_PX + usable_h * (y_mm / FIELD_HEIGHT_MM))
    return x_px, y_px


def newest_recording() -> Path:
    files = sorted(DEFAULT_RECORD_DIR.glob("*.mcap"), key=lambda p: p.stat().st_mtime)
    if not files:
        raise SystemExit(f"no .mcap recordings found in {DEFAULT_RECORD_DIR}")
    return files[-1]


def read_pose_track(mcap_path: Path, topic: str) -> list[tuple[float, float, float]]:
    """Return [(t_sec, x_mm, y_mm), ...] in bottom-left-origin field mm.

    The sidecar logs /robot/pose as a center-origin PoseInFrame in meters, so we undo
    both transforms to land back in LidarLocalizer's native frame.
    """
    half_w = FIELD_WIDTH_MM / 2.0
    half_h = FIELD_HEIGHT_MM / 2.0
    samples: list[tuple[float, float, float]] = []
    with mcap_path.open("rb") as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, proto in reader.iter_decoded_messages(topics=[topic]):
            ts = proto.timestamp
            t_sec = ts.seconds + ts.nanos * 1e-9
            x_mm = proto.pose.position.x * 1000.0 + half_w
            y_mm = proto.pose.position.y * 1000.0 + half_h
            samples.append((t_sec, x_mm, y_mm))
    samples.sort(key=lambda s: s[0])
    return samples


def read_velocity(mcap_path: Path, topic: str) -> list[tuple[float, float, float]]:
    """Return [(t_sec, vx_field_m_s, vy_field_m_s), ...] from the JSON /robot/twist channel.

    With kDeskewVelocityFromLidar enabled in the C++ runtime, the pose Kalman filter is
    driven purely by LiDAR, so these field-frame velocities are the LiDAR-derived velocity.
    """
    samples: list[tuple[float, float, float]] = []
    with mcap_path.open("rb") as fh:
        reader = make_reader(fh)
        for _schema, _channel, message in reader.iter_messages(topics=[topic]):
            data = json.loads(message.data)
            t_sec = message.log_time * 1e-9
            samples.append((t_sec, float(data["vx_field_m_s"]), float(data["vy_field_m_s"])))
    samples.sort(key=lambda s: s[0])
    return samples


def read_commanded_path(artifact_path: Path) -> list[tuple[float, float]]:
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from artifact_io import read_artifact

    artifact = read_artifact(artifact_path)
    return [(p["x_mm"], p["y_mm"]) for p in artifact.get("path", [])]


def build_svg(samples: list[tuple[float, float, float]],
              commanded: list[tuple[float, float]],
              label: str) -> str:
    svg: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_WIDTH_PX}" '
        f'height="{CANVAS_HEIGHT_PX}" viewBox="0 0 {CANVAS_WIDTH_PX} {CANVAS_HEIGHT_PX}">',
        '<rect width="100%" height="100%" fill="#0f1216"/>',
    ]

    # Field grid + boundary.
    for gx in range(0, int(FIELD_WIDTH_MM) + 1, int(GRID_SPACING_MM)):
        x0, y0 = to_canvas(gx, 0)
        x1, y1 = to_canvas(gx, FIELD_HEIGHT_MM)
        svg.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#232b33" stroke-width="1"/>')
    for gy in range(0, int(FIELD_HEIGHT_MM) + 1, int(GRID_SPACING_MM)):
        x0, y0 = to_canvas(0, gy)
        x1, y1 = to_canvas(FIELD_WIDTH_MM, gy)
        svg.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#232b33" stroke-width="1"/>')
    bx0, by0 = to_canvas(0, 0)
    bx1, by1 = to_canvas(FIELD_WIDTH_MM, FIELD_HEIGHT_MM)
    svg.append(
        f'<rect x="{bx0:.1f}" y="{by1:.1f}" width="{bx1 - bx0:.1f}" height="{by0 - by1:.1f}" '
        'fill="none" stroke="#9eb6c8" stroke-width="2"/>'
    )

    # Commanded path overlay (dashed, behind the measured track).
    if commanded:
        d = " ".join(
            ("M" if i == 0 else "L") + f"{to_canvas(x, y)[0]:.1f},{to_canvas(x, y)[1]:.1f}"
            for i, (x, y) in enumerate(commanded)
        )
        svg.append(f'<path d="{d}" fill="none" stroke="#9eb6c8" stroke-width="2" '
                   'stroke-dasharray="6 6" opacity="0.5"/>')

    # Measured track: faint connector + time-gradient sample dots.
    if samples:
        t0 = samples[0][0]
        t1 = samples[-1][0]
        span = max(t1 - t0, 1e-9)
        connector = " ".join(
            ("M" if i == 0 else "L") + f"{to_canvas(x, y)[0]:.1f},{to_canvas(x, y)[1]:.1f}"
            for i, (_t, x, y) in enumerate(samples)
        )
        svg.append(f'<path d="{connector}" fill="none" stroke="#3a4750" stroke-width="1" opacity="0.6"/>')
        for t, x, y in samples:
            px, py = to_canvas(x, y)
            svg.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="2.6" fill="{viridis((t - t0) / span)}"/>')

        # Start / end markers.
        sx, sy = to_canvas(samples[0][1], samples[0][2])
        ex, ey = to_canvas(samples[-1][1], samples[-1][2])
        svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="6" fill="none" stroke="#ffffff" stroke-width="2"/>')
        svg.append(f'<rect x="{ex - 5:.1f}" y="{ey - 5:.1f}" width="10" height="10" '
                   'fill="none" stroke="#ffffff" stroke-width="2"/>')

        _append_colorbar(svg, span)

    # Header labels.
    info = [
        label,
        f"Samples: {len(samples)}",
        f"Duration: {samples[-1][0] - samples[0][0]:.1f} s" if samples else "Duration: n/a",
    ]
    for idx, text in enumerate(info):
        svg.append(
            f'<text x="{PADDING_PX:.1f}" y="{24 + idx * 22:.1f}" fill="#d9e3ea" '
            'font-family="Menlo, Monaco, monospace" font-size="15">'
            f"{text}</text>"
        )

    svg.append("</svg>")
    return "\n".join(svg) + "\n"


def _append_colorbar(svg: list[str], span_s: float) -> None:
    bar_x = CANVAS_WIDTH_PX - PADDING_PX - 18
    bar_y = PADDING_PX
    bar_h = 220.0
    bar_w = 14.0
    steps = 40
    for i in range(steps):
        frac_top = i / steps           # i=0 is the top of the bar = latest time
        seg_y = bar_y + frac_top * bar_h
        color = viridis(1.0 - frac_top)
        svg.append(f'<rect x="{bar_x:.1f}" y="{seg_y:.1f}" width="{bar_w:.1f}" '
                   f'height="{bar_h / steps + 1:.1f}" fill="{color}"/>')
    svg.append(f'<rect x="{bar_x:.1f}" y="{bar_y:.1f}" width="{bar_w:.1f}" height="{bar_h:.1f}" '
               'fill="none" stroke="#9eb6c8" stroke-width="1"/>')
    for frac, value in ((0.0, span_s), (1.0, 0.0)):
        ty = bar_y + frac * bar_h + 4
        svg.append(f'<text x="{bar_x - 6:.1f}" y="{ty:.1f}" fill="#d9e3ea" text-anchor="end" '
                   'font-family="Menlo, Monaco, monospace" font-size="12">'
                   f"{value:.1f}s</text>")


def build_velocity_svg(samples: list[tuple[float, float, float]], label: str) -> str:
    """Two stacked line panels: vx_field(t) on top, vy_field(t) below (m/s vs seconds)."""
    width = 1000.0
    panel_h = 280.0
    gap = 70.0
    top = 70.0
    height = top + 2 * panel_h + gap + 50.0
    left = 70.0
    right = 40.0
    plot_w = width - left - right

    t0 = samples[0][0]
    t_max = max(s[0] - t0 for s in samples) or 1e-9
    v_abs = max((max(abs(s[1]), abs(s[2])) for s in samples), default=0.5)
    v_abs = max(v_abs * 1.15, 0.05)  # padding, avoid zero-height axis

    svg: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#0f1216"/>',
        f'<text x="{left:.1f}" y="36" fill="#d9e3ea" '
        'font-family="Menlo, Monaco, monospace" font-size="16">'
        f"LiDAR velocity (field frame) — {label}</text>",
    ]

    panels = (
        (top, 1, "vx_field (m/s)", "#4cf5ae"),
        (top + panel_h + gap, 2, "vy_field (m/s)", "#f5a14c"),
    )
    for panel_top, idx, title, color in panels:
        _velocity_panel(svg, left, panel_top, plot_w, panel_h, samples, idx,
                        title, color, t0, t_max, v_abs)

    svg.append("</svg>")
    return "\n".join(svg) + "\n"


def _velocity_panel(svg: list[str], x0: float, y0: float, w: float, h: float,
                    samples: list[tuple[float, float, float]], value_idx: int,
                    title: str, color: str, t0: float, t_max: float, v_abs: float) -> None:
    def sx(t_rel: float) -> float:
        return x0 + w * (t_rel / t_max)

    def sy(v: float) -> float:
        return y0 + h * (0.5 - 0.5 * (v / v_abs))

    # Axis box + zero line.
    svg.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{w:.1f}" height="{h:.1f}" '
               'fill="#151a1f" stroke="#3a4750" stroke-width="1"/>')
    zero_y = sy(0.0)
    svg.append(f'<line x1="{x0:.1f}" y1="{zero_y:.1f}" x2="{x0 + w:.1f}" y2="{zero_y:.1f}" '
               'stroke="#5a6b78" stroke-width="1" stroke-dasharray="4 4"/>')

    # Y ticks at -v_abs, 0, +v_abs.
    for v in (-v_abs, 0.0, v_abs):
        ty = sy(v)
        svg.append(f'<text x="{x0 - 8:.1f}" y="{ty + 4:.1f}" fill="#9eb6c8" text-anchor="end" '
                   'font-family="Menlo, Monaco, monospace" font-size="11">'
                   f"{v:+.2f}</text>")
    # X ticks every ~2 s.
    tick = 2.0
    n = int(t_max // tick)
    for i in range(n + 1):
        t = i * tick
        tx = sx(t)
        svg.append(f'<line x1="{tx:.1f}" y1="{y0:.1f}" x2="{tx:.1f}" y2="{y0 + h:.1f}" '
                   'stroke="#232b33" stroke-width="1"/>')
        svg.append(f'<text x="{tx:.1f}" y="{y0 + h + 16:.1f}" fill="#9eb6c8" text-anchor="middle" '
                   'font-family="Menlo, Monaco, monospace" font-size="11">'
                   f"{t:.0f}s</text>")

    # The trace.
    d = " ".join(
        ("M" if i == 0 else "L") + f"{sx(s[0] - t0):.1f},{sy(s[value_idx]):.1f}"
        for i, s in enumerate(samples)
    )
    svg.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2"/>')
    svg.append(f'<text x="{x0 + 6:.1f}" y="{y0 + 16:.1f}" fill="{color}" '
               'font-family="Menlo, Monaco, monospace" font-size="13">'
               f"{title}</text>")


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot a recorded LiDAR pose track as a time-gradient SVG")
    parser.add_argument("mcap", nargs="?", type=Path, help="MCAP recording (default: newest in recordings/)")
    parser.add_argument("--output", type=Path, help="Output SVG path")
    parser.add_argument("--topic", default="/robot/pose", help="Pose topic to read (default: /robot/pose)")
    parser.add_argument("--velocity-topic", default="/robot/twist",
                        help="Velocity topic to read (default: /robot/twist)")
    parser.add_argument("--artifact", type=Path,
                        help="Optional .traj artifact to overlay its commanded path")
    parser.add_argument("--no-velocity", action="store_true",
                        help="Skip the vx/vy velocity time-series graph")
    args = parser.parse_args()

    mcap_path = args.mcap or newest_recording()
    if not mcap_path.exists():
        raise SystemExit(f"recording not found: {mcap_path}")

    samples = read_pose_track(mcap_path, args.topic)
    if not samples:
        raise SystemExit(f"no messages on {args.topic} in {mcap_path}")

    commanded: list[tuple[float, float]] = []
    if args.artifact:
        commanded = read_commanded_path(args.artifact)

    output_path = args.output or mcap_path.with_suffix(".pose_track.svg")
    svg = build_svg(samples, commanded, f"Recording: {mcap_path.name}")
    output_path.write_text(svg, encoding="utf-8")
    print(output_path)

    if not args.no_velocity:
        vel = read_velocity(mcap_path, args.velocity_topic)
        if vel:
            vel_path = output_path.with_suffix("")
            vel_path = vel_path.with_name(vel_path.name + ".velocity.svg")
            vel_path.write_text(build_velocity_svg(vel, mcap_path.name), encoding="utf-8")
            print(vel_path)
        else:
            print(f"(no messages on {args.velocity_topic}; skipped velocity graph)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
