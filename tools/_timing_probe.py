"""Throwaway timing probe: mirrors run_runtime()'s per-loop work and reports
per-stage millis + the real Teensy send cadence. Delete after use."""
from __future__ import annotations

import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from main import (  # noqa: E402
    CAMERA_FPS, LOST_SENTINEL, CameraDetector, DistanceEstimator,
    OrbitDerivativeTracker, PiCameraSource, _init_lidar, load_params,
    load_thresholds, open_serial,
)
from estimation import BallKalman, PoseEstimate, PoseKalman  # noqa: E402
from estimation.kalman import polar_to_body_xy  # noqa: E402

RUN_SECONDS = 10.0


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1))))
    return s[k]


def summarize(name, vals):
    if not vals:
        print(f"  {name:<22} (no samples)")
        return
    print(f"  {name:<22} mean={statistics.mean(vals):6.2f}  p50={pct(vals,50):6.2f}  "
          f"p95={pct(vals,95):6.2f}  max={max(vals):6.2f}   (ms, n={len(vals)})")


def main():
    params = load_params()
    thresholds = load_thresholds()
    detector = CameraDetector(thresholds, DistanceEstimator(params))
    camera = PiCameraSource(thresholds)
    serial_link = open_serial()
    orbit = OrbitDerivativeTracker()
    lidar_reader, lidar_localizer, lidar_window = _init_lidar(True)
    ball_filter = BallKalman(params, camera_fps=CAMERA_FPS)
    pose_filter = PoseKalman(params)

    print(f"[probe] lidar_connected={lidar_reader is not None} serial={serial_link is not None}")

    t_cap, t_det, t_lpoll, t_loc, t_send, t_loop = [], [], [], [], [], []
    send_wall = []  # wall-clock timestamp of each Teensy send
    pts_per_poll, localize_runs = [], 0

    camera.start()
    last_t = time.perf_counter()
    t_end = time.perf_counter() + RUN_SECONDS
    loops = 0
    try:
        while time.perf_counter() < t_end:
            loop_t0 = time.perf_counter()
            now = loop_t0
            dt_s = max(1e-4, now - last_t)
            last_t = now

            telemetry = serial_link.poll_telemetry() if serial_link else None
            heading = telemetry.heading_deg if telemetry else 0.0

            a = time.perf_counter(); frame = camera.capture_bgr(); b = time.perf_counter()
            detections = detector.detect(frame, annotate=False); c = time.perf_counter()

            # lidar
            d = c
            if lidar_reader is not None and lidar_window is not None:
                new_points = lidar_reader.poll_points()
                if new_points:
                    lidar_window.extend(new_points)
                pts_per_poll.append(len(new_points))
                last_pts = list(lidar_window)
                d = time.perf_counter()
                if lidar_localizer is not None and last_pts:
                    result = lidar_localizer.update(last_pts, heading)
                    localize_runs += 1
                    pose_filter.update(
                        PoseEstimate(valid=bool(result.get("valid")),
                                     x_mm=float(result.get("x_mm") or 0.0),
                                     y_mm=float(result.get("y_mm") or 0.0)),
                        heading,
                    )
            e = time.perf_counter()

            pose_filter.predict(dt_s)
            pose_state = pose_filter.state(heading)
            ball_filter.predict(dt_s)
            if detections.ball.found:
                xm, ym = polar_to_body_xy(detections.ball_angle_deg, detections.ball_distance_cm * 0.01)
                ball_filter.update(xm, ym, True)

            f = time.perf_counter()
            if serial_link is not None:
                deriv = orbit.update(detections.ball_angle_deg, detections.ball_distance_cm)
                serial_link.send_detection(
                    detections.ball_angle_deg, detections.ball_distance_cm,
                    detections.blue_goal_angle_deg, detections.yellow_goal_angle_deg,
                    deriv,
                    pose_state.x_mm if pose_state.valid else LOST_SENTINEL,
                    pose_state.y_mm if pose_state.valid else LOST_SENTINEL,
                )
            g = time.perf_counter()
            send_wall.append(g)

            t_cap.append((b - a) * 1e3)
            t_det.append((c - b) * 1e3)
            t_lpoll.append((d - c) * 1e3)
            t_loc.append((e - d) * 1e3)
            t_send.append((g - f) * 1e3)
            t_loop.append((g - loop_t0) * 1e3)
            loops += 1
    finally:
        camera.close()
        if serial_link is not None:
            serial_link.close()
        if lidar_reader is not None:
            lidar_reader.close()

    # send-to-send interval (this is the rate the Teensy actually receives packets)
    intervals = [(send_wall[i] - send_wall[i - 1]) * 1e3 for i in range(1, len(send_wall))]

    print(f"\n==== RESULTS over {RUN_SECONDS:.0f}s, {loops} loops ====")
    rate = loops / RUN_SECONDS
    print(f"  Effective Teensy send rate: {rate:6.1f} Hz  (~{1000.0/rate:5.1f} ms/packet)")
    print(f"  localizer ran on {localize_runs}/{loops} loops; "
          f"avg new lidar pts/poll = {statistics.mean(pts_per_poll) if pts_per_poll else 0:.1f}\n")
    summarize("camera capture", t_cap)
    summarize("detect (vision)", t_det)
    summarize("lidar poll/parse", t_lpoll)
    summarize("lidar localize", t_loc)
    summarize("serial send", t_send)
    summarize("FULL LOOP", t_loop)
    print()
    summarize("send-to-send gap", intervals)


if __name__ == "__main__":
    main()
