from __future__ import annotations

import argparse
import errno
import math
import signal
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Add parent foxglove_sim/ to path so config and field_geometry are importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import load_config
from fake_common import (
    centered_cm_to_mm,
    clamp,
    encode_snapshot,
    make_snapshot,
    now_timestamp_ns,
    path_length_mm,
    point_heading_deg,
)


@dataclass(slots=True)
class FakeRuntimeConfig:
    socket_path: str
    scenario: str
    hz: float
    duration_s: float
    label: str


class FakeRuntimePublisher:
    def __init__(self, cfg: FakeRuntimeConfig):
        self.cfg = cfg
        self.running = True
        self.socket: socket.socket | None = None
        self.generated_frames = 0
        self.sent_frames = 0
        self.connect_count = 0
        self.last_connect_error = ""

    def _ensure_connected(self) -> bool:
        if self.socket is not None:
            return True
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(self.cfg.socket_path)
            self.socket = sock
            self.connect_count += 1
            self.last_connect_error = ""
            return True
        except (FileNotFoundError, ConnectionRefusedError, OSError) as exc:
            if 'sock' in locals():
                sock.close()
            self.socket = None
            self.last_connect_error = f"{type(exc).__name__}: {exc}"
            return False

    def stop(self, *_args: object) -> None:
        self.running = False

    def _send(self, payload: dict[str, Any]) -> bool:
        if not self._ensure_connected():
            return False
        try:
            assert self.socket is not None
            framed = encode_snapshot(payload) + b"\n"
            self.socket.sendall(framed)
            self.sent_frames += 1
            return True
        except (BlockingIOError, BrokenPipeError, FileNotFoundError, ConnectionRefusedError):
            if self.socket is not None:
                self.socket.close()
                self.socket = None
            return False
        except OSError as exc:
            if exc.errno in {errno.ENOENT, errno.ECONNREFUSED, errno.ENOTCONN, errno.ENOBUFS}:
                if self.socket is not None:
                    self.socket.close()
                    self.socket = None
                return False
            raise

    def run(self) -> bool:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)

        frame_period_s = 1.0 / self.cfg.hz
        start_wall = time.monotonic()
        next_frame = start_wall
        start_time_s = start_wall

        while self.running:
            now = time.monotonic()
            elapsed_s = now - start_time_s
            if self.cfg.duration_s > 0 and elapsed_s >= self.cfg.duration_s:
                break

            snapshot = build_scenario_snapshot(self.cfg.scenario, elapsed_s, self.cfg.label)
            self.generated_frames += 1
            self._send(snapshot)

            next_frame += frame_period_s
            sleep_for = next_frame - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_frame = time.monotonic()

        if self.socket is not None:
            self.socket.close()
            self.socket = None

        print(
            f"Fake runtime scenario={self.cfg.scenario} generated={self.generated_frames} "
            f"sent={self.sent_frames} connects={self.connect_count} "
            f"socket_path={self.cfg.socket_path} "
            f"last_connect_error={self.last_connect_error or 'none'}"
        )
        return self.sent_frames > 0


def build_scenario_snapshot(scenario: str, t_s: float, label: str) -> dict[str, Any]:
    builders = {
        "replan_demo": build_replan_demo_snapshot,
        "orbit_ball": build_orbit_ball_snapshot,
        "straight_line": build_straight_line_snapshot,
        "idle": build_idle_snapshot,
    }
    builder = builders.get(scenario)
    if builder is None:
        raise ValueError(f"unsupported scenario: {scenario}")
    return builder(t_s, label)


def build_replan_demo_snapshot(t_s: float, label: str) -> dict[str, Any]:
    cycle_s = 14.0
    phase = (t_s % cycle_s) / cycle_s
    omega = 2.0 * math.pi / cycle_s

    center_x, center_y = centered_cm_to_mm(0.0, 0.0)
    radius_x_mm = 420.0
    radius_y_mm = 650.0

    robot_x = center_x + radius_x_mm * math.cos(2.0 * math.pi * phase)
    robot_y = center_y + radius_y_mm * math.sin(2.0 * math.pi * phase)
    robot_vx_field = -radius_x_mm * omega * math.sin(2.0 * math.pi * phase) / 1000.0
    robot_vy_field = radius_y_mm * omega * math.cos(2.0 * math.pi * phase) / 1000.0

    target_phase = phase + 0.18
    target_x = center_x + radius_x_mm * math.cos(2.0 * math.pi * target_phase)
    target_y = center_y + radius_y_mm * math.sin(2.0 * math.pi * target_phase)
    heading_deg = point_heading_deg(robot_x, robot_y, target_x, target_y)

    robot_vx_body, robot_vy_body = field_to_body(robot_vx_field, robot_vy_field, heading_deg)
    robot_ax_field = -(radius_x_mm / 1000.0) * (omega**2) * math.cos(2.0 * math.pi * phase)
    robot_ay_field = -(radius_y_mm / 1000.0) * (omega**2) * math.sin(2.0 * math.pi * phase)
    robot_ax_body, robot_ay_body = field_to_body(robot_ax_field, robot_ay_field, heading_deg)

    ball_x = center_x + 90.0 * math.sin(0.7 * t_s)
    ball_y = center_y + 140.0 * math.cos(0.9 * t_s)
    ball_vx_field = 0.063 * math.cos(0.7 * t_s)
    ball_vy_field = -0.126 * math.sin(0.9 * t_s)
    ball_body_x, ball_body_y = field_to_body_position(ball_x - robot_x, ball_y - robot_y, heading_deg)
    ball_vx_body, ball_vy_body = field_to_body(ball_vx_field, ball_vy_field, heading_deg)

    path_points = make_bezier_path(
        (robot_x, robot_y),
        (
            robot_x + 0.4 * (target_x - robot_x),
            robot_y + 0.2 * (target_y - robot_y),
        ),
        (
            robot_x + 0.75 * (target_x - robot_x),
            robot_y + 0.9 * (target_y - robot_y),
        ),
        (target_x, target_y),
        samples=18,
    )
    planner = make_planner_path(
        trajectory_id=int(t_s * 5.0) + 1,
        target_x_mm=target_x,
        target_y_mm=target_y,
        target_heading_deg=point_heading_deg(target_x, target_y, ball_x, ball_y),
        points=path_points,
    )

    return make_snapshot(
        timestamp_ns=now_timestamp_ns(),
        pose={
            "valid": True,
            "heading_deg": heading_deg,
            "x_mm": robot_x,
            "y_mm": robot_y,
            "vx_mm_s": robot_vx_field * 1000.0,
            "vy_mm_s": robot_vy_field * 1000.0,
            "vx_body_m_s": robot_vx_body,
            "vy_body_m_s": robot_vy_body,
        },
        ball={
            "visible": True,
            "field_visible": True,
            "body_x_m": ball_body_x,
            "body_y_m": ball_body_y,
            "body_vx_m_s": ball_vx_body,
            "body_vy_m_s": ball_vy_body,
            "field_x_mm": ball_x,
            "field_y_mm": ball_y,
            "field_vx_m_s": ball_vx_field,
            "field_vy_m_s": ball_vy_field,
            "vision_angle_deg": point_heading_deg(0.0, 0.0, ball_body_x, ball_body_y),
            "vision_dist_cal": math.hypot(ball_body_x, ball_body_y),
        },
        planner_path=planner,
        robot_twist=make_robot_twist(robot_vx_body, robot_vy_body, robot_vx_field, robot_vy_field),
        robot_accel=make_robot_accel(robot_ax_body, robot_ay_body, robot_ax_field, robot_ay_field),
        robot_angular={
            "heading_deg": heading_deg,
            "omega_deg_s": 360.0 / cycle_s,
            "alpha_deg_s2": 0.0,
        },
        ball_twist=make_ball_twist(True, ball_vx_body, ball_vy_body, ball_vx_field, ball_vy_field),
        log_message=f"{label}: replan_demo t={t_s:.2f}s traj={planner['trajectory_id']}",
    )


def build_orbit_ball_snapshot(t_s: float, label: str) -> dict[str, Any]:
    ball_x, ball_y = centered_cm_to_mm(10.0, -15.0)
    orbit_radius_mm = 320.0
    orbit_period_s = 10.0
    theta = 2.0 * math.pi * (t_s / orbit_period_s)
    robot_x = ball_x + orbit_radius_mm * math.cos(theta)
    robot_y = ball_y + orbit_radius_mm * math.sin(theta)
    robot_vx_field = -(orbit_radius_mm / 1000.0) * (2.0 * math.pi / orbit_period_s) * math.sin(theta)
    robot_vy_field = (orbit_radius_mm / 1000.0) * (2.0 * math.pi / orbit_period_s) * math.cos(theta)
    heading_deg = point_heading_deg(robot_x, robot_y, ball_x, ball_y)

    robot_vx_body, robot_vy_body = field_to_body(robot_vx_field, robot_vy_field, heading_deg)
    robot_ax_field = -(orbit_radius_mm / 1000.0) * ((2.0 * math.pi / orbit_period_s) ** 2) * math.cos(theta)
    robot_ay_field = -(orbit_radius_mm / 1000.0) * ((2.0 * math.pi / orbit_period_s) ** 2) * math.sin(theta)
    robot_ax_body, robot_ay_body = field_to_body(robot_ax_field, robot_ay_field, heading_deg)

    goal_x, goal_y = centered_cm_to_mm(70.0, 0.0)
    strike_x = ball_x - 120.0 * math.cos(math.radians(point_heading_deg(ball_x, ball_y, goal_x, goal_y)))
    strike_y = ball_y - 120.0 * math.sin(math.radians(point_heading_deg(ball_x, ball_y, goal_x, goal_y)))
    ball_body_x, ball_body_y = field_to_body_position(ball_x - robot_x, ball_y - robot_y, heading_deg)
    path_points = make_bezier_path(
        (robot_x, robot_y),
        ((robot_x + ball_x) * 0.5, robot_y),
        ((ball_x + strike_x) * 0.5, (ball_y + strike_y) * 0.5),
        (strike_x, strike_y),
        samples=16,
    )

    return make_snapshot(
        timestamp_ns=now_timestamp_ns(),
        pose={
            "valid": True,
            "heading_deg": heading_deg,
            "x_mm": robot_x,
            "y_mm": robot_y,
            "vx_mm_s": robot_vx_field * 1000.0,
            "vy_mm_s": robot_vy_field * 1000.0,
            "vx_body_m_s": robot_vx_body,
            "vy_body_m_s": robot_vy_body,
        },
        ball={
            "visible": True,
            "field_visible": True,
            "body_x_m": ball_body_x,
            "body_y_m": ball_body_y,
            "body_vx_m_s": 0.0,
            "body_vy_m_s": 0.0,
            "field_x_mm": ball_x,
            "field_y_mm": ball_y,
            "field_vx_m_s": 0.0,
            "field_vy_m_s": 0.0,
            "vision_angle_deg": point_heading_deg(robot_x, robot_y, ball_x, ball_y),
            "vision_dist_cal": math.hypot(ball_x - robot_x, ball_y - robot_y) / 1000.0,
        },
        planner_path=make_planner_path(
            trajectory_id=int(t_s * 4.0) + 1,
            target_x_mm=strike_x,
            target_y_mm=strike_y,
            target_heading_deg=point_heading_deg(strike_x, strike_y, ball_x, ball_y),
            points=path_points,
        ),
        robot_twist=make_robot_twist(robot_vx_body, robot_vy_body, robot_vx_field, robot_vy_field),
        robot_accel=make_robot_accel(robot_ax_body, robot_ay_body, robot_ax_field, robot_ay_field),
        robot_angular={
            "heading_deg": heading_deg,
            "omega_deg_s": 36.0,
            "alpha_deg_s2": 0.0,
        },
        ball_twist=make_ball_twist(True, 0.0, 0.0, 0.0, 0.0),
        log_message=f"{label}: orbit_ball t={t_s:.2f}s",
    )


def build_straight_line_snapshot(t_s: float, label: str) -> dict[str, Any]:
    start_x, start_y = centered_cm_to_mm(-55.0, -60.0)
    end_x, end_y = centered_cm_to_mm(55.0, 75.0)
    duration_s = 9.0
    progress = clamp(t_s / duration_s, 0.0, 1.0)
    eased = 0.5 - 0.5 * math.cos(math.pi * progress)
    robot_x = start_x + (end_x - start_x) * eased
    robot_y = start_y + (end_y - start_y) * eased
    robot_vx_field = ((end_x - start_x) / 1000.0) * 0.5 * math.pi / duration_s * math.sin(math.pi * progress)
    robot_vy_field = ((end_y - start_y) / 1000.0) * 0.5 * math.pi / duration_s * math.sin(math.pi * progress)
    heading_deg = point_heading_deg(robot_x, robot_y, end_x, end_y)
    robot_vx_body, robot_vy_body = field_to_body(robot_vx_field, robot_vy_field, heading_deg)
    robot_ax_field = ((end_x - start_x) / 1000.0) * 0.5 * (math.pi / duration_s) ** 2 * math.cos(math.pi * progress)
    robot_ay_field = ((end_y - start_y) / 1000.0) * 0.5 * (math.pi / duration_s) ** 2 * math.cos(math.pi * progress)
    robot_ax_body, robot_ay_body = field_to_body(robot_ax_field, robot_ay_field, heading_deg)

    ball_x, ball_y = centered_cm_to_mm(25.0, 15.0)
    ball_vx_field = -0.03
    ball_vy_field = 0.015
    ball_x += ball_vx_field * 1000.0 * t_s
    ball_y += ball_vy_field * 1000.0 * t_s
    ball_body_x, ball_body_y = field_to_body_position(ball_x - robot_x, ball_y - robot_y, heading_deg)
    ball_vx_body, ball_vy_body = field_to_body(ball_vx_field, ball_vy_field, heading_deg)

    path_points = [(robot_x, robot_y), ((robot_x + end_x) * 0.5, (robot_y + end_y) * 0.5), (end_x, end_y)]
    return make_snapshot(
        timestamp_ns=now_timestamp_ns(),
        pose={
            "valid": True,
            "heading_deg": heading_deg,
            "x_mm": robot_x,
            "y_mm": robot_y,
            "vx_mm_s": robot_vx_field * 1000.0,
            "vy_mm_s": robot_vy_field * 1000.0,
            "vx_body_m_s": robot_vx_body,
            "vy_body_m_s": robot_vy_body,
        },
        ball={
            "visible": True,
            "field_visible": True,
            "body_x_m": ball_body_x,
            "body_y_m": ball_body_y,
            "body_vx_m_s": ball_vx_body,
            "body_vy_m_s": ball_vy_body,
            "field_x_mm": ball_x,
            "field_y_mm": ball_y,
            "field_vx_m_s": ball_vx_field,
            "field_vy_m_s": ball_vy_field,
            "vision_angle_deg": point_heading_deg(robot_x, robot_y, ball_x, ball_y),
            "vision_dist_cal": math.hypot(ball_x - robot_x, ball_y - robot_y) / 1000.0,
        },
        planner_path=make_planner_path(
            trajectory_id=int(t_s * 2.0) + 1,
            target_x_mm=end_x,
            target_y_mm=end_y,
            target_heading_deg=heading_deg,
            points=path_points,
        ),
        robot_twist=make_robot_twist(robot_vx_body, robot_vy_body, robot_vx_field, robot_vy_field),
        robot_accel=make_robot_accel(robot_ax_body, robot_ay_body, robot_ax_field, robot_ay_field),
        robot_angular={"heading_deg": heading_deg, "omega_deg_s": 0.0, "alpha_deg_s2": 0.0},
        ball_twist=make_ball_twist(True, ball_vx_body, ball_vy_body, ball_vx_field, ball_vy_field),
        log_message=f"{label}: straight_line progress={progress:.2f}",
    )


def build_idle_snapshot(t_s: float, label: str) -> dict[str, Any]:
    robot_x, robot_y = centered_cm_to_mm(0.0, 0.0)
    ball_x, ball_y = centered_cm_to_mm(25.0, -10.0)
    heading_deg = 0.0
    ball_body_x, ball_body_y = field_to_body_position(ball_x - robot_x, ball_y - robot_y, heading_deg)
    return make_snapshot(
        timestamp_ns=now_timestamp_ns(),
        pose={
            "valid": True,
            "heading_deg": heading_deg,
            "x_mm": robot_x,
            "y_mm": robot_y,
            "vx_mm_s": 0.0,
            "vy_mm_s": 0.0,
            "vx_body_m_s": 0.0,
            "vy_body_m_s": 0.0,
        },
        ball={
            "visible": True,
            "field_visible": True,
            "body_x_m": ball_body_x,
            "body_y_m": ball_body_y,
            "body_vx_m_s": 0.0,
            "body_vy_m_s": 0.0,
            "field_x_mm": ball_x,
            "field_y_mm": ball_y,
            "field_vx_m_s": 0.0,
            "field_vy_m_s": 0.0,
            "vision_angle_deg": point_heading_deg(robot_x, robot_y, ball_x, ball_y),
            "vision_dist_cal": math.hypot(ball_x - robot_x, ball_y - robot_y) / 1000.0,
        },
        planner_path=make_planner_path(
            trajectory_id=1,
            target_x_mm=ball_x - 120.0,
            target_y_mm=ball_y,
            target_heading_deg=0.0,
            points=[(robot_x, robot_y), ((robot_x + ball_x) * 0.5, (robot_y + ball_y) * 0.5), (ball_x - 120.0, ball_y)],
        ),
        robot_twist=make_robot_twist(0.0, 0.0, 0.0, 0.0),
        robot_accel=make_robot_accel(0.0, 0.0, 0.0, 0.0),
        robot_angular={"heading_deg": 0.0, "omega_deg_s": 0.0, "alpha_deg_s2": 0.0},
        ball_twist=make_ball_twist(True, 0.0, 0.0, 0.0, 0.0),
        log_message=f"{label}: idle t={t_s:.2f}s",
    )


def make_bezier_path(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    *,
    samples: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(samples):
        u = index / max(samples - 1, 1)
        one_minus = 1.0 - u
        x = (
            (one_minus**3) * p0[0]
            + 3.0 * (one_minus**2) * u * p1[0]
            + 3.0 * one_minus * (u**2) * p2[0]
            + (u**3) * p3[0]
        )
        y = (
            (one_minus**3) * p0[1]
            + 3.0 * (one_minus**2) * u * p1[1]
            + 3.0 * one_minus * (u**2) * p2[1]
            + (u**3) * p3[1]
        )
        points.append((x, y))
    return points


def make_planner_path(
    *,
    trajectory_id: int,
    target_x_mm: float,
    target_y_mm: float,
    target_heading_deg: float,
    points: list[tuple[float, float]],
) -> dict[str, Any]:
    s_values = path_length_mm(points)
    path_points = []
    for index, ((x_mm, y_mm), s_mm) in enumerate(zip(points, s_values)):
        if index + 1 < len(points):
            next_x, next_y = points[index + 1]
            heading_deg = point_heading_deg(x_mm, y_mm, next_x, next_y)
        else:
            heading_deg = target_heading_deg
        path_points.append(
            {
                "x_mm": x_mm,
                "y_mm": y_mm,
                "heading_deg": heading_deg,
                "s_mm": s_mm,
            }
        )
    return {
        "commanded_goal_mode": False,
        "within_tolerance": False,
        "used_center_fallback": False,
        "used_body_chase_fallback": False,
        "used_strike_pose_plan": True,
        "trajectory_id": trajectory_id,
        "dt_ms": 4,
        "target_x_mm": target_x_mm,
        "target_y_mm": target_y_mm,
        "target_heading_deg": target_heading_deg,
        "points": path_points,
    }


def make_robot_twist(vx_body: float, vy_body: float, vx_field: float, vy_field: float) -> dict[str, Any]:
    return {
        "vx_body_m_s": vx_body,
        "vy_body_m_s": vy_body,
        "speed_body_m_s": math.hypot(vx_body, vy_body),
        "vx_field_m_s": vx_field,
        "vy_field_m_s": vy_field,
        "speed_field_m_s": math.hypot(vx_field, vy_field),
    }


def make_robot_accel(ax_body: float, ay_body: float, ax_field: float, ay_field: float) -> dict[str, Any]:
    return {
        "ax_body_m_s2": ax_body,
        "ay_body_m_s2": ay_body,
        "magnitude_body_m_s2": math.hypot(ax_body, ay_body),
        "ax_field_m_s2": ax_field,
        "ay_field_m_s2": ay_field,
        "magnitude_field_m_s2": math.hypot(ax_field, ay_field),
    }


def make_ball_twist(
    visible: bool,
    vx_body: float,
    vy_body: float,
    vx_field: float,
    vy_field: float,
) -> dict[str, Any]:
    return {
        "visible": visible,
        "vx_body_m_s": vx_body,
        "vy_body_m_s": vy_body,
        "speed_body_m_s": math.hypot(vx_body, vy_body),
        "vx_field_m_s": vx_field,
        "vy_field_m_s": vy_field,
        "speed_field_m_s": math.hypot(vx_field, vy_field),
    }


def field_to_body(vx_field: float, vy_field: float, heading_deg: float) -> tuple[float, float]:
    radians = math.radians(heading_deg)
    c = math.cos(radians)
    s = math.sin(radians)
    return c * vx_field + s * vy_field, -s * vx_field + c * vy_field


def field_to_body_position(dx_mm: float, dy_mm: float, heading_deg: float) -> tuple[float, float]:
    vx_body, vy_body = field_to_body(dx_mm / 1000.0, dy_mm / 1000.0, heading_deg)
    return vx_body, vy_body


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fake BallAlgo runtime publisher for Foxglove testing")
    parser.add_argument(
        "--config",
        default="foxglove_sim/foxglove.conf",
        help="Path to the shared Foxglove config file.",
    )
    parser.add_argument(
        "--scenario",
        default="replan_demo",
        choices=["replan_demo", "orbit_ball", "straight_line", "idle"],
        help="Deterministic fake telemetry scenario to stream.",
    )
    parser.add_argument("--hz", type=float, default=30.0, help="Snapshot publish rate in Hz.")
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Optional finite run length. Use 0 to stream until interrupted.",
    )
    parser.add_argument("--label", default="fake-runtime", help="Scenario label for log messages.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = (repo_root / config_path).resolve()
    cfg = load_config(config_path)
    print(f"Fake runtime config_path={config_path} socket_path={cfg.socket_path}")
    runtime = FakeRuntimePublisher(
        FakeRuntimeConfig(
            socket_path=cfg.socket_path,
            scenario=args.scenario,
            hz=max(args.hz, 1.0),
            duration_s=max(args.duration_s, 0.0),
            label=args.label,
        )
    )
    return 0 if runtime.run() else 1


if __name__ == "__main__":
    raise SystemExit(main())
