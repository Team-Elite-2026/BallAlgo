from __future__ import annotations

import math
import time
from dataclasses import dataclass

try:
    import serial
except ModuleNotFoundError:  # pragma: no cover - hardware dependency
    serial = None


LOST_SENTINEL = -5


@dataclass(slots=True)
class TeensyTelemetry:
    timestamp_s: float = 0.0
    heading_deg: float = 0.0
    raw_line: str = ""

    @property
    def telemetry_fresh(self) -> bool:
        return self.timestamp_s > 0.0


def _to_float(fields: dict[str, str], key: str, default: float) -> float:
    try:
        return float(fields.get(key, default))
    except (TypeError, ValueError):
        return default


def parse_teensy_telemetry_line(line: str) -> TeensyTelemetry | None:
    """Parse a Teensy telemetry line.

    Expected wire shape:

        T,heading=1.2

    Unknown keys are ignored, but the Pi only consumes heading from Teensy.
    """
    text = line.strip()
    if not text.startswith("T,"):
        return None

    fields: dict[str, str] = {}
    for part in text[2:].split(","):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value.strip()

    return TeensyTelemetry(
        timestamp_s=time.monotonic(),
        heading_deg=_to_float(fields, "heading", 0.0),
        raw_line=text,
    )


def _wire_int(value: float | int | None) -> int:
    if value is None:
        return LOST_SENTINEL
    return int(value)


def compute_orbit_derivative(
    ball_angle_deg: float,
    ball_distance_cm: float,
    prev_ball_angle_deg: float,
    prev_ball_distance_cm: float,
) -> float:
    """Replicate the legacy camera-side derivative used by orbit.cpp."""
    if (
        ball_angle_deg == LOST_SENTINEL
        or ball_distance_cm == LOST_SENTINEL
        or prev_ball_angle_deg == LOST_SENTINEL
        or prev_ball_distance_cm == LOST_SENTINEL
    ):
        return LOST_SENTINEL

    new_ball_angle = 360 - ball_angle_deg if ball_angle_deg > 180 else ball_angle_deg
    prev_new_ball_angle = 360 - prev_ball_angle_deg if prev_ball_angle_deg > 180 else prev_ball_angle_deg
    d_input = (
        math.sin(math.radians(int(new_ball_angle))) * int(ball_distance_cm)
        - math.sin(math.radians(int(prev_new_ball_angle))) * int(prev_ball_distance_cm)
    )
    if d_input < 0 and (ball_angle_deg < 90 or ball_angle_deg > 270):
        return round((-d_input) * 100) / 100
    return LOST_SENTINEL


@dataclass(slots=True)
class OrbitDerivativeTracker:
    prev_ball_angle_deg: float = LOST_SENTINEL
    prev_ball_distance_cm: float = LOST_SENTINEL

    def update(self, ball_angle_deg: float, ball_distance_cm: float) -> float:
        derivative = compute_orbit_derivative(
            ball_angle_deg,
            ball_distance_cm,
            self.prev_ball_angle_deg,
            self.prev_ball_distance_cm,
        )
        if ball_angle_deg != LOST_SENTINEL and ball_distance_cm != LOST_SENTINEL:
            self.prev_ball_angle_deg = ball_angle_deg
            self.prev_ball_distance_cm = ball_distance_cm
        return derivative


def format_detection_packet(
    ball_angle_deg: float,
    ball_distance_cm: float,
    blue_goal_angle_deg: float,
    yellow_goal_angle_deg: float,
    derivative: float = LOST_SENTINEL,
    pose_x_mm: float = LOST_SENTINEL,
    pose_y_mm: float = LOST_SENTINEL,
) -> str:
    """Format the ASCII packet consumed by Offense2026's Cam.cpp."""
    packet = (
        f"{_wire_int(ball_angle_deg)}b"
        f"{_wire_int(ball_distance_cm)}a"
        f"{_wire_int(blue_goal_angle_deg)}c"
        f"{_wire_int(yellow_goal_angle_deg)}d"
        f"{derivative}f"
    )
    if pose_x_mm != LOST_SENTINEL or pose_y_mm != LOST_SENTINEL:
        packet += f"{_wire_int(pose_x_mm)}x{_wire_int(pose_y_mm)}y"
    return packet


class SerialLink:
    """Full-duplex ASCII serial link.

    Pi -> Teensy is the existing camera packet.  Teensy -> Pi is newline-framed
    telemetry beginning with ``T,``.
    """

    def __init__(self, port: str, baud: int, timeout: float = 0.01):
        if serial is None:
            raise RuntimeError("pyserial is not installed; cannot open Teensy serial")
        self._serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self._rx_text = ""
        self.latest_telemetry = TeensyTelemetry()
        time.sleep(0.1)
        print(f"[SERIAL] Connected on {port} @ {baud}")

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    def close(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            print("[SERIAL] Closed")

    def send_detection(
        self,
        ball_angle_deg: float,
        ball_distance_cm: float,
        blue_goal_angle_deg: float,
        yellow_goal_angle_deg: float,
        derivative: float = LOST_SENTINEL,
        pose_x_mm: float = LOST_SENTINEL,
        pose_y_mm: float = LOST_SENTINEL,
    ) -> None:
        packet = format_detection_packet(
            ball_angle_deg,
            ball_distance_cm,
            blue_goal_angle_deg,
            yellow_goal_angle_deg,
            derivative,
            pose_x_mm,
            pose_y_mm,
        )
        self._serial.write(packet.encode("ascii"))

    def poll_telemetry(self) -> TeensyTelemetry:
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._rx_text += self._serial.read(waiting).decode("ascii", errors="ignore")
        except Exception:
            return self.latest_telemetry

        while "\n" in self._rx_text:
            line, self._rx_text = self._rx_text.split("\n", 1)
            telemetry = parse_teensy_telemetry_line(line)
            if telemetry is not None:
                self.latest_telemetry = telemetry
        return self.latest_telemetry
