from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

try:
    import serial
except ModuleNotFoundError:  # pragma: no cover - hardware dependency
    serial = None


LOST_SENTINEL = -5


@dataclass(slots=True)
class TeensyTelemetry:
    timestamp_s: float = 0.0
    heading_deg: float = 0.0
    has_ball: bool = False
    start_enabled: bool = False
    goal_is_blue: bool = True
    robot_mode: str = "unknown"
    line_angle_deg: float = LOST_SENTINEL
    avoidance_angle_deg: float = LOST_SENTINEL
    chord_length: float = LOST_SENTINEL
    cross_line: bool = False
    ball_angle_deg: float = LOST_SENTINEL
    ball_distance_cm: float = LOST_SENTINEL
    blue_goal_angle_deg: float = LOST_SENTINEL
    yellow_goal_angle_deg: float = LOST_SENTINEL
    camera_fresh: bool = False
    mouse_vx_body_mm_s: float = 0.0
    mouse_vy_body_mm_s: float = 0.0
    omega_rad_s: float = 0.0
    raw_line: str = ""

    @property
    def telemetry_fresh(self) -> bool:
        return self.timestamp_s > 0.0


def _to_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "on", "high"}


def _to_float(fields: dict[str, str], key: str, default: float) -> float:
    try:
        return float(fields.get(key, default))
    except (TypeError, ValueError):
        return default


def _to_str(fields: dict[str, str], key: str, default: str) -> str:
    value = fields.get(key)
    return default if value is None else value


def parse_teensy_telemetry_line(line: str) -> TeensyTelemetry | None:
    """Parse a Teensy telemetry line.

    Expected wire shape:

        T,heading=1.2,has_ball=0,start=1,goal_blue=1,mode=offense,...

    Unknown keys are ignored so the Teensy can add fields without breaking the
    Pi runtime.
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
        has_ball=_to_bool(fields.get("has_ball", "0")),
        start_enabled=_to_bool(fields.get("start", "0")),
        goal_is_blue=_to_bool(fields.get("goal_blue", "1")),
        robot_mode=_to_str(fields, "mode", "unknown"),
        line_angle_deg=_to_float(fields, "line", LOST_SENTINEL),
        avoidance_angle_deg=_to_float(fields, "avoid", LOST_SENTINEL),
        chord_length=_to_float(fields, "chord", LOST_SENTINEL),
        cross_line=_to_bool(fields.get("cross", "0")),
        ball_angle_deg=_to_float(fields, "ball", LOST_SENTINEL),
        ball_distance_cm=_to_float(fields, "dist", LOST_SENTINEL),
        blue_goal_angle_deg=_to_float(fields, "blue", LOST_SENTINEL),
        yellow_goal_angle_deg=_to_float(fields, "yellow", LOST_SENTINEL),
        camera_fresh=_to_bool(fields.get("cam_fresh", "0")),
        mouse_vx_body_mm_s=_to_float(fields, "mouse_vx", 0.0),
        mouse_vy_body_mm_s=_to_float(fields, "mouse_vy", 0.0),
        omega_rad_s=_to_float(fields, "omega", 0.0),
        raw_line=text,
    )


def _wire_int(value: float | int | None) -> int:
    if value is None:
        return LOST_SENTINEL
    return int(value)


def format_detection_packet(
    ball_angle_deg: float,
    ball_distance_cm: float,
    blue_goal_angle_deg: float,
    yellow_goal_angle_deg: float,
) -> str:
    """Format the ASCII packet consumed by Offense2026's Cam.cpp."""
    return (
        f"{_wire_int(ball_angle_deg)}b"
        f"{_wire_int(ball_distance_cm)}a"
        f"{_wire_int(blue_goal_angle_deg)}c"
        f"{_wire_int(yellow_goal_angle_deg)}d"
    )


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
    ) -> None:
        packet = format_detection_packet(
            ball_angle_deg,
            ball_distance_cm,
            blue_goal_angle_deg,
            yellow_goal_angle_deg,
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
