from dataclasses import dataclass
import math

import serial


CRC_TABLE = (
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8
)


@dataclass(frozen=True)
class LidarPoint:
    distance_mm: int
    intensity: int
    angle_cd: int


class LidarLocalizer:
    def __init__(self, field_width_mm, field_height_mm, lidar_yaw_offset_deg=0.0):
        self.field_width_mm = field_width_mm
        self.field_height_mm = field_height_mm
        self.lidar_yaw_offset_deg = lidar_yaw_offset_deg
        self.min_distance_mm = 80.0
        self.max_distance_mm = 6000.0
        self.min_intensity = 20

    def update(self, points, heading_deg):
        x_estimates = []
        y_estimates = []

        for point in points:
            distance = float(point.distance_mm)
            if distance < self.min_distance_mm or distance > self.max_distance_mm or point.intensity < self.min_intensity:
                continue

            lidar_angle_deg = point.angle_cd * 0.01
            field_angle_deg = heading_deg + self.lidar_yaw_offset_deg + lidar_angle_deg
            radians = math.radians(field_angle_deg)
            dx = math.cos(radians)
            dy = math.sin(radians)

            if abs(dx) >= abs(dy):
                if abs(dx) < 1e-4:
                    continue
                x_est = (self.field_width_mm - distance * dx) if dx > 0.0 else (-distance * dx)
                if -200.0 < x_est < (self.field_width_mm + 200.0):
                    x_estimates.append(x_est)
            else:
                if abs(dy) < 1e-4:
                    continue
                y_est = (self.field_height_mm - distance * dy) if dy > 0.0 else (-distance * dy)
                if -200.0 < y_est < (self.field_height_mm + 200.0):
                    y_estimates.append(y_est)

        valid = (len(x_estimates) >= 4 and len(y_estimates) >= 4)
        if not valid:
            return {"valid": False, "x_mm": None, "y_mm": None, "x_samples": len(x_estimates), "y_samples": len(y_estimates)}

        x_mm = self._clamp(self._robust_axis_estimate(x_estimates), 0.0, self.field_width_mm)
        y_mm = self._clamp(self._robust_axis_estimate(y_estimates), 0.0, self.field_height_mm)
        return {"valid": True, "x_mm": x_mm, "y_mm": y_mm, "x_samples": len(x_estimates), "y_samples": len(y_estimates)}

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def _median(values):
        if not values:
            return None
        ordered = sorted(values)
        mid = len(ordered) // 2
        if len(ordered) % 2 == 0:
            return 0.5 * (ordered[mid - 1] + ordered[mid])
        return ordered[mid]

    def _robust_axis_estimate(self, estimates):
        med = self._median(estimates)
        if med is None:
            return None
        filtered = [value for value in estimates if abs(value - med) < 180.0]
        if len(filtered) >= 3:
            return self._median(filtered)
        return med


def ld19_crc8(payload):
    crc = 0
    for byte_value in payload:
        crc = CRC_TABLE[(crc ^ byte_value) & 0xFF]
    return crc


class LD19Reader:
    FRAME_LEN = 47  # 0x54 + verlen + 45 trailing bytes

    def __init__(self, port, baud, timeout):
        self._serial = None
        self._buffer = bytearray()
        try:
            self._serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            print(f"[LIDAR] Connected on {port} @ {baud}")
        except Exception as exc:
            print(f"[LIDAR] Failed to open serial: {exc}")

    @property
    def is_connected(self):
        return self._serial is not None

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None
            print("[LIDAR] Closed")

    def poll_points(self):
        if self._serial is None:
            return []

        points = []
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._buffer.extend(self._serial.read(waiting))
        except Exception:
            return []

        while True:
            if len(self._buffer) < self.FRAME_LEN:
                break
            try:
                start = self._buffer.index(0x54)
            except ValueError:
                self._buffer.clear()
                break

            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < self.FRAME_LEN:
                break
            if self._buffer[1] != 0x2C:
                del self._buffer[0]
                continue

            frame = bytes(self._buffer[:self.FRAME_LEN])
            del self._buffer[:self.FRAME_LEN]
            frame_points = self._parse_frame(frame)
            if frame_points:
                points.extend(frame_points)
        return points

    def _parse_frame(self, frame):
        if len(frame) != self.FRAME_LEN:
            return []
        if frame[0] != 0x54 or frame[1] != 0x2C:
            return []
        if ld19_crc8(frame[:-1]) != frame[-1]:
            return []

        start_angle = frame[4] | (frame[5] << 8)
        end_angle = frame[42] | (frame[43] << 8)
        step = self._angle_step(start_angle, end_angle, 11)

        points = []
        base = 6
        for i in range(12):
            idx = base + i * 3
            distance = frame[idx] | (frame[idx + 1] << 8)
            intensity = frame[idx + 2]
            angle_cd = (start_angle + step * i) % 36000
            points.append(LidarPoint(distance_mm=distance, intensity=intensity, angle_cd=angle_cd))
        return points

    @staticmethod
    def _angle_step(start_angle, end_angle, len_minus_one):
        if start_angle <= end_angle:
            return (end_angle - start_angle) // len_minus_one
        return (36000 + end_angle - start_angle) // len_minus_one
