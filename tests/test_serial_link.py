from __future__ import annotations

import struct
import unittest
import zlib

from communication import (
    PI_CONTROL_PAYLOAD_SIZE,
    format_control_frame,
    format_detection_packet,
)
from communication.serial_link import parse_teensy_telemetry_line


class SerialLinkTests(unittest.TestCase):
    def test_detection_packet_stays_legacy_ascii(self):
        self.assertEqual(format_detection_packet(12.9, 34.2, -5, 270), "12b34a-5c270d")

    def test_control_frame_layout_and_crc(self):
        frame = format_control_frame(
            seq=7,
            pi_time_us=123456,
            ball_angle_deg=1.5,
            ball_distance_cm=22.0,
            blue_goal_angle_deg=-10.0,
            yellow_goal_angle_deg=45.0,
            ball_found=True,
            blue_goal_found=False,
            yellow_goal_found=True,
            role_command=1,
            manual_mode=0,
            offense_command=7,
            dribbler_power=255,
            kick_request=True,
        )

        self.assertEqual(PI_CONTROL_PAYLOAD_SIZE, 36)
        self.assertEqual(len(frame), 7 + PI_CONTROL_PAYLOAD_SIZE + 4)
        magic, msg_type, payload_len = struct.unpack("<IBH", frame[:7])
        self.assertEqual(magic, 0xCEFAEDFE)
        self.assertEqual(msg_type, 0x05)
        self.assertEqual(payload_len, PI_CONTROL_PAYLOAD_SIZE)

        crc_expected = zlib.crc32(frame[:-4]) & 0xFFFFFFFF
        (crc_actual,) = struct.unpack("<I", frame[-4:])
        self.assertEqual(crc_actual, crc_expected)

        payload = struct.unpack("<IQffffBBBBBBBB", frame[7:-4])
        self.assertEqual(payload[0], 7)
        self.assertEqual(payload[1], 123456)
        self.assertAlmostEqual(payload[2], 1.5)
        self.assertAlmostEqual(payload[3], 22.0)
        self.assertAlmostEqual(payload[4], -10.0)
        self.assertAlmostEqual(payload[5], 45.0)
        self.assertEqual(payload[6:], (1, 0, 1, 1, 0, 7, 255, 1))

    def test_unknown_telemetry_fields_are_ignored(self):
        telemetry = parse_teensy_telemetry_line(
            "T,heading=12.5,has_ball=1,legacy_velocity=999,legacy_omega=3.2"
        )
        self.assertIsNotNone(telemetry)
        assert telemetry is not None
        self.assertEqual(telemetry.heading_deg, 12.5)
        self.assertTrue(telemetry.has_ball)
        self.assertFalse(hasattr(telemetry, "legacy_velocity"))


if __name__ == "__main__":
    unittest.main()
