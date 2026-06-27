from __future__ import annotations

import unittest

from communication import (
    OrbitDerivativeTracker,
    compute_orbit_derivative,
    format_detection_packet,
)
from communication.serial_link import parse_teensy_telemetry_line


class SerialLinkTests(unittest.TestCase):
    def test_detection_packet_stays_legacy_ascii(self):
        self.assertEqual(format_detection_packet(12.9, 34.2, -5, 270), "12b34a-5c270d-5f")

    def test_detection_packet_can_append_pose_ascii(self):
        self.assertEqual(
            format_detection_packet(12.9, 34.2, -5, 270, -5, 101.6, 202.4),
            "12b34a-5c270d-5f101x202y",
        )

    def test_compute_orbit_derivative_matches_legacy_camera_logic(self):
        self.assertEqual(compute_orbit_derivative(350, 20, 340, 30), 6.79)
        self.assertEqual(compute_orbit_derivative(300, 40, 320, 30), -5)
        self.assertEqual(compute_orbit_derivative(-5, 20, 340, 30), -5)

    def test_orbit_derivative_tracker_only_updates_on_valid_ball(self):
        tracker = OrbitDerivativeTracker()

        self.assertEqual(tracker.update(340, 30), -5)
        self.assertEqual(tracker.prev_ball_angle_deg, 340)
        self.assertEqual(tracker.prev_ball_distance_cm, 30)

        self.assertEqual(tracker.update(350, 20), 6.79)
        self.assertEqual(tracker.update(-5, -5), -5)
        self.assertEqual(tracker.prev_ball_angle_deg, 350)
        self.assertEqual(tracker.prev_ball_distance_cm, 20)

    def test_unknown_telemetry_fields_are_ignored(self):
        telemetry = parse_teensy_telemetry_line(
            "T,heading=12.5,ignored=1,legacy_velocity=999,legacy_omega=3.2"
        )
        self.assertIsNotNone(telemetry)
        assert telemetry is not None
        self.assertEqual(telemetry.heading_deg, 12.5)
        self.assertEqual(telemetry.raw_line, "T,heading=12.5,ignored=1,legacy_velocity=999,legacy_omega=3.2")
        self.assertFalse(hasattr(telemetry, "ignored"))
        self.assertFalse(hasattr(telemetry, "legacy_velocity"))

    def test_parse_teensy_heading_telemetry_line(self):
        telemetry = parse_teensy_telemetry_line(
            "T,heading=42"
        )
        self.assertIsNotNone(telemetry)
        assert telemetry is not None
        self.assertEqual(telemetry.heading_deg, 42)
        self.assertEqual(telemetry.raw_line, "T,heading=42")


if __name__ == "__main__":
    unittest.main()
