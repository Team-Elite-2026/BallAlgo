from __future__ import annotations

import unittest

from control import (
    BasicOffenseCommand,
    BasicOffenseController,
    BasicOffenseInput,
    wrap_control_angle,
)


PARAMS = {
    "ball_front_angle_deg": 90.0,
    "kick_max_scoring_goal_heading_deg": 70.0,
    "own_goal_heading_block_deg": 70.0,
    "kick_aim_tolerance_deg": 12.0,
    "spin_shot_possession_hold_ms": 90,
    "spin_shot_align_hold_ms": 80,
    "kick_cooldown_us": 750000,
    "dribbler_capture_power": 140,
    "dribbler_capture_dist_mm": 220.0,
    "spin_shot_dribbler_power": 255,
}


def active_input(**overrides):
    values = {
        "offense_active": True,
        "command_link_fresh": True,
        "has_ball": False,
        "ball_found": True,
        "ball_angle_deg": 0.0,
        "ball_distance_cm": 20.0,
        "scoring_goal_found": True,
        "scoring_goal_angle_deg": 0.0,
        "own_goal_found": False,
        "own_goal_angle_deg": -5.0,
    }
    values.update(overrides)
    return BasicOffenseInput(**values)


class BasicOffenseTests(unittest.TestCase):
    def test_wrap_control_angle_preserves_lost_and_wraps(self):
        self.assertEqual(wrap_control_angle(-5), -5.0)
        self.assertEqual(wrap_control_angle(270), -90.0)
        self.assertEqual(wrap_control_angle(181), -179.0)
        self.assertEqual(wrap_control_angle(180), 180.0)

    def test_inactive_idles_and_clears_timers(self):
        controller = BasicOffenseController(PARAMS)
        out = controller.update(1000, active_input(offense_active=False, has_ball=True))
        self.assertEqual(out.command, BasicOffenseCommand.Idle)
        self.assertEqual(out.dribbler_power, 0)
        self.assertFalse(out.kick_request)
        self.assertIsNone(controller.possession_since_us)

    def test_no_ball_searches(self):
        controller = BasicOffenseController(PARAMS)
        out = controller.update(1000, active_input(ball_found=False, ball_angle_deg=-5, ball_distance_cm=-5))
        self.assertEqual(out.command, BasicOffenseCommand.SearchBall)
        self.assertEqual(out.dribbler_power, 0)

    def test_ball_behind_turns_to_ball(self):
        controller = BasicOffenseController(PARAMS)
        out = controller.update(1000, active_input(ball_angle_deg=120.0))
        self.assertEqual(out.command, BasicOffenseCommand.TurnToBehindBall)
        self.assertEqual(out.dribbler_power, 140)

    def test_ball_in_front_and_scoring_goal_in_front_captures_fast(self):
        controller = BasicOffenseController(PARAMS)
        out = controller.update(1000, active_input(ball_angle_deg=25.0, scoring_goal_angle_deg=15.0))
        self.assertEqual(out.command, BasicOffenseCommand.CaptureFast)
        self.assertEqual(out.dribbler_power, 140)

    def test_far_ball_chase_does_not_run_dribbler(self):
        controller = BasicOffenseController(PARAMS)
        out = controller.update(1000, active_input(ball_distance_cm=80.0, scoring_goal_angle_deg=120.0))
        self.assertEqual(out.command, BasicOffenseCommand.ChaseBall)
        self.assertEqual(out.dribbler_power, 0)

    def test_own_goal_blocks_capture_fast(self):
        controller = BasicOffenseController(PARAMS)
        out = controller.update(1000, active_input(own_goal_found=True, own_goal_angle_deg=0.0))
        self.assertEqual(out.command, BasicOffenseCommand.ChaseBall)

    def test_has_ball_spins_until_possession_and_aim_are_stable(self):
        controller = BasicOffenseController(PARAMS)
        first = controller.update(0, active_input(has_ball=True, scoring_goal_angle_deg=6.0))
        self.assertEqual(first.command, BasicOffenseCommand.SpinAlignLeft)
        self.assertEqual(first.dribbler_power, 255)
        self.assertFalse(first.kick_request)

        almost = controller.update(70000, active_input(has_ball=True, scoring_goal_angle_deg=6.0))
        self.assertEqual(almost.command, BasicOffenseCommand.SpinAlignLeft)
        self.assertFalse(almost.kick_request)

        kick = controller.update(100000, active_input(has_ball=True, scoring_goal_angle_deg=6.0))
        self.assertEqual(kick.command, BasicOffenseCommand.CommitKick)
        self.assertTrue(kick.kick_request)
        self.assertEqual(kick.dribbler_power, 0)

    def test_own_goal_blocks_kick_even_when_aimed(self):
        controller = BasicOffenseController(PARAMS)
        controller.update(
            0,
            active_input(has_ball=True, scoring_goal_angle_deg=0.0, own_goal_found=True, own_goal_angle_deg=0.0),
        )
        out = controller.update(
            200000,
            active_input(has_ball=True, scoring_goal_angle_deg=0.0, own_goal_found=True, own_goal_angle_deg=0.0),
        )
        self.assertEqual(out.command, BasicOffenseCommand.SpinAlignLeft)
        self.assertFalse(out.kick_request)

    def test_kick_is_one_frame_then_cooldown(self):
        controller = BasicOffenseController(PARAMS)
        controller.update(0, active_input(has_ball=True, scoring_goal_angle_deg=0.0))
        kick = controller.update(200000, active_input(has_ball=True, scoring_goal_angle_deg=0.0))
        self.assertEqual(kick.command, BasicOffenseCommand.CommitKick)
        self.assertTrue(kick.kick_request)

        cooldown = controller.update(201000, active_input(has_ball=True, scoring_goal_angle_deg=0.0))
        self.assertEqual(cooldown.command, BasicOffenseCommand.KickCooldown)
        self.assertFalse(cooldown.kick_request)
        self.assertEqual(cooldown.dribbler_power, 140)

    def test_command_link_must_be_fresh_to_kick(self):
        controller = BasicOffenseController(PARAMS)
        controller.update(0, active_input(has_ball=True, command_link_fresh=False, scoring_goal_angle_deg=0.0))
        out = controller.update(200000, active_input(has_ball=True, command_link_fresh=False, scoring_goal_angle_deg=0.0))
        self.assertNotEqual(out.command, BasicOffenseCommand.CommitKick)
        self.assertFalse(out.kick_request)


if __name__ == "__main__":
    unittest.main()
