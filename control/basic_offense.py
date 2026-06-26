from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping


LOST_SENTINEL = -5.0
ANGLE_EPSILON_DEG = 1e-3


class BasicOffenseCommand(IntEnum):
    Idle = 0
    ChaseBall = 2
    TurnToBehindBall = 3
    CaptureFast = 4
    SpinAlignLeft = 5
    SpinAlignRight = 6
    CommitKick = 7
    KickCooldown = 8


@dataclass(slots=True)
class BasicOffenseInput:
    offense_active: bool = False
    command_link_fresh: bool = False
    has_ball: bool = False
    ball_found: bool = False
    ball_angle_deg: float = LOST_SENTINEL
    ball_distance_cm: float = LOST_SENTINEL
    scoring_goal_found: bool = False
    scoring_goal_angle_deg: float = LOST_SENTINEL
    own_goal_found: bool = False
    own_goal_angle_deg: float = LOST_SENTINEL


@dataclass(slots=True)
class BasicOffenseOutput:
    command: BasicOffenseCommand = BasicOffenseCommand.Idle
    dribbler_power: int = 0
    kick_request: bool = False
    preferred_orbit_dir: int = 1
    ball_in_front: bool = False
    facing_scoring_goal: bool = False
    facing_own_goal: bool = False
    possession_stable: bool = False
    aim_stable: bool = False
    cooldown_active: bool = False

    @property
    def command_name(self) -> str:
        return self.command.name

    @classmethod
    def disabled(cls) -> "BasicOffenseOutput":
        return cls(command=BasicOffenseCommand.Idle, dribbler_power=0, kick_request=False)


def _param(params: Mapping[str, Any], key: str, default: float) -> float:
    try:
        return float(params.get(key, default))
    except (TypeError, ValueError):
        return default


def _param_int(params: Mapping[str, Any], key: str, default: int) -> int:
    try:
        return int(params.get(key, default))
    except (TypeError, ValueError):
        return default


def _valid_angle(angle_deg: float) -> bool:
    return angle_deg != LOST_SENTINEL


def _clamp_power(value: int) -> int:
    return max(0, min(255, int(value)))


def wrap_control_angle(angle_deg: float | int | None) -> float:
    """Convert valid camera angles to -180..180 while preserving the lost sentinel."""
    if angle_deg is None:
        return LOST_SENTINEL
    angle = float(angle_deg)
    if angle == LOST_SENTINEL:
        return LOST_SENTINEL
    wrapped = angle % 360.0
    if wrapped > 180.0:
        wrapped -= 360.0
    return wrapped


class BasicOffenseController:
    """Small Pi-side offense state machine.

    This computes intent only. The Teensy still owns wheel control, dribbler
    motor behavior, and the physical one-shot kicker pulse.
    """

    def __init__(self, params: Mapping[str, Any] | None = None):
        self.params = params or {}
        self.preferred_orbit_dir = 1
        self.possession_since_us: int | None = None
        self.aim_since_us: int | None = None
        self.last_kick_us: int | None = None

    def reset(self) -> None:
        self.preferred_orbit_dir = 1
        self.possession_since_us = None
        self.aim_since_us = None
        self.last_kick_us = None

    def update(self, now_us: int, offense_input: BasicOffenseInput) -> BasicOffenseOutput:
        ball_angle = wrap_control_angle(offense_input.ball_angle_deg)
        scoring_goal_angle = wrap_control_angle(offense_input.scoring_goal_angle_deg)
        own_goal_angle = wrap_control_angle(offense_input.own_goal_angle_deg)

        ball_found = bool(offense_input.ball_found and _valid_angle(ball_angle))
        scoring_goal_found = bool(offense_input.scoring_goal_found and _valid_angle(scoring_goal_angle))
        own_goal_found = bool(offense_input.own_goal_found and _valid_angle(own_goal_angle))

        ball_front_angle = _param(self.params, "ball_front_angle_deg", 90.0)
        kick_goal_cone = _param(self.params, "kick_max_scoring_goal_heading_deg", 70.0)
        own_goal_block = _param(self.params, "own_goal_heading_block_deg", 70.0)
        kick_aim_tolerance = _param(self.params, "kick_aim_tolerance_deg", 12.0)

        ball_in_front = ball_found and abs(ball_angle) <= ball_front_angle
        facing_scoring_goal = scoring_goal_found and abs(scoring_goal_angle) <= kick_goal_cone
        facing_own_goal = own_goal_found and abs(own_goal_angle) <= own_goal_block

        if not offense_input.offense_active:
            self.reset()
            return BasicOffenseOutput(
                command=BasicOffenseCommand.Idle,
                preferred_orbit_dir=self.preferred_orbit_dir,
                ball_in_front=ball_in_front,
                facing_scoring_goal=facing_scoring_goal,
                facing_own_goal=facing_own_goal,
            )

        cooldown_output = self._cooldown_output(now_us, ball_in_front, facing_scoring_goal, facing_own_goal)
        if cooldown_output is not None:
            return cooldown_output

        self._update_preferred_orbit(
            ball_found=ball_found,
            ball_angle_deg=ball_angle,
            scoring_goal_found=scoring_goal_found,
            scoring_goal_angle_deg=scoring_goal_angle,
        )

        if not offense_input.has_ball:
            self.possession_since_us = None
            self.aim_since_us = None
            return self._without_possession(
                offense_input,
                ball_found=ball_found,
                ball_in_front=ball_in_front,
                facing_scoring_goal=facing_scoring_goal,
                facing_own_goal=facing_own_goal,
            )

        return self._with_possession(
            now_us,
            offense_input,
            ball_in_front=ball_in_front,
            facing_scoring_goal=facing_scoring_goal,
            facing_own_goal=facing_own_goal,
            scoring_goal_found=scoring_goal_found,
            scoring_goal_angle_deg=scoring_goal_angle,
            kick_aim_tolerance=kick_aim_tolerance,
        )

    def _without_possession(
        self,
        offense_input: BasicOffenseInput,
        *,
        ball_found: bool,
        ball_in_front: bool,
        facing_scoring_goal: bool,
        facing_own_goal: bool,
    ) -> BasicOffenseOutput:
        dribbler_power = 0
        if self._ball_close_for_capture(offense_input):
            dribbler_power = _clamp_power(_param_int(self.params, "dribbler_capture_power", 140))

        if not ball_found:
            command = BasicOffenseCommand.Idle
            dribbler_power = 0
        elif not ball_in_front:
            command = BasicOffenseCommand.TurnToBehindBall
        elif facing_scoring_goal and not facing_own_goal:
            command = BasicOffenseCommand.CaptureFast
        else:
            command = BasicOffenseCommand.ChaseBall

        return BasicOffenseOutput(
            command=command,
            dribbler_power=dribbler_power,
            kick_request=False,
            preferred_orbit_dir=self.preferred_orbit_dir,
            ball_in_front=ball_in_front,
            facing_scoring_goal=facing_scoring_goal,
            facing_own_goal=facing_own_goal,
        )

    def _with_possession(
        self,
        now_us: int,
        offense_input: BasicOffenseInput,
        *,
        ball_in_front: bool,
        facing_scoring_goal: bool,
        facing_own_goal: bool,
        scoring_goal_found: bool,
        scoring_goal_angle_deg: float,
        kick_aim_tolerance: float,
    ) -> BasicOffenseOutput:
        if self.possession_since_us is None:
            self.possession_since_us = now_us

        align_hold_us = _param_int(self.params, "spin_shot_align_hold_ms", 80) * 1000
        possession_stable = True
        aim_geometry_ok = (
            scoring_goal_found
            and facing_scoring_goal
            and abs(scoring_goal_angle_deg) <= kick_aim_tolerance
            and not facing_own_goal
        )

        if aim_geometry_ok:
            if self.aim_since_us is None:
                self.aim_since_us = now_us
        else:
            self.aim_since_us = None
        aim_stable = self.aim_since_us is not None and now_us - self.aim_since_us >= align_hold_us

        can_kick = (
            offense_input.offense_active
            and offense_input.command_link_fresh
            and offense_input.has_ball
            and aim_geometry_ok
            and aim_stable
        )
        if can_kick:
            self.last_kick_us = now_us
            self.aim_since_us = None
            return BasicOffenseOutput(
                command=BasicOffenseCommand.CommitKick,
                dribbler_power=0,
                kick_request=True,
                preferred_orbit_dir=self.preferred_orbit_dir,
                ball_in_front=ball_in_front,
                facing_scoring_goal=facing_scoring_goal,
                facing_own_goal=facing_own_goal,
                possession_stable=possession_stable,
                aim_stable=aim_stable,
            )

        return BasicOffenseOutput(
            command=self._spin_command(),
            dribbler_power=_clamp_power(_param_int(self.params, "spin_shot_dribbler_power", 255)),
            kick_request=False,
            preferred_orbit_dir=self.preferred_orbit_dir,
            ball_in_front=ball_in_front,
            facing_scoring_goal=facing_scoring_goal,
            facing_own_goal=facing_own_goal,
            possession_stable=possession_stable,
            aim_stable=aim_stable,
        )

    def _cooldown_output(
        self,
        now_us: int,
        ball_in_front: bool,
        facing_scoring_goal: bool,
        facing_own_goal: bool,
    ) -> BasicOffenseOutput | None:
        if self.last_kick_us is None:
            return None
        cooldown_us = _param_int(self.params, "kick_cooldown_us", 750000)
        if now_us - self.last_kick_us >= cooldown_us:
            return None
        return BasicOffenseOutput(
            command=BasicOffenseCommand.KickCooldown,
            dribbler_power=0,
            kick_request=False,
            preferred_orbit_dir=self.preferred_orbit_dir,
            ball_in_front=ball_in_front,
            facing_scoring_goal=facing_scoring_goal,
            facing_own_goal=facing_own_goal,
            cooldown_active=True,
        )

    def _ball_close_for_capture(self, offense_input: BasicOffenseInput) -> bool:
        if not offense_input.ball_found or offense_input.ball_distance_cm == LOST_SENTINEL:
            return False
        capture_dist_mm = _param(self.params, "dribbler_capture_dist_mm", 220.0)
        return offense_input.ball_distance_cm * 10.0 <= capture_dist_mm

    def _spin_command(self) -> BasicOffenseCommand:
        return (
            BasicOffenseCommand.SpinAlignLeft
            if self.preferred_orbit_dir >= 0
            else BasicOffenseCommand.SpinAlignRight
        )

    def _update_preferred_orbit(
        self,
        *,
        ball_found: bool,
        ball_angle_deg: float,
        scoring_goal_found: bool,
        scoring_goal_angle_deg: float,
    ) -> None:
        if scoring_goal_found and abs(scoring_goal_angle_deg) > ANGLE_EPSILON_DEG:
            self.preferred_orbit_dir = 1 if scoring_goal_angle_deg > 0.0 else -1
            return
        if ball_found and abs(ball_angle_deg) > ANGLE_EPSILON_DEG:
            self.preferred_orbit_dir = 1 if ball_angle_deg > 0.0 else -1
