# Other Team Shooting And Ball-Control Strategies

This document describes the active behavior code in:

- `other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py`
- `other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py`

It focuses on:

- how they decide which attack/shoot behavior to use
- whether they use a state machine
- what spin-shot or spin-assisted shooting behaviors they have
- whether they use ball-hiding, shielding, or backward-movement style behaviors

Everything here is based on the active code, not on legacy variants.

## Short answer

Yes, they do have a real state-machine-style attack system.

In the active attacker, the main states are:

- `SHOOT`
- `BALL_DIRECT`
- `BALL_NORTH`
- `NO_BALL`
- `EDGE_TRICK`
- `DUEL`

These are defined in [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:27).

They also have multiple distinct shot-related strategies:

- normal direct shooting
- spin-assisted turn-around-ball shooting
- an edge/wall trick shot
- a short scripted duel move around a robot in front
- a manual kickoff shot

They do have some shielding-like and safety-like motion, but they do **not** have a sophisticated explicit "ball hiding" model. There is no dedicated state that says "shield the ball from the opponent" or "keep the ball on the far side of the body" in a geometric, opponent-aware way.

What they do have instead is:

- approach-from-behind behavior
- side-wall trick behavior
- a brief backward retreat before kicking in some positions
- sidestep-around-opponent behavior during duels

## State machine structure

The attacker uses a hand-written state machine inside `on_update()`. It is not a formal framework, but it is absolutely state-machine logic.

The top-level decision tree is in:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:227)

The core branches are:

1. If the tracker still sees the ball:
2. If the robot currently has the ball in the front slot:
3. If an enemy is in front long enough, run `DUEL`
4. Else if the robot is in a side/back zone and heading toward the edge, run `EDGE_TRICK`
5. Else run `SHOOT`
6. If the robot sees the ball but does not have it:
7. If it is near the edge/back zone, run `BALL_DIRECT`
8. Else run `BALL_NORTH`
9. If it does not see the ball, run `NO_BALL`

This is the exact high-level selection policy in the active attacker.

## Attacker strategies

### 1. `SHOOT`: default has-ball finishing behavior

This is the normal finishing strategy when:

- the ball is seen
- the robot has the ball in the front dribbler slot
- there is no duel currently triggered
- they are not doing the edge trick

State selection:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:227)

Behavior implementation:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:403)

#### How it aims

If the front camera sees the goal and the robot is not in a corner, it aims using the camera goal center angle:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:412)

Otherwise it falls back to a lidar/tracker-based estimate of the vector from current robot position to the center of the opponent goal:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:407)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:415)

#### How it moves while shooting

This is not just a straight kick.

They call `get_turn_around_ball_movement(angle)`, which adds a sideways and rotational motion around the ball while aligning to the target angle:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:421)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:689)

That helper:

- normalizes the target angle
- chooses a sign for rotation
- moves laterally around the ball using `TURNING_BALL_RADIUS = 120`
- chooses spin from `TURNING_BALL_SPINS = [0, 150, 200, 200]`

Constants:

- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:153)

So their normal shot is better described as:

- drive toward shooting alignment
- rotate around the ball while holding it
- accelerate into the shot once alignment is good enough

#### When it actually kicks

They keep a `shoot_timer`. If the target angle is within 17 degrees, they ramp speed toward `SHOOT_SPEED = 1000`. If the angle is not within 17 degrees, they reset the timer and keep adjusting.

Constants:

- `SHOOT_SPEED_TURNING = 500`
- `SHOOT_SPEED = 1000`
- `SHOOT_SPEED_TIME = 1` ms in the file, but because `Timer.get()` returns milliseconds, that literal `1` really means 1 ms

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:422)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:424)
- [timer.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/utils/timer.py:10)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:167)

They kick when either:

- `shoot_timer > SHOOT_SPEED_TIME`, or
- they are fairly close to goal (`y < 700`) and alignment is within 17 degrees

This is in:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:434)

Important note:

- because `SHOOT_SPEED_TIME` is set to `1`, the timer-based condition is effectively almost immediate
- in practice, this makes the angle threshold the more meaningful condition

### 2. `EDGE_TRICK`: side-wall / spin trick shot

This is their most specialized shot behavior.

State selection:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:243)

Behavior implementation:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:631)

It is chosen when:

- the robot has the ball
- it is in a side sector, meaning `x < SIDE_SECTOR_WIDTH` or `x > FIELD_SIZE.x - SIDE_SECTOR_WIDTH`
- and it is already roughly facing the side edge

The heading condition is:

- left half: heading near 270 degrees
- right half: heading near 90 degrees
- tolerance `EDGE_TRICK_ANGLE_THRES = 60`

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:232)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:187)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:216)

#### How the edge trick works

It has two phases.

Phase 1:

- move toward a fixed side "trick line"
- rotate toward a target side-facing angle of `+/- 90` degrees
- use `EDGE_TRICK_SPEED = 350`
- use spin `EDGE_TRICK_ANGLE_SPIN = 30`

Phase 2:

- once close enough to goal depth (`y < EDGE_TRICK_KICK_DIST`, where `EDGE_TRICK_KICK_DIST = 500`)
- stop forward motion
- spin toward the goal using `EDGE_TRICK_GOAL_SPIN = 60`
- when the goal-facing angular error is below `EDGE_TRICK_GOAL_THRES = 10`, kick

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:634)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:645)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:647)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:650)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:663)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:187)

This is the clearest example of a spin-shot style routine in the code.

### 3. `DUEL`: short scripted move around a robot in front

This is not exactly a shot type, but it is a possession-preserving attack maneuver that can end in a kick.

The attacker watches for a detected robot within 100 mm of a point 100 mm in front of itself:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:189)

If that condition persists for at least `DUEL_TIME_THRES = 500` ms, it starts a 3-step `Procedure`:

1. hold position briefly with dribbler on
2. move at angle offset `+/- 45` degrees around the detected robot with spin `+/- 300`
3. request a kick

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:113)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:239)
- [procedure.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/utils/procedure.py:3)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:196)

This is basically:

- opponent in front
- sidestep around it
- then kick

### 4. Manual kickoff shot

They also have a separate kickoff mode, enabled by the manual kickoff flag from the undercarriage module:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:339)

When active:

- for the first `KICKOFF_KICK_TIME = 400` ms, drive forward at `KICKOFF_SPEED = 500`
- after that, set `kick = True`

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:345)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:183)

This is a simple scripted straight kickoff, not a spin shot.

## Ball acquisition and pre-shot strategies

### 1. `BALL_DIRECT`: direct chase with spin-to-ball

This is used when:

- the robot sees the ball
- does not currently have it
- and is in an edge or back sector

State selection:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:258)

Behavior:

- drive toward the ball
- choose translational speed from `BALL_CHASING_DISTS/BALL_CHASING_SPEEDS`
- choose spin from `BALL_CHASING_TIMES/BALL_CHASING_SPINS`
- if the ball angle is large enough, stop translating and mainly rotate to watch/reacquire the ball

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:442)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:150)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:157)

This is a direct chase, not a hiding strategy.

### 2. `BALL_NORTH`: approach behind the ball

This is used when:

- the robot sees the ball
- does not have it
- and is not in the edge/back zone

State selection:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:262)

Behavior:

- compute a target point behind the ball rather than going straight into it
- for front-camera ball detection, use `BEHIND_BALL_CAPTURE_DIST = 80`
- drive to that behind-ball position
- while doing so, add spin to keep heading near north / forward

References:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:471)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:496)
- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:518)
- [constants_robot1.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/constants_robot1.py:227)

This is an important strategic point:

- they prefer to get behind the ball before shooting when they are in the central attacking area

That is not "hiding the ball", but it is definitely a ball-control and shot-setup strategy.

## Do they hide the ball or back up with possession?

### What they do have

They have a few behaviors that partially resemble hiding, shielding, or retreating:

1. In `go_shoot()`, if the robot is very close to the opponent goal line (`self.tracker.position.y < 450`), it overrides the shot and drives backward/downfield with:

- `angle = 180 - self.heading`
- `v = 250`
- `spin = 0`

This is in:

- [robot_attacker.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_attacker.py:428)

This is the clearest real backward-with-ball behavior in the active attacker.

2. `EDGE_TRICK` intentionally carries the ball along a side-wall lane before rotating and shooting:

- this can function like a concealment or angle-creation maneuver
- but it is coded as an edge-based trick shot, not as explicit shielding

3. `DUEL` sidesteps around a front robot before kicking:

- that can protect the ball briefly by moving around the obstacle rather than kicking immediately

4. `BALL_NORTH` approaches behind the ball:

- this is about ball setup and cleaner possession, not about opponent shielding

### What they do not have

They do **not** appear to have an explicit, opponent-aware ball shielding controller such as:

- keep body between enemy and ball
- deliberately put ball on far side of robot from defender
- continuous possession-preserving orbit around an opponent
- dedicated "hide ball" state with geometric reasoning about enemy pose

So the accurate summary is:

- yes, they have some retreating and side-stepping behaviors
- no, they do not have a sophisticated explicit ball-hiding system

## Goalkeeper attack and shooting behavior

The active goalkeeper also has its own simpler attack/kick logic in:

- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:210)

### Goalkeeper kickoff / kick behavior

If the goalkeeper has the ball, or is close enough with the ball in front and the geometry is favorable, it enters `kickoff()`:

- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:181)

Inside `kickoff()`:

1. rotate toward the ball with a strong spin term
2. if it has secured the ball, either:
3. kick when enough time has passed or it has advanced far enough upfield
4. otherwise rotate toward the goal while driving forward

References:

- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:210)
- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:216)
- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:226)

This is simpler than the attacker and is not really a multi-strategy shooting system.

### Goalkeeper positional defense

If it is not kicking, the goalkeeper mainly chooses between:

- blocking/interposing relative to the ball using `medzi()`
- a special attacking mode using `offence()` if `self.attack` is true

References:

- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:185)
- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:242)
- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:382)

Important note:

- `self.attack` is initialized to `False`
- the line that would set `self.attack = not self.see_enemy` is commented out

So in the active code, the goalkeeper is mostly defensive.

References:

- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:74)
- [robot_miso.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/robot_miso.py:179)

## How "smart" the final kick is

The final kick itself is not a complicated actuation strategy.

They call `do_kick()`, which simply sets an internal kick request flag:

- [soccer_robot.py](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/other-teams/rcj-soccer-open-gen3/RoboCupOpen/soccer_robot/soccer_robot.py:281)

So most of the strategic intelligence is in:

- where they move before the kick
- how they align
- when they decide to request the kick

not in the kick command itself.

## Bottom line

Their active attacking code has a genuine strategy tree, and it is more than just "drive to ball and kick."

The strongest attack-related behaviors are:

- `SHOOT`: normal finishing with turn-around-ball alignment
- `EDGE_TRICK`: side-wall spin trick shot
- `DUEL`: short scripted side-step around a robot in front
- `BALL_NORTH`: behind-ball setup approach

For "ball hiding" specifically, the accurate answer is:

- they have a few hiding-like or shielding-like maneuvers
- especially the edge trick, duel sidestep, and a small backward retreat near goal
- but they do not have a fully explicit ball shielding strategy as a dedicated control policy
