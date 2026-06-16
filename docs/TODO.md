# BallAlgo TODO

## Next Fixes

### Ball Velocity Frame Semantics

- **Issue:** `BallState.vx/vy` does not have one canonical meaning across the ball KF, team-ball fusion, offense, and defense.
- **Why it matters:** moving-robot interception and defense pacing can use the robot's own velocity as if it were ball velocity.
- **Recommended fix:**
  - Move the canonical tracked ball velocity to the field frame.
  - Define one explicit contract for `BallState.vx/vy` and document it in the headers.
  - Update `makeFusedBodyBall()` and `fieldBallFromBody()` so they are exact inverses of that contract.
  - Add a regression test: stationary field ball + moving robot should still produce approximately zero field-frame ball velocity.
