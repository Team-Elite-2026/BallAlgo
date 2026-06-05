/**
 * 3-D A* pathfinder for a 182×243 cm RoboCup field on a 5 cm grid.
 *
 * State space: (col, row, heading_index)
 *   37 columns (col 0‥36): x = (col-18)×5 cm  →  -90 to +90 cm
 *   49 rows    (row 0‥48): y = (row-24)×5 cm  →  -120 to +120 cm
 *   8  headings (hi 0‥7):  hi×45°             →  0° to 315°
 *
 * Coordinate convention (CLAUDE.md): +x = right, +y = forward, origin = field centre.
 * Heading 0° = facing +y, 90° = facing +x, clockwise.
 *
 * Step cost from node A(col,row,hi) to neighbour B(nc,nr,nhi):
 *   g(B) = g(A)
 *        + Distance(A→B) / V_MAX_CM_S          (translation time)
 *        + K_ROT   × |Δθ_deg|                  (heading-change penalty)
 *        + K_CURVE × |Δφ_deg|                  (path-direction-change penalty)
 *
 * Usage
 *   auto res = AStar::plan(robot_x, robot_y, robot_heading_deg, ball_x, ball_y);
 *   // res.path[i]    = grid cell at step i
 *   // res.heading[i] = robot facing angle (°) at step i — snapped to nearest 45° slice
 *                      heading.back() = approach heading (snapped), strike angle
 */

#pragma once
#include <vector>
#include <cmath>

struct GridPos {
    int col, row;
    bool operator==(const GridPos& o) const { return col == o.col && row == o.row; }
};

struct AStarResult {
    bool                 found;
    std::vector<GridPos> path;     // [0]=start, back()=approach cell behind ball
    std::vector<double>  heading;  // degrees per node — parallel to path[]
                                   // snapped to nearest 45° slice
                                   // heading.back() = approachHeading (snapped)
};

class AStar {
public:
    static constexpr int    COLS          = 37;   // field x: -90 to +90 cm
    static constexpr int    ROWS          = 49;   // field y: -120 to +120 cm
    static constexpr double CELL_CM       = 5.0;  // cm per grid step
    static constexpr double BALL_CLEAR_CM = 8.0;  // obstacle exclusion radius around ball (cm)

    // ── heading discretisation ────────────────────────────────────────────────

    static constexpr int    NUM_HEADINGS = 8;                            // slices per revolution
    static constexpr double HEADING_STEP = 360.0 / NUM_HEADINGS;        // 45.0°

    // Peak robot speed used as the denominator in the translation cost term:
    //   translation_cost = Distance / V_MAX_CM_S
    // Units: cm/s.  Set to the fastest the robot can travel in any direction.
    static constexpr double V_MAX_CM_S = 150.0;

    // Cost penalty per degree of robot heading change between consecutive nodes
    // (Δθ = |heading_B_deg − heading_A_deg|, shortest arc).
    // Captures the speed loss due to rotation-translation coupling on the drivetrain.
    static constexpr double K_ROT = 0.001;

    // Cost penalty per degree of travel-direction change between consecutive moves
    // (Δφ = |φ_out − φ_in|, shortest arc).
    // Discourages sharp kinks in the path.
    static constexpr double K_CURVE = 0.005;

    // ── coordinate conversions ────────────────────────────────────────────────

    /** World position (cm) → nearest grid cell (clamped to valid range). */
    static GridPos worldToGrid(double x_cm, double y_cm);

    /** Grid cell → world position (cm) at cell centre. */
    static void gridToWorld(GridPos g, double& x_cm, double& y_cm);

    // ── heading helpers ───────────────────────────────────────────────────────

    /** Degrees → nearest heading index [0, NUM_HEADINGS). */
    static int    degToHi(double deg);

    /** Heading index → centre angle in degrees [0, 360). */
    static double hiToDeg(int hi);

    // ── geometry ──────────────────────────────────────────────────────────────

    /**
     * Compute the behind-ball approach point in world coords (cm).
     * The point lies on the goal-to-ball line, approach_cm behind the ball.
     */
    static void approachPoint(double ball_x,   double ball_y,
                              double goal_x,   double goal_y,
                              double approach_cm,
                              double& out_x,   double& out_y);

    /**
     * Heading the robot must face at the approach point to aim at the goal
     * through the ball.  Returns degrees, 0°=+y, 90°=+x, clockwise.
     */
    static double approachHeading(double approach_x, double approach_y,
                                  double ball_x,     double ball_y);

    // ── obstacle query ────────────────────────────────────────────────────────

    /** True if grid cell (col, row) falls inside the ball obstacle zone. */
    static bool isBlocked(int col, int row, double ball_x, double ball_y);

    // ── planner ───────────────────────────────────────────────────────────────

    /**
     * Plan a path from the robot to the approach point behind the ball.
     *
     * Uses 3-D A* over (col, row, heading_index) states.  Each move step pays:
     *   1. Distance / V_MAX_CM_S              — translation time
     *   2. K_ROT   × |Δheading_deg|           — rotation coupling penalty
     *   3. K_CURVE × |Δtravel_direction_deg|  — path kink penalty
     *
     * The heading schedule in the result is derived from the planned states —
     * every entry is snapped to the nearest 45° slice.
     *
     * @param robot_heading_deg  Current robot facing angle (°). Snapped to nearest slice.
     * @param goal_x/y           Centre of goal to attack (default 0 cm, 120 cm).
     * @param approach_cm        Distance behind ball to stand (> BALL_CLEAR_CM = 8 cm).
     *
     * @return AStarResult with found=false if no path exists (ball cornered).
     */
    static AStarResult plan(double robot_x,
                            double robot_y,
                            double robot_heading_deg,
                            double ball_x,
                            double ball_y,
                            double goal_x      = 0.0,
                            double goal_y      = 120.0,
                            double approach_cm = 10.0);

private:
    static bool   inBounds   (int col, int row);
    static int    angularDist(int hi_a, int hi_b);  // shortest arc distance in slices [0, 4]
    static double heuristic  (int col, int row, int hi, int tcol, int trow, int thi);
    static int    idx3       (int col, int row, int hi);
};
