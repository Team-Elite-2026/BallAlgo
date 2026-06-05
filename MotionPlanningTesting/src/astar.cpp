#include "astar.hpp"

#include <queue>
#include <tuple>
#include <limits>
#include <algorithm>

// 8-directional neighbour offsets: {Δcol, Δrow}
static const int DIRS[8][2] = {
    { 1,  0}, {-1,  0}, { 0,  1}, { 0, -1},   // cardinal
    { 1,  1}, { 1, -1}, {-1,  1}, {-1, -1}    // diagonal
};

// World movement angle (degrees, 0°=+y, 90°=+x, CW) for each DIRS entry.
// Derived from atan2(dc, dr) for each {dc, dr} pair.
static const double DIR_WORLD_ANGLE[8] = {
     90.0,  // { 1,  0}  right
    270.0,  // {-1,  0}  left
      0.0,  // { 0,  1}  forward
    180.0,  // { 0, -1}  backward
     45.0,  // { 1,  1}  forward-right
    135.0,  // { 1, -1}  backward-right
    315.0,  // {-1,  1}  forward-left
    225.0,  // {-1, -1}  backward-left
};

// ── private helpers ───────────────────────────────────────────────────────────

int AStar::idx3(int col, int row, int hi) {
    return (col * ROWS + row) * NUM_HEADINGS + hi;
}

int AStar::angularDist(int hi_a, int hi_b) {
    const int d = std::abs(hi_a - hi_b);
    return std::min(d, NUM_HEADINGS - d);
}

bool AStar::inBounds(int col, int row) {
    return col >= 0 && col < COLS && row >= 0 && row < ROWS;
}

// Admissible heuristic: straight-line travel time plus minimum rotation cost.
// K_CURVE is omitted — future path-direction changes can't be bounded from here.
double AStar::heuristic(int col, int row, int hi, int tcol, int trow, int thi) {
    const double dx      = (col  - tcol) * CELL_CM;
    const double dy      = (row  - trow) * CELL_CM;
    const double spatial = std::sqrt(dx*dx + dy*dy);
    const double rot_deg = angularDist(hi, thi) * HEADING_STEP;
    return spatial / V_MAX_CM_S + K_ROT * rot_deg;
}

// ── coordinate conversions ─────────────────────────────────────────────────────

GridPos AStar::worldToGrid(double x_cm, double y_cm) {
    const int col = std::clamp(static_cast<int>(std::round(x_cm / CELL_CM)) + 18, 0, COLS - 1);
    const int row = std::clamp(static_cast<int>(std::round(y_cm / CELL_CM)) + 24, 0, ROWS - 1);
    return {col, row};
}

void AStar::gridToWorld(GridPos g, double& x_cm, double& y_cm) {
    x_cm = (g.col - 18) * CELL_CM;
    y_cm = (g.row - 24) * CELL_CM;
}

// ── heading helpers ────────────────────────────────────────────────────────────

int AStar::degToHi(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return static_cast<int>(std::round(deg / HEADING_STEP)) % NUM_HEADINGS;
}

double AStar::hiToDeg(int hi) {
    return hi * HEADING_STEP;
}

// ── geometry ───────────────────────────────────────────────────────────────────

bool AStar::isBlocked(int col, int row, double ball_x, double ball_y) {
    const double wx = (col - 18) * CELL_CM;
    const double wy = (row - 24) * CELL_CM;
    const double dx = wx - ball_x;
    const double dy = wy - ball_y;
    return (dx * dx + dy * dy) < (BALL_CLEAR_CM * BALL_CLEAR_CM);
}

void AStar::approachPoint(double ball_x, double ball_y,
                          double goal_x, double goal_y,
                          double approach_cm,
                          double& out_x, double& out_y) {
    double dx   = goal_x - ball_x;
    double dy   = goal_y - ball_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-9) dist = 1e-9;
    out_x = ball_x - (dx / dist) * approach_cm;
    out_y = ball_y - (dy / dist) * approach_cm;
}

double AStar::approachHeading(double approach_x, double approach_y,
                              double ball_x,     double ball_y) {
    // atan2(dx, dy) gives clockwise-from-+y heading (0°=forward, 90°=right)
    const double dx = ball_x - approach_x;
    const double dy = ball_y - approach_y;
    double deg = std::atan2(dx, dy) * (180.0 / M_PI);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

// ── 3-D A* planner ────────────────────────────────────────────────────────────

AStarResult AStar::plan(double robot_x,      // NOLINT: defaults in header
                        double robot_y,
                        double robot_heading_deg,
                        double ball_x,
                        double ball_y,
                        double goal_x,
                        double goal_y,
                        double approach_cm) {
    // Approach point and required final heading
    double tgt_wx, tgt_wy;
    approachPoint(ball_x, ball_y, goal_x, goal_y, approach_cm, tgt_wx, tgt_wy);

    const GridPos start  = worldToGrid(robot_x, robot_y);
    const GridPos target = worldToGrid(tgt_wx,  tgt_wy);

    const int start_hi  = degToHi(robot_heading_deg);
    const int target_hi = degToHi(approachHeading(tgt_wx, tgt_wy, ball_x, ball_y));

    // Trivial: already at target cell with correct heading
    if (start == target) {
        return {true, {start}, {hiToDeg(target_hi)}};
    }

    const int N3 = COLS * ROWS * NUM_HEADINGS;
    std::vector<double> g_cost(N3, std::numeric_limits<double>::infinity());

    struct State3D { int col, row, hi; };
    std::vector<State3D> came_from(N3, {-1, -1, -1});

    // min-heap: (f=g+h, g, col, row, hi)
    using Entry = std::tuple<double, double, int, int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    const int start_id = idx3(start.col, start.row, start_hi);
    g_cost[start_id] = 0.0;
    open.emplace(heuristic(start.col, start.row, start_hi,
                           target.col, target.row, target_hi),
                 0.0, start.col, start.row, start_hi);

    while (!open.empty()) {
        auto [f, g_val, col, row, hi] = open.top();
        open.pop();

        if (g_val > g_cost[idx3(col, row, hi)] + 1e-9) continue;  // stale entry

        if (col == target.col && row == target.row && hi == target_hi) {
            // Reconstruct path and heading schedule from the came_from chain
            std::vector<GridPos> path;
            std::vector<double>  headings;
            int cc = col, cr = row, chi = hi;
            while (!(cc == start.col && cr == start.row && chi == start_hi)) {
                path.push_back({cc, cr});
                headings.push_back(hiToDeg(chi));
                auto [pc, pr, phi] = came_from[idx3(cc, cr, chi)];
                cc = pc; cr = pr; chi = phi;
            }
            path.push_back({start.col, start.row});
            headings.push_back(hiToDeg(start_hi));
            std::reverse(path.begin(),     path.end());
            std::reverse(headings.begin(), headings.end());
            return {true, std::move(path), std::move(headings)};
        }

        // φ_in: direction we travelled to arrive at (col, row), in degrees.
        // Used for K_CURVE; absent at the start node (parent_col == -1).
        const auto& [par_col, par_row, par_hi] = came_from[idx3(col, row, hi)];
        const bool  has_parent = (par_col >= 0);
        const double phi_in    = has_parent
            ? std::atan2(col - par_col, row - par_row) * (180.0 / M_PI)
            : 0.0;

        for (int d = 0; d < 8; ++d) {
            const int nc = col + DIRS[d][0];
            const int nr = row + DIRS[d][1];
            if (!inBounds(nc, nr)) continue;
            if (isBlocked(nc, nr, ball_x, ball_y)) continue;

            const bool   diagonal  = (DIRS[d][0] != 0 && DIRS[d][1] != 0);
            const double base_dist = diagonal ? CELL_CM * M_SQRT2 : CELL_CM;

            // Translation cost: time to cover this step at peak speed
            const double move_cost = base_dist / V_MAX_CM_S;

            // Curve cost: penalise kinks in the travel direction (zero at start node)
            double curve_cost = 0.0;
            if (has_parent) {
                double dphi = DIR_WORLD_ANGLE[d] - phi_in;
                while (dphi >  180.0) dphi -= 360.0;
                while (dphi < -180.0) dphi += 360.0;
                curve_cost = K_CURVE * std::abs(dphi);
            }

            // Try every achievable heading at the neighbour cell
            for (int nhi = 0; nhi < NUM_HEADINGS; ++nhi) {
                const double dtheta_deg = angularDist(hi, nhi) * HEADING_STEP;
                const double rot_cost   = K_ROT * dtheta_deg;
                const double ng         = g_val + move_cost + rot_cost + curve_cost;
                const int    nid        = idx3(nc, nr, nhi);
                if (ng < g_cost[nid]) {
                    g_cost[nid]    = ng;
                    came_from[nid] = {col, row, hi};
                    open.emplace(ng + heuristic(nc, nr, nhi, target.col, target.row, target_hi),
                                 ng, nc, nr, nhi);
                }
            }
        }
    }

    return {false, {}, {}};
}
