/**
 * Simulation: robot EKF sensor fusion demo + A* pathfinding test.
 *
 * Usage
 *   ./motion_planning          — human-readable EKF table (10 ms print rate)
 *   ./motion_planning --csv    — machine-readable CSV for plot_ekf.py
 *   ./motion_planning --astar  — A* path planning test scenarios
 *   ./motion_planning --json   — JSON export consumed by visualize_planner.py
 *
 * CSV columns: t,true_x,true_y,ekf_x,ekf_y,event
 *   event = "pred"     : mouse-sensor-only update step
 *           "pre_snap" : EKF state just BEFORE a LiDAR hard-reset
 *           "lidar"    : EKF state just AFTER a LiDAR hard-reset
 *
 * Coordinate system (CLAUDE.md): +x = right, +y = forward, origin = field centre.
 * Heading 0° = facing +y, 90° = facing +x, clockwise.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <array>

#include "robot_ekf.hpp"
#include "astar.hpp"
#include "hermite_spline.hpp"
#include "velocity_profiler.hpp"

// ── A* test helpers ───────────────────────────────────────────────────────────

static double pathLengthCm(const std::vector<GridPos>& path) {
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        const double dc = path[i].col - path[i-1].col;
        const double dr = path[i].row - path[i-1].row;
        total += std::sqrt(dc*dc + dr*dr) * AStar::CELL_CM;
    }
    return total;
}

static void printGrid(double robot_x, double robot_y,
                      double ball_x,  double ball_y,
                      double goal_x,  double goal_y,
                      double approach_cm,
                      const AStarResult& result) {
    double tgt_wx, tgt_wy;
    AStar::approachPoint(ball_x, ball_y, goal_x, goal_y, approach_cm, tgt_wx, tgt_wy);

    const GridPos start_g  = AStar::worldToGrid(robot_x, robot_y);
    const GridPos ball_g   = AStar::worldToGrid(ball_x,  ball_y);
    const GridPos target_g = AStar::worldToGrid(tgt_wx,  tgt_wy);

    // Build character grid  ('.' = passable, '#' = ball obstacle)
    std::array<std::array<char, AStar::COLS>, AStar::ROWS> grid;
    for (int r = 0; r < AStar::ROWS; ++r)
        for (int c = 0; c < AStar::COLS; ++c)
            grid[r][c] = AStar::isBlocked(c, r, ball_x, ball_y) ? '#' : '.';

    // Mark path cells
    if (result.found) {
        for (const auto& p : result.path) {
            if (!(p == start_g) && !(p == target_g))
                grid[p.row][p.col] = '*';
        }
    }

    // Key markers override everything
    grid[ball_g.row][ball_g.col]     = 'B';  // ball centre
    grid[start_g.row][start_g.col]   = 'S';  // robot start
    grid[target_g.row][target_g.col] = 'G';  // approach target

    // Print top-down (row 48 = y=120 = attack goal at top)
    std::cout << "  [Attack goal y=+120]\n";
    for (int r = AStar::ROWS - 1; r >= 0; --r) {
        std::cout << std::setw(5) << (r - 24) * 5 << " |";
        for (int c = 0; c < AStar::COLS; ++c)
            std::cout << grid[r][c];
        std::cout << "|\n";
    }
    const std::string hbar(AStar::COLS, '-');
    std::cout << "       +" << hbar << "+\n";
    std::cout << "  [Defence goal y=-120]\n";
    std::cout << "  Legend: S=robot  B=ball  G=approach-target  *=path  #=obstacle\n";
}

static void printHeadingTable(const AStarResult& result) {
    if (!result.found) return;
    const auto& path    = result.path;
    const auto& heading = result.heading;

    std::cout << "\n  Heading schedule (smooth rotation to strike angle):\n";
    std::cout << "  Node  World(cm)           Grid        Heading\n";
    std::cout << "  ────  ──────────────────  ──────────  ───────\n";

    for (size_t i = 0; i < path.size(); ++i) {
        double wx, wy;
        AStar::gridToWorld(path[i], wx, wy);

        std::cout << "  " << std::setw(4) << i
                  << "  (" << std::setw(6) << std::setprecision(0) << wx
                  << ", " << std::setw(6) << wy << ")"
                  << "  (" << std::setw(2) << path[i].col
                  << ", " << std::setw(2) << path[i].row << ")"
                  << "  " << std::setw(7) << std::setprecision(1) << heading[i] << "°";

        if (i == 0)
            std::cout << "  <- robot start";
        else if (i == path.size() - 1)
            std::cout << "  <- strike angle";

        std::cout << "\n";
    }
}

static void printSpline(const std::vector<SplinePoint>& spline,
                        int samples_per_segment,
                        const AStarResult& astar) {
    if (spline.empty()) return;

    const int n_astar = static_cast<int>(astar.path.size());
    const int S       = samples_per_segment;

    std::cout << "\n  Hermite spline (" << S << " samples/segment → "
              << spline.size() << " total points):\n";
    std::cout << "  Pt    x (cm)   y (cm)   Heading\n";
    std::cout << "  ────  ───────  ───────  ───────\n";

    for (int i = 0; i < static_cast<int>(spline.size()); ++i) {
        const auto& p = spline[i];
        std::cout << "  " << std::setw(4) << i
                  << "  " << std::setw(7) << std::setprecision(1) << p.x
                  << "  " << std::setw(7) << p.y
                  << "  " << std::setw(7) << p.heading << "°";

        // Mark points that coincide with the original A* grid nodes
        const int astar_idx = i / S;   // which A* segment
        if (i % S == 0 && astar_idx < n_astar) {
            double ax, ay;
            AStar::gridToWorld(astar.path[astar_idx], ax, ay);
            std::cout << "  [A* node " << astar_idx
                      << " @ (" << std::setprecision(0) << ax
                      << "," << ay << ")]";
        }
        std::cout << "\n";
    }
}

// ── JSON export  (consumed by visualize_planner.py) ──────────────────────────

static void runJsonExport() {
    constexpr int N_SCENARIOS    = 4;
    constexpr int SPLINE_SAMPLES = 10;
    constexpr double GOAL_W_CM   = 60.0;

    struct Scenario {
        const char* title;
        double rx, ry, rh;
        double bx, by;
        double gx, gy;
        double approach_cm;
        double vx_start, vy_start;
        double vx_end,   vy_end;
    };

    // ═══════════════════════════════════════════════════════════════════════
    // VELOCITY INPUTS  —  edit these to test the S'(0) = v_start constraint
    //
    //   vx_start / vy_start : robot's velocity at the moment of planning
    //                         (cm/s, global frame, +x=right +y=forward)
    //   vx_end   / vy_end   : desired velocity when reaching the approach pose
    //
    //   Rebuild + re-run visualize_planner.py to see the effect.
    // ═══════════════════════════════════════════════════════════════════════
    struct VelIn { double vx_start, vy_start, vx_end, vy_end; };
    constexpr VelIn vel[N_SCENARIOS] = {
        /* Sc1: Centre, ball ahead-right  */ {  0,    0,   0,   0 },
        /* Sc2: Behind ball, facing right */ {  0,    -10,   0,   0 },
        /* Sc3: Defence goal, facing back */ {  -10,    0,   0,   0 },
        /* Sc4: Right side, ball on left  */ {  -10,    0,   0,   0 },
    };
    // ═══════════════════════════════════════════════════════════════════════

    constexpr Scenario scenarios[N_SCENARIOS] = {
        //  title                             rx   ry   rh   bx   by   gx   gy  appr
        {"Sc1: Centre, ball ahead-right",      50,  50,   320,   0,   0,  0, 120,  10,
         vel[0].vx_start, vel[0].vy_start, vel[0].vx_end, vel[0].vy_end},
        {"Sc2: Behind ball, facing right",   -50, -70,  90,  10,  20,  0, 120,  10,
         vel[1].vx_start, vel[1].vy_start, vel[1].vx_end, vel[1].vy_end},
        {"Sc3: Defence goal, facing back",    20, -90, 180,   5,   0,  0, 120,  10,
         vel[2].vx_start, vel[2].vy_start, vel[2].vx_end, vel[2].vy_end},
        {"Sc4: Right side, ball on left",     60,  10, 270, -45,  50,  0, 120,  10,
         vel[3].vx_start, vel[3].vy_start, vel[3].vx_end, vel[3].vy_end},
    };

    // Helper: emit a float without scientific notation
    std::cout << std::fixed << std::setprecision(4);

    std::cout << "{\n";

    // ── Top-level constants (Python reads these to stay in sync with C++) ──
    std::cout << "  \"field_w_cm\":    " << 182.0                  << ",\n"
              << "  \"field_h_cm\":    " << 243.0                  << ",\n"
              << "  \"cell_cm\":       " << AStar::CELL_CM         << ",\n"
              << "  \"cols\":          " << AStar::COLS             << ",\n"
              << "  \"rows\":          " << AStar::ROWS             << ",\n"
              << "  \"ball_clear_cm\": " << AStar::BALL_CLEAR_CM   << ",\n"
              << "  \"goal_y_cm\":     " << 120.0                  << ",\n"
              << "  \"goal_w_cm\":     " << GOAL_W_CM              << ",\n"
              << "  \"spline_samples\": " << SPLINE_SAMPLES        << ",\n";

    std::cout << "  \"scenarios\": [\n";

    for (int s = 0; s < N_SCENARIOS; ++s) {
        const auto& sc = scenarios[s];

        // Approach point + required heading
        double ap_x, ap_y;
        AStar::approachPoint(sc.bx, sc.by, sc.gx, sc.gy, sc.approach_cm, ap_x, ap_y);
        const double tgt_h = AStar::approachHeading(ap_x, ap_y, sc.bx, sc.by);

        // A* search
        AStarResult result = AStar::plan(sc.rx, sc.ry, sc.rh,
                                          sc.bx, sc.by,
                                          sc.gx, sc.gy, sc.approach_cm);

        // Path length (world-coord arc length)
        double path_len = 0.0;
        for (size_t i = 1; i < result.path.size(); ++i) {
            double wx0, wy0, wx1, wy1;
            AStar::gridToWorld(result.path[i-1], wx0, wy0);
            AStar::gridToWorld(result.path[i],   wx1, wy1);
            path_len += std::sqrt((wx1-wx0)*(wx1-wx0) + (wy1-wy0)*(wy1-wy0));
        }

        // Net rotation (shortest-angle delta, same as computeHeadings)
        double rot = 0.0;
        if (result.found && result.heading.size() >= 2) {
            rot = result.heading.back() - result.heading.front();
            while (rot >  180.0) rot -= 360.0;
            while (rot < -180.0) rot += 360.0;
        }

        // Hermite spline + velocity profile
        const auto sdata = HermiteSpline::buildData(result, SPLINE_SAMPLES,
                                                     sc.vx_start, sc.vy_start,
                                                     sc.vx_end,   sc.vy_end);
        const auto& spline = sdata.samples;

        ProfilerConfig pcfg;
        pcfg.v_start_mps = static_cast<float>(
            std::hypot(sc.vx_start, sc.vy_start) * 0.01);  // cm/s → m/s
        pcfg.v_end_mps  = 0.0f;
        pcfg.num_steps  = static_cast<int>(sdata.samples.size());

        const auto profile = VelocityProfiler::compute(sdata, pcfg);

        std::cout << "    {\n";
        std::cout << "      \"title\":    \"" << sc.title << "\",\n";

        std::cout << "      \"robot\":    {\"x\": " << sc.rx << ", \"y\": " << sc.ry
                  << ", \"heading\": " << sc.rh << "},\n";
        std::cout << "      \"ball\":     {\"x\": " << sc.bx << ", \"y\": " << sc.by << "},\n";
        std::cout << "      \"goal\":     {\"x\": " << sc.gx << ", \"y\": " << sc.gy << "},\n";
        std::cout << "      \"approach\":  {\"x\": " << ap_x  << ", \"y\": " << ap_y
                  << ", \"heading\": " << tgt_h << "},\n";
        std::cout << "      \"robot_vel\": {\"vx\": " << sc.vx_start << ", \"vy\": " << sc.vy_start << "},\n";
        std::cout << "      \"end_vel\":   {\"vx\": " << sc.vx_end   << ", \"vy\": " << sc.vy_end   << "},\n";

        // ── A* block ────────────────────────────────────────────────────────
        std::cout << "      \"astar\": {\n";
        std::cout << "        \"found\":        " << (result.found ? "true" : "false") << ",\n";
        std::cout << "        \"node_count\":   " << result.path.size() << ",\n";
        std::cout << "        \"path_len_cm\":  " << path_len << ",\n";
        std::cout << "        \"rotation_deg\": " << rot << ",\n";
        if (result.path.empty()) {
            std::cout << "        \"path_cm\": [],\n";
        } else {
            std::cout << "        \"path_cm\": [\n";
            for (size_t i = 0; i < result.path.size(); ++i) {
                double wx, wy;
                AStar::gridToWorld(result.path[i], wx, wy);
                std::cout << "          [" << wx << ", " << wy << "]";
                if (i + 1 < result.path.size()) std::cout << ",";
                std::cout << "\n";
            }
            std::cout << "        ],\n";
        }
        // Per-node heading schedule — Python uses this to recompute the
        // heading interpolation when velocities change in the interactive UI.
        std::cout << "        \"headings\": [";
        for (size_t i = 0; i < result.heading.size(); ++i) {
            std::cout << result.heading[i];
            if (i + 1 < result.heading.size()) std::cout << ", ";
        }
        std::cout << "]\n";
        std::cout << "      },\n";

        // ── Spline block ─────────────────────────────────────────────────────
        if (spline.empty()) {
            std::cout << "      \"spline\": [],\n";
        } else {
            std::cout << "      \"spline\": [\n";
            for (size_t i = 0; i < spline.size(); ++i) {
                std::cout << "        {\"x\": " << spline[i].x
                          << ", \"y\": "        << spline[i].y
                          << ", \"heading\": "  << spline[i].heading << "}";
                if (i + 1 < spline.size()) std::cout << ",";
                std::cout << "\n";
            }
            std::cout << "      ],\n";
        }

        // ── Velocity profile block ────────────────────────────────────────────
        // One speed (m/s) per spline sample — used by visualiser to colour the
        // spline by instantaneous path speed.
        if (profile.empty()) {
            std::cout << "      \"profile\": []\n";
        } else {
            std::cout << "      \"profile\": [";
            for (size_t i = 0; i < profile.size(); ++i) {
                const float spd = std::sqrt(profile[i].vx * profile[i].vx
                                          + profile[i].vy * profile[i].vy);
                std::cout << spd;
                if (i + 1 < profile.size()) std::cout << ", ";
            }
            std::cout << "]\n";
        }

        std::cout << "    }";
        if (s < N_SCENARIOS - 1) std::cout << ",";
        std::cout << "\n";
    }

    std::cout << "  ]\n}\n";
}

static void runAStarTest() {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "=== A* Pathfinding Test ===\n";
    std::cout << "Field: 182×243 cm   Grid: " << AStar::COLS << "×" << AStar::ROWS
              << " nodes at " << AStar::CELL_CM << " cm/cell\n";
    std::cout << "Ball obstacle radius: " << AStar::BALL_CLEAR_CM << " cm   "
              << "Attack goal: (0, 120) cm\n\n";

    struct Scenario {
        const char* label;
        double robot_x, robot_y, robot_heading_deg;
        double ball_x,  ball_y;
        double goal_x,  goal_y;
        double approach_cm;
    };
    // Scenario List for ASCII ASTAR terminal view
    const Scenario scenarios[] = {
        // Label                      robot              hdg    ball         goal        appr
        {"Robot at centre facing forward, ball ahead-right",
                                      50.0,   0.0,   0.0,  40.0,  60.0,  0.0, 120.0, 10.0},
        {"Robot behind ball, facing right",
                                    -50.0, -70.0,  90.0,  10.0,  20.0,  0.0, 120.0, 10.0},
        {"Robot near defence goal facing backward",
                                     20.0, -90.0, 180.0,   5.0,   0.0,  0.0, 120.0, 10.0},
        {"Ball on left, robot on right facing left",
                                     60.0,  10.0, 270.0, -45.0,  50.0,  0.0, 120.0, 10.0},
    };

    for (int s = 0; s < 4; ++s) {
        const auto& sc = scenarios[s];
        std::cout << "──────────────────────────────────────────\n";
        std::cout << "Scenario " << (s+1) << ": " << sc.label << "\n";

        double ap_x, ap_y;
        AStar::approachPoint(sc.ball_x, sc.ball_y,
                             sc.goal_x, sc.goal_y,
                             sc.approach_cm, ap_x, ap_y);
        const double strike_h = AStar::approachHeading(ap_x, ap_y, sc.ball_x, sc.ball_y);

        const GridPos rg = AStar::worldToGrid(sc.robot_x, sc.robot_y);
        const GridPos bg = AStar::worldToGrid(sc.ball_x,  sc.ball_y);
        const GridPos ag = AStar::worldToGrid(ap_x,       ap_y);

        std::cout << "  Robot:          (" << std::setw(6) << sc.robot_x << ", "
                  << std::setw(6) << sc.robot_y << ") cm  grid ("
                  << rg.col << "," << rg.row << ")  heading " << sc.robot_heading_deg << "°\n";
        std::cout << "  Ball:           (" << std::setw(6) << sc.ball_x  << ", "
                  << std::setw(6) << sc.ball_y  << ") cm  grid ("
                  << bg.col << "," << bg.row << ")\n";
        std::cout << "  Approach point: (" << std::setw(6) << ap_x << ", "
                  << std::setw(6) << ap_y << ") cm  grid ("
                  << ag.col << "," << ag.row << ")  required heading " << strike_h << "°\n";

        const AStarResult result = AStar::plan(sc.robot_x, sc.robot_y,
                                               sc.robot_heading_deg,
                                               sc.ball_x,  sc.ball_y,
                                               sc.goal_x,  sc.goal_y,
                                               sc.approach_cm);

        if (result.found) {
            const double len = pathLengthCm(result.path);
            // Shortest-angle delta (mirrors computeHeadings logic)
            double rot = result.heading.back() - result.heading.front();
            while (rot >  180.0) rot -= 360.0;
            while (rot < -180.0) rot += 360.0;
            std::cout << "  Path: FOUND   " << result.path.size() << " nodes   "
                      << std::setprecision(1) << len << " cm   "
                      << "total rotation: " << std::abs(rot) << "° "
                      << (rot >= 0.0 ? "(CW)" : "(CCW)") << "\n";
        } else {
            std::cout << "  Path: NOT FOUND (ball may be cornered)\n";
        }

        std::cout << "\n";
        printGrid(sc.robot_x, sc.robot_y,
                  sc.ball_x,  sc.ball_y,
                  sc.goal_x,  sc.goal_y,
                  sc.approach_cm, result);

        printHeadingTable(result);

        // Build and print the Hermite spline
        constexpr int SAMPLES = 5;
        const auto spline = HermiteSpline::build(result, SAMPLES);
        printSpline(spline, SAMPLES, result);

        std::cout << "\n";
    }
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--astar") {
        runAStarTest();
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "--json") {
        runJsonExport();
        return 0;
    }

    const bool csv_mode = (argc > 1 && std::string(argv[1]) == "--csv");

    RobotEKF ekf(0.5, 1e-4, 0.0);  // r_lidar=0: LiDAR is absolute authority — no residual uncertainty after snap

    constexpr double dt_mouse   = 1e-3;
    constexpr double dt_lidar   = 1.0 / 15.0;
    constexpr double sim_time   = 1.0;
    constexpr double true_speed = 1.0;
    constexpr double omega_dps  = 45.0;
    constexpr int    print_every_human = 10;  // ms between rows in human mode
    constexpr int    print_every_csv   = 5;   // ms between rows in csv mode

    const int n_steps      = static_cast<int>(sim_time / dt_mouse);
    const int lidar_period = static_cast<int>(dt_lidar / dt_mouse);

    auto mouse_noise_vx = [](int i) -> double {
        const double bias    = 0.05;
        const double hf[4]   = { 0.01, -0.008, 0.006, -0.012 };
        return bias + hf[i % 4] * 0.1;
    };
    auto mouse_noise_vy = [](int i) -> double {
        const double hf[4] = { 0.005, -0.007, 0.003, -0.009 };
        return hf[i % 4] * 0.1;
    };
    ekf.init(0.0, 0.0);

    double true_x = 0.0, true_y = 0.0, true_h_deg = 0.0;
    int lidar_frame = 0;

    if (csv_mode) {
        std::cout << "t,true_x,true_y,ekf_x,ekf_y,event\n";
    } else {
        std::cout << std::fixed << std::setprecision(4);
        const std::string sep(90, '-');
        std::cout
            << "  t(s)  |  true_x   true_y  |   ekf_x    ekf_y  |  err_x   err_y  | event\n"
            << sep << "\n";
    }

    std::cout << std::fixed << std::setprecision(6);

    for (int i = 0; i < n_steps; ++i) {
        const double t         = i * dt_mouse;
        const double theta_rad = true_h_deg * M_PI / 180.0;

        true_x     += std::sin(theta_rad) * true_speed * dt_mouse;
        true_y     += std::cos(theta_rad) * true_speed * dt_mouse;
        true_h_deg += omega_dps * dt_mouse;
        if (true_h_deg >= 360.0) true_h_deg -= 360.0;

        ekf.updateMouse(0.0 + mouse_noise_vx(i),
                        true_speed + mouse_noise_vy(i),
                        true_h_deg, dt_mouse);

        bool   did_lidar = false;
        double pre_x = 0, pre_y = 0;
        double lidar_x = 0, lidar_y = 0;

        if (i > 0 && i % lidar_period == 0) {
            pre_x   = ekf.getX();
            pre_y   = ekf.getY();
            lidar_x = true_x;   // LiDAR is absolute authority — no noise
            lidar_y = true_y;
            ekf.updateLidar(lidar_x, lidar_y);
            did_lidar = true;
            ++lidar_frame;
        }

        if (csv_mode) {
            const int period = print_every_csv;
            if (did_lidar) {
                // pre-snap row: shows where EKF drifted to
                std::cout
                    << t << "," << true_x << "," << true_y << ","
                    << pre_x << "," << pre_y << ",pre_snap\n";
                // post-snap row: shows corrected position
                std::cout
                    << t << "," << true_x << "," << true_y << ","
                    << ekf.getX() << "," << ekf.getY() << ",lidar\n";
            } else if (i % period == 0) {
                std::cout
                    << t << "," << true_x << "," << true_y << ","
                    << ekf.getX() << "," << ekf.getY() << ",pred\n";
            }
        } else {
            const int period = print_every_human;
            const double ex = ekf.getX() - true_x;
            const double ey = ekf.getY() - true_y;
            const std::string sep(90, '-');

            if (did_lidar) {
                std::cout << sep << "\n";
                std::cout << std::setw(7) << t << "  | "
                    << std::setw(7) << true_x << "  " << std::setw(7) << true_y << "  | "
                    << std::setw(7) << pre_x  << "  " << std::setw(7) << pre_y  << "  | "
                    << std::setw(7) << (pre_x - true_x) << "  "
                    << std::setw(7) << (pre_y - true_y) << "  | pre-snap\n";
                std::cout << std::setw(7) << t << "  | "
                    << std::setw(7) << true_x << "  " << std::setw(7) << true_y << "  | "
                    << std::setw(7) << ekf.getX() << "  " << std::setw(7) << ekf.getY() << "  | "
                    << std::setw(7) << ex << "  " << std::setw(7) << ey << "  | "
                    << "LIDAR SNAP  lidar=(" << std::setw(7) << lidar_x << ", "
                    << std::setw(7) << lidar_y << ")  snap_d=("
                    << std::setw(7) << (ekf.getX() - pre_x) << ", "
                    << std::setw(7) << (ekf.getY() - pre_y) << ")\n";
                std::cout << sep << "\n";
            } else if (i % period == 0) {
                std::cout << std::setw(7) << t << "  | "
                    << std::setw(7) << true_x << "  " << std::setw(7) << true_y << "  | "
                    << std::setw(7) << ekf.getX() << "  " << std::setw(7) << ekf.getY() << "  | "
                    << std::setw(7) << ex << "  " << std::setw(7) << ey << "  | pred\n";
            }
        }
    }

    if (!csv_mode)
        std::cout << "\nDone. LiDAR frames processed: " << lidar_frame << "\n";

    return 0;
}
