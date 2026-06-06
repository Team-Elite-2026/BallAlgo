#pragma once

#include <cstdint>
#include <vector>

namespace ballalgo {

struct Waypoint3 {
  float xMm, yMm, thetaDeg;
};

class AStar3D {
 public:
  AStar3D(float fieldW, float fieldH, int cellMm, int headingBins);

  // Inflate a circular obstacle (the ball, or an enemy robot) of the given
  // radius (mm) centred at field (xMm, yMm). Cells whose centre falls within the
  // radius become strictly impassable. Pass radius <= 0 to disable.
  void setObstacle(float xMm, float yMm, float clearMm);
  void clearObstacle();

  bool plan(float sx, float sy, int stheta, float gx, float gy, int gtheta,
            std::vector<Waypoint3>& out, float* costS = nullptr);

 private:
  struct Node {
    int ix, iy, it;
    float g, f;
  };
  int index(int ix, int iy, int it) const;
  float heuristic(int ix, int iy, int it, int gx, int gy, int gt) const;
  bool blocked(int ix, int iy) const;

  float fieldW_, fieldH_;
  int cellMm_, cols_, rows_, hBins_;
  float maxStraightVMps_;
  bool hasObstacle_ = false;
  float obsXMm_ = 0, obsYMm_ = 0, obsClearMm_ = 0;
  std::vector<float> gScore_;
  std::vector<int> parent_;
  std::vector<uint8_t> closed_;
};

}  // namespace ballalgo
