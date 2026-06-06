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
  bool plan(float sx, float sy, int stheta, float gx, float gy, int gtheta,
            std::vector<Waypoint3>& out, float* costS = nullptr);

 private:
  struct Node {
    int ix, iy, it;
    float g, f;
  };
  int index(int ix, int iy, int it) const;
  float heuristic(int ix, int iy, int it, int gx, int gy, int gt) const;

  float fieldW_, fieldH_;
  int cellMm_, cols_, rows_, hBins_;
  std::vector<float> gScore_;
  std::vector<int> parent_;
  std::vector<uint8_t> closed_;
};

}  // namespace ballalgo
