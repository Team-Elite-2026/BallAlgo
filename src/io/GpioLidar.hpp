#pragma once

#if defined(BALLALGO_HAS_GPIOD)
struct gpiod_chip;
struct gpiod_line;
#endif

namespace ballalgo {

class GpioLidar {
 public:
  bool init(int pwmGpioBcm);
  void cleanup();
  ~GpioLidar() { cleanup(); }

 private:
  int lineFd_ = -1;
  int gpio_ = -1;
#if defined(BALLALGO_HAS_GPIOD)
  gpiod_chip* chip_ = nullptr;
  gpiod_line* line_ = nullptr;
#endif
};

}  // namespace ballalgo
