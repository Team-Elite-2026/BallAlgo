#include "io/GpioLidar.hpp"

#include <iostream>

#if defined(BALLALGO_HAS_GPIOD)
#include <gpiod.h>
#endif

namespace ballalgo {

bool GpioLidar::init(int pwmGpioBcm) {
  gpio_ = pwmGpioBcm;
#if defined(BALLALGO_HAS_GPIOD)
  cleanup();
  chip_ = gpiod_chip_open_by_name("gpiochip0");
  if (!chip_) return false;
  line_ = gpiod_chip_get_line(chip_, pwmGpioBcm);
  if (!line_ || gpiod_line_request_output(line_, "ballalgo_lidar_pwm", 0) < 0) {
    cleanup();
    return false;
  }
  lineFd_ = 1;
  std::cout << "[GPIO] LD19 PWM pin " << pwmGpioBcm << " LOW\n";
  return true;
#else
  (void)pwmGpioBcm;
  std::cout << "[GPIO] stub (no libgpiod)\n";
  return true;
#endif
}

void GpioLidar::cleanup() {
#if defined(BALLALGO_HAS_GPIOD)
  if (line_) {
    gpiod_line_release(line_);
    line_ = nullptr;
  }
  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
#endif
  lineFd_ = -1;
}

}  // namespace ballalgo
