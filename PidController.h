#pragma once

#include "RobotConfig.h"

namespace lf {

class PidController {
 public:
  void reset();
  int16_t update(int16_t error, const RobotConfig::PidGainsQ8& gains, int16_t maxCorrection,
                 bool allowIntegral);

 private:
  int32_t integral_ = 0;
  int16_t previousError_ = 0;
  bool hasPrevious_ = false;

  static int32_t clamp32(int32_t value, int32_t low, int32_t high);
  static int16_t clamp16(int32_t value, int16_t low, int16_t high);
};

} // namespace lf
