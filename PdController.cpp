#include "PdController.h"

namespace lf {
namespace {

int16_t clamp16(const int32_t value, const int16_t low, const int16_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return static_cast<int16_t>(value);
}

} // namespace

void PdController::reset() {
  previousError_ = 0;
  hasPrevious_ = false;
}

int16_t PdController::update(const int16_t error, const PdGainsQ8& gains,
                             const int16_t maxCorrection) {
  // caller 在 RobotConfig 用 static_assert 保证 maxCorrection 非负，无需运行时校验。
  const int16_t derivative = hasPrevious_ ? static_cast<int16_t>(error - previousError_) : 0;

  // Q8 定点：raw 单位是 (PWM * 256)，最后右移 8 位换算回 PWM 量级。
  const int32_t raw =
      static_cast<int32_t>(gains.kp) * error + static_cast<int32_t>(gains.kd) * derivative;
  const int16_t correction = clamp16(raw / 256, -maxCorrection, maxCorrection);

  previousError_ = error;
  hasPrevious_ = true;
  return correction;
}

} // namespace lf
