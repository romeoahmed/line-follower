#include "PidController.h"

namespace lf {

void PidController::reset() {
  integral_ = 0;
  previousError_ = 0;
  hasPrevious_ = false;
}

int16_t PidController::update(const int16_t error, const bool allowIntegral) {
  const int16_t derivative = hasPrevious_ ? static_cast<int16_t>(error - previousError_) : 0;
  const int32_t nextIntegral =
      clamp32(static_cast<int32_t>(integral_) + error, -RobotConfig::kPidIntegralLimit,
              RobotConfig::kPidIntegralLimit);

  const int32_t usedIntegral = allowIntegral ? nextIntegral : integral_;
  const int32_t raw = static_cast<int32_t>(RobotConfig::kPidKpQ8) * error +
                      static_cast<int32_t>(RobotConfig::kPidKiQ8) * usedIntegral +
                      static_cast<int32_t>(RobotConfig::kPidKdQ8) * derivative;

  const int16_t correction =
      clamp16(raw / 256, -RobotConfig::kPidMaxCorrection, RobotConfig::kPidMaxCorrection);

  const bool saturated =
      correction == RobotConfig::kPidMaxCorrection || correction == -RobotConfig::kPidMaxCorrection;
  if (allowIntegral && !saturated) {
    // 输出饱和时冻结积分，避免恢复循迹后长时间反向补偿。
    integral_ = nextIntegral;
  }

  previousError_ = error;
  hasPrevious_ = true;
  return correction;
}

int32_t PidController::clamp32(const int32_t value, const int32_t low, const int32_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

int16_t PidController::clamp16(const int32_t value, const int16_t low, const int16_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return static_cast<int16_t>(value);
}

} // namespace lf
