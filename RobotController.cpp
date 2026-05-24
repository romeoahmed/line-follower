#include "RobotController.h"

namespace lf {

void RobotController::begin() {
  state_ = RobotState::kBoot;
  Timer1MotorPwm::begin();
  motors_.begin();
  sensors_.begin();
  pid_.reset();

  settleTicks_ = 0;
  ambiguousTicks_ = 0;
  lostTicks_ = 0;
  lastError_ = 0;
  missedControlTicks_ = 0;
  state_ = RobotState::kSensorSettle;
}

void RobotController::poll() {
  const uint8_t ticks = Timer1MotorPwm::takeControlTicks();
  if (ticks == 0) {
    return;
  }

  if (ticks > 1) {
    missedControlTicks_ += static_cast<uint16_t>(ticks - 1);
  }

  // 控制闭环只跑最新一帧，不补跑历史 tick，避免延迟继续堆积。
  runControlStep();
}

void RobotController::runControlStep() {
  if (state_ == RobotState::kSensorSettle) {
    handleSensorSettle();
    return;
  }

  const LineSensorSample sample = sensors_.sample();
  const LineEstimate estimate = LineEstimator::estimate(sample);

  switch (state_) {
  case RobotState::kFollowLine:
    handleFollowLine(estimate);
    break;
  case RobotState::kLineLost:
    handleLineLost(estimate);
    break;
  case RobotState::kStopped:
    motors_.setTargetSpeeds(0, 0);
    motors_.update();
    break;
  case RobotState::kBoot:
  case RobotState::kSensorSettle:
  default:
    enterStopped();
    break;
  }
}

void RobotController::handleSensorSettle() {
  sensors_.sample();
  motors_.setTargetSpeeds(0, 0);
  motors_.update();

  if (settleTicks_ < RobotConfig::kSensorSettleControlTicks) {
    ++settleTicks_;
    return;
  }

  state_ = RobotState::kFollowLine;
  pid_.reset();
}

void RobotController::handleFollowLine(const LineEstimate& estimate) {
  if (!estimate.valid || estimate.lost) {
    enterLineLost();
    return;
  }

  if (estimate.ambiguous) {
    if (ambiguousTicks_ < 255) {
      ++ambiguousTicks_;
    }
    if (ambiguousTicks_ > RobotConfig::kAmbiguousCenterLimitTicks) {
      enterLineLost();
      return;
    }
  } else {
    ambiguousTicks_ = 0;
    lastError_ = estimate.error;
  }

  // 双数字传感器信息量有限：疑似居中/交叉时保留速度，但冻结积分。
  const int16_t correction =
      pid_.update(estimate.error, !estimate.ambiguous && !estimate.intersectionLike);
  const int16_t left =
      clampMotorCommand(static_cast<int16_t>(RobotConfig::kMotorBasePwm) + correction);
  const int16_t right =
      clampMotorCommand(static_cast<int16_t>(RobotConfig::kMotorBasePwm) - correction);

  motors_.setTargetSpeeds(left, right);
  motors_.update();
}

void RobotController::handleLineLost(const LineEstimate& estimate) {
  if (estimate.valid && !estimate.lost && !estimate.ambiguous) {
    state_ = RobotState::kFollowLine;
    lostTicks_ = 0;
    ambiguousTicks_ = 0;
    lastError_ = estimate.error;
    pid_.reset();
    handleFollowLine(estimate);
    return;
  }

  if (lostTicks_ < 255) {
    ++lostTicks_;
  }

  if (lostTicks_ > RobotConfig::kLineLostStopTicks) {
    enterStopped();
    return;
  }

  const int16_t search = RobotConfig::kMotorSearchPwm;
  // 失线后按最后偏差原地低速搜索；超过超时直接停车。
  if (lastError_ < 0) {
    motors_.setTargetSpeeds(-search, search);
  } else if (lastError_ > 0) {
    motors_.setTargetSpeeds(search, -search);
  } else {
    motors_.setTargetSpeeds(0, 0);
  }
  motors_.update();
}

void RobotController::enterLineLost() {
  state_ = RobotState::kLineLost;
  lostTicks_ = 0;
  ambiguousTicks_ = 0;
  pid_.reset();
}

void RobotController::enterStopped() {
  state_ = RobotState::kStopped;
  pid_.reset();
  motors_.stopNow();
}

int16_t RobotController::clampMotorCommand(const int16_t value) {
  if (value > RobotConfig::kMotorMaxPwm) {
    return RobotConfig::kMotorMaxPwm;
  }
  if (value < -static_cast<int16_t>(RobotConfig::kMotorMaxPwm)) {
    return -static_cast<int16_t>(RobotConfig::kMotorMaxPwm);
  }
  return value;
}

} // namespace lf
