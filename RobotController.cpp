#include "RobotController.h"

namespace lf {
namespace {

struct FollowProfile {
  uint8_t basePwm;
  int16_t maxCorrection;
  RobotConfig::PidGainsQ8 gains;
  bool usePid;
  bool allowIntegral;
};

int16_t abs16(const int16_t value) {
  return (value < 0) ? static_cast<int16_t>(-value) : value;
}

FollowProfile profileForEstimate(const LineEstimate& estimate, const int16_t lastError) {
  if (estimate.intersectionLike) {
    return FollowProfile{RobotConfig::kMotorCautiousPwm, 0, RobotConfig::kPidStraightGainsQ8, false,
                         false};
  }

  if (estimate.ambiguous) {
    const uint8_t base =
        (lastError == 0) ? RobotConfig::kMotorStraightPwm : RobotConfig::kMotorCautiousPwm;
    return FollowProfile{base, 0, RobotConfig::kPidStraightGainsQ8, false, false};
  }

  if (abs16(estimate.error) >= RobotConfig::kLineCurveErrorThreshold) {
    return FollowProfile{RobotConfig::kMotorCurvePwm, RobotConfig::kPidCurveMaxCorrection,
                         RobotConfig::kPidCurveGainsQ8, true, true};
  }

  return FollowProfile{RobotConfig::kMotorStraightPwm, RobotConfig::kPidStraightMaxCorrection,
                       RobotConfig::kPidStraightGainsQ8, true, true};
}

bool ambiguousCenterTimedOut(const uint8_t ticks, const int16_t lastError) {
  return RobotConfig::kAmbiguousCenterLimitTicks > 0 && lastError != 0 &&
         ticks > RobotConfig::kAmbiguousCenterLimitTicks;
}

} // namespace

void RobotController::begin() {
  state_ = RobotState::kBoot;
  Timer1MotorPwm::begin();
  motors_.begin();
  sensors_.begin();
  ultrasonic_.begin();
  pid_.reset();

  settleTicks_ = 0;
  ambiguousTicks_ = 0;
  lostTicks_ = 0;
  obstacleStopTicks_ = 0;
  obstacleTurnTicks_ = 0;
  lastError_ = 0;
  missedControlTicks_ = 0;
  state_ = RobotState::kSensorSettle;
}

void RobotController::poll() {
  ultrasonic_.poll();

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
  ultrasonic_.poll();

  if (state_ == RobotState::kObstacleStop) {
    handleObstacleStop();
    return;
  }

  if (state_ == RobotState::kObstacleTurnRight) {
    handleObstacleTurnRight();
    return;
  }

  if (RobotConfig::kObstacleAvoidanceEnabled && ultrasonic_.obstaclePresent()) {
    enterObstacleStop();
    return;
  }

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
  case RobotState::kObstacleStop:
    handleObstacleStop();
    break;
  case RobotState::kObstacleTurnRight:
    handleObstacleTurnRight();
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
    if (ambiguousCenterTimedOut(ambiguousTicks_, lastError_)) {
      enterLineLost();
      return;
    }
  } else {
    ambiguousTicks_ = 0;
  }

  if (!estimate.ambiguous && !estimate.intersectionLike) {
    lastError_ = estimate.error;
  }

  const FollowProfile profile = profileForEstimate(estimate, lastError_);
  const int16_t correction =
      profile.usePid
          ? pid_.update(estimate.error, profile.gains, profile.maxCorrection, profile.allowIntegral)
          : 0;
  const int16_t base = profile.basePwm;
  const int16_t left = clampMotorCommand(static_cast<int16_t>(base + correction));
  const int16_t right = clampMotorCommand(static_cast<int16_t>(base - correction));

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

void RobotController::handleObstacleStop() {
  motors_.stopNow();

  if (obstacleStopTicks_ < RobotConfig::kObstacleStopHoldControlTicks) {
    ++obstacleStopTicks_;
    return;
  }

  enterObstacleTurnRight();
}

void RobotController::handleObstacleTurnRight() {
  if (obstacleTurnTicks_ >= RobotConfig::kObstacleRightTurnControlTicks) {
    motors_.stopNow();
    ultrasonic_.restartAfterManeuver();
    enterSensorSettle();
    return;
  }

  motors_.setTargetSpeeds(RobotConfig::kObstacleRightTurnPwm,
                          -static_cast<int16_t>(RobotConfig::kObstacleRightTurnPwm));
  motors_.update();
  ++obstacleTurnTicks_;
}

void RobotController::enterLineLost() {
  state_ = RobotState::kLineLost;
  lostTicks_ = 0;
  ambiguousTicks_ = 0;
  pid_.reset();
}

void RobotController::enterObstacleStop() {
  state_ = RobotState::kObstacleStop;
  ambiguousTicks_ = 0;
  lostTicks_ = 0;
  obstacleStopTicks_ = 0;
  obstacleTurnTicks_ = 0;
  pid_.reset();
  motors_.stopNow();
}

void RobotController::enterObstacleTurnRight() {
  state_ = RobotState::kObstacleTurnRight;
  obstacleTurnTicks_ = 0;
  pid_.reset();
  ultrasonic_.restartAfterManeuver();
}

void RobotController::enterSensorSettle() {
  settleTicks_ = 0;
  ambiguousTicks_ = 0;
  lostTicks_ = 0;
  obstacleStopTicks_ = 0;
  obstacleTurnTicks_ = 0;
  lastError_ = 0;
  pid_.reset();
  state_ = RobotState::kSensorSettle;
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
