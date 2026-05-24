#pragma once

#include "LineEstimator.h"
#include "MotorDriver.h"
#include "PidController.h"

namespace lf {

enum class RobotState : uint8_t {
  kBoot = 0,
  kSensorSettle,
  kFollowLine,
  kLineLost,
  kStopped,
};

class RobotController {
 public:
  void begin();
  void poll();

  RobotState state() const {
    return state_;
  }
  uint16_t missedControlTicks() const {
    return missedControlTicks_;
  }

 private:
  LineSensors sensors_;
  MotorDriver motors_;
  PidController pid_;
  RobotState state_ = RobotState::kBoot;
  int16_t lastError_ = 0;
  uint8_t settleTicks_ = 0;
  uint8_t ambiguousTicks_ = 0;
  uint8_t lostTicks_ = 0;
  uint16_t missedControlTicks_ = 0;

  void runControlStep();
  void handleSensorSettle();
  void handleFollowLine(const LineEstimate& estimate);
  void handleLineLost(const LineEstimate& estimate);
  void enterLineLost();
  void enterStopped();
  static int16_t clampMotorCommand(int16_t value);
};

} // namespace lf
