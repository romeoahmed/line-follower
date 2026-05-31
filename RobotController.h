#pragma once

#include "LineEstimator.h"
#include "MotorDriver.h"
#include "PdController.h"
#include "UltrasonicRangeSensor.h"

namespace lf {

// 互斥状态；共享一个 stateTicks_ 计数器；所有写入走 transitionTo()。详见 ADR-009。
// kFault：boot 时 MCUSR.WDRF 触发，永久停车并关 WDT（ADR-011 §1）。
enum class RobotState : uint8_t {
  kSensorSettle = 0,
  kFollowLine,
  kLineLost,
  kObstacleStop,
  kObstacleTurnRight,
  kStopped,
  kFault,
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
  UltrasonicRangeSensor ultrasonic_;
  MotorDriver motors_;
  PdController pd_;

  // 共享计数器：同一时刻只有一个状态活跃。transitionTo() 会清零它。
  uint16_t stateTicks_ = 0;
  // 跨状态的短期记忆，用于失线方向搜索；只有 kSensorSettle 转入时才清零。
  int16_t lastError_ = 0;
  // 控制循环错过的 tick 数（监控用，不参与控制决策）。
  uint16_t missedControlTicks_ = 0;
  RobotState state_ = RobotState::kSensorSettle;

  void runControlStep();
  void runFollowLine(const LineEstimate& estimate);
  void runLineLost(const LineEstimate& estimate);
  void runObstacleStop();
  void runObstacleTurnRight();

  // 唯一允许写 state_ 的入口；集中所有"进入新状态"的副作用。
  void transitionTo(RobotState newState);
};

} // namespace lf
