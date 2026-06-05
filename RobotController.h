#pragma once

#include "LineEstimator.h"
#include "MotorDriver.h"
#include "UltrasonicRangeSensor.h"

namespace lf {

// 互斥状态；共享一个 stateTicks_ 计数器；所有写入走 transitionTo()。详见 ADR-009、
// 状态集与命名 ADR-012：
//   kSensorSettle       → 上电后等传感器多数表决窗口跑满。
//   kGoStraight         → 默认行为；双轮 kMotorCruisePwm 直行；任一传感器见黑
//                         （kOffsetLeft/kOffsetRight/kIntersection/kCentered）连续
//                         kEncounterConfirmTicks 个 tick 触发 kEncounterTurnLeft。
//   kEncounterTurnLeft  → 开环左转（L 反转 / R 正转）持续 kEncounterTurnLeftControlTicks，
//                         然后回 kSensorSettle。不可被新事件打断。
//   kObstacleStop       → 超声波连续确认近距离障碍，电机断电短暂停车。
//   kAvoidanceTurnRight → 开环右转（L 正转 / R 反转）持续 kObstacleRightTurnControlTicks，
//                         然后回 kSensorSettle。不可打断。
//   kStopped            → 终态：失活或外部要求停车后永不再消耗控制循环。
//   kFault              → boot 时 MCUSR.WDRF 触发，永久停车并关 WDT（ADR-011 §1）。
enum class RobotState : uint8_t {
  kSensorSettle = 0,
  kGoStraight,
  kEncounterTurnLeft,
  kObstacleStop,
  kAvoidanceTurnRight,
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

  // 共享计数器：同一时刻只有一个状态活跃。transitionTo() 会清零它。
  // 各状态语义：
  //   kSensorSettle       → 已 settle 的 tick 数。
  //   kGoStraight         → 连续见黑的 tick 数（去抖计数）。
  //   kEncounterTurnLeft  → 左转已持续的 tick 数。
  //   kObstacleStop       → 停车已持续的 tick 数。
  //   kAvoidanceTurnRight → 右转已持续的 tick 数。
  uint16_t stateTicks_ = 0;
  // 控制循环错过的 tick 数（监控用，不参与控制决策）。
  uint16_t missedControlTicks_ = 0;
  RobotState state_ = RobotState::kSensorSettle;

  void runControlStep();
  void runGoStraight(const LineEstimate& estimate);
  void runEncounterTurnLeft();
  void runObstacleStop();
  void runAvoidanceTurnRight();

  // 唯一允许写 state_ 的入口；集中所有"进入新状态"的副作用。
  void transitionTo(RobotState newState);
};

} // namespace lf
