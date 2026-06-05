#include "RobotController.h"

#include "BootStatus.h"

#include <avr/wdt.h>

namespace lf {
namespace {

// 行为入口判定：**两个传感器都**见到黑才算"遇到黑线"。
//   - kIntersection 是 kBetweenSensors 模式下双黑（宽线 / 交叉路口 / 满足整条
//     黑线落入两个传感器之间）。
//   - kCentered 是 kOnLine 模式下双黑（车正压在黑线上）；默认模式下不会出现，
//     列在这里是 forward-looking。
//   - kOffsetLeft / kOffsetRight（单边偏黑）不再触发——它们多发生在车从白底
//     斜插入黑线边缘、或传感器扫过细线边缘的瞬间，对"识别一条完整黑线"是
//     误报。新行为要求"两个传感器都进入黑线区域"才反应，明显更严苛。
//   - kAmbiguous / kInvalid 视为"没见到黑"，继续直行。
bool estimateSawBlack(const LineEstimate& estimate) {
  switch (estimate.state) {
  case LineState::kIntersection:
  case LineState::kCentered:
    return true;
  case LineState::kOffsetLeft:
  case LineState::kOffsetRight:
  case LineState::kAmbiguous:
  case LineState::kInvalid:
    return false;
  }
  return false;
}

} // namespace

void RobotController::begin() {
  Timer1MotorPwm::begin();
  motors_.begin();
  sensors_.begin();
  ultrasonic_.begin();

  missedControlTicks_ = 0;
  stateTicks_ = 0;

  // WDRF：上一帧固件失活过，永久停 kFault 而不是无声重启循环冲出去（ADR-011 §1）。
  if (BootStatus::lastResetWasWatchdog()) {
    wdt_disable();
    state_ = RobotState::kFault;
    motors_.stopNow();
    return;
  }

  state_ = RobotState::kSensorSettle;
  // 12 × 控制 tick：吸收一次性抖动，持续饥饿 120 ms 内强复位。
  wdt_enable(WDTO_120MS);
}

void RobotController::poll() {
  if (state_ == RobotState::kFault) {
    return;
  }

  // 超声波 TRIG/ECHO 非阻塞状态机在主循环高频轮询，独立于控制 tick。
  ultrasonic_.poll();

  const uint8_t ticks = Timer1MotorPwm::takeControlTicks();
  if (ticks == 0) {
    return;
  }

  // tick-gated 喂狗：抓"loop 活但 Timer1 COMPA ISR 死"——顶部喂会掩盖（ADR-011 §1）。
  wdt_reset();

  if (ticks > 1) {
    // 诊断计数器，0xFFFF 饱和（wrap 会让长时间运行后数值回小，掩盖实时性问题）。
    const uint16_t add = static_cast<uint16_t>(ticks - 1);
    const uint16_t headroom = static_cast<uint16_t>(0xFFFFu - missedControlTicks_);
    missedControlTicks_ += (add <= headroom) ? add : headroom;
  }
  // 不补跑历史 tick：那只会把陈旧决策灌进电机命令，放大延迟。
  runControlStep();
}

void RobotController::runControlStep() {
  // 终态/故障态：永久停车，不再消耗控制循环。
  if (state_ == RobotState::kStopped || state_ == RobotState::kFault) {
    return;
  }

  // 进行中的转向机动（左转或右转）一旦启动就不能被新事件打断——避免动作中途反悔。
  if (state_ == RobotState::kObstacleStop) {
    runObstacleStop();
    return;
  }
  if (state_ == RobotState::kAvoidanceTurnRight) {
    runAvoidanceTurnRight();
    return;
  }
  if (state_ == RobotState::kEncounterTurnLeft) {
    runEncounterTurnLeft();
    return;
  }

  // 任何"非机动"状态下，障碍 latch 最高优先级（高于遇黑左转）。
  if (RobotConfig::kObstacleAvoidanceEnabled && ultrasonic_.obstaclePresent()) {
    transitionTo(RobotState::kObstacleStop);
    return;
  }

  // Settle 阶段：跑采样让 majority filter 预热满窗口，但忽略其值，等计数器到点。
  if (state_ == RobotState::kSensorSettle) {
    sensors_.sample();
    motors_.setTargetSpeeds(0, 0);
    motors_.update();
    if (stateTicks_ < RobotConfig::kSensorSettleControlTicks) {
      ++stateTicks_;
      return;
    }
    transitionTo(RobotState::kGoStraight);
    return;
  }

  const LineSensorSample sample = sensors_.sample();
  const LineEstimate estimate = LineEstimator::estimate(sample);

  switch (state_) {
  case RobotState::kGoStraight:
    runGoStraight(estimate);
    return;
  case RobotState::kSensorSettle:
  case RobotState::kEncounterTurnLeft:
  case RobotState::kObstacleStop:
  case RobotState::kAvoidanceTurnRight:
  case RobotState::kStopped:
  case RobotState::kFault:
    // 不可达路径兜底：转 kFault 永久停车。
    transitionTo(RobotState::kFault);
    return;
  }
  // 无 default：新增枚举值时编译器会给出未处理警告。
}

void RobotController::runGoStraight(const LineEstimate& estimate) {
  // 见黑去抖：连续 kEncounterConfirmTicks 个 tick 见黑才触发左转；任一次见白即清零。
  // 镜像 UltrasonicRangeSensor 的 obstacleSamples_ 设计。
  if (estimateSawBlack(estimate)) {
    if (stateTicks_ < 0xFFFF) {
      ++stateTicks_;
    }
    if (stateTicks_ >= RobotConfig::kEncounterConfirmTicks) {
      transitionTo(RobotState::kEncounterTurnLeft);
      return;
    }
  } else {
    stateTicks_ = 0;
  }

  // 默认行为：双轮匀速直行。signed 命令对左右等价 → MotorDriver 的 trim、ramp、
  // 死区跳变照常生效，无需控制层介入。
  const int16_t cruise = static_cast<int16_t>(RobotConfig::kMotorCruisePwm);
  motors_.setTargetSpeeds(cruise, cruise);
  motors_.update();
}

void RobotController::runEncounterTurnLeft() {
  if (stateTicks_ >= RobotConfig::kEncounterTurnLeftControlTicks) {
    motors_.stopNow();
    transitionTo(RobotState::kSensorSettle);
    return;
  }
  // 左转：左轮反转、右轮正转。开环、按 tick 数计时；无几何保证（ADR-005 / ADR-012）。
  const int16_t magnitude = static_cast<int16_t>(RobotConfig::kEncounterTurnLeftPwm);
  motors_.setTargetSpeeds(static_cast<int16_t>(-magnitude), magnitude);
  motors_.update();
  ++stateTicks_;
}

void RobotController::runObstacleStop() {
  motors_.stopNow();
  if (stateTicks_ < RobotConfig::kObstacleStopHoldControlTicks) {
    ++stateTicks_;
    return;
  }
  transitionTo(RobotState::kAvoidanceTurnRight);
}

void RobotController::runAvoidanceTurnRight() {
  if (stateTicks_ >= RobotConfig::kObstacleRightTurnControlTicks) {
    motors_.stopNow();
    ultrasonic_.restartAfterManeuver();
    transitionTo(RobotState::kSensorSettle);
    return;
  }
  // 右转：左轮正转、右轮反转，与左转对称。
  const int16_t magnitude = static_cast<int16_t>(RobotConfig::kObstacleRightTurnPwm);
  motors_.setTargetSpeeds(magnitude, static_cast<int16_t>(-magnitude));
  motors_.update();
  ++stateTicks_;
}

void RobotController::transitionTo(const RobotState newState) {
  // 所有状态转换都过这里——这是唯一允许写 state_ 的入口，避免漏掉副作用。
  state_ = newState;
  stateTicks_ = 0;
  switch (newState) {
  case RobotState::kSensorSettle:
  case RobotState::kObstacleStop:
  case RobotState::kStopped:
    motors_.stopNow();
    return;
  case RobotState::kFault:
    // 关 WDT 防止无声重启循环；MCU 仍跑 loop()，便于硬件调试。
    wdt_disable();
    motors_.stopNow();
    return;
  case RobotState::kAvoidanceTurnRight:
    // 避障右转是被 obstacle latch 触发的——清掉旧 latch，转向完毕重启测距周期。
    ultrasonic_.restartAfterManeuver();
    return;
  case RobotState::kEncounterTurnLeft:
    // 不调 restartAfterManeuver()：左转入口不是 obstacle latch，没有需要清掉的状态；
    // 让超声波测距在左转期间持续运行，万一转向中途真撞上障碍也能尽快重新 latch。
    return;
  case RobotState::kGoStraight:
    return;
  }
}

} // namespace lf
