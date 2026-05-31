#include "RobotController.h"

namespace lf {
namespace {

struct FollowProfile {
  uint8_t basePwm;
  int16_t maxCorrection;
  PdGainsQ8 gains;
  bool usePd;
};

// 选 profile：基于 LineState 穷举；error 只在确定要给 PD 时用到。
// 交叉/双白都不更新 PD（偏差信息不可信，让 PD 介入只会引入噪声），按低风险直行。
FollowProfile profileForEstimate(const LineEstimate& estimate, const int16_t lastError) {
  switch (estimate.state) {
  case LineState::kIntersection:
    return FollowProfile{RobotConfig::kMotorCautiousPwm, 0, RobotConfig::kPdStraightGainsQ8, false};
  case LineState::kAmbiguous: {
    const uint8_t base =
        (lastError == 0) ? RobotConfig::kMotorStraightPwm : RobotConfig::kMotorCautiousPwm;
    return FollowProfile{base, 0, RobotConfig::kPdStraightGainsQ8, false};
  }
  case LineState::kOffsetLeft:
  case LineState::kOffsetRight: {
    const int16_t absError =
        (estimate.error < 0) ? static_cast<int16_t>(-estimate.error) : estimate.error;
    if (absError >= RobotConfig::kLineCurveErrorThreshold) {
      return FollowProfile{RobotConfig::kMotorCurvePwm, RobotConfig::kPdCurveMaxCorrection,
                           RobotConfig::kPdCurveGainsQ8, true};
    }
    return FollowProfile{RobotConfig::kMotorStraightPwm, RobotConfig::kPdStraightMaxCorrection,
                         RobotConfig::kPdStraightGainsQ8, true};
  }
  case LineState::kCentered:
    // kOnLine 模式下双黑居中：error 已经是 0，按直线 profile 跑 PD 等价于"无修正直行"。
    return FollowProfile{RobotConfig::kMotorStraightPwm, RobotConfig::kPdStraightMaxCorrection,
                         RobotConfig::kPdStraightGainsQ8, true};
  case LineState::kInvalid:
    // runFollowLine 在调用 profileForEstimate 前已经把 kInvalid 路到 kLineLost，
    // 不会进到这里；返回 cautious 直行作为最低风险 fallback。
    return FollowProfile{RobotConfig::kMotorCautiousPwm, 0, RobotConfig::kPdStraightGainsQ8, false};
  }
  return FollowProfile{RobotConfig::kMotorCautiousPwm, 0, RobotConfig::kPdStraightGainsQ8, false};
}

// 双白超时只在过去见过偏差时才计时；长直线居中场景 lastError 一直是 0，永远不会触发。
bool ambiguousCenterTimedOut(const uint16_t ticks, const int16_t lastError) {
  return RobotConfig::kAmbiguousCenterLimitTicks > 0 && lastError != 0 &&
         ticks > RobotConfig::kAmbiguousCenterLimitTicks;
}

} // namespace

void RobotController::begin() {
  Timer1MotorPwm::begin();
  motors_.begin();
  sensors_.begin();
  ultrasonic_.begin();
  pd_.reset();

  lastError_ = 0;
  missedControlTicks_ = 0;
  state_ = RobotState::kSensorSettle;
  stateTicks_ = 0;
}

void RobotController::poll() {
  // 超声波 TRIG/ECHO 非阻塞状态机在主循环高频轮询，独立于控制 tick。
  ultrasonic_.poll();

  const uint8_t ticks = Timer1MotorPwm::takeControlTicks();
  if (ticks == 0) {
    return;
  }
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
  // 终态：永久停车，不再消耗控制循环。
  if (state_ == RobotState::kStopped) {
    return;
  }

  // 避障机动一旦启动就不能被新事件打断；放在最前面。
  if (state_ == RobotState::kObstacleStop) {
    runObstacleStop();
    return;
  }
  if (state_ == RobotState::kObstacleTurnRight) {
    runObstacleTurnRight();
    return;
  }

  // 任何"循迹相关"状态下都让障碍 latch 优先级最高。
  if (RobotConfig::kObstacleAvoidanceEnabled && ultrasonic_.obstaclePresent()) {
    transitionTo(RobotState::kObstacleStop);
    return;
  }

  // Settle 不需要读传感器值，只等待计数器到点。单独走，避免无意义的 sensors_.sample()。
  if (state_ == RobotState::kSensorSettle) {
    sensors_.sample();
    motors_.setTargetSpeeds(0, 0);
    motors_.update();
    if (stateTicks_ < RobotConfig::kSensorSettleControlTicks) {
      ++stateTicks_;
      return;
    }
    transitionTo(RobotState::kFollowLine);
    return;
  }

  const LineSensorSample sample = sensors_.sample();
  const LineEstimate estimate = LineEstimator::estimate(sample);

  switch (state_) {
  case RobotState::kFollowLine:
    runFollowLine(estimate);
    return;
  case RobotState::kLineLost:
    runLineLost(estimate);
    return;
  case RobotState::kSensorSettle:
  case RobotState::kObstacleStop:
  case RobotState::kObstacleTurnRight:
  case RobotState::kStopped:
    // 上面已经穷举处理；落到这里说明状态机路径出错——按最低风险停车便于硬件排查。
    transitionTo(RobotState::kStopped);
    return;
  }
  // 无 default：新增枚举值时编译器会给出未处理警告。
}

void RobotController::runFollowLine(const LineEstimate& estimate) {
  if (estimate.isLost()) {
    transitionTo(RobotState::kLineLost);
    return;
  }

  // 在 kFollowLine 内 stateTicks_ 的语义是"连续 ambiguous 的 tick 数"：每次拿到明确
  // 偏差就清零；连续双白超过 kAmbiguousCenterLimitTicks 则进入失线。这是单一状态内的
  // 子计时器，不需要单独字段。
  if (estimate.isAmbiguous()) {
    if (stateTicks_ < 0xFFFF) {
      ++stateTicks_;
    }
    if (ambiguousCenterTimedOut(stateTicks_, lastError_)) {
      transitionTo(RobotState::kLineLost);
      return;
    }
  } else {
    stateTicks_ = 0;
  }

  // 交叉/双白下不更新 lastError——这两种状态下 error 字段不携带方向信息。
  if (!estimate.isAmbiguous() && !estimate.isIntersection()) {
    lastError_ = estimate.error;
  }

  const FollowProfile profile = profileForEstimate(estimate, lastError_);
  const int16_t correction =
      profile.usePd ? pd_.update(estimate.error, profile.gains, profile.maxCorrection) : 0;
  const int16_t base = profile.basePwm;
  // trim 之前在控制层 clamp 是物理正确的：保证饱和时 trim 在物理域内做差分补偿，
  // 不被 MotorDriver 内层 clamp 抹平。详见 ADR-010 §7。
  const int16_t left = clampMotorCommand(static_cast<int16_t>(base + correction));
  const int16_t right = clampMotorCommand(static_cast<int16_t>(base - correction));

  motors_.setTargetSpeeds(left, right);
  motors_.update();
}

void RobotController::runLineLost(const LineEstimate& estimate) {
  // 回到任何带方向信息（含 kIntersection）的状态就视为找回线了，先进 kFollowLine 再
  // 立即让 follow 处理这一帧；只有 kAmbiguous 与 kInvalid 维持搜索/停车。
  if (!estimate.isLost() && !estimate.isAmbiguous()) {
    lastError_ = estimate.error;
    transitionTo(RobotState::kFollowLine);
    runFollowLine(estimate);
    return;
  }

  if (stateTicks_ < 0xFFFF) {
    ++stateTicks_;
  }
  if (stateTicks_ > RobotConfig::kLineLostStopTicks) {
    transitionTo(RobotState::kStopped);
    return;
  }

  // 按最后偏差方向原地低速搜线：偏左→左轮反转；偏右→右轮反转；无偏差则不动。
  const int16_t search = RobotConfig::kMotorSearchPwm;
  if (lastError_ < 0) {
    motors_.setTargetSpeeds(-search, search);
  } else if (lastError_ > 0) {
    motors_.setTargetSpeeds(search, -search);
  } else {
    motors_.setTargetSpeeds(0, 0);
  }
  motors_.update();
}

void RobotController::runObstacleStop() {
  motors_.stopNow();
  if (stateTicks_ < RobotConfig::kObstacleStopHoldControlTicks) {
    ++stateTicks_;
    return;
  }
  transitionTo(RobotState::kObstacleTurnRight);
}

void RobotController::runObstacleTurnRight() {
  if (stateTicks_ >= RobotConfig::kObstacleRightTurnControlTicks) {
    motors_.stopNow();
    ultrasonic_.restartAfterManeuver();
    transitionTo(RobotState::kSensorSettle);
    return;
  }
  motors_.setTargetSpeeds(RobotConfig::kObstacleRightTurnPwm,
                          -static_cast<int16_t>(RobotConfig::kObstacleRightTurnPwm));
  motors_.update();
  ++stateTicks_;
}

void RobotController::transitionTo(const RobotState newState) {
  // 所有状态转换都过这里——这是唯一允许写 state_ 的入口，避免漏掉副作用。
  state_ = newState;
  stateTicks_ = 0;
  pd_.reset();
  switch (newState) {
  case RobotState::kSensorSettle:
    // 从避障机动后回到 Settle 时清掉短期记忆，让搜索从零开始。
    lastError_ = 0;
    motors_.stopNow();
    return;
  case RobotState::kObstacleStop:
  case RobotState::kStopped:
    motors_.stopNow();
    return;
  case RobotState::kObstacleTurnRight:
    ultrasonic_.restartAfterManeuver();
    return;
  case RobotState::kFollowLine:
  case RobotState::kLineLost:
    // 这两个状态在 transition 时不需要副作用：lastError_ 是跨态短期记忆；电机由
    // 后续 run*() 立即写新命令；PD 已经被开头的 reset() 清掉。
    return;
  }
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
