#include "MotorDriver.h"

namespace lf {

void MotorDriver::begin() {
  left_ = {};
  right_ = {};
  Timer1MotorPwm::submit(makeDutyFrame());
}

void MotorDriver::setTargetSpeeds(const int16_t left, const int16_t right) {
  left_.target = clampSignedPwm(left);
  right_.target = clampSignedPwm(right);
}

void MotorDriver::update() {
  stepMotor(&left_);
  stepMotor(&right_);
  Timer1MotorPwm::submit(makeDutyFrame());
}

void MotorDriver::stopNow() {
  left_ = {};
  right_ = {};
  Timer1MotorPwm::emergencyStop();
}

int16_t MotorDriver::clampSignedPwm(const int16_t value) {
  if (value > RobotConfig::kMotorMaxPwm) {
    return RobotConfig::kMotorMaxPwm;
  }
  if (value < -static_cast<int16_t>(RobotConfig::kMotorMaxPwm)) {
    return -static_cast<int16_t>(RobotConfig::kMotorMaxPwm);
  }
  return value;
}

int16_t MotorDriver::rampToward(const int16_t current, const int16_t target) {
  const int16_t step = RobotConfig::kMotorRampStepPerControlTick;
  if (current < target) {
    const int16_t next = current + step;
    return (next > target) ? target : next;
  }
  if (current > target) {
    const int16_t next = current - step;
    return (next < target) ? target : next;
  }
  return current;
}

void MotorDriver::stepMotor(MotorState* state) {
  if (state == nullptr) {
    return;
  }

  const int8_t targetSign = signOf(state->target);
  const int8_t currentSign = signOf(state->current);

  if (state->blankTicks > 0) {
    // 方向切换后的空档期：IA/IB 都低，避免 L9110S 输入瞬间冲突。
    state->current = 0;
    --state->blankTicks;
    return;
  }

  if (state->current != 0 && targetSign != 0 && currentSign != targetSign) {
    // 先按斜率降到 0，再进入空档期，最后才允许反向 PWM。
    state->current = rampToward(state->current, 0);
    if (state->current == 0) {
      state->lastDirection = targetSign;
      state->blankTicks = RobotConfig::kDirectionBlankControlTicks;
    }
    return;
  }

  if (state->current == 0 && targetSign != 0 && state->lastDirection != 0 &&
      targetSign != state->lastDirection) {
    state->lastDirection = targetSign;
    state->blankTicks = RobotConfig::kDirectionBlankControlTicks;
    return;
  }

  state->current = rampToward(state->current, state->target);
  if (state->current != 0) {
    state->lastDirection = signOf(state->current);
  }
}

uint8_t MotorDriver::magnitude(const int16_t value) {
  const int16_t positive = (value < 0) ? -value : value;
  return static_cast<uint8_t>(positive);
}

int8_t MotorDriver::signOf(const int16_t value) {
  if (value > 0) {
    return 1;
  }
  if (value < 0) {
    return -1;
  }
  return 0;
}

void MotorDriver::applySide(const int16_t signedPwm, const bool invert, const bool forwardUsesIb,
                            uint8_t* iaDuty, uint8_t* ibDuty) {
  if (iaDuty == nullptr || ibDuty == nullptr) {
    return;
  }

  *iaDuty = 0;
  *ibDuty = 0;

  int16_t command = signedPwm;
  if (invert) {
    command = -command;
  }

  if (command == 0) {
    return;
  }

  const bool useForwardInput = command > 0;
  const bool useIb = (useForwardInput == forwardUsesIb);
  // 同一侧任意时刻只驱动 IA 或 IB 一路；另一端保持低电平滑行。
  if (useIb) {
    *ibDuty = magnitude(command);
  } else {
    *iaDuty = magnitude(command);
  }
}

Timer1MotorPwm::DutyFrame MotorDriver::makeDutyFrame() const {
  Timer1MotorPwm::DutyFrame frame = {};
  applySide(left_.current, RobotConfig::kInvertLeftMotor, RobotConfig::kLeftForwardUsesIb,
            &frame.leftIa, &frame.leftIb);
  applySide(right_.current, RobotConfig::kInvertRightMotor, RobotConfig::kRightForwardUsesIb,
            &frame.rightIa, &frame.rightIb);
  return frame;
}

} // namespace lf
