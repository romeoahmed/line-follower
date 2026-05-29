#include "MotorDriver.h"

namespace lf {

void MotorDriver::begin() {
  left_ = {};
  right_ = {};
  Timer1MotorPwm::submit(makeDutyFrame());
}

void MotorDriver::setTargetSpeeds(const int16_t left, const int16_t right) {
  left_.target = applyCompensation(left, RobotConfig::kLeftMotorTrimPermille);
  right_.target = applyCompensation(right, RobotConfig::kRightMotorTrimPermille);
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

int16_t MotorDriver::applyCompensation(const int16_t value, const int16_t trimPermille) {
  if (value == 0) {
    return 0;
  }

  const int32_t scale = 1000L + trimPermille;
  int32_t scaled = static_cast<int32_t>(value) * scale;
  if (scaled >= 0) {
    scaled += 500;
  } else {
    scaled -= 500;
  }

  const int32_t divided = scaled / 1000L;
  int16_t compensated = 0;
  if (divided > RobotConfig::kMotorMaxPwm) {
    compensated = RobotConfig::kMotorMaxPwm;
  } else if (divided < -static_cast<int32_t>(RobotConfig::kMotorMaxPwm)) {
    compensated = -static_cast<int16_t>(RobotConfig::kMotorMaxPwm);
  } else {
    compensated = static_cast<int16_t>(divided);
  }
  const int16_t minimum = RobotConfig::kMotorMinimumEffectivePwm;
  if (minimum > 0 && compensated != 0 && magnitude(compensated) < minimum) {
    compensated = signOf(compensated) * minimum;
  }

  return clampSignedPwm(compensated);
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

  const uint8_t duty = magnitude(command);
  const bool useForwardInput = command > 0;
  const bool directionInputUsesIb = (useForwardInput == forwardUsesIb);

  if (RobotConfig::kMotorDriveMode == MotorDriveMode::kBrakeHighSideInversePwm) {
    const uint8_t inverseDuty = static_cast<uint8_t>(RobotConfig::kPwmFullScale - duty);
    if (directionInputUsesIb) {
      *ibDuty = RobotConfig::kPwmFullScale;
      *iaDuty = inverseDuty;
    } else {
      *iaDuty = RobotConfig::kPwmFullScale;
      *ibDuty = inverseDuty;
    }
    return;
  }

  // 低侧滑行 PWM 回退模式：同一侧任意时刻只驱动 IA 或 IB 一路。
  if (directionInputUsesIb) {
    *ibDuty = duty;
  } else {
    *iaDuty = duty;
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
