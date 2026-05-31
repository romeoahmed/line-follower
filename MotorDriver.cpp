#include "MotorDriver.h"

#include "MathUtils.h"

namespace lf {
namespace {

constexpr int16_t kMotorMaxPwmSigned = static_cast<int16_t>(RobotConfig::kMotorMaxPwm);

// trim 千分比：scale = 1 + trim/1000；±500 后整除做 round-half-away-from-zero，
// 避免 ±1 极小命令被 truncate 成 0。这里的 clamp 挡 trim 放大后越限（如 +50%
// 下 200→300）；不与控制层 mixSaturate 冗余——守不同域（ADR-011 §3）。
int16_t applyCompensation(const int16_t value, const int16_t trimPermille) {
  if (value == 0) {
    return 0;
  }
  const int32_t scaled = static_cast<int32_t>(value) * (1000L + trimPermille);
  const int32_t rounded = (scaled >= 0) ? scaled + 500 : scaled - 500;
  const int32_t divided = rounded / 1000L;
  return clampSigned<int16_t>(static_cast<int16_t>(divided), kMotorMaxPwmSigned);
}

uint8_t magnitude(const int16_t value) {
  const int16_t positive = (value < 0) ? static_cast<int16_t>(-value) : value;
  return static_cast<uint8_t>(positive);
}

int8_t signOf(const int16_t value) {
  if (value > 0) {
    return 1;
  }
  if (value < 0) {
    return -1;
  }
  return 0;
}

int16_t rampToward(const int16_t current, const int16_t target) {
  const int16_t step = RobotConfig::kMotorRampStepPerControlTick;
  const int16_t minimum = RobotConfig::kMotorMinimumEffectivePwm;

  if (current == target) {
    return current;
  }

  if (current < target) {
    // 静止→正向跨 0：直接跳到 minimum，避免 ramp 头几步停在电机死区。
    if (current <= 0 && target > 0 && minimum > 0) {
      const int16_t kicked = (target < minimum) ? target : minimum;
      if (kicked > current) {
        return kicked;
      }
    }
    const int16_t next = static_cast<int16_t>(current + step);
    return (next > target) ? target : next;
  }

  // 静止→反向跨 0：对称跳到 -minimum。
  if (current >= 0 && target < 0 && minimum > 0) {
    const int16_t kicked =
        (target > static_cast<int16_t>(-minimum)) ? target : static_cast<int16_t>(-minimum);
    if (kicked < current) {
      return kicked;
    }
  }
  const int16_t next = static_cast<int16_t>(current - step);
  return (next < target) ? target : next;
}

// 一侧电机的 L9110 输入映射：有符号命令 → (iaDuty, ibDuty)。
// 命令为 0：双输入低（默认滑行停转，ADR-006）。
// kBrakeHighSideInversePwm：方向输入整周期高，另一输入输出 255-duty 反相 PWM。
// kCoastLowSidePwm：方向输入按 duty PWM，另一输入低。
void applySide(const int16_t signedPwm, const bool invert, const bool forwardUsesIb,
               uint8_t* iaDuty, uint8_t* ibDuty) {
  *iaDuty = 0;
  *ibDuty = 0;

  int16_t command = signedPwm;
  if (invert) {
    command = static_cast<int16_t>(-command);
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

} // namespace

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

void MotorDriver::stepMotor(MotorState* state) {
  const int8_t targetSign = signOf(state->target);
  const int8_t currentSign = signOf(state->current);

  // 方向切换序列分三步：① 当前与目标反号 → ramp 到 0；② 到 0 后开 blank 空档；
  // ③ blank 倒计时结束才允许反向 PWM。这是为了避免 L9110S 两输入瞬时同高。
  if (state->blankTicks > 0) {
    state->current = 0;
    --state->blankTicks;
    return;
  }

  if (state->current != 0 && targetSign != 0 && currentSign != targetSign) {
    state->current = rampToward(state->current, 0);
    if (state->current == 0) {
      state->lastDirection = targetSign;
      state->blankTicks = RobotConfig::kDirectionBlankControlTicks;
    }
    return;
  }

  // current 已经 0 但 lastDirection 与 target 反号：也需要走一次 blank。
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

Timer1MotorPwm::DutyFrame MotorDriver::makeDutyFrame() const {
  Timer1MotorPwm::DutyFrame frame = {};
  applySide(left_.current, RobotConfig::kInvertLeftMotor, RobotConfig::kLeftForwardUsesIb,
            &frame.leftIa, &frame.leftIb);
  applySide(right_.current, RobotConfig::kInvertRightMotor, RobotConfig::kRightForwardUsesIb,
            &frame.rightIa, &frame.rightIb);
  return frame;
}

} // namespace lf
