#pragma once

#include "BoardProfile.h"

namespace lf {

enum class ActiveLevel : uint8_t {
  kLow = 0,
  kHigh = 1,
};

enum class SensorMode : uint8_t {
  kDigital = 0,
  kAdc = 1,
};

enum class CenterMode : uint8_t {
  kBetweenSensors = 0,
  kOnLine = 1,
};

namespace RobotConfig {

// Timer1: 16 MHz / 8 = 2 MHz，TOP=499 得到 4 kHz 软件 PWM。
constexpr uint16_t kTimer1PwmTop = 499;
constexpr uint8_t kTimer1Prescaler = 8;
constexpr uint8_t kControlPeriodsPerTick = 40;
constexpr uint16_t kPwmPeriodTicks = kTimer1PwmTop + 1;
constexpr uint16_t kPwmFrequencyHz = BoardProfile::kCpuHz / kTimer1Prescaler / kPwmPeriodTicks;
constexpr uint16_t kControlTickHz = kPwmFrequencyHz / kControlPeriodsPerTick;

static_assert(kPwmFrequencyHz == 4000, "默认 PWM 频率必须是 4 kHz。");
static_assert(kControlTickHz == 100, "控制周期必须是 10 ms。");
static_assert(kTimer1PwmTop < 65535, "Timer1 TOP 必须落在 16-bit 范围内。");

// L9110S-MS 供应链参数为 2.5-12 V、1.2 A continuous、2.0 A peak；固件默认保守限幅。
constexpr uint8_t kPwmFullScale = 255;
constexpr uint8_t kMotorBasePwm = 70;
constexpr uint8_t kMotorSearchPwm = 46;
constexpr uint8_t kMotorMaxPwm = 120;
constexpr uint8_t kMotorRampStepPerControlTick = 5;
constexpr uint8_t kDirectionBlankControlTicks = 1;

static_assert(kMotorBasePwm <= kMotorMaxPwm, "基础速度不能超过电机限幅。");
static_assert(kMotorSearchPwm <= kMotorMaxPwm, "寻线速度不能超过电机限幅。");

constexpr bool kInvertLeftMotor = false;
constexpr bool kInvertRightMotor = false;
constexpr bool kLeftForwardUsesIb = true;
constexpr bool kRightForwardUsesIb = true;

constexpr SensorMode kSensorMode = SensorMode::kDigital;
constexpr ActiveLevel kSensorEnableActiveLevel = ActiveLevel::kHigh;
constexpr ActiveLevel kSensorBlackLevel = ActiveLevel::kLow;
constexpr CenterMode kCenterMode = CenterMode::kBetweenSensors;

constexpr uint16_t kAdcBlackThreshold = 512;
constexpr uint16_t kAdcHysteresis = 24;

constexpr uint8_t kSensorSettleControlTicks = 10;
constexpr uint8_t kAmbiguousCenterLimitTicks = 50;
constexpr uint8_t kLineLostStopTicks = 80;

// PID 使用 Q8 定点增益：实际增益 = 常量 / 256。
constexpr int16_t kPidKpQ8 = 18;
constexpr int16_t kPidKiQ8 = 0;
constexpr int16_t kPidKdQ8 = 8;
constexpr int16_t kPidIntegralLimit = 4000;
constexpr int16_t kPidMaxCorrection = 80;
constexpr int16_t kLineErrorUnit = 1000;

static_assert(kPidMaxCorrection <= kMotorMaxPwm, "PID 修正量不能超过电机限幅。");

} // namespace RobotConfig
} // namespace lf
