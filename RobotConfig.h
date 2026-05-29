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
constexpr uint8_t kTimer1TicksPerMicrosecond = BoardProfile::kCpuHz / kTimer1Prescaler / 1000000UL;

static_assert(kPwmFrequencyHz == 4000, "默认 PWM 频率必须是 4 kHz。");
static_assert(kControlTickHz == 100, "控制周期必须是 10 ms。");
static_assert(kTimer1PwmTop < 65535, "Timer1 TOP 必须落在 16-bit 范围内。");
static_assert(kTimer1TicksPerMicrosecond == 2, "Timer1 时间戳必须保持 0.5 us 分辨率。");

// L9110S-MS 供应链参数为 2.5-12 V、1.2 A continuous、2.0 A peak；固件默认保守限幅。
constexpr uint8_t kPwmFullScale = 255;
constexpr uint8_t kMotorStraightPwm = 76;
constexpr uint8_t kMotorCurvePwm = 58;
constexpr uint8_t kMotorCautiousPwm = 48;
constexpr uint8_t kMotorSearchPwm = 46;
constexpr uint8_t kMotorMaxPwm = 120;
constexpr uint8_t kMotorRampStepPerControlTick = 5;
constexpr uint8_t kDirectionBlankControlTicks = 1;
constexpr uint8_t kMotorMinimumEffectivePwm = 0;
constexpr int16_t kLeftMotorTrimPermille = 0;
constexpr int16_t kRightMotorTrimPermille = 0;

static_assert(kMotorStraightPwm <= kMotorMaxPwm, "直线速度不能超过电机限幅。");
static_assert(kMotorCurvePwm <= kMotorMaxPwm, "弯道速度不能超过电机限幅。");
static_assert(kMotorCautiousPwm <= kMotorMaxPwm, "保守速度不能超过电机限幅。");
static_assert(kMotorSearchPwm <= kMotorMaxPwm, "寻线速度不能超过电机限幅。");
static_assert(kMotorMinimumEffectivePwm <= kMotorMaxPwm, "启动死区补偿不能超过电机限幅。");
static_assert(kLeftMotorTrimPermille >= -500 && kLeftMotorTrimPermille <= 500,
              "左电机补偿默认限制在 +/-50%。");
static_assert(kRightMotorTrimPermille >= -500 && kRightMotorTrimPermille <= 500,
              "右电机补偿默认限制在 +/-50%。");

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
// 0 表示不凭“传感器夹线时的双白中心候选”强行判定失线。
constexpr uint8_t kAmbiguousCenterLimitTicks = 0;
constexpr uint8_t kLineLostStopTicks = 80;

constexpr bool kObstacleAvoidanceEnabled = true;
constexpr uint16_t kObstacleStopDistanceMm = 200;
constexpr uint8_t kObstacleConfirmSamples = 2;
constexpr uint8_t kObstacleClearSamples = 2;
constexpr uint8_t kObstacleStopHoldControlTicks = 8;
constexpr uint8_t kObstacleRightTurnPwm = 55;
constexpr uint16_t kObstacleRightTurnControlTicks = 55;

constexpr uint8_t kUltrasonicTriggerPulseUs = 10;
constexpr uint16_t kUltrasonicMeasurementIntervalMs = 60;
constexpr uint16_t kUltrasonicEchoTimeoutUs = 38000;
constexpr uint16_t kUltrasonicMaxDistanceMm = 4000;
constexpr uint16_t kUltrasonicNearFieldDistanceMm = 20;

constexpr uint32_t kUltrasonicTriggerPulseTimerTicks =
    static_cast<uint32_t>(kUltrasonicTriggerPulseUs) * kTimer1TicksPerMicrosecond;
constexpr uint32_t kUltrasonicMeasurementIntervalTimerTicks =
    static_cast<uint32_t>(kUltrasonicMeasurementIntervalMs) * 1000UL * kTimer1TicksPerMicrosecond;
constexpr uint32_t kUltrasonicEchoTimeoutTimerTicks =
    static_cast<uint32_t>(kUltrasonicEchoTimeoutUs) * kTimer1TicksPerMicrosecond;

static_assert(kObstacleStopDistanceMm > kUltrasonicNearFieldDistanceMm,
              "避障停车距离必须高于 HC-SR04 近场不可靠区。");
static_assert(kObstacleRightTurnPwm <= kMotorMaxPwm, "避障右转速度不能超过电机限幅。");
static_assert(kObstacleRightTurnControlTicks > 0, "避障右转持续时间必须为正。");
static_assert(kUltrasonicTriggerPulseUs >= 10, "HC-SR04 TRIG 高电平至少需要 10 us。");
static_assert(kUltrasonicMeasurementIntervalMs >= 60, "HC-SR04 两次测距间隔至少保守取 60 ms。");

// PID 使用 Q8 定点增益：实际增益 = 常量 / 256。
struct PidGainsQ8 {
  int16_t kp;
  int16_t ki;
  int16_t kd;
};

constexpr PidGainsQ8 kPidStraightGainsQ8 = {12, 0, 4};
constexpr PidGainsQ8 kPidCurveGainsQ8 = {22, 0, 10};
constexpr int16_t kPidIntegralLimit = 4000;
constexpr int16_t kPidStraightMaxCorrection = 50;
constexpr int16_t kPidCurveMaxCorrection = 86;
constexpr int16_t kLineErrorUnit = 1000;
constexpr int16_t kLineCurveErrorThreshold = kLineErrorUnit / 2;

static_assert(kPidStraightMaxCorrection <= kMotorMaxPwm, "直线 PID 修正量不能超过电机限幅。");
static_assert(kPidCurveMaxCorrection <= kMotorMaxPwm, "弯道 PID 修正量不能超过电机限幅。");
static_assert(kLineCurveErrorThreshold > 0, "弯道判断阈值必须为正。");

} // namespace RobotConfig
} // namespace lf
