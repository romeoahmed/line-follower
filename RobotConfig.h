#pragma once

#include "BoardProfile.h"
#include "PdController.h"

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

enum class MotorDriveMode : uint8_t {
  kCoastLowSidePwm = 0,
  kBrakeHighSideInversePwm = 1,
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

// L9110S-MS 供应链参数 2.5-12 V、1.2 A continuous、2.0 A peak，固件默认保守限幅。
//
// PWM 数值语义（默认 kBrakeHighSideInversePwm 驱动模式，ADR-006）：方向输入整
// 周期 HIGH，另一输入输出 (255-duty) 反相 PWM。kMotor*Pwm 数值 = 电机两端有效
// 驱动占空比 × 255：duty=128 ≈ 50% 驱动，duty=255 = 整周期驱动。
//
// 教学小车减速直流电机典型空载启动电压 1.5-2.5 V，带摩擦后通常需要 40-60% 驱动
// 才能可靠起步。当前默认 50-65% 配合 kMotorMinimumEffectivePwm 跳过死区。硬件
// 阶段实测电流、温升、电池压降后再决定是否继续上调 kMotorMaxPwm。
constexpr uint8_t kPwmFullScale = 255;
constexpr uint8_t kMotorStraightPwm = 160;
constexpr uint8_t kMotorCurvePwm = 130;
constexpr uint8_t kMotorCautiousPwm = 110;
constexpr uint8_t kMotorSearchPwm = 140;
constexpr uint8_t kMotorMaxPwm = 200;
constexpr uint8_t kMotorRampStepPerControlTick = 12;
constexpr uint8_t kDirectionBlankControlTicks = 1;
// 启动死区跳变：current 跨 0 时 MotorDriver 直接跳到至少这个量级，避免 ramp 头
// 几步停在电机死区。设为 0 关闭此机制。
constexpr uint8_t kMotorMinimumEffectivePwm = 90;
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
constexpr bool kRightForwardUsesIb = false;
constexpr MotorDriveMode kMotorDriveMode = MotorDriveMode::kBrakeHighSideInversePwm;

constexpr SensorMode kSensorMode = SensorMode::kDigital;
constexpr ActiveLevel kSensorEnableActiveLevel = ActiveLevel::kHigh;
constexpr ActiveLevel kSensorBlackLevel = ActiveLevel::kLow;
constexpr CenterMode kCenterMode = CenterMode::kBetweenSensors;

constexpr uint16_t kAdcBlackThreshold = 512;
constexpr uint16_t kAdcHysteresis = 24;

constexpr uint8_t kSensorSettleControlTicks = 10;
// 在 BetweenSensors 模式下，双白可能是“线在两个传感器之间正常居中”，也可能是“车已经
// 飞出轨道”。只有当过去看到过偏差（lastError != 0）后才计时，避免长直线居中场景误判。
// 之后再持续双白达到这个 tick 数，就进入失线状态去搜线/超时停车——这是默认安全行为，
// 不是可选启发式。0 关闭该机制（不推荐：意味着脱轨后小车会一直按 cautious 速度直行）。
constexpr uint8_t kAmbiguousCenterLimitTicks = 80;
constexpr uint8_t kLineLostStopTicks = 80;

constexpr bool kObstacleAvoidanceEnabled = true;
constexpr uint16_t kObstacleStopDistanceMm = 200;
constexpr uint8_t kObstacleConfirmSamples = 2;
constexpr uint8_t kObstacleClearSamples = 2;
constexpr uint8_t kObstacleStopHoldControlTicks = 8;
// 原地转向比直行更难启动（两轮反向、轮胎横向摩擦更大），右转 PWM 要明显高于直行下限。
constexpr uint8_t kObstacleRightTurnPwm = 140;
constexpr uint16_t kObstacleRightTurnControlTicks = 55;

// HC-SR04 datasheet 标注最小 10 us；部分 SR04 克隆要求严格大于 10 us。这里留 2 us
// 余量，吸收主循环到 finishTriggerIfDue() 的轮询抖动以及 Timer1/PCINT ISR 抢占。
constexpr uint8_t kUltrasonicTriggerPulseUs = 12;
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

// PD 增益类型由 PdController 定义；这里只给出每个 profile 的标定值。
constexpr PdGainsQ8 kPdStraightGainsQ8 = {12, 4};
constexpr PdGainsQ8 kPdCurveGainsQ8 = {22, 10};
constexpr int16_t kPdStraightMaxCorrection = 50;
constexpr int16_t kPdCurveMaxCorrection = 86;
constexpr int16_t kLineErrorUnit = 1000;
constexpr int16_t kLineCurveErrorThreshold = kLineErrorUnit / 2;

static_assert(kPdStraightMaxCorrection >= 0 && kPdStraightMaxCorrection <= kMotorMaxPwm,
              "直线 PD 修正量必须非负且不超过电机限幅。");
static_assert(kPdCurveMaxCorrection >= 0 && kPdCurveMaxCorrection <= kMotorMaxPwm,
              "弯道 PD 修正量必须非负且不超过电机限幅。");
static_assert(kLineCurveErrorThreshold > 0, "弯道判断阈值必须为正。");

} // namespace RobotConfig
} // namespace lf
