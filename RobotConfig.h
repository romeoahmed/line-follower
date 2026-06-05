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
// ADR-012 把行为从「黑线循迹」改为「默认直行、遇黑左转」后，速度档位也合并：
//   - kMotorCruisePwm：默认直行基速。原 kMotorStraightPwm=160，新值 180（+12.5%）。
//   - kEncounterTurnLeftPwm：遇黑触发的开环左转 PWM。
//   - 原 kMotorCurvePwm / kMotorCautiousPwm / kMotorSearchPwm 在新行为下没有消费者，
//     删除以避免「看起来在调但实际无效」的死配置。
//
// kMotorMaxPwm 从 200 上调到 220（+10%）。仍显著低于满量程 255，给电池压降和
// L9110S 温升留余量。硬件阶段必须实测电流/温升/电池压降后再决定是否继续上调。
constexpr uint8_t kPwmFullScale = 255;
constexpr uint8_t kMotorCruisePwm = 180;
constexpr uint8_t kMotorMaxPwm = 220;
constexpr uint8_t kMotorRampStepPerControlTick = 12;
constexpr uint8_t kDirectionBlankControlTicks = 1;
// 启动死区跳变：current 跨 0 时 MotorDriver 直接跳到至少这个量级，避免 ramp 头
// 几步停在电机死区。设为 0 关闭此机制。
constexpr uint8_t kMotorMinimumEffectivePwm = 90;
constexpr int16_t kLeftMotorTrimPermille = 0;
constexpr int16_t kRightMotorTrimPermille = 0;

static_assert(kMotorCruisePwm <= kMotorMaxPwm, "巡航速度不能超过电机限幅。");
static_assert(kMotorMinimumEffectivePwm <= kMotorMaxPwm, "启动死区补偿不能超过电机限幅。");
static_assert(kMotorMaxPwm < kPwmFullScale, "kMotorMaxPwm 必须留余量给 255-duty 反相 PWM。");
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

// 遇黑左转（ADR-012；触发严苛化见 ADR-012 Revision 1 / 同步更新）：
//   - kEncounterTurnLeftPwm：开环左转期间双轮的速度幅值；L 反转、R 正转。
//     与原地避障右转 (kObstacleRightTurnPwm=140) 同量级；左转 PWM 略高 (160)
//     补偿首次校准的预算。
//   - kEncounterTurnLeftControlTicks：开环左转持续多少个 10 ms 控制 tick。
//     **初值 = 右转开环 tick 数**（对称起点），硬件阶段按实测目标转角重新标定；
//     与 kObstacleRightTurnControlTicks 一样不是几何保证，参考 ADR-005 的诚实化做法。
//   - kEncounterConfirmTicks：连续 N 个 control tick **两个传感器都见黑**才
//     触发左转。触发判定本身已经从"任一见黑"严苛到"双黑"（estimateSawBlack
//     在 RobotController.cpp）；去抖窗口默认 5 tick（= 50 ms）覆盖一次性反光、
//     单帧噪声、车头跨黑线时的边缘抖动。设 1 即关闭去抖。
constexpr uint8_t kEncounterTurnLeftPwm = 160;
constexpr uint16_t kEncounterTurnLeftControlTicks = 55;
constexpr uint8_t kEncounterConfirmTicks = 5;

static_assert(kEncounterTurnLeftPwm <= kMotorMaxPwm, "遇黑左转速度不能超过电机限幅。");
static_assert(kEncounterTurnLeftControlTicks > 0, "遇黑左转持续时间必须为正。");
static_assert(kEncounterConfirmTicks >= 1, "遇黑触发的去抖样本数必须 ≥ 1。");

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

// LineEstimator 发出的离散误差幅值；新行为 (ADR-012) 不消费 error 数值（控制层
// 只基于 LineState 分支），保留是为了：① 不破坏 LineEstimator 与 LineEstimate 的
// 现有约定 ② 未来若切换 kOnLine 模式或加入 ADC 加权再使用。
constexpr int16_t kLineErrorUnit = 1000;

} // namespace RobotConfig
} // namespace lf
