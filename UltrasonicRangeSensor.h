#pragma once

#include "RobotConfig.h"
#include "Timer1MotorPwm.h"

namespace lf {

// 单例：ECHO 边沿由 PCINT0_vect 捕获，ISR 状态保存在 .cpp 的匿名命名空间里，因此
// 同一时刻最多能有一个实例存在。begin() 会断言这一点（构造便宜，断言只在初始化时检查
// 一次，不影响控制路径）。如果将来需要多通道超声波，必须把 ISR 共享状态显式 sharded。
class UltrasonicRangeSensor {
 public:
  UltrasonicRangeSensor();
  ~UltrasonicRangeSensor();

  UltrasonicRangeSensor(const UltrasonicRangeSensor&) = delete;
  UltrasonicRangeSensor& operator=(const UltrasonicRangeSensor&) = delete;

  void begin();
  void poll();
  void restartAfterManeuver();

  bool obstaclePresent() const {
    return obstaclePresent_;
  }

  bool hasDistance() const {
    return hasDistance_;
  }

  uint16_t distanceMillimeters() const {
    return distanceMillimeters_;
  }

  uint16_t echoMicroseconds() const {
    return echoMicroseconds_;
  }

 private:
  uint32_t lastTriggerTicks_ = 0;
  uint32_t triggerEndTicks_ = 0;
  bool triggerHigh_ = false;
  bool hasDistance_ = false;
  bool obstaclePresent_ = false;
  uint16_t distanceMillimeters_ = 0;
  uint16_t echoMicroseconds_ = 0;
  uint8_t obstacleSamples_ = 0;
  uint8_t clearSamples_ = 0;

  void resetState();
  void startTrigger(uint32_t nowTicks);
  void finishTriggerIfDue(uint32_t nowTicks);
  void startMeasurementIfDue(uint32_t nowTicks);
  void handleEchoTimeout(uint32_t nowTicks);
  void consumeEchoResult();
  void updateObstacleLatch(bool closeObstacle);

  static uint16_t pulseTicksToMicroseconds(uint32_t pulseTicks);
  static uint16_t pulseTicksToMillimeters(uint32_t pulseTicks);
};

} // namespace lf
