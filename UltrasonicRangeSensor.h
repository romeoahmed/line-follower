#pragma once

#include "RobotConfig.h"
#include "Timer1MotorPwm.h"

namespace lf {

class UltrasonicRangeSensor {
 public:
  void begin();
  void poll();

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
