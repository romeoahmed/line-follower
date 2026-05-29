#pragma once

#include "Timer1MotorPwm.h"

namespace lf {

class MotorDriver {
 public:
  void begin();
  void setTargetSpeeds(int16_t left, int16_t right);
  void update();
  void stopNow();

  int16_t currentLeft() const {
    return left_.current;
  }
  int16_t currentRight() const {
    return right_.current;
  }

 private:
  struct MotorState {
    int16_t target;
    int16_t current;
    int8_t lastDirection;
    uint8_t blankTicks;
  };

  MotorState left_ = {};
  MotorState right_ = {};

  static int16_t clampSignedPwm(int16_t value);
  static int16_t applyCompensation(int16_t value, int16_t trimPermille);
  static int16_t rampToward(int16_t current, int16_t target);
  static void stepMotor(MotorState* state);
  static uint8_t magnitude(int16_t value);
  static int8_t signOf(int16_t value);
  static void applySide(int16_t signedPwm, bool invert, bool forwardUsesIb, uint8_t* iaDuty,
                        uint8_t* ibDuty);
  Timer1MotorPwm::DutyFrame makeDutyFrame() const;
};

} // namespace lf
