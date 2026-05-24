#include "LineSensors.h"

namespace lf {
namespace {

uint8_t countThreeBits(const uint8_t value) {
  uint8_t count = 0;
  if ((value & 0x01) != 0) {
    ++count;
  }
  if ((value & 0x02) != 0) {
    ++count;
  }
  if ((value & 0x04) != 0) {
    ++count;
  }
  return count;
}

} // namespace

void LineSensors::begin() {
  FastIo::beginSensorPins();
  FastIo::writeSensorEnable(true);

  leftHistory_ = 0;
  rightHistory_ = 0;
  sampleCount_ = 0;
  leftAnalogBlack_ = false;
  rightAnalogBlack_ = false;

  AdcDriver::begin(RobotConfig::kSensorMode == SensorMode::kAdc);
}

LineSensorSample LineSensors::sample() {
  LineSensorSample result =
      (RobotConfig::kSensorMode == SensorMode::kAdc) ? sampleAdc() : sampleDigital();

  if (sampleCount_ < 3) {
    ++sampleCount_;
  }

  // 3 样本多数表决只跨控制 tick 过滤毛刺，不在单个 tick 内忙等。
  result.leftBlack = applyHistory(&leftHistory_, result.leftBlack, sampleCount_);
  result.rightBlack = applyHistory(&rightHistory_, result.rightBlack, sampleCount_);
  return result;
}

bool LineSensors::levelMeansBlack(const bool levelHigh) {
  return levelHigh == (RobotConfig::kSensorBlackLevel == ActiveLevel::kHigh);
}

bool LineSensors::applyHistory(uint8_t* history, const bool black, const uint8_t sampleCount) {
  if (history == nullptr) {
    return black;
  }

  *history = static_cast<uint8_t>(((*history << 1) | (black ? 1 : 0)) & 0x07);
  if (sampleCount < 3) {
    return black;
  }
  return countThreeBits(*history) >= 2;
}

bool LineSensors::adcMeansBlack(const uint16_t value, const bool previousBlack) {
  const uint16_t threshold = RobotConfig::kAdcBlackThreshold;
  const uint16_t hysteresis = RobotConfig::kAdcHysteresis;

  if (RobotConfig::kSensorBlackLevel == ActiveLevel::kHigh) {
    const uint16_t risingThreshold = threshold + hysteresis;
    const uint16_t fallingThreshold = (threshold > hysteresis) ? (threshold - hysteresis) : 0;
    return previousBlack ? (value >= fallingThreshold) : (value >= risingThreshold);
  }

  // 模拟模式使用滞回，避免黑白阈值附近反复抖动。
  const uint16_t lowThreshold = (threshold > hysteresis) ? (threshold - hysteresis) : 0;
  const uint16_t releaseThreshold = threshold + hysteresis;
  return previousBlack ? (value <= releaseThreshold) : (value <= lowThreshold);
}

LineSensorSample LineSensors::sampleDigital() {
  const uint8_t portC = FastIo::readSensorPortC();
  LineSensorSample result = {};
  result.leftRaw = (portC & FastIo::kLeftSensorOutMask) != 0 ? 1 : 0;
  result.rightRaw = (portC & FastIo::kRightSensorOutMask) != 0 ? 1 : 0;
  result.leftBlack = levelMeansBlack(result.leftRaw != 0);
  result.rightBlack = levelMeansBlack(result.rightRaw != 0);
  result.valid = true;
  return result;
}

LineSensorSample LineSensors::sampleAdc() {
  LineSensorSample result = {};

  uint16_t leftValue = 0;
  uint16_t rightValue = 0;
  const bool leftOk = AdcDriver::read(AdcDriver::Channel::kAdc1, &leftValue);
  const bool rightOk = AdcDriver::read(AdcDriver::Channel::kAdc0, &rightValue);

  result.valid = leftOk && rightOk;
  result.leftRaw = leftValue;
  result.rightRaw = rightValue;
  if (!result.valid) {
    return result;
  }

  leftAnalogBlack_ = adcMeansBlack(leftValue, leftAnalogBlack_);
  rightAnalogBlack_ = adcMeansBlack(rightValue, rightAnalogBlack_);
  result.leftBlack = leftAnalogBlack_;
  result.rightBlack = rightAnalogBlack_;
  return result;
}

} // namespace lf
