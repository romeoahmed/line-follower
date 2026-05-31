#include "LineSensors.h"

namespace lf {
namespace {

// AVR 上比 __builtin_popcount 小：后者会链接一个库函数。kHistoryDepth=3 时这是单字节 ≤7。
uint8_t countBits(uint8_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += (value & 1u);
    value >>= 1;
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

  if (sampleCount_ < kHistoryDepth) {
    ++sampleCount_;
  }

  // 跨 tick 多数表决，不在单 tick 内忙等。warmup 早返回（sampleCount_ < depth）在
  // 正常控制流中不会命中——kSensorSettle 已先跑满采样窗口；保留为防御性 fallback。
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
  *history = static_cast<uint8_t>(((*history << 1) | (black ? 1u : 0u)) & kHistoryMask);
  if (sampleCount < kHistoryDepth) {
    return black;
  }
  return countBits(*history) >= kHistoryMajority;
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
