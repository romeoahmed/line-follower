#pragma once

#include "LineSensors.h"

namespace lf {

struct LineEstimate {
  int16_t error;
  bool valid;
  bool ambiguous;
  bool intersectionLike;
  bool lost;
};

class LineEstimator {
 public:
  static LineEstimate estimate(const LineSensorSample& sample);
};

} // namespace lf
