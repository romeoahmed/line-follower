#include "RobotController.h"

namespace {
lf::RobotController g_robot;
}

void setup() {
  g_robot.begin();
}

void loop() {
  g_robot.poll();
}
