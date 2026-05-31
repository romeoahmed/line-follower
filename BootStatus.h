#pragma once

#include <stdint.h>

namespace lf {
namespace BootStatus {

// 上一次复位时 MCUSR 的镜像，由 .init3 段钩子在 main() 之前捕获并 wdt_disable()；
// 详见 ADR-011 §1。
uint8_t lastMcusr();
bool lastResetWasWatchdog();
bool lastResetWasBrownOut();

} // namespace BootStatus
} // namespace lf
