#include "BootStatus.h"

#include <avr/io.h>
#include <avr/wdt.h>

namespace lf {
namespace BootStatus {
namespace {

// .noinit：C 运行时不在 main() 前清零，保证读到 .init3 写入的值。
uint8_t g_mcusrMirror __attribute__((section(".noinit")));

// .init3 + naked + used：在 .data/.bss 初始化前运行；无 prologue；LTO 不会丢。
// 顺序敏感：MCUSR 必须先清零再 wdt_disable()，否则 WDRF 让 WDE 复位 → 死循环。
// 模板出自 avr-libc <avr/wdt.h> "Handling the Watchdog Reset Flag"。
extern "C" void lf_captureMcusrAndDisableWdt(void) __attribute__((naked, used, section(".init3")));

extern "C" void lf_captureMcusrAndDisableWdt(void) {
  g_mcusrMirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

} // namespace

uint8_t lastMcusr() {
  return g_mcusrMirror;
}

bool lastResetWasWatchdog() {
  return (g_mcusrMirror & _BV(WDRF)) != 0;
}

bool lastResetWasBrownOut() {
  return (g_mcusrMirror & _BV(BORF)) != 0;
}

} // namespace BootStatus
} // namespace lf
