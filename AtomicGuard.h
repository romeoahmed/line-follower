#pragma once

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

namespace lf {

// RAII 临界区：ctor 保存 SREG 并 cli()，dtor 原样恢复 SREG。用于保护与 ISR 共享
// 的多字节状态；AVR 单字节 load/store 本身原子，无需此守卫。non-copyable 保证
// dtor 只跑一次。
class AtomicGuard {
 public:
  AtomicGuard() : saved_(SREG) {
    cli();
  }
  ~AtomicGuard() {
    SREG = saved_;
  }

  AtomicGuard(const AtomicGuard&) = delete;
  AtomicGuard& operator=(const AtomicGuard&) = delete;
  AtomicGuard(AtomicGuard&&) = delete;
  AtomicGuard& operator=(AtomicGuard&&) = delete;

 private:
  uint8_t saved_;
};

} // namespace lf
