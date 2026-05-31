#pragma once

#include <stdint.h>

namespace lf {

template <typename T> inline constexpr T clampSigned(const T value, const T magnitude) {
  return value > magnitude ? magnitude : (value < -magnitude ? -magnitude : value);
}

template <typename T> inline constexpr T maxOf(const T a, const T b) {
  return a > b ? a : b;
}

template <typename T> inline constexpr T minOf(const T a, const T b) {
  return a < b ? a : b;
}

} // namespace lf
