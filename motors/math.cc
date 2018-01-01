#include "motors/math.h"

#include <math.h>

namespace frc971 {
namespace salsa {
namespace math_internal {

float sin_int_table[SinCosTableSize()];
float cos_int_table[SinCosTableSize()];
float sin_float_table[SinCosTableSize() + 1];
float cos_float_table[SinCosTableSize() + 1];

}  // namespace math_internal

using math_internal::SinCosTableSize;

__attribute__((cold)) void MathInit() {
  for (uint32_t i = 0; i < SinCosTableSize(); ++i) {
    const double int_theta =
        ((static_cast<double>(i) + 0.5) / SinCosTableSize()) * 2.0 * M_PI;
    math_internal::sin_int_table[i] = sin(int_theta);
    math_internal::cos_int_table[i] = cos(int_theta);
  }
  for (uint32_t i = 0; i < SinCosTableSize() + 1; ++i) {
    const double float_theta =
        (static_cast<int32_t>(i) -
         static_cast<int32_t>(SinCosTableSize() / 2)) *
        static_cast<double>(math_internal::FloatMaxMagnitude()) /
        static_cast<double>(SinCosTableSize() / 2);
    math_internal::sin_float_table[i] = sin(float_theta);
    math_internal::cos_float_table[i] = cos(float_theta);
  }
}

}  // namespace salsa
}  // namespace frc971
