#include "motors/math.h"

#include <math.h>

namespace frc971 {
namespace motors {
namespace math_internal {

float sin_float_table[SinCosFloatTableSize() + 1]
    __attribute__((section(".sram_l")));
float cos_float_table[SinCosFloatTableSize() + 1]
    __attribute__((section(".sram_l")));

::std::array<GenericInitializer *, 10> global_initializers{};

}  // namespace math_internal

using math_internal::SinCosFloatTableSize;

__attribute__((cold)) void MathInit() {
  for (uint32_t i = 0; i < SinCosFloatTableSize() + 1; ++i) {
    const double float_theta =
        (static_cast<int32_t>(i) -
         static_cast<int32_t>(SinCosFloatTableSize() / 2)) *
        static_cast<double>(math_internal::FloatMaxMagnitude()) /
        static_cast<double>(SinCosFloatTableSize() / 2);
    math_internal::sin_float_table[i] = sin(float_theta);
    math_internal::cos_float_table[i] = cos(float_theta);
  }
  for (math_internal::GenericInitializer *initializer :
       math_internal::global_initializers) {
    if (initializer != nullptr) {
      initializer->Initialize();
    }
  }
}

}  // namespace motors
}  // namespace frc971
