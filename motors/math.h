#ifndef MOTORS_MATH_H_
#define MOTORS_MATH_H_

#include <limits.h>

#include <complex>
#include <ratio>

// This file has some specialized math functions useful for implementing our
// controls in a minimal number of cycles.

namespace frc971 {
namespace motors {

inline constexpr unsigned int Log2RoundUp(unsigned int x) {
  return (x < 2) ? x : (1 + Log2RoundUp(x / 2));
}

template <typename T>
inline constexpr const T &ConstexprMax(const T &a, const T &b) {
  return (a < b) ? b : a;
}

namespace math_internal {

constexpr uint32_t SinCosTableSize() { return 4096; }

constexpr float FloatMaxMagnitude() { return 1.0f; }

constexpr bool IsPowerOf2(uint32_t value) {
  return value == (1u << (Log2RoundUp(value) - 1));
}

static_assert(IsPowerOf2(SinCosTableSize()), "Tables need to be a power of 2");

extern float sin_int_table[SinCosTableSize()];
extern float cos_int_table[SinCosTableSize()];
extern float sin_float_table[SinCosTableSize() + 1];
extern float cos_float_table[SinCosTableSize() + 1];

template <class Rotation>
float FastTableLookupInt(uint32_t theta, const float *table) {
  static_assert(IsPowerOf2(Rotation::den),
                "Denominator needs to be a power of 2");

  // Don't need to worry about the sizes of intermediates given this constraint.
  static_assert(
      ConstexprMax<uint32_t>(Rotation::den, SinCosTableSize()) * Rotation::num <
          UINT32_MAX,
      "Numerator and denominator are too big");

  // Rounding/truncating here isn't supported.
  static_assert(Rotation::den <= SinCosTableSize(),
                "Tables need to be bigger");

  // Don't feel like thinking through the consequences of this not being true.
  static_assert(Rotation::num > 0 && Rotation::den > 0,
                "Need a positive ratio");

  constexpr uint32_t kDenominatorRatio = SinCosTableSize() / Rotation::den;

  // These should always be true given the other constraints.
  static_assert(kDenominatorRatio * Rotation::den == SinCosTableSize(),
                "Math is broken");
  static_assert(IsPowerOf2(kDenominatorRatio), "Math is broken");

  return table[(theta * kDenominatorRatio * Rotation::num +
                kDenominatorRatio * Rotation::num / 2) %
               SinCosTableSize()];
}

inline float FastTableLookupFloat(float theta, const float *table) {
  static constexpr float kScalar =
      (SinCosTableSize() / 2) / FloatMaxMagnitude();
  const int index =
      (SinCosTableSize() / 2) + static_cast<int32_t>(theta * kScalar);
  return table[index];
}

}  // namespace math_internal

// All theta arguments to the float-based functions must be in [-0.2, 0.2].

inline float FastSinFloat(float theta) {
  return math_internal::FastTableLookupFloat(theta,
                                             math_internal::sin_float_table);
}

inline float FastCosFloat(float theta) {
  return math_internal::FastTableLookupFloat(theta,
                                             math_internal::cos_float_table);
}

inline ::std::complex<float> ImaginaryExpFloat(float theta) {
  return ::std::complex<float>(FastCosFloat(theta), FastSinFloat(theta));
}

// The integer-based sin/cos functions all have a Rotation template argument,
// which should be a ::std::ratio. The real argument to the trigonometric
// function is this ratio multiplied by the theta argument.
//
// Specifically, they return the function evaluated at
// (Rotation * (theta + 0.5)).
//
// All theta arguments must be in [0, Rotation::den).
//
// All denominators must be powers of 2.

template<class Rotation>
float FastSinInt(uint32_t theta) {
  return math_internal::FastTableLookupInt<Rotation>(
      theta, math_internal::sin_int_table);
}

template<class Rotation>
float FastCosInt(uint32_t theta) {
  return math_internal::FastTableLookupInt<Rotation>(
      theta, math_internal::cos_int_table);
}

template<class Rotation>
::std::complex<float> ImaginaryExpInt(uint32_t theta) {
  return ::std::complex<float>(FastCosInt<Rotation>(theta),
                               FastSinInt<Rotation>(theta));
}

void MathInit();

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_MATH_H_
