#ifndef MOTORS_MATH_H_
#define MOTORS_MATH_H_

#include <limits.h>

#include <array>
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

constexpr uint32_t SinCosFloatTableSize() { return 2048; }

constexpr float FloatMaxMagnitude() { return 1.0f; }

constexpr bool IsPowerOf2(uint32_t value) {
  return value == (1u << (Log2RoundUp(value) - 1));
}

static_assert(IsPowerOf2(SinCosFloatTableSize()),
              "Tables need to be a power of 2");

extern float sin_float_table[SinCosFloatTableSize() + 1];
extern float cos_float_table[SinCosFloatTableSize() + 1];

template <class Rotation, int kTableSize>
float FastTableLookupInt(uint32_t theta, const float *table) {
  static_assert(IsPowerOf2(Rotation::den),
                "Denominator needs to be a power of 2");

  // Don't need to worry about the sizes of intermediates given this constraint.
  static_assert(
      ConstexprMax<uint32_t>(Rotation::den, kTableSize) * Rotation::num <
          UINT32_MAX,
      "Numerator and denominator are too big");

  // Rounding/truncating here isn't supported.
  static_assert(Rotation::den <= kTableSize, "Tables need to be bigger");

  // Don't feel like thinking through the consequences of this not being true.
  static_assert(Rotation::num > 0 && Rotation::den > 0,
                "Need a positive ratio");

  constexpr uint32_t kDenominatorRatio = kTableSize / Rotation::den;

  // These should always be true given the other constraints.
  static_assert(kDenominatorRatio * Rotation::den == kTableSize,
                "Math is broken");
  static_assert(IsPowerOf2(kDenominatorRatio), "Math is broken");

  return table[(theta * kDenominatorRatio * Rotation::num +
                kDenominatorRatio * Rotation::num / 2) %
               kTableSize];
}

inline float FastTableLookupFloat(float theta, const float *table) {
  static constexpr float kScalar =
      (SinCosFloatTableSize() / 2) / FloatMaxMagnitude();
  const int index =
      (SinCosFloatTableSize() / 2) + static_cast<int32_t>(theta * kScalar);
  return table[index];
}

// A simple parent class for arranging to initialize all the templates.
//
// This will be lower overhead than ::std::function and also adds less
// complicated stuff around static initialization time.
class GenericInitializer {
 public:
  virtual void Initialize() = 0;

 protected:
  // Don't destroy instances via pointers to this type. Do it through subclass
  // pointers instead.
  ~GenericInitializer() = default;
};

// We build up this list at static construction time so we can call all the
// contained objects in MathInit(). The contained pointers are not owned by this
// list.
//
// This has to be big enough to hold all the different sizes of integer tables
// somebody might use in one program. If it overflows, there will be a
// __builtin_abort during static initialization.
extern ::std::array<GenericInitializer *, 10> global_initializers;

// Manages initializing and access to an integer table of a single size.
template<int kTableSize>
class SinCosIntTable {
 public:
  static const float *sin_int_table() {
    // Empirically, this is enough to get GCC to actually instantiate and link
    // in the variable, without any runtime overhead.
    (void)add_my_initializer;
    return &static_sin_int_table[0];
  }
  static const float *cos_int_table() {
    (void)add_my_initializer;
    return &static_cos_int_table[0];
  }

 private:
  // An object which adds a MyInitializer to global_initializers at static
  // construction time.
  class AddMyInitializer {
   public:
    AddMyInitializer() {
      static MyInitializer initializer;
      for (size_t i = 0; i < global_initializers.size(); ++i) {
        if (global_initializers[i] == nullptr) {
          global_initializers[i] = &initializer;
          return;
        }
      }
      __builtin_trap();
    }
  };

  class MyInitializer : public GenericInitializer {
    void Initialize() override {
      for (uint32_t i = 0; i < kTableSize; ++i) {
        const double int_theta =
            ((static_cast<double>(i) + 0.5) / kTableSize) * 2.0 * M_PI;
        static_sin_int_table[i] = sin(int_theta);
        static_cos_int_table[i] = cos(int_theta);
      }
    }
  };

  static AddMyInitializer add_my_initializer;

  static float static_sin_int_table[kTableSize];
  static float static_cos_int_table[kTableSize];
};

template <int kTableSize>
typename SinCosIntTable<kTableSize>::AddMyInitializer
    SinCosIntTable<kTableSize>::add_my_initializer;

template <int kTableSize>
float SinCosIntTable<kTableSize>::static_sin_int_table[kTableSize];
template <int kTableSize>
float SinCosIntTable<kTableSize>::static_cos_int_table[kTableSize];

}  // namespace math_internal

// theta must be in [-0.2, 0.2].
inline float FastSinFloat(float theta) {
  return math_internal::FastTableLookupFloat(theta,
                                             math_internal::sin_float_table);
}

// theta must be in [-0.2, 0.2].
inline float FastCosFloat(float theta) {
  return math_internal::FastTableLookupFloat(theta,
                                             math_internal::cos_float_table);
}

// theta must be in [-0.2, 0.2].
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
//
// kTableSize is the size table it will actually use. This must be a power of 2
// >= Rotation::den. The default of Rotation::den makes the most sense, unless
// multiple denominators are needed in the same program, in which case using the
// biggest denominator for all of them will use the least memory.

template<class Rotation, int kTableSize = Rotation::den>
float FastSinInt(uint32_t theta) {
  return math_internal::FastTableLookupInt<Rotation, kTableSize>(
      theta, math_internal::SinCosIntTable<kTableSize>::sin_int_table());
}

template <class Rotation, int kTableSize = Rotation::den>
float FastCosInt(uint32_t theta) {
  return math_internal::FastTableLookupInt<Rotation, kTableSize>(
      theta, math_internal::SinCosIntTable<kTableSize>::cos_int_table());
}

template<class Rotation>
::std::complex<float> ImaginaryExpInt(uint32_t theta) {
  return ::std::complex<float>(FastCosInt<Rotation>(theta),
                               FastSinInt<Rotation>(theta));
}

// This must be called before any of the other functions.
void MathInit();

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_MATH_H_
