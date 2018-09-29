#ifndef AOS_MATH_H_
#define AOS_MATH_H_

#include <cmath>

namespace aos {

// Clips a value so that it is in [min, max]
static inline double Clip(double value, double min, double max) {
  if (value > max) {
    value = max;
  } else if (value < min) {
    value = min;
  }
  return value;
}

template <typename T>
static inline int sign(T val) {
  if (val > T(0)) {
    return 1;
  } else {
    return -1;
  }
}

// Adds deadband to provided value.  deadband is the region close to the origin
// to add the deadband to, and max is the maximum input value used to re-scale
// the output after adding the deadband.
static inline double Deadband(double value, const double deadband,
                              const double max) {
  if (::std::abs(value) < deadband) {
    value = 0.0;
  } else if (value > 0.0) {
    value = (value - deadband) / (max - deadband);
  } else {
    value = (value + deadband) / (max - deadband);
  }
  return value;
}
}  // namespace aos

#endif  // AOS_MATH_H_
