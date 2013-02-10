#ifndef AOS_COMMON_MATH_H_
#define AOS_COMMON_MATH_H_

namespace aos {

// Clips a value so that it is in [min, max]
inline double Clip(double value, double min, double max) {
  if (value > max) {
    value = max;
  } else if (value < min) {
    value = min;
  }
  return value;
}

}  // namespace aos

#endif  // AOS_COMMON_MATH_H_
