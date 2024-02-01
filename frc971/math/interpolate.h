#ifndef FRC971_MATH_INTERPOLATE_H_
#define FRC971_MATH_INTERPOLATE_H_
namespace frc971::math {

// Takes a and b linear interpolates between the two based on the scalar t.
// If t == 0, returns a; if t == 1.0, returns b.
// The semantics of this should be identical to std::lerp().
template <typename T, typename Scalar>
T lerp(const T &a, const T &b, Scalar t) {
  return (static_cast<Scalar>(1.0) - t) * a + t * b;
}

// For two points (x1, y1) and (x2, y2) uses a linear interpolation
// to identify the value of y at x.
// Will linearly extrapolate if x is outside of [x1, x2].
template <typename T, typename Scalar>
T Interpolate(Scalar x1, Scalar x2, const T &y1, const T &y2, Scalar x) {
  return lerp(y1, y2, (x - x1) / (x2 - x1));
}
}  // namespace frc971::math
#endif  // FRC971_MATH_INTERPOLATE_H_
