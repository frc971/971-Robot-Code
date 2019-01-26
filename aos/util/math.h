#ifndef AOS_UTIL_MATH_H_
#define AOS_UTIL_MATH_H_

#include <cmath>

#include "Eigen/Dense"

namespace aos {
namespace math {

// Normalizes an angle to be in (-M_PI, M_PI]
template <typename Scalar>
constexpr Scalar NormalizeAngle(Scalar theta) {
  // First clause takes care of getting theta into
  // (-3 * M_PI, M_PI)
  const int n_pi_pos = (theta + M_PI) / 2.0 / M_PI;
  theta -= n_pi_pos * 2.0 * M_PI;
  // Next we fix it to cut off the bottom half of the above
  // range and bring us into (-M_PI, M_PI]
  const int n_pi_neg = (theta - M_PI) / 2.0 / M_PI;
  theta -= n_pi_neg * 2.0 * M_PI;
  return theta;
}

// Calculate a - b and return the result in (-M_PI, M_PI]
template <typename Scalar>
constexpr Scalar DiffAngle(Scalar a, Scalar b) {
  return NormalizeAngle(a - b);
}

// Returns whether points A, B, C are arranged in a counter-clockwise manner on
// a 2-D plane.
// Collinear points of any sort will cause this to return false.
// Source: https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
// Mathod:
// 3 points on a plane will form a triangle (unless they are collinear), e.g.:
//                        A-------------------C
//                         \                 /
//                          \               /
//                           \             /
//                            \           /
//                             \         /
//                              \       /
//                               \     /
//                                \   /
//                                 \ /
//                                  B
// We are interested in whether A->B->C is the counter-clockwise direction
// around the triangle (it is in this picture).
// Essentially, we want to know whether the angle between A->B and A->C is
// positive or negative.
// For this, consider the cross-product, where we imagine a third z-axis
// coming out of the page. The cross-product AB x AC will be positive if ABC
// is counter-clockwise and negative if clockwise (and zero if collinear).
// The z-component (which is the only non-zero component) of the cross-product
// is AC.y * AB.x - AB.y * AC.x > 0, which turns into:
// AC.y * AB.x > AB.y * AC.x
// (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)
// which is exactly what we have below.
template <typename Scalar>
constexpr bool PointsAreCCW(const Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>> &A,
                            const Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>> &B,
                            const Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>> &C) {
  return (C.y() - A.y()) * (B.x() - A.x()) > (B.y() - A.y()) * (C.x() - A.x());
}

}  // namespace math
}  // namespace aos

#endif  // AOS_UTIL_MATH_H_
