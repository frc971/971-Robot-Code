#ifndef FRC971_CONTROL_LOOPS_COERCE_GOAL_H_
#define FRC971_CONTROL_LOOPS_COERCE_GOAL_H_

#include "Eigen/Dense"

#include "aos/controls/polytope.h"

namespace frc971 {
namespace control_loops {

template <typename Scalar = double>
Eigen::Matrix<Scalar, 2, 1> DoCoerceGoal(
    const aos::controls::HVPolytope<2, 4, 4, Scalar> &region,
    const Eigen::Matrix<Scalar, 1, 2> &K, Scalar w,
    const Eigen::Matrix<Scalar, 2, 1> &R, bool *is_inside);

// Intersects a line with a region, and finds the closest point to R.
// Finds a point that is closest to R inside the region, and on the line
// defined by K X = w.  If it is not possible to find a point on the line,
// finds a point that is inside the region and closest to the line.
template <typename Scalar = double>
static inline Eigen::Matrix<Scalar, 2, 1> CoerceGoal(
    const aos::controls::HVPolytope<2, 4, 4, Scalar> &region,
    const Eigen::Matrix<Scalar, 1, 2> &K, Scalar w,
    const Eigen::Matrix<Scalar, 2, 1> &R) {
  return DoCoerceGoal(region, K, w, R, nullptr);
}

template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> DoCoerceGoal(
    const aos::controls::HVPolytope<2, 4, 4, Scalar> &region,
    const Eigen::Matrix<Scalar, 1, 2> &K, Scalar w,
    const Eigen::Matrix<Scalar, 2, 1> &R, bool *is_inside) {
  if (region.IsInside(R)) {
    if (is_inside) *is_inside = true;
    return R;
  }
  Eigen::Matrix<Scalar, 2, 1> parallel_vector;
  Eigen::Matrix<Scalar, 2, 1> perpendicular_vector;
  perpendicular_vector = K.transpose().normalized();
  parallel_vector << perpendicular_vector(1, 0), -perpendicular_vector(0, 0);

  Eigen::Matrix<Scalar, 4, 1> projectedh = region.static_H() * parallel_vector;
  Eigen::Matrix<Scalar, 4, 1> projectedk =
      region.static_k() - region.static_H() * perpendicular_vector * w;

  Scalar min_boundary = ::std::numeric_limits<Scalar>::lowest();
  Scalar max_boundary = ::std::numeric_limits<Scalar>::max();
  for (int i = 0; i < 4; ++i) {
    if (projectedh(i, 0) > 0) {
      max_boundary =
          ::std::min(max_boundary, projectedk(i, 0) / projectedh(i, 0));
    } else {
      min_boundary =
          ::std::max(min_boundary, projectedk(i, 0) / projectedh(i, 0));
    }
  }

  Eigen::Matrix<Scalar, 1, 2> vertices;
  vertices << max_boundary, min_boundary;

  if (max_boundary > min_boundary) {
    Scalar min_distance_sqr = 0;
    Eigen::Matrix<Scalar, 2, 1> closest_point;
    for (int i = 0; i < vertices.innerSize(); i++) {
      Eigen::Matrix<Scalar, 2, 1> point;
      point = parallel_vector * vertices(0, i) + perpendicular_vector * w;
      const Scalar length = (R - point).squaredNorm();
      if (i == 0 || length < min_distance_sqr) {
        closest_point = point;
        min_distance_sqr = length;
      }
    }
    if (is_inside) *is_inside = true;
    return closest_point;
  } else {
    Eigen::Matrix<Scalar, 2, 4> region_vertices = region.StaticVertices();
#ifdef __linux__
    CHECK_GT(reinterpret_cast<ssize_t>(region_vertices.outerSize()), 0);
#else
    assert(region_vertices.outerSize() > 0);
#endif
    Scalar min_distance = INFINITY;
    int closest_i = 0;
    for (int i = 0; i < region_vertices.outerSize(); i++) {
      const Scalar length = ::std::abs(
          (perpendicular_vector.transpose() * (region_vertices.col(i)))(0, 0));
      if (i == 0 || length < min_distance) {
        closest_i = i;
        min_distance = length;
      }
    }
    if (is_inside) *is_inside = false;
    return (Eigen::Matrix<Scalar, 2, 1>() << region_vertices(0, closest_i),
            region_vertices(1, closest_i))
        .finished();
  }
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_COERCE_GOAL_H_
