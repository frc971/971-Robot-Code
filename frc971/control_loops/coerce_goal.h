#ifndef FRC971_CONTROL_LOOPS_COERCE_GOAL_H_
#define FRC971_CONTROL_LOOPS_COERCE_GOAL_H_

#include "Eigen/Dense"
#include "frc971/control_loops/polytope.h"

namespace frc971 {
namespace control_loops {

template <typename Scalar = double>
Eigen::Matrix<Scalar, 2, 1> DoCoerceGoal(
    const frc971::controls::HVPolytope<2, 4, 4, Scalar> &region,
    const Eigen::Matrix<Scalar, 1, 2> &K, Scalar w,
    const Eigen::Matrix<Scalar, 2, 1> &R, bool *is_inside);

// Intersects a line with a region, and finds the closest point to R.
// Finds a point that is closest to R inside the region, and on the line
// defined by K X = w.  If it is not possible to find a point on the line,
// finds a point that is inside the region and closest to the line.
template <typename Scalar = double>
static inline Eigen::Matrix<Scalar, 2, 1> CoerceGoal(
    const frc971::controls::HVPolytope<2, 4, 4, Scalar> &region,
    const Eigen::Matrix<Scalar, 1, 2> &K, Scalar w,
    const Eigen::Matrix<Scalar, 2, 1> &R) {
  return DoCoerceGoal(region, K, w, R, nullptr);
}

template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> DoCoerceGoal(
    const frc971::controls::HVPolytope<2, 4, 4, Scalar> &region,
    const Eigen::Matrix<Scalar, 1, 2> &K, Scalar w,
    const Eigen::Matrix<Scalar, 2, 1> &R, bool *is_inside) {
  if (region.IsInside(R)) {
    if (is_inside) *is_inside = true;
    return R;
  }
  const Scalar norm_K = K.norm();
  Eigen::Matrix<Scalar, 2, 1> parallel_vector;
  Eigen::Matrix<Scalar, 2, 1> perpendicular_vector;
  // Calculate a vector that is perpendicular to the line defined by K * x = w.
  perpendicular_vector = K.transpose() / norm_K;
  // Calculate a vector that is parallel to the line defined by K * x = w.
  parallel_vector << perpendicular_vector(1, 0), -perpendicular_vector(0, 0);

  // Calculate the location along the K x = w line where each boundary of the
  // polytope would intersect.
  // I.e., we want to calculate the distance along the K x = w line, as
  // represented by
  // parallel_vector * dist + perpendicular_vector * w / norm(K),
  // such that it intersects with H_i * x = k_i.
  // This gives us H_i * (parallel * dist + perp * w / norm(K)) = k_i.
  // dist = (k_i - H_i * perp * w / norm(K)) / (H_i * parallel)
  // projectedh is the numerator, projectedk is the denominator.
  // The case where H_i * parallel is zero indicates a scenario where the given
  // boundary of the polytope is parallel to the line, and so there is no
  // meaningful value of dist to return.
  // Note that the sign of projectedh will also indicate whether the distance is
  // an upper or lower bound. If since valid points satisfy H_i * x < k_i, then
  // if H_i * parallel is less than zero, then dist will be a lower bound and
  // if it is greater than zero, then dist will be an upper bound.
  const Eigen::Matrix<Scalar, 4, 1> projectedh =
      region.static_H() * parallel_vector;
  const Eigen::Matrix<Scalar, 4, 1> projectedk =
      region.static_k() - region.static_H() * perpendicular_vector * w / norm_K;

  Scalar min_boundary = ::std::numeric_limits<Scalar>::lowest();
  Scalar max_boundary = ::std::numeric_limits<Scalar>::max();
  for (int i = 0; i < 4; ++i) {
    if (projectedh(i, 0) > 0) {
      max_boundary =
          ::std::min(max_boundary, projectedk(i, 0) / projectedh(i, 0));
    } else if (projectedh(i, 0) != 0) {
      min_boundary =
          ::std::max(min_boundary, projectedk(i, 0) / projectedh(i, 0));
    }
  }

  Eigen::Matrix<Scalar, 1, 2> vertices;
  vertices << max_boundary, min_boundary;

  if (max_boundary > min_boundary) {
    // The line goes through the region (if the line just intersects a single
    // vertex, then we fall through to the next clause), but R is not
    // inside the region. The returned point will be one of the two points where
    // the line intersects the edge of the region.
    Scalar min_distance_sqr = 0;
    Eigen::Matrix<Scalar, 2, 1> closest_point;
    for (int i = 0; i < vertices.innerSize(); i++) {
      Eigen::Matrix<Scalar, 2, 1> point;
      point =
          parallel_vector * vertices(0, i) + perpendicular_vector * w / norm_K;
      const Scalar length = (R - point).squaredNorm();
      if (i == 0 || length < min_distance_sqr) {
        closest_point = point;
        min_distance_sqr = length;
      }
    }
    if (is_inside) *is_inside = true;
    return closest_point;
  } else {
    // The line does not pass through the region; identify the vertex closest to
    // the line.
    Eigen::Matrix<Scalar, 2, 4> region_vertices = region.StaticVertices();
#ifdef __linux__
    CHECK_GT(reinterpret_cast<ssize_t>(region_vertices.outerSize()), 0);
#else
    assert(region_vertices.outerSize() > 0);
#endif
    Scalar min_distance = INFINITY;
    int closest_i = 0;
    for (int i = 0; i < region_vertices.outerSize(); i++) {
      // Calculate the distance of the vertex from the line. The closest vertex
      // will be the one to return.
      const Scalar length = ::std::abs(
          (perpendicular_vector.transpose() * (region_vertices.col(i)))(0, 0) -
          w);
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
