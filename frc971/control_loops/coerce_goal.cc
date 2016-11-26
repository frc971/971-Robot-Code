#include "frc971/control_loops/coerce_goal.h"

#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"

namespace frc971 {
namespace control_loops {

Eigen::Matrix<double, 2, 1> DoCoerceGoal(
    const aos::controls::HVPolytope<2, 4, 4> &region,
    const Eigen::Matrix<double, 1, 2> &K, double w,
    const Eigen::Matrix<double, 2, 1> &R, bool *is_inside) {
  if (region.IsInside(R)) {
    if (is_inside) *is_inside = true;
    return R;
  }
  Eigen::Matrix<double, 2, 1> parallel_vector;
  Eigen::Matrix<double, 2, 1> perpendicular_vector;
  perpendicular_vector = K.transpose().normalized();
  parallel_vector << perpendicular_vector(1, 0), -perpendicular_vector(0, 0);

  Eigen::Matrix<double, 4, 1> projectedh = region.static_H() * parallel_vector;
  Eigen::Matrix<double, 4, 1> projectedk =
      region.static_k() - region.static_H() * perpendicular_vector * w;

  double min_boundary = ::std::numeric_limits<double>::lowest();
  double max_boundary = ::std::numeric_limits<double>::max();
  for (int i = 0; i < 4; ++i) {
    if (projectedh(i, 0) > 0) {
      max_boundary =
          ::std::min(max_boundary, projectedk(i, 0) / projectedh(i, 0));
    } else {
      min_boundary =
          ::std::max(min_boundary, projectedk(i, 0) / projectedh(i, 0));
    }
  }

  Eigen::Matrix<double, 1, 2> vertices;
  vertices << max_boundary, min_boundary;

  if (max_boundary > min_boundary) {
    double min_distance_sqr = 0;
    Eigen::Matrix<double, 2, 1> closest_point;
    for (int i = 0; i < vertices.innerSize(); i++) {
      Eigen::Matrix<double, 2, 1> point;
      point = parallel_vector * vertices(0, i) + perpendicular_vector * w;
      const double length = (R - point).squaredNorm();
      if (i == 0 || length < min_distance_sqr) {
        closest_point = point;
        min_distance_sqr = length;
      }
    }
    if (is_inside) *is_inside = true;
    return closest_point;
  } else {
    Eigen::Matrix<double, 2, 4> region_vertices =
        region.StaticVertices();
    CHECK_GT(region_vertices.outerSize(), 0);
    double min_distance = INFINITY;
    int closest_i = 0;
    for (int i = 0; i < region_vertices.outerSize(); i++) {
      const double length = ::std::abs(
          (perpendicular_vector.transpose() * (region_vertices.col(i)))(0, 0));
      if (i == 0 || length < min_distance) {
        closest_i = i;
        min_distance = length;
      }
    }
    if (is_inside) *is_inside = false;
    return (Eigen::Matrix<double, 2, 1>() << region_vertices(0, closest_i),
            region_vertices(1, closest_i)).finished();
  }
}

}  // namespace control_loops
}  // namespace frc971
