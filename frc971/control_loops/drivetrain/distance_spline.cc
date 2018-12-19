#include "frc971/control_loops/drivetrain/distance_spline.h"

#include "frc971/control_loops/drivetrain/spline.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DistanceSpline::DistanceSpline(const Spline &spline, int num_alpha)
    : spline_(spline) {
  distances_.push_back(0.0);
  const double dalpha = 1.0 / static_cast<double>(num_alpha - 1);

  double last_alpha = 0.0;
  for (int i = 1; i < num_alpha; ++i) {
    const double alpha = dalpha * i;
    distances_.push_back(
        distances_.back() +
        GaussianQuadrature5(
            [this](double alpha) { return this->spline_.DPoint(alpha).norm(); },
            last_alpha, alpha));
    last_alpha = alpha;
  }
}

::Eigen::Matrix<double, 2, 1> DistanceSpline::DDXY(double distance) const {
  const double alpha = DistanceToAlpha(distance);
  const ::Eigen::Matrix<double, 2, 1> dspline_point = spline_.DPoint(alpha);
  const ::Eigen::Matrix<double, 2, 1> ddspline_point = spline_.DDPoint(alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddspline_point / squared_norm -
         dspline_point * (dspline_point(0) * ddspline_point(0) +
                          dspline_point(1) * ddspline_point(1)) /
             ::std::pow(squared_norm, 2);
}

double DistanceSpline::DDTheta(double distance) const {
  const double alpha = DistanceToAlpha(distance);

  // TODO(austin): We are re-computing DPoint here even worse
  const ::Eigen::Matrix<double, 2, 1> dspline_point = spline_.DPoint(alpha);
  const ::Eigen::Matrix<double, 2, 1> ddspline_point = spline_.DDPoint(alpha);

  const double dtheta = spline_.DTheta(alpha);
  const double ddtheta = spline_.DDTheta(alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddtheta / squared_norm -
         dtheta * (dspline_point(0) * ddspline_point(0) +
                   dspline_point(1) * ddspline_point(1)) /
             ::std::pow(squared_norm, 2);
}

double DistanceSpline::DistanceToAlpha(double distance) const {
  if (distance <= 0.0) {
    return 0.0;
  }
  if (distance >= length()) {
    return 1.0;
  }

  // Find the distance right below our number using a binary search.
  size_t after = ::std::distance(
      distances_.begin(),
      ::std::lower_bound(distances_.begin(), distances_.end(), distance));
  size_t before = after - 1;
  const double distance_step_size =
      (1.0 / static_cast<double>(distances_.size() - 1));

  return (distance - distances_[before]) /
             (distances_[after] - distances_[before]) * distance_step_size +
         static_cast<double>(before) * distance_step_size;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
