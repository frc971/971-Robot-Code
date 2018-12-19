#include "frc971/control_loops/drivetrain/spline.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// TODO(austin): We are re-computing dpoints a lot here.  Figure out how to pass
// them in.
double Spline::Theta(double alpha) const {
  const ::Eigen::Matrix<double, 2, 1> dp = DPoint(alpha);
  return ::std::atan2(dp(1), dp(0));
}

double Spline::DTheta(double alpha) const {
  const ::Eigen::Matrix<double, 2, 1> dp = DPoint(alpha);
  const ::Eigen::Matrix<double, 2, 1> ddp = DDPoint(alpha);
  const double dx = dp(0);
  const double dy = dp(1);

  const double ddx = ddp(0);
  const double ddy = ddp(1);

  return 1.0 / (::std::pow(dx, 2) + ::std::pow(dy, 2)) * (dx * ddy - dy * ddx);
}

double Spline::DDTheta(double alpha) const {
  const ::Eigen::Matrix<double, 2, 1> dp = DPoint(alpha);
  const ::Eigen::Matrix<double, 2, 1> ddp = DDPoint(alpha);
  const ::Eigen::Matrix<double, 2, 1> dddp = DDDPoint(alpha);
  const double dx = dp(0);
  const double dy = dp(1);

  const double ddx = ddp(0);
  const double ddy = ddp(1);

  const double dddx = dddp(0);
  const double dddy = dddp(1);

  const double magdxy2 = ::std::pow(dx, 2) + ::std::pow(dy, 2);

  return -1.0 / (::std::pow(magdxy2, 2)) * (dx * ddy - dy * ddx) * 2.0 *
             (dy * ddy + dx * ddx) +
         1.0 / magdxy2 * (dx * dddy - dy * dddx);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
