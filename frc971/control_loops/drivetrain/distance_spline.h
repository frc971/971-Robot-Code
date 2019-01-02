#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_

#include <vector>

#include "Eigen/Dense"
#include "frc971/control_loops/drivetrain/spline.h"
#include "frc971/control_loops/fixed_quadrature.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// Class to hold a spline as a function of distance.
class DistanceSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DistanceSpline(const Spline &spline, int num_alpha = 100);

  // Returns a point on the spline as a function of distance.
  ::Eigen::Matrix<double, 2, 1> XY(double distance) const {
    return spline_.Point(DistanceToAlpha(distance));
  }

  // Returns the velocity as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DXY(double distance) const {
    return spline_.DPoint(DistanceToAlpha(distance)).normalized();
  }

  // Returns the acceleration as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DDXY(double distance) const;

  // Returns the heading as a function of distance.
  double Theta(double distance) const {
    return spline_.Theta(DistanceToAlpha(distance));
  }

  // Returns the angular velocity as a function of distance.
  double DTheta(double distance) const {
    // TODO(austin): We are re-computing DPoint here!
    const double alpha = DistanceToAlpha(distance);
    return spline_.DTheta(alpha) / spline_.DPoint(alpha).norm();
  }

  // Returns the angular acceleration as a function of distance.
  double DDTheta(double distance) const;

  // Returns the length of the path in meters.
  double length() const { return distances_.back(); }

 private:
  // Computes alpha for a distance
  double DistanceToAlpha(double distance) const;

  // The spline we are converting to a distance.
  const Spline spline_;
  // An interpolation table of distances evenly distributed in alpha.
  ::std::vector<double> distances_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
