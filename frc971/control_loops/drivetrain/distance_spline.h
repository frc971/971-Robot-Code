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

  DistanceSpline(const Spline &spline, int num_alpha = 0);
  DistanceSpline(::std::vector<Spline> &&splines, int num_alpha = 0);

  // Returns a point on the spline as a function of distance.
  ::Eigen::Matrix<double, 2, 1> XY(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines_[a.index].Point(a.alpha);
  }

  // Returns the velocity as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DXY(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines_[a.index].DPoint(a.alpha).normalized();
  }

  // Returns the acceleration as a function of distance.
  ::Eigen::Matrix<double, 2, 1> DDXY(double distance) const;

  // Returns the heading as a function of distance.
  double Theta(double distance) const {
    const AlphaAndIndex a = DistanceToAlpha(distance);
    return splines_[a.index].Theta(a.alpha);
  }

  // Returns the angular velocity as a function of distance.
  double DTheta(double distance) const {
    // TODO(austin): We are re-computing DPoint here!
    const AlphaAndIndex a = DistanceToAlpha(distance);
    const Spline &spline = splines_[a.index];
    return spline.DTheta(a.alpha) / spline.DPoint(a.alpha).norm();
  }

  double DThetaDt(double distance, double velocity) const {
    return DTheta(distance) * velocity;
  }

  // Returns the angular acceleration as a function of distance.
  double DDTheta(double distance) const;

  // Returns the length of the path in meters.
  double length() const { return distances_.back(); }

 private:
  struct AlphaAndIndex {
    size_t index;
    double alpha;
  };

  // Computes alpha for a distance
  AlphaAndIndex DistanceToAlpha(double distance) const;

  ::std::vector<double> BuildDistances(size_t num_alpha);

  // The spline we are converting to a distance.
  const ::std::vector<Spline> splines_;
  // An interpolation table of distances evenly distributed in alpha.
  const ::std::vector<double> distances_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DISTANCE_SPLINE_H_
