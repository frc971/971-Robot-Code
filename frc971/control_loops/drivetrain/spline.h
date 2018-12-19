#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINE_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINE_H_

#include "Eigen/Dense"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// Class to hold a spline as a function of alpha.  Alpha can only range between
// 0.0 and 1.0.
// TODO(austin): Need to be able to represent splines which have more than 2
// control points at some point.  Or splines chained together.  This is close
// enough for now.
class Spline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructs a spline.  control_points is a matrix of start, control1,
  // control2, end.
  Spline(::Eigen::Matrix<double, 2, 4> control_points)
      : control_points_(control_points) {}

  // Returns the xy coordiate of the spline for a given alpha.
  ::Eigen::Matrix<double, 2, 1> Point(double alpha) const {
    return control_points_ *
           (::Eigen::Matrix<double, 4, 1>() << ::std::pow((1.0 - alpha), 3),
            3.0 * ::std::pow((1.0 - alpha), 2) * alpha,
            3.0 * (1.0 - alpha) * ::std::pow(alpha, 2), ::std::pow(alpha, 3))
               .finished();
  }

  // Returns the dspline/dalpha for a given alpha.
  ::Eigen::Matrix<double, 2, 1> DPoint(double alpha) const {
    return control_points_ *
           (::Eigen::Matrix<double, 4, 1>()
                << -3.0 * ::std::pow((1.0 - alpha), 2),
            3.0 * ::std::pow((1.0 - alpha), 2) +
                -2.0 * 3.0 * (1.0 - alpha) * alpha,
            -3.0 * ::std::pow(alpha, 2) + 2.0 * 3.0 * (1.0 - alpha) * alpha,
            3.0 * ::std::pow(alpha, 2))
               .finished();
  }

  // Returns the d^2spline/dalpha^2 for a given alpha.
  ::Eigen::Matrix<double, 2, 1> DDPoint(double alpha) const {
    return control_points_ *
           (::Eigen::Matrix<double, 4, 1>() << 2.0 * 3.0 * (1.0 - alpha),
            -2.0 * 3.0 * (1.0 - alpha) + -2.0 * 3.0 * (1.0 - alpha) +
                2.0 * 3.0 * alpha,
            -2.0 * 3.0 * alpha + 2.0 * 3.0 * (1.0 - alpha) - 2.0 * 3.0 * alpha,
            2.0 * 3.0 * alpha)
               .finished();
  }

  // Returns the d^3spline/dalpha^3 for a given alpha.
  ::Eigen::Matrix<double, 2, 1> DDDPoint(double /*alpha*/) const {
    return control_points_ *
           (::Eigen::Matrix<double, 4, 1>() << -2.0 * 3.0,
            2.0 * 3.0 + 2.0 * 3.0 + 2.0 * 3.0,
            -2.0 * 3.0 - 2.0 * 3.0 - 2.0 * 3.0, 2.0 * 3.0)
               .finished();
  }

  // Returns theta for a given alpha.
  double Theta(double alpha) const;

  // Returns dtheta/dalpha for a given alpha.
  double DTheta(double alpha) const;

  // Returns d^2 theta/dalpha^2 for a given alpha.
  double DDTheta(double alpha) const;

 private:
  ::Eigen::Matrix<double, 2, 4> control_points_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINE_H_
