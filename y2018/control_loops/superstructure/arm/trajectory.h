#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_

#include "Eigen/Dense"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

// This class represents a path in theta0, theta1 space.  It also returns the
// position, velocity and acceleration of the path as a function of path
// distance.
class Path {
 public:
  // Constructs a path from an initializer list of (theta0, theta1, omega0,
  // omega1, alpha0, alpha1)
  Path(::std::initializer_list<::std::array<double, 6>> list);

  // Returns the length of the path in radians.
  double length() const { return distances_.back(); }

  // Returns theta, omega, and alpha as a function of path distance.  Distances
  // before the beginning of the path or after the end of the path will get
  // truncated to the nearest end.
  ::Eigen::Matrix<double, 2, 1> Theta(double distance) const {
    return Interpolate(thetas_, distance);
  }
  ::Eigen::Matrix<double, 2, 1> Omega(double distance) const {
    return Interpolate(omegas_, distance);
  }
  ::Eigen::Matrix<double, 2, 1> Alpha(double distance) const {
    return Interpolate(alphas_, distance);
  }

 private:
  // Interpolates the function represented by the list of points at the provided
  // distance.  Distances outside the range will get truncated.  This is a
  // log(n) algorithm so we can run it live on the bot.
  ::Eigen::Matrix<double, 2, 1> Interpolate(
      const ::std::vector<::Eigen::Matrix<double, 2, 1>> &points,
      double distance) const {
    if (distance <= 0.0) {
      return points[0];
    }
    if (distance >= length()) {
      return points.back();
    }
    // Keep a max and min, and pull them in towards eachother until they bound
    // our point exactly.
    size_t before = ::std::distance(
        distances_.begin(),
        ::std::lower_bound(distances_.begin(), distances_.end(), distance));
    size_t after = before + 1;
    return (distance - distances_[before]) * (points[after] - points[before]) /
               (distances_[after] - distances_[before]) +
           points[before];
  }

  // The list of theta points in the path.
  ::std::vector<::Eigen::Matrix<double, 2, 1>> thetas_;
  // The list of omega points in the path.
  ::std::vector<::Eigen::Matrix<double, 2, 1>> omegas_;
  // The list of alpha points in the path.
  ::std::vector<::Eigen::Matrix<double, 2, 1>> alphas_;

  // The distance along the path of each point.
  ::std::vector<double> distances_;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
