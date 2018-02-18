#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_

#include "Eigen/Dense"

#include "y2018/control_loops/superstructure/arm/dynamics.h"

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
  // TODO(austin): I'd like to not have 2 constructors, but it looks like C++
  // doesn't want that.
  Path(::std::vector<::std::array<double, 6>> list);

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

  const ::std::vector<double> &distances() const { return distances_; }

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
    size_t after = ::std::distance(
        distances_.begin(),
        ::std::lower_bound(distances_.begin(), distances_.end(), distance));
    size_t before = after - 1;
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

namespace testing {
class TrajectoryTest_IndicesForDistanceTest_Test;
}  // namespace testing

// A trajectory is a path and a set of velocities as a function of distance.
class Trajectory {
 public:
  // Constructs a trajectory (but doesn't calculate it) given a path and a step
  // size.
  Trajectory(Path *path, double gridsize)
      : path_(path),
        num_plan_points_(
            static_cast<size_t>(::std::ceil(path_->length() / gridsize) + 1)),
        step_size_(path_->length() /
                   static_cast<double>(num_plan_points_ - 1)) {}

  // Optimizes the trajectory.  The path will adhere to the constraints that
  // || angular acceleration * alpha_unitizer || < 1, and the applied voltage <
  // vmax.
  void OptimizeTrajectory(const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer,
                          double vmax) {
    max_dvelocity_unfiltered_ = CurvatureOptimizationPass(alpha_unitizer, vmax);

    // We need to start and end the trajectory with 0 velocity.
    max_dvelocity_unfiltered_[0] = 0.0;
    max_dvelocity_unfiltered_[max_dvelocity_unfiltered_.size() - 1] = 0.0;

    max_dvelocity_ = BackwardsOptimizationPass(max_dvelocity_unfiltered_,
                                               alpha_unitizer, vmax);
    max_dvelocity_forward_pass_ = ForwardsOptimizationPass(
        max_dvelocity_, alpha_unitizer, vmax);
  }

  // Returns an array of the distances used in the plan.  The starting point
  // (0.0), and end point (path->length()) are included in the array.
  ::std::vector<double> DistanceArray() const {
    ::std::vector<double> result;
    result.reserve(num_plan_points_);
    for (size_t i = 0; i < num_plan_points_; ++i) {
      result.push_back(DistanceForIndex(i));
    }
    return result;
  }

  // Computes the maximum velocity that we can follow the path while adhering to
  // the constraints that || angular acceleration * alpha_unitizer || < 1, and
  // the applied voltage < vmax.  Returns the velocities.
  ::std::vector<double> CurvatureOptimizationPass(
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double vmax);

  // Computes the maximum forwards feasable acceleration at a given position
  // while adhering to the vmax and alpha_unitizer constraints.
  // This gives us the maximum path distance acceleration (d^2d/dt^2) for any
  // initial position and velocity for the forwards path.
  double FeasableForwardsAcceleration(
      double goal_distance, double goal_velocity, double vmax,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer);

  // Computes the maximum backwards feasable acceleration at a given position
  // while adhering to the vmax and alpha_unitizer constraints.
  // This gives us the maximum path distance acceleration (d^2d/dt^2) for any
  // initial position and velocity for the backwards path.
  // Note: positive acceleration means speed up while going in the negative
  // direction on the path.
  double FeasableBackwardsAcceleration(
      double goal_distance, double goal_velocity, double vmax,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer);

  // Executes the backwards path optimization pass.
  ::std::vector<double> BackwardsOptimizationPass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double vmax);

  // Executes the forwards path optimization pass.
  ::std::vector<double> ForwardsOptimizationPass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double vmax);

  // Returns the number of points used for the plan.
  size_t num_plan_points() const { return num_plan_points_; }

  // Returns the curvature only velocity plan.
  const ::std::vector<double> &max_dvelocity_unfiltered() const {
    return max_dvelocity_unfiltered_;
  }
  // Returns the backwards pass + curvature plan.
  const ::std::vector<double> &max_dvelocity() const { return max_dvelocity_; }
  // Returns the full plan.  This isn't useful at runtime since we don't want to
  // be tied to a specific acceleration plan when we hit saturation.
  const ::std::vector<double> &max_dvelocity_forward_pass() const {
    return max_dvelocity_forward_pass_;
  }

 private:
  friend class testing::TrajectoryTest_IndicesForDistanceTest_Test;

  // Returns the distance along the path for a specific index in the plan.
  double DistanceForIndex(size_t index) const {
    return static_cast<double>(index) * step_size_;
  }

  // Returns the before and after indices for a specific distance in the plan.
  ::std::pair<size_t, size_t> IndicesForDistance(double distance) const {
    const double path_length = path_->length();
    if (distance <= 0.0) {
      return ::std::pair<size_t, size_t>(0, 1);
    }
    const size_t lower_index =
        ::std::min(static_cast<size_t>(num_plan_points_ - 2),
                   static_cast<size_t>(::std::floor((num_plan_points_ - 1) *
                                                    distance / path_length)));

    return ::std::pair<size_t, size_t>(lower_index, lower_index + 1);
  }

  // The path to follow.
  const Path *path_ = nullptr;
  // The number of points in the plan.
  const size_t num_plan_points_;
  // A cached version of the step size since we need this a *lot*.
  const double step_size_;

  ::std::vector<double> max_dvelocity_unfiltered_;
  ::std::vector<double> max_dvelocity_;
  ::std::vector<double> max_dvelocity_forward_pass_;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
