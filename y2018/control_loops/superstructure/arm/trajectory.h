#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_

#include <array>
#include <initializer_list>
#include <memory>
#include <vector>

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
  // TODO(austin): I'd like to not have 2 constructors, but it looks like C++
  // doesn't want that.
  Path(::std::vector<::std::array<double, 6>> list);

  static ::std::unique_ptr<Path> Reversed(::std::unique_ptr<Path> p);
  static Path Reversed(const Path &p);

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
  Trajectory(::std::unique_ptr<const Path> path, double gridsize)
      : path_(::std::move(path)),
        num_plan_points_(
            static_cast<size_t>(::std::ceil(path_->length() / gridsize) + 1)),
        step_size_(path_->length() /
                   static_cast<double>(num_plan_points_ - 1)) {
    alpha_unitizer_.setZero();
  }

  // Optimizes the trajectory.  The path will adhere to the constraints that
  // || angular acceleration * alpha_unitizer || < 1, and the applied voltage <
  // plan_vmax.
  void OptimizeTrajectory(const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer,
                          double plan_vmax) {
    if (path_ == nullptr) {
      abort();
    }
    alpha_unitizer_ = alpha_unitizer;

    max_dvelocity_unfiltered_ =
        CurvatureOptimizationPass(alpha_unitizer, plan_vmax);

    // We need to start and end the trajectory with 0 velocity.
    max_dvelocity_unfiltered_[0] = 0.0;
    max_dvelocity_unfiltered_[max_dvelocity_unfiltered_.size() - 1] = 0.0;

    max_dvelocity_ = BackwardsOptimizationPass(max_dvelocity_unfiltered_,
                                               alpha_unitizer, plan_vmax);
    max_dvelocity_forward_pass_ =
        ForwardsOptimizationPass(max_dvelocity_, alpha_unitizer, plan_vmax);
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
  // the applied voltage < plan_vmax.  Returns the velocities.
  ::std::vector<double> CurvatureOptimizationPass(
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double plan_vmax);

  // Computes the maximum forwards feasable acceleration at a given position
  // while adhering to the plan_vmax and alpha_unitizer constraints.
  // This gives us the maximum path distance acceleration (d^2d/dt^2) for any
  // initial position and velocity for the forwards path.
  double FeasableForwardsAcceleration(
      double goal_distance, double goal_velocity, double plan_vmax,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer) const;

  // Computes the maximum backwards feasable acceleration at a given position
  // while adhering to the plan_vmax and alpha_unitizer constraints.
  // This gives us the maximum path distance acceleration (d^2d/dt^2) for any
  // initial position and velocity for the backwards path.
  // Note: positive acceleration means speed up while going in the negative
  // direction on the path.
  double FeasableBackwardsAcceleration(
      double goal_distance, double goal_velocity, double plan_vmax,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer) const;

  // Executes the backwards path optimization pass.
  ::std::vector<double> BackwardsOptimizationPass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double plan_vmax);

  // Executes the forwards path optimization pass.
  ::std::vector<double> ForwardsOptimizationPass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer, double plan_vmax);

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

  // Returns the interpolated velocity at the distance d along the path.  The
  // math assumes constant acceleration from point (d0, v0), to point (d1, v1),
  // and that we are at point d between the two.
  static double InterpolateVelocity(double d, double d0, double d1, double v0,
                                    double v1) {
    // We don't support negative velocities.  Report 0 velocity in that case.
    // TODO(austin): Verify this doesn't show up in the real world.
    if (v0 < 0 || v1 < 0) {
      return 0.0;
    }
    if (d <= d0) {
      return v0;
    }
    if (d >= d1) {
      return v1;
    }
    return ::std::sqrt(v0 * v0 + (v1 * v1 - v0 * v0) * (d - d0) / (d1 - d0));
  }

  // Computes the path distance velocity of the plan as a function of the
  // distance.
  double GetDVelocity(double distance) const {
    return GetDVelocity(distance, max_dvelocity_);
  }
  double GetDVelocity(double d, const ::std::vector<double> &plan) const {
    ::std::pair<size_t, size_t> indices = IndicesForDistance(d);
    const double v0 = plan[indices.first];
    const double v1 = plan[indices.second];
    const double d0 = DistanceForIndex(indices.first);
    const double d1 = DistanceForIndex(indices.second);
    return InterpolateVelocity(d, d0, d1, v0, v1);
  }

  // Computes the path distance acceleration of the plan as a function of the
  // distance.
  double GetDAcceleration(double distance) const {
    return GetDAcceleration(distance, max_dvelocity_);
  }
  double GetDAcceleration(double distance, const ::std::vector<double> &plan) const {
    ::std::pair<size_t, size_t> indices = IndicesForDistance(distance);
    const double v0 = plan[indices.first];
    const double v1 = plan[indices.second];
    const double d0 = DistanceForIndex(indices.first);
    const double d1 = DistanceForIndex(indices.second);
    return InterpolateAcceleration(d0, d1, v0, v1);
  }

  // Returns the acceleration along the path segment assuming constant
  // acceleration.
  static double InterpolateAcceleration(double d0, double d1, double v0,
                                        double v1) {
    return 0.5 * (::std::pow(v1, 2) - ::std::pow(v0, 2)) / (d1 - d0);
  }

  ::Eigen::Matrix<double, 2, 1> ThetaT(double d) const {
    return path_->Theta(d);
  }

  // Returns d theta/dt at a specified path distance and velocity.
  ::Eigen::Matrix<double, 2, 1> OmegaT(double distance, double velocity) const {
    if (distance > path_->length() || distance < 0.0) {
      return ::Eigen::Matrix<double, 2, 1>::Zero();
    } else {
      return path_->Omega(distance) * velocity;
    }
  }

  // Returns d^2 theta/dt^2 at a specified path distance, velocity and
  // acceleration.
  ::Eigen::Matrix<double, 2, 1> AlphaT(double distance, double velocity,
                                       double acceleration) const {
    if (distance > path_->length() || distance < 0.0) {
      return ::Eigen::Matrix<double, 2, 1>::Zero();
    } else {
      return path_->Alpha(distance) * ::std::pow(velocity, 2) +
             path_->Omega(distance) * acceleration;
    }
  }

  // Converts a theta and omega vector to a full state vector.
  static ::Eigen::Matrix<double, 6, 1> R(
      ::Eigen::Matrix<double, 2, 1> theta_t,
      ::Eigen::Matrix<double, 2, 1> omega_t) {
    return (::Eigen::Matrix<double, 6, 1>() << theta_t(0, 0), omega_t(0, 0),
            theta_t(1, 0), omega_t(1, 0), 0.0, 0.0)
        .finished();
  }

  const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer() const {
    return alpha_unitizer_;
  }

  const Path &path() const { return *path_; }

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
  ::std::unique_ptr<const Path> path_;
  // The number of points in the plan.
  const size_t num_plan_points_;
  // A cached version of the step size since we need this a *lot*.
  const double step_size_;

  ::std::vector<double> max_dvelocity_unfiltered_;
  ::std::vector<double> max_dvelocity_;
  ::std::vector<double> max_dvelocity_forward_pass_;

  ::Eigen::Matrix<double, 2, 2> alpha_unitizer_;
};

// This class tracks the current goal along trajectories and paths.
class TrajectoryFollower {
 public:
  TrajectoryFollower(const ::Eigen::Matrix<double, 2, 1> &theta)
      : trajectory_(nullptr), theta_(theta) {
    omega_.setZero();
    last_K_.setZero();
    Reset();
  }

  TrajectoryFollower(Trajectory *const trajectory) : trajectory_(trajectory) {
    last_K_.setZero();
    Reset();
  }

  bool has_path() const { return trajectory_ != nullptr; }

  void set_theta(const ::Eigen::Matrix<double, 2, 1> &theta) { theta_ = theta; }

  // Returns the goal distance along the path.
  const ::Eigen::Matrix<double, 2, 1> &goal() const { return goal_; }
  double goal(int i) const { return goal_(i); }

  // Starts over at the beginning of the path.
  void Reset();

  // Switches paths and starts at the beginning of the path.
  void SwitchTrajectory(const Trajectory *trajectory) {
    trajectory_ = trajectory;
    Reset();
  }

  // Returns the controller gain at the provided state.
  ::Eigen::Matrix<double, 2, 6> K_at_state(
      const ::Eigen::Matrix<double, 6, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U);

  // Returns the voltage, velocity and acceleration if we were to be partially
  // along the path.
  void USaturationSearch(double goal_distance, double last_goal_distance,
                         double goal_velocity, double last_goal_velocity,
                         double saturation_fraction_along_path,
                         const ::Eigen::Matrix<double, 2, 6> &K,
                         const ::Eigen::Matrix<double, 6, 1> &X,
                         const Trajectory &trajectory,
                         ::Eigen::Matrix<double, 2, 1> *U,
                         double *saturation_goal_velocity,
                         double *saturation_goal_acceleration);

  // Returns the next goal given a planning plan_vmax and timestep.  This
  // ignores the
  // backwards pass.
  ::Eigen::Matrix<double, 2, 1> PlanNextGoal(
      const ::Eigen::Matrix<double, 2, 1> &goal, double plan_vmax, double dt);

  // Plans the next cycle and updates the internal state for consumption.
  void Update(const ::Eigen::Matrix<double, 6, 1> &X, bool disabled, double dt,
              double plan_vmax, double voltage_limit);

  // Returns the goal acceleration for this cycle.
  double goal_acceleration() const { return goal_acceleration_; }

  // Returns U(s) for this cycle.
  const ::Eigen::Matrix<double, 2, 1> &U() const { return U_; }
  double U(int i) const { return U_(i); }
  const ::Eigen::Matrix<double, 2, 1> &U_unsaturated() const {
    return U_unsaturated_;
  }
  const ::Eigen::Matrix<double, 2, 1> &U_ff() const { return U_ff_; }

  double saturation_fraction_along_path() const {
    return saturation_fraction_along_path_;
  }

  const ::Eigen::Matrix<double, 2, 1> &theta() const { return theta_; }
  double theta(int i) const { return theta_(i); }
  const ::Eigen::Matrix<double, 2, 1> &omega() const { return omega_; }
  double omega(int i) const { return omega_(i); }

  // Distance left on the path before we get to the end of the path.
  double path_distance_to_go() const {
    if (has_path()) {
      return ::std::max(0.0, path()->length() - goal_(0));
    } else {
      return 0.0;
    }
  }

  const Path *path() const { return &trajectory_->path(); }

  int failed_solutions() const { return failed_solutions_; }

 private:
  // The trajectory plan.
  const Trajectory *trajectory_ = nullptr;

  // The current goal.
  ::Eigen::Matrix<double, 2, 1> goal_;
  // The previously executed goal.
  ::Eigen::Matrix<double, 2, 1> last_goal_;
  // The goal to use next cycle.  We always plan 1 cycle ahead.
  ::Eigen::Matrix<double, 2, 1> next_goal_;

  ::Eigen::Matrix<double, 2, 1> U_;
  ::Eigen::Matrix<double, 2, 1> U_unsaturated_;
  ::Eigen::Matrix<double, 2, 1> U_ff_;
  double goal_acceleration_ = 0.0;

  double saturation_fraction_along_path_ = 1.0;

  // Holds the last valid goal position for when we loose our path.
  ::Eigen::Matrix<double, 2, 1> theta_;
  ::Eigen::Matrix<double, 2, 1> omega_;

  ::Eigen::Matrix<double, 2, 6> last_K_;
  int failed_solutions_ = 0;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
