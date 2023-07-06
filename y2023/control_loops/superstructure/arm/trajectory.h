#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_

#include <array>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include "frc971/control_loops/binomial.h"
#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/fixed_quadrature.h"
#include "frc971/control_loops/hybrid_state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2023/control_loops/superstructure/arm/arm_trajectories_generated.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {

using frc971::control_loops::Binomial;
using frc971::control_loops::GaussianQuadrature5;

template <int N, int M>
class NSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Computes the characteristic matrix of a given spline order. This is an
  // upper triangular matrix rather than lower because our splines are
  // represented as rows rather than columns.
  // Each row represents the impact of each point with increasing powers of
  // alpha. Row i, column j contains the effect point i has with the j'th power
  // of alpha.
  static ::Eigen::Matrix<double, N, N> SplineMatrix() {
    ::Eigen::Matrix<double, N, N> matrix =
        ::Eigen::Matrix<double, N, N>::Zero();

    for (int i = 0; i < N; ++i) {
      // Binomial(N - 1, i) * (1 - t) ^ (N - i - 1) * (t ^ i) * P[i]
      const double binomial = Binomial(N - 1, i);
      const int one_minus_t_power = N - i - 1;

      // Then iterate over the powers of t and add the pieces to the matrix.
      for (int j = i; j < N; ++j) {
        // j is the power of t we are placing in the matrix.
        // k is the power of t in the (1 - t) expression that we need to
        // evaluate.
        const int k = j - i;
        const double tscalar =
            binomial * Binomial(one_minus_t_power, k) * ::std::pow(-1.0, k);
        matrix(i, j) = tscalar;
      }
    }
    return matrix;
  }

  // Computes the matrix to multiply by [1, a, a^2, ...] to evaluate the spline.
  template <int D>
  static ::Eigen::Matrix<double, M, N - D> SplinePolynomial(
      const ::Eigen::Matrix<double, M, N> &control_points) {
    // We use rows for the spline, so the multiplication looks "backwards"
    ::Eigen::Matrix<double, M, N> polynomial = control_points * SplineMatrix();

    // Now, compute the derivative requested.
    for (int i = D; i < N; ++i) {
      // Start with t^i, and multiply i, i-1, i-2 until you have done this
      // Derivative times.
      double scalar = 1.0;
      for (int j = i; j > i - D; --j) {
        scalar *= j;
      }
      polynomial.template block<M, 1>(0, i) =
          polynomial.template block<M, 1>(0, i) * scalar;
    }
    return polynomial.template block<M, N - D>(0, D);
  }

  // Computes an order K-1 polynomial matrix in alpha.  [1, alpha, alpha^2, ...]
  template <int K>
  static ::Eigen::Matrix<double, K, 1> AlphaPolynomial(double alpha) {
    alpha = std::min(1.0, std::max(0.0, alpha));
    ::Eigen::Matrix<double, K, 1> polynomial =
        ::Eigen::Matrix<double, K, 1>::Zero();
    polynomial(0) = 1.0;
    for (int i = 1; i < K; ++i) {
      polynomial(i) = polynomial(i - 1) * alpha;
    }
    return polynomial;
  }

  // Constructs a spline.  control_points is a matrix of start, control1,
  // control2, ..., end.
  NSpline(::Eigen::Matrix<double, M, N> control_points)
      : control_points_(control_points),
        spline_polynomial_(SplinePolynomial<0>(control_points_)),
        dspline_polynomial_(SplinePolynomial<1>(control_points_)),
        ddspline_polynomial_(SplinePolynomial<2>(control_points_)) {}

  // Returns the xy coordiate of the spline for a given alpha.
  ::Eigen::Matrix<double, M, 1> Theta(double alpha) const {
    return spline_polynomial_ * AlphaPolynomial<N>(alpha);
  }

  // Returns the dspline/dalpha for a given alpha.
  ::Eigen::Matrix<double, M, 1> Omega(double alpha) const {
    return dspline_polynomial_ * AlphaPolynomial<N - 1>(alpha);
  }

  // Returns the d^2spline/dalpha^2 for a given alpha.
  ::Eigen::Matrix<double, M, 1> Alpha(double alpha) const {
    return ddspline_polynomial_ * AlphaPolynomial<N - 2>(alpha);
  }

  const ::Eigen::Matrix<double, M, N> &control_points() const {
    return control_points_;
  }

 private:
  const ::Eigen::Matrix<double, M, N> control_points_;

  // Each of these polynomials gets multiplied by [x^(n-1), x^(n-2), ..., x, 1]
  // depending on the size of the polynomial.
  const ::Eigen::Matrix<double, M, N> spline_polynomial_;
  const ::Eigen::Matrix<double, M, N - 1> dspline_polynomial_;
  const ::Eigen::Matrix<double, M, N - 2> ddspline_polynomial_;
};

// Add a cos wave to the bottom of the 2d spline to handle the roll.
class CosSpline {
 public:
  // Struct defining pairs of alphas and thetas.
  struct AlphaTheta {
    double alpha;
    double theta;
  };

  CosSpline(NSpline<4, 2> spline, std::vector<AlphaTheta> roll)
      : spline_(spline), roll_(std::move(roll)) {
    CHECK_GE(roll_.size(), 2u);
    CHECK_EQ(roll_[0].alpha, 0.0);
    CHECK_EQ(roll_[roll_.size() - 1].alpha, 1.0);
  }

  // Returns the xy coordiate of the spline for a given alpha.
  ::Eigen::Matrix<double, 3, 1> Theta(double alpha) const;

  // Returns the dspline/dalpha for a given alpha.
  ::Eigen::Matrix<double, 3, 1> Omega(double alpha) const;

  // Returns the d^2spline/dalpha^2 for a given alpha.
  ::Eigen::Matrix<double, 3, 1> Alpha(double alpha) const;

  const NSpline<4, 2> &spline() const { return spline_; }

  const std::vector<AlphaTheta> &roll() { return roll_; }

  CosSpline Reversed() const;

 private:
  NSpline<4, 2> spline_;
  const std::vector<AlphaTheta> roll_;

  // Returns the two control points for the roll for an alpha.
  std::pair<AlphaTheta, AlphaTheta> RollPoints(double alpha) const;
};

// Class to hold a spline as a function of distance.
class Path {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Path(const CosSpline &spline, int num_alpha = 100)
      : spline_(spline), distances_(BuildDistances(num_alpha)) {}

  Path(const CosSpline &spline, std::vector<float> distances)
      : spline_(spline), distances_(distances) {}

  virtual ~Path() {}

  // Returns a point on the spline as a function of distance.
  ::Eigen::Matrix<double, 3, 1> Theta(double distance) const;

  // Returns the velocity as a function of distance.
  ::Eigen::Matrix<double, 3, 1> Omega(double distance) const;

  // Returns the acceleration as a function of distance.
  ::Eigen::Matrix<double, 3, 1> Alpha(double distance) const;

  // Returns the length of the path in meters.
  double length() const { return distances().back(); }

  const absl::Span<const float> distances() const { return distances_; }

  const CosSpline &spline() const { return spline_; }

  Path Reversed() const;
  static ::std::unique_ptr<Path> Reversed(::std::unique_ptr<Path> p);

 private:
  std::vector<float> BuildDistances(size_t num_alpha);

  // Computes alpha for a distance
  double DistanceToAlpha(double distance) const;

  const CosSpline spline_;
  // An interpolation table of distances evenly distributed in alpha.
  const ::std::vector<float> distances_;
};

namespace testing {
class TrajectoryTest_IndicesForDistanceTest_Test;
}  // namespace testing

// A trajectory is a path and a set of velocities as a function of distance.
class Trajectory {
 public:
  // Constructs a trajectory (but doesn't calculate it) given a path and a step
  // size.
  Trajectory(const frc971::control_loops::arm::Dynamics *dynamics,
             const StateFeedbackHybridPlant<3, 1, 1> *roll,
             std::unique_ptr<const Path> path, double gridsize)
      : dynamics_(dynamics),
        roll_(roll),
        path_(::std::move(path)),
        num_plan_points_(
            static_cast<size_t>(::std::ceil(path_->length() / gridsize) + 1)),
        step_size_(path_->length() /
                   static_cast<double>(num_plan_points_ - 1)) {
    alpha_unitizer_.setZero();
  }

  Trajectory(const frc971::control_loops::arm::Dynamics *dynamics,
             const StateFeedbackHybridPlant<3, 1, 1> *roll,
             const TrajectoryFbs &trajectory_fbs);

  // Optimizes the trajectory.  The path will adhere to the constraints that
  // || angular acceleration * alpha_unitizer || < 1, and the applied voltage <
  // plan_vmax.
  void OptimizeTrajectory(const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer,
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

    max_dvelocity_backwards_accel_ = BackwardsOptimizationAccelerationPass(
        max_dvelocity_unfiltered_, alpha_unitizer, plan_vmax);
    max_dvelocity_forwards_accel_ = ForwardsOptimizationAccelerationPass(
        max_dvelocity_backwards_accel_, alpha_unitizer, plan_vmax);

    max_dvelocity_backwards_voltage_ = BackwardsOptimizationVoltagePass(
        max_dvelocity_forwards_accel_, alpha_unitizer, plan_vmax);

    max_dvelocity_forwards_voltage_ = ForwardsOptimizationVoltagePass(
        max_dvelocity_backwards_voltage_, alpha_unitizer, plan_vmax);
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
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax);

  double MaxCurvatureSpeed(double goal_distance,
                           const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer,
                           double plan_vmax);

  // Computes the maximum forwards feasable acceleration at a given position
  // while adhering to the plan_vmax and alpha_unitizer constraints.
  // This gives us the maximum path distance acceleration (d^2d/dt^2) for any
  // initial position and velocity for the forwards path.
  double FeasableForwardsAcceleration(
      double goal_distance, double goal_velocity, double plan_vmax,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const;
  double FeasableForwardsVoltage(
      double goal_distance, double goal_velocity, double plan_vmax,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const;

  // Computes the maximum backwards feasable acceleration at a given position
  // while adhering to the plan_vmax and alpha_unitizer constraints.
  // This gives us the maximum path distance acceleration (d^2d/dt^2) for any
  // initial position and velocity for the backwards path.
  // Note: positive acceleration means speed up while going in the negative
  // direction on the path.
  double FeasableBackwardsAcceleration(
      double goal_distance, double goal_velocity, double plan_vmax,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const;
  double FeasableBackwardsVoltage(
      double goal_distance, double goal_velocity, double plan_vmax,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer) const;

  // Executes the backwards path optimization pass.
  ::std::vector<double> BackwardsOptimizationAccelerationPass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax);
  ::std::vector<double> BackwardsOptimizationVoltagePass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax);

  // Executes the forwards path optimization pass.
  ::std::vector<double> ForwardsOptimizationAccelerationPass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax);
  ::std::vector<double> ForwardsOptimizationVoltagePass(
      const ::std::vector<double> &max_dvelocity,
      const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double plan_vmax);

  // Returns the number of points used for the plan.
  size_t num_plan_points() const { return num_plan_points_; }

  // Returns the curvature only velocity plan.
  const ::std::vector<double> &max_dvelocity_unfiltered() const {
    return max_dvelocity_unfiltered_;
  }
  // Returns the backwards pass + curvature plan.
  const ::std::vector<double> &max_dvelocity_backward_accel() const {
    return max_dvelocity_backwards_accel_;
  }
  const ::std::vector<double> &max_dvelocity_backward_voltage() const {
    return max_dvelocity_backwards_voltage_;
  }
  // Returns the full plan.  This isn't useful at runtime since we don't want to
  // be tied to a specific acceleration plan when we hit saturation.
  const ::std::vector<double> &max_dvelocity_forwards_accel() const {
    return max_dvelocity_forwards_accel_;
  }
  const ::std::vector<double> &max_dvelocity_forwards_voltage() const {
    return max_dvelocity_forwards_voltage_;
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
    return GetDVelocity(distance, max_dvelocity_forwards_voltage_);
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
    return GetDAcceleration(distance, max_dvelocity_forwards_voltage_);
  }
  double GetDAcceleration(double distance,
                          const ::std::vector<double> &plan) const {
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

  ::Eigen::Matrix<double, 3, 1> ThetaT(double d) const {
    return path_->Theta(d);
  }

  // Returns d theta/dt at a specified path distance and velocity.
  ::Eigen::Matrix<double, 3, 1> OmegaT(double distance, double velocity) const {
    if (distance > path_->length() || distance < 0.0) {
      return ::Eigen::Matrix<double, 3, 1>::Zero();
    } else {
      return path_->Omega(distance) * velocity;
    }
  }

  // Returns d^2 theta/dt^2 at a specified path distance, velocity and
  // acceleration.
  ::Eigen::Matrix<double, 3, 1> AlphaT(double distance, double velocity,
                                       double acceleration) const {
    if (distance > path_->length() || distance < 0.0) {
      return ::Eigen::Matrix<double, 3, 1>::Zero();
    } else {
      return path_->Alpha(distance) * ::std::pow(velocity, 2) +
             path_->Omega(distance) * acceleration;
    }
  }

  // Converts a theta and omega vector to a full state vector.
  static ::Eigen::Matrix<double, 9, 1> R(
      ::Eigen::Matrix<double, 3, 1> theta_t,
      ::Eigen::Matrix<double, 3, 1> omega_t) {
    return (::Eigen::Matrix<double, 9, 1>() << theta_t(0, 0), omega_t(0, 0),
            theta_t(1, 0), omega_t(1, 0), 0.0, 0.0, theta_t(2, 0),
            omega_t(2, 0), 0.0)
        .finished();
  }

  const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer() const {
    return alpha_unitizer_;
  }

  const Path &path() const { return *path_; }

  double step_size() const { return step_size_; }

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

  const frc971::control_loops::arm::Dynamics *dynamics_;
  const StateFeedbackHybridPlant<3, 1, 1> *roll_;

  // The path to follow.
  std::unique_ptr<const Path> path_;
  // The number of points in the plan.
  const size_t num_plan_points_;
  // A cached version of the step size since we need this a *lot*.
  const double step_size_;

  ::std::vector<double> max_dvelocity_unfiltered_;
  ::std::vector<double> max_dvelocity_backwards_voltage_;
  ::std::vector<double> max_dvelocity_backwards_accel_;
  ::std::vector<double> max_dvelocity_forwards_accel_;
  ::std::vector<double> max_dvelocity_forwards_voltage_;

  ::Eigen::Matrix<double, 3, 3> alpha_unitizer_;
};

// This class tracks the current goal along trajectories and paths.
class TrajectoryFollower {
 public:
  TrajectoryFollower(const frc971::control_loops::arm::Dynamics *dynamics,
                     const StateFeedbackLoop<3, 1, 1, double,
                                             StateFeedbackHybridPlant<3, 1, 1>,
                                             HybridKalman<3, 1, 1>> *roll,
                     const ::Eigen::Matrix<double, 3, 1> &theta)
      : dynamics_(dynamics), roll_(roll), trajectory_(nullptr), theta_(theta) {
    omega_.setZero();
    last_K_.setZero();
    Reset();
  }

  TrajectoryFollower(const frc971::control_loops::arm::Dynamics *dynamics,
                     const StateFeedbackLoop<3, 1, 1, double,
                                             StateFeedbackHybridPlant<3, 1, 1>,
                                             HybridKalman<3, 1, 1>> *roll,
                     Trajectory *const trajectory)
      : dynamics_(dynamics), roll_(roll), trajectory_(trajectory) {
    last_K_.setZero();
    Reset();
  }

  bool has_path() const { return trajectory_ != nullptr; }

  void set_theta(const ::Eigen::Matrix<double, 3, 1> &theta) { theta_ = theta; }

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
  ::Eigen::Matrix<double, 2, 6> ArmK_at_state(
      const Eigen::Ref<const ::Eigen::Matrix<double, 6, 1>> arm_X,
      const Eigen::Ref<const ::Eigen::Matrix<double, 2, 1>> arm_U);

  // Returns the voltage, velocity and acceleration if we were to be partially
  // along the path.
  void USaturationSearch(double goal_distance, double last_goal_distance,
                         double goal_velocity, double last_goal_velocity,
                         double saturation_fraction_along_path,
                         const ::Eigen::Matrix<double, 2, 6> &K,
                         const ::Eigen::Matrix<double, 9, 1> &X,
                         const Trajectory &trajectory,
                         ::Eigen::Matrix<double, 3, 1> *U,
                         double *saturation_goal_velocity,
                         double *saturation_goal_acceleration);

  // Returns the next goal given a planning plan_vmax and timestep.  This
  // ignores the backwards pass.
  ::Eigen::Matrix<double, 2, 1> PlanNextGoal(
      const ::Eigen::Matrix<double, 2, 1> &goal, double plan_vmax, double dt);

  // Plans the next cycle and updates the internal state for consumption.
  void Update(const ::Eigen::Matrix<double, 9, 1> &X, bool disabled, double dt,
              double plan_vmax, double voltage_limit);

  // Returns the goal acceleration for this cycle.
  double goal_acceleration() const { return goal_acceleration_; }

  // Returns U(s) for this cycle.
  const ::Eigen::Matrix<double, 3, 1> &U() const { return U_; }
  double U(int i) const { return U_(i); }
  const ::Eigen::Matrix<double, 3, 1> &U_unsaturated() const {
    return U_unsaturated_;
  }
  const ::Eigen::Matrix<double, 3, 1> &U_ff() const { return U_ff_; }

  double saturation_fraction_along_path() const {
    return saturation_fraction_along_path_;
  }

  const ::Eigen::Matrix<double, 3, 1> &theta() const { return theta_; }
  double theta(int i) const { return theta_(i); }
  const ::Eigen::Matrix<double, 3, 1> &omega() const { return omega_; }
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

  Eigen::Matrix<double, 3, 1> ComputeFF_U(
      const Eigen::Matrix<double, 9, 1> &X,
      const Eigen::Matrix<double, 3, 1> &omega_t,
      const Eigen::Matrix<double, 3, 1> &alpha_t) const;

 private:
  const frc971::control_loops::arm::Dynamics *dynamics_;
  const StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                          HybridKalman<3, 1, 1>> *roll_;
  // The trajectory plan.
  const Trajectory *trajectory_ = nullptr;

  // The current goal.
  ::Eigen::Matrix<double, 2, 1> goal_;
  // The previously executed goal.
  ::Eigen::Matrix<double, 2, 1> last_goal_;
  // The goal to use next cycle.  We always plan 1 cycle ahead.
  ::Eigen::Matrix<double, 2, 1> next_goal_;

  ::Eigen::Matrix<double, 3, 1> U_;
  ::Eigen::Matrix<double, 3, 1> U_unsaturated_;
  ::Eigen::Matrix<double, 3, 1> U_ff_;
  double goal_acceleration_ = 0.0;

  double saturation_fraction_along_path_ = 1.0;

  // Holds the last valid goal position for when we loose our path.
  ::Eigen::Matrix<double, 3, 1> theta_;
  ::Eigen::Matrix<double, 3, 1> omega_;

  ::Eigen::Matrix<double, 2, 6> last_K_;
  int failed_solutions_ = 0;
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_TRAJECTORY_H_
