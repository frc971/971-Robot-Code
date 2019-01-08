#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_H_

#include <chrono>

#include "Eigen/Dense"
#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/hybrid_state_feedback_loop.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

template <typename F>
double IntegrateAccelForDistance(const F &fn, double v, double x, double dx) {
  // Use a trick from
  // https://www.johndcook.com/blog/2012/02/21/care-and-treatment-of-singularities/
  const double a0 = fn(x, v);

  return (RungeKutta(
              [&fn, &a0](double t, double y) {
                // Since we know that a0 == a(0) and that they are asymtotically
                // the same at 0, we know that the limit is 0 at 0.  This is
                // true because when starting from a stop, under sane
                // accelerations, we can assume that we will start with a
                // constant acceleration.  So, hard-code it.
                if (::std::abs(y) < 1e-6) {
                  return 0.0;
                }
                return (fn(t, y) - a0) / y;
              },
              v, x, dx) -
          v) +
         ::std::sqrt(2.0 * a0 * dx + v * v);
}

// Class to plan and hold the velocity plan for a spline.
class Trajectory {
 public:
  Trajectory(const DistanceSpline *spline,
             const DrivetrainConfig<double> &config,
             double vmax = 10.0, int num_distance = 500);
  // Sets the plan longitudal acceleration limit
  void set_longitudal_acceleration(double longitudal_acceleration) {
    longitudal_acceleration_ = longitudal_acceleration;
  }
  // Sets the plan lateral acceleration limit
  void set_lateral_acceleration(double lateral_acceleration) {
    lateral_acceleration_ = lateral_acceleration;
  }
  // Sets the voltage limit
  void set_voltage_limit(double voltage_limit) {
    voltage_limit_ = voltage_limit;
  }

  // Returns the velocity limit for a defined distance.
  double LateralVelocityCurvature(double distance) const {
    return ::std::sqrt(lateral_acceleration_ / spline_->DDXY(distance).norm());
  }

  // Runs the lateral acceleration (curvature) pass on the plan.
  void LateralAccelPass();

  // Returns the forward acceleration for a distance along the spline taking
  // into account the lateral acceleration, longitudinal acceleration, and
  // voltage limits.
  double ForwardAcceleration(const double x, const double v);

  // Runs the forwards pass, setting the starting velocity to 0 m/s
  void ForwardPass();

  // Returns the backwards acceleration for a distance along the spline taking
  // into account the lateral acceleration, longitudinal acceleration, and
  // voltage limits.
  double BackwardAcceleration(double x, double v);

  // Runs the forwards pass, setting the ending velocity to 0 m/s
  void BackwardPass();

  // Runs all the planning passes.
  void Plan() {
    LateralAccelPass();
    ForwardPass();
    BackwardPass();
  }

  // Returns the feed forwards position, velocity, acceleration for an explicit
  // distance.
  ::Eigen::Matrix<double, 3, 1> FFAcceleration(double distance);

  // Returns the feed forwards voltage for an explicit distance.
  ::Eigen::Matrix<double, 2, 1> FFVoltage(double distance);

  // Returns the length of the path in meters.
  double length() const { return spline_->length(); }

  // Returns a list of the distances.  Mostly useful for plotting.
  const ::std::vector<double> Distances() const;
  // Returns the distance for an index in the plan.
  double Distance(int index) const {
    return static_cast<double>(index) * length() /
           static_cast<double>(plan_.size() - 1);
  }

  // Returns the plan.  This is the pathwise velocity as a function of distance.
  // To get the distance for an index, use the Distance(index) function provided
  // with the index.
  const ::std::vector<double> plan() const { return plan_; }

  // Returns the left, right to linear, angular transformation matrix.
  const ::Eigen::Matrix<double, 2, 2> &Tlr_to_la() const { return Tlr_to_la_; }
  // Returns the linear, angular to left, right transformation matrix.
  const ::Eigen::Matrix<double, 2, 2> &Tla_to_lr() const { return Tla_to_lr_; }

  // Returns the goal state as a function of path distance, velocity.
  const ::Eigen::Matrix<double, 5, 1> GoalState(double distance,
                                                double velocity);

  // Returns the velocity drivetrain in use.
  const StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                          HybridKalman<2, 2, 2>>
      &velocity_drivetrain() const {
    return *velocity_drivetrain_;
  }

  // Returns the continuous statespace A and B matricies for [x, y, theta, vl,
  // vr] for the linearized system (around the provided state).
  ::Eigen::Matrix<double, 5, 5> ALinearizedContinuous(
      const ::Eigen::Matrix<double, 5, 1> &state) const;
  ::Eigen::Matrix<double, 5, 2> BLinearizedContinuous() const;

  // Returns the discrete time A and B matricies for the provided state,
  // assuming the provided timestep.
  void AB(const ::Eigen::Matrix<double, 5, 1> &state,
          ::std::chrono::nanoseconds dt, ::Eigen::Matrix<double, 5, 5> *A,
          ::Eigen::Matrix<double, 5, 2> *B) const;

  // Returns the lqr controller for the current state, timestep, and Q and R
  // gains.
  // TODO(austin): This feels like it should live somewhere else, but I'm not
  // sure where.  So, throw it here...
  ::Eigen::Matrix<double, 2, 5> KForState(
      const ::Eigen::Matrix<double, 5, 1> &state, ::std::chrono::nanoseconds dt,
      const ::Eigen::DiagonalMatrix<double, 5> &Q,
      const ::Eigen::DiagonalMatrix<double, 2> &R) const;

  ::std::vector<::Eigen::Matrix<double, 3, 1>> PlanXVA(
      ::std::chrono::nanoseconds dt);

 private:
  // Computes alpha for a distance.
  double DistanceToAlpha(double distance) const;

  // Returns K1 and K2.
  // K2 * d^x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
  const ::Eigen::Matrix<double, 2, 1> K1(double current_ddtheta) const {
    return (::Eigen::Matrix<double, 2, 1>()
                << -robot_radius_l_ * current_ddtheta,
            robot_radius_r_ * current_ddtheta)
        .finished();
  }

  const ::Eigen::Matrix<double, 2, 1> K2(double current_dtheta) const {
    return (::Eigen::Matrix<double, 2, 1>()
                << 1.0 - robot_radius_l_ * current_dtheta,
            1.0 + robot_radius_r_ * current_dtheta)
        .finished();
  }

  // Computes K3, K4, and K5 for the provided distance.
  // K5 a + K3 v^2 + K4 v = U
  void K345(const double x, ::Eigen::Matrix<double, 2, 1> *K3,
            ::Eigen::Matrix<double, 2, 1> *K4,
            ::Eigen::Matrix<double, 2, 1> *K5) {
    const double current_ddtheta = spline_->DDTheta(x);
    const double current_dtheta = spline_->DTheta(x);
    // We've now got the equation:
    //     K2 * d^x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
    const ::Eigen::Matrix<double, 2, 1> my_K2 = K2(current_dtheta);

    const ::Eigen::Matrix<double, 2, 2> B_inverse =
        velocity_drivetrain_->plant().coefficients().B_continuous.inverse();

    // Now, rephrase it as K5 a + K3 v^2 + K4 v = U
    *K3 = B_inverse * K1(current_ddtheta);
    *K4 = -B_inverse *
          velocity_drivetrain_->plant().coefficients().A_continuous * my_K2;
    *K5 = B_inverse * my_K2;
  }

  // The spline we are planning for.
  const DistanceSpline *spline_;
  // The drivetrain we are controlling.
  ::std::unique_ptr<
      StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>>
      velocity_drivetrain_;

  // Left and right robot radiuses.
  const double robot_radius_l_;
  const double robot_radius_r_;
  // Acceleration limits.
  double longitudal_acceleration_;
  double lateral_acceleration_;
  // Transformation matrix from left, right to linear, angular
  const ::Eigen::Matrix<double, 2, 2> Tlr_to_la_;
  // Transformation matrix from linear, angular to left, right
  const ::Eigen::Matrix<double, 2, 2> Tla_to_lr_;
  // Velocities in the plan (distance for each index is defined by distance())
  ::std::vector<double> plan_;
  // Plan voltage limit.
  double voltage_limit_ = 12.0;
};

// Returns the continuous time dynamics given the [x, y, theta, vl, vr] state
// and the [vl, vr] input voltage.
inline ::Eigen::Matrix<double, 5, 1> ContinuousDynamics(
    const StateFeedbackHybridPlant<2, 2, 2> &velocity_drivetrain,
    const ::Eigen::Matrix<double, 2, 2> &Tlr_to_la,
    const ::Eigen::Matrix<double, 5, 1> X,
    const ::Eigen::Matrix<double, 2, 1> U) {
  const auto &velocity = X.block<2, 1>(3, 0);
  const double theta = X(2);
  ::Eigen::Matrix<double, 2, 1> la = Tlr_to_la * velocity;
  return (::Eigen::Matrix<double, 5, 1>() << ::std::cos(theta) * la(0),
          ::std::sin(theta) * la(0), la(1),
          (velocity_drivetrain.coefficients().A_continuous * velocity +
           velocity_drivetrain.coefficients().B_continuous * U))
      .finished();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_H_
