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
             const DrivetrainConfig<double> &config, double vmax = 10.0,
             int num_distance = 0);
  // Sets the plan longitudinal acceleration limit
  void set_longitudinal_acceleration(double longitudinal_acceleration) {
    longitudinal_acceleration_ = longitudinal_acceleration;
  }
  // Sets the plan lateral acceleration limit
  void set_lateral_acceleration(double lateral_acceleration) {
    lateral_acceleration_ = lateral_acceleration;
  }
  // Sets the voltage limit
  void set_voltage_limit(double voltage_limit) {
    voltage_limit_ = voltage_limit;
  }

  // Returns the friction-constrained velocity limit at a given distance along
  // the path. At the returned velocity, one or both wheels will be on the edge
  // of slipping.
  // There are some very disorganized thoughts on the math here and in some of
  // the other functions in spline_math.tex.
  double LateralVelocityCurvature(double distance) const;

  // Returns the range of allowable longitudinal accelerations for the center of
  // the robot at a particular distance (x) along the path and velocity (v).
  // min_accel and max_accel correspodn to the min/max accelerations that can be
  // achieved without breaking friction limits on one or both wheels.
  // If max_accel < min_accel, that implies that v is too high for there to be
  // any valid acceleration. FrictionLngAccelLimits(x,
  // LateralVelocityCurvature(x), &min_accel, &max_accel) should result in
  // min_accel == max_accel.
  void FrictionLngAccelLimits(double x, double v, double *min_accel,
                              double *max_accel) const;

  enum class VoltageLimit {
    kConservative,
    kAggressive,
  };

  // Calculates the maximum voltage at which we *can* track the path. In some
  // cases there will be two ranges of feasible velocities for traversing the
  // path--in such a situation, from zero to velocity A we will be able to track
  // the path, from velocity A to B we can't, from B to C we can and above C we
  // can't. If limit_type = kConservative, we return A; if limit_type =
  // kAggressive, we return C. We currently just use the kConservative limit
  // because that way we can guarantee that all velocities between zero and A
  // are allowable and don't have to handle a more complicated planning problem.
  // constraint_voltages will be populated by the only wheel voltages that are
  // valid at the returned limit.
  double VoltageVelocityLimit(
      double distance, VoltageLimit limit_type,
      Eigen::Matrix<double, 2, 1> *constraint_voltages = nullptr) const;

  // Limits the velocity in the specified segment to the max velocity.
  void LimitVelocity(double starting_distance, double ending_distance,
                     double max_velocity);

  // Runs the lateral acceleration (curvature) pass on the plan.
  void LateralAccelPass();
  void VoltageFeasibilityPass(VoltageLimit limit_type);

  // Runs the forwards pass, setting the starting velocity to 0 m/s
  void ForwardPass();

  // Returns the forwards/backwards acceleration for a distance along the spline
  // taking into account the lateral acceleration, longitudinal acceleration,
  // and voltage limits.
  double BestAcceleration(double x, double v, bool backwards);
  double BackwardAcceleration(double x, double v) {
    return BestAcceleration(x, v, true);
  }
  double ForwardAcceleration(double x, double v) {
    return BestAcceleration(x, v, false);
  }

  // Runs the forwards pass, setting the ending velocity to 0 m/s
  void BackwardPass();

  // Runs all the planning passes.
  void Plan() {
    VoltageFeasibilityPass(VoltageLimit::kConservative);
    LateralAccelPass();
    ForwardPass();
    BackwardPass();
  }

  // Returns the feed forwards position, velocity, acceleration for an explicit
  // distance.
  ::Eigen::Matrix<double, 3, 1> FFAcceleration(double distance);

  // Returns the feed forwards voltage for an explicit distance.
  ::Eigen::Matrix<double, 2, 1> FFVoltage(double distance);

  // Returns whether a state represents a state at the end of the spline.
  bool is_at_end(::Eigen::Matrix<double, 2, 1> state) const {
    return state(0) > length() - 1e-4;
  }

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

  // Return the next position, velocity, acceleration based on the current
  // state. Updates the passed in state for the next iteration.
  ::Eigen::Matrix<double, 3, 1> GetNextXVA(
      ::std::chrono::nanoseconds dt, ::Eigen::Matrix<double, 2, 1> *state);
  ::std::vector<::Eigen::Matrix<double, 3, 1>> PlanXVA(
      ::std::chrono::nanoseconds dt);

  enum SegmentType : uint8_t {
    VELOCITY_LIMITED,
    CURVATURE_LIMITED,
    ACCELERATION_LIMITED,
    DECELERATION_LIMITED,
    VOLTAGE_LIMITED,
  };

  const ::std::vector<SegmentType> &plan_segment_type() const {
    return plan_segment_type_;
  }

  // Returns K1 and K2.
  // K2 * d^x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
  const ::Eigen::Matrix<double, 2, 1> K1(double current_ddtheta) const {
    return (::Eigen::Matrix<double, 2, 1>()
                << -robot_radius_l_ * current_ddtheta,
            robot_radius_r_ * current_ddtheta).finished();
  }

  const ::Eigen::Matrix<double, 2, 1> K2(double current_dtheta) const {
    return (::Eigen::Matrix<double, 2, 1>()
                << 1.0 - robot_radius_l_ * current_dtheta,
            1.0 + robot_radius_r_ * current_dtheta)
        .finished();
  }

 private:
  // Computes alpha for a distance.
  size_t DistanceToSegment(double distance) const {
    return ::std::max(
        static_cast<size_t>(0),
        ::std::min(plan_segment_type_.size() - 1,
                   static_cast<size_t>(::std::floor(distance / length() *
                                                    (plan_.size() - 1)))));
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

  // Robot radiuses.
  const double robot_radius_l_;
  const double robot_radius_r_;
  // Acceleration limits.
  double longitudinal_acceleration_;
  double lateral_acceleration_;
  // Transformation matrix from left, right to linear, angular
  const ::Eigen::Matrix<double, 2, 2> Tlr_to_la_;
  // Transformation matrix from linear, angular to left, right
  const ::Eigen::Matrix<double, 2, 2> Tla_to_lr_;
  // Velocities in the plan (distance for each index is defined by distance())
  ::std::vector<double> plan_;
  ::std::vector<SegmentType> plan_segment_type_;
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
