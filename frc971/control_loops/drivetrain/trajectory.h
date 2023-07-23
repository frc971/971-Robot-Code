#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_H_

#include <chrono>

#include "Eigen/Dense"

#include "aos/flatbuffers.h"
#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/spline_goal_generated.h"
#include "frc971/control_loops/drivetrain/trajectory_generated.h"
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
                if (std::abs(y) < 1e-6) {
                  return 0.0;
                }
                return (fn(t, y) - a0) / y;
              },
              v, x, dx) -
          v) +
         std::sqrt(2.0 * a0 * dx + v * v);
}

class BaseTrajectory {
 public:
  BaseTrajectory(
      const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
      const DrivetrainConfig<double> &config)
      : BaseTrajectory(constraints, config,
                       std::make_shared<StateFeedbackLoop<
                           2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                           HybridKalman<2, 2, 2>>>(
                           config.make_hybrid_drivetrain_velocity_loop())) {}

  BaseTrajectory(
      const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
      const DrivetrainConfig<double> &config,
      std::shared_ptr<
          StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                            HybridKalman<2, 2, 2>>>
          velocity_drivetrain);

  virtual ~BaseTrajectory() = default;

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

  // Returns the forwards/backwards acceleration for a distance along the spline
  // taking into account the lateral acceleration, longitudinal acceleration,
  // and voltage limits.
  double BestAcceleration(double x, double v, bool backwards) const;
  double BackwardAcceleration(double x, double v) const {
    return BestAcceleration(x, v, true);
  }
  double ForwardAcceleration(double x, double v) const {
    return BestAcceleration(x, v, false);
  }

  const StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                          HybridKalman<2, 2, 2>> &
  velocity_drivetrain() const {
    return *velocity_drivetrain_;
  }

  // Returns K1 and K2.
  // K2 * d^x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
  const Eigen::Matrix<double, 2, 1> K1(double current_ddtheta) const;
  const Eigen::Matrix<double, 2, 1> K2(double current_dtheta) const;

  // Computes K3, K4, and K5 for the provided distance.
  // K5 a + K3 v^2 + K4 v = U
  void K345(const double x, Eigen::Matrix<double, 2, 1> *K3,
            Eigen::Matrix<double, 2, 1> *K4,
            Eigen::Matrix<double, 2, 1> *K5) const;

  virtual const DistanceSplineBase &spline() const = 0;

  // Returns the length of the path in meters.
  double length() const { return spline().length(); }

  // Returns whether a state represents a state at the end of the spline.
  bool is_at_end(Eigen::Matrix<double, 2, 1> state) const {
    return state(0) > length() - 1e-4;
  }

  // Returns true if the state is invalid or unreasonable in some way.
  bool state_is_faulted(Eigen::Matrix<double, 2, 1> state) const {
    // Consider things faulted if the current velocity implies we are going
    // backwards or if any infinities/NaNs have crept in.
    return state(1) < 0 || !state.allFinite();
  }

  virtual float plan_velocity(size_t index) const = 0;
  virtual size_t distance_plan_size() const = 0;

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

  float max_lateral_accel() const { return lateral_acceleration_; }

  float max_longitudinal_accel() const { return longitudinal_acceleration_; }

  float max_voltage() const { return voltage_limit_; }

  // Return the next position, velocity, acceleration based on the current
  // state. Updates the passed in state for the next iteration.
  Eigen::Matrix<double, 3, 1> GetNextXVA(
      std::chrono::nanoseconds dt, Eigen::Matrix<double, 2, 1> *state) const;

  // Returns the distance for an index in the plan.
  double Distance(int index) const {
    return static_cast<double>(index) * length() /
           static_cast<double>(distance_plan_size() - 1);
  }

  virtual fb::SegmentConstraint plan_constraint(size_t index) const = 0;

  // Returns the feed forwards position, velocity, acceleration for an explicit
  // distance.
  Eigen::Matrix<double, 3, 1> FFAcceleration(double distance) const;

  // Returns the feed forwards voltage for an explicit distance.
  Eigen::Matrix<double, 2, 1> FFVoltage(double distance) const;

  // Computes alpha for a distance.
  size_t DistanceToSegment(double distance) const {
    return std::max(
        static_cast<size_t>(0),
        std::min(distance_plan_size() - 1,
                 static_cast<size_t>(std::floor(distance / length() *
                                                (distance_plan_size() - 1)))));
  }

  // Returns the goal state as a function of path distance, velocity.
  const ::Eigen::Matrix<double, 5, 1> GoalState(double distance,
                                                double velocity) const;

 protected:
  double robot_radius_l() const { return robot_radius_l_; }
  double robot_radius_r() const { return robot_radius_r_; }

 private:
  static float ConstraintValue(
      const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
      ConstraintType type);

  std::shared_ptr<
      StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>>
      velocity_drivetrain_;

  DrivetrainConfig<double> config_;

  // Robot radiuses.
  double robot_radius_l_;
  double robot_radius_r_;
  float lateral_acceleration_ = 3.0;
  float longitudinal_acceleration_ = 2.0;
  float voltage_limit_ = 12.0;
};

// A wrapper around the Trajectory flatbuffer to allow for controlling to a
// spline using a pre-generated trajectory.
class FinishedTrajectory : public BaseTrajectory {
 public:
  // Note: The lifetime of the supplied buffer is assumed to be greater than
  // that of this object.
  explicit FinishedTrajectory(
      const DrivetrainConfig<double> &config, const fb::Trajectory *buffer,
      std::shared_ptr<
          StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                            HybridKalman<2, 2, 2>>>
          velocity_drivetrain);

  explicit FinishedTrajectory(const DrivetrainConfig<double> &config,
                              const fb::Trajectory *buffer)
      : FinishedTrajectory(
            config, buffer,
            std::make_shared<StateFeedbackLoop<
                2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                HybridKalman<2, 2, 2>>>(
                config.make_hybrid_drivetrain_velocity_loop())) {}

  FinishedTrajectory(const FinishedTrajectory &) = delete;
  FinishedTrajectory &operator=(const FinishedTrajectory &) = delete;
  FinishedTrajectory(FinishedTrajectory &&) = default;
  FinishedTrajectory &operator=(FinishedTrajectory &&) = default;

  virtual ~FinishedTrajectory() = default;

  // Takes the 5-element state that is [x, y, theta, v_left, v_right] and
  // converts it to a path-relative state, using distance as a linearization
  // point (i.e., distance should be roughly equal to the actual distance along
  // the path).
  Eigen::Matrix<double, 5, 1> StateToPathRelativeState(
      double distance, const Eigen::Matrix<double, 5, 1> &state,
      bool drive_backwards) const;

  // Retrieves the gain matrix K for a given distance along the path.
  Eigen::Matrix<double, 2, 5> GainForDistance(double distance) const;

  size_t distance_plan_size() const override;
  float plan_velocity(size_t index) const override;
  fb::SegmentConstraint plan_constraint(size_t index) const override;

  bool drive_spline_backwards() const {
    return trajectory().drive_spline_backwards();
  }

  int spline_handle() const { return trajectory().handle(); }
  const fb::Trajectory &trajectory() const { return *buffer_; }

 private:
  const DistanceSplineBase &spline() const override { return spline_; }
  const fb::Trajectory *buffer_;
  FinishedDistanceSpline spline_;
};

// Class to handle plannign a trajectory and producing a flatbuffer containing
// all the information required to create a FinishedTrajectory;
class Trajectory : public BaseTrajectory {
 public:
  Trajectory(const SplineGoal &spline_goal,
             const DrivetrainConfig<double> &config);
  Trajectory(
      DistanceSpline &&spline, const DrivetrainConfig<double> &config,
      const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
      int spline_idx = 0, double vmax = 10.0, int num_distance = 0);

  virtual ~Trajectory() = default;

  std::vector<Eigen::Matrix<double, 3, 1>> PlanXVA(std::chrono::nanoseconds dt);

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

  // Runs the forwards pass, setting the ending velocity to 0 m/s
  void BackwardPass();

  // Runs all the planning passes.
  void Plan() {
    VoltageFeasibilityPass(VoltageLimit::kConservative);
    LateralAccelPass();
    ForwardPass();
    BackwardPass();
    CalculatePathGains();
  }

  // Returns a list of the distances.  Mostly useful for plotting.
  const std::vector<double> Distances() const;
  // Returns the distance for an index in the plan.
  double Distance(int index) const {
    return static_cast<double>(index) * length() /
           static_cast<double>(plan_.size() - 1);
  }

  const std::vector<fb::SegmentConstraint> &plan_segment_type() const {
    return plan_segment_type_;
  }

  // The controller represented by these functions uses a discrete-time,
  // finite-horizon LQR with states that are relative to the predicted path
  // of the robot to produce a gain to be used on the error.
  // The controller does not currently account for saturation, but is defined
  // in a way that would make accounting for saturation feasible.
  // This controller uses a state of:
  // distance along path
  // distance lateral to path (positive when robot is to the left of the path).
  // heading relative to path (positive if robot pointed to left).
  // v_left (speed of left side of robot)
  // v_right (speed of right side of robot).

  // Retrieve the continuous-time A/B matrices for the path-relative system
  // at the given distance along the path. Performs all linearizations about
  // the nominal velocity that the robot should be following at that point
  // along the path.
  void PathRelativeContinuousSystem(double distance,
                                    Eigen::Matrix<double, 5, 5> *A,
                                    Eigen::Matrix<double, 5, 2> *B);
  // Retrieve the continuous-time A/B matrices for the path-relative system
  // given the current path-relative state, as defined above.
  void PathRelativeContinuousSystem(const Eigen::Matrix<double, 5, 1> &X,
                                    Eigen::Matrix<double, 5, 5> *A,
                                    Eigen::Matrix<double, 5, 2> *B);

  // Takes the 5-element state that is [x, y, theta, v_left, v_right] and
  // converts it to a path-relative state, using distance as a linearization
  // point (i.e., distance should be roughly equal to the actual distance along
  // the path).
  Eigen::Matrix<double, 5, 1> StateToPathRelativeState(
      double distance, const Eigen::Matrix<double, 5, 1> &state);

  // Estimates the current distance along the path, given the current expected
  // distance and the [x, y, theta, v_left, v_right] state.
  double EstimateDistanceAlongPath(double nominal_distance,
                                   const Eigen::Matrix<double, 5, 1> &state);

  // Calculates all the gains for each point along the planned trajectory.
  // Only called directly in tests; this is normally a part of the planning
  // phase, and is a relatively expensive operation.
  void CalculatePathGains();

  flatbuffers::Offset<fb::Trajectory> Serialize(
      flatbuffers::FlatBufferBuilder *fbb) const;

  const std::vector<double> plan() const { return plan_; }

  const DistanceSpline &spline() const override { return spline_; }

 private:
  float plan_velocity(size_t index) const override { return plan_[index]; }
  size_t distance_plan_size() const override { return plan_.size(); }

  fb::SegmentConstraint plan_constraint(size_t index) const override {
    return plan_segment_type_[index];
  }

  const int spline_idx_;

  // The spline we are planning for.
  const DistanceSpline spline_;

  const DrivetrainConfig<double> config_;

  // Velocities in the plan (distance for each index is defined by Distance())
  std::vector<double> plan_;
  // Gain matrices to use for the path-relative state error at each stage in the
  // plan Individual elements of the plan_gains_ vector are separated by
  // config_.dt in time.
  // The first value in the pair is the distance along the path corresponding to
  // the gain matrix; the second value is the gain itself.
  std::vector<std::pair<double, Eigen::Matrix<float, 2, 5>>> plan_gains_;
  std::vector<fb::SegmentConstraint> plan_segment_type_;

  bool drive_spline_backwards_ = false;
};

// Returns the continuous time dynamics given the [x, y, theta, vl, vr] state
// and the [vl, vr] input voltage.
inline Eigen::Matrix<double, 5, 1> ContinuousDynamics(
    const StateFeedbackHybridPlant<2, 2, 2> &velocity_drivetrain,
    const Eigen::Matrix<double, 2, 2> &Tlr_to_la,
    const Eigen::Matrix<double, 5, 1> X, const Eigen::Matrix<double, 2, 1> U) {
  const auto &velocity = X.block<2, 1>(3, 0);
  const double theta = X(2);
  Eigen::Matrix<double, 2, 1> la = Tlr_to_la * velocity;
  return (Eigen::Matrix<double, 5, 1>() << std::cos(theta) * la(0),
          std::sin(theta) * la(0), la(1),
          (velocity_drivetrain.coefficients().A_continuous * velocity +
           velocity_drivetrain.coefficients().B_continuous * U))
      .finished();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_H_
