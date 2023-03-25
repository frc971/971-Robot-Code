#include "frc971/control_loops/drivetrain/trajectory.h"

#include <chrono>

#include "aos/util/math.h"
#include "Eigen/Dense"
#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/dlqr.h"
#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/hybrid_state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

namespace {
float DefaultConstraint(ConstraintType type) {
  switch (type) {
    case ConstraintType::LONGITUDINAL_ACCELERATION:
      return 2.0;
    case ConstraintType::LATERAL_ACCELERATION:
      return 3.0;
    case ConstraintType::VOLTAGE:
      return 12.0;
    case ConstraintType::VELOCITY:
    case ConstraintType::CONSTRAINT_TYPE_UNDEFINED:
      LOG(FATAL) << "No default constraint value for "
                 << EnumNameConstraintType(type);
  }
  LOG(FATAL) << "Invalid ConstraintType " << static_cast<int>(type);
}
}  // namespace

FinishedTrajectory::FinishedTrajectory(
    const DrivetrainConfig<double> &config, const fb::Trajectory *buffer,
    std::shared_ptr<
        StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                          HybridKalman<2, 2, 2>>>
        velocity_drivetrain)
    : BaseTrajectory(CHECK_NOTNULL(CHECK_NOTNULL(buffer->spline())->spline())
                         ->constraints(),
                     config, std::move(velocity_drivetrain)),
      buffer_(buffer),
      spline_(*buffer_->spline()) {}

const Eigen::Matrix<double, 2, 1> BaseTrajectory::K1(
    double current_ddtheta) const {
  return (Eigen::Matrix<double, 2, 1>() << -robot_radius_l_ * current_ddtheta,
          robot_radius_r_ * current_ddtheta)
      .finished();
}

const Eigen::Matrix<double, 2, 1> BaseTrajectory::K2(
    double current_dtheta) const {
  return (Eigen::Matrix<double, 2, 1>()
              << 1.0 - robot_radius_l_ * current_dtheta,
          1.0 + robot_radius_r_ * current_dtheta)
      .finished();
}

void BaseTrajectory::K345(const double x, Eigen::Matrix<double, 2, 1> *K3,
                          Eigen::Matrix<double, 2, 1> *K4,
                          Eigen::Matrix<double, 2, 1> *K5) const {
  const double current_ddtheta = spline().DDTheta(x);
  const double current_dtheta = spline().DTheta(x);
  // We've now got the equation:
  //     K2 * d^x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
  const Eigen::Matrix<double, 2, 1> my_K2 = K2(current_dtheta);

  const Eigen::Matrix<double, 2, 2> B_inverse =
      velocity_drivetrain_->plant().coefficients().B_continuous.inverse();

  // Now, rephrase it as K5 a + K3 v^2 + K4 v = U
  *K3 = B_inverse * K1(current_ddtheta);
  *K4 = -B_inverse * velocity_drivetrain_->plant().coefficients().A_continuous *
        my_K2;
  *K5 = B_inverse * my_K2;
}

BaseTrajectory::BaseTrajectory(
    const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
    const DrivetrainConfig<double> &config,
    std::shared_ptr<
        StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                          HybridKalman<2, 2, 2>>>
        velocity_drivetrain)
    : velocity_drivetrain_(std::move(velocity_drivetrain)),
      config_(config),
      robot_radius_l_(config.robot_radius),
      robot_radius_r_(config.robot_radius),
      lateral_acceleration_(
          ConstraintValue(constraints, ConstraintType::LATERAL_ACCELERATION)),
      longitudinal_acceleration_(ConstraintValue(
          constraints, ConstraintType::LONGITUDINAL_ACCELERATION)),
      voltage_limit_(ConstraintValue(constraints, ConstraintType::VOLTAGE)) {}

Trajectory::Trajectory(const SplineGoal &spline_goal,
                       const DrivetrainConfig<double> &config)
    : Trajectory(DistanceSpline{spline_goal.spline()}, config,
                 spline_goal.spline()->constraints(),
                 spline_goal.spline_idx()) {
  drive_spline_backwards_ = spline_goal.drive_spline_backwards();
}

Trajectory::Trajectory(
    DistanceSpline &&input_spline, const DrivetrainConfig<double> &config,
    const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
    int spline_idx, double vmax, int num_distance)
    : BaseTrajectory(constraints, config),
      spline_idx_(spline_idx),
      spline_(std::move(input_spline)),
      config_(config),
      plan_(num_distance == 0
                ? std::max(10000, static_cast<int>(spline_.length() / 0.0025))
                : num_distance,
            vmax),
      plan_segment_type_(plan_.size(),
                         fb::SegmentConstraint::VELOCITY_LIMITED) {
  if (constraints != nullptr) {
    for (const Constraint *constraint : *constraints) {
      if (constraint->constraint_type() == ConstraintType::VELOCITY) {
        LimitVelocity(constraint->start_distance(), constraint->end_distance(),
                      constraint->value());
      }
    }
  }
}

void Trajectory::LateralAccelPass() {
  for (size_t i = 0; i < plan_.size(); ++i) {
    const double distance = Distance(i);
    const double velocity_limit = LateralVelocityCurvature(distance);
    if (velocity_limit < plan_[i]) {
      plan_[i] = velocity_limit;
      plan_segment_type_[i] = fb::SegmentConstraint::CURVATURE_LIMITED;
    }
  }
}

void Trajectory::VoltageFeasibilityPass(VoltageLimit limit_type) {
  for (size_t i = 0; i < plan_.size(); ++i) {
    const double distance = Distance(i);
    const double velocity_limit = VoltageVelocityLimit(distance, limit_type);
    if (velocity_limit < plan_[i]) {
      plan_[i] = velocity_limit;
      plan_segment_type_[i] = fb::SegmentConstraint::VOLTAGE_LIMITED;
    }
  }
}

double BaseTrajectory::BestAcceleration(double x, double v,
                                        bool backwards) const {
  Eigen::Matrix<double, 2, 1> K3;
  Eigen::Matrix<double, 2, 1> K4;
  Eigen::Matrix<double, 2, 1> K5;
  K345(x, &K3, &K4, &K5);

  // Now, solve for all a's and find the best one which meets our criteria.
  const Eigen::Matrix<double, 2, 1> C = K3 * v * v + K4 * v;
  double min_voltage_accel = std::numeric_limits<double>::infinity();
  double max_voltage_accel = -min_voltage_accel;
  for (const double a : {(max_voltage() - C(0, 0)) / K5(0, 0),
                         (max_voltage() - C(1, 0)) / K5(1, 0),
                         (-max_voltage() - C(0, 0)) / K5(0, 0),
                         (-max_voltage() - C(1, 0)) / K5(1, 0)}) {
    const Eigen::Matrix<double, 2, 1> U = K5 * a + K3 * v * v + K4 * v;
    if ((U.array().abs() < max_voltage() + 1e-6).all()) {
      min_voltage_accel = std::min(a, min_voltage_accel);
      max_voltage_accel = std::max(a, max_voltage_accel);
    }
  }
  double best_accel = backwards ? min_voltage_accel : max_voltage_accel;

  double min_friction_accel, max_friction_accel;
  FrictionLngAccelLimits(x, v, &min_friction_accel, &max_friction_accel);
  if (backwards) {
    best_accel = std::max(best_accel, min_friction_accel);
  } else {
    best_accel = std::min(best_accel, max_friction_accel);
  }

  // Ideally, the max would never be less than the min, but due to the way that
  // the runge kutta solver works, it sometimes ticks over the edge.
  if (max_friction_accel < min_friction_accel) {
    VLOG(1) << "At x " << x << " v " << v << " min fric acc "
            << min_friction_accel << " max fric accel " << max_friction_accel;
  }
  if (best_accel < min_voltage_accel || best_accel > max_voltage_accel) {
    LOG(WARNING) << "Viable friction limits and viable voltage limits do not "
                    "overlap (x: "
                 << x << ", v: " << v << ", backwards: " << backwards
                 << ") best_accel = " << best_accel << ", min voltage "
                 << min_voltage_accel << ", max voltage " << max_voltage_accel
                 << " min friction " << min_friction_accel << " max friction "
                 << max_friction_accel << ".";

    // Don't actually do anything--this will just result in attempting to drive
    // higher voltages thatn we have available. In practice, that'll probably
    // work out fine.
  }

  return best_accel;
}

double BaseTrajectory::LateralVelocityCurvature(double distance) const {
  // To calculate these constraints, we first note that:
  // wheel accels = K2 * v_robot' + K1 * v_robot^2
  // All that this logic does is solve for v_robot, leaving v_robot' free,
  // assuming that the wheels are at their limits.
  // To do this, we:
  //
  // 1) Determine what the wheel accels will be at the limit--since we have
  // two free variables (v_robot, v_robot'), both wheels will be at their
  // limits--if in a sufficiently tight turn (such that the signs of the
  // coefficients of K2 are different), then the wheels will be accelerating
  // in opposite directions; otherwise, they accelerate in the same direction.
  // The magnitude of these per-wheel accelerations is a function of velocity,
  // so it must also be solved for.
  //
  // 2) Eliminate that v_robot' term (since we don't care
  // about it) by multiplying be a "K2prime" term (where K2prime * K2 = 0) on
  // both sides of the equation.
  //
  // 3) Solving the relatively tractable remaining equation, which is
  // basically just grouping all the terms together in one spot and taking the
  // 4th root of everything.
  const double dtheta = spline().DTheta(distance);
  const Eigen::Matrix<double, 1, 2> K2prime =
      K2(dtheta).transpose() *
      (Eigen::Matrix<double, 2, 2>() << 0, 1, -1, 0).finished();
  // Calculate whether the wheels are spinning in opposite directions.
  const bool opposites = K2prime(0) * K2prime(1) < 0;
  const Eigen::Matrix<double, 2, 1> K1calc = K1(spline().DDTheta(distance));
  const double lat_accel_squared = std::pow(dtheta / max_lateral_accel(), 2);
  const double curvature_change_term =
      (K2prime * K1calc).value() /
      (K2prime *
       (Eigen::Matrix<double, 2, 1>() << 1.0, (opposites ? -1.0 : 1.0))
           .finished() *
       max_longitudinal_accel())
          .value();
  const double vel_inv = std::sqrt(
      std::sqrt(std::pow(curvature_change_term, 2) + lat_accel_squared));
  if (vel_inv == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return 1.0 / vel_inv;
}

void BaseTrajectory::FrictionLngAccelLimits(double x, double v,
                                            double *min_accel,
                                            double *max_accel) const {
  // First, calculate the max longitudinal acceleration that can be achieved
  // by either wheel given the friction elliipse that we have.
  const double lateral_acceleration = v * v * spline().DDXY(x).norm();
  const double max_wheel_lng_accel_squared =
      1.0 - std::pow(lateral_acceleration / max_lateral_accel(), 2.0);
  if (max_wheel_lng_accel_squared < 0.0) {
    VLOG(1) << "Something (probably Runge-Kutta) queried invalid velocity " << v
            << " at distance " << x;
    // If we encounter this, it means that the Runge-Kutta has attempted to
    // sample points a bit past the edge of the friction boundary. If so, we
    // gradually ramp the min/max accels to be more and more incorrect (note
    // how min_accel > max_accel if we reach this case) to avoid causing any
    // numerical issues.
    *min_accel =
        std::sqrt(-max_wheel_lng_accel_squared) * max_longitudinal_accel();
    *max_accel = -*min_accel;
    return;
  }
  *min_accel = -std::numeric_limits<double>::infinity();
  *max_accel = std::numeric_limits<double>::infinity();

  // Calculate max/min accelerations by calculating what the robots overall
  // longitudinal acceleration would be if each wheel were running at the max
  // forwards/backwards longitudinal acceleration.
  const double max_wheel_lng_accel =
      max_longitudinal_accel() * std::sqrt(max_wheel_lng_accel_squared);
  const Eigen::Matrix<double, 2, 1> K1v2 = K1(spline().DDTheta(x)) * v * v;
  const Eigen::Matrix<double, 2, 1> K2inv =
      K2(spline().DTheta(x)).cwiseInverse();
  // Store the accelerations of the robot corresponding to each wheel being at
  // the max/min acceleration. The first coefficient in each vector
  // corresponds to the left wheel, the second to the right wheel.
  const Eigen::Matrix<double, 2, 1> accels1 =
      K2inv.array() * (-K1v2.array() + max_wheel_lng_accel);
  const Eigen::Matrix<double, 2, 1> accels2 =
      K2inv.array() * (-K1v2.array() - max_wheel_lng_accel);

  // If either term is non-finite, that suggests that a term of K2 is zero
  // (which is physically possible when turning such that one wheel is
  // stationary), so just ignore that side of the drivetrain.
  if (std::isfinite(accels1(0))) {
    // The inner max/min in this case determines which of the two cases (+ or
    // - acceleration on the left wheel) we care about--in a sufficiently
    // tight turning radius, the left hweel may be accelerating backwards when
    // the robot as a whole accelerates forwards. We then use that
    // acceleration to bound the min/max accel.
    *min_accel = std::max(*min_accel, std::min(accels1(0), accels2(0)));
    *max_accel = std::min(*max_accel, std::max(accels1(0), accels2(0)));
  }
  // Same logic as previous if-statement, but for the right wheel.
  if (std::isfinite(accels1(1))) {
    *min_accel = std::max(*min_accel, std::min(accels1(1), accels2(1)));
    *max_accel = std::min(*max_accel, std::max(accels1(1), accels2(1)));
  }
}

double Trajectory::VoltageVelocityLimit(
    double distance, VoltageLimit limit_type,
    Eigen::Matrix<double, 2, 1> *constraint_voltages) const {
  // To sketch an outline of the math going on here, we start with the basic
  // dynamics of the robot along the spline:
  // K2 * v_robot' + K1 * v_robot^2 = A * K2 * v_robot + B * U
  // We need to determine the maximum v_robot given constrained U and free
  // v_robot'.
  // Similarly to the friction constraints, we accomplish this by first
  // multiplying by a K2prime term to eliminate the v_robot' term.
  // As with the friction constraints, we also know that the limits will occur
  // when both sides of the drivetrain are driven at their max magnitude
  // voltages, although they may be driven at different signs.
  // Once we determine whether the voltages match signs, we still have to
  // consider both possible pairings (technically we could probably
  // predetermine which pairing, e.g. +/- or -/+, we acre about, but we don't
  // need to).
  //
  // For each pairing, we then get to solve a quadratic formula for the robot
  // velocity at those voltages. This gives us up to 4 solutions, of which
  // up to 3 will give us positive velocities; each solution velocity
  // corresponds to a transition from feasibility to infeasibility, where a
  // velocity of zero is always feasible, and there will always be 0, 1, or 3
  // positive solutions. Among the positive solutions, we take both the min
  // and the max--the min will be the highest velocity such that all
  // velocities between zero and that velocity are valid; the max will be
  // the highest feasible velocity. Which we return depends on what the
  // limit_type is.
  //
  // Sketching the actual math:
  // K2 * v_robot' + K1 * v_robot^2 = A * K2 * v_robot +/- B * U_max
  // K2prime * K1 * v_robot^2 = K2prime * (A * K2 * v_robot +/- B * U_max)
  // a v_robot^2 + b v_robot +/- c = 0
  const Eigen::Matrix<double, 2, 2> B =
      velocity_drivetrain().plant().coefficients().B_continuous;
  const double dtheta = spline().DTheta(distance);
  const Eigen::Matrix<double, 2, 1> BinvK2 = B.inverse() * K2(dtheta);
  // Because voltages can actually impact *both* wheels, in order to determine
  // whether the voltages will have opposite signs, we need to use B^-1 * K2.
  const bool opposite_voltages = BinvK2(0) * BinvK2(1) > 0.0;
  const Eigen::Matrix<double, 1, 2> K2prime =
      K2(dtheta).transpose() *
      (Eigen::Matrix<double, 2, 2>() << 0, 1, -1, 0).finished();
  const double a = K2prime * K1(spline().DDTheta(distance));
  const double b = -K2prime *
                   velocity_drivetrain().plant().coefficients().A_continuous *
                   K2(dtheta);
  const Eigen::Matrix<double, 1, 2> c_coeff = -K2prime * B;
  // Calculate the "positive" version of the voltage limits we will use.
  const Eigen::Matrix<double, 2, 1> abs_volts =
      max_voltage() *
      (Eigen::Matrix<double, 2, 1>() << 1.0, (opposite_voltages ? -1.0 : 1.0))
          .finished();

  double min_valid_vel = std::numeric_limits<double>::infinity();
  if (limit_type == VoltageLimit::kAggressive) {
    min_valid_vel = 0.0;
  }
  // Iterate over both possibilites for +/- voltage, and solve the quadratic
  // formula. For every positive solution, adjust the velocity limit
  // appropriately.
  for (const double sign : {1.0, -1.0}) {
    const Eigen::Matrix<double, 2, 1> U = sign * abs_volts;
    const double prev_vel = min_valid_vel;
    const double c = c_coeff * U;
    const double determinant = b * b - 4 * a * c;
    if (a == 0) {
      // If a == 0, that implies we are on a constant curvature path, in which
      // case we just have b * v + c = 0.
      // Note that if -b * c > 0.0, then vel will be greater than zero and b
      // will be non-zero.
      if (-b * c > 0.0) {
        const double vel = -c / b;
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = std::min(min_valid_vel, vel);
        } else {
          min_valid_vel = std::max(min_valid_vel, vel);
        }
      } else if (b == 0) {
        // If a and b are zero, then we are travelling in a straight line and
        // have no voltage-based velocity constraints.
        min_valid_vel = std::numeric_limits<double>::infinity();
      }
    } else if (determinant > 0) {
      const double sqrt_determinant = std::sqrt(determinant);
      const double high_vel = (-b + sqrt_determinant) / (2.0 * a);
      const double low_vel = (-b - sqrt_determinant) / (2.0 * a);
      if (low_vel > 0) {
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = std::min(min_valid_vel, low_vel);
        } else {
          min_valid_vel = std::max(min_valid_vel, low_vel);
        }
      }
      if (high_vel > 0) {
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = std::min(min_valid_vel, high_vel);
        } else {
          min_valid_vel = std::max(min_valid_vel, high_vel);
        }
      }
    } else if (determinant == 0 && -b * a > 0) {
      const double vel = -b / (2.0 * a);
      if (vel > 0.0) {
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = std::min(min_valid_vel, vel);
        } else {
          min_valid_vel = std::max(min_valid_vel, vel);
        }
      }
    }
    if (constraint_voltages != nullptr && prev_vel != min_valid_vel) {
      *constraint_voltages = U;
    }
  }
  return min_valid_vel;
}

void Trajectory::ForwardPass() {
  plan_[0] = 0.0;
  const double delta_distance = Distance(1) - Distance(0);
  for (size_t i = 0; i < plan_.size() - 1; ++i) {
    const double distance = Distance(i);

    // Integrate our acceleration forward one step.
    const double new_plan_velocity = IntegrateAccelForDistance(
        [this](double x, double v) { return ForwardAcceleration(x, v); },
        plan_[i], distance, delta_distance);

    if (new_plan_velocity <= plan_[i + 1]) {
      plan_[i + 1] = new_plan_velocity;
      plan_segment_type_[i] = fb::SegmentConstraint::ACCELERATION_LIMITED;
    }
  }
}

void Trajectory::BackwardPass() {
  const double delta_distance = Distance(0) - Distance(1);
  plan_.back() = 0.0;
  for (size_t i = plan_.size() - 1; i > 0; --i) {
    const double distance = Distance(i);

    // Integrate our deceleration back one step.
    const double new_plan_velocity = IntegrateAccelForDistance(
        [this](double x, double v) { return BackwardAcceleration(x, v); },
        plan_[i], distance, delta_distance);

    if (new_plan_velocity <= plan_[i - 1]) {
      plan_[i - 1] = new_plan_velocity;
      plan_segment_type_[i - 1] = fb::SegmentConstraint::DECELERATION_LIMITED;
    }
  }
}

Eigen::Matrix<double, 3, 1> BaseTrajectory::FFAcceleration(
    double distance) const {
  if (distance < 0.0) {
    // Make sure we don't end up off the beginning of the curve.
    distance = 0.0;
  } else if (distance > length()) {
    // Make sure we don't end up off the end of the curve.
    distance = length();
  }
  const size_t before_index = DistanceToSegment(distance);
  const size_t after_index =
      std::min(before_index + 1, distance_plan_size() - 1);

  const double before_distance = Distance(before_index);
  const double after_distance = Distance(after_index);

  // And then also make sure we aren't curvature limited.
  const double vcurvature = LateralVelocityCurvature(distance);

  double acceleration;
  double velocity;
  // TODO(james): While technically correct for sufficiently small segment
  // steps, this method of switching between limits has a tendency to produce
  // sudden jumps in acceelrations, which is undesirable.
  switch (plan_constraint(DistanceToSegment(distance))) {
    case fb::SegmentConstraint::VELOCITY_LIMITED:
      acceleration = 0.0;
      velocity =
          (plan_velocity(before_index) + plan_velocity(after_index)) / 2.0;
      // TODO(austin): Accelerate or decelerate until we hit the limit in the
      // time slice.  Otherwise our acceleration will be lying for this slice.
      // Do note, we've got small slices so the effect will be small.
      break;
    case fb::SegmentConstraint::CURVATURE_LIMITED:
      velocity = vcurvature;
      FrictionLngAccelLimits(distance, velocity, &acceleration, &acceleration);
      break;
    case fb::SegmentConstraint::VOLTAGE_LIMITED:
      // Normally, we expect that voltage limited plans will all get dominated
      // by the acceleration/deceleration limits. This may not always be true;
      // if we ever encounter this error, we just need to back out what the
      // accelerations would be in this case.
      LOG(FATAL) << "Unexpectedly got VOLTAGE_LIMITED plan.";
      break;
    case fb::SegmentConstraint::ACCELERATION_LIMITED:
      // TODO(james): The integration done here and in the DECELERATION_LIMITED
      // can technically cause us to violate friction constraints. We currently
      // don't do anything about it to avoid causing sudden jumps in voltage,
      // but we probably *should* at some point.
      velocity = IntegrateAccelForDistance(
          [this](double x, double v) { return ForwardAcceleration(x, v); },
          plan_velocity(before_index), before_distance,
          distance - before_distance);
      acceleration = ForwardAcceleration(distance, velocity);
      break;
    case fb::SegmentConstraint::DECELERATION_LIMITED:
      velocity = IntegrateAccelForDistance(
          [this](double x, double v) { return BackwardAcceleration(x, v); },
          plan_velocity(after_index), after_distance,
          distance - after_distance);
      acceleration = BackwardAcceleration(distance, velocity);
      break;
    default:
      AOS_LOG(FATAL, "Unknown segment type %d\n",
              static_cast<int>(plan_constraint(DistanceToSegment(distance))));
      break;
  }

  return (Eigen::Matrix<double, 3, 1>() << distance, velocity, acceleration)
      .finished();
}

size_t FinishedTrajectory::distance_plan_size() const {
  return trajectory().has_distance_based_plan()
             ? trajectory().distance_based_plan()->size()
             : 0u;
}

fb::SegmentConstraint FinishedTrajectory::plan_constraint(size_t index) const {
  CHECK_LT(index, distance_plan_size());
  return trajectory().distance_based_plan()->Get(index)->segment_constraint();
}

float FinishedTrajectory::plan_velocity(size_t index) const {
  CHECK_LT(index, distance_plan_size());
  return trajectory().distance_based_plan()->Get(index)->velocity();
}

Eigen::Matrix<double, 2, 1> BaseTrajectory::FFVoltage(double distance) const {
  const Eigen::Matrix<double, 3, 1> xva = FFAcceleration(distance);
  const double velocity = xva(1);
  const double acceleration = xva(2);

  Eigen::Matrix<double, 2, 1> K3;
  Eigen::Matrix<double, 2, 1> K4;
  Eigen::Matrix<double, 2, 1> K5;
  K345(distance, &K3, &K4, &K5);

  return K5 * acceleration + K3 * velocity * velocity + K4 * velocity;
}

const std::vector<double> Trajectory::Distances() const {
  std::vector<double> d;
  d.reserve(plan_.size());
  for (size_t i = 0; i < plan_.size(); ++i) {
    d.push_back(Distance(i));
  }
  return d;
}

Eigen::Matrix<double, 3, 1> BaseTrajectory::GetNextXVA(
    std::chrono::nanoseconds dt, Eigen::Matrix<double, 2, 1> *state) const {
  double dt_float = ::aos::time::DurationInSeconds(dt);

  const double last_distance = (*state)(0);
  // TODO(austin): This feels like something that should be pulled out into
  // a library for re-use.
  *state = RungeKutta(
      [this](const Eigen::Matrix<double, 2, 1> x) {
        Eigen::Matrix<double, 3, 1> xva = FFAcceleration(x(0));
        return (Eigen::Matrix<double, 2, 1>() << x(1), xva(2)).finished();
      },
      *state, dt_float);
  // Force the distance to move forwards, to guarantee that we actually finish
  // the planning.
  constexpr double kMinDistanceIncrease = 1e-7;
  if ((*state)(0) < last_distance + kMinDistanceIncrease) {
    (*state)(0) = last_distance + kMinDistanceIncrease;
  }

  Eigen::Matrix<double, 3, 1> result = FFAcceleration((*state)(0));
  (*state)(1) = result(1);
  return result;
}

std::vector<Eigen::Matrix<double, 3, 1>> Trajectory::PlanXVA(
    std::chrono::nanoseconds dt) {
  Eigen::Matrix<double, 2, 1> state = Eigen::Matrix<double, 2, 1>::Zero();
  std::vector<Eigen::Matrix<double, 3, 1>> result;
  result.emplace_back(FFAcceleration(0));
  result.back()(1) = 0.0;

  while (!is_at_end(state)) {
    if (state_is_faulted(state)) {
      LOG(WARNING)
          << "Found invalid state in generating spline and aborting. This is "
             "likely due to a spline with extremely high jerk/changes in "
             "curvature with an insufficiently small step size.";
      return {};
    }
    result.emplace_back(GetNextXVA(dt, &state));
  }
  return result;
}

void Trajectory::LimitVelocity(double starting_distance, double ending_distance,
                               const double max_velocity) {
  const double segment_length = ending_distance - starting_distance;

  const double min_length = length() / static_cast<double>(plan_.size() - 1);
  if (starting_distance > ending_distance) {
    AOS_LOG(FATAL, "End before start: %f > %f\n", starting_distance,
            ending_distance);
  }
  starting_distance = std::min(length(), std::max(0.0, starting_distance));
  ending_distance = std::min(length(), std::max(0.0, ending_distance));
  if (segment_length < min_length) {
    const size_t plan_index = static_cast<size_t>(
        std::round((starting_distance + ending_distance) / 2.0 / min_length));
    if (max_velocity < plan_[plan_index]) {
      plan_[plan_index] = max_velocity;
    }
  } else {
    for (size_t i = DistanceToSegment(starting_distance) + 1;
         i < DistanceToSegment(ending_distance) + 1; ++i) {
      if (max_velocity < plan_[i]) {
        plan_[i] = max_velocity;
        if (i < DistanceToSegment(ending_distance)) {
          plan_segment_type_[i] = fb::SegmentConstraint::VELOCITY_LIMITED;
        }
      }
    }
  }
}

void Trajectory::PathRelativeContinuousSystem(double distance,
                                              Eigen::Matrix<double, 5, 5> *A,
                                              Eigen::Matrix<double, 5, 2> *B) {
  const double nominal_velocity = FFAcceleration(distance)(1);
  const double dtheta_dt = spline().DThetaDt(distance, nominal_velocity);
  // Calculate the "path-relative" coordinates, which are:
  // [[distance along the path],
  //  [lateral position along path],
  //  [theta],
  //  [left wheel velocity],
  //  [right wheel velocity]]
  Eigen::Matrix<double, 5, 1> nominal_X;
  nominal_X << distance, 0.0, 0.0,
      nominal_velocity - dtheta_dt * robot_radius_l(),
      nominal_velocity + dtheta_dt * robot_radius_r();
  PathRelativeContinuousSystem(nominal_X, A, B);
}

void Trajectory::PathRelativeContinuousSystem(
    const Eigen::Matrix<double, 5, 1> &X, Eigen::Matrix<double, 5, 5> *A,
    Eigen::Matrix<double, 5, 2> *B) {
  A->setZero();
  B->setZero();
  const double theta = X(2);
  const double ctheta = std::cos(theta);
  const double stheta = std::sin(theta);
  const double curvature = spline().DTheta(X(0));
  const double longitudinal_velocity = (X(3) + X(4)) / 2.0;
  const double diameter = robot_radius_l() + robot_radius_r();
  // d_dpath / dt = (v_left + v_right) / 2.0 * cos(theta)
  // (d_dpath / dt) / dv_left = cos(theta) / 2.0
  (*A)(0, 3) = ctheta / 2.0;
  // (d_dpath / dt) / dv_right = cos(theta) / 2.0
  (*A)(0, 4) = ctheta / 2.0;
  // (d_dpath / dt) / dtheta = -(v_left + v_right) / 2.0 * sin(theta)
  (*A)(0, 2) = -longitudinal_velocity * stheta;
  // d_dlat / dt = (v_left + v_right) / 2.0 * sin(theta)
  // (d_dlat / dt) / dv_left = sin(theta) / 2.0
  (*A)(1, 3) = stheta / 2.0;
  // (d_dlat / dt) / dv_right = sin(theta) / 2.0
  (*A)(1, 4) = stheta / 2.0;
  // (d_dlat / dt) / dtheta = (v_left + v_right) / 2.0 * cos(theta)
  (*A)(1, 2) = longitudinal_velocity * ctheta;
  // dtheta / dt = (v_right - v_left) / diameter - curvature * (v_left +
  //                v_right) / 2.0
  // (dtheta / dt) / dv_left = -1.0 / diameter - curvature / 2.0
  (*A)(2, 3) = -1.0 / diameter - curvature / 2.0;
  // (dtheta / dt) / dv_right = 1.0 / diameter - curvature / 2.0
  (*A)(2, 4) = 1.0 / diameter - curvature / 2.0;
  // v_{left,right} / dt = the normal LTI system.
  A->block<2, 2>(3, 3) =
      velocity_drivetrain().plant().coefficients().A_continuous;
  B->block<2, 2>(3, 0) =
      velocity_drivetrain().plant().coefficients().B_continuous;
}

double Trajectory::EstimateDistanceAlongPath(
    double nominal_distance, const Eigen::Matrix<double, 5, 1> &state) {
  const double nominal_theta = spline().Theta(nominal_distance);
  const Eigen::Matrix<double, 2, 1> xy_err =
      state.block<2, 1>(0, 0) - spline().XY(nominal_distance);
  return nominal_distance + xy_err.x() * std::cos(nominal_theta) +
         xy_err.y() * std::sin(nominal_theta);
}

Eigen::Matrix<double, 5, 1> FinishedTrajectory::StateToPathRelativeState(
    double distance, const Eigen::Matrix<double, 5, 1> &state,
    bool drive_backwards) const {
  const double nominal_theta = spline().Theta(distance);
  const Eigen::Matrix<double, 2, 1> nominal_xy = spline().XY(distance);
  const Eigen::Matrix<double, 2, 1> xy_err =
      state.block<2, 1>(0, 0) - nominal_xy;
  const double ctheta = std::cos(nominal_theta);
  const double stheta = std::sin(nominal_theta);
  Eigen::Matrix<double, 5, 1> path_state;
  path_state(0) = distance + xy_err.x() * ctheta + xy_err.y() * stheta;
  path_state(1) = -xy_err.x() * stheta + xy_err.y() * ctheta;
  path_state(2) = aos::math::NormalizeAngle(state(2) - nominal_theta +
                                            (drive_backwards ? M_PI : 0.0));
  path_state(2) = aos::math::NormalizeAngle(state(2) - nominal_theta);
  path_state(3) = state(3);
  path_state(4) = state(4);
  if (drive_backwards) {
    std::swap(path_state(3), path_state(4));
    path_state(3) *= -1.0;
    path_state(4) *= -1.0;
  }
  return path_state;
}

// Path-relative controller method:
// For the path relative controller, we use a non-standard version of LQR to
// perform the control. Essentially, we first transform the system into
// a set of path-relative coordinates (where the reference that we use is the
// desired path reference). This gives us a system that is linear and
// time-varying, i.e. the system is a set of A_k, B_k matrices for each
// timestep k.
// In order to control this, we use a discrete-time finite-horizon LQR, using
// the appropraite [AB]_k for the given timestep. Note that the finite-horizon
// LQR requires choosing a terminal cost (i.e., what the cost should be
// for if we have not precisely reached the goal at the end of the time-period).
// For this, I approximate the infinite-horizon LQR solution by extending the
// finite-horizon much longer (albeit with the extension just using the
// linearization for the infal point).
void Trajectory::CalculatePathGains() {
  const std::vector<Eigen::Matrix<double, 3, 1>> xva_plan = PlanXVA(config_.dt);
  if (xva_plan.empty()) {
    LOG(ERROR) << "Plan is empty--unable to plan trajectory.";
    return;
  }
  plan_gains_.resize(xva_plan.size());

  // Set up reasonable gain matrices. Current choices of gains are arbitrary
  // and just setup to work well enough for the simulation tests.
  // TODO(james): Tune this on a real robot.
  // TODO(james): Pull these out into a config.
  Eigen::Matrix<double, 5, 5> Q;
  Q.setIdentity();
  Q.diagonal() << 30.0, 30.0, 20.0, 15.0, 15.0;
  Q *= 2.0;
  Q = (Q * Q).eval();

  Eigen::Matrix<double, 2, 2> R;
  R.setIdentity();
  R *= 5.0;

  Eigen::Matrix<double, 5, 5> P = Q;

  CHECK_LT(0u, xva_plan.size());
  const int max_index = static_cast<int>(xva_plan.size()) - 1;
  for (int i = max_index; i >= 0; --i) {
    const double distance = xva_plan[i](0);
    Eigen::Matrix<double, 5, 5> A_continuous;
    Eigen::Matrix<double, 5, 2> B_continuous;
    PathRelativeContinuousSystem(distance, &A_continuous, &B_continuous);
    Eigen::Matrix<double, 5, 5> A_discrete;
    Eigen::Matrix<double, 5, 2> B_discrete;
    controls::C2D(A_continuous, B_continuous, config_.dt, &A_discrete,
                  &B_discrete);

    if (i == max_index) {
      // At the final timestep, approximate P by iterating a bunch of times.
      // This is terminal cost mentioned in function-level comments.
      // This does a very loose job of solving the DARE. Ideally, we would
      // actually use a DARE solver directly, but based on some initial testing,
      // this method is a bit more robust (or, at least, it is a bit more robust
      // if we don't want to spend more time handling the potential error
      // cases the DARE solver can encounter).
      constexpr int kExtraIters = 100;
      for (int jj = 0; jj < kExtraIters; ++jj) {
        const Eigen::Matrix<double, 5, 5> AP = A_discrete.transpose() * P;
        const Eigen::Matrix<double, 5, 2> APB = AP * B_discrete;
        const Eigen::Matrix<double, 2, 2> RBPBinv =
            (R + B_discrete.transpose() * P * B_discrete).inverse();
        P = AP * A_discrete - APB * RBPBinv * APB.transpose() + Q;
      }
    }

    const Eigen::Matrix<double, 5, 5> AP = A_discrete.transpose() * P;
    const Eigen::Matrix<double, 5, 2> APB = AP * B_discrete;
    const Eigen::Matrix<double, 2, 2> RBPBinv =
        (R + B_discrete.transpose() * P * B_discrete).inverse();
    plan_gains_[i].first = distance;
    const Eigen::Matrix<double, 2, 5> K = RBPBinv * APB.transpose();
    plan_gains_[i].second = K.cast<float>();
    P = AP * A_discrete - APB * K + Q;
  }
}

Eigen::Matrix<double, 2, 5> FinishedTrajectory::GainForDistance(
    double distance) const {
  const flatbuffers::Vector<flatbuffers::Offset<fb::GainPoint>> &gains =
      *CHECK_NOTNULL(trajectory().gains());
  CHECK_LT(0u, gains.size());
  size_t index = 0;
  for (index = 0; index < gains.size() - 1; ++index) {
    if (gains[index + 1]->distance() > distance) {
      break;
    }
  }
  // ColMajor is the default storage order, but call it out explicitly here.
  return Eigen::Matrix<float, 2, 5, Eigen::ColMajor>{
      gains[index]->gains()->data()}
      .cast<double>();
}

namespace {
flatbuffers::Offset<Constraint> MakeWholeLengthConstraint(
    flatbuffers::FlatBufferBuilder *fbb, ConstraintType constraint_type,
    float value) {
  Constraint::Builder builder(*fbb);
  builder.add_constraint_type(constraint_type);
  builder.add_value(value);
  return builder.Finish();
}
}  // namespace

flatbuffers::Offset<fb::Trajectory> Trajectory::Serialize(
    flatbuffers::FlatBufferBuilder *fbb) const {
  std::array<flatbuffers::Offset<Constraint>, 3> constraints_offsets = {
      MakeWholeLengthConstraint(fbb, ConstraintType::LONGITUDINAL_ACCELERATION,
                                max_longitudinal_accel()),
      MakeWholeLengthConstraint(fbb, ConstraintType::LATERAL_ACCELERATION,
                                max_lateral_accel()),
      MakeWholeLengthConstraint(fbb, ConstraintType::VOLTAGE, max_voltage())};
  const auto constraints = fbb->CreateVector<Constraint>(
      constraints_offsets.data(), constraints_offsets.size());
  const flatbuffers::Offset<fb::DistanceSpline> spline_offset =
      spline().Serialize(fbb, constraints);

  std::vector<flatbuffers::Offset<fb::PlanPoint>> plan_points;
  for (size_t ii = 0; ii < distance_plan_size(); ++ii) {
    plan_points.push_back(fb::CreatePlanPoint(
        *fbb, Distance(ii), plan_velocity(ii), plan_constraint(ii)));
  }

  // TODO(james): What is an appropriate cap?
  CHECK_LT(plan_gains_.size(), 5000u);
  CHECK_LT(0u, plan_gains_.size());
  std::vector<flatbuffers::Offset<fb::GainPoint>> gain_points;
  const size_t matrix_size = plan_gains_[0].second.size();
  for (size_t ii = 0; ii < plan_gains_.size(); ++ii) {
    gain_points.push_back(fb::CreateGainPoint(
        *fbb, plan_gains_[ii].first,
        fbb->CreateVector(plan_gains_[ii].second.data(), matrix_size)));
  }

  return fb::CreateTrajectory(*fbb, spline_idx_, fbb->CreateVector(plan_points),
                              fbb->CreateVector(gain_points), spline_offset,
                              drive_spline_backwards_);
}

float BaseTrajectory::ConstraintValue(
    const flatbuffers::Vector<flatbuffers::Offset<Constraint>> *constraints,
    ConstraintType type) {
  if (constraints != nullptr) {
    for (const Constraint *constraint : *constraints) {
      if (constraint->constraint_type() == type) {
        return constraint->value();
      }
    }
  }
  return DefaultConstraint(type);
}

const Eigen::Matrix<double, 5, 1> BaseTrajectory::GoalState(
    double distance, double velocity) const {
  Eigen::Matrix<double, 5, 1> result;
  result.block<2, 1>(0, 0) = spline().XY(distance);
  result(2, 0) = spline().Theta(distance);

  result.block<2, 1>(3, 0) =
      config_.Tla_to_lr() * (Eigen::Matrix<double, 2, 1>() << velocity,
                             spline().DThetaDt(distance, velocity))
                                .finished();
  return result;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
