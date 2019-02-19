#include "frc971/control_loops/drivetrain/trajectory.h"

#include <chrono>

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

Trajectory::Trajectory(const DistanceSpline *spline,
                       const DrivetrainConfig<double> &config, double vmax,
                       int num_distance)
    : spline_(spline),
      velocity_drivetrain_(::std::unique_ptr<
          StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                            HybridKalman<2, 2, 2>>>(
          new StateFeedbackLoop<2, 2, 2, double,
                                StateFeedbackHybridPlant<2, 2, 2>,
                                HybridKalman<2, 2, 2>>(
              config.make_hybrid_drivetrain_velocity_loop()))),
      robot_radius_l_(config.robot_radius),
      robot_radius_r_(config.robot_radius),
      longitudinal_acceleration_(3.0),
      lateral_acceleration_(2.0),
      Tlr_to_la_((::Eigen::Matrix<double, 2, 2>() << 0.5, 0.5,
                  -1.0 / (robot_radius_l_ + robot_radius_r_),
                  1.0 / (robot_radius_l_ + robot_radius_r_)).finished()),
      Tla_to_lr_(Tlr_to_la_.inverse()),
      plan_(num_distance == 0
                ? ::std::max(100, static_cast<int>(spline_->length() / 0.0025))
                : num_distance,
            vmax),
      plan_segment_type_(plan_.size() - 1, SegmentType::VELOCITY_LIMITED) {}

void Trajectory::LateralAccelPass() {
  for (size_t i = 0; i < plan_.size(); ++i) {
    const double distance = Distance(i);
    const double velocity_limit =  LateralVelocityCurvature(distance);
    if (velocity_limit < plan_[i]) {
      plan_[i] = velocity_limit;
      plan_segment_type_[i] = CURVATURE_LIMITED;
    }
  }
}

void Trajectory::VoltageFeasibilityPass(VoltageLimit limit_type) {
  for (size_t i = 0; i < plan_.size(); ++i) {
    const double distance = Distance(i);
    const double velocity_limit = VoltageVelocityLimit(distance, limit_type);
    if (velocity_limit < plan_[i]) {
      plan_[i] = velocity_limit;
      plan_segment_type_[i] = VOLTAGE_LIMITED;
    }
  }
}

double Trajectory::BestAcceleration(double x, double v, bool backwards) {
  ::Eigen::Matrix<double, 2, 1> K3;
  ::Eigen::Matrix<double, 2, 1> K4;
  ::Eigen::Matrix<double, 2, 1> K5;
  K345(x, &K3, &K4, &K5);

  // Now, solve for all a's and find the best one which meets our criteria.
  const ::Eigen::Matrix<double, 2, 1> C = K3 * v * v + K4 * v;
  double min_voltage_accel = ::std::numeric_limits<double>::infinity();
  double max_voltage_accel = -min_voltage_accel;
  for (const double a : {(voltage_limit_ - C(0, 0)) / K5(0, 0),
                         (voltage_limit_ - C(1, 0)) / K5(1, 0),
                         (-voltage_limit_ - C(0, 0)) / K5(0, 0),
                         (-voltage_limit_ - C(1, 0)) / K5(1, 0)}) {
    const ::Eigen::Matrix<double, 2, 1> U = K5 * a + K3 * v * v + K4 * v;
    if ((U.array().abs() < voltage_limit_ + 1e-6).all()) {
      min_voltage_accel = ::std::min(a, min_voltage_accel);
      max_voltage_accel = ::std::max(a, max_voltage_accel);
    }
  }
  double best_accel = backwards ? min_voltage_accel : max_voltage_accel;

  double min_friction_accel, max_friction_accel;
  FrictionLngAccelLimits(x, v, &min_friction_accel, &max_friction_accel);
  if (backwards) {
    best_accel = ::std::max(best_accel, min_friction_accel);
  } else {
    best_accel = ::std::min(best_accel, max_friction_accel);
  }

  if (max_friction_accel < min_friction_accel - 0.1 ||
      best_accel < min_voltage_accel || best_accel > max_voltage_accel) {
    AOS_LOG(WARNING,
        "Viable friction limits and viable voltage limits do not overlap (x: "
        "%f, v: %f, backwards: %d) best_accel = %f, min voltage %f, max "
        "voltage %f min friction %f max friction %f.\n",
        x, v, backwards, best_accel, min_voltage_accel, max_voltage_accel,
        min_friction_accel, max_friction_accel);
    // Don't actually do anything--this will just result in attempting to drive
    // higher voltages thatn we have available. In practice, that'll probably
    // work out fine.
  }

  return best_accel;
}

double Trajectory::LateralVelocityCurvature(double distance) const {
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
  const double dtheta = spline_->DTheta(distance);
  const ::Eigen::Matrix<double, 1, 2> K2prime =
      K2(dtheta).transpose() *
      (::Eigen::Matrix<double, 2, 2>() << 0, 1, -1, 0).finished();
  // Calculate whether the wheels are spinning in opposite directions.
  const bool opposites = K2prime(0) * K2prime(1) < 0;
  const ::Eigen::Matrix<double, 2, 1> K1calc = K1(spline_->DDTheta(distance));
  const double lat_accel_squared =
      ::std::pow(dtheta / lateral_acceleration_, 2);
  const double curvature_change_term =
      (K2prime * K1calc).value() /
      (K2prime *
       (::Eigen::Matrix<double, 2, 1>() << 1.0, (opposites ? -1.0 : 1.0))
           .finished() *
       longitudinal_acceleration_)
          .value();
  const double vel_inv = ::std::sqrt(
      ::std::sqrt(::std::pow(curvature_change_term, 2) + lat_accel_squared));
  if (vel_inv == 0.0) {
    return ::std::numeric_limits<double>::infinity();
  }
  return 1.0 / vel_inv;
}

void Trajectory::FrictionLngAccelLimits(double x, double v, double *min_accel,
                                        double *max_accel) const {
  // First, calculate the max longitudinal acceleration that can be achieved
  // by either wheel given the friction elliipse that we have.
  const double lateral_acceleration = v * v * spline_->DDXY(x).norm();
  const double max_wheel_lng_accel_squared =
      1.0 - ::std::pow(lateral_acceleration / lateral_acceleration_, 2.0);
  if (max_wheel_lng_accel_squared < 0.0) {
    AOS_LOG(DEBUG,
        "Something (probably Runge-Kutta) queried invalid velocity %f at "
        "distance %f\n",
        v, x);
    // If we encounter this, it means that the Runge-Kutta has attempted to
    // sample points a bit past the edge of the friction boundary. If so, we
    // gradually ramp the min/max accels to be more and more incorrect (note
    // how min_accel > max_accel if we reach this case) to avoid causing any
    // numerical issues.
    *min_accel =
        ::std::sqrt(-max_wheel_lng_accel_squared) * longitudinal_acceleration_;
    *max_accel = -*min_accel;
    return;
  }
  *min_accel = -::std::numeric_limits<double>::infinity();
  *max_accel = ::std::numeric_limits<double>::infinity();

  // Calculate max/min accelerations by calculating what the robots overall
  // longitudinal acceleration would be if each wheel were running at the max
  // forwards/backwards longitudinal acceleration.
  const double max_wheel_lng_accel =
      longitudinal_acceleration_ * ::std::sqrt(max_wheel_lng_accel_squared);
  const ::Eigen::Matrix<double, 2, 1> K1v2 = K1(spline_->DDTheta(x)) * v * v;
  const ::Eigen::Matrix<double, 2, 1> K2inv =
      K2(spline_->DTheta(x)).cwiseInverse();
  // Store the accelerations of the robot corresponding to each wheel being at
  // the max/min acceleration. The first coefficient in each vector
  // corresponds to the left wheel, the second to the right wheel.
  const ::Eigen::Matrix<double, 2, 1> accels1 =
      K2inv.array() * (-K1v2.array() + max_wheel_lng_accel);
  const ::Eigen::Matrix<double, 2, 1> accels2 =
      K2inv.array() * (-K1v2.array() - max_wheel_lng_accel);

  // If either term is non-finite, that suggests that a term of K2 is zero
  // (which is physically possible when turning such that one wheel is
  // stationary), so just ignore that side of the drivetrain.
  if (::std::isfinite(accels1(0))) {
    // The inner max/min in this case determines which of the two cases (+ or
    // - acceleration on the left wheel) we care about--in a sufficiently
    // tight turning radius, the left hweel may be accelerating backwards when
    // the robot as a whole accelerates forwards. We then use that
    // acceleration to bound the min/max accel.
    *min_accel = ::std::max(*min_accel, ::std::min(accels1(0), accels2(0)));
    *max_accel = ::std::min(*max_accel, ::std::max(accels1(0), accels2(0)));
  }
  // Same logic as previous if-statement, but for the right wheel.
  if (::std::isfinite(accels1(1))) {
    *min_accel = ::std::max(*min_accel, ::std::min(accels1(1), accels2(1)));
    *max_accel = ::std::min(*max_accel, ::std::max(accels1(1), accels2(1)));
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
  const ::Eigen::Matrix<double, 2, 2> B =
      velocity_drivetrain_->plant().coefficients().B_continuous;
  const double dtheta = spline_->DTheta(distance);
  const ::Eigen::Matrix<double, 2, 1> BinvK2 = B.inverse() * K2(dtheta);
  // Because voltages can actually impact *both* wheels, in order to determine
  // whether the voltages will have opposite signs, we need to use B^-1 * K2.
  const bool opposite_voltages = BinvK2(0) * BinvK2(1) > 0.0;
  const ::Eigen::Matrix<double, 1, 2> K2prime =
      K2(dtheta).transpose() *
      (::Eigen::Matrix<double, 2, 2>() << 0, 1, -1, 0).finished();
  const double a = K2prime * K1(spline_->DDTheta(distance));
  const double b = -K2prime *
                   velocity_drivetrain_->plant().coefficients().A_continuous *
                   K2(dtheta);
  const ::Eigen::Matrix<double, 1, 2> c_coeff = -K2prime * B;
  // Calculate the "positive" version of the voltage limits we will use.
  const ::Eigen::Matrix<double, 2, 1> abs_volts =
      voltage_limit_ *
      (::Eigen::Matrix<double, 2, 1>() << 1.0, (opposite_voltages ? -1.0 : 1.0))
          .finished();

  double min_valid_vel = ::std::numeric_limits<double>::infinity();
  if (limit_type == VoltageLimit::kAggressive) {
    min_valid_vel = 0.0;
  }
  // Iterate over both possibilites for +/- voltage, and solve the quadratic
  // formula. For every positive solution, adjust the velocity limit
  // appropriately.
  for (const double sign : {1.0, -1.0}) {
    const ::Eigen::Matrix<double, 2, 1> U = sign * abs_volts;
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
          min_valid_vel = ::std::min(min_valid_vel, vel);
        } else {
          min_valid_vel = ::std::max(min_valid_vel, vel);
        }
      } else if (b == 0) {
        // If a and b are zero, then we are travelling in a straight line and
        // have no voltage-based velocity constraints.
        min_valid_vel = ::std::numeric_limits<double>::infinity();
      }
    } else if (determinant > 0) {
      const double sqrt_determinant = ::std::sqrt(determinant);
      const double high_vel = (-b + sqrt_determinant) / (2.0 * a);
      const double low_vel = (-b - sqrt_determinant) / (2.0 * a);
      if (low_vel > 0) {
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = ::std::min(min_valid_vel, low_vel);
        } else {
          min_valid_vel = ::std::max(min_valid_vel, low_vel);
        }
      }
      if (high_vel > 0) {
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = ::std::min(min_valid_vel, high_vel);
        } else {
          min_valid_vel = ::std::max(min_valid_vel, high_vel);
        }
      }
    } else if (determinant == 0 && -b * a > 0) {
      const double vel = -b / (2.0 * a);
      if (vel > 0.0) {
        if (limit_type == VoltageLimit::kConservative) {
          min_valid_vel = ::std::min(min_valid_vel, vel);
        } else {
          min_valid_vel = ::std::max(min_valid_vel, vel);
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
      plan_segment_type_[i] = SegmentType::ACCELERATION_LIMITED;
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
      plan_segment_type_[i - 1] = SegmentType::DECELERATION_LIMITED;
    }
  }
}

::Eigen::Matrix<double, 3, 1> Trajectory::FFAcceleration(double distance) {
  if (distance < 0.0) {
    // Make sure we don't end up off the beginning of the curve.
    distance = 0.0;
  } else if (distance > length()) {
    // Make sure we don't end up off the end of the curve.
    distance = length();
  }
  const size_t before_index = DistanceToSegment(distance);
  const size_t after_index = before_index + 1;

  const double before_distance = Distance(before_index);
  const double after_distance = Distance(after_index);

  // And then also make sure we aren't curvature limited.
  const double vcurvature = LateralVelocityCurvature(distance);

  double acceleration;
  double velocity;
  // TODO(james): While technically correct for sufficiently small segment
  // steps, this method of switching between limits has a tendency to produce
  // sudden jumps in acceelrations, which is undesirable.
  switch (plan_segment_type_[DistanceToSegment(distance)]) {
    case SegmentType::VELOCITY_LIMITED:
      acceleration = 0.0;
      velocity = (plan_[before_index] + plan_[after_index]) / 2.0;
      // TODO(austin): Accelerate or decelerate until we hit the limit in the
      // time slice.  Otherwise our acceleration will be lying for this slice.
      // Do note, we've got small slices so the effect will be small.
      break;
    case SegmentType::CURVATURE_LIMITED:
      velocity = vcurvature;
      FrictionLngAccelLimits(distance, velocity, &acceleration, &acceleration);
      break;
    case SegmentType::VOLTAGE_LIMITED:
      // Normally, we expect that voltage limited plans will all get dominated
      // by the acceleration/deceleration limits. This may not always be true;
      // if we ever encounter this error, we just need to back out what the
      // accelerations would be in this case.
      LOG(FATAL) <<  "Unexpectedly got VOLTAGE_LIMITED plan.";
      break;
    case SegmentType::ACCELERATION_LIMITED:
      // TODO(james): The integration done here and in the DECELERATION_LIMITED
      // can technically cause us to violate friction constraints. We currently
      // don't do anything about it to avoid causing sudden jumps in voltage,
      // but we probably *should* at some point.
      velocity = IntegrateAccelForDistance(
          [this](double x, double v) { return ForwardAcceleration(x, v); },
          plan_[before_index], before_distance, distance - before_distance);
      acceleration = ForwardAcceleration(distance, velocity);
      break;
    case SegmentType::DECELERATION_LIMITED:
      velocity = IntegrateAccelForDistance(
          [this](double x, double v) { return BackwardAcceleration(x, v); },
          plan_[after_index], after_distance, distance - after_distance);
      acceleration = BackwardAcceleration(distance, velocity);
      break;
    default:
      AOS_LOG(
          FATAL, "Unknown segment type %d\n",
          static_cast<int>(plan_segment_type_[DistanceToSegment(distance)]));
      break;
  }

  return (::Eigen::Matrix<double, 3, 1>() << distance, velocity, acceleration)
      .finished();
}

::Eigen::Matrix<double, 2, 1> Trajectory::FFVoltage(double distance) {
  const Eigen::Matrix<double, 3, 1> xva = FFAcceleration(distance);
  const double velocity = xva(1);
  const double acceleration = xva(2);

  ::Eigen::Matrix<double, 2, 1> K3;
  ::Eigen::Matrix<double, 2, 1> K4;
  ::Eigen::Matrix<double, 2, 1> K5;
  K345(distance, &K3, &K4, &K5);

  return K5 * acceleration + K3 * velocity * velocity + K4 * velocity;
}

const ::std::vector<double> Trajectory::Distances() const {
  ::std::vector<double> d;
  d.reserve(plan_.size());
  for (size_t i = 0; i < plan_.size(); ++i) {
    d.push_back(Distance(i));
  }
  return d;
}

::Eigen::Matrix<double, 5, 5> Trajectory::ALinearizedContinuous(
    const ::Eigen::Matrix<double, 5, 1> &state) const {
  const double sintheta = ::std::sin(state(2));
  const double costheta = ::std::cos(state(2));
  const ::Eigen::Matrix<double, 2, 1> linear_angular =
      Tlr_to_la_ * state.block<2, 1>(3, 0);

  // When stopped, just roll with a min velocity.
  double linear_velocity = 0.0;
  constexpr double kMinVelocity = 0.1;
  if (::std::abs(linear_angular(0)) < kMinVelocity / 100.0) {
    linear_velocity = 0.1;
  } else if (::std::abs(linear_angular(0)) > kMinVelocity) {
    linear_velocity = linear_angular(0);
  } else if (linear_angular(0) > 0) {
    linear_velocity = kMinVelocity;
  } else if (linear_angular(0) < 0) {
    linear_velocity = -kMinVelocity;
  }

  ::Eigen::Matrix<double, 5, 5> result = ::Eigen::Matrix<double, 5, 5>::Zero();
  result(0, 2) = -sintheta * linear_velocity;
  result(0, 3) = 0.5 * costheta;
  result(0, 4) = 0.5 * costheta;

  result(1, 2) = costheta * linear_velocity;
  result(1, 3) = 0.5 * sintheta;
  result(1, 4) = 0.5 * sintheta;

  result(2, 3) = Tlr_to_la_(1, 0);
  result(2, 4) = Tlr_to_la_(1, 1);

  result.block<2, 2>(3, 3) =
      velocity_drivetrain_->plant().coefficients().A_continuous;
  return result;
}

::Eigen::Matrix<double, 5, 2> Trajectory::BLinearizedContinuous() const {
  ::Eigen::Matrix<double, 5, 2> result = ::Eigen::Matrix<double, 5, 2>::Zero();
  result.block<2, 2>(3, 0) =
      velocity_drivetrain_->plant().coefficients().B_continuous;
  return result;
}

void Trajectory::AB(const ::Eigen::Matrix<double, 5, 1> &state,
                    ::std::chrono::nanoseconds dt,
                    ::Eigen::Matrix<double, 5, 5> *A,
                    ::Eigen::Matrix<double, 5, 2> *B) const {
  ::Eigen::Matrix<double, 5, 5> A_linearized_continuous =
      ALinearizedContinuous(state);
  ::Eigen::Matrix<double, 5, 2> B_linearized_continuous =
      BLinearizedContinuous();

  // Now, convert it to discrete.
  controls::C2D(A_linearized_continuous, B_linearized_continuous, dt, A, B);
}

::Eigen::Matrix<double, 2, 5> Trajectory::KForState(
    const ::Eigen::Matrix<double, 5, 1> &state, ::std::chrono::nanoseconds dt,
    const ::Eigen::DiagonalMatrix<double, 5> &Q,
    const ::Eigen::DiagonalMatrix<double, 2> &R) const {
  ::Eigen::Matrix<double, 5, 5> A;
  ::Eigen::Matrix<double, 5, 2> B;
  AB(state, dt, &A, &B);

  ::Eigen::Matrix<double, 5, 5> S = ::Eigen::Matrix<double, 5, 5>::Zero();
  ::Eigen::Matrix<double, 2, 5> K = ::Eigen::Matrix<double, 2, 5>::Zero();

  int info = ::frc971::controls::dlqr<5, 2>(A, B, Q, R, &K, &S);
  if (info != 0) {
    AOS_LOG(ERROR, "Failed to solve %d, controllability: %d\n", info,
            controls::Controllability(A, B));
    // TODO(austin): Can we be more clever here?  Use the last one?  We should
    // collect more info about when this breaks down from logs.
    K = ::Eigen::Matrix<double, 2, 5>::Zero();
  }
  ::Eigen::EigenSolver<::Eigen::Matrix<double, 5, 5>> eigensolver(A - B * K);
  const auto eigenvalues = eigensolver.eigenvalues();
  AOS_LOG(DEBUG,
          "Eigenvalues: (%f + %fj), (%f + %fj), (%f + %fj), (%f + %fj), (%f + "
          "%fj)\n",
          eigenvalues(0).real(), eigenvalues(0).imag(), eigenvalues(1).real(),
          eigenvalues(1).imag(), eigenvalues(2).real(), eigenvalues(2).imag(),
          eigenvalues(3).real(), eigenvalues(3).imag(), eigenvalues(4).real(),
          eigenvalues(4).imag());
  return K;
}

const ::Eigen::Matrix<double, 5, 1> Trajectory::GoalState(double distance,
                                                          double velocity) {
  ::Eigen::Matrix<double, 5, 1> result;
  result.block<2, 1>(0, 0) = spline_->XY(distance);
  result(2, 0) = spline_->Theta(distance);

  result.block<2, 1>(3, 0) =
      Tla_to_lr_ * (::Eigen::Matrix<double, 2, 1>() << velocity,
                    spline_->DThetaDt(distance, velocity))
                       .finished();
  return result;
}

::Eigen::Matrix<double, 3, 1> Trajectory::GetNextXVA(
    ::std::chrono::nanoseconds dt, ::Eigen::Matrix<double, 2, 1> *state) {
  double dt_float = ::aos::time::DurationInSeconds(dt);

  // TODO(austin): This feels like something that should be pulled out into
  // a library for re-use.
  *state = RungeKutta(
      [this](const ::Eigen::Matrix<double, 2, 1> x) {
        ::Eigen::Matrix<double, 3, 1> xva = FFAcceleration(x(0));
        return (::Eigen::Matrix<double, 2, 1>() << x(1), xva(2)).finished();
      },
      *state, dt_float);

  ::Eigen::Matrix<double, 3, 1> result = FFAcceleration((*state)(0));
  (*state)(1) = result(1);
  return result;
}

::std::vector<::Eigen::Matrix<double, 3, 1>> Trajectory::PlanXVA(
    ::std::chrono::nanoseconds dt) {
  ::Eigen::Matrix<double, 2, 1> state = ::Eigen::Matrix<double, 2, 1>::Zero();
  ::std::vector<::Eigen::Matrix<double, 3, 1>> result;
  result.emplace_back(FFAcceleration(0));
  result.back()(1) = 0.0;

  while (!is_at_end(state)) {
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
  starting_distance = ::std::min(length(), ::std::max(0.0, starting_distance));
  ending_distance = ::std::min(length(), ::std::max(0.0, ending_distance));
  if (segment_length < min_length) {
    const size_t plan_index = static_cast<size_t>(
        ::std::round((starting_distance + ending_distance) / 2.0 / min_length));
    if (max_velocity < plan_[plan_index]) {
      plan_[plan_index] = max_velocity;
    }
  } else {
    for (size_t i = DistanceToSegment(starting_distance) + 1;
         i < DistanceToSegment(ending_distance) + 1; ++i) {
      if (max_velocity < plan_[i]) {
        plan_[i] = max_velocity;
        if (i < DistanceToSegment(ending_distance)) {
          plan_segment_type_[i] = SegmentType::VELOCITY_LIMITED;
        }
      }
    }
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
