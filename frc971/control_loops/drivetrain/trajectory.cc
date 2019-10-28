#include "frc971/control_loops/drivetrain/trajectory.h"

#include <chrono>

#include "Eigen/Dense"
#include "aos/logging/matrix_logging.h"
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
      velocity_drivetrain_(
          ::std::unique_ptr<StateFeedbackLoop<2, 2, 2, double,
                                              StateFeedbackHybridPlant<2, 2, 2>,
                                              HybridKalman<2, 2, 2>>>(
              new StateFeedbackLoop<2, 2, 2, double,
                                    StateFeedbackHybridPlant<2, 2, 2>,
                                    HybridKalman<2, 2, 2>>(
                  config.make_hybrid_drivetrain_velocity_loop()))),
      robot_radius_l_(config.robot_radius),
      robot_radius_r_(config.robot_radius),
      longitudal_acceleration_(3.0),
      lateral_acceleration_(2.0),
      Tlr_to_la_((::Eigen::Matrix<double, 2, 2>() << 0.5, 0.5,
                  -1.0 / (robot_radius_l_ + robot_radius_r_),
                  1.0 / (robot_radius_l_ + robot_radius_r_))
                     .finished()),
      Tla_to_lr_(Tlr_to_la_.inverse()),
      plan_(num_distance == 0
                ? ::std::max(100, static_cast<int>(spline_->length() / 0.0025))
                : num_distance,
            vmax),
      plan_segment_type_(plan_.size() - 1, SegmentType::VELOCITY_LIMITED) {}

void Trajectory::LateralAccelPass() {
  for (size_t i = 0; i < plan_.size(); ++i) {
    const double distance = Distance(i);
    plan_[i] = ::std::min(plan_[i], LateralVelocityCurvature(distance));
  }
}

// TODO(austin): Deduplicate this potentially with the backward accel function.
// Need to sort out how the max velocity limit is going to work since the
// velocity and acceleration need to match at all points.
// TODO(austin): Accel check the wheels instead of the center of mass.
double Trajectory::ForwardAcceleration(const double x, const double v) {
  ::Eigen::Matrix<double, 2, 1> K3;
  ::Eigen::Matrix<double, 2, 1> K4;
  ::Eigen::Matrix<double, 2, 1> K5;
  K345(x, &K3, &K4, &K5);

  const ::Eigen::Matrix<double, 2, 1> C = K3 * v * v + K4 * v;
  // Now, solve for all a's and find the best one which meets our criteria.
  double maxa = -::std::numeric_limits<double>::infinity();
  for (const double a : {(voltage_limit_ - C(0, 0)) / K5(0, 0),
                         (voltage_limit_ - C(1, 0)) / K5(1, 0),
                         (-voltage_limit_ - C(0, 0)) / K5(0, 0),
                         (-voltage_limit_ - C(1, 0)) / K5(1, 0)}) {
    const ::Eigen::Matrix<double, 2, 1> U = K5 * a + K3 * v * v + K4 * v;
    if ((U.array().abs() < voltage_limit_ + 1e-6).all()) {
      maxa = ::std::max(maxa, a);
    }
  }

  // Then, assume an acceleration oval and stay inside it.
  const double lateral_acceleration = v * v * spline_->DDXY(x).norm();
  const double ellipse_down_shift = longitudal_acceleration_ * 1.0;
  const double ellipse_width_stretch = ::std::sqrt(
      1.0 / (1.0 - ::std::pow(ellipse_down_shift / (longitudal_acceleration_ +
                                                    ellipse_down_shift),
                              2.0)));
  const double squared =
      1.0 - ::std::pow(lateral_acceleration / lateral_acceleration_ /
                           ellipse_width_stretch,
                       2.0);
  // If we would end up with an imaginary number, cap us at 0 acceleration.
  // TODO(austin): Investigate when this happens, why, and fix it.
  if (squared < 0.0) {
    AOS_LOG(ERROR, "Imaginary %f, maxa: %f, fa(%f, %f) -> 0.0\n", squared, maxa,
            x, v);
    return 0.0;
  }
  const double longitudal_acceleration =
      ::std::sqrt(::std::abs(squared)) *
          (longitudal_acceleration_ + ellipse_down_shift) -
      ellipse_down_shift;
  return ::std::min(longitudal_acceleration, maxa);
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

    if (new_plan_velocity < plan_[i + 1]) {
      plan_[i + 1] = new_plan_velocity;
      plan_segment_type_[i] = SegmentType::ACCELERATION_LIMITED;
    }
  }
}

double Trajectory::BackwardAcceleration(double x, double v) {
  ::Eigen::Matrix<double, 2, 1> K3;
  ::Eigen::Matrix<double, 2, 1> K4;
  ::Eigen::Matrix<double, 2, 1> K5;
  K345(x, &K3, &K4, &K5);

  // Now, solve for all a's and find the best one which meets our criteria.
  const ::Eigen::Matrix<double, 2, 1> C = K3 * v * v + K4 * v;
  double mina = ::std::numeric_limits<double>::infinity();
  for (const double a : {(voltage_limit_ - C(0, 0)) / K5(0, 0),
                         (voltage_limit_ - C(1, 0)) / K5(1, 0),
                         (-voltage_limit_ - C(0, 0)) / K5(0, 0),
                         (-voltage_limit_ - C(1, 0)) / K5(1, 0)}) {
    const ::Eigen::Matrix<double, 2, 1> U = K5 * a + K3 * v * v + K4 * v;
    if ((U.array().abs() < voltage_limit_ + 1e-6).all()) {
      mina = ::std::min(mina, a);
    }
  }

  // Then, assume an acceleration oval and stay inside it.
  const double lateral_acceleration = v * v * spline_->DDXY(x).norm();
  const double ellipse_down_shift = longitudal_acceleration_ * 1.0;
  const double ellipse_width_stretch = ::std::sqrt(
      1.0 / (1.0 - ::std::pow(ellipse_down_shift / (longitudal_acceleration_ +
                                                    ellipse_down_shift),
                              2.0)));
  const double squared =
      1.0 - ::std::pow(lateral_acceleration / lateral_acceleration_ /
                           ellipse_width_stretch,
                       2.0);
  // If we would end up with an imaginary number, cap us at 0 acceleration.
  // TODO(austin): Investigate when this happens, why, and fix it.
  if (squared < 0.0) {
    AOS_LOG(ERROR, "Imaginary %f, mina: %f, fa(%f, %f) -> 0.0\n", squared, mina,
            x, v);
    return 0.0;
  }
  const double longitudal_acceleration =
      -::std::sqrt(::std::abs(squared)) *
          (longitudal_acceleration_ + ellipse_down_shift) +
      ellipse_down_shift;
  return ::std::max(longitudal_acceleration, mina);
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

    if (new_plan_velocity < plan_[i - 1]) {
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
      acceleration = 0.0;
      break;
    case SegmentType::ACCELERATION_LIMITED:
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

  if (vcurvature < velocity) {
    velocity = vcurvature;
    acceleration = 0.0;
    AOS_LOG(ERROR, "Curvature won\n");
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
  if (info == 0) {
    AOS_LOG_MATRIX(INFO, "K", K);
  } else {
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
