#include "frc971/control_loops/drivetrain/line_follow_drivetrain.h"

#include <chrono>

#include "aos/commonmath.h"
#include "aos/time/time.h"
#include "aos/util/math.h"
#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/dlqr.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace {
namespace chrono = ::std::chrono;
}  // namespace

constexpr double LineFollowDrivetrain::kMaxVoltage;

namespace {
::Eigen::Matrix<double, 3, 3> AContinuous(
    const DrivetrainConfig<double> &dt_config) {
  ::Eigen::Matrix<double, 3, 3> result;
  result.setZero();
  result(0, 2) = 1.0;
  result.block<2, 2>(1, 1) = dt_config.Tlr_to_la() *
                             dt_config.make_hybrid_drivetrain_velocity_loop()
                                 .plant()
                                 .coefficients()
                                 .A_continuous *
                             dt_config.Tla_to_lr();
  return result;
}
::Eigen::Matrix<double, 3, 2> BContinuous(
    const DrivetrainConfig<double> &dt_config) {
  ::Eigen::Matrix<double, 3, 2> result;
  result.setZero();
  result.block<2, 2>(1, 0) =
      dt_config.Tlr_to_la() * dt_config.make_hybrid_drivetrain_velocity_loop()
                                  .plant()
                                  .coefficients()
                                  .B_continuous;
  return result;
}
void AB(const DrivetrainConfig<double> &dt_config,
        ::Eigen::Matrix<double, 3, 3> *A, ::Eigen::Matrix<double, 3, 2> *B) {
  controls::C2D(AContinuous(dt_config), BContinuous(dt_config), dt_config.dt, A,
                B);
}

::Eigen::Matrix<double, 3, 3> ADiscrete(
    const DrivetrainConfig<double> &dt_config) {
  ::Eigen::Matrix<double, 3, 3> ADiscrete;
  ::Eigen::Matrix<double, 3, 2> BDiscrete;
  AB(dt_config, &ADiscrete, &BDiscrete);
  return ADiscrete;
}

::Eigen::Matrix<double, 3, 2> BDiscrete(
    const DrivetrainConfig<double> &dt_config) {
  ::Eigen::Matrix<double, 3, 3> ADiscrete;
  ::Eigen::Matrix<double, 3, 2> BDiscrete;
  AB(dt_config, &ADiscrete, &BDiscrete);
  return BDiscrete;
}

::Eigen::Matrix<double, 2, 3> CalcK(const ::Eigen::Matrix<double, 3, 3> &A,
                                    const ::Eigen::Matrix<double, 3, 2> &B,
                                    const ::Eigen::Matrix<double, 3, 3> &Q,
                                    const ::Eigen::Matrix<double, 2, 2> &R) {
  Eigen::Matrix<double, 2, 3> K;
  Eigen::Matrix<double, 3, 3> S;
  int info = ::frc971::controls::dlqr<3, 2>(A, B, Q, R, &K, &S);
  if (info != 0) {
    // We allow a FATAL error here since this should only be called during
    // initialization.
    AOS_LOG(FATAL, "Failed to solve %d, controllability: %d\n", info,
            controls::Controllability(A, B));
  }
  return K;
}

::Eigen::Matrix<double, 2, 3> CalcKff(const ::Eigen::Matrix<double, 3, 2> &B) {
  ::Eigen::Matrix<double, 2, 3> Kff;
  Kff.setZero();
  Kff.block<2, 2>(0, 1) = B.block<2, 2>(1, 0).inverse();
  return Kff;
}

double VelocitySignForSide(TargetSelectorInterface::Side side,
                           double goal_velocity) {
  switch (side) {
    case TargetSelectorInterface::Side::FRONT:
      return 1.0;
    case TargetSelectorInterface::Side::BACK:
      return -1.0;
    case TargetSelectorInterface::Side::DONT_CARE:
      return goal_velocity >= 0.0 ? 1.0 : -1.0;
  }
  return 1.0;
}

}  // namespace

// When we create A/B, we do recompute A/B, but we don't really care about
// optimizing it since it is only happening at initialization time...
LineFollowDrivetrain::LineFollowDrivetrain(
    const DrivetrainConfig<double> &dt_config,
    TargetSelectorInterface *target_selector)
    : dt_config_(dt_config),
      Q_(dt_config_.line_follow_config.Q),
      R_(dt_config_.line_follow_config.R),
      A_d_(ADiscrete(dt_config_)),
      B_d_(BDiscrete(dt_config_)),
      K_(CalcK(A_d_, B_d_, Q_, R_)),
      Kff_(CalcKff(B_d_)),
      target_selector_(target_selector),
      U_(0, 0) {}

void LineFollowDrivetrain::SetGoal(
    ::aos::monotonic_clock::time_point now,
    const ::frc971::control_loops::drivetrain::Goal *goal) {
  // TODO(james): More properly calculate goal velocity from throttle.
  goal_velocity_ = 4.0 * goal->throttle();
  // The amount of time to freeze the target choice after the driver releases
  // the button. Depending on the current state, we vary how long this timeout
  // is so that we can avoid button glitches causing issues.
  chrono::nanoseconds freeze_delay = chrono::seconds(0);
  // Freeze the target once the driver presses the button; if we haven't yet
  // confirmed a target when the driver presses the button, we will not do
  // anything and report not ready until we have a target.
  if (goal->controller_type() == drivetrain::ControllerType::LINE_FOLLOWER) {
    last_enable_ = now;
    // If we already acquired a target, we want to keep track if it.
    if (have_target_) {
      freeze_delay = chrono::milliseconds(500);
      // If we've had the target acquired for a while, the driver is probably
      // happy with the current target selection, so we really want to keep it.
      if (now > chrono::milliseconds(1000) + start_of_target_acquire_) {
        freeze_delay = chrono::milliseconds(2000);
      }
    }
    freeze_target_ = true;
  } else {
    freeze_target_ = now <= freeze_delay + last_enable_;
  }
  // Set an adjustment that lets the driver tweak the offset for where we place
  // the target left/right.
  side_adjust_ =
      -goal->wheel() * dt_config_.line_follow_config.max_controllable_offset;
}

bool LineFollowDrivetrain::SetOutput(
    ::frc971::control_loops::drivetrain::OutputT *output) {
  // TODO(james): Account for voltage error terms, and/or provide driver with
  // ability to influence steering.
  if (output != nullptr && have_target_) {
    output->left_voltage = U_(0, 0);
    output->right_voltage = U_(1, 0);
  }
  return have_target_;
}

double LineFollowDrivetrain::GoalTheta(
    const ::Eigen::Matrix<double, 5, 1> &abs_state, double relative_y_offset,
    double velocity_sign) {
  // Calculates the goal angle for the drivetrain given our position.
  // Note: The goal angle is in target-relative coordinates, since our entire
  // control loop is written relative to the target.
  // The calculated goal will be such that a point piece_rad to one side of the
  // drivetrain (the side depends on where we approach from and SignedRadii())
  // will end up hitting the plane of the target exactly target_rad from the
  // center of the target. This allows us to better approach targets in the 2019
  // game from an angle--radii of zero imply driving straight in.
  const double target_rad = target_selector_->TargetRadius();
  const double piece_rad = target_selector_->GamePieceRadius();
  // Depending on whether we are to the right or left of the target, we work off
  // of a different side of the robot.
  const double edge_sign = target_selector_->SignedRadii()
                               ? 1.0
                               : (relative_y_offset > 0 ? 1.0 : -1.0);
  // Note side_adjust which is the input from the driver's wheel to allow
  // shifting the goal target left/right.
  const double edge_offset = edge_sign * target_rad - side_adjust_;
  // The point that we are trying to get the disc to hit.
  const Pose corner = Pose(&target_pose_, {0.0, edge_offset, 0.0}, 0.0);
  // A pose for the current robot position that is square to the target.
  Pose square_robot =
      Pose({abs_state.x(), abs_state.y(), 0.0}, 0.0).Rebase(&corner);
  // To prevent numerical issues, we limit x so that when the localizer isn't
  // working properly and ends up driving past the target, we still get sane
  // results.
  // The min() with -piece_rad ensures that we past well-conditioned numbers
  // to acos() (we must have piece_rad <= dist_to_corner); the min with -0.01
  // ensures that dist_to_corner doesn't become near zero.
  square_robot.mutable_pos()->x() = ::std::min(
      ::std::min(square_robot.mutable_pos()->x(), -std::abs(piece_rad)), -0.01);
  // Distance from the edge of the disc on the robot to the velcro we want to
  // hit on the target.
  const double dist_to_corner = square_robot.xy_norm();
  // The following actually handles calculating the heading we need the robot to
  // take (relative to the plane of the target).
  const double alpha = ::std::acos(piece_rad / dist_to_corner);
  const double heading_to_robot = edge_sign * square_robot.heading();
  double theta = -edge_sign * (M_PI - alpha - (heading_to_robot - M_PI_2));
  if (velocity_sign < 0) {
    theta = ::aos::math::NormalizeAngle(theta + M_PI);
  }
  return theta;
}

void LineFollowDrivetrain::Update(
    ::aos::monotonic_clock::time_point now,
    const ::Eigen::Matrix<double, 5, 1> &abs_state) {
  // Because we assume the target selector may have some internal state (e.g.,
  // not confirming a target until some time as passed), we should call
  // UpdateSelection every time.
  bool new_target =
      target_selector_->UpdateSelection(abs_state, goal_velocity_);
  if (freeze_target_ && !target_selector_->ForceReselectTarget()) {
    // When freezing the target, only make changes if we didn't have a good
    // target before.
    if (!have_target_ && new_target) {
      have_target_ = true;
      start_of_target_acquire_ = now;
      velocity_sign_ = VelocitySignForSide(target_selector_->DriveDirection(),
                                           goal_velocity_);
      target_pose_ = target_selector_->TargetPose();
    }
  } else {
    // If the target selector has lost its target, we *do* want to set
    // have_target_ to false, so long as we aren't trying to freeze the target.
    have_target_ = new_target;
    if (have_target_) {
      target_pose_ = target_selector_->TargetPose();
      velocity_sign_ = VelocitySignForSide(target_selector_->DriveDirection(),
                                           goal_velocity_);
    }
  }

  // Get the robot pose in the target coordinate frame.
  relative_pose_ = Pose({abs_state.x(), abs_state.y(), 0.0}, abs_state(2, 0))
                       .Rebase(&target_pose_);
  double goal_theta =
      GoalTheta(abs_state, relative_pose_.rel_pos().y(), velocity_sign_);

  // Always force a slight negative X, so that the driver can continue to drive
  // past zero if they want.
  // The "slight negative" needs to be large enough that we won't force
  // obnoxiously sharp turning radii if we run past the end of the
  // line--because, in practice, the driver should never be trying to drive to a
  // target where they are significantly off laterally at <0.1m from the target,
  // this should not be a problem.
  const double relative_x = ::std::min(relative_pose_.rel_pos().x(), -0.1);
  const ::Eigen::Matrix<double, 5, 1> rel_state =
      (::Eigen::Matrix<double, 5, 1>() << relative_x,
       relative_pose_.rel_pos().y(), relative_pose_.rel_theta(),
       abs_state(3, 0), abs_state(4, 0))
          .finished();
  if (velocity_sign_ * goal_velocity_ < 0) {
    goal_theta = rel_state(2, 0);
  }
  controls_goal_ << goal_theta, goal_velocity_, 0.0;
  ::Eigen::Matrix<double, 3, 1> controls_state;
  controls_state(0, 0) = rel_state(2, 0);
  controls_state.block<2, 1>(1, 0) =
      dt_config_.Tlr_to_la() * rel_state.block<2, 1>(3, 0);
  ::Eigen::Matrix<double, 3, 1> controls_err = controls_goal_ - controls_state;
  // Because we are taking the difference of an angle, normaliez to [-pi, pi].
  controls_err(0, 0) = ::aos::math::NormalizeAngle(controls_err(0, 0));
  // TODO(james): Calculate the next goal so that we are properly doing
  // feed-forwards.
  ::Eigen::Matrix<double, 2, 1> U_ff =
      Kff_ * (controls_goal_ - A_d_ * controls_goal_);
  U_ = K_ * controls_err + U_ff;
  const double maxU = ::std::max(U_(0, 0), U_(1, 0));
  const double minU = ::std::min(U_(0, 0), U_(1, 0));
  const double diffU = ::std::abs(U_(1, 0) - U_(0, 0));
  const double maxAbsU = ::std::max(maxU, -minU);
  if (diffU > 24.0) {
    U_ *= (maxAbsU > kMaxVoltage) ? kMaxVoltage / maxAbsU : 1.0;
  } else if (maxU > 12.0) {
    U_ = U_ + U_.Ones() * (12.0 - maxU);
  } else if (minU < -12.0) {
    U_ = U_ - U_.Ones() * (minU + 12.0);
  }
  if (!U_.allFinite()) {
    U_.setZero();
  }
}

flatbuffers::Offset<LineFollowLogging> LineFollowDrivetrain::PopulateStatus(
    aos::Sender<drivetrain::Status>::Builder *builder) const {
  LineFollowLogging::Builder line_follow_logging_builder =
      builder->MakeBuilder<LineFollowLogging>();
  line_follow_logging_builder.add_frozen(freeze_target_);
  line_follow_logging_builder.add_have_target(have_target_);
  line_follow_logging_builder.add_x(target_pose_.abs_pos().x());
  line_follow_logging_builder.add_y(target_pose_.abs_pos().y());
  line_follow_logging_builder.add_theta(target_pose_.abs_theta());
  line_follow_logging_builder.add_offset(relative_pose_.rel_pos().y());
  line_follow_logging_builder.add_distance_to_target(
      -relative_pose_.rel_pos().x());
  line_follow_logging_builder.add_goal_theta(controls_goal_(0, 0));
  line_follow_logging_builder.add_rel_theta(relative_pose_.rel_theta());
  line_follow_logging_builder.add_drive_direction(
      target_selector_->DriveDirection());
  return line_follow_logging_builder.Finish();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
