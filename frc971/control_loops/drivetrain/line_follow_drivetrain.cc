#include <iostream>
#include "frc971/control_loops/drivetrain/line_follow_drivetrain.h"

#include "aos/commonmath.h"
#include "aos/util/math.h"
#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/dlqr.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

constexpr double LineFollowDrivetrain::kMaxVoltage;
constexpr double LineFollowDrivetrain::kPolyN;

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
  result.block<2, 2>(1, 0) = dt_config.Tlr_to_la() *
                             dt_config.make_hybrid_drivetrain_velocity_loop()
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

::Eigen::Matrix<double, 2, 3> CalcK(
    const ::Eigen::Matrix<double, 3, 3> & A,
    const ::Eigen::Matrix<double, 3, 2> & B,
    const ::Eigen::Matrix<double, 3, 3> & Q,
    const ::Eigen::Matrix<double, 2, 2> & R) {
  Eigen::Matrix<double, 2, 3> K;
  Eigen::Matrix<double, 3, 3> S;
  int info = ::frc971::controls::dlqr<3, 2>(A, B, Q, R, &K, &S);
  if (info != 0) {
    // We allow a FATAL error here since this should only be called during
    // initialization.
    LOG(FATAL, "Failed to solve %d, controllability: %d\n", info,
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

}  // namespace

// When we create A/B, we do recompute A/B, but we don't really care about
// optimizing it since it is only happening at initialization time...
LineFollowDrivetrain::LineFollowDrivetrain(
    const DrivetrainConfig<double> &dt_config,
    TargetSelectorInterface *target_selector)
    : dt_config_(dt_config),
      A_d_(ADiscrete(dt_config_)),
      B_d_(BDiscrete(dt_config_)),
      K_(CalcK(A_d_, B_d_, Q_, R_)),
      Kff_(CalcKff(B_d_)),
      target_selector_(target_selector),
      U_(0, 0) {}

double LineFollowDrivetrain::GoalTheta(
    const ::Eigen::Matrix<double, 5, 1> &state) const {
  // TODO(james): Consider latching the sign of the goal so that the driver
  // can back up without the robot trying to turn around...
  // On the other hand, we may just want to force the driver to take over
  // entirely if they need to backup.
  int sign = ::aos::sign(goal_velocity_);
  // Given a Nth degree polynomial with just a single term that
  // has its minimum (with a slope of zero) at (0, 0) and passing
  // through (x0, y0), we will have:
  // y = a * x^N
  // a = y0 / x0^N
  // And the slope of the tangent line at (x0, y0) will be
  // N * a * x0^(N-1)
  // = N * y0 / x0^N * x0^(N-1)
  // = N * y0 / x0
  // Giving a heading of
  // atan2(-N * y0, -x0)
  // where we add the negative signs to make things work out properly when we
  // are trying to drive forwards towards zero. We reverse the sign of both
  // terms if we want to drive backwards.
  return ::std::atan2(-sign * kPolyN * state.y(), -sign * state.x());
}

double LineFollowDrivetrain::GoalThetaDot(
    const ::Eigen::Matrix<double, 5, 1> &state) const {
  // theta = atan2(-N * y0, -x0)
  // Note: d(atan2(p(t), q(t)))/dt
  //       = (p(t) * q'(t) - q(t) * p'(t)) / (p(t)^2 + q(t)^2)
  // Note: x0' = cos(theta) * v, y0' = sin(theta) * v
  // Note that for the sin/cos calculations we discard the
  //   negatives in the arctangent to make the signs work out (the
  //   dtheta/dt needs to be correct as we travel along the path). This
  //   also corresponds with the fact that thetadot is agnostic towards
  //   whether the robot is driving forwards or backwards, so long as it is
  //   trying to drive towards the target.
  // Note: sin(theta) = sin(atan2(N * y0, x0))
  //       = N * y0 / sqrt(N^2 * y0^2 + x0^2)
  // Note: cos(theta) = cos(atan2(N * y0, x0))
  //       = x0 / sqrt(N^2 * y0^2 + x0^2)
  // dtheta/dt
  // = (-x0 * (-N * y0') - -N * y0 * (-x0')) / (N^2 * y0^2 + x0^2)
  // = N * (x0 * v * sin(theta) - y0 * v * cos(theta)) / (N^2 * y0^2 + x0^2)
  // = N * v * (x0 * (N * y0) - y0 * (x0)) / (N^2 * y0^2 + x0^2)^1.5
  // = N * v * (N - 1) * x0 * y0 / (N^2 * y0^2 + x0^2)^1.5
  const double linear_vel = (state(3, 0) + state(4, 0)) / 2.0;
  const double x2 = ::std::pow(state.x(), 2);
  const double y2 = ::std::pow(state.y(), 2);
  const double norm = y2 * kPolyN * kPolyN + x2;
  // When we get too near the goal, avoid singularity in a sane manner.
  if (norm < 1e-3) {
    return 0.0;
  }
  return kPolyN * (kPolyN - 1) * linear_vel * state.x() * state.y() /
         (norm * ::std::sqrt(norm));
}

void LineFollowDrivetrain::SetGoal(
    const ::frc971::control_loops::DrivetrainQueue::Goal &goal) {
  // TODO(james): More properly calculate goal velocity from throttle.
  goal_velocity_ = goal.throttle;
  // Freeze the target once the driver presses the button; if we haven't yet
  // confirmed a target when the driver presses the button, we will not do
  // anything and report not ready until we have a target.
  freeze_target_ = goal.controller_type == 3;
}

bool LineFollowDrivetrain::SetOutput(
    ::frc971::control_loops::DrivetrainQueue::Output *output) {
  // TODO(james): Account for voltage error terms, and/or provide driver with
  // ability to influence steering.
  if (output != nullptr && have_target_) {
    output->left_voltage = U_(0, 0);
    output->right_voltage = U_(1, 0);
  }
  return have_target_;
}

void LineFollowDrivetrain::Update(
    const ::Eigen::Matrix<double, 5, 1> &abs_state) {
  // Because we assume the target selector may have some internal state (e.g.,
  // not confirming a target until some time as passed), we should call
  // UpdateSelection every time.
  bool new_target =
      target_selector_->UpdateSelection(abs_state, goal_velocity_);
  if (freeze_target_) {
    // When freezing the target, only make changes if we didn't have a good
    // target before.
    if (!have_target_ && new_target) {
      have_target_ = true;
      target_pose_ = target_selector_->TargetPose();
    }
  } else {
    // If the target selector has lost its target, we *do* want to set
    // have_target_ to false, so long as we aren't trying to freeze the target.
    have_target_ = new_target;
    if (have_target_) {
      target_pose_ = target_selector_->TargetPose();
    }
  }

  // Get the robot pose in the target coordinate frame.
  const Pose relative_robot_pose = Pose({abs_state.x(), abs_state.y(), 0.0},
                                        abs_state(2, 0)).Rebase(&target_pose_);
  // Always force a slight negative X, so that the driver can continue to drive
  // past zero if they want.
  // The "slight negative" needs to be large enough that we won't force
  // obnoxiously sharp turning radii if we run past the end of the
  // line--because, in practice, the driver should never be trying to drive to a
  // target where they are significantly off laterally at <0.1m from the target,
  // this should not be a problem.
  const double relative_x = ::std::min(relative_robot_pose.rel_pos().x(), -0.1);
  const ::Eigen::Matrix<double, 5, 1> rel_state =
      (::Eigen::Matrix<double, 5, 1>() << relative_x,
       relative_robot_pose.rel_pos().y(), relative_robot_pose.rel_theta(),
       abs_state(3, 0), abs_state(4, 0))
          .finished();
  ::Eigen::Matrix<double, 3, 1> controls_goal(
      GoalTheta(rel_state), goal_velocity_, GoalThetaDot(rel_state));
  ::Eigen::Matrix<double, 3, 1> controls_state;
  controls_state(0, 0) = rel_state(2, 0);
  controls_state.block<2, 1>(1, 0) =
      dt_config_.Tlr_to_la() * rel_state.block<2, 1>(3, 0);
  ::Eigen::Matrix<double, 3, 1> controls_err = controls_goal - controls_state;
  // Because we are taking the difference of an angle, normaliez to [-pi, pi].
  controls_err(0, 0) = ::aos::math::NormalizeAngle(controls_err(0, 0));
  // TODO(james): Calculate the next goal so that we are properly doing
  // feed-forwards.
  ::Eigen::Matrix<double, 2, 1> U_ff =
      Kff_ * (controls_goal - A_d_ * controls_goal);
  U_ = K_ * controls_err + U_ff;
  const double maxU = U_.lpNorm<::Eigen::Infinity>();
  U_ *= (maxU > kMaxVoltage) ? kMaxVoltage / maxU : 1.0;
}

void LineFollowDrivetrain::PopulateStatus(
    ::frc971::control_loops::DrivetrainQueue::Status *status) const {
  status->line_follow_logging.frozen = freeze_target_;
  status->line_follow_logging.have_target = have_target_;
  status->line_follow_logging.x = target_pose_.abs_pos().x();
  status->line_follow_logging.y = target_pose_.abs_pos().y();
  status->line_follow_logging.theta = target_pose_.abs_theta();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
