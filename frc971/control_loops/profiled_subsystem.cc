#include "frc971/control_loops/profiled_subsystem.h"

namespace frc971 {
namespace control_loops {

namespace {
double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}
}  // namespace

SingleDOFProfiledSubsystem::SingleDOFProfiledSubsystem(
    ::std::unique_ptr<SimpleCappedStateFeedbackLoop<3, 1, 1>> loop,
    const ::frc971::constants::ZeroingConstants &zeroing_constants,
    const ::frc971::constants::Range &range, double default_velocity,
    double default_acceleration)
    : ProfiledSubsystem(::std::move(loop), {{zeroing_constants}}),
      profile_(::aos::controls::kLoopFrequency),
      range_(range),
      default_velocity_(default_velocity),
      default_acceleration_(default_acceleration) {
  Y_.setZero();
  offset_.setZero();
  AdjustProfile(0.0, 0.0);
}

void SingleDOFProfiledSubsystem::UpdateOffset(double offset) {
  const double doffset = offset - offset_(0, 0);
  LOG(INFO, "Adjusting offset from %f to %f\n", offset_(0, 0), offset);

  loop_->mutable_X_hat()(0, 0) += doffset;
  Y_(0, 0) += doffset;
  loop_->mutable_R(0, 0) += doffset;

  profile_.MoveGoal(doffset);
  offset_(0, 0) = offset;

  CapGoal("R", &loop_->mutable_R());
}

void SingleDOFProfiledSubsystem::Correct(PotAndIndexPosition position) {
  estimators_[0].UpdateEstimate(position);

  if (estimators_[0].error()) {
    LOG(ERROR, "zeroing error with intake_estimator\n");
    return;
  }

  if (!initialized_) {
    if (estimators_[0].offset_ready()) {
      UpdateOffset(estimators_[0].offset());
      initialized_ = true;
    }
  }

  if (!zeroed(0) && estimators_[0].zeroed()) {
    UpdateOffset(estimators_[0].offset());
    set_zeroed(0, true);
  }

  Y_ << position.encoder;
  Y_ += offset_;
  loop_->Correct(Y_);
}

void SingleDOFProfiledSubsystem::CapGoal(const char *name,
                                         Eigen::Matrix<double, 3, 1> *goal) {
  // Limit the goal to min/max allowable angles.
  if ((*goal)(0, 0) > range_.upper) {
    LOG(WARNING, "Intake goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        range_.upper);
    (*goal)(0, 0) = range_.upper;
  }
  if ((*goal)(0, 0) < range_.lower) {
    LOG(WARNING, "Intake goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        range_.lower);
    (*goal)(0, 0) = range_.lower;
  }
}

void SingleDOFProfiledSubsystem::ForceGoal(double goal) {
  set_unprofiled_goal(goal);
  loop_->mutable_R() = unprofiled_goal_;
  loop_->mutable_next_R() = loop_->R();

  profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
}

void SingleDOFProfiledSubsystem::set_unprofiled_goal(double unprofiled_goal) {
  unprofiled_goal_(0, 0) = unprofiled_goal;
  unprofiled_goal_(1, 0) = 0.0;
  unprofiled_goal_(2, 0) = 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void SingleDOFProfiledSubsystem::Update(bool disable) {
  if (!disable) {
    ::Eigen::Matrix<double, 2, 1> goal_state =
        profile_.Update(unprofiled_goal_(0, 0), unprofiled_goal_(1, 0));

    loop_->mutable_next_R(0, 0) = goal_state(0, 0);
    loop_->mutable_next_R(1, 0) = goal_state(1, 0);
    loop_->mutable_next_R(2, 0) = 0.0;
    CapGoal("next R", &loop_->mutable_next_R());
  }

  loop_->Update(disable);

  if (!disable && loop_->U(0, 0) != loop_->U_uncapped(0, 0)) {
    profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
  }
}

bool SingleDOFProfiledSubsystem::CheckHardLimits() {
  // Returns whether hard limits have been exceeded.

  if (angle() > range_.upper_hard || angle() < range_.lower_hard) {
    LOG(ERROR,
        "SingleDOFProfiledSubsystem at %f out of bounds [%f, %f], ESTOPing\n",
        angle(), range_.lower_hard, range_.upper_hard);
    return true;
  }

  return false;
}

void SingleDOFProfiledSubsystem::AdjustProfile(
    double max_angular_velocity, double max_angular_acceleration) {
  profile_.set_maximum_velocity(
      UseUnlessZero(max_angular_velocity, default_velocity_));
  profile_.set_maximum_acceleration(
      UseUnlessZero(max_angular_acceleration, default_acceleration_));
}

}  // namespace control_loops
}  // namespace frc971
