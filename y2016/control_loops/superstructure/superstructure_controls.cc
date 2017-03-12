#include "y2016/control_loops/superstructure/superstructure_controls.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2016/control_loops/superstructure/integral_intake_plant.h"
#include "y2016/control_loops/superstructure/integral_arm_plant.h"

#include "y2016/constants.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {

using ::frc971::PotAndIndexPosition;

namespace {
double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}

enum ArmIndices { kShoulderIndex = 0, kWristIndex = 1 };

}  // namespace

// Intake
Intake::Intake()
    : ::frc971::control_loops::SingleDOFProfiledSubsystem<>(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  3, 1, 1>(::y2016::control_loops::superstructure::
                               MakeIntegralIntakeLoop())),
          constants::GetValues().intake.zeroing,
          constants::Values::kIntakeRange, 10.0, 10.0) {}

Arm::Arm()
    : ProfiledSubsystem(
          ::std::unique_ptr<ArmControlLoop>(new ArmControlLoop(
              ::y2016::control_loops::superstructure::MakeIntegralArmLoop())),
          {{constants::GetValues().shoulder.zeroing,
            constants::GetValues().wrist.zeroing}}),
      shoulder_profile_(::aos::controls::kLoopFrequency),
      wrist_profile_(::aos::controls::kLoopFrequency) {
  Y_.setZero();
  offset_.setZero();
  AdjustProfile(0.0, 0.0, 0.0, 0.0);
}

void Arm::UpdateWristOffset(double offset) {
  const double doffset = offset - offset_(1, 0);
  LOG(INFO, "Adjusting Wrist offset from %f to %f\n", offset_(1, 0), offset);

  loop_->mutable_X_hat()(2, 0) += doffset;
  Y_(1, 0) += doffset;
  loop_->mutable_R(2, 0) += doffset;
  loop_->mutable_next_R(2, 0) += doffset;
  unprofiled_goal_(2, 0) += doffset;

  wrist_profile_.MoveGoal(doffset);
  offset_(1, 0) = offset;

  CapGoal("R", &loop_->mutable_R());
  CapGoal("unprofiled R", &loop_->mutable_next_R());
}

void Arm::UpdateShoulderOffset(double offset) {
  const double doffset = offset - offset_(0, 0);
  LOG(INFO, "Adjusting Shoulder offset from %f to %f\n", offset_(0, 0), offset);

  loop_->mutable_X_hat()(0, 0) += doffset;
  loop_->mutable_X_hat()(2, 0) += doffset;
  Y_(0, 0) += doffset;
  loop_->mutable_R(0, 0) += doffset;
  loop_->mutable_R(2, 0) += doffset;
  loop_->mutable_next_R(0, 0) += doffset;
  loop_->mutable_next_R(2, 0) += doffset;
  unprofiled_goal_(0, 0) += doffset;
  unprofiled_goal_(2, 0) += doffset;

  shoulder_profile_.MoveGoal(doffset);
  wrist_profile_.MoveGoal(doffset);
  offset_(0, 0) = offset;

  CapGoal("R", &loop_->mutable_R());
  CapGoal("unprofiled R", &loop_->mutable_next_R());
}

// TODO(austin): Handle zeroing errors.

void Arm::Correct(PotAndIndexPosition position_shoulder,
                  PotAndIndexPosition position_wrist) {
  estimators_[kShoulderIndex].UpdateEstimate(position_shoulder);
  estimators_[kWristIndex].UpdateEstimate(position_wrist);

  // Handle zeroing errors
  if (estimators_[kShoulderIndex].error()) {
    LOG(ERROR, "zeroing error with shoulder_estimator\n");
    return;
  }
  if (estimators_[kWristIndex].error()) {
    LOG(ERROR, "zeroing error with wrist_estimator\n");
    return;
  }

  if (!initialized_) {
    if (estimators_[kShoulderIndex].offset_ready() &&
        estimators_[kWristIndex].offset_ready()) {
      UpdateShoulderOffset(estimators_[kShoulderIndex].offset());
      UpdateWristOffset(estimators_[kWristIndex].offset());
      initialized_ = true;
    }
  }

  if (!zeroed(kShoulderIndex) && estimators_[kShoulderIndex].zeroed()) {
    UpdateShoulderOffset(estimators_[kShoulderIndex].offset());
    set_zeroed(kShoulderIndex, true);
  }
  if (!zeroed(kWristIndex) && estimators_[kWristIndex].zeroed()) {
    UpdateWristOffset(estimators_[kWristIndex].offset());
    set_zeroed(kWristIndex, true);
  }

  {
    Y_ << position_shoulder.encoder, position_wrist.encoder;
    Y_ += offset_;
    loop_->Correct(Y_);
  }
}

void Arm::CapGoal(const char *name, Eigen::Matrix<double, 6, 1> *goal) {
  // Limit the goals to min/max allowable angles.

  if ((*goal)(0, 0) > constants::Values::kShoulderRange.upper) {
    LOG(WARNING, "Shoulder goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        constants::Values::kShoulderRange.upper);
    (*goal)(0, 0) = constants::Values::kShoulderRange.upper;
  }
  if ((*goal)(0, 0) < constants::Values::kShoulderRange.lower) {
    LOG(WARNING, "Shoulder goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        constants::Values::kShoulderRange.lower);
    (*goal)(0, 0) = constants::Values::kShoulderRange.lower;
  }

  const double wrist_goal_angle_ungrounded = (*goal)(2, 0) - (*goal)(0, 0);

  if (wrist_goal_angle_ungrounded > constants::Values::kWristRange.upper) {
    LOG(WARNING, "Wrist goal %s above limit, %f > %f\n", name,
        wrist_goal_angle_ungrounded, constants::Values::kWristRange.upper);
    (*goal)(2, 0) = constants::Values::kWristRange.upper + (*goal)(0, 0);
  }
  if (wrist_goal_angle_ungrounded < constants::Values::kWristRange.lower) {
    LOG(WARNING, "Wrist goal %s below limit, %f < %f\n", name,
        wrist_goal_angle_ungrounded, constants::Values::kWristRange.lower);
    (*goal)(2, 0) = constants::Values::kWristRange.lower + (*goal)(0, 0);
  }
}

void Arm::ForceGoal(double goal_shoulder, double goal_wrist) {
  set_unprofiled_goal(goal_shoulder, goal_wrist);
  loop_->mutable_R() = unprofiled_goal_;
  loop_->mutable_next_R() = loop_->R();

  shoulder_profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
  wrist_profile_.MoveCurrentState(loop_->R().block<2, 1>(2, 0));
}

void Arm::set_unprofiled_goal(double unprofiled_goal_shoulder,
                              double unprofiled_goal_wrist) {
  unprofiled_goal_ << unprofiled_goal_shoulder, 0.0, unprofiled_goal_wrist, 0.0,
      0.0, 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void Arm::AdjustProfile(double max_angular_velocity_shoulder,
                        double max_angular_acceleration_shoulder,
                        double max_angular_velocity_wrist,
                        double max_angular_acceleration_wrist) {
  shoulder_profile_.set_maximum_velocity(
      UseUnlessZero(max_angular_velocity_shoulder, 10.0));
  shoulder_profile_.set_maximum_acceleration(
      UseUnlessZero(max_angular_acceleration_shoulder, 10.0));
  wrist_profile_.set_maximum_velocity(
      UseUnlessZero(max_angular_velocity_wrist, 10.0));
  wrist_profile_.set_maximum_acceleration(
      UseUnlessZero(max_angular_acceleration_wrist, 10.0));
}

bool Arm::CheckHardLimits() {
  if (shoulder_angle() > constants::Values::kShoulderRange.upper_hard ||
      shoulder_angle() < constants::Values::kShoulderRange.lower_hard) {
    LOG(ERROR, "Shoulder at %f out of bounds [%f, %f], ESTOPing\n",
        shoulder_angle(), constants::Values::kShoulderRange.lower_hard,
        constants::Values::kShoulderRange.upper_hard);
    return true;
  }

  if (wrist_angle() - shoulder_angle() >
          constants::Values::kWristRange.upper_hard ||
      wrist_angle() - shoulder_angle() <
          constants::Values::kWristRange.lower_hard) {
    LOG(ERROR, "Wrist at %f out of bounds [%f, %f], ESTOPing\n",
        wrist_angle() - shoulder_angle(),
        constants::Values::kWristRange.lower_hard,
        constants::Values::kWristRange.upper_hard);
    return true;
  }

  return false;
}

void Arm::Update(bool disable) {
  // TODO(austin): Reset the state vectors and internal state here.
  if (should_reset_) {
    should_reset_ = false;
  }

  if (!disable) {
    // Compute next goal.
    loop_->mutable_next_R().block<2, 1>(0, 0) = shoulder_profile_.Update(
        unprofiled_goal_(0, 0), unprofiled_goal_(1, 0));

    loop_->mutable_next_R().block<2, 1>(2, 0) =
        wrist_profile_.Update(unprofiled_goal_(2, 0), unprofiled_goal_(3, 0));

    loop_->mutable_next_R().block<2, 1>(4, 0) =
        unprofiled_goal_.block<2, 1>(4, 0);
    CapGoal("next R", &loop_->mutable_next_R());
  }

  // Move loop
  loop_->Update(disable);

  // Shoulder saturated
  if (!disable && loop_->U(0, 0) != loop_->U_uncapped(0, 0)) {
    LOG(DEBUG, "Moving shoulder state.  U: %f, %f\n", loop_->U(0, 0),
        loop_->U_uncapped(0, 0));
    shoulder_profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
  }

  // Wrist saturated
  if (!disable && loop_->U(1, 0) != loop_->U_uncapped(1, 0)) {
    LOG(DEBUG, "Moving shooter state.  U: %f, %f\n", loop_->U(1, 0),
        loop_->U_uncapped(1, 0));
    wrist_profile_.MoveCurrentState(loop_->R().block<2, 1>(2, 0));
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016
