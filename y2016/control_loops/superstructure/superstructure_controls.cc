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
using ::frc971::EstimatorState;

namespace {
double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}
}  // namespace

// Intake
Intake::Intake()
    : loop_(new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>(
          ::y2016::control_loops::superstructure::MakeIntegralIntakeLoop())),
      estimator_(constants::GetValues().intake.zeroing),
      profile_(::aos::controls::kLoopFrequency) {
  Y_.setZero();
  unprofiled_goal_.setZero();
  offset_.setZero();
  AdjustProfile(0.0, 0.0);
}

void Intake::UpdateIntakeOffset(double offset) {
  const double doffset = offset - offset_(0, 0);
  LOG(INFO, "Adjusting Intake offset from %f to %f\n", offset_(0, 0), offset);

  loop_->mutable_X_hat()(0, 0) += doffset;
  Y_(0, 0) += doffset;
  loop_->mutable_R(0, 0) += doffset;

  profile_.MoveGoal(doffset);
  offset_(0, 0) = offset;

  CapGoal("R", &loop_->mutable_R());
}

void Intake::Correct(PotAndIndexPosition position) {
  estimator_.UpdateEstimate(position);

  if (estimator_.error()) {
    LOG(ERROR, "zeroing error with intake_estimator\n");
    return;
  }

  if (!initialized_) {
    if (estimator_.offset_ready()) {
      UpdateIntakeOffset(estimator_.offset());
      initialized_ = true;
    }
  }

  if (!zeroed_ && estimator_.zeroed()) {
    UpdateIntakeOffset(estimator_.offset());
    zeroed_ = true;
  }

  Y_ << position.encoder;
  Y_ += offset_;
  loop_->Correct(Y_);
}

void Intake::CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal) {
  // Limit the goal to min/max allowable angles.
  if ((*goal)(0, 0) > constants::Values::kIntakeRange.upper) {
    LOG(WARNING, "Intake goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        constants::Values::kIntakeRange.upper);
    (*goal)(0, 0) = constants::Values::kIntakeRange.upper;
  }
  if ((*goal)(0, 0) < constants::Values::kIntakeRange.lower) {
    LOG(WARNING, "Intake goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        constants::Values::kIntakeRange.lower);
    (*goal)(0, 0) = constants::Values::kIntakeRange.lower;
  }
}

void Intake::ForceGoal(double goal) {
  set_unprofiled_goal(goal);
  loop_->mutable_R() = unprofiled_goal_;
  loop_->mutable_next_R() = loop_->R();

  profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
}

void Intake::set_unprofiled_goal(double unprofiled_goal) {
  unprofiled_goal_(0, 0) = unprofiled_goal;
  unprofiled_goal_(1, 0) = 0.0;
  unprofiled_goal_(2, 0) = 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void Intake::Update(bool disable) {
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

bool Intake::CheckHardLimits() {
  // Returns whether hard limits have been exceeded.

  if (angle() > constants::Values::kIntakeRange.upper_hard ||
      angle() < constants::Values::kIntakeRange.lower_hard) {
    LOG(ERROR, "Intake at %f out of bounds [%f, %f], ESTOPing\n", angle(),
        constants::Values::kIntakeRange.lower_hard,
        constants::Values::kIntakeRange.upper_hard);
    return true;
  }

  return false;
}

void Intake::set_max_voltage(double voltage) {
  loop_->set_max_voltage(0, voltage);
}

void Intake::AdjustProfile(double max_angular_velocity,
                           double max_angular_acceleration) {
  profile_.set_maximum_velocity(UseUnlessZero(max_angular_velocity, 10.0));
  profile_.set_maximum_acceleration(
      UseUnlessZero(max_angular_acceleration, 10.0));
}

void Intake::Reset() {
  estimator_.Reset();
  initialized_ = false;
  zeroed_ = false;
}

EstimatorState Intake::IntakeEstimatorState() {
  EstimatorState estimator_state;
  ::frc971::zeroing::PopulateEstimatorState(estimator_, &estimator_state);

  return estimator_state;
}

Arm::Arm()
    : loop_(new ArmControlLoop(
          ::y2016::control_loops::superstructure::MakeIntegralArmLoop())),
      shoulder_profile_(::aos::controls::kLoopFrequency),
      wrist_profile_(::aos::controls::kLoopFrequency),
      shoulder_estimator_(constants::GetValues().shoulder.zeroing),
      wrist_estimator_(constants::GetValues().wrist.zeroing) {
  Y_.setZero();
  offset_.setZero();
  unprofiled_goal_.setZero();
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
  shoulder_estimator_.UpdateEstimate(position_shoulder);
  wrist_estimator_.UpdateEstimate(position_wrist);

  // Handle zeroing errors
  if (shoulder_estimator_.error()) {
    LOG(ERROR, "zeroing error with shoulder_estimator\n");
    return;
  }
  if (wrist_estimator_.error()) {
    LOG(ERROR, "zeroing error with wrist_estimator\n");
    return;
  }

  if (!initialized_) {
    if (shoulder_estimator_.offset_ready() && wrist_estimator_.offset_ready()) {
      UpdateShoulderOffset(shoulder_estimator_.offset());
      UpdateWristOffset(wrist_estimator_.offset());
      initialized_ = true;
    }
  }

  if (!shoulder_zeroed_ && shoulder_estimator_.zeroed()) {
    UpdateShoulderOffset(shoulder_estimator_.offset());
    shoulder_zeroed_ = true;
  }
  if (!wrist_zeroed_ && wrist_estimator_.zeroed()) {
    UpdateWristOffset(wrist_estimator_.offset());
    wrist_zeroed_ = true;
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

void Arm::set_max_voltage(double shoulder_max_voltage,
                          double wrist_max_voltage) {
  loop_->set_max_voltage(0, shoulder_max_voltage);
  loop_->set_max_voltage(1, wrist_max_voltage);
}

void Arm::Reset() {
  shoulder_estimator_.Reset();
  wrist_estimator_.Reset();
  initialized_ = false;
  shoulder_zeroed_ = false;
  wrist_zeroed_ = false;
}

EstimatorState Arm::ShoulderEstimatorState() {
  EstimatorState estimator_state;
  ::frc971::zeroing::PopulateEstimatorState(shoulder_estimator_,
                                            &estimator_state);

  return estimator_state;
}

EstimatorState Arm::WristEstimatorState() {
  EstimatorState estimator_state;
  ::frc971::zeroing::PopulateEstimatorState(wrist_estimator_, &estimator_state);

  return estimator_state;
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016
