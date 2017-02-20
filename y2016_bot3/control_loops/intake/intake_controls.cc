#include "y2016_bot3/control_loops/intake/intake_controls.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2016_bot3/control_loops/intake/integral_intake_plant.h"

#include "y2016_bot3/control_loops/intake/intake.h"

namespace y2016_bot3 {
namespace constants {
IntakeZero intake_zero;
}
namespace control_loops {
namespace intake {

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
IntakeArm::IntakeArm()
    : loop_(new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>(
          ::y2016_bot3::control_loops::intake::MakeIntegralIntakeLoop())),
      estimator_(y2016_bot3::constants::intake_zero.zeroing),
      profile_(::aos::controls::kLoopFrequency) {
  Y_.setZero();
  unprofiled_goal_.setZero();
  offset_.setZero();
  AdjustProfile(0.0, 0.0);
}

void IntakeArm::UpdateIntakeOffset(double offset) {
  const double doffset = offset - offset_(0, 0);
  LOG(INFO, "Adjusting Intake offset from %f to %f\n", offset_(0, 0), offset);

  loop_->mutable_X_hat()(0, 0) += doffset;
  Y_(0, 0) += doffset;
  loop_->mutable_R(0, 0) += doffset;

  profile_.MoveGoal(doffset);
  offset_(0, 0) = offset;

  CapGoal("R", &loop_->mutable_R());
}

void IntakeArm::Correct(PotAndIndexPosition position) {
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

void IntakeArm::CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal) {
  // Limit the goal to min/max allowable angles.
  if ((*goal)(0, 0) > y2016_bot3::constants::kIntakeRange.upper) {
    LOG(WARNING, "Intake goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        y2016_bot3::constants::kIntakeRange.upper);
    (*goal)(0, 0) = y2016_bot3::constants::kIntakeRange.upper;
  }
  if ((*goal)(0, 0) < y2016_bot3::constants::kIntakeRange.lower) {
    LOG(WARNING, "Intake goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        y2016_bot3::constants::kIntakeRange.lower);
    (*goal)(0, 0) = y2016_bot3::constants::kIntakeRange.lower;
  }
}

void IntakeArm::ForceGoal(double goal) {
  set_unprofiled_goal(goal);
  loop_->mutable_R() = unprofiled_goal_;
  loop_->mutable_next_R() = loop_->R();

  profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
}

void IntakeArm::set_unprofiled_goal(double unprofiled_goal) {
  unprofiled_goal_(0, 0) = unprofiled_goal;
  unprofiled_goal_(1, 0) = 0.0;
  unprofiled_goal_(2, 0) = 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void IntakeArm::Update(bool disable) {
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

bool IntakeArm::CheckHardLimits() {
  // Returns whether hard limits have been exceeded.

  if (angle() > y2016_bot3::constants::kIntakeRange.upper_hard ||
      angle() < y2016_bot3::constants::kIntakeRange.lower_hard) {
    LOG(ERROR, "Intake at %f out of bounds [%f, %f], ESTOPing\n", angle(),
        y2016_bot3::constants::kIntakeRange.lower_hard,
        y2016_bot3::constants::kIntakeRange.upper_hard);
    return true;
  }

  return false;
}

void IntakeArm::set_max_voltage(double voltage) {
  loop_->set_max_voltage(0, voltage);
}

void IntakeArm::AdjustProfile(double max_angular_velocity,
                              double max_angular_acceleration) {
  profile_.set_maximum_velocity(UseUnlessZero(max_angular_velocity, 10.0));
  profile_.set_maximum_acceleration(
      UseUnlessZero(max_angular_acceleration, 10.0));
}

void IntakeArm::Reset() {
  estimator_.Reset();
  initialized_ = false;
  zeroed_ = false;
}

EstimatorState IntakeArm::IntakeEstimatorState() {
  return estimator_.GetEstimatorState();
}

}  // namespace intake
}  // namespace control_loops
}  // namespace y2016_bot3
