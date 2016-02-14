#include "y2016/control_loops/superstructure/superstructure.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2016/control_loops/superstructure/integral_intake_plant.h"
#include "y2016/control_loops/superstructure/integral_arm_plant.h"

#include "y2016/constants.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {

namespace {
constexpr double kZeroingVoltage = 4.0;

double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}

}  // namespace

void SimpleCappedStateFeedbackLoop::CapU() {
  mutable_U(0, 0) = ::std::min(U(0, 0), max_voltage_);
  mutable_U(0, 0) = ::std::max(U(0, 0), -max_voltage_);
}

void DoubleCappedStateFeedbackLoop::CapU() {
  mutable_U(0, 0) = ::std::min(U(0, 0), shoulder_max_voltage_);
  mutable_U(0, 0) = ::std::max(U(0, 0), -shoulder_max_voltage_);
  mutable_U(1, 0) = ::std::min(U(1, 0), wrist_max_voltage_);
  mutable_U(1, 0) = ::std::max(U(1, 0), -wrist_max_voltage_);
}

// Intake
Intake::Intake()
    : loop_(new SimpleCappedStateFeedbackLoop(StateFeedbackLoop<3, 1, 1>(
          ::y2016::control_loops::superstructure::MakeIntegralIntakeLoop()))),
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
  const auto &values = constants::GetValues();

  // Limit the goal to min/max allowable angles.
  if ((*goal)(0, 0) >= values.intake.limits.upper) {
    LOG(WARNING, "Intake goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        values.intake.limits.upper);
    (*goal)(0, 0) = values.intake.limits.upper;
  }
  if ((*goal)(0, 0) <= values.intake.limits.lower) {
    LOG(WARNING, "Intake goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        values.intake.limits.lower);
    (*goal)(0, 0) = values.intake.limits.lower;
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
  const auto &values = constants::GetValues();
  // Returns whether hard limits have been exceeded.

  if (angle() >= values.intake.limits.upper_hard ||
      angle() <= values.intake.limits.lower_hard) {
    LOG(ERROR, "Intake at %f out of bounds [%f, %f], ESTOPing\n", angle(),
        values.intake.limits.lower_hard, values.intake.limits.upper_hard);
    return true;
  }

  return false;
}

void Intake::set_max_voltage(double voltage) {
  loop_->set_max_voltage(voltage);
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
    : loop_(new DoubleCappedStateFeedbackLoop(
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
  const auto &values = constants::GetValues();

  if ((*goal)(0, 0) >= values.shoulder.limits.upper) {
    LOG(WARNING, "Shoulder goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        values.shoulder.limits.upper);
    (*goal)(0, 0) = values.shoulder.limits.upper;
  }
  if ((*goal)(0, 0) <= values.shoulder.limits.lower) {
    LOG(WARNING, "Shoulder goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        values.shoulder.limits.lower);
    (*goal)(0, 0) = values.shoulder.limits.lower;
  }

  const double wrist_goal_angle_ungrounded = (*goal)(2, 0) - (*goal)(0, 0);

  if (wrist_goal_angle_ungrounded >= values.wrist.limits.upper) {
    LOG(WARNING, "Wrist goal %s above limit, %f > %f\n", name,
        wrist_goal_angle_ungrounded, values.wrist.limits.upper);
    (*goal)(2, 0) = values.wrist.limits.upper + (*goal)(0, 0);
  }
  if (wrist_goal_angle_ungrounded <= values.wrist.limits.lower) {
    LOG(WARNING, "Wrist goal %s below limit, %f < %f\n", name,
        wrist_goal_angle_ungrounded, values.wrist.limits.lower);
    (*goal)(2, 0) = values.wrist.limits.lower + (*goal)(0, 0);
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
  const auto &values = constants::GetValues();
  if (shoulder_angle() >= values.shoulder.limits.upper_hard ||
      shoulder_angle() <= values.shoulder.limits.lower_hard) {
    LOG(ERROR, "Shoulder at %f out of bounds [%f, %f], ESTOPing\n",
        shoulder_angle(), values.shoulder.limits.lower_hard,
        values.shoulder.limits.upper_hard);
    return true;
  }

  if (wrist_angle() - shoulder_angle() >= values.wrist.limits.upper_hard ||
      wrist_angle() - shoulder_angle() <= values.wrist.limits.lower_hard) {
    LOG(ERROR, "Wrist at %f out of bounds [%f, %f], ESTOPing\n",
        wrist_angle() - shoulder_angle(), values.wrist.limits.lower_hard,
        values.wrist.limits.upper_hard);
    return true;
  }

  return false;
}

void Arm::Update(bool disable) {
  if (!disable) {
    // Compute next goal.
    ::Eigen::Matrix<double, 2, 1> goal_state_shoulder =
        shoulder_profile_.Update(unprofiled_goal_(0, 0),
                                 unprofiled_goal_(1, 0));
    loop_->mutable_next_R(0, 0) = goal_state_shoulder(0, 0);
    loop_->mutable_next_R(1, 0) = goal_state_shoulder(1, 0);

    ::Eigen::Matrix<double, 2, 1> goal_state_wrist =
        wrist_profile_.Update(unprofiled_goal_(2, 0), unprofiled_goal_(3, 0));
    loop_->mutable_next_R(2, 0) = goal_state_wrist(0, 0);
    loop_->mutable_next_R(3, 0) = goal_state_wrist(1, 0);

    loop_->mutable_next_R(4, 0) = unprofiled_goal_(4, 0);
    loop_->mutable_next_R(5, 0) = unprofiled_goal_(5, 0);
    CapGoal("next R", &loop_->mutable_next_R());
  }

  // Move loop
  loop_->Update(disable);

  // Shoulder saturated
  if (!disable && loop_->U(0, 0) != loop_->U_uncapped(0, 0)) {
    shoulder_profile_.MoveCurrentState(loop_->R().block<2, 1>(0, 0));
  }

  // Wrist saturated
  if (!disable && loop_->U(1, 0) != loop_->U_uncapped(1, 0)) {
    wrist_profile_.MoveCurrentState(loop_->R().block<2, 1>(2, 0));
  }
}

void Arm::set_max_voltage(double shoulder_max_voltage,
                          double wrist_max_voltage) {
  loop_->set_max_voltage(shoulder_max_voltage, wrist_max_voltage);
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

// ///// Superstructure /////
Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue) {}

void Superstructure::UpdateZeroingState() {
  // TODO(austin): Explicit state transitions instead of this.
  // TODO(adam): Change this once we have zeroing written.
  if (!arm_.initialized() || !intake_.initialized()) {
    state_ = INITIALIZING;
  } else if (!intake_.zeroed()) {
    state_ = ZEROING_INTAKE;
  } else if (!arm_.zeroed()) {
    state_ = ZEROING_ARM;
  } else {
    state_ = RUNNING;
  }
}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position *position,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    arm_.Reset();
    intake_.Reset();
    state_ = UNINITIALIZED;
  }

  // Bool to track if we should turn the motors on or not.
  bool disable = output == nullptr;

  arm_.Correct(position->shoulder, position->wrist);
  intake_.Correct(position->intake);

  // Zeroing will work as follows:
  // Start with the intake. Move it towards the center. Once zeroed, move it
  // back to the bottom. Rotate the shoulder towards the center. Once zeroed,
  // move it up enough to rotate the wrist towards the center.

  // We'll then need code to do sanity checking on values.

  switch (state_) {
    case UNINITIALIZED:
      LOG(DEBUG, "Uninitialized\n");
      state_ = INITIALIZING;
      disable = true;
      break;

    case INITIALIZING:
      LOG(DEBUG, "Waiting for accurate initial position.\n");
      disable = true;
      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      if (state_ != INITIALIZING) {
        // Set the goals to where we are now.
        intake_.ForceGoal(intake_.angle());
        arm_.ForceGoal(arm_.shoulder_angle(), arm_.wrist_angle());
      }
      break;

    case ZEROING_INTAKE:
    case ZEROING_ARM:
      // TODO(adam): Add your magic here.
      state_ = RUNNING;
      break;

    case RUNNING:
      if (unsafe_goal) {
        arm_.AdjustProfile(unsafe_goal->max_angular_velocity_shoulder,
                           unsafe_goal->max_angular_acceleration_shoulder,
                           unsafe_goal->max_angular_velocity_wrist,
                           unsafe_goal->max_angular_acceleration_wrist);
        intake_.AdjustProfile(unsafe_goal->max_angular_velocity_wrist,
                              unsafe_goal->max_angular_acceleration_intake);

        arm_.set_unprofiled_goal(unsafe_goal->angle_shoulder,
                                 unsafe_goal->angle_wrist);
        intake_.set_unprofiled_goal(unsafe_goal->angle_intake);
      }

      // Update state_ to accurately represent the state of the zeroing
      // estimators.

      if (state_ != RUNNING && state_ != ESTOP) {
        state_ = UNINITIALIZED;
      }
      break;

    case ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // ESTOP if we hit any of the limits.  It is safe(ish) to hit the limits while
  // zeroing since we use such low power.
  if (state_ == RUNNING) {
    // ESTOP if we hit the hard limits.
    if ((arm_.CheckHardLimits() || intake_.CheckHardLimits()) && output) {
      state_ = ESTOP;
    }
  }

  // Set the voltage limits.
  const double max_voltage = state_ == RUNNING ? 12.0 : kZeroingVoltage;
  arm_.set_max_voltage(max_voltage, max_voltage);
  intake_.set_max_voltage(max_voltage);

  // Calculate the loops for a cycle.
  arm_.Update(disable);
  intake_.Update(disable);

  // Write out all the voltages.
  if (output) {
    output->voltage_intake = intake_.intake_voltage();
    output->voltage_shoulder = arm_.shoulder_voltage();
    output->voltage_wrist = arm_.wrist_voltage();
  }

  // Save debug/internal state.
  // TODO(austin): Save the voltage errors.
  status->zeroed = state_ == RUNNING;

  status->shoulder.angle = arm_.X_hat(0, 0);
  status->shoulder.angular_velocity = arm_.X_hat(1, 0);
  status->shoulder.goal_angle = arm_.goal(0, 0);
  status->shoulder.goal_angular_velocity = arm_.goal(1, 0);
  status->shoulder.unprofiled_goal_angle = arm_.unprofiled_goal(0, 0);
  status->shoulder.unprofiled_goal_angular_velocity =
      arm_.unprofiled_goal(1, 0);
  status->shoulder.estimator_state = arm_.ShoulderEstimatorState();

  status->wrist.angle = arm_.X_hat(2, 0);
  status->wrist.angular_velocity = arm_.X_hat(3, 0);
  status->wrist.goal_angle = arm_.goal(2, 0);
  status->wrist.goal_angular_velocity = arm_.goal(3, 0);
  status->wrist.unprofiled_goal_angle = arm_.unprofiled_goal(2, 0);
  status->wrist.unprofiled_goal_angular_velocity = arm_.unprofiled_goal(3, 0);
  status->wrist.estimator_state = arm_.WristEstimatorState();

  status->intake.angle = intake_.X_hat(0, 0);
  status->intake.angular_velocity = intake_.X_hat(1, 0);
  status->intake.goal_angle = intake_.goal(0, 0);
  status->intake.goal_angular_velocity = intake_.goal(1, 0);
  status->intake.unprofiled_goal_angle = intake_.unprofiled_goal(0, 0);
  status->intake.unprofiled_goal_angular_velocity =
      intake_.unprofiled_goal(1, 0);
  status->intake.estimator_state = intake_.IntakeEstimatorState();

  status->estopped = (state_ == ESTOP);

  status->state = state_;

  last_state_ = state_;
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016
