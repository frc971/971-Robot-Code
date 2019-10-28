#include "y2018/control_loops/superstructure/intake/intake.h"

#include <chrono>

#include "aos/commonmath.h"
#include "aos/controls/control_loops.q.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"

#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/intake/intake_delayed_plant.h"
#include "y2018/control_loops/superstructure/intake/intake_plant.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace intake {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

constexpr double IntakeController::kDt;

IntakeController::IntakeController()
    : loop_(new StateFeedbackLoop<5, 1, 2, double, StateFeedbackPlant<5, 1, 2>,
                                  StateFeedbackObserver<5, 1, 2>>(
          superstructure::intake::MakeDelayedIntakeLoop())),
      intake_range_(::y2018::constants::Values::kIntakeRange()) {
  Y_.setZero();
}

void IntakeController::set_position(double spring_angle,
                                    double output_position) {
  // Update position in the model.
  Y_ << spring_angle, (output_position + offset_);
}

double IntakeController::voltage() const { return loop_->U(0, 0); }

void IntakeController::Reset() { reset_ = true; }

void IntakeController::UpdateOffset(double offset) {
  const double doffset = offset - offset_;
  offset_ = offset;

  loop_->mutable_X_hat(0) += doffset;
  loop_->mutable_X_hat(2) += doffset;
}

double IntakeController::goal_angle(const double *unsafe_goal) {
  if (unsafe_goal == nullptr) {
    return 0;
  } else {
    return ::aos::Clip(*unsafe_goal, intake_range_.lower, intake_range_.upper);
  }
}

void IntakeController::Update(bool disabled, const double *unsafe_goal) {
  if (reset_) {
    loop_->mutable_X_hat().setZero();
    loop_->mutable_X_hat(0) = Y_(0) + Y_(1);
    loop_->mutable_X_hat(2) = Y_(1);
    reset_ = false;
  }

  double goal_velocity;
  loop_->Correct(Y_);

  if (unsafe_goal == nullptr) {
    disabled = true;
    goal_velocity = 0.0;
  } else {
    goal_velocity = ::aos::Clip(
        ((goal_angle(unsafe_goal) - loop_->X_hat(0, 0)) * 12.0), -16.0, 16.0);
  }
  // Computes the goal.
  loop_->mutable_R() << 0.0, goal_velocity, 0.0, goal_velocity,
      (goal_velocity / (kGearRatio * kMotorVelocityConstant));

  loop_->Update(disabled);
}

void IntakeController::SetStatus(IntakeSideStatus *status,
                                 const double *unsafe_goal) {
  status->goal_position = goal_angle(unsafe_goal);
  status->goal_velocity = loop_->R(1, 0);
  status->spring_position = loop_->X_hat(0) - loop_->X_hat(2);
  status->spring_velocity = loop_->X_hat(1) - loop_->X_hat(3);
  status->motor_position = loop_->X_hat(2);
  status->motor_velocity = loop_->X_hat(3);
  status->delayed_voltage = loop_->X_hat(4);
}

IntakeSide::IntakeSide(
    const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
        &zeroing_constants)
    : zeroing_estimator_(zeroing_constants) {}

void IntakeSide::Reset() { state_ = State::UNINITIALIZED; }

void IntakeSide::Iterate(const double *unsafe_goal,
                         const control_loops::IntakeElasticSensors *position,
                         control_loops::IntakeVoltage *output,
                         control_loops::IntakeSideStatus *status) {
  zeroing_estimator_.UpdateEstimate(position->motor_position);

  switch (state_) {
    case State::UNINITIALIZED:
      // Wait in the uninitialized state until the intake is initialized.
      AOS_LOG(DEBUG, "Uninitialized, waiting for intake\n");
      zeroing_estimator_.Reset();
      controller_.Reset();
      state_ = State::ZEROING;
      break;

    case State::ZEROING:
      // Zero by not moving.
      if (zeroing_estimator_.zeroed()) {
        AOS_LOG(INFO, "Now zeroed\n");
        controller_.UpdateOffset(zeroing_estimator_.offset());
        state_ = State::RUNNING;
      }
      break;

    case State::RUNNING:
      if (!(zeroing_estimator_.zeroed())) {
        AOS_LOG(ERROR, "Zeroing estimator is no longer zeroed\n");
        state_ = State::UNINITIALIZED;
      }
      if (zeroing_estimator_.error()) {
        AOS_LOG(ERROR, "Zeroing estimator error\n");
        state_ = State::UNINITIALIZED;
      }
      // ESTOP if we hit the hard limits.
      if ((status->motor_position) > controller_.intake_range_.upper ||
          (status->motor_position) < controller_.intake_range_.lower) {
        AOS_LOG(ERROR, "Hit hard limits\n");
        state_ = State::ESTOP;
      }
      break;

    case State::ESTOP:
      AOS_LOG(ERROR, "Estop\n");
      break;
  }

  const bool disable = (output == nullptr) || state_ != State::RUNNING;
  controller_.set_position(position->spring_angle,
                           position->motor_position.encoder);

  controller_.Update(disable, unsafe_goal);

  if (output) {
    output->voltage_elastic = controller_.voltage();
  }

  // Save debug/internal state.
  status->estimator_state = zeroing_estimator_.GetEstimatorState();
  controller_.SetStatus(status, unsafe_goal);
  status->calculated_velocity =
      (status->estimator_state.position - intake_last_position_) /
      controller_.kDt;
  status->zeroed = zeroing_estimator_.zeroed();
  status->estopped = (state_ == State::ESTOP);
  status->state = static_cast<int32_t>(state_);
  intake_last_position_ = status->estimator_state.position;
}

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
