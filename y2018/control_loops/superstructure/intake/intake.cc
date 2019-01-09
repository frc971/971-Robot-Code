#include "y2018/control_loops/superstructure/intake/intake.h"

#include <chrono>

#include "aos/commonmath.h"
#include "aos/logging/logging.h"
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

void IntakeController::SetStatus(IntakeSideStatus::Builder *status,
                                 const double *unsafe_goal) {
  status->add_goal_position(goal_angle(unsafe_goal));
  status->add_goal_velocity(loop_->R(1, 0));
  status->add_spring_position(loop_->X_hat(0) - loop_->X_hat(2));
  status->add_spring_velocity(loop_->X_hat(1) - loop_->X_hat(3));
  status->add_motor_position(loop_->X_hat(2));
  status->add_motor_velocity(loop_->X_hat(3));
  status->add_delayed_voltage(loop_->X_hat(4));
}

IntakeSide::IntakeSide(
    const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
        &zeroing_constants,
    const double spring_offset)
    : zeroing_estimator_(zeroing_constants), spring_offset_(spring_offset) {}

void IntakeSide::Reset() { state_ = State::UNINITIALIZED; }

flatbuffers::Offset<superstructure::IntakeSideStatus> IntakeSide::Iterate(
    const double *unsafe_goal,
    const superstructure::IntakeElasticSensors *position,
    superstructure::IntakeVoltageT *output,
    flatbuffers::FlatBufferBuilder *fbb) {
  zeroing_estimator_.UpdateEstimate(*position->motor_position());

  switch (state_) {
    case State::UNINITIALIZED:
      // Wait in the uninitialized state until the intake is initialized.
      AOS_LOG(DEBUG, "Uninitialized, waiting for intake\n");
      zeroing_estimator_.Reset();
      controller_.Reset();
      spring_unwrap_.Reset();
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
      if ((controller_.motor_position()) >
              controller_.intake_range().upper_hard ||
          (controller_.motor_position()) <
              controller_.intake_range().lower_hard) {
        AOS_LOG(ERROR, "Hit hard limits\n");
        state_ = State::ESTOP;
      }
      break;

    case State::ESTOP:
      AOS_LOG(ERROR, "Estop\n");
      break;
  }

  const bool disable = (output == nullptr) || state_ != State::RUNNING;
  controller_.set_position(spring_unwrap_.Unwrap(position->spring_angle()),
                           position->motor_position()->encoder());

  controller_.Update(disable, unsafe_goal);

  if (output) {
    output->voltage_elastic = controller_.voltage();
  }

  flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
      estimator_state = zeroing_estimator_.GetEstimatorState(fbb);

  superstructure::IntakeSideStatus::Builder status_builder(*fbb);
  // Save debug/internal state.
  status_builder.add_estimator_state(estimator_state);
  // Save the spring wrapped status.
  status_builder.add_spring_wrapped(spring_unwrap_.sensor_wrapped());

  controller_.SetStatus(&status_builder, unsafe_goal);
  status_builder.add_calculated_velocity(
      (zeroing_estimator_.offset() + position->motor_position()->encoder() -
       intake_last_position_) /
      controller_.kDt);
  status_builder.add_zeroed(zeroing_estimator_.zeroed());
  status_builder.add_estopped(estopped());
  status_builder.add_state(static_cast<int32_t>(state_));
  intake_last_position_ =
      zeroing_estimator_.offset() + position->motor_position()->encoder();

  return status_builder.Finish();
}

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
