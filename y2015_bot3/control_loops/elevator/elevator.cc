#include "y2015_bot3/control_loops/elevator/elevator.h"

#include <cmath>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2015_bot3/control_loops/elevator/integral_elevator_motor_plant.h"

namespace y2015_bot3 {
namespace control_loops {

void SimpleCappedStateFeedbackLoop::CapU() {
  mutable_U(0, 0) = ::std::min(U(0, 0), max_voltage_);
  mutable_U(0, 0) = ::std::max(U(0, 0), -max_voltage_);
}

double SimpleCappedStateFeedbackLoop::UnsaturateOutputGoalChange() {
  // Compute K matrix to compensate for position errors.
  double Kp = K(0, 0);

  // Compute how much we need to change R in order to achieve the change in U
  // that was observed.
  return -(1.0 / Kp) * (U_uncapped() - U())(0, 0);
}

Elevator::Elevator(control_loops::ElevatorQueue *elevator)
    : aos::controls::ControlLoop<control_loops::ElevatorQueue>(elevator),
      loop_(new SimpleCappedStateFeedbackLoop(
          elevator::MakeIntegralElevatorLoop())),
      profile_(::aos::controls::kLoopFrequency) {}

bool Elevator::CheckZeroed() {
  return state_ == RUNNING;
}

void Elevator::Correct() {
  Eigen::Matrix<double, 1, 1> Y;
  Y << current_position();
  loop_->Correct(Y);
}

double Elevator::current_position() {
  return current_position_.encoder + offset_;
}

double Elevator::GetZeroingVelocity() {
  return zeroing_velocity_;
}

// If the hall_effect is true that means we need to move up until is false.
// Then we should move down.
double Elevator::FindZeroingVelocity() {
  if (glitch_filter_.filtered_value()) {
    zeroing_velocity_ = kZeroingSlowVelocity;
  } else {
    zeroing_velocity_ = -kZeroingVelocity;
  }

  return zeroing_velocity_;
}

double Elevator::UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}

void Elevator::SetOffset(double offset) {
  LOG(INFO, "Changing Elevator offset from %f to %f\n", offset_, offset);
  double doffset = offset - offset_;

  loop_->mutable_X_hat(0, 0) += doffset;

  // Modify the zeroing goal.
  goal_ += doffset;

  // Update the cached offset values to the actual values.
  offset_ = offset;
}

void Elevator::RunIteration(
    const control_loops::ElevatorQueue::Goal *unsafe_goal,
    const control_loops::ElevatorQueue::Position *position,
    control_loops::ElevatorQueue::Output *output,
    control_loops::ElevatorQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    state_ = UNINITIALIZED;
  }
  glitch_filter_.Update(position->bottom_hall_effect, position->encoder);

  // Bool to track if we should turn the motors on or not.
  bool disable = output == nullptr;

  // Save the current position so it can be used easily in the class.
  current_position_ = *position;

  if (state_ != UNINITIALIZED) {
    Correct();
  }

  switch (state_) {
    case UNINITIALIZED:
      LOG(DEBUG, "Uninitialized\n");
      // Startup.  Assume that we are at the origin everywhere.
      offset_ = -position->encoder;
      loop_->mutable_X_hat().setZero();
      LOG(INFO, "Initializing elevator offset to %f\n", offset_);
      Correct();
      state_ = ZEROING;
      disable = true;
      glitch_filter_.Reset(position->bottom_hall_effect);
      break;

    case ZEROING:
      LOG(DEBUG, "Zeroing elevator\n");

      if (glitch_filter_.negedge()) {
        state_ = RUNNING;
        SetOffset(-glitch_filter_.negedge_value() + kHallEffectPosition);
      }

      // We need to check FindZeroingVelocity() every time
      // in order for zeroing to work.
      {
        double zeroing_velocity_temp = FindZeroingVelocity();
        if (state_ != RUNNING && !disable) {
          // Move the elevator either up or down based on where the zeroing hall
          // effect is located.

          goal_velocity_ = zeroing_velocity_temp; 
          goal_ += goal_velocity_ * ::aos::controls::kLoopFrequency.ToSeconds();
        }
      }

      // Bypass motion profiles while we are zeroing.
      // This is also an important step right after the elevator is zeroed and
      // we reach into the elevator's state matrix and change it based on the
      // newly-obtained offset.
      {
        Eigen::Matrix<double, 2, 1> current;
        current.setZero();
        current << goal_, goal_velocity_;
        profile_.MoveCurrentState(current);
      }
      break;

    case RUNNING:
      if (unsafe_goal) {
        profile_.set_maximum_velocity(
            UseUnlessZero(unsafe_goal->max_velocity, 0.50));
        profile_.set_maximum_acceleration(
            UseUnlessZero(unsafe_goal->max_acceleration, 2.0));

        // Use the profiles to limit the elevator's movements.
        const double unfiltered_goal = ::std::max(
            ::std::min(unsafe_goal->height, kElevUpperLimit), kElevLowerLimit);
        ::Eigen::Matrix<double, 2, 1> goal_state =
            profile_.Update(unfiltered_goal, unsafe_goal->velocity);
        goal_ = goal_state(0, 0);
        goal_velocity_ = goal_state(1, 0);
      }

      if (state_ != RUNNING && state_ != ESTOP) {
        state_ = UNINITIALIZED;
      }
      break;

    case ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // Limit the goals so we can't exceed the hardware limits if we are RUNNING.
  if (state_ == RUNNING) {
    // Limit the elevator goal to min/max allowable heights.
    if (goal_ >= kElevUpperLimit) {
      LOG(WARNING, "Elevator goal above limit, %f > %f\n", goal_,
          kElevUpperLimit);
      goal_ = kElevUpperLimit;
    }

    if (goal_ <= kElevLowerLimit) {
      LOG(WARNING, "Elevator goal below limit, %f < %f\n", goal_,
          kElevLowerLimit);
      goal_ = kElevLowerLimit;
    }
  }

  // Check the hard limits.
  if (state_ == RUNNING) {
    if (current_position() >= kElevUpperHardLimit) {
      LOG(ERROR, "Elevator at %f out of bounds [%f, %f], ESTOPing\n",
          current_position(), kElevLowerHardLimit, kElevUpperHardLimit);
      if (output) {
        state_ = ESTOP;
      }
    }

    if (current_position() <= kElevLowerHardLimit) {
      LOG(ERROR, "Elevator at %f out of bounds [%f, %f], ESTOPing\n",
          current_position(), kElevLowerHardLimit, kElevUpperHardLimit);
      if (output) {
        state_ = ESTOP;
      }
    }
  }

  // Set the goals.
  loop_->mutable_R() << goal_, goal_velocity_, 0.0;

  const double max_voltage = state_ == RUNNING ? 12.0 : kZeroingVoltage;
  loop_->set_max_voltage(max_voltage);

  if (state_ == ESTOP) {
    disable = true;
  }
  loop_->Update(disable);

  if (state_ == ZEROING || state_ == RUNNING) {
    if (loop_->U() != loop_->U_uncapped()) {
      double deltaR = loop_->UnsaturateOutputGoalChange();

      // Move the elevator goal by the amount observed.
      LOG(WARNING, "Moving elevator goal by %f to handle saturation\n", deltaR);
      goal_ += deltaR;

      Eigen::Matrix<double, 2, 1> current;
      current.setZero();
      current << goal_, goal_velocity_;
      profile_.MoveCurrentState(current);
    }
  }

  if (output) {
    output->elevator = loop_->U(0, 0);
    if(unsafe_goal) {
      output->passive_support = unsafe_goal->passive_support;
      output->can_support = unsafe_goal->can_support;
    }
  }

  status->zeroed = state_ == RUNNING;

  status->height = loop_->X_hat(0, 0);
  status->velocity = loop_->X_hat(1, 0);

  status->goal_height = goal_;
  status->goal_velocity = goal_velocity_;

  status->estopped = (state_ == ESTOP);
  status->state = state_;
  status->has_tote = position->has_tote;
}

void GlitchFilter::Update(bool hall_effect, double encoder) {
  posedge_ = false;
  negedge_ = false;
  if (hall_effect != accepted_value_) {
    if (count_ == 0) {
      first_encoder_ = encoder;
    }
    ++count_;
  } else {
    last_encoder_ = encoder;
    count_ = 0;
  }
  if (count_ >= 2) {
    if (hall_effect) {
      posedge_ = true;
      posedge_value_ = (first_encoder_ + last_encoder_) / 2.0;
    } else {
      negedge_ = true;
      negedge_value_ = (first_encoder_ + last_encoder_) / 2.0;
    }
    accepted_value_ = hall_effect;
    count_ = 0;
  }
}

}  // namespace control_loops
}  // namespace y2015_bot3
