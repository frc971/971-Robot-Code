#include "frc971/control_loops/wrist/wrist.h"

#include <stdio.h>

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/wrist/wrist_motor_plant.h"

namespace frc971 {
namespace control_loops {

WristMotor::WristMotor(control_loops::WristLoop *my_wrist)
    : aos::control_loops::ControlLoop<control_loops::WristLoop>(my_wrist),
      loop_(new WristStateFeedbackLoop(MakeWristLoop(), this)),
      state_(UNINITIALIZED),
      error_count_(0),
      zero_offset_(0.0) {
}

bool WristMotor::FetchConstants() {
  if (!constants::wrist_lower_limit(&wrist_lower_limit_)) {
    LOG(ERROR, "Failed to fetch the wrist lower limit constant.\n");
    return false;
  }
  if (!constants::wrist_upper_limit(&wrist_upper_limit_)) {
    LOG(ERROR, "Failed to fetch the wrist upper limit constant.\n");
    return false;
  }
  if (!constants::wrist_hall_effect_start_angle(
          &wrist_hall_effect_start_angle_)) {
    LOG(ERROR, "Failed to fetch the wrist start angle constant.\n");
    return false;
  }
  if (!constants::wrist_zeroing_speed(
          &wrist_zeroing_speed_)) {
    LOG(ERROR, "Failed to fetch the wrist zeroing speed constant.\n");
    return false;
  }

  return true;
}

double WristMotor::ClipGoal(double goal) const {
  return std::min(wrist_upper_limit_,
                  std::max(wrist_lower_limit_, goal));
}

const double kMaxZeroingVoltage = 5.0;

void WristMotor::WristStateFeedbackLoop::CapU() {
  if (wrist_motor_->state_ == READY) {
    if (Y(0, 0) >= wrist_motor_->wrist_upper_limit_) {
      U(0, 0) = std::min(0.0, U(0, 0));
    }
    if (Y(0, 0) <= wrist_motor_->wrist_lower_limit_) {
      U(0, 0) = std::max(0.0, U(0, 0));
    }
  }

  double limit = (wrist_motor_->state_ == READY) ? 12.0 : kMaxZeroingVoltage;

  U(0, 0) = std::min(limit, U(0, 0));
  U(0, 0) = std::max(-limit, U(0, 0));
}

// Positive angle is up, and positive power is up.
void WristMotor::RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::WristLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * /*status*/) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->voltage = 0;
  }

  // Cache the constants to avoid error handling down below.
  if (!FetchConstants()) {
    return;
  }

  // Uninitialize the bot if too many cycles pass without an encoder.
  if (position == NULL) {
    LOG(WARNING, "no new pos given\n");
    error_count_++;
  } else {
    error_count_ = 0;
  }
  if (error_count_ >= 4) {
    LOG(WARNING, "err_count is %d so forcing a re-zero\n", error_count_);
    state_ = UNINITIALIZED;
  }

  // Compute the absolute position of the wrist.
  double absolute_position;
  if (position) {
    absolute_position =
        position->pos + wrist_hall_effect_start_angle_;
    if (state_ == READY) {
      absolute_position -= zero_offset_;
    }
    loop_->Y << absolute_position;
    if (!position->hall_effect) {
      last_off_position_ = position->pos;
    }
  } else {
    // Dead recon for now.
    absolute_position = loop_->X_hat(0, 0);
  }

  switch (state_) {
    case UNINITIALIZED:
      if (position) {
        // Reset the zeroing goal.
        zeroing_position_ = absolute_position;
        // Clear the observer state.
        loop_->X_hat << absolute_position, 0.0;
        // Set the goal to here to make it so it doesn't move when disabled.
        loop_->R = loop_->X_hat;
        // Only progress if we are enabled.
        if (::aos::robot_state->enabled) {
          if (position->hall_effect) {
            state_ = MOVING_OFF;
          } else {
            state_ = ZEROING;
          }
        }
      }
      break;
    case MOVING_OFF:
      // Move off the hall effect sensor.
      if (!::aos::robot_state->enabled) {
        // Start over if disabled.
        state_ = UNINITIALIZED;
      } else if (position && !position->hall_effect) {
        // We are now off the sensor.  Time to zero now.
        state_ = ZEROING;
      } else {
        // Slowly creep off the sensor.
        zeroing_position_ -= wrist_zeroing_speed_ / 100;
        loop_->R << zeroing_position_, -wrist_zeroing_speed_;
        break;
      }
    case ZEROING:
      if (!::aos::robot_state->enabled) {
        // Start over if disabled.
        state_ = UNINITIALIZED;
      } else if (position && position->hall_effect) {
        state_ = READY;
        // Verify that the calibration number is between the last off position
        // and the current on position.  If this is not true, move off and try
        // again.
        if (position->calibration <= last_off_position_ ||
            position->calibration > position->pos) {
          LOG(ERROR, "Got a bogus calibration number.  Trying again.\n");
          LOG(ERROR,
              "Last off position was %f, current is %f, calibration is %f\n",
              last_off_position_, position->pos, position->calibration);
          state_ = MOVING_OFF;
        } else {
          // Save the zero, and then offset the observer to deal with the
          // phantom step change.
          const double old_zero_offset = zero_offset_;
          zero_offset_ = position->calibration;
          loop_->X_hat(0, 0) += old_zero_offset - zero_offset_;
          loop_->Y(0, 0) += old_zero_offset - zero_offset_;
        }
      } else {
        // Slowly creep towards the sensor.
        zeroing_position_ += wrist_zeroing_speed_ / 100;
        loop_->R << zeroing_position_, wrist_zeroing_speed_;
      }
      break;

    case READY:
      {
        const double limited_goal = ClipGoal(goal->goal);
        loop_->R << limited_goal, 0.0;
        break;
      }

    case ESTOP:
      LOG(WARNING, "have already given up\n");
      return;
  }

  // Update the observer.
  loop_->Update(position != NULL, output == NULL);

  // Verify that the zeroing goal hasn't run away.
  switch (state_) {
    case UNINITIALIZED:
    case READY:
    case ESTOP:
      // Not zeroing.  No worries.
      break;
    case MOVING_OFF:
    case ZEROING:
      // Check if we have cliped and adjust the goal.
      if (loop_->U_uncapped(0, 0) > kMaxZeroingVoltage) {
        double dx = (loop_->U_uncapped(0, 0) -
                     kMaxZeroingVoltage) / loop_->K(0, 0);
        zeroing_position_ -= dx;
      } else if(loop_->U_uncapped(0, 0) < -kMaxZeroingVoltage) {
        double dx = (loop_->U_uncapped(0, 0) +
                     kMaxZeroingVoltage) / loop_->K(0, 0);
        zeroing_position_ -= dx;
      }
      break;
  }

  if (position) {
    LOG(DEBUG, "pos=%f zero=%f currently %f hall: %s\n",
        position->pos, zero_offset_, absolute_position,
        position->hall_effect ? "true" : "false");
  }

  if (output) {
    output->voltage = loop_->U(0, 0);
  }
}

}  // namespace control_loops
}  // namespace frc971
