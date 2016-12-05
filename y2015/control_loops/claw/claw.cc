#include "y2015/control_loops/claw/claw.h"

#include <algorithm>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2015/constants.h"
#include "y2015/control_loops/claw/claw_motor_plant.h"
#include "aos/common/util/trapezoid_profile.h"

namespace y2015 {
namespace control_loops {

using ::aos::time::Time;
namespace chrono = ::std::chrono;

constexpr double kZeroingVoltage = 4.0;

void ClawCappedStateFeedbackLoop::CapU() {
  mutable_U(0, 0) = ::std::min(mutable_U(0, 0), max_voltage_);
  mutable_U(0, 0) = ::std::max(mutable_U(0, 0), -max_voltage_);
}

double ClawCappedStateFeedbackLoop::UnsaturateOutputGoalChange() {
  // Compute K matrix to compensate for position errors.
  double Kp = K(0, 0);

  // Compute how much we need to change R in order to achieve the change in U
  // that was observed.
  return -(1.0 / Kp) * (U_uncapped() - U())(0, 0);
}

Claw::Claw(control_loops::ClawQueue *claw)
    : aos::controls::ControlLoop<control_loops::ClawQueue>(claw),
      last_piston_edge_(Time::Now()),
      claw_loop_(new ClawCappedStateFeedbackLoop(
          ::y2015::control_loops::claw::MakeClawLoop())),
      claw_estimator_(constants::GetValues().claw.zeroing),
      profile_(::aos::controls::kLoopFrequency) {}

void Claw::UpdateZeroingState() {
  if (claw_estimator_.offset_ratio_ready() < 1.0) {
    state_ = INITIALIZING;
  } else if (!claw_estimator_.zeroed()) {
    state_ = ZEROING;
  } else {
    state_ = RUNNING;
  }
}

void Claw::Correct() {
  Eigen::Matrix<double, 1, 1> Y;
  Y << claw_position();
  claw_loop_->Correct(Y);
}

void Claw::SetClawOffset(double offset) {
  LOG(INFO, "Changing claw offset from %f to %f.\n", claw_offset_, offset);
  const double doffset = offset - claw_offset_;

  // Adjust the height. The derivative should not need to be updated since the
  // speed is not changing.
  claw_loop_->mutable_X_hat(0, 0) += doffset;

  // Modify claw zeroing goal.
  claw_goal_ += doffset;
  // Update the cached offset value to the actual value.
  claw_offset_ = offset;
}

double Claw::estimated_claw_position() const {
  return current_position_.joint.encoder + claw_estimator_.offset();
}

double Claw::claw_position() const {
  return current_position_.joint.encoder + claw_offset_;
}

constexpr double kClawZeroingVelocity = 0.2;

double Claw::claw_zeroing_velocity() {
  const auto &values = constants::GetValues();

  // Zeroing will work as following. At startup, record the offset of the claw.
  // Then, start moving the claw towards where the index pulse should be. We
  // search around it a little, and if we don't find anything, we estop.
  // Otherwise, we're done.

  const double target_pos = values.claw.zeroing.measured_index_position;
  // How far away we need to stay from the ends of the range while zeroing.
  constexpr double zeroing_limit = 0.1375;
  // Keep the zeroing range within the bounds of the mechanism.
  const double zeroing_range =
      ::std::min(target_pos - values.claw.wrist.lower_limit - zeroing_limit,
                 values.claw.zeroing_range);

  if (claw_zeroing_velocity_ == 0) {
    if (estimated_claw_position() > target_pos) {
      claw_zeroing_velocity_ = -kClawZeroingVelocity;
    } else {
      claw_zeroing_velocity_ = kClawZeroingVelocity;
    }
  } else if (claw_zeroing_velocity_ > 0 &&
             estimated_claw_position() > target_pos + zeroing_range) {
    claw_zeroing_velocity_ = -kClawZeroingVelocity;
  } else if (claw_zeroing_velocity_ < 0 &&
             estimated_claw_position() < target_pos - zeroing_range) {
    claw_zeroing_velocity_ = kClawZeroingVelocity;
  }

  return claw_zeroing_velocity_;
}

void Claw::RunIteration(const control_loops::ClawQueue::Goal *unsafe_goal,
                        const control_loops::ClawQueue::Position *position,
                        control_loops::ClawQueue::Output *output,
                        control_loops::ClawQueue::Status *status) {
  const auto &values = constants::GetValues();

  if (WasReset()) {
    LOG(ERROR, "WPILib reset! Restarting.\n");
    claw_estimator_.Reset();
    state_ = UNINITIALIZED;
  }

  current_position_ = *position;

  // Bool to track if we should turn the motor on or not.
  bool disable = output == nullptr;
  double claw_goal_velocity = 0.0;

  claw_estimator_.UpdateEstimate(position->joint);

  if (state_ != UNINITIALIZED) {
    Correct();
  }

  switch (state_) {
    case UNINITIALIZED:
      LOG(INFO, "Uninitialized.\n");
      // Startup. Assume that we are at the origin.
      claw_offset_ = -position->joint.encoder;
      claw_loop_->mutable_X_hat().setZero();
      Correct();
      state_ = INITIALIZING;
      disable = true;
      break;

    case INITIALIZING:
      LOG(INFO, "Waiting for accurate initial position.\n");
      disable = true;
      // Update state_ to accurately represent the state of the zeroing
      // estimator.
      UpdateZeroingState();

      if (state_ != INITIALIZING) {
        // Set the goals to where we are now.
        claw_goal_ = claw_position();
      }
      break;

    case ZEROING:
      LOG(DEBUG, "Zeroing.\n");

      // Update state_.
      UpdateZeroingState();
      if (claw_estimator_.zeroed()) {
        LOG(INFO, "Zeroed!\n");
        SetClawOffset(claw_estimator_.offset());
      } else if (!disable) {
        claw_goal_velocity = claw_zeroing_velocity();
        claw_goal_ += claw_goal_velocity *
                      chrono::duration_cast<chrono::duration<double>>(
                          ::aos::controls::kLoopFrequency).count();
      }

      // Clear the current profile state if we are zeroing.
      {
        Eigen::Matrix<double, 2, 1> current;
        current.setZero();
        current << claw_goal_, claw_goal_velocity;
        profile_.MoveCurrentState(current);
      }
      break;

    case RUNNING:
      LOG(DEBUG, "Running!\n");

      // Update state_.
      UpdateZeroingState();
      if (unsafe_goal) {
        // Pick a set of sane defaults if none are specified.
        if (unsafe_goal->max_velocity != 0.0) {
          profile_.set_maximum_velocity(unsafe_goal->max_velocity);
        } else {
          profile_.set_maximum_velocity(2.5);
        }
        if (unsafe_goal->max_acceleration != 0.0) {
          profile_.set_maximum_acceleration(unsafe_goal->max_acceleration);
        } else {
          profile_.set_maximum_acceleration(4.0);
        }

        const double unfiltered_goal = ::std::max(
            ::std::min(unsafe_goal->angle, values.claw.wrist.upper_limit),
            values.claw.wrist.lower_limit);
        ::Eigen::Matrix<double, 2, 1> goal_state =
            profile_.Update(unfiltered_goal, unsafe_goal->angular_velocity);
        claw_goal_ = goal_state(0, 0);
        claw_goal_velocity = goal_state(1, 0);
      }

      if (state_ != RUNNING && state_ != ESTOP) {
        state_ = UNINITIALIZED;
      }
      break;

    case ESTOP:
      LOG(ERROR, "Estop!\n");
      disable = true;
      break;
  }

  // Make sure goal and position do not exceed the hardware limits if we are
  // RUNNING.
  if (state_ == RUNNING) {
    // Limit goal.
    claw_goal_ = ::std::min(claw_goal_, values.claw.wrist.upper_limit);
    claw_goal_ = ::std::max(claw_goal_, values.claw.wrist.lower_limit);

    // Check position.
    if (claw_position() >= values.claw.wrist.upper_hard_limit ||
        claw_position() <= values.claw.wrist.lower_hard_limit) {
      LOG(ERROR, "Claw at %f out of bounds [%f, %f].\n", claw_position(),
          values.claw.wrist.lower_limit, values.claw.wrist.upper_limit);
    }
  }

  // Set the goals.
  claw_loop_->mutable_R() << claw_goal_, claw_goal_velocity;

  const double max_voltage = (state_ == RUNNING) ? 12.0 : kZeroingVoltage;
  claw_loop_->set_max_voltage(max_voltage);

  if (state_ == ESTOP) {
    disable = true;
  }
  claw_loop_->Update(disable);

  if (state_ == INITIALIZING || state_ == ZEROING) {
    if (claw_loop_->U() != claw_loop_->U_uncapped()) {
      double deltaR = claw_loop_->UnsaturateOutputGoalChange();

      // Move the claw goal by the amount observed.
      LOG(WARNING, "Moving claw goal by %f to handle saturation.\n", deltaR);
      claw_goal_ += deltaR;
    }
  }

  if (output) {
    output->voltage = claw_loop_->U(0, 0);
    if (state_ != RUNNING) {
      output->intake_voltage = 0.0;
      output->rollers_closed = false;
    } else {
      if (unsafe_goal) {
        output->intake_voltage = unsafe_goal->intake;
        output->rollers_closed = unsafe_goal->rollers_closed;
      } else {
        output->intake_voltage = 0.0;
        output->rollers_closed = false;
      }
    }
    if (output->rollers_closed != last_rollers_closed_) {
      last_piston_edge_ = Time::Now();
    }
  }

  status->zeroed = state_ == RUNNING;
  status->estopped = state_ == ESTOP;
  status->state = state_;
  ::frc971::zeroing::PopulateEstimatorState(claw_estimator_,
                                            &status->zeroing_state);

  status->angle = claw_loop_->X_hat(0, 0);
  status->angular_velocity = claw_loop_->X_hat(1, 0);

  if (output) {
    status->intake = output->intake_voltage;
  } else {
    status->intake = 0;
  }
  status->goal_angle = claw_goal_;
  status->goal_velocity = claw_goal_velocity;

  if (output) {
    status->rollers_open = !output->rollers_closed &&
                           ((Time::Now() - last_piston_edge_).ToSeconds() >=
                            values.claw.piston_switch_time);
    status->rollers_closed = output->rollers_closed &&
                             ((Time::Now() - last_piston_edge_).ToSeconds() >=
                              values.claw.piston_switch_time);
  } else {
    status->rollers_open = false;
    status->rollers_closed = false;
  }

  if (output) {
    last_rollers_closed_ = output->rollers_closed;
  }
}

}  // namespace control_loops
}  // namespace y2015
