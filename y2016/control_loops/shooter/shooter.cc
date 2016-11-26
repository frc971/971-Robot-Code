#include "y2016/control_loops/shooter/shooter.h"

#include <chrono>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "y2016/control_loops/shooter/shooter_plant.h"


namespace y2016 {
namespace control_loops {
namespace shooter {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

// TODO(austin): Pseudo current limit?

ShooterSide::ShooterSide()
    : loop_(new StateFeedbackLoop<3, 1, 1>(MakeIntegralShooterLoop())) {
  history_.fill(0);
  Y_.setZero();
}

void ShooterSide::set_goal(double angular_velocity_goal) {
  loop_->mutable_next_R() << 0.0, angular_velocity_goal, 0.0;
}

void ShooterSide::set_position(double current_position) {
  // Update position in the model.
  Y_ << current_position;

  // Add the position to the history.
  history_[history_position_] = current_position;
  history_position_ = (history_position_ + 1) % kHistoryLength;
}

double ShooterSide::voltage() const {
  return loop_->U(0, 0);
}

void ShooterSide::Update(bool disabled) {
  loop_->mutable_R() = loop_->next_R();
  if (loop_->R(1, 0) < 1.0) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  loop_->Correct(Y_);
  loop_->Update(disabled);
}

void ShooterSide::SetStatus(ShooterSideStatus *status) {
  // Compute the oldest point in the history.
  const int oldest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  // Compute the distance moved over that time period.
  status->avg_angular_velocity =
      (history_[oldest_history_position] - history_[history_position_]) /
      (chrono::duration_cast<chrono::duration<double>>(
           ::aos::controls::kLoopFrequency).count() *
       static_cast<double>(kHistoryLength - 1));

  status->angular_velocity = loop_->X_hat(1, 0);

  // Ready if average angular velocity is close to the goal.
  status->ready = (std::abs(loop_->next_R(1, 0) -
                            status->avg_angular_velocity) < kTolerance &&
                   loop_->next_R(1, 0) > 1.0);
}

Shooter::Shooter(ShooterQueue *my_shooter)
    : aos::controls::ControlLoop<ShooterQueue>(my_shooter),
      shots_(0),
      last_pre_shot_timeout_(::aos::monotonic_clock::min_time) {}

void Shooter::RunIteration(const ShooterQueue::Goal *goal,
                           const ShooterQueue::Position *position,
                           ShooterQueue::Output *output,
                           ShooterQueue::Status *status) {
  if (goal) {
    // Update position/goal for our two shooter sides.
    left_.set_goal(goal->angular_velocity);
    right_.set_goal(goal->angular_velocity);

    // Turn the lights on if we are supposed to spin.
    if (output) {
      if (::std::abs(goal->angular_velocity) > 0.0) {
        output->lights_on = true;
        if (goal->shooting_forwards) {
          output->forwards_flashlight = true;
          output->backwards_flashlight = false;
        } else {
          output->forwards_flashlight = false;
          output->backwards_flashlight = true;
        }
      }
      if (goal->force_lights_on) {
        output->lights_on = true;
      }
    }
  }

  left_.set_position(position->theta_left);
  right_.set_position(position->theta_right);

  left_.Update(output == nullptr);
  right_.Update(output == nullptr);

  left_.SetStatus(&status->left);
  right_.SetStatus(&status->right);
  status->ready = (status->left.ready && status->right.ready);

  if (output) {
    output->voltage_left = left_.voltage();
    output->voltage_right = right_.voltage();

    if (goal) {
      bool shoot = false;
      switch (state_) {
        case ShooterLatchState::PASS_THROUGH:
          if (goal->push_to_shooter) {
            if (::std::abs(goal->angular_velocity) > 10) {
              if (status->ready) {
                state_ = ShooterLatchState::WAITING_FOR_SPINDOWN;
                shoot = true;
              }
            } else {
              shoot = true;
            }
          }
          last_pre_shot_timeout_ = monotonic_clock::now() + chrono::seconds(1);
          break;
        case ShooterLatchState::WAITING_FOR_SPINDOWN:
          shoot = true;
          if (left_.velocity() < goal->angular_velocity * 0.9 ||
              right_.velocity() < goal->angular_velocity * 0.9) {
            state_ = ShooterLatchState::WAITING_FOR_SPINUP;
          }
          if (::std::abs(goal->angular_velocity) < 10 ||
              last_pre_shot_timeout_ < monotonic_clock::now()) {
            state_ = ShooterLatchState::INCREMENT_SHOT_COUNT;
          }
          break;
        case ShooterLatchState::WAITING_FOR_SPINUP:
          shoot = true;
          if (left_.velocity() > goal->angular_velocity * 0.95 &&
              right_.velocity() > goal->angular_velocity * 0.95) {
            state_ = ShooterLatchState::INCREMENT_SHOT_COUNT;
          }
          if (::std::abs(goal->angular_velocity) < 10 ||
              last_pre_shot_timeout_ < monotonic_clock::now()) {
            state_ = ShooterLatchState::INCREMENT_SHOT_COUNT;
          }
          break;
        case ShooterLatchState::INCREMENT_SHOT_COUNT:
          ++shots_;
          state_ = ShooterLatchState::WAITING_FOR_SHOT_NEGEDGE;
          break;
        case ShooterLatchState::WAITING_FOR_SHOT_NEGEDGE:
          shoot = true;
          if (!goal->push_to_shooter) {
            state_ = ShooterLatchState::PASS_THROUGH;
          }
          break;
      }

      output->clamp_open = goal->clamp_open;
      output->push_to_shooter = shoot;
    }
  }

  status->shots = shots_;
}

}  // namespace shooter
}  // namespace control_loops
}  // namespace y2016
