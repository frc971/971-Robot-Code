#include "y2016/control_loops/shooter/shooter.h"

#include <chrono>

#include "aos/logging/logging.h"
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

double ShooterSide::voltage() const { return loop_->U(0, 0); }

void ShooterSide::Update(bool disabled) {
  loop_->mutable_R() = loop_->next_R();
  if (loop_->R(1, 0) < 1.0) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  loop_->Correct(Y_);
  loop_->Update(disabled);
}

flatbuffers::Offset<ShooterSideStatus> ShooterSide::SetStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  ShooterSideStatus::Builder shooter_side_status_builder(*fbb);
  // Compute the oldest point in the history.
  const int oldest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  // Compute the distance moved over that time period.
  const double avg_angular_velocity =
      (history_[oldest_history_position] - history_[history_position_]) /
      (::aos::time::DurationInSeconds(::aos::controls::kLoopFrequency) *
       static_cast<double>(kHistoryLength - 1));
  shooter_side_status_builder.add_avg_angular_velocity(avg_angular_velocity);

  shooter_side_status_builder.add_angular_velocity(loop_->X_hat(1, 0));

  // Ready if average angular velocity is close to the goal.
  shooter_side_status_builder.add_ready(
      (std::abs(loop_->next_R(1, 0) - avg_angular_velocity) < kTolerance &&
       loop_->next_R(1, 0) > 1.0));

  return shooter_side_status_builder.Finish();
}

Shooter::Shooter(::aos::EventLoop *event_loop, const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      shots_(0),
      last_pre_shot_timeout_(::aos::monotonic_clock::min_time) {}

void Shooter::RunIteration(const Goal *goal, const Position *position,
                           aos::Sender<Output>::Builder *output,
                           aos::Sender<Status>::Builder *status) {
  const ::aos::monotonic_clock::time_point monotonic_now =
      event_loop()->monotonic_now();
  if (goal) {
    // Update position/goal for our two shooter sides.
    left_.set_goal(goal->angular_velocity());
    right_.set_goal(goal->angular_velocity());
  }

  left_.set_position(position->theta_left());
  right_.set_position(position->theta_right());

  left_.Update(output == nullptr);
  right_.Update(output == nullptr);

  flatbuffers::Offset<ShooterSideStatus> left_status_offset =
      left_.SetStatus(status->fbb());
  flatbuffers::Offset<ShooterSideStatus> right_status_offset =
      right_.SetStatus(status->fbb());

  ShooterSideStatus *left_status =
      GetMutableTemporaryPointer(*status->fbb(), left_status_offset);
  ShooterSideStatus *right_status =
      GetMutableTemporaryPointer(*status->fbb(), right_status_offset);

  const bool ready = (left_status->ready() && right_status->ready());

  Status::Builder status_builder = status->MakeBuilder<Status>();
  status_builder.add_ready((left_status->ready() && right_status->ready()));
  status_builder.add_left(left_status_offset);
  status_builder.add_right(right_status_offset);

  if (output) {
    Output::Builder output_builder = output->MakeBuilder<Output>();

    output_builder.add_voltage_left(left_.voltage());
    output_builder.add_voltage_right(right_.voltage());
    // Turn the lights on if we are supposed to spin.
    if (goal) {
      bool lights_on = false;
      if (::std::abs(goal->angular_velocity()) > 0.0) {
        lights_on = true;
        if (goal->shooting_forwards()) {
          output_builder.add_forwards_flashlight(true);
          output_builder.add_backwards_flashlight(false);
        } else {
          output_builder.add_forwards_flashlight(false);
          output_builder.add_backwards_flashlight(true);
        }
      }
      if (goal->force_lights_on()) {
        lights_on = true;
      }
      output_builder.add_lights_on(lights_on);

      bool shoot = false;
      switch (state_) {
        case ShooterLatchState::PASS_THROUGH:
          if (goal->push_to_shooter()) {
            if (::std::abs(goal->angular_velocity()) > 10) {
              if (ready) {
                state_ = ShooterLatchState::WAITING_FOR_SPINDOWN;
                shoot = true;
              }
            } else {
              shoot = true;
            }
          }
          last_pre_shot_timeout_ = monotonic_now + chrono::seconds(1);
          break;
        case ShooterLatchState::WAITING_FOR_SPINDOWN:
          shoot = true;
          if (left_.velocity() < goal->angular_velocity() * 0.9 ||
              right_.velocity() < goal->angular_velocity() * 0.9) {
            state_ = ShooterLatchState::WAITING_FOR_SPINUP;
          }
          if (::std::abs(goal->angular_velocity()) < 10 ||
              last_pre_shot_timeout_ < monotonic_now) {
            state_ = ShooterLatchState::INCREMENT_SHOT_COUNT;
          }
          break;
        case ShooterLatchState::WAITING_FOR_SPINUP:
          shoot = true;
          if (left_.velocity() > goal->angular_velocity() * 0.95 &&
              right_.velocity() > goal->angular_velocity() * 0.95) {
            state_ = ShooterLatchState::INCREMENT_SHOT_COUNT;
          }
          if (::std::abs(goal->angular_velocity()) < 10 ||
              last_pre_shot_timeout_ < monotonic_now) {
            state_ = ShooterLatchState::INCREMENT_SHOT_COUNT;
          }
          break;
        case ShooterLatchState::INCREMENT_SHOT_COUNT:
          ++shots_;
          state_ = ShooterLatchState::WAITING_FOR_SHOT_NEGEDGE;
          break;
        case ShooterLatchState::WAITING_FOR_SHOT_NEGEDGE:
          shoot = true;
          if (!goal->push_to_shooter()) {
            state_ = ShooterLatchState::PASS_THROUGH;
          }
          break;
      }

      output_builder.add_clamp_open(goal->clamp_open());
      output_builder.add_push_to_shooter(shoot);
    }

    output->Send(output_builder.Finish());
  }

  status_builder.add_shots(shots_);

  status->Send(status_builder.Finish());
}

}  // namespace shooter
}  // namespace control_loops
}  // namespace y2016
