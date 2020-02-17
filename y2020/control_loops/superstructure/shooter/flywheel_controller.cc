#include "y2020/control_loops/superstructure/shooter/flywheel_controller.h"

#include <chrono>

#include "aos/logging/logging.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

FlywheelController::FlywheelController(StateFeedbackLoop<3, 1, 1> &&loop)
    : loop_(new StateFeedbackLoop<3, 1, 1>(std::move(loop))) {
  history_.fill(0);
  Y_.setZero();
}

void FlywheelController::set_goal(double angular_velocity_goal) {
  loop_->mutable_next_R() << 0.0, angular_velocity_goal, 0.0;
  last_goal_ = angular_velocity_goal;
}

void FlywheelController::set_position(double current_position) {
  // Update position in the model.
  Y_ << current_position;

  // Add the position to the history.
  history_[history_position_] = current_position;
  history_position_ = (history_position_ + 1) % kHistoryLength;
}

double FlywheelController::voltage() const { return loop_->U(0, 0); }

void FlywheelController::Update(bool disabled) {
  loop_->mutable_R() = loop_->next_R();
  if (loop_->R(1, 0) < 1.0) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  loop_->Correct(Y_);
  loop_->Update(disabled);
}

flatbuffers::Offset<FlywheelControllerStatus> FlywheelController::SetStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  // Compute the oldest point in the history.
  const int oldest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  // Compute the distance moved over that time period.
  const double avg_angular_velocity =
      (history_[oldest_history_position] - history_[history_position_]) /
      (::aos::time::DurationInSeconds(::aos::controls::kLoopFrequency) *
       static_cast<double>(kHistoryLength - 1));

  FlywheelControllerStatusBuilder builder(*fbb);

  builder.add_avg_angular_velocity(avg_angular_velocity);
  builder.add_angular_velocity(loop_->X_hat(1, 0));
  builder.add_angular_velocity_goal(last_goal_);
  return builder.Finish();
}

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
