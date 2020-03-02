#include "y2020/control_loops/superstructure/shooter/flywheel_controller.h"

#include <chrono>

#include "aos/logging/logging.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

FlywheelController::FlywheelController(
    StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                      HybridKalman<3, 1, 1>> &&loop)
    : loop_(new StateFeedbackLoop<3, 1, 1, double,
                                  StateFeedbackHybridPlant<3, 1, 1>,
                                  HybridKalman<3, 1, 1>>(std::move(loop))) {
  history_.fill(std::pair<double, ::aos::monotonic_clock::time_point>(
      0, ::aos::monotonic_clock::epoch()));
  Y_.setZero();
}

void FlywheelController::set_goal(double angular_velocity_goal) {
  loop_->mutable_next_R() << 0.0, angular_velocity_goal, 0.0;
  last_goal_ = angular_velocity_goal;
}

void FlywheelController::set_position(
    double current_position,
    const aos::monotonic_clock::time_point position_timestamp) {
  // Project time forwards.
  const int newest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  if (!first_) {
    loop_->UpdateObserver(
        loop_->U(),
        position_timestamp - std::get<1>(history_[newest_history_position]));
  } else {
    first_ = false;
  }

  // Update position in the model.
  Y_ << current_position;

  // Add the position to the history.
  history_[history_position_] =
      std::pair<double, ::aos::monotonic_clock::time_point>(current_position,
                                                            position_timestamp);
  history_position_ = (history_position_ + 1) % kHistoryLength;

  loop_->Correct(Y_);
}

double FlywheelController::voltage() const { return loop_->U(0, 0); }

void FlywheelController::Update(bool disabled) {
  loop_->mutable_R() = loop_->next_R();
  if (loop_->R(1, 0) < 1.0) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  loop_->UpdateController(disabled);
}

flatbuffers::Offset<FlywheelControllerStatus> FlywheelController::SetStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  // Compute the oldest point in the history.
  const int oldest_history_position = history_position_;
  const int newest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  const double total_loop_time = ::aos::time::DurationInSeconds(
      std::get<1>(history_[newest_history_position]) -
      std::get<1>(history_[oldest_history_position]));

  const double distance_traveled =
      std::get<0>(history_[newest_history_position]) -
      std::get<0>(history_[oldest_history_position]);

  // Compute the distance moved over that time period.
  avg_angular_velocity_ = (distance_traveled) / (total_loop_time);

  FlywheelControllerStatusBuilder builder(*fbb);

  builder.add_avg_angular_velocity(avg_angular_velocity_);
  builder.add_angular_velocity(loop_->X_hat(1, 0));
  builder.add_voltage_error(loop_->X_hat(2, 0));
  builder.add_angular_velocity_goal(last_goal_);
  return builder.Finish();
}

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
