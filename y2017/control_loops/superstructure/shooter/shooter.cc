#include "y2017/control_loops/superstructure/shooter/shooter.h"

#include <chrono>

#include "aos/logging/logging.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace shooter {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

namespace {
constexpr double kTolerance = 10.0;
}  // namespace

// TODO(austin): Pseudo current limit?

ShooterController::ShooterController()
    : loop_(new StateFeedbackLoop<4, 1, 1, double,
                                  StateFeedbackHybridPlant<4, 1, 1>,
                                  HybridKalman<4, 1, 1>>(
          superstructure::shooter::MakeIntegralShooterLoop())) {
  history_.fill(0);
  Y_.setZero();
}

void ShooterController::set_goal(double angular_velocity_goal) {
  loop_->mutable_next_R() << 0.0, angular_velocity_goal, angular_velocity_goal,
      0.0;
}

void ShooterController::set_position(double current_position) {
  // Update position in the model.
  Y_ << current_position;

  // Add the position to the history.
  history_[history_position_] = current_position;
  history_position_ = (history_position_ + 1) % kHistoryLength;

  dt_position_ = current_position - last_position_;
  last_position_ = current_position;
}

double ShooterController::voltage() const { return loop_->U(0, 0); }

void ShooterController::Reset() { reset_ = true; }

void ShooterController::Update(bool disabled, chrono::nanoseconds dt) {
  loop_->mutable_R() = loop_->next_R();
  if (::std::abs(loop_->R(2, 0)) < 1.0) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  loop_->Correct(Y_);

  // Compute the oldest point in the history.
  const int oldest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  // Compute the distance moved over that time period.
  average_angular_velocity_ =
      (history_[oldest_history_position] - history_[history_position_]) /
      (0.00505 * static_cast<double>(kHistoryLength - 1));

  // Ready if average angular velocity is close to the goal.
  error_ = average_angular_velocity_ - loop_->next_R(2, 0);

  ready_ = std::abs(error_) < kTolerance && loop_->next_R(2, 0) > 1.0;

  // If we are no longer ready, but were, and are spinning, then we shot a ball.
  // Reset the KF.
  if (last_ready_ && !ready_ && loop_->next_R(2, 0) > 1.0 && error_ < 0.0) {
    needs_reset_ = true;
    min_velocity_ = velocity();
  }
  if (needs_reset_) {
    min_velocity_ = ::std::min(min_velocity_, velocity());
    if (velocity() > min_velocity_ + 5.0) {
      reset_ = true;
      needs_reset_ = false;
    }
  }
  if (reset_) {
    // TODO(austin): I'd rather not be incrementing X_hat each time.  Sort out
    // something better.
    loop_->mutable_X_hat(3, 0) += 1.0;
    reset_ = false;
  }
  last_ready_ = ready_;

  X_hat_current_ = loop_->X_hat();
  position_error_ = X_hat_current_(0, 0) - Y_(0, 0);
  dt_velocity_ = dt_position_ / ::aos::time::DurationInSeconds(dt);
  fixed_dt_velocity_ = dt_position_ / 0.00505;

  loop_->Update(disabled, dt);
}

flatbuffers::Offset<ShooterStatus> ShooterController::BuildStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  ShooterStatus::Builder status_builder(*fbb);
  status_builder.add_avg_angular_velocity(average_angular_velocity_);

  status_builder.add_filtered_velocity(X_hat_current_(1, 0));
  status_builder.add_angular_velocity(X_hat_current_(2, 0));
  status_builder.add_ready(ready_);

  status_builder.add_voltage_error(X_hat_current_(3, 0));
  status_builder.add_position_error(position_error_);
  status_builder.add_instantaneous_velocity(dt_velocity_);
  status_builder.add_fixed_instantaneous_velocity(fixed_dt_velocity_);

  return status_builder.Finish();
}

void Shooter::Reset() { wheel_.Reset(); }

flatbuffers::Offset<ShooterStatus> Shooter::Iterate(
    const ShooterGoalT *goal, const double position,
    ::aos::monotonic_clock::time_point position_time, double *output,
    flatbuffers::FlatBufferBuilder *fbb) {
  if (goal) {
    // Update position/goal for our wheel.
    wheel_.set_goal(goal->angular_velocity);
  }

  wheel_.set_position(position);

  chrono::nanoseconds dt = ::aos::controls::kLoopFrequency;
  if (last_time_ != ::aos::monotonic_clock::min_time) {
    dt = position_time - last_time_;
  }
  last_time_ = position_time;

  wheel_.Update(output == nullptr, dt);

  flatbuffers::Offset<ShooterStatus> status_offset = wheel_.BuildStatus(fbb);

  if (last_ready_ && !wheel_.ready()) {
    min_ = wheel_.dt_velocity();
  } else if (!wheel_.ready()) {
    min_ = ::std::min(min_, wheel_.dt_velocity());
  } else if (!last_ready_ && wheel_.ready()) {
    AOS_LOG(INFO, "Shot min was [%f]\n", min_);
  }

  if (output) {
    *output = wheel_.voltage();
  }
  last_ready_ = wheel_.ready();

  return status_offset;
}

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
