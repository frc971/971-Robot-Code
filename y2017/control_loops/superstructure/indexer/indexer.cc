#include "y2017/control_loops/superstructure/indexer/indexer.h"

#include <chrono>

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "y2017/control_loops/superstructure/indexer/indexer_integral_plant.h"
#include "y2017/control_loops/superstructure/indexer/stuck_indexer_integral_plant.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace indexer {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

namespace {
constexpr double kTolerance = 10.0;
constexpr chrono::milliseconds kForwardTimeout{500};
constexpr chrono::milliseconds kReverseTimeout{500};
constexpr chrono::milliseconds kReverseMinTimeout{100};
}  // namespace

// TODO(austin): Pseudo current limit?

IndexerController::IndexerController()
    : loop_(new StateFeedbackLoop<3, 1, 1>(
          superstructure::indexer::MakeIntegralIndexerLoop())),
      stuck_indexer_detector_(new StateFeedbackLoop<3, 1, 1>(
          superstructure::indexer::MakeStuckIntegralIndexerLoop())) {
  history_.fill(0);
  Y_.setZero();
  X_hat_current_.setZero();
  stuck_indexer_X_hat_current_.setZero();
}

void IndexerController::set_goal(double angular_velocity_goal) {
  loop_->mutable_next_R() << 0.0, angular_velocity_goal, 0.0;
}

void IndexerController::set_position(double current_position) {
  // Update position in the model.
  Y_ << current_position;

  // Add the position to the history.
  history_[history_position_] = current_position;
  history_position_ = (history_position_ + 1) % kHistoryLength;

  dt_velocity_ = (current_position - last_position_) /
                 chrono::duration_cast<chrono::duration<double>>(
                     ::aos::controls::kLoopFrequency)
                     .count();
  last_position_ = current_position;
}

double IndexerController::voltage() const { return loop_->U(0, 0); }

double IndexerController::StuckVoltage() const {
  const double applied_voltage = voltage() + loop_->X_hat(2, 0);
  if (applied_voltage < 0) {
    return +stuck_indexer_X_hat_current_(2, 0) + applied_voltage;
  } else {
    return -stuck_indexer_X_hat_current_(2, 0) - applied_voltage;
  }
}
bool IndexerController::IsStuck() const { return StuckVoltage() > 1.5; }

void IndexerController::Reset() { reset_ = true; }

void IndexerController::PartialReset() { loop_->mutable_X_hat(2, 0) = 0.0; }

void IndexerController::Update(bool disabled) {
  loop_->mutable_R() = loop_->next_R();
  if (::std::abs(loop_->R(1, 0)) < 0.1) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  if (reset_) {
    loop_->mutable_X_hat(0, 0) = Y_(0, 0);
    loop_->mutable_X_hat(1, 0) = 0.0;
    loop_->mutable_X_hat(2, 0) = 0.0;
    stuck_indexer_detector_->mutable_X_hat(0, 0) = Y_(0, 0);
    stuck_indexer_detector_->mutable_X_hat(1, 0) = 0.0;
    stuck_indexer_detector_->mutable_X_hat(2, 0) = 0.0;
    reset_ = false;
  }

  loop_->Correct(Y_);
  stuck_indexer_detector_->Correct(Y_);

  // Compute the oldest point in the history.
  const int oldest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  // Compute the distance moved over that time period.
  average_angular_velocity_ =
      (history_[oldest_history_position] - history_[history_position_]) /
      (chrono::duration_cast<chrono::duration<double>>(
           ::aos::controls::kLoopFrequency)
           .count() *
       static_cast<double>(kHistoryLength - 1));

  // Ready if average angular velocity is close to the goal.
  error_ = average_angular_velocity_ - loop_->next_R(1, 0);

  ready_ = std::abs(error_) < kTolerance && loop_->next_R(1, 0) > 1.0;

  X_hat_current_ = loop_->X_hat();
  stuck_indexer_X_hat_current_ = stuck_indexer_detector_->X_hat();
  position_error_ = X_hat_current_(0, 0) - Y_(0, 0);

  loop_->Update(disabled);
  stuck_indexer_detector_->UpdateObserver(loop_->U(),
                                          ::aos::controls::kLoopFrequency);
}

void IndexerController::SetStatus(IndexerStatus *status) {
  status->avg_angular_velocity = average_angular_velocity_;

  status->angular_velocity = X_hat_current_(1, 0);
  status->ready = ready_;

  status->voltage_error = X_hat_current_(2, 0);
  status->stuck_voltage_error = stuck_indexer_X_hat_current_(2, 0);
  status->position_error = position_error_;
  status->instantaneous_velocity = dt_velocity_;

  status->stuck = IsStuck();

  status->stuck_voltage = StuckVoltage();
}

void Indexer::Reset() { indexer_.Reset(); }

void Indexer::Iterate(const control_loops::IndexerGoal *goal,
                      const double *position, double *output,
                      control_loops::IndexerStatus *status) {
  if (goal) {
    // Start indexing at the suggested velocity.
    // If a "stuck" event is detected, reverse.  Stay reversed until either
    // unstuck, or 0.5 seconds have elapsed.
    // Then, start going forwards.  Don't detect stuck for 0.5 seconds.

    monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    switch (state_) {
      case State::RUNNING:
        // Pass the velocity goal through.
        indexer_.set_goal(goal->angular_velocity);
        // If we are stuck and weren't just reversing, try reversing to unstick
        // us.  We don't want to chatter back and forth too fast if reversing
        // isn't working.
        if (indexer_.IsStuck() &&
            monotonic_now > kForwardTimeout + last_transition_time_) {
          state_ = State::REVERSING;
          last_transition_time_ = monotonic_now;
          indexer_.Reset();
        }
        break;
      case State::REVERSING:
        // "Reverse" "slowly".
        indexer_.set_goal(-5.0 * aos::sign(goal->angular_velocity));

        // If we've timed out or are no longer stuck, try running again.
        if ((!indexer_.IsStuck() &&
             monotonic_now > last_transition_time_ + kReverseMinTimeout) ||
            monotonic_now > kReverseTimeout + last_transition_time_) {
          state_ = State::RUNNING;

          // Only reset if we got stuck going this way too.
          if (monotonic_now > kReverseTimeout + last_transition_time_) {
            indexer_.Reset();
          }
          last_transition_time_ = monotonic_now;
        }
        break;
    }
  }

  indexer_.set_position(*position);

  indexer_.Update(output == nullptr);

  indexer_.SetStatus(status);
  status->state = static_cast<int32_t>(state_);

  if (output) {
    *output = indexer_.voltage();
  }
}

}  // namespace indexer
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
