#include "y2016/control_loops/shooter/shooter.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "y2016/control_loops/shooter/shooter_plant.h"


namespace y2016 {
namespace control_loops {

ShooterSide::ShooterSide()
    : loop_(new StateFeedbackLoop<2, 1, 1>(
          ::y2016::control_loops::shooter::MakeShooterLoop())) {
  memset(history_, 0, sizeof(history_));
}

void ShooterSide::SetGoal(double angular_velocity_goal_uncapped) {
  angular_velocity_goal_ = std::min(angular_velocity_goal_uncapped,
                                    kMaxSpeed);
}

void ShooterSide::EstimatePositionTimestep() {
  // NULL position, so look at the loop.
  SetPosition(loop_->X_hat(0, 0));
}

void ShooterSide::SetPosition(double current_position) {
  current_position_ = current_position;

  // Track the current position if the velocity goal is small.
  if (angular_velocity_goal_ <= 1.0) position_goal_ = current_position_;

  // Update position in the model.
  Eigen::Matrix<double, 1, 1> Y;
  Y << current_position_;
  loop_->Correct(Y);

  // Prevents integral windup by limiting the position error such that the
  // error can't produce much more than full power.
  const double max_reference =
      (loop_->U_max(0, 0) -
       kAngularVelocityWeightScalar *
           (angular_velocity_goal_ - loop_->X_hat(1, 0)) * loop_->K(0, 1)) /
          loop_->K(0, 0) +
      loop_->X_hat(0, 0);
  const double min_reference =
      (loop_->U_min(0, 0) -
       kAngularVelocityWeightScalar *
           (angular_velocity_goal_ - loop_->X_hat(1, 0)) * loop_->K(0, 1)) /
          loop_->K(0, 0) +
      loop_->X_hat(0, 0);
  position_goal_ =
      ::std::max(::std::min(position_goal_, max_reference), min_reference);

  loop_->mutable_R() << position_goal_, angular_velocity_goal_;
  position_goal_ +=
      angular_velocity_goal_ * ::aos::controls::kLoopFrequency.ToSeconds();

  // Add the position to the history.
  history_[history_position_] = current_position_;
  history_position_ = (history_position_ + 1) % kHistoryLength;
}

const ShooterStatus ShooterSide::GetStatus() {
  // Calculate average over dt * kHistoryLength.
  int old_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;
  double avg_angular_velocity =
      (history_[old_history_position] - history_[history_position_]) /
      ::aos::controls::kLoopFrequency.ToSeconds() /
      static_cast<double>(kHistoryLength - 1);

  // Ready if average angular velocity is close to the goal.
  bool ready = (std::abs(angular_velocity_goal_ - avg_angular_velocity) <
                    kTolerance &&
                angular_velocity_goal_ != 0.0);

  return {avg_angular_velocity, ready};
}

double ShooterSide::GetOutput() {
  if (angular_velocity_goal_ < 1.0) {
    // Kill power at low angular velocities.
    return 0.0;
  }

  return loop_->U(0, 0);
}

void ShooterSide::UpdateLoop(bool output_is_null) {
  loop_->Update(output_is_null);
}

Shooter::Shooter(control_loops::ShooterQueue *my_shooter)
    : aos::controls::ControlLoop<control_loops::ShooterQueue>(my_shooter) {}

void Shooter::RunIteration(
    const control_loops::ShooterQueue::Goal *goal,
    const control_loops::ShooterQueue::Position *position,
    control_loops::ShooterQueue::Output *output,
    control_loops::ShooterQueue::Status *status) {
  if (goal) {
    // Update position/goal for our two shooter sides.
    left_.SetGoal(goal->angular_velocity_left);
    right_.SetGoal(goal->angular_velocity_right);

    if (position == nullptr) {
      left_.EstimatePositionTimestep();
      right_.EstimatePositionTimestep();
    } else {
      left_.SetPosition(position->theta_left);
      right_.SetPosition(position->theta_right);
    }
  }

  ShooterStatus status_left = left_.GetStatus();
  ShooterStatus status_right = right_.GetStatus();
  status->avg_angular_velocity_left = status_left.avg_angular_velocity;
  status->avg_angular_velocity_right = status_right.avg_angular_velocity;

  status->ready_left = status_left.ready;
  status->ready_right = status_right.ready;
  status->ready_both = (status_left.ready && status_right.ready);

  left_.UpdateLoop(output == nullptr);
  right_.UpdateLoop(output == nullptr);

  if (output) {
    output->voltage_left = left_.GetOutput();
    output->voltage_right = right_.GetOutput();
  }
}

}  // namespace control_loops
}  // namespace y2016
