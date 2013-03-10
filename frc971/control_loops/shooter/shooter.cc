#include "frc971/control_loops/shooter/shooter.h"

#include "aos/aos_core.h"

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/shooter/shooter_motor_plant.h"

namespace frc971 {
namespace control_loops {

ShooterMotor::ShooterMotor(control_loops::ShooterLoop *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterLoop>(my_shooter),
    loop_(new StateFeedbackLoop<2, 1, 1>(MakeShooterLoop())),
    history_position_(0),
    position_goal_(0.0),
    last_position_(0.0) {
  memset(history_, 0, sizeof(history_));
}

/*static*/ const double ShooterMotor::dt = 0.01;
/*static*/ const double ShooterMotor::kMaxSpeed =
    10000.0 * (2.0 * M_PI) / 60.0 * 15.0 / 34.0;

void ShooterMotor::RunIteration(
    const control_loops::ShooterLoop::Goal *goal,
    const control_loops::ShooterLoop::Position *position,
    ::aos::control_loops::Output *output,
    control_loops::ShooterLoop::Status *status) {
  const double velocity_goal = std::min(goal->velocity, kMaxSpeed);
  const double current_position = 
      (position == NULL ? loop_->X_hat[0] : position->position);
  double output_voltage = 0.0;

  // Track the current position if the velocity goal is small.
  if (velocity_goal <= 1.0) {
    position_goal_ = current_position;
  }

  loop_->Y << current_position;

  // Add the position to the history.
  history_[history_position_] = current_position;
  history_position_ = (history_position_ + 1) % kHistoryLength;

  // Prevents integral windup by limiting the position error such that the
  // error can't produce much more than full power.
  const double kVelocityWeightScalar = 0.35;
  const double max_reference =
      (loop_->plant.U_max[0] - kVelocityWeightScalar * 
       (velocity_goal - loop_->X_hat[1]) * loop_->K[1])
      / loop_->K[0] + loop_->X_hat[0];
  const double min_reference =
      (loop_->plant.U_min[0] - kVelocityWeightScalar * 
       (velocity_goal - loop_->X_hat[1]) * loop_->K[1]) 
      / loop_->K[0] + loop_->X_hat[0];

  position_goal_ = ::std::max(::std::min(position_goal_, max_reference),
                              min_reference);
  loop_->R << position_goal_, velocity_goal;
  position_goal_ += velocity_goal * dt;
  
  loop_->Update(position, output == NULL);

  // Kill power at low velocity goals.
  if (velocity_goal < 1.0) {
    loop_->U[0] = 0.0;
  } else {
    output_voltage = loop_->U[0];
  }

  LOG(DEBUG,
      "PWM: %f, raw_pos: %f rotations: %f "
      "junk velocity: %f, xhat[0]: %f xhat[1]: %f, R[0]: %f R[1]: %f\n",
      output_voltage, current_position,
      current_position / (2 * M_PI),
      (current_position - last_position_) / dt,
      loop_->X_hat[0], loop_->X_hat[1], loop_->R[0], loop_->R[1]);

  // Calculates the velocity over the last kHistoryLength * .01 seconds
  // by taking the difference between the current and next history positions.
  int old_history_position = ((history_position_ == 0) ?
        kHistoryLength : history_position_) - 1;
  average_velocity_ = (history_[old_history_position] -
      history_[history_position_]) * 100.0 / (double)(kHistoryLength - 1);

  status->average_velocity = average_velocity_;

  // Determine if the velocity is close enough to the goal to be ready.
  if (std::abs(velocity_goal - average_velocity_) < 10.0 &&
      velocity_goal != 0.0) {
    LOG(DEBUG, "Steady: ");
    status->ready = true;
  } else {
    LOG(DEBUG, "Not ready: ");
    status->ready = false;
  }
  LOG(DEBUG, "avg = %f goal = %f\n", average_velocity_, velocity_goal);
  
  last_position_ = current_position;

  if (output) {
    output->voltage = output_voltage;
  }
}

}  // namespace control_loops
}  // namespace frc971
