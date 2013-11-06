#include "bot3/control_loops/shooter/shooter.h"

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "bot3/control_loops/shooter/shooter_motor_plant.h"

namespace bot3 {
namespace control_loops {

ShooterMotor::ShooterMotor(control_loops::ShooterLoop *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterLoop>(my_shooter),
    loop_(new StateFeedbackLoop<1, 1, 1>(MakeShooterLoop())),
    last_velocity_goal_(0) {
    loop_->Reset();
    U_add = loop_->B().inverse() * (loop_->A().Identity() - loop_->A());
}

/*static*/ const double ShooterMotor::dt = 0.01;

void ShooterMotor::RunIteration(
    const control_loops::ShooterLoop::Goal *goal,
    const control_loops::ShooterLoop::Position *position,
    ::aos::control_loops::Output *output,
    control_loops::ShooterLoop::Status *status) {
  double velocity_goal = goal->velocity;
  // Our position here is actually a velocity.
  average_velocity_ =
      (position == NULL ? loop_->X_hat(0, 0) : position->position);
  double output_voltage = 0.0;

// TODO (danielp): This must be modified for our index.
/*  if (index_loop.status.FetchLatest() || index_loop.status.get()) {
    if (index_loop.status->is_shooting) {
      if (velocity_goal != last_velocity_goal_ &&
          velocity_goal < 130) {
        velocity_goal = last_velocity_goal_;
      }
    }
  } else {
    LOG(WARNING, "assuming index isn't shooting\n");
  }*/
  last_velocity_goal_ = velocity_goal;

  loop_->Y << average_velocity_;
  loop_->R << velocity_goal;

  loop_->Update(position, output == NULL, U_add * loop_->R);

  // Kill power at low velocity goals.
  if (velocity_goal < 1.0) {
    loop_->U(0) = 0.0;
  } else {
    output_voltage = loop_->U(0);
  }

  LOG(DEBUG,
      "PWM: %f, raw_velocity: %f,  xhat[0]: %f, R[0]: %f\n",
      output_voltage, average_velocity_, loop_->X_hat[0], loop_->R[0]);

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
  
  if (output) {
    output->voltage = output_voltage;
  }
}

}  // namespace control_loops
}  // namespace bot3
