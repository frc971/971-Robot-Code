#include "frc971/control_loops/shooter.h"

#include "aos/aos_core.h"

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/shooter_motor_plant.h"

namespace frc971 {
namespace control_loops {

namespace {
// Motor controllers have a range of PWM values where they don't trigger,
// and we use these values to avoid that.
// TODO: find these values for the Talons.
static const double positive_deadband_power = 0.0;
static const double negative_deadband_power = 0.0;
}

ShooterMotor::ShooterMotor(control_loops::ShooterLoop *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterLoop>(my_shooter),
    loop_(new StateFeedbackLoop<2, 1, 1>(MakeShooterLoop())),
    history_position_(0),
    position_goal_(0.0),
    time_(0.0) {
  memset(history, 0, sizeof(history));
  // Creates the header for the data file. Overwrites anything already there.
  loop_->StartDataFile("shooter.csv");
}

void ShooterMotor::RunIteration(
    const control_loops::ShooterLoop::Goal *goal,
    const control_loops::ShooterLoop::Position *position,
    ::aos::control_loops::Output *output,
    control_loops::ShooterLoop::Status *status) {

  bool bad_pos = false;
  if (position == NULL) {
    LOG(WARNING, "no position given\n");
    bad_pos = true;
  }

  const double velocity_goal = std::min(goal->velocity, max_speed);
  static double last_position = 0.0;
  double output_voltage = 0.0;

  if (!bad_pos) {
    static bool first_time = true;
    if (first_time) {
      position_goal_ = position->position;
      first_time = false;
    }

    // Add position to the history.
    history[history_position_] = position->position;
    history_position_ = (history_position_ + 1) % kHistoryLength;

    loop_->Y << position->position;
    // Prevents the position from going out of the bounds 
    // where the control loop will run the motor at full power.
    const double velocity_weight_scalar = 0.35;
    const double max_reference = (loop_->plant.U_max[0] - velocity_weight_scalar * 
                                 (velocity_goal - loop_->X_hat[1]) * loop_->K[1])
                                 / loop_->K[0] + loop_->X_hat[0];
    const double min_reference = (loop_->plant.U_min[0] - velocity_weight_scalar * 
                                 (velocity_goal - loop_->X_hat[1]) * loop_->K[1]) 
                                 / loop_->K[0] + loop_->X_hat[0];
   position_goal_ = std::max(std::min(position_goal_, max_reference), min_reference);
    loop_->R << position_goal_, velocity_goal;
    position_goal_ += velocity_goal * dt;
  }
  
  loop_->Update(!bad_pos, bad_pos || (output == NULL));
  // There is no need to make the motors actively assist the spin-down.
  if (velocity_goal < 1.0) {
    output_voltage = 0.0;
    // Also, reset the position incrementer to avoid accumulating error.
    position_goal_ = position->position;
  } else {
    output_voltage = loop_->U[0] / 12.0;
  }
  LOG(DEBUG,
      "PWM: %f, raw_pos: %f rotations: %f "
      "junk velocity: %f, xhat[0]: %f xhat[1]: %f, R[0]: %f R[1]: %f\n",
      output_voltage, position->position,
      position->position / (2 * M_PI),
      (position->position - last_position) / dt,
      loop_->X_hat[0], loop_->X_hat[1], loop_->R[0], loop_->R[1]);

//  aos::DriverStationDisplay::Send(2, "RPS%3.0f(%3.0f) RPM%4.0f",
//      (position->position - last_position) * 100.0, goal->goal,
//      (position->position - last_position) / (2.0 * M_PI) * 6000.0);

  // Calculates the velocity over the last kHistoryLength*.001 seconds
  // by taking the difference between the current and next history positions.
  int old_history_position = ((history_position_ == 0) ?
        kHistoryLength : history_position_) - 1;
  average_velocity_ = (history[old_history_position] 
      - history[history_position_]) * 100.0 / (double)(kHistoryLength - 1);
  status->average_velocity = average_velocity_;
  // Determines if the velocity is close enough to the goal to be ready.
  if (std::abs(velocity_goal - average_velocity_) < 10.0 &&
      velocity_goal != 0.0) {
    LOG(DEBUG, "Steady: ");
    status->ready = true;
  } else {
    LOG(DEBUG, "Not ready: ");
    status->ready = false;
  }
  LOG(DEBUG, "avg = %f goal = %f\n", average_velocity_, velocity_goal);
  
  // Deal with motor controller deadbands.
  if (output_voltage > 0) {
    output_voltage += positive_deadband_power;
  }
  if (output_voltage < 0) {
    output_voltage -= negative_deadband_power;
  }

  if (bad_pos) {
    last_position = position->position;
  } else {
    // use the predicted position
    last_position = loop_->X_hat[0];
  }
  if (output) {
    output->voltage = output_voltage;
  }

  // If anything is happening, record it. Otherwise, reset the time.
  // This should be removed once we are done with testing.
  if (output_voltage != 0 && average_velocity_ != 0 && velocity_goal != 0) {
    loop_->RecordDatum("shooter.csv", time_);
    time_ += dt;
  }
  else {
    time_ = 0;
  }
}  // RunIteration

}  // namespace control_loops
}  // namespace frc971
