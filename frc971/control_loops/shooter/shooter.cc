#include "frc971/control_loops/shooter/shooter.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/shooter/shooter_motor_plant.h"

namespace frc971 {
namespace control_loops {

using ::aos::time::Time;

void ZeroedStateFeedbackLoop::CapU() {
  const double old_voltage = voltage_;
  voltage_ += U(0, 0);

  uncapped_voltage_ = voltage_;

  // TODO(ben): Limit the voltage if we are ever not certain that things are
  // working.
  double limit = 12.0;

  // Make sure that reality and the observer can't get too far off.  There is a
  // delay by one cycle between the applied voltage and X_hat(2, 0), so compare
  // against last cycle's voltage.
  if (X_hat(2, 0) > last_voltage_ + 2.0) {
    voltage_ -= X_hat(2, 0) - (last_voltage_ + 2.0);
    //LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  } else if (X_hat(2, 0) < last_voltage_ - 2.0) {
    voltage_ += X_hat(2, 0) - (last_voltage_ - 2.0);
    //LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  }

  voltage_ = std::min(limit, voltage_);
  voltage_ = std::max(-limit, voltage_);
  U(0, 0) = voltage_ - old_voltage;
  //LOG(DEBUG, "abc %f\n", X_hat(2, 0) - voltage_);
  //LOG(DEBUG, "error %f\n", X_hat(0, 0) - R(0, 0));

  last_voltage_ = voltage_;
}

ShooterMotor::ShooterMotor(control_loops::ShooterGroup *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterGroup>(my_shooter),
      shooter_(MakeShooterLoop()),
      calibration_position_(0.0),
      state_(STATE_INITIALIZE),
      loading_problem_end_time_(0, 0),
      shooter_brake_set_time_(0, 0),
      prepare_fire_end_time_(0, 0),
      shot_end_time_(0, 0),
      cycles_not_moved_(0),
      initial_loop_(true) {}

// Positive is out, and positive power is out.
void ShooterMotor::RunIteration(
    const control_loops::ShooterGroup::Goal *goal,
    const control_loops::ShooterGroup::Position *position,
    control_loops::ShooterGroup::Output *output,
    control_loops::ShooterGroup::Status *status) {
  constexpr double dt = 0.01;

  // we must always have these or we have issues.
  if (goal == NULL || status == NULL) {
    if (output) output->voltage = 0;
    LOG(ERROR, "Thought I would just check for null and die.\n");
    return;
  }

  if (initial_loop_) {
    // TODO(austin): If 'reset()', we are lost, start over.
    initial_loop_ = false;
    shooter_.SetPositionDirectly(position->position);
  } else {
    shooter_.SetPositionValues(position->position);
  }

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) output->voltage = 0;

  const frc971::constants::Values &values = constants::GetValues();

  double real_position = shooter_.position();
  double adjusted_position = shooter_.position() - calibration_position_;

  if (position) {
    last_position_ = *position;
    LOG(DEBUG,
        "pos > real: %.2f adjusted: %.2f raw: %.2f calib: %.2f state= %d\n",
        real_position, adjusted_position, position->position,
        calibration_position_, state_);
  }

  // Don't even let the control loops run.
  bool shooter_loop_disable = false;

  // Adds voltage to take up slack in gears before shot.
  bool apply_some_voltage = false;

  switch (state_) {
    case STATE_INITIALIZE:
      // Start off with the assumption that we are at the value
      // futhest back given our sensors.
      if (position && position->pusher_distal.current) {
        //TODO(ben): use posedge
        calibration_position_ =
            position->position - values.shooter.pusher_distal.lower_angle;
      } else if (position && position->pusher_proximal.current) {
        //TODO(ben): use posedge
        calibration_position_ =
            position->position - values.shooter.pusher_proximal.lower_angle;
      }

      state_ = STATE_REQUEST_LOAD;

      // Zero out initial goal.
      shooter_.SetGoalPosition(real_position, 0.0);
      if (position) {
        output->latch_piston = position->plunger;
      } else {
        // We don't know what is going on so just close the latch to be safe.
        output->latch_piston = true;
      }
      output->brake_piston = false;
      break;
    case STATE_REQUEST_LOAD:
      if (position->plunger && position->latch) {
        // Already latched.
        state_ = STATE_PREPARE_SHOT;
      } else if (position->pusher_distal.current || (adjusted_position) < 0) {
        state_ = STATE_LOAD_BACKTRACK;
        // TODO(ben): double check that rezero is the right thing to do here
        if (position) {
          calibration_position_ = position->position;
        }
      } else {
        state_ = STATE_LOAD;
      }

      shooter_.SetGoalPosition(0.0, 0.0);
      if (position && output) {
        output->latch_piston = position->plunger;
      }
      if (output) {
        output->brake_piston = false;
      }
      break;
    case STATE_LOAD_BACKTRACK:
      if (adjusted_position > values.shooter.pusher_distal.upper_angle + 0.01) {
        shooter_.SetGoalPosition(
            real_position - values.shooter.zeroing_speed * dt,
            values.shooter.zeroing_speed);
      } else {
        state_ = STATE_LOAD;
      }

      if (output) output->latch_piston = false;
      if (output) output->brake_piston = true;
      break;
    case STATE_LOAD:
      if (position && position->pusher_proximal.current &&
          !last_position_.pusher_proximal.current) {
        //TODO(ben): use posedge
        calibration_position_ =
            position->position - values.shooter.pusher_proximal.upper_angle;
      }
      if (position && position->pusher_distal.current &&
          !last_position_.pusher_distal.current) {
        //TODO(ben): use posedge
        calibration_position_ =
            position->position - values.shooter.pusher_distal.lower_angle;
      }

      shooter_.SetGoalPosition(calibration_position_, 0.0);
      if (position && output) {
        output->latch_piston = position->plunger;
      }

      if (position->plunger && position->latch) {
        state_ = STATE_PREPARE_SHOT;
      } else if (position->plunger &&
                 fabs(adjusted_position - PowerToPosition(goal->shot_power)) <
                     0.05) {
        state_ = STATE_LOADING_PROBLEM;
        loading_problem_end_time_ =
            Time::Now(Time::kDefaultClock) + Time::InSeconds(3.0);
      }
      if (output) output->brake_piston = false;
      break;
    case STATE_LOADING_PROBLEM:
      if (Time::Now() > loading_problem_end_time_) {
        state_ = STATE_UNLOAD;
      } else if (position->plunger && position->latch) {
        state_ = STATE_PREPARE_SHOT;
      }
      shooter_.SetGoalPosition(calibration_position_, 0.0);
      LOG(DEBUG, "Waiting on latch: plunger %d, latch: %d\n",
          position->plunger, position->latch);

      if (output) output->latch_piston = true;
      if (output) output->brake_piston = false;
      break;
    case STATE_PREPARE_SHOT:
      shooter_.SetGoalPosition(PowerToPosition(goal->shot_power), 0.0);
      LOG(DEBUG, "PDIFF: adjusted_position: %.2f, pow: %.2f\n",
          adjusted_position, PowerToPosition(goal->shot_power));
      if (fabs(adjusted_position - PowerToPosition(goal->shot_power)) < 0.05) {
        state_ = STATE_READY;
        output->latch_piston = true;
        output->brake_piston = true;
        shooter_brake_set_time_ =
            Time::Now(Time::kDefaultClock) + Time::InSeconds(0.03);
      } else {
        output->latch_piston = true;
        output->brake_piston = false;
      }
      break;
    case STATE_READY:
      if (Time::Now() > shooter_brake_set_time_) {
        shooter_loop_disable = true;
        if (goal->unload_requested) {
			printf("GHA\n");
          state_ = STATE_UNLOAD;
        } else if (fabs(adjusted_position - PowerToPosition(goal->shot_power)) >
                   0.05) {
			printf("GHB\n");
          state_ = STATE_PREPARE_SHOT;
        } else if (goal->shot_requested) {
			printf("GHC\n");
          state_ = STATE_REQUEST_FIRE;
        }
      }
      shooter_.SetGoalPosition(PowerToPosition(goal->shot_power), 0.0);

      output->latch_piston = true;
      output->brake_piston = true;
      break;
    case STATE_REQUEST_FIRE:
      shooter_loop_disable = true;
      if (position->plunger) {
        prepare_fire_end_time_ =
            Time::Now(Time::kDefaultClock) + Time::InMS(40.0);
        apply_some_voltage = true;
        state_ = STATE_PREPARE_FIRE;
      } else {
        state_ = STATE_REQUEST_LOAD;
      }
      break;
    case STATE_PREPARE_FIRE:
      shooter_loop_disable = true;
      if (Time::Now(Time::kDefaultClock) < prepare_fire_end_time_) {
        apply_some_voltage = true;
      } else {
        state_ = STATE_FIRE;
        cycles_not_moved_ = 0;
        shot_end_time_ = Time::Now(Time::kDefaultClock) + Time::InMS(500);
      }

      output->latch_piston = true;
      output->brake_piston = true;

      break;
    case STATE_FIRE:
      shooter_loop_disable = true;
      //TODO(ben): need approamately equal
      if (fabs(last_position_.position - adjusted_position) < 0.07) {
        cycles_not_moved_++;
      } else {
        cycles_not_moved_ = 0;
      }
      if ((adjusted_position < 0.10 && cycles_not_moved_ > 5) ||
          Time::Now(Time::kDefaultClock) > shot_end_time_) {
        state_ = STATE_REQUEST_LOAD;
      }
      output->latch_piston = true;
      output->brake_piston = true;
      break;
    case STATE_UNLOAD:
      if (position->plunger && position->latch) {
        shooter_.SetGoalPosition(0.02, 0.0);
        if (adjusted_position < 0.04) {
          output->latch_piston = false;
        }
      } else {
        output->latch_piston = false;
        state_ = STATE_UNLOAD_MOVE;
      }

      output->brake_piston = false;
      break;
    case STATE_UNLOAD_MOVE:
      if (adjusted_position > values.shooter.upper_limit - 0.03) {
        shooter_.SetGoalPosition(real_position, 0.0);
        state_ = STATE_READY_UNLOAD;
      } else {
        shooter_.SetGoalPosition(
            real_position + values.shooter.zeroing_speed * dt,
            values.shooter.zeroing_speed);
      }

      output->latch_piston = false;
      output->brake_piston = false;
      break;
    case STATE_READY_UNLOAD:
      if (!goal->unload_requested) {
        state_ = STATE_REQUEST_LOAD;
      }

      output->latch_piston = false;
      output->brake_piston = false;
      break;
  }

  if (apply_some_voltage) {
    shooter_.Update(true);
    if (output) output->voltage = 2.0;
  } else if (!shooter_loop_disable) {
    LOG(DEBUG, "Running the loop, goal is %f\n", shooter_.R(0, 0));
    shooter_.Update(output == NULL);
    if (output) output->voltage = shooter_.voltage();
  } else {
    shooter_.Update(true);
    if (output) output->voltage = 0.0;
  }

  status->done =
      ::std::fabs(adjusted_position - PowerToPosition(goal->shot_power)) < 0.004;
}

}  // namespace control_loops
}  // namespace frc971
