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

  // Make sure that reality and the observer can't get too far off.  There is a
  // delay by one cycle between the applied voltage and X_hat(2, 0), so compare
  // against last cycle's voltage.
  if (X_hat(2, 0) > last_voltage_ + 4.0) {
    voltage_ -= X_hat(2, 0) - (last_voltage_ + 4.0);
    LOG(INFO, "Capping due to runawway\n");
  } else if (X_hat(2, 0) < last_voltage_ - 4.0) {
    voltage_ += X_hat(2, 0) - (last_voltage_ - 4.0);
    LOG(INFO, "Capping due to runawway\n");
  }

  voltage_ = std::min(max_voltage_, voltage_);
  voltage_ = std::max(-max_voltage_, voltage_);
  U(0, 0) = voltage_ - old_voltage;

  LOG(INFO, "X_hat is %f, applied is %f\n", X_hat(2, 0), voltage_);

  last_voltage_ = voltage_;
  capped_goal_ = false;
}

void ZeroedStateFeedbackLoop::CapGoal() {
  if (uncapped_voltage() > max_voltage_) {
    double dx;
    if (controller_index() == 0) {
      dx = (uncapped_voltage() - max_voltage_) /
           (K(0, 0) - A(1, 0) * K(0, 2) / A(1, 2));
      R(0, 0) -= dx;
      R(2, 0) -= -A(1, 0) / A(1, 2) * dx;
    } else {
      dx = (uncapped_voltage() - max_voltage_) / K(0, 0);
      R(0, 0) -= dx;
    }
    capped_goal_ = true;
    LOG(DEBUG, "Moving the goal by %f to prevent windup\n", dx);
  } else if (uncapped_voltage() < -max_voltage_) {
    double dx;
    if (controller_index() == 0) {
      dx = (uncapped_voltage() + max_voltage_) /
           (K(0, 0) - A(1, 0) * K(0, 2) / A(1, 2));
      R(0, 0) -= dx;
      R(2, 0) -= -A(1, 0) / A(1, 2) * dx;
    } else {
      dx = (uncapped_voltage() + max_voltage_) / K(0, 0);
      R(0, 0) -= dx;
    }
    capped_goal_ = true;
    LOG(DEBUG, "Moving the goal by %f to prevent windup\n", dx);
  } else {
    capped_goal_ = false;
  }
}

void ZeroedStateFeedbackLoop::RecalculatePowerGoal() {
  if (controller_index() == 0) {
    R(2, 0) = (-A(1, 0) / A(1, 2) * R(0, 0) - A(1, 1) / A(1, 2) * R(1, 0));
  } else {
    R(2, 0) = -A(1, 1) / A(1, 2) * R(1, 0);
  }
}

void ZeroedStateFeedbackLoop::SetCalibration(double encoder_val,
                                             double known_position) {
  LOG(INFO, "Setting calibration such that %f -> %f\n", encoder_val,
      known_position);
  LOG(INFO, "Position was %f\n", absolute_position());
  double previous_offset = offset_;
  offset_ = known_position - encoder_val;
  double doffset = offset_ - previous_offset;
  LOG(INFO, "Changing offset from %f to %f\n", previous_offset, offset_);
  X_hat(0, 0) += doffset;
  // Offset our measurements because the offset is baked into them.
  Y_(0, 0) += doffset;
  // Offset the goal so we don't move.
  R(0, 0) += doffset;
  if (controller_index() == 0) {
    R(2, 0) += -A(1, 0) / A(1, 2) * (doffset);
  }
  LOG(INFO, "Validation: position is %f\n", absolute_position());
}

ShooterMotor::ShooterMotor(control_loops::ShooterGroup *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterGroup>(my_shooter),
      shooter_(MakeShooterLoop()),
      state_(STATE_INITIALIZE),
      loading_problem_end_time_(0, 0),
      load_timeout_(0, 0),
      shooter_brake_set_time_(0, 0),
      unload_timeout_(0, 0),
      prepare_fire_end_time_(0, 0),
      shot_end_time_(0, 0),
      cycles_not_moved_(0) {}

double ShooterMotor::PowerToPosition(double power) {
  // LOG(WARNING, "power to position not correctly implemented\n");
  const frc971::constants::Values &values = constants::GetValues();
  double new_pos = ::std::min(::std::max(power, values.shooter.lower_limit),
                              values.shooter.upper_limit);
  return new_pos;
}

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

  if (reset()) {
    state_ = STATE_INITIALIZE;
  }
  if (position) {
    shooter_.CorrectPosition(position->position);
  }

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) output->voltage = 0;

  const frc971::constants::Values &values = constants::GetValues();

  // Don't even let the control loops run.
  bool shooter_loop_disable = false;

  // Adds voltage to take up slack in gears before shot.
  bool apply_some_voltage = false;


  const bool disabled = !::aos::robot_state->enabled;
  // If true, move the goal if we saturate.
  bool cap_goal = false;

  // TODO(austin): Move the offset if we see or don't see a hall effect when we
  // expect to see one.
  // Probably not needed yet.

  if (position) {
    int last_controller_index = shooter_.controller_index();
    if (position->plunger && position->latch) {
      // Use the controller without the spring if the latch is set and the
      // plunger is back
      shooter_.set_controller_index(1);
      LOG(DEBUG, "Using controller 1\n");
    } else {
      // Otherwise use the controller with the spring.
      shooter_.set_controller_index(0);
      LOG(DEBUG, "Using controller 0\n");
    }
    if (shooter_.controller_index() != last_controller_index) {
      shooter_.RecalculatePowerGoal();
    }
  }

  switch (state_) {
    case STATE_INITIALIZE:
      if (position) {
        // Reinitialize the internal filter state.
        shooter_.InitializeState(position->position);

        // Start off with the assumption that we are at the value
        // futhest back given our sensors.
        if (position->pusher_distal.current) {
          shooter_.SetCalibration(position->position,
                                  values.shooter.pusher_distal.lower_angle);
        } else if (position->pusher_proximal.current) {
          shooter_.SetCalibration(position->position,
                                  values.shooter.pusher_proximal.upper_angle);
        } else {
          shooter_.SetCalibration(position->position,
                                  values.shooter.upper_limit);
        }

        state_ = STATE_REQUEST_LOAD;

        // Go to the current position.
        shooter_.SetGoalPosition(shooter_.absolute_position(), 0.0);
        // If the plunger is all the way back, we want to be latched.
        latch_piston_ = position->plunger;
        brake_piston_ = false;
      } else {
        // If we can't start yet because we don't know where we are, set the
        // latch and brake to their defaults.
        latch_piston_ = true;
        brake_piston_ = true;
      }
      break;
    case STATE_REQUEST_LOAD:
      if (position) {
        if (position->pusher_distal.current) {
          // We started on the sensor, back up until we are found.
          // If the plunger is all the way back and not latched, it won't be
          // there for long.
          state_ = STATE_LOAD_BACKTRACK;

          // The plunger is already back and latched.  Don't release it.
          if (position->plunger && position->latch) {
            latch_piston_ = true;
          } else {
            latch_piston_ = false;
          }
        } else if (position->plunger && position->latch) {
          // The plunger is back and we are latched.  We most likely got here
          // from Initialize, in which case we want to 'load' again anyways to
          // zero.
          Load();
          latch_piston_ = true;
        } else {
          // Off the sensor, start loading.
          Load();
          latch_piston_ = false;
        }
      }

      // Hold our current position.
      shooter_.SetGoalPosition(shooter_.absolute_position(), 0.0);
      brake_piston_ = false;
      break;
    case STATE_LOAD_BACKTRACK:
      // If we are here, then that means we started past the edge where we want
      // to zero.  Move backwards until we don't see the sensor anymore.
      // The plunger is contacting the pusher (or will be shortly).

      if (!disabled) {
        shooter_.SetGoalPosition(
            shooter_.goal_position() + values.shooter.zeroing_speed * dt,
            values.shooter.zeroing_speed);
      }
      cap_goal = true;
      shooter_.set_max_voltage(4.0);

      if (position) {
        if (!position->pusher_distal.current) {
          Load();
        }
      }

      latch_piston_ = false;
      brake_piston_ = false;
      break;
    case STATE_LOAD:
      // If we are disabled right now, reset the timer.
      if (disabled) {
        Load();
        // Latch defaults to true when disabled.  Leave it latched until we have
        // useful sensor data.
        latch_piston_ = true;
      }
      // Go to 0, which should be the latch position, or trigger a hall effect
      // on the way.  If we don't see edges where we are supposed to, the
      // offset will be updated by code above.
      shooter_.SetGoalPosition(0.0, 0.0);

      if (position) {
        // If we see a posedge on any of the hall effects,
        if (position->pusher_proximal.posedge_count !=
            last_proximal_posedge_count_) {
          LOG(DEBUG, "Setting calibration using proximal sensor\n");
          shooter_.SetCalibration(position->pusher_proximal.posedge_value,
                                  values.shooter.pusher_proximal.upper_angle);
        }
        if (position->pusher_distal.posedge_count !=
            last_distal_posedge_count_) {
          LOG(DEBUG, "Setting calibration using distal sensor\n");
          shooter_.SetCalibration(position->pusher_distal.posedge_value,
                                  values.shooter.pusher_distal.upper_angle);
        }

        // Latch if the plunger is far enough back to trigger the hall effect.
        // This happens when the distal sensor is triggered.
        latch_piston_ = position->pusher_distal.current || position->plunger;

        // Check if we are latched and back.  Make sure the plunger is all the
        // way back as well.
        if (position->plunger && position->latch &&
            position->pusher_distal.current) {
          state_ = STATE_PREPARE_SHOT;
        } else if (position->plunger &&
                   ::std::abs(shooter_.absolute_position() -
                              shooter_.goal_position()) < 0.001) {
          // We are at the goal, but not latched.
          state_ = STATE_LOADING_PROBLEM;
          loading_problem_end_time_ = Time::Now() + kLoadProblemEndTimeout;
        }
      }
      if (load_timeout_ < Time::Now()) {
        if (position) {
          if (!position->pusher_distal.current ||
              !position->pusher_proximal.current) {
            state_ = STATE_ESTOP;
          }
        }
      } else if (goal->unload_requested) {
        Unload();
      }
      brake_piston_ = false;
      break;
    case STATE_LOADING_PROBLEM:
      if (disabled) {
        Load();
      }
      // We got to the goal, but the latch hasn't registered as down.  It might
      // be stuck, or on it's way but not there yet.
      if (Time::Now() > loading_problem_end_time_) {
        // Timeout by unloading.
        Unload();
      } else if (position && position->plunger && position->latch) {
        // If both trigger, we are latched.
        state_ = STATE_PREPARE_SHOT;
      }
      // Move a bit further back to help it trigger.
      // If the latch is slow due to the air flowing through the tubes or
      // inertia, but is otherwise free, this won't have much time to do
      // anything and is safe.  Otherwise this gives us a bit more room to free
      // up the latch.
      shooter_.SetGoalPosition(values.shooter.lower_limit, 0.0);
      LOG(DEBUG, "Waiting on latch: plunger %d, latch: %d\n",
          position->plunger, position->latch);

      latch_piston_ = true;
      brake_piston_ = false;
      break;
    case STATE_PREPARE_SHOT:
      // Move the shooter to the shot power set point and then lock the brake.
      // TODO(austin): Timeout.  Low priority.

      shooter_.SetGoalPosition(PowerToPosition(goal->shot_power), 0.0);

      LOG(DEBUG, "PDIFF: absolute_position: %.2f, pow: %.2f\n",
          shooter_.absolute_position(), PowerToPosition(goal->shot_power));
      if (::std::abs(shooter_.absolute_position() -
                     PowerToPosition(goal->shot_power)) +
              ::std::abs(shooter_.absolute_velocity()) <
          0.001) {
        // We are there, set the brake and move on.
        latch_piston_ = true;
        brake_piston_ = true;
        shooter_brake_set_time_ = Time::Now() + kShooterBrakeSetTime;
        state_ = STATE_READY;
      } else {
        latch_piston_ = true;
        brake_piston_ = false;
      }
      if (goal->unload_requested) {
        Unload();
      }
      break;
    case STATE_READY:
      LOG(DEBUG, "In ready\n");
      // Wait until the brake is set, and a shot is requested or the shot power
      // is changed.
      if (::std::abs(shooter_.absolute_position() -
                     PowerToPosition(goal->shot_power)) > 0.002) {
        // TODO(austin): Add a state to release the brake.

        // TODO(austin): Do we want to set the brake here or after shooting?
        // Depends on air usage.
        LOG(DEBUG, "Preparing shot again.\n");
        state_ = STATE_PREPARE_SHOT;
      } else if (Time::Now() > shooter_brake_set_time_) {
        // We have waited long enough for the brake to set, turn the shooter
        // control loop off.
        shooter_loop_disable = true;
        LOG(DEBUG, "Brake is now set\n");
        if (goal->shot_requested && !disabled) {
          LOG(DEBUG, "Shooting now\n");
          shooter_loop_disable = true;
          prepare_fire_end_time_ = Time::Now() + kPrepareFireEndTime;
          apply_some_voltage = true;
          state_ = STATE_PREPARE_FIRE;
        }
      } else {
        LOG(DEBUG, "Nothing %d %d\n", goal->shot_requested, !disabled);
      }
      shooter_.SetGoalPosition(PowerToPosition(goal->shot_power), 0.0);

      latch_piston_ = true;
      brake_piston_ = true;

      if (goal->unload_requested) {
        Unload();
      }
      break;

    case STATE_PREPARE_FIRE:
      // Apply a bit of voltage to bias the gears for a little bit of time, and
      // then fire.
      shooter_loop_disable = true;
      if (disabled) {
        // If we are disabled, reset the backlash bias timer.
        prepare_fire_end_time_ = Time::Now() + kPrepareFireEndTime;
        break;
      }
      if (Time::Now() > prepare_fire_end_time_) {
        cycles_not_moved_ = 0;
        firing_starting_position_ = shooter_.absolute_position();
        shot_end_time_ = Time::Now() + kShotEndTimeout;
        state_ = STATE_FIRE;
        latch_piston_ = false;
      } else {
        apply_some_voltage = true;
        latch_piston_ = true;
      }

      brake_piston_ = true;
      break;

    case STATE_FIRE:
      if (disabled) {
        if (position) {
          if (position->plunger) {
            // If disabled and the plunger is still back there, reset the
            // timeout.
            shot_end_time_ = Time::Now() + kShotEndTimeout;
          }
        }
      }
      shooter_loop_disable = true;
      // Count the number of contiguous cycles during which we haven't moved.
      if (::std::abs(last_position_.position - shooter_.absolute_position()) <
          0.0005) {
        ++cycles_not_moved_;
      } else {
        cycles_not_moved_ = 0;
      }

      // If we have moved any amount since the start and the shooter has now
      // been still for a couple cycles, the shot finished.
      // Also move on if it times out.
      if ((::std::abs(firing_starting_position_ -
                      shooter_.absolute_position()) > 0.0005 &&
           cycles_not_moved_ > 3) ||
          Time::Now() > shot_end_time_) {
        state_ = STATE_REQUEST_LOAD;
      }
      latch_piston_ = false;
      brake_piston_ = true;
      break;
    case STATE_UNLOAD:
      // Reset the timeouts.
      if (disabled) Unload();

      // If it is latched and the plunger is back, move the pusher back to catch
      // the plunger.
      bool all_back;
      if (position) {
        all_back = position->plunger && position->latch;
      } else {
        all_back = last_position_.plunger && last_position_.latch;
      }

      if (all_back) {
        // Pull back to 0, 0.
        shooter_.SetGoalPosition(0.0, 0.0);
        if (shooter_.absolute_position() < 0.005) {
          // When we are close enough, 'fire'.
          latch_piston_ = false;
        } else {
          latch_piston_ = true;
        }
      } else {
        // The plunger isn't all the way back, or it is and it is unlatched, so
        // we can now unload.
        shooter_.SetGoalPosition(shooter_.absolute_position(), 0.0);
        latch_piston_ = false;
        state_ = STATE_UNLOAD_MOVE;
        unload_timeout_ = Time::Now() + kUnloadTimeout;
      }

      if (Time::Now() > unload_timeout_) {
        // We have been stuck trying to unload for way too long, give up and
        // turn everything off.
        state_ = STATE_ESTOP;
      }

      brake_piston_ = false;
      break;
    case STATE_UNLOAD_MOVE: {
      if (disabled) {
        unload_timeout_ = Time::Now() + kUnloadTimeout;
        shooter_.SetGoalPosition(shooter_.absolute_position(), 0.0);
      }
      cap_goal = true;
      shooter_.set_max_voltage(5.0);

      // Slowly move back until we hit the upper limit.
      // If we were at the limit last cycle, we are done unloading.
      // This is because if we saturate, we might hit the limit before we are
      // actually there.
      if (shooter_.goal_position() >= values.shooter.upper_limit) {
        shooter_.SetGoalPosition(values.shooter.upper_limit, 0.0);
        // We don't want the loop fighting the spring when we are unloaded.
        // Turn it off.
        shooter_loop_disable = true;
        state_ = STATE_READY_UNLOAD;
      } else {
        shooter_.SetGoalPosition(
            ::std::min(
                values.shooter.upper_limit,
                shooter_.goal_position() + values.shooter.zeroing_speed * dt),
            values.shooter.zeroing_speed);
      }

      latch_piston_ = false;
      brake_piston_ = false;
    } break;
    case STATE_READY_UNLOAD:
      if (goal->load_requested) {
        state_ = STATE_REQUEST_LOAD;
      }
      // If we are ready to load again, 
      shooter_loop_disable = true;

      latch_piston_ = false;
      brake_piston_ = false;
      break;

    case STATE_ESTOP:
      // Totally lost, go to a safe state.
      shooter_loop_disable = true;
      latch_piston_ = true;
      brake_piston_ = true;
      break;
  }

  if (apply_some_voltage) {
    shooter_.Update(true);
    shooter_.ZeroPower();
    if (output) output->voltage = 2.0;
  } else if (!shooter_loop_disable) {
    LOG(DEBUG, "Running the loop, goal is %f, position is %f\n",
        shooter_.goal_position(), shooter_.absolute_position());
    if (!cap_goal) {
      shooter_.set_max_voltage(12.0);
    }
    shooter_.Update(output == NULL);
    if (cap_goal) {
      shooter_.CapGoal();
    }
    if (output) output->voltage = shooter_.voltage();
  } else {
    shooter_.Update(true);
    shooter_.ZeroPower();
    if (output) output->voltage = 0.0;
  }

  if (output) {
    output->latch_piston = latch_piston_;
    output->brake_piston = brake_piston_;
  }

  status->done = ::std::abs(shooter_.absolute_position() -
                            PowerToPosition(goal->shot_power)) < 0.004;

  if (position) {
    last_position_ = *position;
    LOG(DEBUG, "pos > absolute: %f velocity: %f state= %d l= %d pp= %d, pd= %d "
               "p= %d b=%d\n",
        shooter_.absolute_position(), shooter_.absolute_velocity(),
		state_, position->latch, position->pusher_proximal.current,
		position->pusher_distal.current,
		position->plunger, brake_piston_); 
  }
  if (position) {
    last_distal_posedge_count_ = position->pusher_distal.posedge_count;
    last_proximal_posedge_count_ = position->pusher_proximal.posedge_count;
  }
}

}  // namespace control_loops
}  // namespace frc971
