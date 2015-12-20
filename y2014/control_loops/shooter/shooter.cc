#include "y2014/control_loops/shooter/shooter.h"

#include <stdio.h>

#include <algorithm>
#include <limits>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "y2014/constants.h"
#include "y2014/control_loops/shooter/shooter_motor_plant.h"

namespace y2014 {
namespace control_loops {

using ::y2014::control_loops::shooter::kSpringConstant;
using ::y2014::control_loops::shooter::kMaxExtension;
using ::y2014::control_loops::shooter::kDt;
using ::y2014::control_loops::shooter::MakeShooterLoop;
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
    LOG(DEBUG, "Capping due to runaway\n");
  } else if (X_hat(2, 0) < last_voltage_ - 4.0) {
    voltage_ += X_hat(2, 0) - (last_voltage_ - 4.0);
    LOG(DEBUG, "Capping due to runaway\n");
  }

  voltage_ = std::min(max_voltage_, voltage_);
  voltage_ = std::max(-max_voltage_, voltage_);
  mutable_U(0, 0) = voltage_ - old_voltage;

  LOG_STRUCT(
      DEBUG, "shooter_output",
      ::y2014::control_loops::ShooterVoltageToLog(X_hat(2, 0), voltage_));

  last_voltage_ = voltage_;
  capped_goal_ = false;
}

void ZeroedStateFeedbackLoop::CapGoal() {
  if (uncapped_voltage() > max_voltage_) {
    double dx;
    if (controller_index() == 0) {
      dx = (uncapped_voltage() - max_voltage_) /
           (K(0, 0) - A(1, 0) * K(0, 2) / A(1, 2));
      mutable_R(0, 0) -= dx;
      mutable_R(2, 0) -= -A(1, 0) / A(1, 2) * dx;
    } else {
      dx = (uncapped_voltage() - max_voltage_) / K(0, 0);
      mutable_R(0, 0) -= dx;
    }
    capped_goal_ = true;
    LOG_STRUCT(DEBUG, "to prevent windup",
               ::y2014::control_loops::ShooterMovingGoal(dx));
  } else if (uncapped_voltage() < -max_voltage_) {
    double dx;
    if (controller_index() == 0) {
      dx = (uncapped_voltage() + max_voltage_) /
           (K(0, 0) - A(1, 0) * K(0, 2) / A(1, 2));
      mutable_R(0, 0) -= dx;
      mutable_R(2, 0) -= -A(1, 0) / A(1, 2) * dx;
    } else {
      dx = (uncapped_voltage() + max_voltage_) / K(0, 0);
      mutable_R(0, 0) -= dx;
    }
    capped_goal_ = true;
    LOG_STRUCT(DEBUG, "to prevent windup",
               ::y2014::control_loops::ShooterMovingGoal(dx));
  } else {
    capped_goal_ = false;
  }
}

void ZeroedStateFeedbackLoop::RecalculatePowerGoal() {
  if (controller_index() == 0) {
    mutable_R(2, 0) = (-A(1, 0) / A(1, 2) * R(0, 0) - A(1, 1) / A(1, 2) * R(1, 0));
  } else {
    mutable_R(2, 0) = -A(1, 1) / A(1, 2) * R(1, 0);
  }
}

void ZeroedStateFeedbackLoop::SetCalibration(double encoder_val,
                                             double known_position) {
  double old_position = absolute_position();
  double previous_offset = offset_;
  offset_ = known_position - encoder_val;
  double doffset = offset_ - previous_offset;
  mutable_X_hat(0, 0) += doffset;
  // Offset the goal so we don't move.
  mutable_R(0, 0) += doffset;
  if (controller_index() == 0) {
    mutable_R(2, 0) += -A(1, 0) / A(1, 2) * (doffset);
  }
  LOG_STRUCT(DEBUG, "sensor edge (fake?)",
             ::y2014::control_loops::ShooterChangeCalibration(
                 encoder_val, known_position, old_position, absolute_position(),
                 previous_offset, offset_));
}

ShooterMotor::ShooterMotor(::y2014::control_loops::ShooterQueue *my_shooter)
    : aos::controls::ControlLoop<::y2014::control_loops::ShooterQueue>(
          my_shooter),
      shooter_(MakeShooterLoop()),
      state_(STATE_INITIALIZE),
      loading_problem_end_time_(0, 0),
      load_timeout_(0, 0),
      shooter_brake_set_time_(0, 0),
      unload_timeout_(0, 0),
      shot_end_time_(0, 0),
      cycles_not_moved_(0),
      shot_count_(0),
      zeroed_(false),
      distal_posedge_validation_cycles_left_(0),
      proximal_posedge_validation_cycles_left_(0),
      last_distal_current_(true),
      last_proximal_current_(true) {}

double ShooterMotor::PowerToPosition(double power) {
  const constants::Values &values = constants::GetValues();
  double maxpower = 0.5 * kSpringConstant *
                    (kMaxExtension * kMaxExtension -
                     (kMaxExtension - values.shooter.upper_limit) *
                         (kMaxExtension - values.shooter.upper_limit));
  if (power < 0) {
    LOG_STRUCT(WARNING, "negative power",
               ::y2014::control_loops::PowerAdjustment(power, 0));
    power = 0;
  } else if (power > maxpower) {
    LOG_STRUCT(WARNING, "power too high",
               ::y2014::control_loops::PowerAdjustment(power, maxpower));
    power = maxpower;
  }

  double mp = kMaxExtension * kMaxExtension - (power + power) / kSpringConstant;
  double new_pos = 0.10;
  if (mp < 0) {
    LOG(ERROR,
        "Power calculation has negative number before square root (%f).\n", mp);
  } else {
    new_pos = kMaxExtension - ::std::sqrt(mp);
  }

  new_pos = ::std::min(::std::max(new_pos, values.shooter.lower_limit),
                              values.shooter.upper_limit);
  return new_pos;
}

double ShooterMotor::PositionToPower(double position) {
  double power = kSpringConstant * position * (kMaxExtension - position / 2.0);
  return power;
}

void ShooterMotor::CheckCalibrations(
    const ::y2014::control_loops::ShooterQueue::Position *position) {
  CHECK_NOTNULL(position);
  const constants::Values &values = constants::GetValues();

  // TODO(austin): Validate that this is the right edge.
  // If we see a posedge on any of the hall effects,
  if (position->pusher_proximal.posedge_count != last_proximal_posedge_count_ &&
      !last_proximal_current_) {
    proximal_posedge_validation_cycles_left_ = 2;
  }
  if (proximal_posedge_validation_cycles_left_ > 0) {
    if (position->pusher_proximal.current) {
      --proximal_posedge_validation_cycles_left_;
      if (proximal_posedge_validation_cycles_left_ == 0) {
        shooter_.SetCalibration(
            position->pusher_proximal.posedge_value,
            values.shooter.pusher_proximal.upper_angle);

        LOG(DEBUG, "Setting calibration using proximal sensor\n");
        zeroed_ = true;
      }
    } else {
      proximal_posedge_validation_cycles_left_ = 0;
    }
  }

  if (position->pusher_distal.posedge_count != last_distal_posedge_count_ &&
      !last_distal_current_) {
    distal_posedge_validation_cycles_left_ = 2;
  }
  if (distal_posedge_validation_cycles_left_ > 0) {
    if (position->pusher_distal.current) {
      --distal_posedge_validation_cycles_left_;
      if (distal_posedge_validation_cycles_left_ == 0) {
        shooter_.SetCalibration(
            position->pusher_distal.posedge_value,
            values.shooter.pusher_distal.upper_angle);

        LOG(DEBUG, "Setting calibration using distal sensor\n");
        zeroed_ = true;
      }
    } else {
      distal_posedge_validation_cycles_left_ = 0;
    }
  }
}

// Positive is out, and positive power is out.
void ShooterMotor::RunIteration(
    const ::y2014::control_loops::ShooterQueue::Goal *goal,
    const ::y2014::control_loops::ShooterQueue::Position *position,
    ::y2014::control_loops::ShooterQueue::Output *output,
    ::y2014::control_loops::ShooterQueue::Status *status) {
  if (goal && ::std::isnan(goal->shot_power)) {
	  state_ = STATE_ESTOP;
    LOG(ERROR, "Estopping because got a shot power of NAN.\n");
  }

  // we must always have these or we have issues.
  if (status == NULL) {
    if (output) output->voltage = 0;
    LOG(ERROR, "Thought I would just check for null and die.\n");
    return;
  }
  status->ready = false;

  if (WasReset()) {
    state_ = STATE_INITIALIZE;
    last_distal_current_ = position->pusher_distal.current;
    last_proximal_current_ = position->pusher_proximal.current;
  }
  if (position) {
    shooter_.CorrectPosition(position->position);
  }

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) output->voltage = 0;

  const constants::Values &values = constants::GetValues();

  // Don't even let the control loops run.
  bool shooter_loop_disable = false;

  const bool disabled =
      !::aos::joystick_state.get() || !::aos::joystick_state->enabled;

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
    } else {
      // Otherwise use the controller with the spring.
      shooter_.set_controller_index(0);
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

        // Go to the current position.
        shooter_.SetGoalPosition(shooter_.absolute_position(), 0.0);
        // If the plunger is all the way back, we want to be latched.
        latch_piston_ = position->plunger;
        brake_piston_ = false;
        if (position->latch == latch_piston_) {
          state_ = STATE_REQUEST_LOAD;
        } else {
          shooter_loop_disable = true;
          LOG(DEBUG,
              "Not moving on until the latch has moved to avoid a crash\n");
        }
      } else {
        // If we can't start yet because we don't know where we are, set the
        // latch and brake to their defaults.
        latch_piston_ = true;
        brake_piston_ = true;
      }
      break;
    case STATE_REQUEST_LOAD:
      if (position) {
        zeroed_ = false;
        if (position->pusher_distal.current ||
            position->pusher_proximal.current) {
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
            shooter_.goal_position() + values.shooter.zeroing_speed * kDt,
            values.shooter.zeroing_speed);
      }
      cap_goal = true;
      shooter_.set_max_voltage(4.0);

      if (position) {
        if (!position->pusher_distal.current &&
            !position->pusher_proximal.current) {
          Load();
        }
        latch_piston_ = position->plunger;
      }

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
      if (output == nullptr) {
        load_timeout_ += ::aos::controls::kLoopFrequency;
      }
      // Go to 0, which should be the latch position, or trigger a hall effect
      // on the way.  If we don't see edges where we are supposed to, the
      // offset will be updated by code above.
      shooter_.SetGoalPosition(0.0, 0.0);

      if (position) {
        CheckCalibrations(position);

        // Latch if the plunger is far enough back to trigger the hall effect.
        // This happens when the distal sensor is triggered.
        latch_piston_ = position->pusher_distal.current || position->plunger;

        // Check if we are latched and back.  Make sure the plunger is all the
        // way back as well.
        if (position->plunger && position->latch &&
            position->pusher_distal.current) {
          if (!zeroed_) {
            state_ = STATE_REQUEST_LOAD;
          } else {
            state_ = STATE_PREPARE_SHOT;
          }
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
          // If none of the sensors is triggered, estop.
          // Otherwise, trigger anyways if it has been 0.5 seconds more.
          if (!(position->pusher_distal.current ||
                position->pusher_proximal.current) ||
              (load_timeout_ + Time::InSeconds(0.5) < Time::Now())) {
            state_ = STATE_ESTOP;
            LOG(ERROR, "Estopping because took too long to load.\n");
          }
        }
      }
      brake_piston_ = false;
      break;
    case STATE_LOADING_PROBLEM:
      if (disabled) {
        state_ = STATE_REQUEST_LOAD;
        break;
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
      if (position) {
        LOG(DEBUG, "Waiting on latch: plunger %d, latch: %d\n",
            position->plunger, position->latch);
      }

      latch_piston_ = true;
      brake_piston_ = false;
      break;
    case STATE_PREPARE_SHOT:
      // Move the shooter to the shot power set point and then lock the brake.
      // TODO(austin): Timeout.  Low priority.

      if (goal) {
        shooter_.SetGoalPosition(PowerToPosition(goal->shot_power), 0.0);
      }

      LOG(DEBUG, "PDIFF: absolute_position: %.2f, pow: %.2f\n",
          shooter_.absolute_position(),
          goal ? PowerToPosition(goal->shot_power)
               : ::std::numeric_limits<double>::quiet_NaN());
      if (goal &&
          ::std::abs(shooter_.absolute_position() -
                     PowerToPosition(goal->shot_power)) < 0.001 &&
          ::std::abs(shooter_.absolute_velocity()) < 0.005) {
        // We are there, set the brake and move on.
        latch_piston_ = true;
        brake_piston_ = true;
        shooter_brake_set_time_ = Time::Now() + kShooterBrakeSetTime;
        state_ = STATE_READY;
      } else {
        latch_piston_ = true;
        brake_piston_ = false;
      }
      if (goal && goal->unload_requested) {
        Unload();
      }
      break;
    case STATE_READY:
      LOG(DEBUG, "In ready\n");
      // Wait until the brake is set, and a shot is requested or the shot power
      // is changed.
      if (Time::Now() > shooter_brake_set_time_) {
        status->ready = true;
        // We have waited long enough for the brake to set, turn the shooter
        // control loop off.
        shooter_loop_disable = true;
        LOG(DEBUG, "Brake is now set\n");
        if (goal && goal->shot_requested && !disabled) {
          LOG(DEBUG, "Shooting now\n");
          shooter_loop_disable = true;
          shot_end_time_ = Time::Now() + kShotEndTimeout;
          firing_starting_position_ = shooter_.absolute_position();
          state_ = STATE_FIRE;
        }
      }
      if (state_ == STATE_READY && goal &&
          ::std::abs(shooter_.absolute_position() -
                     PowerToPosition(goal->shot_power)) > 0.002) {
        // TODO(austin): Add a state to release the brake.

        // TODO(austin): Do we want to set the brake here or after shooting?
        // Depends on air usage.
        status->ready = false;
        LOG(DEBUG, "Preparing shot again.\n");
        state_ = STATE_PREPARE_SHOT;
      }

      if (goal) {
        shooter_.SetGoalPosition(PowerToPosition(goal->shot_power), 0.0);
      }

      latch_piston_ = true;
      brake_piston_ = true;

      if (goal && goal->unload_requested) {
        Unload();
      }
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
      if (((::std::abs(firing_starting_position_ -
                       shooter_.absolute_position()) > 0.0005 &&
            cycles_not_moved_ > 6) ||
           Time::Now() > shot_end_time_) &&
          ::aos::robot_state->voltage_battery > 10.5) {
        state_ = STATE_REQUEST_LOAD;
        ++shot_count_;
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

          if (position) {
            CheckCalibrations(position);
          }
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
        LOG(ERROR, "Estopping because took too long to unload.\n");
      }

      brake_piston_ = false;
      break;
    case STATE_UNLOAD_MOVE: {
      if (disabled) {
        unload_timeout_ = Time::Now() + kUnloadTimeout;
        shooter_.SetGoalPosition(shooter_.absolute_position(), 0.0);
      }
      cap_goal = true;
      shooter_.set_max_voltage(6.0);

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
                shooter_.goal_position() + values.shooter.unload_speed * kDt),
            values.shooter.unload_speed);
      }

      latch_piston_ = false;
      brake_piston_ = false;
    } break;
    case STATE_READY_UNLOAD:
      if (goal && goal->load_requested) {
        state_ = STATE_REQUEST_LOAD;
      }
      // If we are ready to load again,
      shooter_loop_disable = true;

      latch_piston_ = false;
      brake_piston_ = false;
      break;

    case STATE_ESTOP:
      LOG(WARNING, "estopped\n");
      // Totally lost, go to a safe state.
      shooter_loop_disable = true;
      latch_piston_ = true;
      brake_piston_ = true;
      break;
  }

  if (!shooter_loop_disable) {
    LOG_STRUCT(DEBUG, "running the loop",
               ::y2014::control_loops::ShooterStatusToLog(
                   shooter_.goal_position(), shooter_.absolute_position()));
    if (!cap_goal) {
      shooter_.set_max_voltage(12.0);
    }
    shooter_.Update(output == NULL);
    if (cap_goal) {
      shooter_.CapGoal();
    }
    // We don't really want to output anything if we went through everything
    // assuming the motors weren't working.
    if (output) output->voltage = shooter_.voltage();
  } else {
    shooter_.Update(true);
    shooter_.ZeroPower();
    if (output) output->voltage = 0.0;
  }

  status->hard_stop_power = PositionToPower(shooter_.absolute_position());

  if (output) {
    output->latch_piston = latch_piston_;
    output->brake_piston = brake_piston_;
  }

  if (position) {
    LOG_STRUCT(DEBUG, "internal state",
               ::y2014::control_loops::ShooterStateToLog(
                   shooter_.absolute_position(), shooter_.absolute_velocity(),
                   state_, position->latch, position->pusher_proximal.current,
                   position->pusher_distal.current, position->plunger,
                   brake_piston_, latch_piston_));

    last_position_ = *position;

    last_distal_posedge_count_ = position->pusher_distal.posedge_count;
    last_proximal_posedge_count_ = position->pusher_proximal.posedge_count;
    last_distal_current_ = position->pusher_distal.current;
    last_proximal_current_ = position->pusher_proximal.current;
  }

  status->absolute_position = shooter_.absolute_position();
  status->absolute_velocity = shooter_.absolute_velocity();
  status->state = state_;

  status->shots = shot_count_;
}

void ShooterMotor::ZeroOutputs() {
  queue_group()->output.MakeWithBuilder()
      .voltage(0)
      .latch_piston(latch_piston_)
      .brake_piston(brake_piston_)
      .Send();
}

}  // namespace control_loops
}  // namespace y2014
