#include "frc971/control_loops/shooter/shooter.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/shooter/shooter_motor_plant.h"

namespace frc971 {
namespace control_loops {

	
void ZeroedStateFeedbackLoop::CapU() {
  const double old_voltage = voltage_;
  voltage_ += U(0, 0);

  uncapped_voltage_ = voltage_;

  double limit = zeroing_state() != UNKNOWN_POSITION ? 12.0 : kZeroingMaxVoltage;

  // Make sure that reality and the observer can't get too far off.  There is a
  // delay by one cycle between the applied voltage and X_hat(2, 0), so compare
  // against last cycle's voltage.
  if (X_hat(2, 0) > last_voltage_ + 2.0) {
    voltage_ -= X_hat(2, 0) - (last_voltage_ + 2.0);
    LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  } else if (X_hat(2, 0) < last_voltage_ -2.0) {
    voltage_ += X_hat(2, 0) - (last_voltage_ - 2.0);
    LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  }

  voltage_ = std::min(limit, voltage_);
  voltage_ = std::max(-limit, voltage_);
  U(0, 0) = voltage_ - old_voltage;
  LOG(DEBUG, "abc %f\n", X_hat(2, 0) - voltage_);
  LOG(DEBUG, "error %f\n", X_hat(0, 0) - R(0, 0));

  last_voltage_ = voltage_;
}

ShooterMotor::ShooterMotor(control_loops::ShooterLoop *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterLoop>(my_shooter),
      shooter_(MakeShooterLoop()) {
  {
    using ::frc971::constants::GetValues;
    ZeroedJoint<1>::ConfigurationData config_data;

    config_data.lower_limit = GetValues().shooter_lower_limit;
    config_data.upper_limit = GetValues().shooter_upper_limit;
    //config_data.hall_effect_start_position[0] =
    //    GetValues().shooter_hall_effect_start_position;
    config_data.zeroing_off_speed = GetValues().shooter_zeroing_off_speed;
    config_data.zeroing_speed = GetValues().shooter_zeroing_speed;
    config_data.max_zeroing_voltage = 5.0;
    config_data.deadband_voltage = 0.0;

    zeroed_joint_.set_config_data(config_data);
  }
}


// Positive is out, and positive power is out.
void ShooterMotor::RunIteration(
    const ShooterLoop::Goal *goal,
    const control_loops::ShooterLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * status) {
  constexpr double dt = 0.01;

  // we must always have these or we have issues.
  if (goal == NULL || status == NULL) {
      transform-position_ptr = NULL;
      if (output) output->voltage = 0;
      LOG(ERROR, "Thought I would just check for null and die.\n");
	  return;
  }

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) output->voltage = 0;

  ZeroedJoint<1>::PositionData transformed_position;
  ZeroedJoint<1>::PositionData *transformed_position_ptr =
      &transformed_position;
  if (position) {
  	transformed_position.position = position->pos;
  	transformed_position.hall_effects[0] = position->hall_effect;
  	transformed_position.hall_effect_positions[0] = position->calibration;
  }

  const double voltage = shooter_.Update(transformed_position_ptr,
    output != NULL,
    goal->goal, 0.0);

  const frc971::constants::Values &values = constants::GetValues();

  double absolute_position = postiion->position - calibration_position_;


  switch (state_) {
	  case STATE_INITIALIZE:
	  	  shooter_.zeroing_state() = ZeroedStateFeedbackLoop::UNKNOWN_POSITION;

		  // start off with the assumption that we are at the value
		  // futhest back given our sensors
		  if (position && position->pusher_distal_hall_effect){
		  	  calibration_position_ = position->position -
			  	  values.pusher_distal_heffect.lower_edge;
		  } else if (position && position->pusher_proximal_hall_effect) {
		  	  calibration_position_ = position->position -
			  	  values.pusher_proximal_heffect.lower_edge;
		  } else {
		  	  calibration_position_ = values.shooter_total_length;
		  }

      state_ = STATE_REQUEST_LOAD;

		  // zero out initial goal
		  shooter_.SetGoalPositionVelocity(0.0, 0.0);
      if (position) {
        output->latch_pistion = position->plunger_back_hall_effect;
      } else {
          // we don't know what is going on so just close the latch to be safe
          output->latch_piston = true;
      }
      output->brake_piston = false;
	  	break;
	  case STATE_REQUEST_LOAD:
      if (position->plunger_back_hall_effect && position->latch_hall_effect) {
          // already latched
          state_ = STATE_PREPARE_SHOT;
      } else if (postion->pusher_back_distal_hall_effect ||
              (relative_position) < 0) {
          state_ = STATE_LOADING_BACKTRACK;
          if (relative_position) {
              calibration_position_ = position->position;
          }
      } else {
          state_ = STATE_LOAD;
      }

		  shooter_.SetGoalPositionVelocity(0.0, 0.0);
      if (position && output) output->latch_piston = position->plunger_back_hall_effect;
      output->brake_piston = false;
	  	break;
	  case STATE_LOAD_BACKTRACK:
      if (absolute_position < values.pusher_back_distal_heffect.lower_edge + 0.01) {
		    shooter_.SetGoalPositionVelocity(position->position + values.shooter_zero_speed*dt,
                values.shooter_zero_speed);
      } else {
          state = STATE_LOAD;
      }

      output->latch_piston = false;
      output->brake_piston = true;
	  	  break;
	  case STATE_LOAD:
        if (position->pusher_proximal_hall_effect &&
              !last_position_.pusher_back_proximal_hall_effect) {
		  	  calibration_position_ = position->position -
			  	  values.pusher_promimal_heffect.lower_edge;
        }
        if (position->pusher_distal_hall_effect &&
              !last_position_.pusher_back_distal_hall_effect) {
		  	  calibration_position_ = position->position -
			  	  values.pusher_distal_heffect.lower_edge;

        }

		  shooter_.SetGoalPositionVelocity(calibration_position_, 0.0);
      if (position && output) output->latch_piston = position->plunger_back_hall_effect;
      if(output) output->brake_piston = false;

      if (position->plunger_back_hall_effect && position->latch_hall_effect) {
          state_ = STATE_PREPARE_SHOT;
      } else if (position->plunger_back_hall_effect &&
              position->position == PowerToPosition(goal->shot_power)) {
          //TODO_ben: I'm worried it will bounce states if the position is drifting slightly
          state_ = STATE_LOADING_PROBLEM;
          loading_problem_end_time_ = clock() + 3 * CLOCKS_PER_SECOND;
      }
	  	  break;
	  case STATE_LOADING_PROBLEM:
      if (position->plunger_back_hall_effect && position->latch_hall_effect) {
          state_ = STATE_PREPARE_SHOT;
      } else if (absolute_position < -0.02 || clock() > loading_problem_end_time_) {
          state = STATE_UNLOAD;
      }

		  shooter_.SetGoalPositionVelocity(position->position - values.shooter_zero_speed*dt,
                values.shooter_zero_speed);
      if (output) output->latch_piston = true;
      if (output) output->brake_piston = false;
	  	  break;
	  case STATE_PREPARE_SHOT:
        shooter_.SetGoalPosition(
                PowerToPosition(shot_power), 0.0);
        if (position->position == shooter.goal_position) {
            state_ = STATE_READY;
            output->latch_piston = true;
            output->brake_piston = true;
            shooter_brake_set_time_ = clock() + 5 * CLOCKS_PER_SECOND;
        } else {
            output->latch_piston =true;
            output->brake_piston = false;
        }
	  	  break;
	  case STATE_READY:
        if (clock() > shooter_brake_set_time_) {
          shooter_loop_disable = true;
          if (goal->unload_requested) {
              state_ = STATE_UNLOAD;
          } else if (PowerToPosition(goal->shot_power)
                  != position->position) {
              //TODO_ben: I'm worried it will bounce states if the position is drifting slightly
              state_ = STATE_PREPARE_SHOT;
          }else if (goal->shot_requested) {
              state_ = STATE_REQUEST_FIRE;
          }

        }
        output->latch_piston = true;
        output->brake_piston = true;
	  	  break;
	  case STATE_REQUEST_FIRE:
        shooter_loop_disable = true;
        if (position->plunger_back_hall_effect) {
            prepare_fire_end_time_ = clock() + 10;
            state_ = STATE_PREPARE_FIRE;
        } else {
            state_ = STATE_REQUEST_LOAD;
        }
	  	  break;
	  case STATE_PREPARE_FIRE:
        shooter_loop_disable = true;
        if (clock() < prepare_fire_end_time_) {
            shooter_.ApplySomeVoltage();
        } else {
            State_ = STATE_FIRE;
            cycles_not_moved_ = 0;
            shot_end_time_ = clock() + 0.5 * CLOCKS_PER_SECOND;
        }

        output->latch_piston = true;
        output->brake_piston = true;

	  	  break;
	  case STATE_FIRE:
        shooter_loop_disable = true;
        //TODO_ben: need approamately equal
        if (last_position->position - position->position < 7) {
            cycles_not_moved++;
        } else {
            cycles_not_moved = 0;
        }
        output->latch_piston = true;
        ouput->brake_piston = true;
	  	  break;
	  case STATE_UNLOAD:
        if (position->plunger_back_hall_effect && position->latch_piston) {
            shooter_SetGoalPosition(0.02, 0.0);
            if (ablsolute_position == 0.02) {
                output->latch_piston = false;
            }
        } else {
            output->latch_piston = false;
            state_ = STATE_UNLOAD_MOVE;
        }

        output->brake_piston = false;
	  	  break;
	  case STATE_UNLOAD_MOVE:
        if (position->position > values.shooter_length - 0.03) {
          shooter_.SetPosition(position->position, 0.0);
          state_ = STATE_READY_UNLOADED;
        } else {
            shooter_.SetPosition(
                    position->position + values.shooter_zeroing_speed*dt
                    values.shooter_zeroing_speed);
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

  if (position) {
    LOG(DEBUG, "pos:  hall:  absolute: %f\n",
        //position->pos,
        //position->hall_effect ? "true" : "false",
        zeroed_joint_.absolute_position());
  }

  output->voltage = voltage;
  status->done = ::std::abs(zeroed_joint_.absolute_position() - goal->goal) < 0.004;
}

}  // namespace control_loops
}  // namespace frc971
