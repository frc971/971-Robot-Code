#include "frc971/control_loops/shooter/shooter.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/shooter/shooter_motor_plant.h"

namespace frc971 {
namespace control_loops {

ShooterMotor::ShooterMotor(control_loops::ShooterLoop *my_shooter)
    : aos::control_loops::ControlLoop<control_loops::ShooterLoop>(my_shooter),
      zeroed_joint_(MakeShooterLoop()) {
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

enum {
	STATE_INITIALIZE,
	STATE_REQUEST_LOAD,
	STATE_LOAD_BACKTRACK,
	STATE_LOAD,
	STATE_LOADING_PROBLEM,
	STATE_PREPARE_SHOT,
	STATE_BRAKE_SET,
	STATE_READY,
	STATE_REQUEST_FIRE,
	STATE_PREPARE_FIRE,
	STATE_FIRE,
	STATE_UNLOAD,
	STATE_UNLOAD_MOVE,
	STATE_READY_UNLOAD
} State;

// Positive is out, and positive power is out.
void ShooterMotor::RunIteration(
    const ShooterLoop::Goal *goal,
    const control_loops::ShooterLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * status) {
  constexpr double dt = 0.01;

  if (goal == NULL || position == NULL ||
          output == NULL || status == NULL) {
      transform-position_ptr = NULL;
      if (output) output->voltage = 0;
      LOG(ERROR, "Thought I would just check for null and die.\n");
  }

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  output->voltage = 0;

  ZeroedJoint<1>::PositionData transformed_position;
  ZeroedJoint<1>::PositionData *transformed_position_ptr =
      &transformed_position;
  transformed_position.position = position->pos;
  transformed_position.hall_effects[0] = position->hall_effect;
  transformed_position.hall_effect_positions[0] = position->calibration;

  const double voltage = zeroed_joint_.Update(transformed_position_ptr,
    output != NULL,
    goal->goal, 0.0);

  const frc971::constants::Values &values = constants::GetValues();

  switch (state_) {
	  case STATE_INITIALIZE:
	  	  shooter_.zeroing_state() = ZeroedStateFeedbackLoop::UNKNOWN_POSITION;
		  if (position->pusher_distal_hall_effect){
		  } else if (position->pusher_proximal_hall_effect) {
		  	  calibration_position_ = position->position -
		  } else {
		  }


	  	  break;
	  case STATE_REQUEST_LOAD:
	  	  break;
	  case STATE_LOAD_BACKTRACK:
	  	  break;
	  case STATE_LOAD:
	  	  break;
	  case STATE_LOADING_PROBLEM:
	  	  break;
	  case STATE_PREPARE_SHOT:
	  	  break;
	  case STATE_BRAKE_SET:
	  	  break;
	  case STATE_READY:
	  	  break;
	  case STATE_REQUEST_FIRE:
	  	  break;
	  case STATE_PREPARE_FIRE:
	  	  break;
	  case STATE_FIRE:
	  	  break;
	  case STATE_UNLOAD:
	  	  break;
	  case STATE_UNLOAD_MOVE:
	  	  break;
	  case STATE_READY_UNLOAD:
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
