#include "frc971/control_loops/angle_adjust/angle_adjust.h"

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor_plant.h"

namespace frc971 {
namespace control_loops {

AngleAdjustMotor::AngleAdjustMotor(
    control_loops::AngleAdjustLoop *my_angle_adjust)
    : aos::control_loops::ControlLoop<control_loops::AngleAdjustLoop>(
        my_angle_adjust),
      zeroed_joint_(MakeAngleAdjustLoop()) {
}

bool AngleAdjustMotor::FetchConstants(
    ZeroedJoint<2>::ConfigurationData *config_data) {
  if (!constants::angle_adjust_lower_limit(
          &config_data->lower_limit)) {
    LOG(ERROR, "Failed to fetch the angle adjust lower limit constant.\n");
    return false;
  }
  if (!constants::angle_adjust_upper_limit(
          &config_data->upper_limit)) {
    LOG(ERROR, "Failed to fetch the angle adjust upper limit constant.\n");
    return false;
  }
  if (!constants::angle_adjust_hall_effect_start_angle(
          config_data->hall_effect_start_angle)) {
    LOG(ERROR, "Failed to fetch the hall effect start angle constants.\n");
    return false;
  }
  if (!constants::angle_adjust_zeroing_off_speed(
          &config_data->zeroing_off_speed)) {
    LOG(ERROR,
        "Failed to fetch the angle adjust zeroing off speed constant.\n");
    return false;
  }
  if (!constants::angle_adjust_zeroing_speed(
          &config_data->zeroing_speed)) {
    LOG(ERROR, "Failed to fetch the angle adjust zeroing speed constant.\n");
    return false;
  }

  config_data->max_zeroing_voltage = 4.0;
  return true;
}

// Positive angle is up, and positive power is up.
void AngleAdjustMotor::RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::AngleAdjustLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status *status) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->voltage = 0;
  }

  // Cache the constants to avoid error handling down below.
  ZeroedJoint<2>::ConfigurationData config_data;
  if (!FetchConstants(&config_data)) {
    LOG(WARNING, "Failed to fetch constants.\n");
    return;
  } else {
    zeroed_joint_.set_config_data(config_data);
  }

  ZeroedJoint<2>::PositionData transformed_position;
  ZeroedJoint<2>::PositionData *transformed_position_ptr =
      &transformed_position;
  if (!position) {
    transformed_position_ptr = NULL;
  } else {
    transformed_position.position = position->angle;
    transformed_position.hall_effects[0] = position->bottom_hall_effect;
    transformed_position.hall_effect_positions[0] =
        position->bottom_calibration;
    transformed_position.hall_effects[1] = position->middle_hall_effect;
    transformed_position.hall_effect_positions[1] =
        position->middle_calibration;
  }

  const double voltage = zeroed_joint_.Update(transformed_position_ptr,
      output != NULL,
      goal->goal, 0.0);

  if (position) {
    LOG(DEBUG, "pos: %f bottom_hall: %s middle_hall: %s absolute: %f\n",
        position->angle,
        position->bottom_hall_effect ? "true" : "false",
        position->middle_hall_effect ? "true" : "false",
        zeroed_joint_.absolute_position());
  }

  if (output) {
    output->voltage = voltage;
  }
  status->done = ::std::abs(zeroed_joint_.absolute_position() - goal->goal) < 0.002;
}

}  // namespace control_loops
}  // namespace frc971
