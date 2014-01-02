#include "frc971/control_loops/angle_adjust/angle_adjust.h"

#include <algorithm>

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
  {
    using ::frc971::constants::GetValues;
    ZeroedJoint<2>::ConfigurationData config_data;

    config_data.lower_limit = GetValues().angle_adjust_lower_limit;
    config_data.upper_limit = GetValues().angle_adjust_upper_limit;
    memcpy(config_data.hall_effect_start_angle,
           GetValues().angle_adjust_hall_effect_start_angle,
           sizeof(config_data.hall_effect_start_angle));
    config_data.zeroing_off_speed = GetValues().angle_adjust_zeroing_off_speed;
    config_data.zeroing_speed = GetValues().angle_adjust_zeroing_speed;

    config_data.max_zeroing_voltage = 4.0;
    config_data.deadband_voltage = GetValues().angle_adjust_deadband;

    zeroed_joint_.set_config_data(config_data);
  }
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
