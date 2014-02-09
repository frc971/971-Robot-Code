#include "frc971/control_loops/shooters/shooters.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/shooters/top_shooter_motor_plant.h"
#include "frc971/control_loops/shooters/bottom_shooter_motor_plant.h"

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
    config_data.hall_effect_start_position[0] =
        GetValues().shooter_hall_effect_start_position;
    config_data.zeroing_off_speed = GetValues().shooter_zeroing_off_speed;
    config_data.zeroing_speed = GetValues().shooter_zeroing_speed;

    config_data.max_zeroing_voltage = 5.0;
    config_data.deadband_voltage = 0.0;

    zeroed_joint_.set_config_data(config_data);
  }
}

// Positive is up, and positive power is up.
void ShooterMotor::RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::ShooterLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * status) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->voltage = 0;
  }

  ZeroedJoint<1>::PositionData transformed_position;
  ZeroedJoint<1>::PositionData *transformed_position_ptr =
      &transformed_position;
  if (!position) {
    transformed_position_ptr = NULL;
  } else {
    transformed_position.position = position->pos;
    transformed_position.hall_effects[0] = position->hall_effect;
    transformed_position.hall_effect_positions[0] = position->calibration;
  }

  const double voltage = zeroed_joint_.Update(transformed_position_ptr,
    output != NULL,
    goal->goal, 0.0);

  if (position) {
    LOG(DEBUG, "pos: %f hall: %s absolute: %f\n",
        position->pos,
        position->hall_effect ? "true" : "false",
        zeroed_joint_.absolute_position());
  }

  if (output) {
    output->voltage = voltage;
  }
  status->done = ::std::abs(zeroed_joint_.absolute_position() - goal->goal) < 0.004;
}

}  // namespace control_loops
}  // namespace frc971
