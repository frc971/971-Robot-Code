#include "frc971/control_loops/wrist/wrist.h"

#include <stdio.h>

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/wrist/wrist_motor_plant.h"

namespace frc971 {
namespace control_loops {

WristMotor::WristMotor(control_loops::WristLoop *my_wrist)
    : aos::control_loops::ControlLoop<control_loops::WristLoop>(my_wrist),
      zeroed_joint_(MakeWristLoop()) {
}

bool WristMotor::FetchConstants(
    ZeroedJoint<1>::ConfigurationData *config_data) {
  if (!constants::wrist_lower_limit(&config_data->lower_limit)) {
    LOG(ERROR, "Failed to fetch the wrist lower limit constant.\n");
    return false;
  }
  if (!constants::wrist_upper_limit(&config_data->upper_limit)) {
    LOG(ERROR, "Failed to fetch the wrist upper limit constant.\n");
    return false;
  }
  if (!constants::wrist_hall_effect_start_angle(
          &config_data->hall_effect_start_angle[0])) {
    LOG(ERROR, "Failed to fetch the wrist start angle constant.\n");
    return false;
  }
  if (!constants::wrist_zeroing_off_speed(&config_data->zeroing_off_speed)) {
    LOG(ERROR, "Failed to fetch the wrist zeroing off speed constant.\n");
    return false;
  }

  if (!constants::wrist_zeroing_speed(&config_data->zeroing_speed)) {
    LOG(ERROR, "Failed to fetch the wrist zeroing speed constant.\n");
    return false;
  }

  config_data->max_zeroing_voltage = 5.0;
  return true;
}

// Positive angle is up, and positive power is up.
void WristMotor::RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::WristLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * status) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->voltage = 0;
  }

  // Cache the constants to avoid error handling down below.
  ZeroedJoint<1>::ConfigurationData config_data;
  if (!FetchConstants(&config_data)) {
    return;
  } else {
    zeroed_joint_.set_config_data(config_data);
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
