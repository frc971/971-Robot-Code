#include "frc971/wpilib/wpilib_interface.h"

#include "aos/robot_state/robot_state.q.h"

#include "hal/HAL.h"

namespace frc971 {
namespace wpilib {

void PopulateRobotState(::aos::RobotState *robot_state, int32_t my_pid) {
  int32_t status = 0;

  robot_state->reader_pid = my_pid;
  robot_state->outputs_enabled = HAL_GetSystemActive(&status);
  robot_state->browned_out = HAL_GetBrownedOut(&status);

  robot_state->is_3v3_active = HAL_GetUserActive3V3(&status);
  robot_state->is_5v_active = HAL_GetUserActive5V(&status);
  robot_state->voltage_3v3 = HAL_GetUserVoltage3V3(&status);
  robot_state->voltage_5v = HAL_GetUserVoltage5V(&status);

  robot_state->voltage_roborio_in = HAL_GetVinVoltage(&status);
  robot_state->voltage_battery = HAL_GetVinVoltage(&status);

  if (status != 0) {
    LOG(FATAL, "Failed to get robot state: %d\n", status);
  }
}

}  // namespace wpilib
}  // namespace frc971
