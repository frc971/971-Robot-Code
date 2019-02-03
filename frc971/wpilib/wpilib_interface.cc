#include "frc971/wpilib/wpilib_interface.h"

#include "aos/robot_state/robot_state.q.h"
#include "aos/logging/queue_logging.h"

#include "hal/HAL.h"

namespace frc971 {
namespace wpilib {

void SendRobotState(int32_t my_pid) {
  auto new_state = ::aos::robot_state.MakeMessage();

  int32_t status = 0;

  new_state->reader_pid = my_pid;
  new_state->outputs_enabled = HAL_GetSystemActive(&status);
  new_state->browned_out = HAL_GetBrownedOut(&status);

  new_state->is_3v3_active = HAL_GetUserActive3V3(&status);
  new_state->is_5v_active = HAL_GetUserActive5V(&status);
  new_state->voltage_3v3 = HAL_GetUserVoltage3V3(&status);
  new_state->voltage_5v = HAL_GetUserVoltage5V(&status);

  new_state->voltage_roborio_in = HAL_GetVinVoltage(&status);
  new_state->voltage_battery = HAL_GetVinVoltage(&status);

  LOG_STRUCT(DEBUG, "robot_state", *new_state);

  new_state.Send();
}

}  // namespace wpilib
}  // namespace frc971
