#include "frc971/wpilib/wpilib_interface.h"

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/queue_logging.h"

#include "DriverStation.h"
#include "ControllerPower.h"
#undef ERROR

namespace frc971 {
namespace wpilib {

void SendRobotState(int32_t my_pid, DriverStation *ds) {
  auto new_state = ::aos::robot_state.MakeMessage();

  new_state->reader_pid = my_pid;
  new_state->outputs_enabled = ds->IsSysActive();
  new_state->browned_out = ds->IsSysBrownedOut();

  new_state->is_3v3_active = ControllerPower::GetEnabled3V3();
  new_state->is_5v_active = ControllerPower::GetEnabled5V();
  new_state->voltage_3v3 = ControllerPower::GetVoltage3V3();
  new_state->voltage_5v = ControllerPower::GetVoltage5V();

  new_state->voltage_roborio_in = ControllerPower::GetInputVoltage();
  new_state->voltage_battery = ds->GetBatteryVoltage();

  LOG_STRUCT(DEBUG, "robot_state", *new_state);

  new_state.Send();
}

}  // namespace wpilib
}  // namespace frc971
