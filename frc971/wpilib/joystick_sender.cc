#include "frc971/wpilib/joystick_sender.h"

#include "aos/common/messages/robot_state.q.h"
#include "aos/linux_code/init.h"
#include "aos/common/network/team_number.h"
#include "aos/common/logging/queue_logging.h"

#include "DriverStation.h"

namespace frc971 {
namespace wpilib {

void JoystickSender::operator()() {
  DriverStation *ds = DriverStation::GetInstance();
  ::aos::SetCurrentThreadName("DSReader");
  uint16_t team_id = ::aos::network::GetTeamNumber();

  ::aos::SetCurrentThreadRealtimePriority(29);

  while (run_) {
    ds->WaitForData();
    auto new_state = ::aos::robot_state.MakeMessage();

    new_state->test_mode = ds->IsAutonomous();
    new_state->fms_attached = ds->IsFMSAttached();
    new_state->enabled = ds->IsEnabled();
    new_state->autonomous = ds->IsAutonomous();
    new_state->team_id = team_id;
    new_state->fake = false;

    for (int i = 0; i < 4; ++i) {
      new_state->joysticks[i].buttons = 0;
      for (int button = 0; button < 16; ++button) {
        new_state->joysticks[i].buttons |= ds->GetStickButton(i, button + 1)
                                           << button;
      }
      for (int j = 0; j < 4; ++j) {
        new_state->joysticks[i].axis[j] = ds->GetStickAxis(i, j);
      }
    }
    LOG_STRUCT(DEBUG, "robot_state", *new_state);

    if (!new_state.Send()) {
      LOG(WARNING, "sending robot_state failed\n");
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
