#include "frc971/wpilib/joystick_sender.h"

#include "aos/init.h"
#include "aos/logging/queue_logging.h"
#include "aos/network/team_number.h"
#include "aos/robot_state/robot_state.q.h"

#include "frc971/wpilib/ahal/DriverStation.h"
#include "hal/HAL.h"

namespace frc971 {
namespace wpilib {

void JoystickSender::operator()() {
  frc::DriverStation *const ds = &frc::DriverStation::GetInstance();
  ::aos::SetCurrentThreadName("DSReader");
  uint16_t team_id = ::aos::network::GetTeamNumber();

  ::aos::SetCurrentThreadRealtimePriority(29);

  // TODO(Brian): Fix the potential deadlock when stopping here (condition
  // variable / mutex needs to get exposed all the way out or something).
  while (run_) {
    ds->RunIteration([&]() {
      auto new_state = joystick_state_sender_.MakeMessage();

      HAL_MatchInfo match_info;
      auto status = HAL_GetMatchInfo(&match_info);
      if (status == 0) {
        new_state->switch_left = match_info.gameSpecificMessage[0] == 'L' ||
                                 match_info.gameSpecificMessage[0] == 'l';
        new_state->scale_left = match_info.gameSpecificMessage[1] == 'L' ||
                                match_info.gameSpecificMessage[1] == 'l';
      }

      new_state->test_mode = ds->IsTestMode();
      new_state->fms_attached = ds->IsFmsAttached();
      new_state->enabled = ds->IsEnabled();
      new_state->autonomous = ds->IsAutonomous();
      new_state->team_id = team_id;
      new_state->fake = false;

      for (uint8_t i = 0;
           i < sizeof(new_state->joysticks) / sizeof(::aos::Joystick); ++i) {
        new_state->joysticks[i].buttons = ds->GetStickButtons(i);
        for (int j = 0; j < 6; ++j) {
          new_state->joysticks[i].axis[j] = ds->GetStickAxis(i, j);
        }
        if (ds->GetStickPOVCount(i) > 0) {
          new_state->joysticks[i].pov = ds->GetStickPOV(i, 0);
        }
        LOG_STRUCT(DEBUG, "joystick_state", *new_state);
      }
      if (!new_state.Send()) {
        LOG(WARNING, "sending joystick_state failed\n");
      }
    });
  }
}

}  // namespace wpilib
}  // namespace frc971
