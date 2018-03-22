#include "frc971/wpilib/joystick_sender.h"

#include "aos/common/messages/robot_state.q.h"
#include "aos/linux_code/init.h"
#include "aos/common/network/team_number.h"
#include "aos/common/logging/queue_logging.h"

#include "DriverStation.h"
#if defined(WPILIB2017) || defined(WPILIB2018)
#include "HAL/HAL.h"
#else
#include "HAL/HAL.hpp"
#endif

namespace frc971 {
namespace wpilib {

void JoystickSender::operator()() {
  DriverStation *ds =
#ifdef WPILIB2015
      DriverStation::GetInstance();
#else
      &DriverStation::GetInstance();
#endif
  ::aos::SetCurrentThreadName("DSReader");
  uint16_t team_id = ::aos::network::GetTeamNumber();

  ::aos::SetCurrentThreadRealtimePriority(29);

  while (run_) {
    ds->WaitForData();
    auto new_state = ::aos::joystick_state.MakeMessage();

    HAL_ControlWord control_word;
    HAL_GetControlWord(&control_word);
    HAL_MatchInfo match_info;
    auto status = HAL_GetMatchInfo(&match_info);
    if (status == 0) {
      new_state->switch_left = match_info.gameSpecificMessage[0] == 'L' ||
                               match_info.gameSpecificMessage[0] == 'l';
      new_state->scale_left = match_info.gameSpecificMessage[1] == 'L' ||
                              match_info.gameSpecificMessage[1] == 'l';
    }
    HAL_FreeMatchInfo(&match_info);

    new_state->test_mode = control_word.test;
    new_state->fms_attached = control_word.fmsAttached;
    new_state->enabled = control_word.enabled;
    new_state->autonomous = control_word.autonomous;
    new_state->team_id = team_id;
    new_state->fake = false;

    for (int i = 0; i < 4; ++i) {
      new_state->joysticks[i].buttons = ds->GetStickButtons(i);
      for (int j = 0; j < 6; ++j) {
        new_state->joysticks[i].axis[j] = ds->GetStickAxis(i, j);
      }
      new_state->joysticks[i].pov = ds->GetStickPOV(i, 0);
    }
    LOG_STRUCT(DEBUG, "joystick_state", *new_state);

    if (!new_state.Send()) {
      LOG(WARNING, "sending joystick_state failed\n");
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
