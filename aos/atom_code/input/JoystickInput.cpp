#include "aos/atom_code/input/JoystickInput.h"

#include "aos/common/Configuration.h"
#include "aos/common/network/ReceiveSocket.h"
#include "aos/common/messages/RobotState.q.h"

namespace aos {

void JoystickInput::SetupButtons() {
  for (int i = 0; i < 4; ++i) {
    old_buttons[i] = buttons[i];
  }
  buttons[0] = control_data_.stick0Buttons;
  buttons[1] = control_data_.stick1Buttons;
  buttons[2] = control_data_.stick2Buttons;
  buttons[3] = control_data_.stick3Buttons;

  // Put the ENABLED, AUTONOMOUS, and FMS_ATTACHED values into unused bits in
  // the values for joystick 0 so that PosEdge and NegEdge can be used with
  // them.
  // Windows only supports 12 buttons, so we know there will never be any more.
  // Not using MASK because it doesn't make it any cleaner.
  buttons[0] |= (control_data_.enabled << (ENABLED - 9)) |
      (control_data_.autonomous << (AUTONOMOUS - 9)) |
      (control_data_.fmsAttached << (FMS_ATTACHED - 9));

  for (int j = 0; j < 4; ++j) {
    for (int k = 1; k <= 12; ++k) {
      if (PosEdge(j, k)) {
        LOG(INFO, "PosEdge(%d, %d)\n", j, k);
      }
      if (NegEdge(j, k)) {
        LOG(INFO, "NegEdge(%d, %d)\n", j, k);
      }
    }
  }
  if (PosEdge(0, ENABLED)) LOG(INFO, "PosEdge(ENABLED)\n");
  if (NegEdge(0, ENABLED)) LOG(INFO, "NegEdge(ENABLED)\n");
  if (PosEdge(0, AUTONOMOUS)) LOG(INFO, "PosEdge(AUTONOMOUS)\n");
  if (NegEdge(0, AUTONOMOUS)) LOG(INFO, "NegEdge(AUTONOMOUS)\n");
  if (PosEdge(0, FMS_ATTACHED)) LOG(INFO, "PosEdge(FMS_ATTACHED)\n");
  if (NegEdge(0, FMS_ATTACHED)) LOG(INFO, "NegEdge(FMS_ATTACHED)\n");
}

void JoystickInput::Run() {
  ReceiveSocket sock(NetworkPort::kDS);
  while (true) {
    if (sock.Receive(&control_data_, sizeof(control_data_)) !=
        sizeof(control_data_)) {
      LOG(WARNING, "socket receive failed\n");
      continue;
    }
    SetupButtons();
    if (!robot_state.MakeWithBuilder()
        .enabled(Pressed(0, ENABLED))
        .autonomous(Pressed(0, AUTONOMOUS))
        .team_id(ntohs(control_data_.teamID))
        .Send()) {
			LOG(WARNING, "sending robot_state failed\n");
		}
		if (robot_state.FetchLatest()) {
    	char state[1024];
    	robot_state->Print(state, sizeof(state));
    	LOG(DEBUG, "robot_state={%s}\n", state);
		} else {
			LOG(WARNING, "fetching robot_state failed\n");
		}
    RunIteration();
  }
}

}  // namespace aos

