#include "aos/atom_code/input/joystick_input.h"

#include <string.h>

#include "aos/externals/WPILib/WPILib/NetworkRobot/NetworkRobotValues.h"

#include "aos/common/Configuration.h"
#include "aos/common/network/ReceiveSocket.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace input {

void JoystickInput::Run() {
  ReceiveSocket sock(NetworkPort::kDS);

  NetworkRobotJoysticks joysticks;
  char buffer[sizeof(joysticks) + ::buffers::kOverhead];
  driver_station::Data data;

  while (true) {
    int received = sock.Receive(buffer, sizeof(buffer));
    if (received == -1) {
      LOG(WARNING, "socket receive failed with %d: %s\n",
          errno, strerror(errno));
      continue;
    }

    if (!joysticks.DeserializeFrom(buffer, received)) {
      LOG(WARNING, "deserializing data from %zd bytes failed\n", received);
      continue;
    }

    if (!robot_state.MakeWithBuilder()
        .enabled(joysticks.control.enabled())
        .autonomous(joysticks.control.autonomous())
        .team_id(joysticks.team_number)
        .Send()) {
			LOG(WARNING, "sending robot_state failed\n");
		} else {
      LOG(DEBUG, "sent robot_state{%s, %s, %hu}\n",
          joysticks.control.enabled() ? "enabled" : "disabled",
          joysticks.control.autonomous() ? "auto" : "not auto",
          joysticks.team_number);
    }

    data.Update(joysticks);
    // TODO(brians): posedge/negedge logging

    RunIteration(data);
  }
}

}  // namespace input
}  // namespace aos
