#include "aos/prime/input/joystick_input.h"

#include <string.h>

#include "aos/externals/WPILib/WPILib/NetworkRobot/NetworkRobotValues.h"

#include "aos/common/network_port.h"
#include "aos/common/network/ReceiveSocket.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

namespace aos {
namespace input {

void JoystickInput::Run() {
  ReceiveSocket sock(NetworkPort::kDS);
  // If true, this code won't try to read anything from the network and instead
  // feed all 0s to the joystick code.
  // The RobotState messages will be marked as fake so anything that outputs
  // values won't while this is enabled.
  static const bool kFakeJoysticks = false;

  NetworkRobotJoysticks joysticks;
  char buffer[sizeof(joysticks) + ::buffers::kOverhead];
  driver_station::Data data;

  while (true) {
    if (kFakeJoysticks) {
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.02));
      memset(&joysticks, 0, sizeof(joysticks));
    } else {
      int received = sock.Receive(buffer, sizeof(buffer));
      if (received == -1) {
        LOG(WARNING, "socket receive failed with %d: %s\n",
            errno, strerror(errno));
        continue;
      }

      if (!joysticks.DeserializeFrom(buffer, received)) {
        LOG(WARNING, "deserializing data from %d bytes failed\n", received);
        continue;
      }
    }

    auto new_state = robot_state.MakeMessage();
    new_state->enabled = joysticks.control.enabled();
    new_state->autonomous = joysticks.control.autonomous();
    new_state->team_id = joysticks.team_number;
    new_state->fake = kFakeJoysticks;
    LOG_STRUCT(DEBUG, "sending", *new_state);

    if (!new_state.Send()) {
      LOG(WARNING, "sending robot_state failed\n");
    }

    data.Update(joysticks);

    {
      using driver_station::JoystickFeature;
      using driver_station::ButtonLocation;
      for (int joystick = 1; joystick <= JoystickFeature::kJoysticks;
           ++joystick) {
        for (int button = 1; button <= ButtonLocation::kButtons; ++button) {
          ButtonLocation location(joystick, button);
          if (data.PosEdge(location)) {
            LOG(INFO, "PosEdge(%d, %d)\n", joystick, button);
          }
          if (data.NegEdge(location)) {
            LOG(INFO, "NegEdge(%d, %d)\n", joystick, button);
          }
        }
      }

      using driver_station::ControlBit;
      if (data.PosEdge(ControlBit::kFmsAttached)) {
        LOG(INFO, "PosEdge(kFmsAttached)\n");
      }
      if (data.NegEdge(ControlBit::kFmsAttached)) {
        LOG(INFO, "NegEdge(kFmsAttached)\n");
      }
      if (data.PosEdge(ControlBit::kAutonomous)) {
        LOG(INFO, "PosEdge(kAutonomous)\n");
      }
      if (data.NegEdge(ControlBit::kAutonomous)) {
        LOG(INFO, "NegEdge(kAutonomous)\n");
      }
      if (data.PosEdge(ControlBit::kEnabled)) {
        LOG(INFO, "PosEdge(kEnabled)\n");
      }
      if (data.NegEdge(ControlBit::kEnabled)) {
        LOG(INFO, "NegEdge(kEnabled)\n");
      }
    }

    RunIteration(data);
  }
}

}  // namespace input
}  // namespace aos
