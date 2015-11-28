#include "aos/input/joystick_input.h"

#include <string.h>

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

namespace aos {
namespace input {

void JoystickInput::Run() {
  driver_station::Data data;
  while (true) {
    joystick_state.FetchAnother();

    data.Update(*joystick_state);

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
        if (data.GetPOV(joystick) != data.GetOldPOV(joystick)) {
          LOG(INFO, "POV %d %d->%d\n", joystick, data.GetOldPOV(joystick),
              data.GetPOV(joystick));
        }
      }
    }
    {
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
