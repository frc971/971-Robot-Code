#include "aos/input/joystick_input.h"

#include <string.h>
#include <atomic>

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

namespace aos {
namespace input {

::std::atomic<bool> JoystickInput::run_;

void JoystickInput::Quit(int /*signum*/) { run_ = false; }

void JoystickInput::Run() {
  run_ = true;
  struct sigaction action;
  action.sa_handler = &JoystickInput::Quit;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESETHAND;

  PCHECK(sigaction(SIGTERM, &action, nullptr));
  PCHECK(sigaction(SIGQUIT, &action, nullptr));
  PCHECK(sigaction(SIGINT, &action, nullptr));

  driver_station::Data data;
  while (run_) {
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
  LOG(INFO, "Shutting down\n");
}

}  // namespace input
}  // namespace aos
