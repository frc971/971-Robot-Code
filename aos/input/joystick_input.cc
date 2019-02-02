#include "aos/input/joystick_input.h"

#include <string.h>
#include <atomic>

#include "aos/robot_state/robot_state.q.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"

namespace aos {
namespace input {

::std::atomic<bool> JoystickInput::run_;

void JoystickInput::Quit(int /*signum*/) { run_ = false; }

void JoystickInput::HandleData(const ::aos::JoystickState &joystick_state) {
  data_.Update(joystick_state);

  mode_ = static_cast<int>(joystick_state.switch_left) |
          (static_cast<int>(joystick_state.scale_left) << 1);

  {
    using driver_station::JoystickFeature;
    using driver_station::ButtonLocation;
    for (int joystick = 1; joystick <= JoystickFeature::kJoysticks;
         ++joystick) {
      for (int button = 1; button <= ButtonLocation::kButtons; ++button) {
        ButtonLocation location(joystick, button);
        if (data_.PosEdge(location)) {
          LOG(INFO, "PosEdge(%d, %d)\n", joystick, button);
        }
        if (data_.NegEdge(location)) {
          LOG(INFO, "NegEdge(%d, %d)\n", joystick, button);
        }
      }
      if (data_.GetPOV(joystick) != data_.GetOldPOV(joystick)) {
        LOG(INFO, "POV %d %d->%d\n", joystick, data_.GetOldPOV(joystick),
            data_.GetPOV(joystick));
      }
    }
  }
  {
    using driver_station::ControlBit;
    if (data_.PosEdge(ControlBit::kFmsAttached)) {
      LOG(INFO, "PosEdge(kFmsAttached)\n");
    }
    if (data_.NegEdge(ControlBit::kFmsAttached)) {
      LOG(INFO, "NegEdge(kFmsAttached)\n");
    }
    if (data_.PosEdge(ControlBit::kAutonomous)) {
      LOG(INFO, "PosEdge(kAutonomous)\n");
    }
    if (data_.NegEdge(ControlBit::kAutonomous)) {
      LOG(INFO, "NegEdge(kAutonomous)\n");
    }
    if (data_.PosEdge(ControlBit::kEnabled)) {
      LOG(INFO, "PosEdge(kEnabled)\n");
    }
    if (data_.NegEdge(ControlBit::kEnabled)) {
      LOG(INFO, "NegEdge(kEnabled)\n");
    }
  }

  RunIteration(data_);

  if (!run_) {
    event_loop_->Exit();
  }
}

void JoystickInput::Run() {
  // TODO(austin): We need a better sigint story for event loops in general.
  run_ = true;
  struct sigaction action;
  action.sa_handler = &JoystickInput::Quit;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESETHAND;

  PCHECK(sigaction(SIGTERM, &action, nullptr));
  PCHECK(sigaction(SIGQUIT, &action, nullptr));
  PCHECK(sigaction(SIGINT, &action, nullptr));

  event_loop_->Run();

  LOG(INFO, "Shutting down\n");
}

}  // namespace input
}  // namespace aos
