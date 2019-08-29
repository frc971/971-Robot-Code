#include "aos/input/joystick_input.h"

#include <string.h>
#include <atomic>

#include "aos/logging/logging.h"
#include "aos/robot_state/robot_state_generated.h"

namespace aos {
namespace input {

void JoystickInput::HandleData(const ::aos::JoystickState *joystick_state) {
  data_.Update(joystick_state);

  mode_ = static_cast<int>(joystick_state->switch_left()) |
          (static_cast<int>(joystick_state->scale_left()) << 1);

  {
    using driver_station::JoystickFeature;
    using driver_station::ButtonLocation;
    for (int joystick = 1; joystick <= JoystickFeature::kJoysticks;
         ++joystick) {
      for (int button = 1; button <= ButtonLocation::kButtons; ++button) {
        ButtonLocation location(joystick, button);
        if (data_.PosEdge(location)) {
          AOS_LOG(INFO, "PosEdge(%d, %d)\n", joystick, button);
        }
        if (data_.NegEdge(location)) {
          AOS_LOG(INFO, "NegEdge(%d, %d)\n", joystick, button);
        }
      }
      if (data_.GetPOV(joystick) != data_.GetOldPOV(joystick)) {
        AOS_LOG(INFO, "POV %d %d->%d\n", joystick, data_.GetOldPOV(joystick),
                data_.GetPOV(joystick));
      }
    }
  }
  {
    using driver_station::ControlBit;
    if (data_.PosEdge(ControlBit::kFmsAttached)) {
      AOS_LOG(INFO, "PosEdge(kFmsAttached)\n");
    }
    if (data_.NegEdge(ControlBit::kFmsAttached)) {
      AOS_LOG(INFO, "NegEdge(kFmsAttached)\n");
    }
    if (data_.PosEdge(ControlBit::kAutonomous)) {
      AOS_LOG(INFO, "PosEdge(kAutonomous)\n");
    }
    if (data_.NegEdge(ControlBit::kAutonomous)) {
      AOS_LOG(INFO, "NegEdge(kAutonomous)\n");
    }
    if (data_.PosEdge(ControlBit::kEnabled)) {
      AOS_LOG(INFO, "PosEdge(kEnabled)\n");
    }
    if (data_.NegEdge(ControlBit::kEnabled)) {
      AOS_LOG(INFO, "NegEdge(kEnabled)\n");
    }
  }

  RunIteration(data_);
}

}  // namespace input
}  // namespace aos
