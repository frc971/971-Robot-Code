#include "aos/input/driver_station_data.h"

#include "glog/logging.h"

namespace aos {
namespace input {
namespace driver_station {

Data::Data() : current_values_(), old_values_() {}

void Data::Update(const JoystickState *new_values) {
  old_values_ = current_values_;
  CHECK(new_values->has_joysticks());
  CHECK_EQ(new_values->joysticks()->size(), current_values_.joysticks.size());
  for (size_t i = 0; i < current_values_.joysticks.size(); ++i) {
    const Joystick *joystick = new_values->joysticks()->Get(i);
    current_values_.joysticks[i].buttons =
        joystick->buttons();
    current_values_.joysticks[i].pov = joystick->pov();
    for (size_t j = 0; j < joystick->axis()->size(); ++j) {
      current_values_.joysticks[i].axis[j] = joystick->axis()->Get(j);
    }

    current_values_.joysticks[i].pov = joystick->pov();
  }
  current_values_.test_mode = new_values->test_mode();
  current_values_.fms_attached = new_values->fms_attached();
  current_values_.enabled = new_values->enabled();
  current_values_.autonomous = new_values->autonomous();
  current_values_.team_id = new_values->team_id();
  current_values_.switch_left = new_values->switch_left();
  current_values_.scale_left = new_values->scale_left();
}

bool Data::GetButton(const ButtonLocation location,
                     const Data::SavedJoystickState &values) {
  if (location.joystick() < 0 ||
      location.joystick() > static_cast<int>(values.joysticks.size())) {
    return false;
  }
  if (location.number() <= 0 || location.number() > 16) {
    return false;
  }
  return values.joysticks[location.joystick() - 1].buttons &
         (1 << (location.number() - 1));
}

bool Data::DoGetPOV(const POVLocation location,
                    const Data::SavedJoystickState &values) {
  return values.joysticks[location.joystick() - 1].pov == location.number();
}

bool Data::GetControlBitValue(const ControlBit bit,
                              const Data::SavedJoystickState &values) {
  switch (bit) {
    case ControlBit::kTestMode:
      return values.test_mode;
    case ControlBit::kFmsAttached:
      return values.fms_attached;
    case ControlBit::kAutonomous:
      return values.autonomous;
    case ControlBit::kEnabled:
      return values.enabled;
    default:
      __builtin_unreachable();
  }
}

bool Data::IsPressed(const ButtonLocation location) const {
  return GetButton(location, current_values_);
}

bool Data::PosEdge(const ButtonLocation location) const {
  return !GetButton(location, old_values_) &&
      GetButton(location, current_values_);
}

bool Data::NegEdge(const ButtonLocation location) const {
  return GetButton(location, old_values_) &&
      !GetButton(location, current_values_);
}

bool Data::IsPressed(const POVLocation location) const {
  return DoGetPOV(location, current_values_);
}

bool Data::PosEdge(const POVLocation location) const {
  return !DoGetPOV(location, old_values_) &&
         DoGetPOV(location, current_values_);
}

bool Data::NegEdge(const POVLocation location) const {
  return DoGetPOV(location, old_values_) &&
         !DoGetPOV(location, current_values_);
}

int32_t Data::GetPOV(int joystick) const {
  return current_values_.joysticks[joystick - 1].pov;
}

int32_t Data::GetOldPOV(int joystick) const {
  return old_values_.joysticks[joystick - 1].pov;
}

bool Data::GetControlBit(const ControlBit bit) const {
  return GetControlBitValue(bit, current_values_);
}

bool Data::PosEdge(const ControlBit bit) const {
  return !GetControlBitValue(bit, old_values_) &&
      GetControlBitValue(bit, current_values_);
}

bool Data::NegEdge(const ControlBit bit) const {
  return GetControlBitValue(bit, old_values_) &&
      !GetControlBitValue(bit, current_values_);
}

float Data::GetAxis(JoystickAxis axis) const {
  return current_values_.joysticks[axis.joystick() - 1].axis[axis.number() - 1];
}

}  // namespace driver_station
}  // namespace input
}  // namespace aos
