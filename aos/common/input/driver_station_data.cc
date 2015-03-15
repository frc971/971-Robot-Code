#include "aos/common/input/driver_station_data.h"

namespace aos {
namespace input {
namespace driver_station {

Data::Data() : current_values_(), old_values_() {}

void Data::Update(const JoystickState &new_values) {
  old_values_ = current_values_;
  current_values_ = new_values;
}

namespace {

bool GetButton(const ButtonLocation location,
               const JoystickState &values) {
  return values.joysticks[location.joystick() - 1].buttons &
      (1 << (location.number() - 1));
}

bool DoGetPOV(const POVLocation location, const JoystickState &values) {
  return values.joysticks[location.joystick() - 1].pov == location.number();
}

bool GetControlBitValue(const ControlBit bit,
                        const JoystickState &values) {
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

}  // namespace

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
