#include "aos/common/input/driver_station_data.h"

namespace aos {
namespace input {
namespace driver_station {

Data::Data() : current_values_(), old_values_() {}

void Data::Update(const NetworkRobotJoysticks &new_values) {
  old_values_ = current_values_;
  current_values_ = new_values;
}

namespace {

bool GetButton(const ButtonLocation location,
               const NetworkRobotJoysticks &values) {
  return values.joysticks[location.joystick() - 1].buttons &
      (1 << (location.number() - 1));
}

bool GetControlBitValue(const ControlBit bit,
                        const NetworkRobotJoysticks &values) {
  switch (bit) {
    case ControlBit::kTestMode:
      return values.control.test_mode();
    case ControlBit::kFmsAttached:
      return values.control.fms_attached();
    case ControlBit::kAutonomous:
      return values.control.autonomous();
    case ControlBit::kEnabled:
      return values.control.enabled();
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
  // TODO(brians): check this math against what our joysticks report as their
  // logical minimums and maximums
  return current_values_.joysticks[axis.joystick()].axes[axis.number()] / 127.0;
}

}  // namespace driver_station
}  // namespace input
}  // namespace aos
