#include "frc971/input/redundant_joystick_data.h"

#include "aos/logging/logging.h"

namespace frc971 {
namespace input {
namespace driver_station {

RedundantData::RedundantData(const Data &data) : joystick_map_(), data_(data) {
  // Start with a naive map.
  for (int i = 0; i < JoystickFeature::kJoysticks; i++) {
    joystick_map_.at(i) = -1;
  }

  for (int i = 0; i < JoystickFeature::kJoysticks; i++) {
    ButtonLocation id_bit0_location(i + 1, kIdBit0Button);
    ButtonLocation id_bit1_location(i + 1, kIdBit1Button);
    ButtonLocation redundant_bit_location(i + 1, kRedundantBitButton);

    int id_bit0 = data_.IsPressed(id_bit0_location);
    int id_bit1 = data_.IsPressed(id_bit1_location);
    int is_redundant = data_.IsPressed(redundant_bit_location);

    // We don't care if this is the redundant or primary one.  Pick the second
    // one.
    (void)is_redundant;

    int packed_joystick_number = (id_bit1 << 1u) | (id_bit0 << 0u);

    joystick_map_.at(packed_joystick_number) = i + 1;
  }
};

int RedundantData::MapRedundantJoystick(int joystick) const {
  if (joystick < 0 || joystick >= static_cast<int>(joystick_map_.size())) {
    return -1;
  }
  return joystick_map_.at(joystick);
}

bool RedundantData::IsPressed(POVLocation location) const {
  POVLocation mapped_location(MapRedundantJoystick(location.joystick()),
                              location.number());
  return data_.IsPressed(mapped_location);
}

bool RedundantData::PosEdge(POVLocation location) const {
  POVLocation mapped_location(MapRedundantJoystick(location.joystick()),
                              location.number());
  return data_.PosEdge(mapped_location);
}

bool RedundantData::NegEdge(POVLocation location) const {
  POVLocation mapped_location(MapRedundantJoystick(location.joystick()),
                              location.number());
  return data_.NegEdge(mapped_location);
}

// Returns the current and previous "values" for the POV.
int32_t RedundantData::GetPOV(int joystick) const {
  return data_.GetPOV(MapRedundantJoystick(joystick));
}

int32_t RedundantData::GetOldPOV(int joystick) const {
  return data_.GetOldPOV(MapRedundantJoystick(joystick));
}

bool RedundantData::IsPressed(ButtonLocation location) const {
  ButtonLocation mapped_location(MapRedundantJoystick(location.joystick()),
                                 location.number());
  return data_.IsPressed(mapped_location);
}

bool RedundantData::PosEdge(ButtonLocation location) const {
  ButtonLocation mapped_location(MapRedundantJoystick(location.joystick()),
                                 location.number());
  return data_.PosEdge(mapped_location);
}

bool RedundantData::NegEdge(ButtonLocation location) const {
  ButtonLocation mapped_location(MapRedundantJoystick(location.joystick()),
                                 location.number());
  return data_.NegEdge(mapped_location);
}

bool RedundantData::GetControlBit(ControlBit bit) const {
  return data_.GetControlBit(bit);
}

bool RedundantData::PosEdge(ControlBit bit) const { return data_.PosEdge(bit); }

bool RedundantData::NegEdge(ControlBit bit) const { return data_.NegEdge(bit); }

// Returns the value in the range [-1.0, 1.0].
float RedundantData::GetAxis(JoystickAxis axis) const {
  JoystickAxis mapped_location(MapRedundantJoystick(axis.joystick()),
                               axis.number());
  return data_.GetAxis(mapped_location);
}

}  // namespace driver_station
}  // namespace input
}  // namespace frc971
