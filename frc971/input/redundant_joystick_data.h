#ifndef AOS_INPUT_REDUNDANT_JOYSTICK_DATA_H_
#define AOS_INPUT_REDUNDANT_JOYSTICK_DATA_H_

#include "frc971/input/driver_station_data.h"

namespace frc971 {
namespace input {
namespace driver_station {

// A class to wrap driver_station::Data and map logical joystick numbers to
// their actual numbers in the order they are on the driverstation.
//
// Bits 13 and 14 of the joystick bitmap are defined to be a two bit number
// corresponding to the joystick's logical joystick number.
class RedundantData : public Data {
 public:
  RedundantData(const Data &data);

  bool IsPressed(POVLocation location) const override;
  bool PosEdge(POVLocation location) const override;
  bool NegEdge(POVLocation location) const override;

  // Returns the current and previous "values" for the POV.
  int32_t GetPOV(int joystick) const override;
  int32_t GetOldPOV(int joystick) const override;

  bool IsPressed(ButtonLocation location) const override;
  bool PosEdge(ButtonLocation location) const override;
  bool NegEdge(ButtonLocation location) const override;

  bool GetControlBit(ControlBit bit) const override;
  bool PosEdge(ControlBit bit) const override;
  bool NegEdge(ControlBit bit) const override;

  // Returns the value in the range [-1.0, 1.0].
  float GetAxis(JoystickAxis axis) const override;

 private:
  static constexpr int kIdBit0Button = 14;
  static constexpr int kIdBit1Button = 15;
  static constexpr int kRedundantBitButton = 16;

  int MapRedundantJoystick(int joystick) const;

  // A mapping from logical joystick numbers to their actual order on the
  // driverstation.
  //
  // Index is logical joystick number, Value is mapped joystick number.
  std::array<int, JoystickFeature::kJoysticks> joystick_map_;

  const Data &data_;
};

}  // namespace driver_station
}  // namespace input
}  // namespace frc971

#endif  // AOS_INPUT_REDUNDANT_JOYSTICK_DATA_H_
