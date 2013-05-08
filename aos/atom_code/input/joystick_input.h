#ifndef AOS_ATOM_CODE_INPUT_JOYSTICKS_INPUT_H_
#define AOS_ATOM_CODE_INPUT_JOYSTICKS_INPUT_H_

#include "aos/common/input/driver_station_data.h"

namespace aos {
namespace input {

class JoystickInput {
 public:
  void Run();

 private:
  virtual void RunIteration(const driver_station::Data &data) = 0;
};

}  // namespace input
}  // namespace aos

#endif  // AOS_ATOM_CODE_INPUT_JOYSTICKS_INPUT_H_
