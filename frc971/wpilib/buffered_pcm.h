#ifndef FRC971_WPILIB_BUFFERED_PCM_H_
#define FRC971_WPILIB_BUFFERED_PCM_H_

#include <memory>

#include <hal/HAL.h>

#include "frc971/wpilib/buffered_solenoid.h"

namespace frc971 {
namespace wpilib {

// Manages setting values for all solenoids for a single PCM in a single CAN
// message.
//
// The way to use this is to call MakeSolenoid for whichever solenoid numbers
// you want, call Set on those, and then periodically call Flush on this object
// to write all of the buffered values out.
class BufferedPcm {
 public:
  explicit BufferedPcm(int module = 0);

  // Creates a new BufferedSolenoid for a specified port number.
  ::std::unique_ptr<BufferedSolenoid> MakeSolenoid(int number);

  // Returns a bitmask of the state of all the solenoids.
  int32_t GetAll();

  // Actually sends all of the values out.
  void Flush();

 private:
  void DoSet(int number, bool value);

  int module_;
  ::std::array<HAL_SolenoidHandle, 8> solenoid_handles_{
      {HAL_kInvalidHandle, HAL_kInvalidHandle, HAL_kInvalidHandle,
       HAL_kInvalidHandle, HAL_kInvalidHandle, HAL_kInvalidHandle,
       HAL_kInvalidHandle, HAL_kInvalidHandle}};

  uint8_t values_ = 0;

  friend class BufferedSolenoid;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_BUFFERED_PCM_H_
