#ifndef FRC971_WPILIB_BUFFERED_PCM_H_
#define FRC971_WPILIB_BUFFERED_PCM_H_

#include <memory>

#include "SolenoidBase.h"
#undef ERROR

#include "frc971/wpilib/buffered_solenoid.h"

namespace frc971 {
namespace wpilib {

// Manages setting values for all solenoids for a single PCM in a single CAN
// message.
//
// The way to use this is to call MakeSolenoid for whichever solenoid numbers
// you want, call Set on those, and then periodically call Flush on this object
// to write all of the buffered values out.
class BufferedPcm : public SolenoidBase {
 public:
  explicit BufferedPcm(uint8_t module = GetDefaultSolenoidModule())
      : SolenoidBase(module) {}

  // Creates a new BufferedSolenoid for a specified port number.
  ::std::unique_ptr<BufferedSolenoid> MakeSolenoid(int number);

  // Actually sends all of the values out.
  void Flush();

 private:
  // WPILib declares this pure virtual and then never calls it...
#ifdef WPILIB2015
  virtual void InitSolenoid() override {}
#endif

  void Set(int number, bool value);

  uint8_t values_ = 0;

  friend class BufferedSolenoid;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_BUFFERED_PCM_H_
