#ifndef FRC971_WPILIB_JOYSTICK_SENDER_H_
#define FRC971_WPILIB_JOYSTICK_SENDER_H_

#include <atomic>

namespace frc971 {
namespace wpilib {

class JoystickSender {
 public:
  void operator()();

  void Quit() { run_ = false; }

 private:
  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_JOYSTICK_SENDER_H_
