#ifndef FRC971_WPILIB_JOYSTICK_SENDER_H_
#define FRC971_WPILIB_JOYSTICK_SENDER_H_

#include <atomic>

#include "aos/events/event-loop.h"
#include "aos/robot_state/robot_state.q.h"

namespace frc971 {
namespace wpilib {

class JoystickSender {
 public:
  JoystickSender(::aos::EventLoop *event_loop);

 private:
  ::aos::EventLoop *event_loop_;
  ::aos::Sender<::aos::JoystickState> joystick_state_sender_;
  const uint16_t team_id_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_JOYSTICK_SENDER_H_
