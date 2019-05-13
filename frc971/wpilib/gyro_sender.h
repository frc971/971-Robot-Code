#ifndef FRC971_WPILIB_GYRO_H_
#define FRC971_WPILIB_GYRO_H_

#include <stdint.h>

#include <atomic>

#include "aos/events/event-loop.h"
#include "aos/robot_state/robot_state.q.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/wpilib/gyro_interface.h"

namespace frc971 {
namespace wpilib {

// Handles reading the gyro over SPI and sending out angles on a queue.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class GyroSender {
 public:
  GyroSender(::aos::EventLoop *event_loop);

  // For ::std::thread to call.
  //
  // Initializes the gyro and then loops until Quit() is called taking readings.
  void operator()();

  void Quit() { run_ = false; }

 private:
  ::aos::EventLoop *event_loop_;
  ::aos::Fetcher<::aos::JoystickState> joystick_state_fetcher_;
  ::aos::Sender<::frc971::sensors::Uid> uid_sender_;

  // Readings per second.
  static const int kReadingRate = 200;

  GyroInterface gyro_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_GYRO_H_
