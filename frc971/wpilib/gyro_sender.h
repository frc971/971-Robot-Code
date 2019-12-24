#ifndef FRC971_WPILIB_GYRO_H_
#define FRC971_WPILIB_GYRO_H_

#include <stdint.h>

#include <atomic>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/robot_state/robot_state_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/gyro_interface.h"
#include "frc971/zeroing/averager.h"

namespace frc971 {
namespace wpilib {

// Handles reading the gyro over SPI and sending out angles on a queue.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class GyroSender {
 public:
  GyroSender(::aos::ShmEventLoop *event_loop);

  enum class State { INITIALIZING, RUNNING };

 private:
  // Initializes the gyro and then loops until Exit() is called on the event
  // loop, taking readings.
  void Loop(const int iterations);

  ::aos::EventLoop *event_loop_;
  ::aos::Fetcher<::aos::RobotState> joystick_state_fetcher_;
  ::aos::Sender<::frc971::sensors::Uid> uid_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_reading_sender_;

  // Readings per second.
  static constexpr int kReadingRate = 200;

  GyroInterface gyro_;

  State state_ = State::INITIALIZING;

  // In radians, ready to send out.
  double angle_ = 0;
  // Calibrated offset.
  double zero_offset_ = 0;

  ::aos::monotonic_clock::time_point last_initialize_time_ =
      ::aos::monotonic_clock::min_time;
  int startup_cycles_left_ = 2 * kReadingRate;

  zeroing::Averager<double, 6 * kReadingRate> zeroing_data_;

  bool zeroed_ = false;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_GYRO_H_
