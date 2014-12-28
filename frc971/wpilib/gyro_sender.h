#ifndef FRC971_WPILIB_GYRO_H_
#define FRC971_WPILIB_GYRO_H_

#include <stdint.h>

#include <atomic>

#include "frc971/wpilib/gyro_interface.h"

namespace frc971 {
namespace wpilib {

// Handles reading the gyro over SPI and sending out angles on a queue.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class GyroSender {
 public:
  GyroSender();

  // For ::std::thread to call.
  //
  // Initializes the gyro and then loops forever taking readings.
  void operator()();

  void Quit() { run_ = false; }

 private:

  // Readings per second.
  static const int kReadingRate = 200;

  GyroInterface gyro_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_GYRO_H_
