#ifndef FRC971_WPILIB_LPD8806_H_
#define FRC971_WPILIB_LPD8806_H_

#include <memory>
#include <atomic>

#include "aos/common/mutex.h"

#include "SPI.h"
#undef ERROR

namespace frc971 {
namespace wpilib {

// Handles sending out colors to a string of LPD8806 LED drivers with a chip to
// gate the clock+data lines attached to SPI CS1.
//
// This is currently implemented by sending data for an entire 64 LED strip
// right after receiving a gyro message to avoid interfering with the SPI
// transactions for the gyro.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class LPD8806 {
 public:
  // chips is the number of actual chips in the string. There will be twice this
  // many LEDs.
  LPD8806(int chips);

  // For ::std::thread to call.
  //
  // Loops until Quit() is called writing values out.
  void operator()();

  void Quit() { run_ = false; }

  int chips() const { return chips_; }

  // Sets the color to be sent out next refresh cycle for a given LED.
  // red, green, and blue are 0-1 (inclusive).
  void SetColor(int led, uint32_t hex_color);

 private:
  struct LED {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
  };

  enum Type { GREEN, RED, BLUE };

  static_assert(sizeof(LED[4]) == 12, "LED needs to pack into arrays nicely");

  uint8_t TranslateColor(uint32_t hex_color, Type type);

  const int chips_;
  int next_led_to_send_ = 0;

  ::std::unique_ptr<LED[]> data_;
  ::aos::Mutex data_mutex_;

  ::std::unique_ptr<SPI> spi_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_LPD8806_H_
