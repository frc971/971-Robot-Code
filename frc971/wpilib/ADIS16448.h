#ifndef FRC971_WPILIB_ADIS16448_H_
#define FRC971_WPILIB_ADIS16448_H_

#include <stdint.h>

#include <memory>
#include <atomic>

#include "SPI.h"
#include "DigitalInput.h"
#include "DigitalOutput.h"
#undef ERROR

#include "aos/common/logging/logging.h"

namespace frc971 {
namespace wpilib {

// Handles interfacing with an Analog Devices ADIS16448 Inertial Sensor over
// SPI and sending values out on a queue.
//
// The sensor is configured to generate samples at 204.8 Hz, and the values are
// sent out as each sample is received.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class ADIS16448 {
 public:
  // port is where to find the sensor over SPI.
  // dio1 must be connected to DIO1 on the sensor.
  ADIS16448(SPI::Port port, DigitalInput *dio1);

  // Sets the dummy SPI port to send values on to make the roboRIO deassert the
  // chip select line. This is mainly useful when there are no other devices
  // sharing the bus.
  void SetDummySPI(SPI::Port port);

  // Sets the reset line for the IMU to use for error recovery.
  void set_reset(DigitalOutput *output) { reset_ = output; }

  // For ::std::thread to call.
  //
  // Initializes the sensor and then loops until Quit() is called taking
  // readings.
  void operator()();

  void Quit() { run_ = false; }

  double gyro_x_zeroed_offset() const { return gyro_x_zeroed_offset_; }
  double gyro_y_zeroed_offset() const { return gyro_y_zeroed_offset_; }
  double gyro_z_zeroed_offset() const { return gyro_z_zeroed_offset_; }

 private:
  // Try to initialize repeatedly as long as we're supposed to be running.
  void InitializeUntilSuccessful();

  // Converts a 16-bit value at data to a scaled output value where a value of 1
  // corresponds to lsb_per_output.
  float ConvertValue(uint8_t *data, double lsb_per_output, bool sign = true);

  // Performs an SPI transaction.
  // Returns true if it succeeds.
  template <uint8_t size>
  bool DoTransaction(uint8_t to_send[size], uint8_t to_receive[size]);

  // Reads one of the gyro's registers and returns the value in value.
  // next_address is the address of the *next* register to read.
  // Not sure what gets stored in value for the first read, but it should be
  // ignored. Passing nullptr for value is allowed to completely ignore it.
  // Returns true if it succeeds.
  bool ReadRegister(uint8_t next_address, uint16_t *value);

  // Writes a value to one of the registers.
  // Returns true if it succeeds.
  bool WriteRegister(uint8_t address, uint16_t value);

  // Checks the given value of the DIAG_STAT register and logs any errors.
  // Returns true if there are no errors we care about.
  bool CheckDiagStatValue(uint16_t value) const;

  // Starts everything up and runs a self test.
  // Returns true if it succeeds.
  bool Initialize();

  // TODO(Brian): This object has no business owning these ones.
  const ::std::unique_ptr<SPI> spi_;
  ::std::unique_ptr<SPI> dummy_spi_;
  DigitalInput *const dio1_;
  DigitalOutput *reset_ = nullptr;

  ::std::atomic<bool> run_{true};

  // The averaged values of the gyro over 6 seconds after power up.
  bool gyros_are_zeroed_ = false;
  double gyro_x_zeroed_offset_ = 0.0;
  double gyro_y_zeroed_offset_ = 0.0;
  double gyro_z_zeroed_offset_ = 0.0;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_ADIS16448_H_
