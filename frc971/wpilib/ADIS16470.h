#ifndef FRC971_WPILIB_ADIS16470_H_
#define FRC971_WPILIB_ADIS16470_H_

#include "absl/types/span.h"

#include "aos/events/event_loop.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/ahal/DigitalOutput.h"
#include "frc971/wpilib/ahal/DigitalSource.h"
#include "frc971/wpilib/ahal/SPI.h"
#include "frc971/wpilib/fpga_time_conversion.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "frc971/wpilib/imu_generated.h"

namespace frc971 {
namespace wpilib {

// Handles interfacing with an Analog Devices ADIS16470 over SPI and sending the
// resulting values out on a channel.
//
// This relies on the AutoRead functionality in the FPGA to read values when
// data is ready. It then allows the FPGA to buffer those until right before the
// relevant control loops run, at which point they are all sent out on the
// relevant channel.
class ADIS16470 {
 public:
  // event_loop's thread will be hijacked before processing any events.
  // spi is how to talk to the sensor over SPI.
  // data_ready is the Data Ready (DR) pin (J6).
  // reset is the Reset (RST) pin (F3).
  ADIS16470(aos::EventLoop *event_loop, frc::SPI *spi,
            frc::DigitalInput *data_ready, frc::DigitalOutput *reset);

  ADIS16470(const ADIS16470 &) = delete;
  ADIS16470 &operator=(const ADIS16470 &) = delete;

  // Reads all the queued-up data and sends out any complete readings.
  void DoReads();

 private:
  enum class State {
    kUninitialized,
    kWaitForReset,
    kRunning,
  };

  // Performs one (non-blocking) initialization step.
  void DoInitializeStep();

  // Processes a complete reading in read_data_.
  flatbuffers::Offset<IMUValues> ProcessReading(
      flatbuffers::FlatBufferBuilder *fbb);

  // Converts a 32-bit value at data to a scaled output value where a value of 1
  // corresponds to lsb_per_output.
  static double ConvertValue32(absl::Span<const uint32_t> data,
                               double lsb_per_output);
  static double ConvertValue16(absl::Span<const uint32_t> data,
                               double lsb_per_output);

  static flatbuffers::Offset<ADIS16470DiagStat> PackDiagStat(
      flatbuffers::FlatBufferBuilder *fbb, uint16_t value);

  static bool DiagStatHasError(const ADIS16470DiagStat &diag_stat);

  // These may only be called during configuration, when spi_ is not in
  // automatic mode.
  uint16_t ReadRegister(uint8_t register_address,
                        uint8_t next_register_address);
  void WriteRegister(uint8_t register_address, uint16_t value);

  void BeginInitialization() {
    state_ = State::kUninitialized;
    initialize_timer_->Schedule(event_loop_->monotonic_now() +
                                std::chrono::milliseconds(25));
  }

  aos::EventLoop *const event_loop_;
  aos::Sender<::frc971::IMUValuesBatch> imu_values_sender_;
  aos::TimerHandler *const initialize_timer_;

  frc::SPI *const spi_;
  frc::DigitalInput *const data_ready_;
  frc::DigitalOutput *const reset_;

  State state_ = State::kUninitialized;

  // Data we've read from the FPGA.
  std::array<uint32_t, 23> read_data_;
  // Data that we need to read from the FPGA to get a complete reading.
  absl::Span<uint32_t> to_read_;

  FpgaTimeConverter time_converter_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_ADIS16470_H_
