#ifndef FRC971_WPILIB_FPGA_TIME_CONVERSION_H_
#define FRC971_WPILIB_FPGA_TIME_CONVERSION_H_

#include <chrono>
#include <optional>

#include "glog/logging.h"

#include "aos/time/time.h"
#include "hal/cpp/fpga_clock.h"

namespace frc971 {
namespace wpilib {

// Returns the offset from the monotonic clock to the FPGA time. This is defined
// as `fpga_time - monotonic_time`.
// Returns nullopt if sampling the time once failed.
std::optional<std::chrono::nanoseconds> CalculateFpgaOffset();

class FpgaTimeConverter {
 public:
  aos::monotonic_clock::time_point FpgaToMonotonic(
      hal::fpga_clock::time_point fpga_time) {
    UpdateOffset();
    return aos::monotonic_clock::epoch() +
           (fpga_time.time_since_epoch() + offset_);
  }

  hal::fpga_clock::time_point MonotonicToFpga(
      aos::monotonic_clock::time_point monotonic_time) {
    UpdateOffset();
    return hal::fpga_clock::epoch() +
           std::chrono::duration_cast<hal::fpga_clock::duration>(
               monotonic_time.time_since_epoch() - offset_);
  }

 private:
  void UpdateOffset() {
    for (int i = 0; i < 10; ++i) {
      const auto new_offset = CalculateFpgaOffset();
      if (new_offset) {
        offset_ = *new_offset;
        return;
      } else if (offset_ != offset_.min()) {
        return;
      }
    }
    LOG(FATAL) << "Failed to calculate FPGA offset";
  }

  std::chrono::nanoseconds offset_ = std::chrono::nanoseconds::min();
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_FPGA_TIME_CONVERSION_H_
