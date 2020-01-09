#include "frc971/wpilib/fpga_time_conversion.h"

#include "aos/util/compiler_memory_barrier.h"

namespace frc971 {
namespace wpilib {

std::optional<std::chrono::nanoseconds> CalculateFpgaOffset() {
  aos_compiler_memory_barrier();
  const hal::fpga_clock::time_point fpga_time_before = hal::fpga_clock::now();
  aos_compiler_memory_barrier();
  const aos::monotonic_clock::time_point monotonic_now =
      aos::monotonic_clock::now();
  aos_compiler_memory_barrier();
  const hal::fpga_clock::time_point fpga_time_after = hal::fpga_clock::now();
  aos_compiler_memory_barrier();

  const std::chrono::nanoseconds fpga_sample_length =
      fpga_time_after - fpga_time_before;

  if (fpga_sample_length < fpga_sample_length.zero()) {
    return std::nullopt;
  }
  if (fpga_sample_length > std::chrono::microseconds(20)) {
    return std::nullopt;
  }

  const std::chrono::nanoseconds fpga_average =
      (fpga_time_after.time_since_epoch() +
       fpga_time_before.time_since_epoch()) /
      2;
  return monotonic_now.time_since_epoch() - fpga_average;
}

}  // namespace wpilib
}  // namespace frc971
