#ifndef FRC971_WPILIB_SENSOR_READER_H_
#define FRC971_WPILIB_SENSOR_READER_H_

#include <atomic>
#include <chrono>

#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc971 {
namespace wpilib {

class SensorReader {
 public:
  SensorReader();

  // All of the DMA-related set_* calls must be made before this, and it
  // doesn't hurt to do all of them.
  void set_dma(::std::unique_ptr<DMA> dma);

  void set_pwm_trigger(::std::unique_ptr<frc::DigitalInput> pwm_trigger);

 protected:
  void RunPWMDetecter();

  ::std::unique_ptr<frc::DigitalInput> pwm_trigger_;

  frc::DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_,
      hall_filter_;

  // Mutex to manage access to the period and tick time variables.
  ::aos::stl_mutex tick_time_mutex_;
  monotonic_clock::time_point last_tick_time_monotonic_timepoint_ =
      monotonic_clock::min_time;
  chrono::nanoseconds last_period_ = chrono::microseconds(5050);

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_SENSOR_READER_H_
