#include "frc971/wpilib/sensor_reader.h"

#include <inttypes.h>
#include <unistd.h>

#include "aos/init.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/phased_loop.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/ahal/Utility.h"

namespace frc971 {
namespace wpilib {

SensorReader::SensorReader() {}

void SensorReader::set_drivetrain_left_encoder(
    ::std::unique_ptr<frc::Encoder> encoder) {
  fast_encoder_filter_.Add(encoder.get());
  drivetrain_left_encoder_ = ::std::move(encoder);
}

void SensorReader::set_drivetrain_right_encoder(
    ::std::unique_ptr<frc::Encoder> encoder) {
  fast_encoder_filter_.Add(encoder.get());
  drivetrain_right_encoder_ = ::std::move(encoder);
}

// All of the DMA-related set_* calls must be made before this, and it
// doesn't hurt to do all of them.
void SensorReader::set_dma(::std::unique_ptr<DMA> dma) {
  dma_synchronizer_.reset(
      new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
}

void SensorReader::set_pwm_trigger(
    ::std::unique_ptr<frc::DigitalInput> pwm_trigger) {
  medium_encoder_filter_.Add(pwm_trigger.get());
  pwm_trigger_ = ::std::move(pwm_trigger);
}

void SensorReader::RunPWMDetecter() {
  ::aos::SetCurrentThreadRealtimePriority(41);

  pwm_trigger_->RequestInterrupts();
  // Rising edge only.
  pwm_trigger_->SetUpSourceEdge(true, false);

  monotonic_clock::time_point last_posedge_monotonic =
      monotonic_clock::min_time;

  while (run_) {
    auto ret = pwm_trigger_->WaitForInterrupt(1.0, true);
    if (ret == frc::InterruptableSensorBase::WaitResult::kRisingEdge) {
      // Grab all the clocks.
      const double pwm_fpga_time = pwm_trigger_->ReadRisingTimestamp();

      aos_compiler_memory_barrier();
      const double fpga_time_before = frc::GetFPGATime() * 1e-6;
      aos_compiler_memory_barrier();
      const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
      aos_compiler_memory_barrier();
      const double fpga_time_after = frc::GetFPGATime() * 1e-6;
      aos_compiler_memory_barrier();

      const double fpga_offset =
          (fpga_time_after + fpga_time_before) / 2.0 - pwm_fpga_time;

      // Compute when the edge was.
      const monotonic_clock::time_point monotonic_edge =
          monotonic_now - chrono::duration_cast<chrono::nanoseconds>(
                              chrono::duration<double>(fpga_offset));

      LOG(DEBUG, "Got PWM pulse %f spread, %f offset, %lld trigger\n",
          fpga_time_after - fpga_time_before, fpga_offset,
          monotonic_edge.time_since_epoch().count());

      // Compute bounds on the timestep and sampling times.
      const double fpga_sample_length = fpga_time_after - fpga_time_before;
      const chrono::nanoseconds elapsed_time =
          monotonic_edge - last_posedge_monotonic;

      last_posedge_monotonic = monotonic_edge;

      // Verify that the values are sane.
      if (fpga_sample_length > 2e-5 || fpga_sample_length < 0) {
        continue;
      }
      if (fpga_offset < 0 || fpga_offset > 0.00015) {
        continue;
      }
      if (elapsed_time > chrono::microseconds(5050) + chrono::microseconds(4) ||
          elapsed_time < chrono::microseconds(5050) - chrono::microseconds(4)) {
        continue;
      }
      // Good edge!
      {
        ::std::unique_lock<::aos::stl_mutex> locker(tick_time_mutex_);
        last_tick_time_monotonic_timepoint_ = last_posedge_monotonic;
        last_period_ = elapsed_time;
      }
    } else {
      LOG(INFO, "PWM triggered %d\n", ret);
    }
  }
  pwm_trigger_->CancelInterrupts();
}

void SensorReader::operator()() {
  ::aos::SetCurrentThreadName("SensorReader");

  my_pid_ = getpid();

  dma_synchronizer_->Start();

  ::aos::time::PhasedLoop phased_loop(last_period_,
                                      ::std::chrono::milliseconds(3));
  chrono::nanoseconds filtered_period = last_period_;

  ::std::thread pwm_detecter_thread(
      ::std::bind(&SensorReader::RunPWMDetecter, this));

  ::aos::SetCurrentThreadRealtimePriority(40);
  while (run_) {
    {
      const int iterations = phased_loop.SleepUntilNext();
      if (iterations != 1) {
        LOG(WARNING, "SensorReader skipped %d iterations\n", iterations - 1);
      }
    }
    RunIteration();

    monotonic_clock::time_point last_tick_timepoint;
    chrono::nanoseconds period;
    {
      ::std::unique_lock<::aos::stl_mutex> locker(tick_time_mutex_);
      last_tick_timepoint = last_tick_time_monotonic_timepoint_;
      period = last_period_;
    }

    if (last_tick_timepoint == monotonic_clock::min_time) {
      continue;
    }
    chrono::nanoseconds new_offset = phased_loop.OffsetFromIntervalAndTime(
        period, last_tick_timepoint + chrono::microseconds(2050));

    // TODO(austin): If this is the first edge in a while, skip to it (plus
    // an offset). Otherwise, slowly drift time to line up.

    phased_loop.set_interval_and_offset(period, new_offset);
  }
  pwm_detecter_thread.join();
}

}  // namespace wpilib
}  // namespace frc971
