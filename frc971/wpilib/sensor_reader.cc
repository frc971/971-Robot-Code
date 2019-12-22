#include "frc971/wpilib/sensor_reader.h"

#include <inttypes.h>
#include <unistd.h>

#include "aos/init.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/phased_loop.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Utility.h"
#include "frc971/wpilib/wpilib_interface.h"
#include "hal/PWM.h"

namespace frc971 {
namespace wpilib {

SensorReader::SensorReader(::aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      robot_state_sender_(event_loop_->MakeSender<::aos::RobotState>("/aos")),
      my_pid_(getpid()) {
  // Set some defaults.  We don't tend to exceed these, so old robots should
  // just work with them.
  UpdateFastEncoderFilterHz(500000);
  UpdateMediumEncoderFilterHz(100000);
  ds_ = &::frc::DriverStation::GetInstance();

  event_loop->SetRuntimeRealtimePriority(40);

  // Fill in the no pwm trigger defaults.
  phased_loop_handler_ =
      event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                                 period_, chrono::milliseconds(4));

  event_loop->set_name("SensorReader");
  event_loop->OnRun([this]() { DoStart(); });
}

void SensorReader::UpdateFastEncoderFilterHz(int hz) {
  fast_encoder_filter_.SetPeriodHz(::std::max(hz, 100000));
}

void SensorReader::UpdateMediumEncoderFilterHz(int hz) {
  medium_encoder_filter_.SetPeriodHz(::std::max(hz, 50000));
}

void SensorReader::set_drivetrain_left_encoder(
    ::std::unique_ptr<frc::Encoder> encoder) {
  fast_encoder_filter_.Add(encoder.get());
  drivetrain_left_encoder_ = ::std::move(encoder);
  drivetrain_left_encoder_->SetMaxPeriod(0.005);
}

void SensorReader::set_drivetrain_right_encoder(
    ::std::unique_ptr<frc::Encoder> encoder) {
  fast_encoder_filter_.Add(encoder.get());
  drivetrain_right_encoder_ = ::std::move(encoder);
  drivetrain_right_encoder_->SetMaxPeriod(0.005);
}

monotonic_clock::time_point SensorReader::GetPWMStartTime() {
  int32_t status = 0;
  const hal::fpga_clock::time_point new_fpga_time = hal::fpga_clock::time_point(
      hal::fpga_clock::duration(HAL_GetPWMCycleStartTime(&status)));

  aos_compiler_memory_barrier();
  const hal::fpga_clock::time_point fpga_time_before = hal::fpga_clock::now();
  aos_compiler_memory_barrier();
  const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
  aos_compiler_memory_barrier();
  const hal::fpga_clock::time_point fpga_time_after = hal::fpga_clock::now();
  aos_compiler_memory_barrier();

  const chrono::nanoseconds fpga_sample_length =
      fpga_time_after - fpga_time_before;
  const chrono::nanoseconds fpga_offset =
      hal::fpga_clock::time_point((fpga_time_after.time_since_epoch() +
                                   fpga_time_before.time_since_epoch()) /
                                  2) -
      new_fpga_time;

  // Make sure that there wasn't a context switch while we were sampling the
  // clocks.  If there was, we are better off rejecting the sample than using
  // it.
  if (ds_->IsSysActive() && fpga_sample_length <= chrono::microseconds(20) &&
      fpga_sample_length >= chrono::microseconds(0)) {
    // Compute when the edge was.
    return monotonic_now - fpga_offset;
  } else {
    return monotonic_clock::min_time;
  }
}

void SensorReader::DoStart() {
  Start();
  if (dma_synchronizer_) {
    dma_synchronizer_->Start();
  }

  period_ =
      pwm_trigger_ ? chrono::microseconds(5050) : chrono::microseconds(5000);
  if (pwm_trigger_) {
    AOS_LOG(INFO, "Using PWM trigger and a 5.05 ms period\n");
  } else {
    AOS_LOG(INFO, "Defaulting to open loop pwm synchronization\n");
  }

  // Now that we are configured, actually fill in the defaults.
  phased_loop_handler_->set_interval_and_offset(
      period_,
      pwm_trigger_ ? ::std::chrono::milliseconds(3) : chrono::milliseconds(4));

  last_monotonic_now_ = monotonic_clock::now();
}

void SensorReader::Loop(const int iterations) {
  if (iterations != 1) {
    AOS_LOG(WARNING, "SensorReader skipped %d iterations\n", iterations - 1);
  }

  const monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  {
    auto builder = robot_state_sender_.MakeBuilder();
    builder.Send(::frc971::wpilib::PopulateRobotState(&builder, my_pid_));
  }
  RunIteration();
  if (dma_synchronizer_) {
    dma_synchronizer_->RunIteration();
    RunDmaIteration();
  }

  if (pwm_trigger_) {
    AOS_LOG(DEBUG, "PWM wakeup delta: %lld\n",
            (monotonic_now - last_monotonic_now_).count());
    last_monotonic_now_ = monotonic_now;

    monotonic_clock::time_point last_tick_timepoint = GetPWMStartTime();
    if (last_tick_timepoint == monotonic_clock::min_time) {
      return;
    }

    last_tick_timepoint +=
        ((monotonic_now - last_tick_timepoint) / period_) * period_;
    // If it's over 1/2 of a period back in time, that's wrong.  Move it
    // forwards to now.
    if (last_tick_timepoint - monotonic_now < -period_ / 2) {
      last_tick_timepoint += period_;
    }

    // We should be sampling our sensors to kick off the control cycle 50 uS
    // after the falling edge.  This gives us a little bit of buffer for
    // errors in waking up.  The PWM cycle starts at the falling edge of the
    // PWM pulse.
    chrono::nanoseconds new_offset =
        ::aos::time::PhasedLoop::OffsetFromIntervalAndTime(
            period_, last_tick_timepoint + chrono::microseconds(50));

    phased_loop_handler_->set_interval_and_offset(period_, new_offset);
  }
}

}  // namespace wpilib
}  // namespace frc971
