#include "frc971/wpilib/sensor_reader.h"

#include <unistd.h>

#include <cinttypes>

#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "aos/util/compiler_memory_barrier.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/fpga_time_conversion.h"
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
  // The timer interrupt fires on CPU1.  Since nothing else is pinned, it will
  // be cheapest to pin this there so it transitions directly and doesn't
  // need to ever migrate.
  event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({1}));

  // Fill in the no pwm trigger defaults.
  timer_handler_ = event_loop_->AddTimer([this]() { Loop(); });
  timer_handler_->set_name("SensorReader Loop");

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
  const auto new_fpga_time =
      hal::fpga_clock::duration(HAL_GetPWMCycleStartTime(&status));

  if (!ds_->IsSysActive()) {
    return monotonic_clock::min_time;
  }

  const auto fpga_offset = CalculateFpgaOffset();
  // If we failed to sample the offset, just ignore this reading.
  if (!fpga_offset) {
    return monotonic_clock::min_time;
  }

  return monotonic_clock::epoch() + (new_fpga_time + *fpga_offset);
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
  timer_handler_->Setup(
      event_loop_->monotonic_now() +
          (pwm_trigger_ ? chrono::milliseconds(3) : chrono::milliseconds(4)),
      period_);

  last_monotonic_now_ = monotonic_clock::now();
}

void SensorReader::Loop() {
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
    // TODO(austin): Put this in a status message.
    VLOG(1) << "PWM wakeup delta: "
            << (monotonic_now - last_monotonic_now_).count();
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
    const auto next_time =
        last_tick_timepoint + period_ + chrono::microseconds(50);

    timer_handler_->Setup(next_time, period_);
  }
}

}  // namespace wpilib
}  // namespace frc971
