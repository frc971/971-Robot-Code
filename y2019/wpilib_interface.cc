#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/make_unique.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/wpilib_interface.h"
#include "frc971/wpilib/wpilib_robot_base.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
using aos::make_unique;

namespace y2019 {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

// TODO(brian): Use ::std::max instead once we have C++14 so that can be
// constexpr.
template <typename T>
constexpr T max(T a, T b) {
  return (a > b) ? a : b;
}

template <typename T, typename... Rest>
constexpr T max(T a, T b, T c, Rest... rest) {
  return max(max(a, b), c, rest...);
}

double drivetrain_translate(int32_t in) {
  return ((static_cast<double>(in)
           /* / Values::kDrivetrainEncoderCountsPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius*/));
}

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) /* / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius*/));
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(/*Values::kMaxDrivetrainEncoderPulsesPerSecond(),
        Values::kMaxIntakeMotorEncoderPulsesPerSecond()*/ 1.0, 1.0);
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(/*Values::kMaxProximalEncoderPulsesPerSecond(),
        Values::kMaxDistalEncoderPulsesPerSecond()*/ 1.0, 1.0);
static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

// Class to send position messages with sensor readings to our loops.
class SensorReader {
 public:
  SensorReader() {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    fast_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxFastEncoderPulsesPerSecond * 1e9 +
                         0.5));
    medium_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxMediumEncoderPulsesPerSecond * 1e9 +
                         0.5));
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  // Left drivetrain side.
  void set_drivetrain_left_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_left_encoder_ = ::std::move(encoder);
  }

  // Right drivetrain side.
  void set_drivetrain_right_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  void set_pwm_trigger(::std::unique_ptr<frc::DigitalInput> pwm_trigger) {
    medium_encoder_filter_.Add(pwm_trigger.get());
    pwm_trigger_ = ::std::move(pwm_trigger);
  }

  // All of the DMA-related set_* calls must be made before this, and it
  // doesn't hurt to do all of them.
  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(
        new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
  }

  void RunPWMDetecter() {
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
        const monotonic_clock::time_point monotonic_now =
            monotonic_clock::now();
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
        if (elapsed_time >
                chrono::microseconds(5050) + chrono::microseconds(4) ||
            elapsed_time <
                chrono::microseconds(5050) - chrono::microseconds(4)) {
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

  void operator()() {
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

  void RunIteration() {
    ::frc971::wpilib::SendRobotState(my_pid_);

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());

      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->right_speed = -drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();
  }

  void Quit() { run_ = false; }

 private:
  double encoder_translate(int32_t value, double counts_per_revolution,
                           double ratio) {
    return static_cast<double>(value) / counts_per_revolution * ratio *
           (2.0 * M_PI);
  }

  int32_t my_pid_;

  // Mutex to manage access to the period and tick time variables.
  ::aos::stl_mutex tick_time_mutex_;
  monotonic_clock::time_point last_tick_time_monotonic_timepoint_ =
      monotonic_clock::min_time;
  chrono::nanoseconds last_period_ = chrono::microseconds(5050);

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  frc::DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_,
      hall_filter_;

  ::std::unique_ptr<frc::Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

  ::std::unique_ptr<frc::DigitalInput> pwm_trigger_;

  ::std::atomic<bool> run_{true};
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_drivetrain_left_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_left_victor_ = ::std::move(t);
  }

  void set_drivetrain_right_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_right_victor_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    drivetrain_left_victor_->SetSpeed(
        ::aos::Clip(queue->left_voltage, -12.0, 12.0) / 12.0);
    drivetrain_right_victor_->SetSpeed(
        ::aos::Clip(-queue->right_voltage, -12.0, 12.0) / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left_victor_->SetDisabled();
    drivetrain_right_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> drivetrain_left_victor_,
      drivetrain_right_victor_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::frc971::wpilib::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader;

    // TODO(Sabina): Update port numbers(Sensors and Victors)
    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    reader.set_pwm_trigger(make_unique<frc::DigitalInput>(25));

    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    auto imu_trigger = make_unique<frc::DigitalInput>(5);
    ::frc971::wpilib::ADIS16448 imu(frc::SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.SetDummySPI(frc::SPI::Port::kOnboardCS2);
    auto imu_reset = make_unique<frc::DigitalOutput>(6);
    imu.set_reset(imu_reset.get());
    ::std::thread imu_thread(::std::ref(imu));

    // While as of 2/9/18 the drivetrain Victors are SPX, it appears as though
    // they are identical, as far as DrivetrainWriter is concerned, to the SP
    // variety so all the Victors are written as SPs.

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_drivetrain_left_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    drivetrain_writer.set_drivetrain_right_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    // Wait forever. Not much else to do...
    while (true) {
      const int r = select(0, nullptr, nullptr, nullptr, nullptr);
      if (r != 0) {
        PLOG(WARNING, "infinite select failed");
      } else {
        PLOG(WARNING, "infinite select succeeded??\n");
      }
    }

    LOG(ERROR, "Exiting WPILibRobot\n");

    joystick_sender.Quit();
    joystick_thread.join();
    pdp_fetcher.Quit();
    pdp_fetcher_thread.join();
    reader.Quit();
    reader_thread.join();
    imu.Quit();
    imu_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2019

AOS_ROBOT_CLASS(::y2019::wpilib::WPILibRobot);
