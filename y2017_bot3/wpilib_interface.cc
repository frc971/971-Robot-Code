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
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/robot_state/robot_state.q.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/init.h"

#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/gyro_sender.h"
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

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017_bot3/control_loops/superstructure/superstructure.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::y2017_bot3::control_loops::superstructure_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
using namespace frc;

namespace y2017_bot3 {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

constexpr double kDrivetrainCyclesPerRevolution = 128.0;
constexpr double kDrivetrainEncoderCountsPerRevolution =
  kDrivetrainCyclesPerRevolution * 4;
constexpr double kDrivetrainEncoderRatio = 1.0;
constexpr double kMaxDrivetrainEncoderPulsesPerSecond =
  control_loops::drivetrain::kFreeSpeed *
  control_loops::drivetrain::kHighOutputRatio /
  kDrivetrainEncoderRatio *
  kDrivetrainEncoderCountsPerRevolution;

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

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

double hall_translate(double in) {
  // Turn voltage from our 3-state halls into a ratio that the loop can use.
  return in / 5.0;
}

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) / (kDrivetrainCyclesPerRevolution /*cpr*/ * 4.0 /*4x*/) *
         kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius *
         2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    kMaxDrivetrainEncoderPulsesPerSecond;
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

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
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_left_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_left_hall(::std::unique_ptr<AnalogInput> analog) {
    drivetrain_left_hall_ = ::std::move(analog);
  }

  void set_drivetrain_right_hall(::std::unique_ptr<AnalogInput> analog) {
    drivetrain_right_hall_ = ::std::move(analog);
  }

  void set_pwm_trigger(::std::unique_ptr<DigitalInput> pwm_trigger) {
    medium_encoder_filter_.Add(pwm_trigger.get());
    pwm_trigger_ = ::std::move(pwm_trigger);
  }

  // All of the DMA-related set_* calls must be made before this, and it
  // doesn't
  // hurt to do all of them.
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
      if (ret == InterruptableSensorBase::WaitResult::kRisingEdge) {
        // Grab all the clocks.
        const double pwm_fpga_time = pwm_trigger_->ReadRisingTimestamp();

        aos_compiler_memory_barrier();
        const double fpga_time_before = GetFPGATime() * 1e-6;
        aos_compiler_memory_barrier();
        const monotonic_clock::time_point monotonic_now =
            monotonic_clock::now();
        aos_compiler_memory_barrier();
        const double fpga_time_after = GetFPGATime() * 1e-6;
        aos_compiler_memory_barrier();

        const double fpga_offset =
            (fpga_time_after + fpga_time_before) / 2.0 - pwm_fpga_time;

        // Compute when the edge was.
        const monotonic_clock::time_point monotonic_edge =
            monotonic_now - chrono::duration_cast<chrono::nanoseconds>(
                                chrono::duration<double>(fpga_offset));

        LOG(INFO, "Got PWM pulse %f spread, %f offset, %lld trigger\n",
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
      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());

      drivetrain_message->left_shifter_position =
          hall_translate(drivetrain_left_hall_->GetVoltage());
      drivetrain_message->right_shifter_position =
          hall_translate(drivetrain_right_hall_->GetVoltage());

      drivetrain_message.Send();
    }

    {
      auto superstructure_message = superstructure_queue.position.MakeMessage();
      superstructure_message.Send();
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

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

  ::std::unique_ptr<AnalogInput> drivetrain_left_hall_, drivetrain_right_hall_;

  ::std::unique_ptr<DigitalInput> pwm_trigger_;

  ::std::atomic<bool> run_{true};

  DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_,
      hall_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain_queue.output"),
        superstructure_(
            ".y2017_bot3.control_loops.superstructure_queue.output") {}

  void set_compressor(::std::unique_ptr<Compressor> compressor) {
    compressor_ = ::std::move(compressor);
  }

  void set_left_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    left_drivetrain_shifter_ = ::std::move(s);
  }

  void set_right_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    right_drivetrain_shifter_ = ::std::move(s);
  }

  void set_fingers(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    fingers_ = ::std::move(s);
  }

  void operator()() {
    compressor_->Start();
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(20),
                                        ::std::chrono::milliseconds(1));

    while (run_) {
      {
        int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      {
        drivetrain_.FetchLatest();
        if (drivetrain_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *drivetrain_);
          left_drivetrain_shifter_->Set(!drivetrain_->left_high);
          right_drivetrain_shifter_->Set(!drivetrain_->right_high);
        }
      }

      {
        superstructure_.FetchLatest();
        if (superstructure_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *superstructure_);
          fingers_->Set(superstructure_->fingers_out);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;
        { to_log.compressor_on = compressor_->Enabled(); }

        pcm_->Flush();
        to_log.read_solenoids = pcm_->GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }
    }
  }

  void Quit() { run_ = false; }

 private:
  const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid>
      left_drivetrain_shifter_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid>
      right_drivetrain_shifter_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> fingers_;

  ::std::unique_ptr<Compressor> compressor_;

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;
  ::aos::Queue<::y2017_bot3::control_loops::SuperstructureQueue::Output>
      superstructure_;

  ::std::atomic<bool> run_{true};
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_drivetrain_left0_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_left0_victor_ = ::std::move(t);
  }
  void set_drivetrain_left1_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_left1_victor_ = ::std::move(t);
  }

  void set_drivetrain_right0_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_right0_victor_ = ::std::move(t);
  }
  void set_drivetrain_right1_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_right1_victor_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    drivetrain_left0_victor_->SetSpeed(queue->left_voltage / 12.0);
    drivetrain_left1_victor_->SetSpeed(queue->left_voltage / 12.0);
    drivetrain_right0_victor_->SetSpeed(-queue->right_voltage / 12.0);
    drivetrain_right1_victor_->SetSpeed(-queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left0_victor_->SetDisabled();
    drivetrain_left1_victor_->SetDisabled();
    drivetrain_right0_victor_->SetDisabled();
    drivetrain_right1_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> drivetrain_left0_victor_,
      drivetrain_left1_victor_, drivetrain_right0_victor_,
      drivetrain_right1_victor_;
};

class SuperstructureWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    rollers_victor_ = ::std::move(t);
  }

  void set_hanger0_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hanger0_victor_ = ::std::move(t);
  }
  void set_hanger1_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hanger1_victor_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2017_bot3::control_loops::superstructure_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2017_bot3::control_loops::superstructure_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    rollers_victor_->SetSpeed(queue->voltage_rollers / 12.0);
    hanger0_victor_->SetSpeed(queue->hanger_voltage / 12.0);
    hanger1_victor_->SetSpeed(queue->hanger_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Superstructure output too old.\n");
    rollers_victor_->SetDisabled();
    hanger0_victor_->SetDisabled();
    hanger1_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> rollers_victor_;
  ::std::unique_ptr<::frc::VictorSP> hanger0_victor_, hanger1_victor_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }

  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::frc971::wpilib::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader;

    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));
    reader.set_drivetrain_left_hall(make_unique<AnalogInput>(0));
    reader.set_drivetrain_right_hall(make_unique<AnalogInput>(1));

    reader.set_pwm_trigger(make_unique<DigitalInput>(0));
    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    ::frc971::wpilib::GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_drivetrain_left0_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    drivetrain_writer.set_drivetrain_left1_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    drivetrain_writer.set_drivetrain_right0_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    drivetrain_writer.set_drivetrain_right1_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    SuperstructureWriter superstructure_writer;
    superstructure_writer.set_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    superstructure_writer.set_hanger0_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)));
    superstructure_writer.set_hanger1_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)));
    ::std::thread superstructure_writer_thread(::std::ref(superstructure_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_left_drivetrain_shifter(pcm->MakeSolenoid(3));
    solenoid_writer.set_right_drivetrain_shifter(pcm->MakeSolenoid(1));
    solenoid_writer.set_fingers(pcm->MakeSolenoid(2));

    solenoid_writer.set_compressor(make_unique<Compressor>());

    ::std::thread solenoid_thread(::std::ref(solenoid_writer));

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
    gyro_sender.Quit();
    gyro_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2017_bot3

AOS_ROBOT_CLASS(::y2017_bot3::wpilib::WPILibRobot);
