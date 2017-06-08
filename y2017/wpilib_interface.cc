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

#include "AnalogInput.h"
#include "Compressor.h"
#include "Counter.h"
#include "DigitalGlitchFilter.h"
#include "DriverStation.h"
#include "Encoder.h"
#include "Relay.h"
#include "Servo.h"
#include "VictorSP.h"
#undef ERROR

#include "aos/common/commonmath.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/stl_mutex.h"
#include "aos/common/time.h"
#include "aos/common/util/compiler_memory_barrier.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/linux_code/init.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/control_loops.q.h"
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
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::y2017::control_loops::superstructure_queue;
using ::y2017::constants::Values;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace y2017 {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

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

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) /
         Values::kDrivetrainEncoderCountsPerRevolution *
         Values::kDrivetrainEncoderRatio * 2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / Values::kDrivetrainCyclesPerRevolution *
         Values::kDrivetrainEncoderRatio * 2.0 * M_PI;
}

// TODO(Travis): Make sure the number of turns is right.
double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakePotRatio * (3.0 /*turns*/ / 5.0 /*volts*/) *
         (2 * M_PI /*radians*/);
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(Values::kMaxDrivetrainEncoderPulsesPerSecond,
        Values::kMaxShooterEncoderPulsesPerSecond);
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(Values::kMaxIntakeEncoderPulsesPerSecond,
        Values::kMaxTurretEncoderPulsesPerSecond,
        Values::kMaxIndexerEncoderPulsesPerSecond);
static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");
constexpr double kMaxSlowEncoderPulsesPerSecond =
    Values::kMaxHoodEncoderPulsesPerSecond;
static_assert(kMaxSlowEncoderPulsesPerSecond <= 100000,
              "slow encoders are too fast");
static_assert(kMaxSlowEncoderPulsesPerSecond < kMaxMediumEncoderPulsesPerSecond,
              "slow encoders are faster than medium?");

// Handles reading the duty cycle on a DigitalInput.
class DutyCycleReader {
 public:
  void set_input(::std::unique_ptr<DigitalInput> input) {
    high_counter_.reset(new Counter(input.get()));
    high_counter_->SetMaxPeriod(kMaxPeriod);
    high_counter_->SetSemiPeriodMode(true);

    period_length_counter_.reset(new Counter(input.get()));
    period_length_counter_->SetMaxPeriod(kMaxPeriod);
    period_length_counter_->SetUpSourceEdge(true, false);

    input_ = ::std::move(input);
  }

  double Read() const {
    const double high_time = high_counter_->GetPeriod();
    const double period_length = period_length_counter_->GetPeriod();
    if (!::std::isfinite(high_time) || !::std::isfinite(period_length)) {
      return ::std::numeric_limits<double>::quiet_NaN();
    }
    return high_time / period_length;
  }

 private:
  static constexpr ::std::chrono::nanoseconds kNominalPeriod =
      ::std::chrono::microseconds(4096);
  static constexpr double kMaxPeriod =
      (::std::chrono::duration_cast<::std::chrono::duration<double>>(
           kNominalPeriod) *
       2).count();

  ::std::unique_ptr<Counter> high_counter_, period_length_counter_;
  ::std::unique_ptr<DigitalInput> input_;
};

class AbsoluteEncoderAndPotentiometer {
 public:
  void set_absolute_pwm(::std::unique_ptr<DigitalInput> input) {
    duty_cycle_.set_input(::std::move(input));
  }

  void set_encoder(::std::unique_ptr<Encoder> encoder) {
    encoder_ = ::std::move(encoder);
  }

  void set_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    potentiometer_ = ::std::move(potentiometer);
  }

  double ReadAbsoluteEncoder() const { return duty_cycle_.Read(); }

  int32_t ReadRelativeEncoder() const { return encoder_->GetRaw(); }

  double ReadPotentiometerVoltage() const {
    return potentiometer_->GetVoltage();
  }

 private:
  DutyCycleReader duty_cycle_;
  ::std::unique_ptr<Encoder> encoder_;
  ::std::unique_ptr<AnalogInput> potentiometer_;
};

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

  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_left_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  void set_shooter_encoder(::std::unique_ptr<Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    shooter_encoder_ = ::std::move(encoder);
  }

  void set_intake_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_absolute(::std::unique_ptr<DigitalInput> input) {
    intake_encoder_.set_absolute_pwm(::std::move(input));
  }

  void set_indexer_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    indexer_counter_.set_encoder(encoder.get());
    indexer_encoder_ = ::std::move(encoder);
  }

  void set_indexer_hall(::std::unique_ptr<DigitalInput> input) {
    hall_filter_.Add(input.get());
    indexer_counter_.set_input(input.get());
    indexer_hall_ = ::std::move(input);
  }

  void set_turret_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    turret_counter_.set_encoder(encoder.get());
    turret_encoder_ = ::std::move(encoder);
  }

  void set_turret_hall(::std::unique_ptr<DigitalInput> input) {
    hall_filter_.Add(input.get());
    turret_counter_.set_input(input.get());
    turret_hall_ = ::std::move(input);
  }

  void set_hood_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    hood_encoder_.set_encoder(::std::move(encoder));
  }

  void set_hood_index(::std::unique_ptr<DigitalInput> index) {
    medium_encoder_filter_.Add(index.get());
    hood_encoder_.set_index(::std::move(index));
  }

  void set_autonomous_mode(int i, ::std::unique_ptr<DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
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
    dma_synchronizer_->Add(&indexer_counter_);
    dma_synchronizer_->Add(&hood_encoder_);
    dma_synchronizer_->Add(&turret_counter_);
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
    ds_ = &DriverStation::GetInstance();

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
    ::frc971::wpilib::SendRobotState(my_pid_, ds_);

    const auto values = constants::GetValues();

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message->left_encoder =
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();

    {
      auto superstructure_message = superstructure_queue.position.MakeMessage();
      CopyPosition(intake_encoder_, &superstructure_message->intake,
                   Values::kIntakeEncoderCountsPerRevolution,
                   Values::kIntakeEncoderRatio, intake_pot_translate, true,
                   values.intake.pot_offset);

      CopyPosition(indexer_counter_, &superstructure_message->column.indexer,
                   Values::kIndexerEncoderCountsPerRevolution,
                   Values::kIndexerEncoderRatio, true);

      superstructure_message->theta_shooter =
          encoder_translate(shooter_encoder_->GetRaw(),
                            Values::kShooterEncoderCountsPerRevolution,
                            Values::kShooterEncoderRatio);

      CopyPosition(hood_encoder_, &superstructure_message->hood,
                   Values::kHoodEncoderCountsPerRevolution,
                   Values::kHoodEncoderRatio, true);

      CopyPosition(turret_counter_, &superstructure_message->column.turret,
                   Values::kTurretEncoderCountsPerRevolution,
                   Values::kTurretEncoderRatio, false);

      superstructure_message.Send();
    }

    {
      auto auto_mode_message = ::frc971::autonomous::auto_mode.MakeMessage();
      auto_mode_message->mode = 0;
      for (size_t i = 0; i < autonomous_modes_.size(); ++i) {
        if (autonomous_modes_[i] && autonomous_modes_[i]->Get()) {
          auto_mode_message->mode |= 1 << i;
        }
      }
      LOG_STRUCT(DEBUG, "auto mode", *auto_mode_message);
      auto_mode_message.Send();
    }
  }

  void Quit() { run_ = false; }

 private:
  double encoder_translate(int32_t value, double counts_per_revolution,
                           double ratio) {
    return static_cast<double>(value) / counts_per_revolution * ratio *
           (2.0 * M_PI);
  }

  void CopyPosition(const ::frc971::wpilib::DMAEncoder &encoder,
                    ::frc971::IndexPosition *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.polled_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->latched_encoder =
        multiplier * encoder_translate(encoder.last_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->index_pulses = encoder.index_posedge_count();
  }

  void CopyPosition(const AbsoluteEncoderAndPotentiometer &encoder,
                    ::frc971::PotAndAbsolutePosition *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    ::std::function<double(double)> potentiometer_translate,
                    bool reverse, double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->pot = multiplier * potentiometer_translate(
                                     encoder.ReadPotentiometerVoltage()) +
                    pot_offset;
    position->encoder =
        multiplier * encoder_translate(encoder.ReadRelativeEncoder(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);

    position->absolute_encoder =
        (reverse ? (1.0 - encoder.ReadAbsoluteEncoder())
                 : encoder.ReadAbsoluteEncoder()) *
        encoder_ratio * (2.0 * M_PI);
  }

  void CopyPosition(const ::frc971::wpilib::DMAEdgeCounter &counter,
                    ::frc971::HallEffectAndPosition *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->position =
        multiplier * encoder_translate(counter.polled_encoder(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->current = !counter.polled_value();
    position->posedge_count = counter.negative_count();
    position->negedge_count = counter.positive_count();
    position->posedge_value =
        multiplier * encoder_translate(counter.last_negative_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->negedge_value =
        multiplier * encoder_translate(counter.last_positive_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
  }

  int32_t my_pid_;
  DriverStation *ds_;

  // Mutex to manage access to the period and tick time variables.
  ::aos::stl_mutex tick_time_mutex_;
  monotonic_clock::time_point last_tick_time_monotonic_timepoint_ =
      monotonic_clock::min_time;
  chrono::nanoseconds last_period_ = chrono::microseconds(5050);

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_,
      hall_filter_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

  AbsoluteEncoderAndPotentiometer intake_encoder_;

  ::std::unique_ptr<Encoder> indexer_encoder_;
  ::std::unique_ptr<DigitalInput> indexer_hall_;
  ::frc971::wpilib::DMAEdgeCounter indexer_counter_;

  ::std::unique_ptr<Encoder> turret_encoder_;
  ::std::unique_ptr<DigitalInput> turret_hall_;
  ::frc971::wpilib::DMAEdgeCounter turret_counter_;

  ::frc971::wpilib::DMAEncoder hood_encoder_;
  ::std::unique_ptr<Encoder> shooter_encoder_;

  ::std::unique_ptr<DigitalInput> pwm_trigger_;

  ::std::array<::std::unique_ptr<DigitalInput>, 4> autonomous_modes_;

  ::std::atomic<bool> run_{true};
};

class SolenoidWriter {
 public:
  SolenoidWriter()
      : superstructure_(".y2017.control_loops.superstructure_queue.output") {}

  ::frc971::wpilib::BufferedPcm *pcm() { return &pcm_; }

  void set_lights(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    lights_ = ::std::move(s);
  }

  void set_rgb_light(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    rgb_lights_ = ::std::move(s);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(20),
                                        ::std::chrono::milliseconds(1));

    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      {
        superstructure_.FetchLatest();
        if (superstructure_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *superstructure_);
          lights_->Set(superstructure_->lights_on);
          rgb_lights_->Set(superstructure_->red_light_on |
                           superstructure_->green_light_on |
                           superstructure_->blue_light_on);
        }
      }

      pcm_.Flush();
    }
  }

  void Quit() { run_ = false; }

 private:
  ::frc971::wpilib::BufferedPcm pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> lights_, rgb_lights_;

  ::aos::Queue<::y2017::control_loops::SuperstructureQueue::Output>
      superstructure_;

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
    drivetrain_left_victor_->SetSpeed(-queue->left_voltage / 12.0);
    drivetrain_right_victor_->SetSpeed(queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left_victor_->SetDisabled();
    drivetrain_right_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> drivetrain_left_victor_,
      drivetrain_right_victor_;
};

class SuperstructureWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_intake_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_victor_ = ::std::move(t);
  }
  void set_intake_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_rollers_victor_ = ::std::move(t);
  }

  void set_indexer_victor(::std::unique_ptr<::frc::VictorSP> t) {
    indexer_victor_ = ::std::move(t);
  }
  void set_indexer_roller_victor(::std::unique_ptr<::frc::VictorSP> t) {
    indexer_roller_victor_ = ::std::move(t);
  }

  void set_gear_servo(::std::unique_ptr<::frc::Servo> t) {
    gear_servo_ = ::std::move(t);
  }
  void set_shooter_victor(::std::unique_ptr<::frc::VictorSP> t) {
    shooter_victor_ = ::std::move(t);
  }
  void set_turret_victor(::std::unique_ptr<::frc::VictorSP> t) {
    turret_victor_ = ::std::move(t);
  }
  void set_hood_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hood_victor_ = ::std::move(t);
  }

  void set_red_light(::std::unique_ptr<DigitalOutput> t) {
    red_light_ = ::std::move(t);
  }
  void set_green_light(::std::unique_ptr<DigitalOutput> t) {
    green_light_ = ::std::move(t);
  }
  void set_blue_light(::std::unique_ptr<DigitalOutput> t) {
    blue_light_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2017::control_loops::superstructure_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2017::control_loops::superstructure_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake_victor_->SetSpeed(::aos::Clip(queue->voltage_intake,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
    intake_rollers_victor_->SetSpeed(queue->voltage_intake_rollers / 12.0);
    indexer_victor_->SetSpeed(-queue->voltage_indexer / 12.0);
    indexer_roller_victor_->SetSpeed(queue->voltage_indexer_rollers / 12.0);
    turret_victor_->SetSpeed(::aos::Clip(-queue->voltage_turret,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
    hood_victor_->SetSpeed(
        ::aos::Clip(queue->voltage_hood, -kMaxBringupPower, kMaxBringupPower) /
        12.0);
    shooter_victor_->SetSpeed(queue->voltage_shooter / 12.0);

    red_light_->Set(queue->red_light_on);
    green_light_->Set(queue->green_light_on);
    blue_light_->Set(queue->blue_light_on);

    gear_servo_->Set(queue->gear_servo);
  }

  virtual void Stop() override {
    LOG(WARNING, "Superstructure output too old.\n");
    intake_victor_->SetDisabled();
    intake_rollers_victor_->SetDisabled();
    indexer_victor_->SetDisabled();
    indexer_roller_victor_->SetDisabled();
    turret_victor_->SetDisabled();
    hood_victor_->SetDisabled();
    shooter_victor_->SetDisabled();

    gear_servo_->SetOffline();

    red_light_->Set(true);
    green_light_->Set(true);
    blue_light_->Set(true);
  }

  ::std::unique_ptr<::frc::VictorSP> intake_victor_, intake_rollers_victor_,
      indexer_victor_, indexer_roller_victor_, shooter_victor_, turret_victor_,
      hood_victor_;

  ::std::unique_ptr<::frc::Servo> gear_servo_;

  ::std::unique_ptr<DigitalOutput> red_light_, green_light_, blue_light_;
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

    // TODO(campbell): Update port numbers
    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    reader.set_intake_encoder(make_encoder(3));
    reader.set_intake_absolute(make_unique<DigitalInput>(0));
    reader.set_intake_potentiometer(make_unique<AnalogInput>(4));

    reader.set_indexer_encoder(make_encoder(5));
    reader.set_indexer_hall(make_unique<DigitalInput>(4));

    reader.set_turret_encoder(make_encoder(6));
    reader.set_turret_hall(make_unique<DigitalInput>(2));

    reader.set_hood_encoder(make_encoder(4));
    reader.set_hood_index(make_unique<DigitalInput>(1));

    reader.set_shooter_encoder(make_encoder(2));

    reader.set_autonomous_mode(0, make_unique<DigitalInput>(9));
    reader.set_autonomous_mode(1, make_unique<DigitalInput>(8));

    reader.set_pwm_trigger(make_unique<DigitalInput>(7));

    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    auto imu_trigger = make_unique<DigitalInput>(3);
    ::frc971::wpilib::ADIS16448 imu(SPI::Port::kOnboardCS1, imu_trigger.get());
    imu.SetDummySPI(SPI::Port::kOnboardCS2);
    auto imu_reset = make_unique<DigitalOutput>(6);
    imu.set_reset(imu_reset.get());
    ::std::thread imu_thread(::std::ref(imu));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_drivetrain_left_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(7)));
    drivetrain_writer.set_drivetrain_right_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    SuperstructureWriter superstructure_writer;
    superstructure_writer.set_intake_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)));
    superstructure_writer.set_intake_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    superstructure_writer.set_indexer_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    superstructure_writer.set_indexer_roller_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    superstructure_writer.set_turret_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(9)));
    superstructure_writer.set_hood_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    superstructure_writer.set_shooter_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(8)));

    superstructure_writer.set_gear_servo(
        ::std::unique_ptr<Servo>(new Servo(0)));

    superstructure_writer.set_red_light(
        ::std::unique_ptr<DigitalOutput>(new DigitalOutput(5)));
    superstructure_writer.set_green_light(
        ::std::unique_ptr<DigitalOutput>(new DigitalOutput(24)));
    superstructure_writer.set_blue_light(
        ::std::unique_ptr<DigitalOutput>(new DigitalOutput(25)));

    ::std::thread superstructure_writer_thread(
        ::std::ref(superstructure_writer));

    SolenoidWriter solenoid_writer;
    solenoid_writer.set_lights(solenoid_writer.pcm()->MakeSolenoid(0));
    solenoid_writer.set_rgb_light(solenoid_writer.pcm()->MakeSolenoid(1));

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
    imu.Quit();
    imu_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();
    superstructure_writer.Quit();
    superstructure_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2017

AOS_ROBOT_CLASS(::y2017::wpilib::WPILibRobot);
