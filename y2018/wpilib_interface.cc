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
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Servo.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#include "ctre/phoenix/CANifier.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state.q.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"

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
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"
#include "y2018/status_light.q.h"
#include "y2018/vision/vision.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::y2018::control_loops::superstructure_queue;
using ::y2018::constants::Values;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
using aos::make_unique;

namespace y2018 {
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
  return ((static_cast<double>(in) /
           Values::kDrivetrainEncoderCountsPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

double proximal_pot_translate(double voltage) {
  return -voltage * Values::kProximalPotRatio() *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double distal_pot_translate(double voltage) {
  return voltage * Values::kDistalPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakeMotorPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double intake_spring_translate(double voltage) {
  return voltage * Values::kIntakeSpringRatio() * (2 * M_PI /*radians*/) /
         (5.0 /*volts*/);
}

// TODO() figure out differnce between max and min voltages on shifter pots.
// Returns value from 0.0 to 1.0, with 0.0 being close to low gear so it can be
// passed drectly into the drivetrain position queue.
double drivetrain_shifter_pot_translate(double voltage) {
  return (voltage - Values::kDrivetrainShifterPotMinVoltage()) /
         (Values::kDrivetrainShifterPotMaxVoltage() -
          Values::kDrivetrainShifterPotMinVoltage());
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(Values::kMaxDrivetrainEncoderPulsesPerSecond(),
        Values::kMaxIntakeMotorEncoderPulsesPerSecond());
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(Values::kMaxProximalEncoderPulsesPerSecond(),
        Values::kMaxDistalEncoderPulsesPerSecond());
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

  void set_left_drivetrain_shifter_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    left_drivetrain_shifter_ = ::std::move(potentiometer);
  }

  // Right drivetrain side.
  void set_drivetrain_right_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  void set_right_drivetrain_shifter_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    right_drivetrain_shifter_ = ::std::move(potentiometer);
  }

  // Proximal joint.
  void set_proximal_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    proximal_encoder_.set_encoder(::std::move(encoder));
  }

  void set_proximal_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    proximal_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_proximal_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    proximal_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Distal joint.
  void set_distal_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    distal_encoder_.set_encoder(::std::move(encoder));
  }

  void set_distal_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(absolute_pwm.get());
    distal_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_distal_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    distal_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Left intake side.
  void set_left_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    left_intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_left_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(absolute_pwm.get());
    left_intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_left_intake_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    left_intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_left_intake_spring_angle(
      ::std::unique_ptr<frc::AnalogInput> encoder) {
    left_intake_spring_angle_ = ::std::move(encoder);
  }

  void set_left_intake_cube_detector(
      ::std::unique_ptr<frc::DigitalInput> input) {
    left_intake_cube_detector_ = ::std::move(input);
  }

  // Right intake side.
  void set_right_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    right_intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_right_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(absolute_pwm.get());
    right_intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_right_intake_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    right_intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_right_intake_spring_angle(
      ::std::unique_ptr<frc::AnalogInput> encoder) {
    right_intake_spring_angle_ = ::std::move(encoder);
  }

  void set_right_intake_cube_detector(
      ::std::unique_ptr<frc::DigitalInput> input) {
    right_intake_cube_detector_ = ::std::move(input);
  }

  void set_claw_beambreak(::std::unique_ptr<frc::DigitalInput> input) {
    claw_beambreak_ = ::std::move(input);
  }

  void set_box_back_beambreak(::std::unique_ptr<frc::DigitalInput> input) {
    box_back_beambreak_ = ::std::move(input);
  }

  // Auto mode switches.
  void set_autonomous_mode(int i, ::std::unique_ptr<frc::DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void set_pwm_trigger(::std::unique_ptr<frc::DigitalInput> pwm_trigger) {
    medium_encoder_filter_.Add(pwm_trigger.get());
    pwm_trigger_ = ::std::move(pwm_trigger);
  }

  void set_lidar_lite_input(::std::unique_ptr<frc::DigitalInput> lidar_lite_input) {
    lidar_lite_input_ = ::std::move(lidar_lite_input);
    lidar_lite_.set_input(lidar_lite_input_.get());
  }

  // All of the DMA-related set_* calls must be made before this, and it
  // doesn't hurt to do all of them.
  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(
        new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
    dma_synchronizer_->Add(&lidar_lite_);
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

    const auto values = constants::GetValues();

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());
      drivetrain_message->left_shifter_position =
          drivetrain_shifter_pot_translate(
              left_drivetrain_shifter_->GetVoltage());

      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->right_speed =
          -drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());
      drivetrain_message->right_shifter_position =
          drivetrain_shifter_pot_translate(
              right_drivetrain_shifter_->GetVoltage());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();

    {
      auto superstructure_message = superstructure_queue.position.MakeMessage();

      CopyPosition(proximal_encoder_, &superstructure_message->arm.proximal,
                   Values::kProximalEncoderCountsPerRevolution(),
                   Values::kProximalEncoderRatio(), proximal_pot_translate,
                   true, values.arm_proximal.potentiometer_offset);

      CopyPosition(distal_encoder_, &superstructure_message->arm.distal,
                   Values::kDistalEncoderCountsPerRevolution(),
                   Values::kDistalEncoderRatio(), distal_pot_translate, true,
                   values.arm_distal.potentiometer_offset);

      CopyPosition(left_intake_encoder_,
                   &superstructure_message->left_intake.motor_position,
                   Values::kIntakeMotorEncoderCountsPerRevolution(),
                   Values::kIntakeMotorEncoderRatio(), intake_pot_translate,
                   false, values.left_intake.potentiometer_offset);

      CopyPosition(right_intake_encoder_,
                   &superstructure_message->right_intake.motor_position,
                   Values::kIntakeMotorEncoderCountsPerRevolution(),
                   Values::kIntakeMotorEncoderRatio(), intake_pot_translate,
                   true, values.right_intake.potentiometer_offset);

      superstructure_message->left_intake.spring_angle =
          intake_spring_translate(left_intake_spring_angle_->GetVoltage()) +
          values.left_intake.spring_offset;
      superstructure_message->left_intake.beam_break =
          !left_intake_cube_detector_->Get();

      superstructure_message->right_intake.spring_angle =
          -intake_spring_translate(right_intake_spring_angle_->GetVoltage()) +
          values.right_intake.spring_offset;
      superstructure_message->right_intake.beam_break =
          !right_intake_cube_detector_->Get();

      superstructure_message->claw_beambreak_triggered = !claw_beambreak_->Get();
      superstructure_message->box_back_beambreak_triggered =
          !box_back_beambreak_->Get();

      superstructure_message->box_distance =
          lidar_lite_.last_width() / 0.00001 / 100.0 / 2;

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

  void CopyPosition(
      const ::frc971::wpilib::AbsoluteEncoderAndPotentiometer &encoder,
      ::frc971::PotAndAbsolutePosition *position,
      double encoder_counts_per_revolution, double encoder_ratio,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double pot_offset) {
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

  ::std::unique_ptr<frc::AnalogInput> left_drivetrain_shifter_,
      right_drivetrain_shifter_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer proximal_encoder_,
      distal_encoder_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer left_intake_encoder_,
      right_intake_encoder_;

  ::std::unique_ptr<frc::AnalogInput> left_intake_spring_angle_,
      right_intake_spring_angle_;
  ::std::unique_ptr<frc::DigitalInput> left_intake_cube_detector_,
      right_intake_cube_detector_;

  ::std::unique_ptr<frc::DigitalInput> claw_beambreak_;
  ::std::unique_ptr<frc::DigitalInput> box_back_beambreak_;

  ::std::unique_ptr<frc::DigitalInput> pwm_trigger_;

  ::std::array<::std::unique_ptr<frc::DigitalInput>, 4> autonomous_modes_;

  ::std::unique_ptr<frc::DigitalInput> lidar_lite_input_;
  ::frc971::wpilib::DMAPulseWidthReader lidar_lite_;

  ::std::atomic<bool> run_{true};
};

class SolenoidWriter {
 public:
  SolenoidWriter(::frc971::wpilib::BufferedPcm *pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain_queue.output"),
        superstructure_(".y2018.control_loops.superstructure_queue.output") {}

  // left drive
  // right drive
  //
  // claw
  // arm brakes
  // hook release
  // fork release
  void set_left_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    left_drivetrain_shifter_ = ::std::move(s);
  }
  void set_right_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    right_drivetrain_shifter_ = ::std::move(s);
  }

  void set_claw(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    claw_ = ::std::move(s);
  }

  void set_arm_brakes(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    arm_brakes_ = ::std::move(s);
  }

  void set_hook(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    hook_ = ::std::move(s);
  }

  void set_forks(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    forks_ = ::std::move(s);
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

          claw_->Set(!superstructure_->claw_grabbed);
          arm_brakes_->Set(superstructure_->release_arm_brake);
          hook_->Set(superstructure_->hook_release);
          forks_->Set(superstructure_->forks_release);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;

        pcm_->Flush();
        to_log.read_solenoids = pcm_->GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }

      status_light.FetchLatest();
      // If we don't have a light request (or it's an old one), we are borked.
      // Flash the red light slowly.
      if (!status_light.get() ||
          status_light.Age() > chrono::milliseconds(100)) {
        StatusLight color;
        color.red = 0.0;
        color.green = 0.0;
        color.blue = 0.0;

        ::y2018::vision::vision_status.FetchLatest();
        ++light_flash_;
        if (light_flash_ > 10) {
          color.red = 0.5;
        } else if (!y2018::vision::vision_status.get() ||
                   y2018::vision::vision_status.Age() > chrono::seconds(1)) {
          color.red = 0.5;
          color.green = 0.5;
        }

        if (light_flash_ > 20) {
          light_flash_ = 0;
        }

        LOG_STRUCT(DEBUG, "color", color);
        SetColor(color);
      } else {
        LOG_STRUCT(DEBUG, "color", *status_light);
        SetColor(*status_light);
      }
    }
  }

  void SetColor(const StatusLight &status_light) {
    // Save CAN bandwidth and CPU at the cost of RT.  Only change the light when
    // it actually changes.  This is pretty low priority anyways.
    static int time_since_last_send = 0;
    ++time_since_last_send;
    if (time_since_last_send > 10) {
      time_since_last_send = 0;
    }
    if (status_light.green != last_green_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(1.0 - status_light.green,
                             ::ctre::phoenix::CANifier::LEDChannelB);
      last_green_ = status_light.green;
    }

    if (status_light.blue != last_blue_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(1.0 - status_light.blue,
                             ::ctre::phoenix::CANifier::LEDChannelA);
      last_blue_ = status_light.blue;
    }

    if (status_light.red != last_red_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(1.0 - status_light.red,
                             ::ctre::phoenix::CANifier::LEDChannelC);
      last_red_ = status_light.red;
    }
  }

  void Quit() { run_ = false; }

 private:
  ::frc971::wpilib::BufferedPcm *pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid>
      left_drivetrain_shifter_, right_drivetrain_shifter_, claw_, arm_brakes_,
      hook_, forks_;

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;
  ::aos::Queue<::y2018::control_loops::SuperstructureQueue::Output>
      superstructure_;

  ::ctre::phoenix::CANifier canifier_{0};

  ::std::atomic<bool> run_{true};

  double last_red_ = -1.0;
  double last_green_ = -1.0;
  double last_blue_ = -1.0;

  int light_flash_ = 0;
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

class SuperstructureWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_proximal_victor(::std::unique_ptr<::frc::VictorSP> t) {
    proximal_victor_ = ::std::move(t);
  }
  void set_distal_victor(::std::unique_ptr<::frc::VictorSP> t) {
    distal_victor_ = ::std::move(t);
  }

  void set_hanger_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hanger_victor_ = ::std::move(t);
  }

  void set_left_intake_elastic_victor(::std::unique_ptr<::frc::VictorSP> t) {
    left_intake_elastic_victor_ = ::std::move(t);
  }
  void set_right_intake_elastic_victor(::std::unique_ptr<::frc::VictorSP> t) {
    right_intake_elastic_victor_ = ::std::move(t);
  }

  void set_left_intake_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    left_intake_rollers_victor_ = ::std::move(t);
  }

  void set_right_intake_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    right_intake_rollers_victor_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2018::control_loops::superstructure_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2018::control_loops::superstructure_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);

    left_intake_elastic_victor_->SetSpeed(
        ::aos::Clip(-queue->left_intake.voltage_elastic, -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    right_intake_elastic_victor_->SetSpeed(
        ::aos::Clip(queue->right_intake.voltage_elastic, -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    left_intake_rollers_victor_->SetSpeed(
        ::aos::Clip(-queue->left_intake.voltage_rollers, -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    right_intake_rollers_victor_->SetSpeed(
        ::aos::Clip(queue->right_intake.voltage_rollers, -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    proximal_victor_->SetSpeed(::aos::Clip(-queue->voltage_proximal,
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                               12.0);

    distal_victor_->SetSpeed(::aos::Clip(queue->voltage_distal,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
    hanger_victor_->SetSpeed(
        ::aos::Clip(-queue->voltage_winch, -kMaxBringupPower, kMaxBringupPower) /
        12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Superstructure output too old.\n");

    left_intake_rollers_victor_->SetDisabled();
    right_intake_rollers_victor_->SetDisabled();
    left_intake_elastic_victor_->SetDisabled();
    right_intake_elastic_victor_->SetDisabled();

    proximal_victor_->SetDisabled();
    distal_victor_->SetDisabled();
    hanger_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> left_intake_rollers_victor_,
      right_intake_rollers_victor_, left_intake_elastic_victor_,
      right_intake_elastic_victor_, proximal_victor_, distal_victor_,
      hanger_victor_;
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
    reader.set_left_drivetrain_shifter_potentiometer(
        make_unique<frc::AnalogInput>(6));
    reader.set_drivetrain_right_encoder(make_encoder(1));
    reader.set_right_drivetrain_shifter_potentiometer(
        make_unique<frc::AnalogInput>(7));

    reader.set_proximal_encoder(make_encoder(4));
    reader.set_proximal_absolute_pwm(make_unique<frc::DigitalInput>(2));
    reader.set_proximal_potentiometer(make_unique<frc::AnalogInput>(2));

    reader.set_distal_encoder(make_encoder(2));
    reader.set_distal_absolute_pwm(make_unique<frc::DigitalInput>(3));
    reader.set_distal_potentiometer(make_unique<frc::AnalogInput>(3));

    reader.set_right_intake_encoder(make_encoder(5));
    reader.set_right_intake_absolute_pwm(make_unique<frc::DigitalInput>(7));
    reader.set_right_intake_potentiometer(make_unique<frc::AnalogInput>(1));
    reader.set_right_intake_spring_angle(make_unique<frc::AnalogInput>(5));
    reader.set_right_intake_cube_detector(make_unique<frc::DigitalInput>(1));

    reader.set_left_intake_encoder(make_encoder(3));
    reader.set_left_intake_absolute_pwm(make_unique<frc::DigitalInput>(4));
    reader.set_left_intake_potentiometer(make_unique<frc::AnalogInput>(0));
    reader.set_left_intake_spring_angle(make_unique<frc::AnalogInput>(4));
    reader.set_left_intake_cube_detector(make_unique<frc::DigitalInput>(0));

    reader.set_claw_beambreak(make_unique<frc::DigitalInput>(8));
    reader.set_box_back_beambreak(make_unique<frc::DigitalInput>(9));

    reader.set_pwm_trigger(make_unique<frc::DigitalInput>(25));

    reader.set_lidar_lite_input(make_unique<frc::DigitalInput>(22));

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

    SuperstructureWriter superstructure_writer;
    superstructure_writer.set_left_intake_elastic_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    superstructure_writer.set_left_intake_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    superstructure_writer.set_right_intake_elastic_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(7)));
    superstructure_writer.set_right_intake_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    superstructure_writer.set_proximal_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)));
    superstructure_writer.set_distal_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)));

    superstructure_writer.set_hanger_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(8)));

    ::std::thread superstructure_writer_thread(
        ::std::ref(superstructure_writer));

    ::frc971::wpilib::BufferedPcm *pcm = new ::frc971::wpilib::BufferedPcm();
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_left_drivetrain_shifter(pcm->MakeSolenoid(0));
    solenoid_writer.set_right_drivetrain_shifter(pcm->MakeSolenoid(1));
    solenoid_writer.set_claw(pcm->MakeSolenoid(2));
    solenoid_writer.set_arm_brakes(pcm->MakeSolenoid(3));
    solenoid_writer.set_hook(pcm->MakeSolenoid(4));
    solenoid_writer.set_forks(pcm->MakeSolenoid(5));

    ::std::thread solenoid_thread(::std::ref(solenoid_writer));

    int32_t status = 0;
    HAL_CompressorHandle compressor = HAL_InitializeCompressor(0, &status);
    if (status != 0) {
      LOG(ERROR, "Compressor status is nonzero, %d\n",
          static_cast<int>(status));
    }
    HAL_SetCompressorClosedLoopControl(compressor, true, &status);
    if (status != 0) {
      LOG(ERROR, "Compressor status is nonzero, %d\n",
          static_cast<int>(status));
    }

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
}  // namespace y2018

AOS_ROBOT_CLASS(::y2018::wpilib::WPILibRobot);
